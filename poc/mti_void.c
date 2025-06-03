/**
 * @file    mti_void.c
 * @brief   Void Detection System Driver Implementation
 * @author  Engineering Team
 * @version 1.2.0 (Converted to millimeter-based integer arithmetic)
 * @date    2025-05-26
 */

#include "mti_void.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "vmt_common_defs.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Conversion constants
#define METERS_TO_MM_FACTOR 1000U
#define MM_TO_METERS_FACTOR 0.001f

// Maximum valid measurement range in millimeters (10 meters)
#define MAX_MEASUREMENT_RANGE_MM 10000U

// Confidence level thresholds
#define CONFIDENCE_LEVEL_LOW    50 // 50% - barely confident
#define CONFIDENCE_LEVEL_MEDIUM 75 // 75% - moderately confident
#define CONFIDENCE_LEVEL_HIGH   90 // 90% - highly confident

static bool               prv_void_is_initialized       = false;
static uint32_t           prv_void_next_radar_poll_time = 0;
static wall_measurement_t prv_void_latest_measurements[MAX_SENSORS];
static wall_profile_t     prv_void_wall_profiles_history[WALL_HISTORY_SIZE];
static uint8_t            prv_void_wall_profile_history_idx = 0;

// Void detection history - uncommented and to be used
static void_detection_t prv_void_detections_history[MAX_VOID_DETECTIONS];
static uint8_t          prv_void_detection_history_write_idx = 0;
static uint8_t          prv_void_detection_history_count     = 0; // Added to track actual number of items

static void_config_t prv_void_current_config;
static geo_model_t   prv_void_geo_model;
static uint16_t      prv_void_expected_borehole_radius_mm = 0;

// Pre-calculated trigonometric values scaled by 1000 for precision
// static const int32_t cos_values[MAX_SENSORS] = { 1000, -500, -500 }; // cos(0°, 120°, 240°) * 1000 - REMOVED (unused global)
// static const int32_t sin_values[MAX_SENSORS] = { 0, 866, -866 };     // sin(0°, 120°, 240°) * 1000 - REMOVED (unused global)

// Private function declarations
static wall_measurement_t prv_void_convert_radar_to_wall_measurement(uint8_t sensor_idx, const radar_data_t *radar_data);
static void               prv_void_store_wall_measurement(uint8_t sensor_idx, wall_measurement_t measurement);
static bool               prv_void_are_all_sensors_fresh(void);
static bool               prv_void_calculate_wall_profile(void);
static void               prv_void_analyze_for_voids(void);
static void               prv_void_store_profile_to_history(wall_profile_t profile);
static void               prv_void_store_detection_to_history(void_detection_t void_data);
static void               prv_void_report_detection(const void_detection_t *void_data);
static uint16_t           prv_void_fit_circle_integer(const wall_measurement_t measurements[MAX_SENSORS], int32_t *center_x_mm, int32_t *center_y_mm);
static uint8_t            prv_void_calculate_confidence(uint8_t snr, uint16_t void_size_mm, uint16_t distance_mm);
static void               prv_void_detect_from_measurements(const wall_measurement_t current_measurements[MAX_SENSORS]);
static uint16_t           prv_void_integer_sqrt(uint32_t value);
static void               prv_void_request_data_from_radars(void);
static void               prv_void_init_wall_profile_history(void);
static uint16_t           prv_void_get_fallback_average_distance(const wall_measurement_t measurements[MAX_SENSORS], uint16_t default_dist_mm);
static wall_measurement_t prv_void_process_multi_point_detection(uint8_t sensor_idx, const radar_data_t *radar_data, uint16_t cal_factor_ppm);
static uint16_t           prv_void_get_sensor_cal_factor_ppm(uint8_t sensor_idx);
static bool               prv_void_is_suspicious_reading(uint8_t sensor_idx, uint16_t distance_mm);
// static uint16_t           prv_void_convert_meters_to_mm(float range_meters); // REMOVED (unused function)
static uint8_t     prv_void_convert_snr_to_uint8(float snr_db);
static void        prv_void_update_measurement_history(uint8_t sensor_idx, wall_measurement_t *measurement);
static const char *prv_void_get_confidence_text(uint8_t confidence);

/**
 * @brief Convert SNR from float dB to uint8_t with saturation
 * @param[in] snr_db SNR value in dB from radar sensor
 * @return SNR as uint8_t (0-255), saturated if needed
 */
static uint8_t prv_void_convert_snr_to_uint8(float snr_db)
{
    if (snr_db <= 0.0f)
    {
        return 0;
    }
    if (snr_db >= 255.0f)
    {
        return 255;
    }
    return (uint8_t)snr_db; // Truncate without rounding
}

bool void_init(void)
{
    DEBUG_SEND("Void: Initializing...");

    // Step 1: Ensure CAN peripheral is ready
    if (can_get_state() != CAN_STATE_READY)
    {
        DEBUG_SEND("Void Init: CAN not ready, attempting can_init().");
        if (can_init() != CAN_STATE_READY)
        {
            DEBUG_SEND("Void Init Error: CAN peripheral init failed within void_init.");
            prv_void_is_initialized = false;
            return false;
        }
    }
    DEBUG_SEND("Void: CAN peripheral confirmed ready.");

    // Step 2: Setup CAN filters specific to the void detection sensors
    if (!can_setup_all_sensors())
    {
        DEBUG_SEND("Void Init Error: CAN filter setup for sensors failed.");
        prv_void_is_initialized = false;
        return false;
    }
    DEBUG_SEND("Void: CAN sensor filters configured.");

    // Step 3: Register the data processing callback with the CAN module
    if (can_register_data_callback(void_process_radar_data_callback) != CAN_STATE_READY)
    {
        DEBUG_SEND("Void Init Error: Failed to register radar data callback.");
        prv_void_is_initialized = false;
        return false;
    }
    DEBUG_SEND("Void: Radar data callback registered.");

    // Step 4: Load default or persisted configuration
    // For now, using default configuration. Persisted config can be added later.
    prv_void_current_config = (void_config_t){
        .baseline_method       = DEFAULT_VOID_BASELINE_METHOD,
        .detection_threshold   = DEFAULT_VOID_DETECTION_THRESHOLD_MM,
        .min_snr               = DEFAULT_MIN_SNR_THRESHOLD,
        .confidence_threshold  = DEFAULT_CONFIDENCE_THRESHOLD_PERCENT,
        .default_wall_distance = DEFAULT_NOMINAL_WALL_DISTANCE_MM,
        .range_min             = DEFAULT_MIN_VALID_RANGE_MM,
        .range_max             = DEFAULT_MAX_VALID_RANGE_MM,
        .mode                  = (uint8_t)VOID_DETECTION_MODE_CONTINUOUS,
        .enable_median_filter  = false,
        .multi_point_algorithm = MULTI_POINT_ALGORITHM_SEQUENTIAL // Use the correct enum value
    };
    DEBUG_SEND("Void: Default void algorithm configuration loaded.");

    // Step 5: Initialize internal buffers and state variables
    prv_void_init_wall_profile_history();
    // Initialize detection history
    memset(prv_void_detections_history, 0, sizeof(prv_void_detections_history));
    prv_void_detection_history_write_idx = 0;
    prv_void_detection_history_count     = 0;

    prv_void_expected_borehole_radius_mm = prv_void_current_config.default_wall_distance / 2U; // Example, adjust as needed
    memset(prv_void_latest_measurements, 0, sizeof(prv_void_latest_measurements));
    for (uint8_t i = 0; i < MAX_SENSORS; ++i)
    {
        prv_void_latest_measurements[i].status    = WALL_STATUS_NO_DETECTION;
        prv_void_latest_measurements[i].timestamp = 0;
    }
    memset(&prv_void_geo_model, 0, sizeof(geo_model_t));

    prv_void_next_radar_poll_time = HAL_GetTick() + RADAR_POLL_INTERVAL_MS;
    prv_void_is_initialized       = true;
    DEBUG_SEND("Void: Subsystem initialization complete. Poll Interval: %lu ms.", (unsigned long)RADAR_POLL_INTERVAL_MS);
    return true;
}

void void_process(void)
{
    if (!prv_void_is_initialized)
    {
        return;
    }

    uint32_t current_time = HAL_GetTick();
    if (current_time >= prv_void_next_radar_poll_time)
    {
        prv_void_request_data_from_radars();
        prv_void_next_radar_poll_time = current_time + RADAR_POLL_INTERVAL_MS;
    }
}

void void_process_radar_data_callback(uint8_t sensor_idx, radar_data_t *radar_data_ptr)
{
    if (!prv_void_is_initialized)
    {
        return;
    }
    if (sensor_idx >= MAX_SENSORS || radar_data_ptr == NULL)
    {
        DEBUG_SEND("Void CB Error: Invalid params (S%u).", sensor_idx);
        return;
    }

    wall_measurement_t meas = prv_void_convert_radar_to_wall_measurement(sensor_idx, radar_data_ptr);
    meas.timestamp          = HAL_GetTick();
    prv_void_store_wall_measurement(sensor_idx, meas);

    if (prv_void_are_all_sensors_fresh())
    {
        if (prv_void_calculate_wall_profile())
        {
            // Profile calculation successful - void analysis completed in prv_void_calculate_wall_profile
        }
    }
}

static wall_measurement_t prv_void_convert_radar_to_wall_measurement(uint8_t sensor_idx, const radar_data_t *radar_data)
{
    wall_measurement_t m = { 0 };
    m.status             = WALL_STATUS_NO_DETECTION;
    m.distance           = prv_void_current_config.default_wall_distance;
    m.snr                = 0;

    // Check sensor hardware status
    if (radar_data->status != RADAR_HW_STATUS_READY && radar_data->status != RADAR_HW_STATUS_CHIRPING)
    {
        m.status = WALL_STATUS_ERROR;
        DEBUG_SEND("Void Convert: S%u hardware status error (%u)", sensor_idx, radar_data->status);
        return m;
    }

    // No detection points
    if (radar_data->numDetPoints == 0)
    {
        DEBUG_SEND("Void Convert: S%u no detection points", sensor_idx);
        return m;
    }

    // Get per-sensor calibration factor in parts per million (1000000 = no calibration)
    uint16_t cal_factor_ppm = prv_void_get_sensor_cal_factor_ppm(sensor_idx);

    // Process detection points using statistics for better accuracy
    if (radar_data->numDetPoints > 1)
    {
        return prv_void_process_multi_point_detection(sensor_idx, radar_data, cal_factor_ppm);
    }
    else
    {
        // Single point processing (common case)
        float x_m = radar_data->detectedPoints[0][0];
        float y_m = radar_data->detectedPoints[0][1];

        // Calculate Euclidean distance in meters
        float dist_m = sqrtf(x_m * x_m + y_m * y_m);

        // Apply calibration factor (parts per million)
        uint32_t dist_mm_raw = (uint32_t)(dist_m * (float)METERS_TO_MM_FACTOR);
        uint32_t dist_mm_cal = (dist_mm_raw * (uint32_t)cal_factor_ppm) / 1000000U;

        // Convert to millimeters and validate
        uint16_t dist_mm = (dist_mm_cal <= 65535U) ? (uint16_t)dist_mm_cal : 65535U;

        // Validate range
        if (dist_mm >= prv_void_current_config.range_min && dist_mm <= prv_void_current_config.range_max && dist_mm > 0)
        {
            m.status = WALL_STATUS_VALID;
        }
        else
        {
            m.status = WALL_STATUS_OUT_OF_RANGE;
        }
    }

    // Update measurement history for this sensor
    prv_void_update_measurement_history(sensor_idx, &m);

    return m;
}

/**
 * @brief Process radar data with multiple detection points using integer arithmetic
 * @param sensor_idx Index of the sensor (0-2)
 * @param radar_data Pointer to the raw radar data structure
 * @param cal_factor_ppm Calibration factor in parts per million (1000000 = no calibration)
 * @return Processed wall measurement
 */
static wall_measurement_t prv_void_process_multi_point_detection(uint8_t sensor_idx, const radar_data_t *radar_data, uint16_t cal_factor_ppm)
{
    wall_measurement_t m = { 0 };
    m.status             = WALL_STATUS_NO_DETECTION;
    m.distance           = prv_void_current_config.default_wall_distance;
    m.snr                = 0;

    uint8_t point_count = (radar_data->numDetPoints > MAX_RADAR_DETECTED_POINTS) ? MAX_RADAR_DETECTED_POINTS : radar_data->numDetPoints;

    // Arrays to store calculated distances and SNR values (integer arithmetic)
    uint16_t distances_mm[MAX_RADAR_DETECTED_POINTS];
    uint8_t  snr_values[MAX_RADAR_DETECTED_POINTS];
    uint8_t  valid_points = 0;

    // Calculate distance for each point using integer arithmetic
    for (uint8_t i = 0; i < point_count; i++)
    {
        float x_m = radar_data->detectedPoints[i][0];
        float y_m = radar_data->detectedPoints[i][1];

        // Calculate distance in meters, then convert to mm
        float    dist_m      = sqrtf(x_m * x_m + y_m * y_m);
        uint32_t dist_mm_raw = (uint32_t)(dist_m * (float)METERS_TO_MM_FACTOR);

        // Apply calibration factor
        uint32_t dist_mm_cal = (dist_mm_raw * (uint32_t)cal_factor_ppm) / 1000000U;
        uint16_t dist_mm     = (dist_mm_cal <= 65535U) ? (uint16_t)dist_mm_cal : 65535U;

        // Validate range
        if (dist_mm >= prv_void_current_config.range_min && dist_mm <= prv_void_current_config.range_max && dist_mm > 0)
        {
            distances_mm[valid_points] = dist_mm;
            snr_values[valid_points]   = prv_void_convert_snr_to_uint8(radar_data->maxSNR);
            valid_points++;
        }
    }

    if (valid_points == 0)
    {
        m.status = WALL_STATUS_OUT_OF_RANGE;
        return m;
    }

    // Multiple valid points - use appropriate algorithm based on configuration
    uint16_t selected_distance_mm;
    uint8_t  selected_snr;

    switch (prv_void_current_config.multi_point_algorithm)
    {
    case MULTI_POINT_CLOSEST:
        // Find minimum distance using integer comparison
        selected_distance_mm = distances_mm[0];
        selected_snr         = snr_values[0];
        for (uint8_t i = 1; i < valid_points; i++)
        {
            if (distances_mm[i] < selected_distance_mm)
            {
                selected_distance_mm = distances_mm[i];
                selected_snr         = snr_values[i];
            }
        }
        break;

    case MULTI_POINT_STRONGEST:
        // Find maximum SNR using integer comparison
        selected_distance_mm = distances_mm[0];
        selected_snr         = snr_values[0];
        for (uint8_t i = 1; i < valid_points; i++)
        {
            if (snr_values[i] > selected_snr)
            {
                selected_distance_mm = distances_mm[i];
                selected_snr         = snr_values[i];
            }
        }
        break;

    case MULTI_POINT_MEDIAN:
        // Bubble sort for median (small arrays)
        for (uint8_t i = 0; i < valid_points - 1; i++)
        {
            for (uint8_t j = 0; j < valid_points - i - 1; j++)
            {
                if (distances_mm[j] > distances_mm[j + 1])
                {
                    uint16_t temp_dist  = distances_mm[j];
                    distances_mm[j]     = distances_mm[j + 1];
                    distances_mm[j + 1] = temp_dist;

                    uint8_t temp_snr  = snr_values[j];
                    snr_values[j]     = snr_values[j + 1];
                    snr_values[j + 1] = temp_snr;
                }
            }
        }
        selected_distance_mm = distances_mm[valid_points / 2U];
        selected_snr         = snr_values[valid_points / 2U];
        break;

    default:
        /* Intentional fallthrough: Default behavior is average calculation. */
    case MULTI_POINT_AVERAGE: // This case now handles its own logic and the default.
    {
        // Calculate average using integer arithmetic
        uint32_t sum_distance = 0;
        uint32_t sum_snr      = 0;
        for (uint8_t i = 0; i < valid_points; i++)
        {
            sum_distance += distances_mm[i];
            sum_snr += snr_values[i];
        }

        uint32_t average_distance_u32 = sum_distance / valid_points;
        selected_distance_mm          = (average_distance_u32 > UINT16_MAX) ? UINT16_MAX : (uint16_t)average_distance_u32;

        uint32_t average_snr_u32 = sum_snr / valid_points;
        selected_snr             = (average_snr_u32 > UINT8_MAX) ? UINT8_MAX : (uint8_t)average_snr_u32;
    }
    break;
    }

    // Set output values
    m.distance = selected_distance_mm;
    m.snr      = selected_snr;
    m.status   = WALL_STATUS_VALID;

    return m;
}

/**
 * @brief Get calibration factor for a specific sensor in parts per million
 * @param sensor_idx Index of the sensor (0-2)
 * @return Calibration factor in ppm (1000000 = no calibration, 1050000 = +5% scaling)
 */
static uint16_t prv_void_get_sensor_cal_factor_ppm(uint8_t sensor_idx)
{
    // Default calibration factors in parts per million (could be read from persistent storage)
    static const uint32_t default_cal_factors_ppm[MAX_SENSORS] = {
        // Change to uint32_t
        1000000U, // Sensor 0: No calibration
        1020000U, // Sensor 1: +2% scaling
        995000U   // Sensor 2: -0.5% scaling
    };

    if (sensor_idx < MAX_SENSORS)
    {
        return (uint16_t)(default_cal_factors_ppm[sensor_idx] / 1000U); // Convert to uint16_t safely
    }
    return 1000U; // Default: no calibration (scaled down)
}

/**
 * @brief Check if a reading is suspicious (large sudden change)
 * @param sensor_idx Index of the sensor (0-2)
 * @param distance_mm New distance reading in millimeters
 * @return true if reading seems suspicious, false otherwise
 */
static bool prv_void_is_suspicious_reading(uint8_t sensor_idx, uint16_t distance_mm)
{
    static uint16_t last_valid_readings[MAX_SENSORS] = { 0 };
    static bool     have_valid_reading[MAX_SENSORS]  = { false };

    if (!have_valid_reading[sensor_idx])
    {
        last_valid_readings[sensor_idx] = distance_mm;
        have_valid_reading[sensor_idx]  = true;
        return false; // First reading is never suspicious
    }

    // Calculate absolute difference from last reading using integer arithmetic
    uint16_t diff = (distance_mm > last_valid_readings[sensor_idx]) ? (distance_mm - last_valid_readings[sensor_idx]) : (last_valid_readings[sensor_idx] - distance_mm);

    // Check if difference exceeds threshold (20% of previous reading)
    uint16_t threshold = (last_valid_readings[sensor_idx] * 20U) / 100U;
    threshold          = (threshold < 100U) ? 100U : threshold; // Minimum threshold 100mm

    if (diff > threshold)
    {
        DEBUG_SEND("Void Suspicious: S%u change %u mm (>%u mm threshold)", sensor_idx, diff, threshold);
        return true;
    }

    // Update last reading and return not suspicious
    last_valid_readings[sensor_idx] = distance_mm;
    return false;
}

/**
 * @brief Update measurement history for statistical processing and flag suspicious readings.
 *        The measurement's status may be updated to WALL_STATUS_SUSPICIOUS.
 * @param sensor_idx Index of the sensor (0-2)
 * @param[in,out] measurement Pointer to the measurement to be checked and potentially updated
 */
static void prv_void_update_measurement_history(uint8_t sensor_idx, wall_measurement_t *measurement) // MODIFIED: removed const from parameter
{
    // Check for suspicious readings
    if (measurement->status == WALL_STATUS_VALID)
    {
        if (prv_void_is_suspicious_reading(sensor_idx, measurement->distance))
        {
            // Mark the measurement itself as suspicious
            // wall_measurement_t modified_measurement = *measurement; // REMOVED
            // modified_measurement.status             = WALL_STATUS_SUSPICIOUS; // REMOVED
            measurement->status = WALL_STATUS_SUSPICIOUS; // ADDED: Directly update the status
            // The measurement, now marked as suspicious, will be stored by the caller.
        }
    }
}

static void prv_void_store_wall_measurement(uint8_t sensor_idx, wall_measurement_t measurement)
{
    if (sensor_idx < MAX_SENSORS)
    {
        prv_void_latest_measurements[sensor_idx] = measurement;
        DEBUG_SEND("Void Store: S%u -> %u mm, SNR %u, Status %u", sensor_idx, measurement.distance, measurement.snr, measurement.status);
    }
}

static bool prv_void_are_all_sensors_fresh(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t max_age      = (RADAR_POLL_INTERVAL_MS * 2U) + (RADAR_POLL_INTERVAL_MS / 2U);

    for (uint8_t i = 0; i < MAX_SENSORS; ++i)
    {
        if (prv_void_latest_measurements[i].timestamp == 0 || (current_time - prv_void_latest_measurements[i].timestamp) > max_age)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Calculate cross-sectional profile from current wall measurements
 * @return true if profile was successfully calculated, false otherwise
 */
static bool prv_void_calculate_wall_profile(void)
{
    // Create a local copy of the current measurements for processing
    wall_measurement_t current_measurements[MAX_SENSORS];
    for (uint8_t i = 0; i < MAX_SENSORS; i++)
    {
        current_measurements[i] = prv_void_latest_measurements[i];
    }

    // Check if we have enough valid measurements for analysis
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < MAX_SENSORS; i++)
    {
        if (current_measurements[i].status == WALL_STATUS_VALID)
        {
            valid_count++;
        }
    }

    // Need at least MIN_VALID_SAMPLES for reliable analysis
    if (valid_count < MIN_VALID_SAMPLES)
    {
        DEBUG_SEND("Void: Insufficient valid measurements (%d/%d)", valid_count, MIN_VALID_SAMPLES);
        return false;
    }

    // Now that we have sufficient valid measurements, analyze for voids
    prv_void_analyze_for_voids();
    return true;
}

/**
 * @brief Analyze latest measurements for voids using selected method
 */
static void prv_void_analyze_for_voids(void)
{
    wall_measurement_t current_measurements[MAX_SENSORS];

    // Create a copy of the latest measurements for processing
    for (uint8_t i = 0; i < MAX_SENSORS; i++)
    {
        current_measurements[i] = prv_void_latest_measurements[i];
    }

    // Use the configured detection method
    switch (prv_void_current_config.baseline_method)
    {
    case DEFAULT_VOID_BASELINE_METHOD:
        // Uphole Baseline Method (default)
        prv_void_detect_from_measurements(current_measurements);
        break;

    case 1:
        // Dynamic Circle Fitting Method (if implemented)
        DEBUG_SEND("Void: Method 1 (Dynamic Circle) not yet implemented");
        // Fall back to default method
        prv_void_detect_from_measurements(current_measurements);
        break;

    case 2:
        // Moving Average Method (future)
        DEBUG_SEND("Void: Method 2 (Moving Average) not yet implemented");
        // Fall back to default method
        prv_void_detect_from_measurements(current_measurements);
        break;

    default:
        DEBUG_SEND("Void: Unknown method %d, using default", prv_void_current_config.baseline_method);
        prv_void_detect_from_measurements(current_measurements);
        break;
    }
}

static uint16_t prv_void_get_fallback_average_distance(const wall_measurement_t measurements[MAX_SENSORS], uint16_t default_dist_mm)
{
    uint32_t sum_dist    = 0;
    uint8_t  valid_count = 0;

    for (uint8_t i = 0; i < MAX_SENSORS; ++i)
    {
        if (measurements[i].status == WALL_STATUS_VALID)
        {
            sum_dist += measurements[i].distance;
            valid_count++;
        }
    }

    uint16_t result = (valid_count == 0) ? default_dist_mm : (uint16_t)(sum_dist / valid_count);
    DEBUG_SEND("Void: Fallback average - %u valid measurements, result: %u mm", valid_count, result);
    return result;
}

static void prv_void_detect_from_measurements(const wall_measurement_t current_measurements[MAX_SENSORS])
{
    int32_t center_x_mm = 0;
    int32_t center_y_mm = 0;

    // Step 1: Perform circle fitting to determine probe position (integer arithmetic)
    uint16_t fitted_radius_mm = prv_void_fit_circle_integer(current_measurements, &center_x_mm, &center_y_mm);

    // If circle fitting failed, use fallback radius
    if (fitted_radius_mm == 0)
    {
        fitted_radius_mm = prv_void_expected_borehole_radius_mm;
        center_x_mm      = 0;
        center_y_mm      = 0;
        DEBUG_SEND("Void: Circle fit failed. Using fallback radius: %u mm", prv_void_expected_borehole_radius_mm);
    }
    else
    {
        DEBUG_SEND("Void: Circle fit SUCCESS - Center: (%ld, %ld) mm, Radius: %u mm", (long)center_x_mm, (long)center_y_mm, fitted_radius_mm);
    }

    // Get nominal radius from uphole-provided configuration for baseline calculation
    uint16_t nominal_radius_mm = prv_void_current_config.default_wall_distance / 2U;
    DEBUG_SEND("Void: Nominal radius from config: %u mm", nominal_radius_mm);

    // Step 2: Calculate expected distances and compare with actual measurements for each sensor
    for (uint8_t i = 0; i < MAX_SENSORS; i++)
    {
        // Skip invalid measurements
        if (current_measurements[i].status != WALL_STATUS_VALID)
        {
            DEBUG_SEND("Void: Sensor %u - SKIPPED (status: %u)", i, current_measurements[i].status);
            continue;
        }

        // Calculate expected distance based on sensor angular position and probe offset
        // Using integer trigonometry approximation (120-degree spacing)
        static const int32_t cos_120_deg[MAX_SENSORS] = { 1000, -500, -500 }; // cos(0°, 120°, 240°) * 1000
        static const int32_t sin_120_deg[MAX_SENSORS] = { 0, 866, -866 };     // sin(0°, 120°, 240°) * 1000

        // Project offset onto sensor direction (fixed point arithmetic with scale 1000)
        int32_t offset_projection = (center_x_mm * cos_120_deg[i] + center_y_mm * sin_120_deg[i]) / 1000;

        // Calculate expected distance (baseline): nominal radius adjusted by offset projection
        // If probe is offset toward a sensor, expected distance decreases
        // If probe is offset away from a sensor, expected distance increases
        int32_t expected_distance_mm = (int32_t)nominal_radius_mm - offset_projection;

        // Ensure expected distance is reasonable (positive and within bounds)
        if (expected_distance_mm < 100) // Minimum 10cm expected distance
        {
            expected_distance_mm = 100;
            DEBUG_SEND("Void: Sensor %u - Expected distance clamped to minimum (100mm)", i);
        }
        else if (expected_distance_mm > (int32_t)prv_void_current_config.range_max)
        {
            expected_distance_mm = (int32_t)prv_void_current_config.range_max;
            DEBUG_SEND("Void: Sensor %u - Expected distance clamped to maximum (%u mm)", i, prv_void_current_config.range_max);
        }

        // Compare actual vs expected distance
        int32_t actual_distance_mm = (int32_t)current_measurements[i].distance;
        int32_t deviation_mm       = actual_distance_mm - expected_distance_mm;

        DEBUG_SEND("Void: Sensor %u - Actual: %ld mm, Expected: %ld mm, Deviation: %ld mm", i, (long)actual_distance_mm, (long)expected_distance_mm, (long)deviation_mm);

        // Only consider positive deviations (farther than expected = potential void)
        if (deviation_mm > (int32_t)prv_void_current_config.detection_threshold)
        {
            // Calculate confidence for this void detection
            uint8_t confidence = prv_void_calculate_confidence(current_measurements[i].snr, (uint16_t)deviation_mm, current_measurements[i].distance);

            DEBUG_SEND("Void: Sensor %u - POTENTIAL VOID detected - Size: %ld mm, SNR: %u, Confidence: %u%%",
                       i,
                       (long)deviation_mm,
                       current_measurements[i].snr,
                       confidence);

            // Check if confidence meets threshold for confirmed void detection
            if (confidence >= prv_void_current_config.confidence_threshold)
            {
                // Create void detection structure with all required parameters
                void_detection_t detection = {
                    .sector     = i,                      // Sensor index (0-2)
                    .size_mm    = (uint16_t)deviation_mm, // Void size in millimeters
                    .confidence = confidence,             // Confidence percentage (0-100%)
                    .timestamp  = HAL_GetTick()           // Current timestamp
                };

                DEBUG_SEND("Void: CONFIRMED VOID DETECTION:");
                DEBUG_SEND("  - Sector: %u", detection.sector);
                DEBUG_SEND("  - Size: %u mm", detection.size_mm);
                DEBUG_SEND("  - Confidence: %u%% (%s)", detection.confidence, prv_void_get_confidence_text(detection.confidence));
                DEBUG_SEND("  - Timestamp: %lu ms", (unsigned long)detection.timestamp);
                DEBUG_SEND("  - Threshold used: %u mm", prv_void_current_config.detection_threshold);
                DEBUG_SEND("  - Min confidence: %u%%", prv_void_current_config.confidence_threshold);

                // Report void detection to uphole system
                prv_void_report_detection(&detection);

                // Store detection in history for analysis
                // TODO: Re-evaluate if void detection history is needed. Call to prv_void_store_detection_to_history is currently disabled.
                // prv_void_store_detection_to_history(detection);
            }
            else
            {
                DEBUG_SEND("Void: Sensor %u - Void rejected due to low confidence (%u%% < %u%%)", i, confidence, prv_void_current_config.confidence_threshold);
            }
        }
        else if (deviation_mm > 0)
        {
            DEBUG_SEND("Void: Sensor %u - Small positive deviation (%ld mm) below threshold (%u mm)",
                       i,
                       (long)deviation_mm,
                       prv_void_current_config.detection_threshold);
        }
        else
        {
            DEBUG_SEND("Void: Sensor %u - Negative/zero deviation (%ld mm) - no void", i, (long)deviation_mm);
        }
    }

    // Update wall profile history with this processed data
    wall_profile_t profile = {
        .center_x_mm        = center_x_mm,      // Circle center X coordinate
        .center_y_mm        = center_y_mm,      // Circle center Y coordinate
        .borehole_radius_mm = fitted_radius_mm, // Fitted borehole radius
        .timestamp          = HAL_GetTick()     // Current timestamp
    };

    // Copy measurements to profile
    for (uint8_t i = 0; i < MAX_SENSORS; i++)
    {
        profile.wall_distance_mm[i] = current_measurements[i].distance;
        profile.wall_status[i]      = current_measurements[i].status;
    }

    prv_void_store_profile_to_history(profile);

    DEBUG_SEND("Void: Profile stored - Center: (%ld, %ld) mm, Radius: %u mm", (long)profile.center_x_mm, (long)profile.center_y_mm, profile.borehole_radius_mm);
}

/**
 * @brief Fit circle using integer arithmetic for 3-point circle fitting
 * @param measurements Array of wall measurements
 * @param center_x_mm Pointer to store center X coordinate in mm
 * @param center_y_mm Pointer to store center Y coordinate in mm
 * @return Fitted radius in mm, 0 if fitting failed
 */
static uint16_t prv_void_fit_circle_integer(const wall_measurement_t measurements[MAX_SENSORS], int32_t *center_x_mm, int32_t *center_y_mm)
{
    // Pre-calculated cos/sin values for 120-degree spacing (scaled by 1000)
    static const int32_t cos_values[MAX_SENSORS] = { 1000, -500, -500 }; // cos(0°, 120°, 240°) * 1000
    static const int32_t sin_values[MAX_SENSORS] = { 0, 866, -866 };     // sin(0°, 120°, 240°) * 1000

    // Convert measurements to Cartesian coordinates (scaled by 1000)
    int32_t pts_x[3], pts_y[3];
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < MAX_SENSORS && valid_count < 3; ++i)
    {
        if (measurements[i].status == WALL_STATUS_VALID)
        {
            // Scale distance and apply trigonometry (all scaled by 1000)
            pts_x[valid_count] = ((int32_t)measurements[i].distance * cos_values[i]) / 1000;
            pts_y[valid_count] = ((int32_t)measurements[i].distance * sin_values[i]) / 1000;
            valid_count++;
        }
    }

    *center_x_mm = 0;
    *center_y_mm = 0;

    if (valid_count < 3)
    {
        return prv_void_get_fallback_average_distance(measurements, prv_void_current_config.default_wall_distance);
    }

    // Perform 3-point circle fit using integer arithmetic
    int32_t x1 = pts_x[0], y1 = pts_y[0];
    int32_t x2 = pts_x[1], y2 = pts_y[1];
    int32_t x3 = pts_x[2], y3 = pts_y[2];

    // Calculate determinant D = 2 * (x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
    int32_t D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

    // Check for collinearity (determinant too small)
    if (abs(D) < 1000) // Scaled threshold
    {
        return prv_void_get_fallback_average_distance(measurements, prv_void_current_config.default_wall_distance);
    }

    // Calculate squared terms
    int32_t x1y1_sq = x1 * x1 + y1 * y1;
    int32_t x2y2_sq = x2 * x2 + y2 * y2;
    int32_t x3y3_sq = x3 * x3 + y3 * y3;

    // Calculate center coordinates
    int32_t center_x_num = x1y1_sq * (y2 - y3) + x2y2_sq * (y3 - y1) + x3y3_sq * (y1 - y2);
    int32_t center_y_num = x1y1_sq * (x3 - x2) + x2y2_sq * (x1 - x3) + x3y3_sq * (x2 - x1);

    *center_x_mm = -center_x_num / D;
    *center_y_mm = -center_y_num / D;

    // Calculate radius using integer arithmetic
    uint32_t r_sum = 0;
    for (uint8_t i = 0; i < 3; ++i)
    {
        int32_t  dx        = pts_x[i] - (*center_x_mm);
        int32_t  dy        = pts_y[i] - (*center_y_mm);
        uint32_t r_squared = (uint32_t)(dx * dx + dy * dy);
        r_sum += prv_void_integer_sqrt(r_squared);
    }

    return (uint16_t)(r_sum / 3U);
}

/**
 * @brief Calculate confidence score using integer arithmetic
 * @param snr Signal-to-Noise Ratio (0-255)
 * @param void_size_mm Estimated void size/depth in mm
 * @param distance_mm Distance to wall at that point in mm
 * @return Confidence score as percentage (0-100%)
 */
static uint8_t prv_void_calculate_confidence(uint8_t snr, uint16_t void_size_mm, uint16_t distance_mm)
{
    uint8_t confidence   = 0;
    uint8_t snr_contrib  = 0;
    uint8_t size_contrib = 0;
    uint8_t dist_contrib = 0;

    // Factor 1: Signal quality (SNR-based) - 40% max contribution
    if (snr >= CONFIDENCE_LEVEL_HIGH)
    {
        snr_contrib = 40; // High SNR contributes 40%
    }
    else if (snr >= CONFIDENCE_LEVEL_MEDIUM)
    {
        snr_contrib = 25; // Medium SNR contributes 25%
    }
    else if (snr >= CONFIDENCE_LEVEL_LOW)
    {
        snr_contrib = 10; // Low SNR contributes 10%
    }
    // SNR below CONFIDENCE_LEVEL_LOW contributes 0%
    confidence += snr_contrib;

    // Factor 2: Void size significance - 30% max contribution
    if (void_size_mm >= (prv_void_current_config.detection_threshold * 2U))
    {
        size_contrib = 30; // Large void (2x threshold) adds 30%
    }
    else if (void_size_mm >= prv_void_current_config.detection_threshold)
    {
        size_contrib = 20; // Threshold-level void adds 20%
    }
    // Void size below threshold contributes 0%
    confidence += size_contrib;

    // Factor 3: Distance-based reliability - 30% max contribution
    if (distance_mm <= 2000U) // Within 2.0m
    {
        dist_contrib = 30; // Close range adds 30%
    }
    else if (distance_mm <= 3000U) // Within 3.0m
    {
        dist_contrib = 15; // Medium range adds 15%
    }
    // Distance beyond 3.0m contributes 0%
    confidence += dist_contrib;

    // Cap confidence at 100%
    if (confidence > 100U)
    {
        confidence = 100U;
    }

    // Debug printout of contributing factors
    DEBUG_SEND("Confidence: SNR=%u->%u%%, Size=%umm->%u%%, Dist=%umm->%u%% = %u%%", snr, snr_contrib, void_size_mm, size_contrib, distance_mm, dist_contrib, confidence);

    return confidence;
}

static void prv_void_init_wall_profile_history(void)
{
    memset(prv_void_wall_profiles_history, 0, sizeof(prv_void_wall_profiles_history));
    prv_void_wall_profile_history_idx = 0;
}

/**
 * @brief Store wall profile in circular history buffer
 * @param profile Wall profile to store
 */
static void prv_void_store_profile_to_history(wall_profile_t profile)
{
    // Store in circular buffer
    prv_void_wall_profiles_history[prv_void_wall_profile_history_idx] = profile;

    // Move to next position in circular buffer
    prv_void_wall_profile_history_idx = (prv_void_wall_profile_history_idx + 1U) % WALL_HISTORY_SIZE;
}

// Implemented as per plan
static void prv_void_store_detection_to_history(void_detection_t void_data)
{
    if (MAX_VOID_DETECTIONS == 0)
    {
        return; // Avoid division by zero or out-of-bounds if history size is zero
    }
    prv_void_detections_history[prv_void_detection_history_write_idx] = void_data;
    prv_void_detection_history_write_idx                              = (prv_void_detection_history_write_idx + 1U) % MAX_VOID_DETECTIONS;

    if (prv_void_detection_history_count < MAX_VOID_DETECTIONS)
    {
        prv_void_detection_history_count++;
    }
    // Optional: Add a DEBUG_SEND here if needed
}

bool void_get_last_detection(void_detection_t *last_detection)
{
    if (last_detection == NULL || prv_void_detection_history_count == 0 || MAX_VOID_DETECTIONS == 0)
    {
        return false;
    }

    // Calculate index of the last (newest) entry
    uint8_t read_idx = (prv_void_detection_history_write_idx - 1 + MAX_VOID_DETECTIONS) % MAX_VOID_DETECTIONS;
    *last_detection  = prv_void_detections_history[read_idx];
    return true;
}

uint8_t void_get_detection_history(void_detection_t *history_buffer, uint8_t buffer_size)
{
    if (history_buffer == NULL || buffer_size == 0 || prv_void_detection_history_count == 0 || MAX_VOID_DETECTIONS == 0)
    {
        return 0;
    }

    uint8_t num_to_copy  = (buffer_size < prv_void_detection_history_count) ? buffer_size : prv_void_detection_history_count;
    uint8_t copied_count = 0;

    // Start from the newest entry (one before current write_idx) and go backwards
    for (uint8_t i = 0; i < num_to_copy; i++)
    {
        // Calculate wrapped-around index for reading in reverse chronological order
        uint8_t read_idx               = (prv_void_detection_history_write_idx - 1 - i + MAX_VOID_DETECTIONS * (i + 1)) % MAX_VOID_DETECTIONS;
        history_buffer[copied_count++] = prv_void_detections_history[read_idx];
    }
    return copied_count;
}

void void_clear_detection_history(void)
{
    memset(prv_void_detections_history, 0, sizeof(prv_void_detections_history));
    prv_void_detection_history_write_idx = 0;
    prv_void_detection_history_count     = 0;
    DEBUG_SEND("Void: Detection history cleared.");
}

bool void_update_config(const void_config_t *new_config)
{
    if (new_config == NULL)
    {
        DEBUG_SEND("Void Config Error: NULL config pointer");
        return false;
    }

    // Validate configuration parameters
    if (new_config->detection_threshold < 10 || new_config->detection_threshold > 1000)
    {
        DEBUG_SEND("Void Config Error: Invalid detection threshold (%u mm)", new_config->detection_threshold);
        return false;
    }

    if (new_config->confidence_threshold > 100)
    {
        DEBUG_SEND("Void Config Error: Invalid confidence threshold (%u%%)", new_config->confidence_threshold);
        return false;
    }

    if (new_config->range_min >= new_config->range_max)
    {
        DEBUG_SEND("Void Config Error: Invalid range (min=%u >= max=%u)", new_config->range_min, new_config->range_max);
        return false;
    }

    if (new_config->default_wall_distance < new_config->range_min || new_config->default_wall_distance > new_config->range_max)
    {
        DEBUG_SEND("Void Config Error: Default wall distance (%u mm) outside valid range (%u-%u mm)",
                   new_config->default_wall_distance,
                   new_config->range_min,
                   new_config->range_max);
        return false;
    }

    if (new_config->baseline_method >= 10) // Arbitrary upper limit for method selection
    {
        DEBUG_SEND("Void Config Error: Invalid baseline method (%u)", new_config->baseline_method);
        return false;
    }

    if (new_config->multi_point_algorithm >= MULTI_POINT_COUNT)
    {
        DEBUG_SEND("Void Config Error: Invalid multi-point algorithm (%u)", new_config->multi_point_algorithm);
        return false;
    }

    // Configuration is valid - apply it
    prv_void_current_config = *new_config;

    // Update derived values
    prv_void_expected_borehole_radius_mm = prv_void_current_config.default_wall_distance / 2U;

    DEBUG_SEND("Void: Configuration updated successfully");
    DEBUG_SEND("  - Detection threshold: %u mm", prv_void_current_config.detection_threshold);
    DEBUG_SEND("  - Confidence threshold: %u%%", prv_void_current_config.confidence_threshold);
    DEBUG_SEND("  - Range: %u-%u mm", prv_void_current_config.range_min, prv_void_current_config.range_max);
    DEBUG_SEND("  - Default wall distance: %u mm", prv_void_current_config.default_wall_distance);
    DEBUG_SEND("  - Baseline method: %u", prv_void_current_config.baseline_method);
    DEBUG_SEND("  - Multi-point algorithm: %u", prv_void_current_config.multi_point_algorithm);

    return true;
}

void_config_t void_get_config(void)
{
    return prv_void_current_config;
}

radar_hw_status_t void_get_subsystem_hw_status(void)
{
    if (!prv_void_is_initialized)
    {
        return RADAR_HW_STATUS_INITIALISING; // Use existing enum value
    }

    // Check CAN bus status first
    if (can_get_state() != CAN_STATE_READY)
    {
        return RADAR_HW_STATUS_ERROR; // Use existing enum value
    }

    // Check individual sensor status from latest measurements
    uint8_t error_count = 0;
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < MAX_SENSORS; i++)
    {
        switch (prv_void_latest_measurements[i].status)
        {
        case WALL_STATUS_VALID:
            valid_count++;
            break;
        case WALL_STATUS_ERROR:
            error_count++;
            break;
        case WALL_STATUS_NO_DETECTION:
        case WALL_STATUS_OUT_OF_RANGE:
        case WALL_STATUS_SUSPICIOUS:
        default:
            // These are not considered hard errors
            break;
        }
    }

    // If any sensors have hard errors, report error status
    if (error_count > 0)
    {
        return RADAR_HW_STATUS_ERROR; // Use existing enum value
    }

    // If we have at least minimum valid sensors, consider system ready
    if (valid_count >= MIN_VALID_SAMPLES)
    {
        return RADAR_HW_STATUS_READY; // Use existing enum value
    }

    // Otherwise, system is initializing or has insufficient data
    return RADAR_HW_STATUS_INITIALISING; // Use existing enum value (changed from INIT)
}

// Update prv_void_report_detection to enable history storage
static void prv_void_report_detection(const void_detection_t *void_data)
{
    if (void_data == NULL)
    {
        DEBUG_SEND("Void Report Error: Null data pointer.");
        return;
    }

    // Store to history first - now enabled
    prv_void_store_detection_to_history(*void_data);

    // Switch to uphole communication channel
    uart_tx_channel_set(UART_UPHOLE);

    // Send void event using established protocol (!vd,<sector>,<size_mm>,<confidence>)
    printf("!vd,%u,%u,%u\n", (unsigned)void_data->sector, (unsigned)void_data->size_mm, (unsigned)void_data->confidence);

    // Restore previous communication channel
    uart_tx_channel_undo();

    DEBUG_SEND("Void: Reported Event - Sector %u, Size %u mm, Confidence %u%%",
               (unsigned int)void_data->sector,
               (unsigned int)void_data->size_mm,
               (unsigned int)void_data->confidence);
}

// Add missing prv_void_get_confidence_text function
static const char *prv_void_get_confidence_text(uint8_t confidence)
{
    if (confidence >= CONFIDENCE_LEVEL_HIGH)
    {
        return "HIGH";
    }
    else if (confidence >= CONFIDENCE_LEVEL_MEDIUM)
    {
        return "MEDIUM";
    }
    else if (confidence >= CONFIDENCE_LEVEL_LOW)
    {
        return "LOW";
    }
    else
    {
        return "VERY_LOW";
    }
}

// Add missing prv_void_integer_sqrt function
static uint16_t prv_void_integer_sqrt(uint32_t value)
{
    if (value == 0)
    {
        return 0;
    }

    // Newton's method for integer square root
    uint32_t x = value;
    uint32_t y = (x + 1) / 2;

    while (y < x)
    {
        x = y;
        y = (x + value / x) / 2;
    }

    return (uint16_t)x;
}

// Add missing prv_void_request_data_from_radars function
static void prv_void_request_data_from_radars(void)
{
    for (uint8_t sensor_idx = 0; sensor_idx < MAX_SENSORS; sensor_idx++)
    {
        if (can_send_to_sensor(sensor_idx, CAN_CMD_PROFILE, NULL, 0) != CAN_STATE_READY)
        {
            DEBUG_SEND("Void: Failed to request data from sensor %u", sensor_idx);
        }
    }
}
