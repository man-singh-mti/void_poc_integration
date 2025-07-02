/**
 * @file mti_void.c
 * @brief Void detection system implementation
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#include "mti_void.h"
#include "mti_radar.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include "mti_radar_types.h"
#include <string.h>
#include <math.h>

/*------------------------------------------------------------------------------
 * Static Variables and Configuration
 *----------------------------------------------------------------------------*/

/** @brief System configuration */
static void_config_t config = { .algorithm              = VOID_ALGORITHM_SIMPLE,
                                .baseline_diameter_mm   = VOID_DEFAULT_BASELINE_MM,
                                .threshold_mm           = VOID_DEFAULT_THRESHOLD_MM,
                                .confidence_min_percent = VOID_DEFAULT_CONFIDENCE_MIN,
                                .min_distance_mm        = 50,
                                .max_distance_mm        = 5000,
                                .auto_fallback_enabled  = true,
                                .median_filter_enabled  = false };

/** @brief Latest detection results */
static void_data_t latest_results = { 0 };

/** @brief Latest measurement data */
static void_measurement_t latest_measurement = { 0 };

/** @brief Processing state */
static struct
{
    bool                   system_initialized;
    bool                   new_results_available;
    bool                   new_measurement_available;
    uint32_t               last_process_time;
    uint32_t               init_time_ms;
    void_detection_state_t detection_state;
    uint32_t               total_detections;
    uint32_t               algorithm_switches;
    // Event-driven fields
    bool     auto_streaming_enabled;
    uint32_t immediate_processing_count;
    uint32_t automatic_stream_count;
    // System test fields
    uint32_t last_test_time;
    uint32_t test_counter;
    uint32_t startup_phase;
    bool     test_sensors_started;
} void_state = { .system_initialized         = false,
                 .auto_streaming_enabled     = false,
                 .immediate_processing_count = 0,
                 .automatic_stream_count     = 0,
                 .last_test_time             = 0,
                 .test_counter               = 0,
                 .startup_phase              = 0,
                 .test_sensors_started       = false };

/** @brief Median filter buffers */
static uint16_t median_buffer[MAX_RADAR_SENSORS][3] = { 0 };
static uint8_t  median_index[MAX_RADAR_SENSORS]     = { 0 };

/** @brief Simple running state flag */
static bool void_system_running = false;

/*------------------------------------------------------------------------------
 * Forward Declarations
 *----------------------------------------------------------------------------*/

static bool     process_radar_data(void);
static bool     run_simple_algorithm(const radar_distance_t *radar_data);
static bool     run_circle_fit_algorithm(const radar_distance_t *radar_data);
static bool     run_bypass_algorithm(const radar_distance_t *radar_data);
static uint16_t apply_median_filter(uint8_t sensor_idx, uint16_t new_value);
static uint8_t  calculate_confidence(const radar_distance_t *radar_data, bool void_detected);
static void     update_measurement_data(const radar_distance_t *radar_data);
static void     generate_void_event(bool detected, uint8_t confidence, const char *algorithm_name);
static bool     validate_radar_data(const radar_distance_t *radar_data);
static void     void_send_automatic_stream_internal(void);
static void     void_send_immediate_stream(void);


/*------------------------------------------------------------------------------
 * Circle Fitting Functions
 *----------------------------------------------------------------------------*/

/**
 * @brief Perform 3-point circle fitting
 *
 * Uses the standard 3-point circle fitting algorithm to determine
 * borehole center and diameter from radar measurements
 */
static bool  calculate_circle_fit(const radar_distance_t *radar_data, circle_fit_result_t *result);
static float calculate_circle_quality(const radar_distance_t *radar_data, const circle_fit_result_t *fit);

/*------------------------------------------------------------------------------
 * System Initialization
 *----------------------------------------------------------------------------*/

bool void_system_init(void)
{
    debug_send("VOID: Initializing detection system");

    // Initialize data structures
    memset(&latest_results, 0, sizeof(latest_results));
    memset(&latest_measurement, 0, sizeof(latest_measurement));
    memset(&median_buffer, 0, sizeof(median_buffer));
    memset(&median_index, 0, sizeof(median_index));

    // Initialize state
    void_state.system_initialized        = true;
    void_state.new_results_available     = false;
    void_state.new_measurement_available = false;
    void_state.last_process_time         = HAL_GetTick();
    void_state.init_time_ms              = HAL_GetTick();
    void_state.detection_state           = VOID_STATE_NO_DETECTION;
    void_state.total_detections          = 0;
    void_state.algorithm_switches        = 0;

    // Initialize results structure
    strcpy(latest_results.status_text, "System initialized");
    latest_results.algorithm_used    = config.algorithm;
    latest_results.detection_time_ms = HAL_GetTick();

    debug_send("VOID: System initialized - Algorithm: %d, Baseline: %dmm, Threshold: %dmm", config.algorithm, config.baseline_diameter_mm, config.threshold_mm);

    return true;
}

bool void_is_system_ready(void)
{
    // For POC: Just check if void and CAN systems are initialized
    if (!void_state.system_initialized)
    {
        return false;
    }

    // Check if CAN system is initialized (not necessarily healthy)
    if (!can_is_system_initialized()) // Or whatever the correct function name is
    {
        return false;
    }

    // For POC: Don't require radar to be "healthy", just initialized
    return true;
}

/*------------------------------------------------------------------------------
 * Main Processing Function
 *----------------------------------------------------------------------------*/

void void_system_process(void)
{
    if (!void_is_system_ready())
    {
        return;
    }

    // NEW: Process immediately when radar has new data (no rate limiting)
    if (!radar_has_new_data())
    {
        return; // Exit immediately if no new radar data
    }

    uint32_t current_time = HAL_GetTick();

    // debug_send("[V]proc");

    // Process new radar data immediately
    if (process_radar_data())
    {
        radar_mark_data_processed();
        void_state.immediate_processing_count++;

        // Send automatic stream if enabled and in operational mode
        if (void_state.auto_streaming_enabled && system_is_operational_mode())
        {
            void_send_immediate_stream();
        }
    }

    void_state.last_process_time = current_time;
}

static bool process_radar_data(void)
{
    radar_distance_t radar_data;

    // Get latest radar measurements
    if (!radar_get_latest_measurements(&radar_data))
    {
        return false;
    }

    // Validate radar data
    if (!validate_radar_data(&radar_data))
    {
        strcpy(latest_results.status_text, "Invalid radar data");
        latest_results.void_detected      = false;
        latest_results.confidence_percent = 0;
        void_state.new_results_available  = true;
        return false;
    }

    // Update measurement data for uphole transmission
    update_measurement_data(&radar_data);

    // Run detection algorithm
    bool        void_detected  = false;
    const char *algorithm_name = "unknown";

    switch (config.algorithm)
    {
    case VOID_ALGORITHM_BYPASS:
        void_detected  = run_bypass_algorithm(&radar_data);
        algorithm_name = "bypass";
        break;

    case VOID_ALGORITHM_SIMPLE:
        void_detected  = run_simple_algorithm(&radar_data);
        algorithm_name = "simple";
        break;

    case VOID_ALGORITHM_CIRCLEFIT:
        void_detected  = run_circle_fit_algorithm(&radar_data);
        algorithm_name = "circlefit";
        break;

    default:
        debug_send("VOID: Unknown algorithm %d", config.algorithm);
        return false;
    }

    // Calculate confidence
    uint8_t confidence = calculate_confidence(&radar_data, void_detected);

    // Apply confidence threshold
    if (confidence < config.confidence_min_percent)
    {
        void_detected = false;
        // debug_send("[V]C%d<%d", confidence, config.confidence_min_percent);
    }

    // Update detection state
    void_detection_state_t new_state = void_detected ? VOID_STATE_CONFIRMED : VOID_STATE_NO_DETECTION;
    if (new_state != void_state.detection_state)
    {
        void_state.detection_state = new_state;
        if (void_detected)
        {
            void_state.total_detections++;
        }
    }

    // Generate results
    latest_results.void_detected        = void_detected;
    latest_results.algorithm_used       = config.algorithm;
    latest_results.confidence_percent   = confidence;
    latest_results.sensor_count_used    = radar_data.valid_sensor_count;
    latest_results.detection_time_ms    = HAL_GetTick();
    latest_results.new_result_available = true;

    // Calculate void size (simple estimation)
    if (void_detected)
    {
        uint16_t max_distance = 0;
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            if (radar_data.data_valid[i] && radar_data.distance_mm[i] > max_distance)
            {
                max_distance = radar_data.distance_mm[i];
            }
        }
        latest_results.void_size_mm = max_distance > config.baseline_diameter_mm ? max_distance - config.baseline_diameter_mm : 0;
    }
    else
    {
        latest_results.void_size_mm = 0;
    }

    // Generate status text
    if (void_detected)
    {
        snprintf(latest_results.status_text, sizeof(latest_results.status_text), "Void detected: %dmm (%s)", latest_results.void_size_mm, algorithm_name);
    }
    else
    {
        strcpy(latest_results.status_text, "No void detected");
    }

    void_state.new_results_available = true;

    // debug_send("[V]%s C%d Sz%d S%d", latest_results.void_detected ? "D" : "N", confidence, latest_results.void_size_mm, radar_data.valid_sensor_count);

    return true;
}

/*------------------------------------------------------------------------------
 * Algorithm Implementations
 *----------------------------------------------------------------------------*/

static bool run_bypass_algorithm(const radar_distance_t *radar_data)
{
    // Bypass algorithm - always return false (no detection)
    return false;
}

static bool run_simple_algorithm(const radar_distance_t *radar_data)
{
    uint8_t  void_sensors = 0;
    uint16_t threshold    = config.baseline_diameter_mm + config.threshold_mm;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_data->data_valid[i])
        {
            uint16_t distance = radar_data->distance_mm[i];

            // Apply median filter if enabled
            if (config.median_filter_enabled)
            {
                distance = apply_median_filter(i, distance);
            }

            // Check if distance exceeds threshold
            if (distance > threshold)
            {
                void_sensors++;
                // debug_send("[V]S%d>%d", i, threshold);
            }
        }
    }

    // Detection requires at least 1 sensor showing void
    bool detected = (void_sensors > 0);

    // debug_send("[V]S%d/%d", void_sensors, radar_data->valid_sensor_count);

    return detected;
}

static bool run_circle_fit_algorithm(const radar_distance_t *radar_data)
{
    // Require at least 3 sensors for circle fitting
    if (radar_data->valid_sensor_count < 3)
    {
        // debug_send("[V]CF need3 S%d", radar_data->valid_sensor_count);

        // Auto-fallback to simple algorithm if enabled
        if (config.auto_fallback_enabled)
        {
            // debug_send("[V]fallback");
            void_state.algorithm_switches++;
            return run_simple_algorithm(radar_data);
        }
        return false;
    }

    circle_fit_result_t fit_result;
    if (!calculate_circle_fit(radar_data, &fit_result))
    {
        // debug_send("[V]CFfail");

        // Auto-fallback to simple algorithm if enabled
        if (config.auto_fallback_enabled)
        {
            // debug_send("[V]fallback");
            void_state.algorithm_switches++;
            return run_simple_algorithm(radar_data);
        }
        return false;
    }

    // Check if fitted diameter is significantly larger than baseline
    uint16_t threshold_diameter = config.baseline_diameter_mm + config.threshold_mm;
    bool     void_detected      = fit_result.diameter_mm > threshold_diameter;

    // debug_send("[V]CF D%d Q%.2f %s", fit_result.diameter_mm, fit_result.fit_quality, void_detected ? "D" : "N");

    return void_detected;
}

/*------------------------------------------------------------------------------
 * Circle Fitting Implementation
 *----------------------------------------------------------------------------*/

static bool calculate_circle_fit(const radar_distance_t *radar_data, circle_fit_result_t *result)
{
    if (!result || radar_data->valid_sensor_count < 3)
    {
        return false;
    }

    // Convert radar measurements to Cartesian coordinates
    // Assuming sensors at 0°, 120°, 240° around borehole
    float   points[3][2]; // [x, y] coordinates
    uint8_t point_count = 0;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS && point_count < 3; i++)
    {
        if (radar_data->data_valid[i])
        {
            float angle_rad  = radar_data->angle_deg[i] * M_PI / 180.0f;
            float distance_m = radar_data->distance_mm[i] / 1000.0f;

            points[point_count][0] = distance_m * cosf(angle_rad); // x
            points[point_count][1] = distance_m * sinf(angle_rad); // y
            point_count++;
        }
    }

    if (point_count < 3)
    {
        return false;
    }

    // 3-point circle fitting algorithm
    float x1 = points[0][0], y1 = points[0][1];
    float x2 = points[1][0], y2 = points[1][1];
    float x3 = points[2][0], y3 = points[2][1];

    // Calculate circle center using determinant method
    float a = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);

    if (fabsf(a) < 0.0001f) // Points are collinear
    {
        return false;
    }

    float b = (x1 * x1 + y1 * y1) * (y3 - y2) + (x2 * x2 + y2 * y2) * (y1 - y3) + (x3 * x3 + y3 * y3) * (y2 - y1);
    float c = (x1 * x1 + y1 * y1) * (x2 - x3) + (x2 * x2 + y2 * y2) * (x3 - x1) + (x3 * x3 + y3 * y3) * (x1 - x2);

    float center_x = -b / (2.0f * a);
    float center_y = -c / (2.0f * a);

    // Calculate radius
    float radius = sqrtf((center_x - x1) * (center_x - x1) + (center_y - y1) * (center_y - y1));

    // Convert back to millimeters
    result->center_x_mm    = (uint16_t)(fabsf(center_x) * 1000.0f);
    result->center_y_mm    = (uint16_t)(fabsf(center_y) * 1000.0f);
    result->radius_mm      = (uint16_t)(radius * 1000.0f);
    result->diameter_mm    = result->radius_mm * 2;
    result->fit_successful = true;

    // Calculate fit quality
    result->fit_quality = calculate_circle_quality(radar_data, result);

    // Check if fit is reasonable
    if (result->diameter_mm < 50 || result->diameter_mm > 1000 || result->fit_quality < 0.5f)
    {
        result->fit_successful = false;
        return false;
    }

    return true;
}

static float calculate_circle_quality(const radar_distance_t *radar_data, const circle_fit_result_t *fit)
{
    if (!fit->fit_successful)
    {
        return 0.0f;
    }

    // Calculate how well the fitted circle matches the actual measurements
    float   total_error  = 0.0f;
    uint8_t valid_points = 0;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_data->data_valid[i])
        {
            float angle_rad         = radar_data->angle_deg[i] * M_PI / 180.0f;
            float measured_distance = radar_data->distance_mm[i];

            // Calculate expected distance from fitted circle
            float expected_distance = fit->radius_mm;

            // Calculate error
            float error = fabsf(measured_distance - expected_distance);
            total_error += error;
            valid_points++;
        }
    }

    if (valid_points == 0)
    {
        return 0.0f;
    }

    float average_error = total_error / valid_points;
    float quality       = 1.0f - (average_error / VOID_CIRCLE_FIT_TOLERANCE_MM);

    return fmaxf(0.0f, fminf(1.0f, quality)); // Clamp to 0-1 range
}

/*------------------------------------------------------------------------------
 * Helper Functions
 *----------------------------------------------------------------------------*/

static uint16_t apply_median_filter(uint8_t sensor_idx, uint16_t new_value)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return new_value;
    }

    // Add new value to circular buffer
    median_buffer[sensor_idx][median_index[sensor_idx]] = new_value;
    median_index[sensor_idx]                            = (median_index[sensor_idx] + 1) % 3;

    // Simple 3-point median (bubble sort)
    uint16_t sorted[3];
    memcpy(sorted, median_buffer[sensor_idx], sizeof(sorted));

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2 - i; j++)
        {
            if (sorted[j] > sorted[j + 1])
            {
                uint16_t temp = sorted[j];
                sorted[j]     = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    return sorted[1]; // Return median value
}

static uint8_t calculate_confidence(const radar_distance_t *radar_data, bool void_detected)
{
    if (!radar_data->system_healthy || radar_data->valid_sensor_count == 0)
    {
        return 0;
    }

    // Base confidence on number of valid sensors and their quality
    float confidence = 0.0f;

    // Sensor count contribution (0-50%)
    confidence += (float)radar_data->valid_sensor_count / MAX_RADAR_SENSORS * 50.0f;

    // Signal quality contribution (0-50%)
    float avg_quality = 0.0f;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_data->data_valid[i])
        {
            avg_quality += radar_data->confidence[i];
        }
    }
    if (radar_data->valid_sensor_count > 0)
    {
        avg_quality /= radar_data->valid_sensor_count;
        confidence += avg_quality * 50.0f;
    }

    // Reduce confidence if system is not fully healthy
    if (!radar_data->system_healthy)
    {
        confidence *= 0.8f;
    }

    return (uint8_t)fminf(100.0f, fmaxf(0.0f, confidence));
}

static void update_measurement_data(const radar_distance_t *radar_data)
{
    // Copy radar data to measurement structure for uphole transmission
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        latest_measurement.distance_mm[i] = radar_data->distance_mm[i];
        latest_measurement.data_valid[i]  = radar_data->data_valid[i];
    }

    latest_measurement.valid_sensor_count = radar_data->valid_sensor_count;
    latest_measurement.timestamp_ms       = radar_data->timestamp_ms;

    // Set flags for transmission
    latest_measurement.flags = 0;
    if (latest_results.void_detected)
    {
        latest_measurement.flags |= 0x01;
    }
    if (radar_data->system_healthy)
    {
        latest_measurement.flags |= 0x02;
    }
    if (config.algorithm == VOID_ALGORITHM_CIRCLEFIT)
    {
        latest_measurement.flags |= 0x04;
    }
    void_state.new_measurement_available = true;
}

static bool validate_radar_data(const radar_distance_t *radar_data)
{
    if (!radar_data)
    {
        return false;
    }

    // Check if we have any valid sensors
    if (radar_data->valid_sensor_count == 0)
    {
        return false;
    }

    // Check if distances are within reasonable range
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_data->data_valid[i])
        {
            if (radar_data->distance_mm[i] < config.min_distance_mm || radar_data->distance_mm[i] > config.max_distance_mm)
            {
                debug_send("VOID: S%d distance %dmm out of range (%d-%dmm)", i, radar_data->distance_mm[i], config.min_distance_mm, config.max_distance_mm);
                return false;
            }
        }
    }

    return true;
}

/*------------------------------------------------------------------------------
 * Data Access Functions
 *----------------------------------------------------------------------------*/

bool void_get_latest_results(void_data_t *result)
{
    if (!result)
    {
        return false;
    }

    memcpy(result, &latest_results, sizeof(void_data_t));
    return void_state.new_results_available;
}

bool void_get_measurement_data(void_measurement_t *measurement)
{
    if (!measurement)
    {
        return false;
    }

    memcpy(measurement, &latest_measurement, sizeof(void_measurement_t));
    return void_state.new_measurement_available;
}

bool void_has_new_results(void)
{
    return void_state.new_results_available;
}

void void_mark_results_processed(void)
{
    void_state.new_results_available     = false;
    void_state.new_measurement_available = false;
    latest_results.new_result_available  = false;
}

/*------------------------------------------------------------------------------
 * Configuration Functions
 *----------------------------------------------------------------------------*/

bool void_set_algorithm(void_algorithm_t algorithm)
{
    if (algorithm > VOID_ALGORITHM_CIRCLEFIT)
    {
        return false;
    }

    if (config.algorithm != algorithm)
    {
        void_state.algorithm_switches++;
        debug_send("VOID: Algorithm changed from %d to %d", config.algorithm, algorithm);
    }

    config.algorithm = algorithm;
    return true;
}

bool void_set_baseline_diameter(uint16_t diameter_mm)
{
    if (diameter_mm < 50 || diameter_mm > 1000)
    {
        return false;
    }

    config.baseline_diameter_mm = diameter_mm;
    debug_send("VOID: Baseline diameter set to %dmm", diameter_mm);
    return true;
}

bool void_set_threshold(uint16_t threshold_mm)
{
    if (threshold_mm < 10 || threshold_mm > 500)
    {
        return false;
    }

    config.threshold_mm = threshold_mm;
    debug_send("VOID: Threshold set to %dmm", threshold_mm);
    return true;
}

bool void_set_confidence_threshold(uint8_t confidence_percent)
{
    if (confidence_percent > 100)
    {
        return false;
    }

    config.confidence_min_percent = confidence_percent;
    debug_send("VOID: Confidence threshold set to %d%%", confidence_percent);
    return true;
}

void void_set_auto_fallback(bool enabled)
{
    config.auto_fallback_enabled = enabled;
    debug_send("VOID: Auto fallback %s", enabled ? "enabled" : "disabled");
}

void void_set_median_filter(bool enabled)
{
    config.median_filter_enabled = enabled;
    debug_send("VOID: Median filter %s", enabled ? "enabled" : "disabled");
}

void void_get_config(void_config_t *config_out)
{
    if (config_out)
    {
        memcpy(config_out, &config, sizeof(void_config_t));
    }
}

/*------------------------------------------------------------------------------
 * Algorithm-Specific Functions
 *----------------------------------------------------------------------------*/

bool void_test_circle_fit(circle_fit_result_t *result)
{
    if (!result)
    {
        return false;
    }

    radar_distance_t radar_data;
    if (!radar_get_latest_measurements(&radar_data))
    {
        return false;
    }

    return calculate_circle_fit(&radar_data, result);
}

void_detection_state_t void_get_detection_state(void)
{
    return void_state.detection_state;
}

/*------------------------------------------------------------------------------
 * System Status Functions
 *----------------------------------------------------------------------------*/

void void_run_diagnostics(void)
{
    debug_send("=== VOID Detection System Diagnostics ===");
    debug_send("System initialized: %s", void_state.system_initialized ? "YES" : "NO");
    debug_send("System ready: %s", void_is_system_ready() ? "YES" : "NO");
    debug_send("Detection state: %d", void_state.detection_state);
    debug_send("");

    debug_send("Configuration:");
    debug_send("  Algorithm: %d (%s)",
               config.algorithm,
               config.algorithm == VOID_ALGORITHM_BYPASS   ? "bypass"
               : config.algorithm == VOID_ALGORITHM_SIMPLE ? "simple"
                                                           : "circlefit");
    debug_send("  Baseline: %dmm", config.baseline_diameter_mm);
    debug_send("  Threshold: %dmm", config.threshold_mm);
    debug_send("  Min confidence: %d%%", config.confidence_min_percent);
    debug_send("  Auto fallback: %s", config.auto_fallback_enabled ? "enabled" : "disabled");
    debug_send("  Median filter: %s", config.median_filter_enabled ? "enabled" : "disabled");
    debug_send("");

    debug_send("Latest Results:");
    debug_send("  Void detected: %s", latest_results.void_detected ? "YES" : "NO");
    debug_send("  Confidence: %d%%", latest_results.confidence_percent);
    debug_send("  Void size: %dmm", latest_results.void_size_mm);
    debug_send("  Sensors used: %d", latest_results.sensor_count_used);
    debug_send("  Status: %s", latest_results.status_text);
    debug_send("");

    debug_send("Event-Driven Stats:");
    debug_send("  Auto streaming: %s", void_state.auto_streaming_enabled ? "enabled" : "disabled");
    debug_send("  Immediate processing count: %lu", void_state.immediate_processing_count);
    debug_send("  Automatic stream count: %lu", void_state.automatic_stream_count);
    debug_send("");

    debug_send("Statistics:");
    debug_send("  Total detections: %d", void_state.total_detections);
    debug_send("  Algorithm switches: %d", void_state.algorithm_switches);
    debug_send("  Uptime: %dms", HAL_GetTick() - void_state.init_time_ms);
    debug_send("========================================");
}

void void_get_statistics(uint32_t *total_detections, uint32_t *algorithm_switches, uint32_t *uptime_ms)
{
    if (total_detections)
    {
        *total_detections = void_state.total_detections;
    }
    if (algorithm_switches)
    {
        *algorithm_switches = void_state.algorithm_switches;
    }
    if (uptime_ms)
    {
        *uptime_ms = HAL_GetTick() - void_state.init_time_ms;
    }
}

// Add this internal function to mti_void.c
static void void_send_automatic_stream_internal(void)
{
    // Only send when we have new measurement data
    if (!void_state.new_measurement_available)
    {
        return;
    }

    uart_tx_channel_set(UART_UPHOLE);

    void_measurement_t measurement = latest_measurement;
    void_data_t        result      = latest_results;

    // Format: &vd,<flags>,<d0>,<d1>,<d2>,<v0>,<v1>,<v2>,<conf>
    uint8_t flags = 0;
    flags |= (config.algorithm & 0x03); // Bits 0-1: Algorithm
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (measurement.data_valid[i])
        {
            flags |= (0x04 << i); // Bits 2-4: Sensor validity
        }
    }
    if (result.void_detected)
    {
        flags |= 0x20; // Bit 5: Void detected
    }

    // Send measurement data
    printf("&vd,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
           flags,
           measurement.distance_mm[0],
           measurement.distance_mm[1],
           measurement.distance_mm[2],
           result.void_detected ? 1 : 0,
           0, // Reserved
           0, // Reserved
           result.confidence_percent);

    uart_tx_channel_undo();

    // Mark data as transmitted
    void_mark_results_processed();
}

static void void_send_immediate_stream(void)
{
    if (!latest_results.new_result_available)
    {
        return;
    }

    uart_tx_channel_set(UART_UPHOLE);

    // Send void detection event immediately
    printf("!vd,%lu,%s,%d,%d,%s\r\n",
           HAL_GetTick(),
           latest_results.void_detected ? "detected" : "clear",
           latest_results.void_size_mm,
           latest_results.confidence_percent,
           void_get_algorithm_string(latest_results.algorithm_used));

    uart_tx_channel_undo();

    // Mark as sent and update statistics
    latest_results.new_result_available = false;
    void_state.automatic_stream_count++;

    debug_send("VOID: Event stream #%lu sent - %s (conf: %d%%)",
               void_state.automatic_stream_count,
               latest_results.void_detected ? "VOID DETECTED" : "clear",
               latest_results.confidence_percent);
}

// Add streaming control functions
void void_set_auto_streaming(bool enabled)
{
    void_state.auto_streaming_enabled = enabled;
    debug_send("VOID: Auto streaming %s", enabled ? "enabled" : "disabled");
}

bool void_is_auto_streaming_enabled(void)
{
    return void_state.auto_streaming_enabled;
}

// Add event statistics function
void void_get_event_stats(uint32_t *processing_count, uint32_t *stream_count)
{
    if (processing_count)
    {
        *processing_count = void_state.immediate_processing_count;
    }
    if (stream_count)
    {
        *stream_count = void_state.automatic_stream_count;
    }
}

/*------------------------------------------------------------------------------
 * Algorithm-Specific Functions
 *----------------------------------------------------------------------------*/

const char *void_get_algorithm_string(void_algorithm_t algorithm)
{
    switch (algorithm)
    {
    case VOID_ALGORITHM_BYPASS:
        return "bypass";
    case VOID_ALGORITHM_SIMPLE:
        return "simple";
    case VOID_ALGORITHM_CIRCLEFIT:
        return "circlefit";
    default:
        return "unknown";
    }
}

bool void_set_baseline(uint16_t baseline_mm)
{
    return void_set_baseline_diameter(baseline_mm);
}

bool void_set_range(uint16_t min_mm, uint16_t max_mm)
{
    if (min_mm >= max_mm || min_mm < 50 || max_mm > 5000)
    {
        return false;
    }

    config.min_distance_mm = min_mm;
    config.max_distance_mm = max_mm;
    debug_send("VOID: Range set to %d-%dmm", min_mm, max_mm);
    return true;
}

void void_clear_statistics(void)
{
    void_state.total_detections   = 0;
    void_state.algorithm_switches = 0;
    void_state.init_time_ms       = HAL_GetTick();
    debug_send("VOID: Statistics cleared");
}

// Add the comprehensive system test function
void void_run_system_test(uint32_t test_interval_ms)
{
    uint32_t now = HAL_GetTick();

    // Run at specified interval - always run, no enable/disable
    if ((now - void_state.last_test_time) >= test_interval_ms)
    {
        void_state.last_test_time = now;
        void_state.test_counter++;

        debug_send("=== VOID SYSTEM TEST #%lu ===", void_state.test_counter);

        // Startup phase: Initialize the complete pipeline
        if (!void_state.test_sensors_started)
        {
            void_state.startup_phase++;
            debug_send("STARTUP PHASE %lu: Initializing void detection pipeline", void_state.startup_phase);

            switch (void_state.startup_phase)
            {
            case 1:
                debug_send("Step 1: Starting radar sensors");
                if (radar_start_sensors())
                {
                    debug_send("  Radar sensors started successfully");
                }
                else
                {
                    debug_send("  ERROR: Failed to start radar sensors");
                    void_state.startup_phase = 0; // Retry next cycle
                    return;
                }
                break;

            case 2:
                debug_send("STARTUP PHASE 2: Initializing void detection pipeline");
                debug_send("Step 2: Sensors already in measurement mode from initialization");
                debug_send("  Measurement mode already active");
                void_state.startup_phase = 3;
                debug_send("=== STARTUP PHASE 2 COMPLETE ===");
                break;

            case 3:
                debug_send("Step 3: Enabling event-driven processing");
                void_set_auto_streaming(true);
                debug_send("  Auto streaming enabled");
                debug_send("  Waiting for system stabilization...");
                break;

            case 4:
                debug_send("STARTUP COMPLETE: Void detection pipeline active");
                void_state.test_sensors_started = true;
                break;
            }

            if (!void_state.test_sensors_started)
            {
                debug_send("=== STARTUP PHASE %lu COMPLETE ===", void_state.startup_phase);
                return;
            }
        }

        // Main test phase: Monitor the complete pipeline
        debug_send("PIPELINE TEST: Monitoring data flow");

        // 1. CAN Layer Status
        debug_send("--- CAN LAYER STATUS ---");
        debug_send("Global flags: radar_ready=%s, new_object=%s", can_has_new_radar_data() ? "TRUE" : "false", can_new_object_data ? "TRUE" : "false");

        uint32_t total_can_frames = 0;
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            can_sensor_t *sensor      = can_get_sensor(i);
            uint32_t      frame_count = can_get_sensor_frame_count(i);
            total_can_frames += frame_count;

            debug_send("S%d: online=%s, ready=%s, frames=%lu, points=%d, msgs=%lu",
                       i,
                       (sensor && sensor->online) ? "YES" : "NO",
                       can_sensor_has_new_data(i) ? "TRUE" : "false",
                       frame_count,
                       sensor ? sensor->current_point_count : 0,
                       sensor ? sensor->msg_count : 0);

            // Show latest detection points for active sensors
            if (sensor && sensor->online && sensor->current_point_count > 0)
            {
                for (uint8_t p = 0; p < sensor->current_point_count && p < 3; p++)
                {
                    debug_send("  Point %d: %.3fm (%.1f SNR)", p, sensor->detection_points[p][0], sensor->detection_points[p][1]);
                }
            }
        }

        // 2. Radar Processing Status
        debug_send("--- RADAR PROCESSING STATUS ---");
        uint32_t radar_events, last_radar_time;
        radar_get_event_stats(&radar_events, &last_radar_time);

        radar_distance_t measurements;
        bool             radar_data_available = radar_get_latest_measurements(&measurements);

        debug_send("Processing events: %lu, last_event: %lums ago", radar_events, now - last_radar_time);

        debug_send("Data available: %s, system_healthy: %s, valid_sensors: %d/%d",
                   radar_data_available ? "YES" : "NO",
                   measurements.system_healthy ? "YES" : "NO",
                   measurements.valid_sensor_count,
                   MAX_RADAR_SENSORS);

        // Show processed radar measurements
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            debug_send("S%d processed: valid=%s, distance=%dmm, confidence=%.2f, quality=%d%%",
                       i,
                       measurements.data_valid[i] ? "YES" : "NO",
                       measurements.distance_mm[i],
                       measurements.confidence[i],
                       measurements.quality_score[i]);
        }

        // 3. Void Detection Status
        debug_send("--- VOID DETECTION STATUS ---");
        uint32_t void_processing, void_streams;
        void_get_event_stats(&void_processing, &void_streams);

        void_data_t void_result;
        bool        void_data_available = void_get_latest_results(&void_result);

        debug_send("Processing events: %lu, streams sent: %lu", void_processing, void_streams);

        debug_send("Auto streaming: %s, system ready: %s", void_is_auto_streaming_enabled() ? "ENABLED" : "disabled", void_is_system_ready() ? "YES" : "NO");

        if (void_data_available)
        {
            // Fix the enum casting issues by using the enum name explicitly
            debug_send("Detection: %s, size: %dmm, confidence: %d%%, algorithm: %s",
                       void_result.void_detected ? "VOID DETECTED" : "clear",
                       void_result.void_size_mm,
                       void_result.confidence_percent,
                       void_get_algorithm_string((void_algorithm_t)void_result.algorithm_used));

            debug_send("Detection time: %lums ago", now - void_result.detection_time_ms);
        }
        else
        {
            debug_send("No void detection data available");
        }

        // 4. Summary for easy parsing
        debug_send("SUMMARY: sensors=%d/%d, valid_data=%d, void=%s, conf=%d%%, health=%s",
                   can_get_online_count(),
                   MAX_RADAR_SENSORS,
                   measurements.valid_sensor_count,
                   void_data_available && void_result.void_detected ? "DETECTED" : "clear",
                   void_data_available ? void_result.confidence_percent : 0,
                   "OK");

        debug_send("=== END VOID SYSTEM TEST #%lu ===", void_state.test_counter);
    }
}

void void_status_brief(void)
{
    debug_send("[V]S:%d D:%d C:%d%%", latest_results.void_detected ? 1 : 0, latest_results.void_size_mm, latest_results.confidence_percent);
}

/*------------------------------------------------------------------------------
 * Simple System Control Functions (POC)
 *----------------------------------------------------------------------------*/

bool void_system_start(void)
{
    debug_send("VOID: Starting system (POC) - called by @st command");

    // Check if system is initialized
    if (!void_state.system_initialized)
    {
        debug_send("VOID: ERROR - System not initialized");
        return false;
    }

    // Check if already running
    if (void_system_running)
    {
        debug_send("VOID: WARNING - System already running");
        return true;
    }

    // Simply start the radar sensors via CAN command
    debug_send("VOID: Sending START command to radar sensors");
    if (!radar_start_sensors())
    {
        debug_send("VOID: ERROR - Failed to send start command to radar sensors");
        return false;
    }

    // Enable auto streaming for operational data
    void_set_auto_streaming(true);

    // Set running flag
    void_system_running = true;

    debug_send("VOID: System started - radar sensors turned ON");
    return true;
}

bool void_system_stop(void)
{
    debug_send("VOID: Stopping system (POC) - called by @fn command");

    // Check if already stopped
    if (!void_system_running)
    {
        debug_send("VOID: WARNING - System already stopped");
        return true;
    }

    // Simply stop the radar sensors via CAN command
    debug_send("VOID: Sending STOP command to radar sensors");
    if (!radar_stop_sensors())
    {
        debug_send("VOID: WARNING - Failed to send stop command to radar sensors");
    }

    // Disable auto streaming
    void_set_auto_streaming(false);

    // Clear running flag
    void_system_running = false;

    debug_send("VOID: System stopped - radar sensors turned OFF");
    return true;
}

bool void_system_is_running(void)
{
    return void_system_running;
}
