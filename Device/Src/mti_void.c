#include "mti_void.h"
#include "mti_radar.h"
#include "vmt_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

// Static system state
static void_system_state_t prv_void_system = { 0 };

// Private function prototypes
static void     prv_load_default_config(void);
static bool     prv_validate_sensor_data(uint16_t distance_mm, uint8_t sensor_idx);
static void     prv_update_measurement_data(void);
static void     prv_add_to_history(const void_status_t *status);
static uint16_t prv_apply_median_filter(uint16_t distances[], uint8_t count);

// Circle fitting private functions
static bool
prv_circle_fit_3_points(uint16_t distances_mm[MAX_RADAR_SENSORS], uint16_t angles_deg[MAX_RADAR_SENSORS], bool data_valid[MAX_RADAR_SENSORS], circle_fit_data_t *result);
static bool    prv_circle_fit_void_detection(void_status_t *result);
static bool    prv_simple_threshold_detection(void_status_t *result);
static uint8_t prv_calculate_circle_confidence(const circle_fit_data_t *circle_data, uint16_t expected_radius);

void void_system_init(void)
{
    // Initialize system state
    memset(&prv_void_system, 0, sizeof(prv_void_system));

    // Load default configuration
    prv_load_default_config();

    // Initialize status
    strcpy(prv_void_system.current_status.status_text, "System Ready");
    prv_void_system.current_status.baseline_diameter_mm = prv_void_system.config.baseline_diameter_mm;
    prv_void_system.current_status.algorithm_used       = prv_void_system.config.active_algorithm;

    prv_void_system.system_initialized   = true;
    prv_void_system.last_process_time_ms = HAL_GetTick();

    // Simple POC initialization feedback
    debug_send("Void detection system initialized (algorithm: %s, baseline: %dmm, threshold: %dmm)",
               void_get_algorithm_string(prv_void_system.config.active_algorithm),
               prv_void_system.config.baseline_diameter_mm,
               prv_void_system.config.detection_threshold_mm);
}

void void_system_process(void)
{
    if (!prv_void_system.system_initialized)
    {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    // Update timestamp for this processing cycle
    prv_void_system.last_process_time_ms = current_time;

    // Update measurement data from radar sensors
    prv_update_measurement_data();

    // Check if we have valid data from all sensors
    uint8_t valid_sensor_count = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (prv_void_system.latest_measurement.data_valid[i])
        {
            valid_sensor_count++;
        }
    }

    // Only proceed if we have data from at least 2 sensors
    if (valid_sensor_count < 2)
    {
        debug_send("Insufficient sensor data for void analysis (%d/3 valid)", valid_sensor_count);
        return;
    }

    // Run detection algorithm
    void_status_t new_status       = { 0 };
    bool          detection_result = false;

    switch (prv_void_system.config.active_algorithm)
    {
    case VOID_ALG_SIMPLE:
        detection_result = prv_simple_threshold_detection(&new_status);
        break;
    case VOID_ALG_CIRCLEFIT:
        detection_result = prv_circle_fit_void_detection(&new_status);
        break;
    }

    // Update current status
    if (detection_result)
    {
        prv_void_system.current_status                     = new_status;
        prv_void_system.current_status.measurement_time_ms = current_time;

        // Add to history
        prv_add_to_history(&new_status);

        debug_send("Void detected (immediate): %s", new_status.status_text);
    }

    // AUTOMATIC: Send data when processing complete
    void_send_data();

    // OPTIONAL: Send immediate event if void detected
    if (prv_void_system.current_status.void_detected)
    {
        void_send_event();
    }
}

bool void_analyze_sensor_data(uint8_t sensor_idx, uint16_t distance_mm, uint16_t angle_deg, void_status_t *result)
{
    if (!result || sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }

    // Initialize result
    memset(result, 0, sizeof(void_status_t));
    result->baseline_diameter_mm = prv_void_system.config.baseline_diameter_mm;
    result->measurement_time_ms  = HAL_GetTick();   // FIXED: was detection_time_ms
    result->algorithm_used       = VOID_ALG_SIMPLE; // Single sensor analysis always uses simple

    // Validate sensor data
    if (!prv_validate_sensor_data(distance_mm, sensor_idx))
    {
        strcpy(result->status_text, "Invalid sensor data");
        return false;
    }

    // Simple threshold-based void detection
    uint16_t expected_distance = prv_void_system.config.baseline_diameter_mm / 2; // Radius
    uint16_t threshold         = prv_void_system.config.detection_threshold_mm;

    if (distance_mm > (expected_distance + threshold))
    {
        // Void detected
        result->void_detected      = true;
        result->void_size_mm       = (distance_mm - expected_distance) * 2; // Convert to diameter
        result->confidence_percent = void_calculate_confidence(distance_mm, expected_distance, threshold);

        // Characterize the detection
        void_characterise_detection(distance_mm, expected_distance, result);

        sprintf(result->status_text, "Void S%d: %dmm (simple)", sensor_idx, result->void_size_mm);
        return true;
    }

    strcpy(result->status_text, "No void detected (simple)");
    return false;
}

// Circle fitting implementation
static bool
prv_circle_fit_3_points(uint16_t distances_mm[MAX_RADAR_SENSORS], uint16_t angles_deg[MAX_RADAR_SENSORS], bool data_valid[MAX_RADAR_SENSORS], circle_fit_data_t *result)
{
    if (!result)
    {
        return false;
    }

    // Initialize result
    memset(result, 0, sizeof(circle_fit_data_t));

    // Convert sensor data to Cartesian coordinates
    int16_t x_coords[MAX_RADAR_SENSORS];
    int16_t y_coords[MAX_RADAR_SENSORS];
    uint8_t valid_points = 0;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (data_valid[i] && distances_mm[i] > 0)
        {
            // Convert polar to Cartesian (sensor at origin, measuring to wall)
            float angle_rad        = (float)angles_deg[i] * 3.14159f / 180.0f;
            x_coords[valid_points] = (int16_t)(distances_mm[i] * cosf(angle_rad));
            y_coords[valid_points] = (int16_t)(distances_mm[i] * sinf(angle_rad));
            valid_points++;
        }
    }

    if (valid_points < prv_void_system.config.min_sensors_for_circle)
    {
        debug_send("Circle fit: insufficient points (%d < %d)", valid_points, prv_void_system.config.min_sensors_for_circle);
        return false;
    }

    // For 2 points, use simple midpoint calculation
    if (valid_points == 2)
    {
        int16_t mid_x = (x_coords[0] + x_coords[1]) / 2;
        int16_t mid_y = (y_coords[0] + y_coords[1]) / 2;

        result->center_x_mm = mid_x;
        result->center_y_mm = mid_y;

        // Calculate radius as distance from center to first point
        int32_t dx        = x_coords[0] - mid_x;
        int32_t dy        = y_coords[0] - mid_y;
        result->radius_mm = (uint16_t)sqrtf((float)(dx * dx + dy * dy));

        // Calculate fitting error
        dx                   = x_coords[1] - mid_x;
        dy                   = y_coords[1] - mid_y;
        uint16_t radius2     = (uint16_t)sqrtf((float)(dx * dx + dy * dy));
        result->fit_error_mm = (result->radius_mm > radius2) ? (result->radius_mm - radius2) : (radius2 - result->radius_mm);

        result->sensors_used = 2;
    }
    else // 3 or more points - use determinant method for 3-point circle
    {
        int32_t x1 = x_coords[0], y1 = y_coords[0];
        int32_t x2 = x_coords[1], y2 = y_coords[1];
        int32_t x3 = x_coords[2], y3 = y_coords[2];

        // Calculate intermediate values
        int32_t d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

        if (d == 0)
        {
            debug_send("Circle fit: points are collinear");
            return false;
        }

        // Calculate circle center
        int32_t ux = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2));

        int32_t uy = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1));

        // Center coordinates
        result->center_x_mm = (int16_t)(ux / d);
        result->center_y_mm = (int16_t)(uy / d);

        // Calculate radius from center to first point
        int32_t dx        = x1 - result->center_x_mm;
        int32_t dy        = y1 - result->center_y_mm;
        result->radius_mm = (uint16_t)sqrtf((float)(dx * dx + dy * dy));

        // Calculate fitting error (RMS distance from circle)
        uint32_t error_sum = 0;
        for (uint8_t i = 0; i < valid_points; i++)
        {
            dx                    = x_coords[i] - result->center_x_mm;
            dy                    = y_coords[i] - result->center_y_mm;
            uint16_t point_radius = (uint16_t)sqrtf((float)(dx * dx + dy * dy));
            uint16_t error        = (point_radius > result->radius_mm) ? (point_radius - result->radius_mm) : (result->radius_mm - point_radius);
            error_sum += error * error;
        }

        result->fit_error_mm = (uint16_t)sqrtf((float)error_sum / valid_points);
        result->sensors_used = valid_points;
    }

    // Validate fit quality
    if (result->fit_error_mm > prv_void_system.config.circle_fit_tolerance_mm)
    {
        debug_send("Circle fit: poor quality (error %dmm > %dmm)", result->fit_error_mm, prv_void_system.config.circle_fit_tolerance_mm);
        return false;
    }

    result->fit_successful = true;
    debug_send("Circle fit: center(%d,%d) radius=%dmm error=%dmm", result->center_x_mm, result->center_y_mm, result->radius_mm, result->fit_error_mm);

    return true;
}

static bool prv_circle_fit_void_detection(void_status_t *result)
{
    uint16_t distances_mm[MAX_RADAR_SENSORS];
    uint16_t angles_deg[MAX_RADAR_SENSORS];
    bool     data_valid[MAX_RADAR_SENSORS];
    uint8_t  valid_sensors = 0;

    // Collect measurement data
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        radar_measurement_t *measurement = radar_get_measurement(i);
        if (measurement && measurement->data_valid)
        {
            distances_mm[i] = measurement->distance_mm;
            angles_deg[i]   = measurement->angle_deg;
            data_valid[i]   = true;
            valid_sensors++;
        }
        else
        {
            data_valid[i] = false;
        }
    }

    // Set partial data flag
    result->partial_data = (valid_sensors < MAX_RADAR_SENSORS);

    // Circle fitting requires at least 3 points for accurate results
    if (valid_sensors < 3)
    {
        if (prv_void_system.config.auto_fallback_enabled)
        {
            debug_send("Circle fit: insufficient sensors (%d < 3), falling back to simple", valid_sensors);
            return prv_simple_threshold_detection(result);
        }
        else
        {
            debug_send("Circle fit: insufficient sensors (%d < 3), analysis failed", valid_sensors);
            result->void_detected      = false;
            result->confidence_percent = 0;
            strcpy(result->status_text, "Insufficient sensor data for circle fit");
            return false;
        }
    }

    // Proceed with circle fitting using 3+ sensors
    circle_fit_data_t circle_data = { 0 };
    bool              fit_success = prv_circle_fit_3_points(distances_mm, angles_deg, data_valid, &circle_data);

    if (!fit_success)
    {
        if (prv_void_system.config.auto_fallback_enabled)
        {
            debug_send("Circle fit failed, falling back to simple threshold");
            return prv_simple_threshold_detection(result);
        }
        else
        {
            result->void_detected      = false;
            result->confidence_percent = 0;
            strcpy(result->status_text, "Circle fitting failed");
            return false;
        }
    }

    // Process successful circle fit results
    uint16_t expected_radius = prv_void_system.config.baseline_diameter_mm / 2;

    if (circle_data.radius_mm > (expected_radius + prv_void_system.config.detection_threshold_mm))
    {
        result->void_detected      = true;
        result->void_size_mm       = circle_data.radius_mm * 2;
        result->confidence_percent = prv_calculate_circle_confidence(&circle_data, expected_radius);

        // Determine which sector has the void (largest deviation)
        uint16_t max_deviation   = 0;
        uint8_t  dominant_sensor = 0; // ADD THIS LINE
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            if (data_valid[i])
            {
                uint16_t deviation = (distances_mm[i] > expected_radius) ? (distances_mm[i] - expected_radius) : 0;
                if (deviation > max_deviation)
                {
                    max_deviation   = deviation;
                    dominant_sensor = i; // CHANGED: was result->void_sector = i;
                }
            }
        }

        void_characterise_detection(circle_data.radius_mm, expected_radius, result);
        // FIXED: Use dominant_sensor instead of result->void_sector
        snprintf(result->status_text, sizeof(result->status_text), "Void S%d: %dmm (circle)", dominant_sensor, result->void_size_mm);
        return true;
    }

    snprintf(result->status_text, sizeof(result->status_text), "No void detected (circle)");
    return false;
}

static bool prv_simple_threshold_detection(void_status_t *result)
{
    uint8_t  valid_sensors   = 0;
    uint16_t total_deviation = 0;
    uint16_t max_deviation   = 0;
    uint8_t  dominant_sensor = 0;

    // Count valid sensors and calculate deviations
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (prv_void_system.latest_measurement.data_valid[i])
        {
            valid_sensors++;
            uint16_t expected  = prv_void_system.config.baseline_diameter_mm / 2;
            uint16_t distance  = prv_void_system.latest_measurement.distance_mm[i];
            uint16_t deviation = (distance > expected) ? (distance - expected) : (expected - distance);

            total_deviation += deviation;
            if (deviation > max_deviation)
            {
                max_deviation   = deviation;
                dominant_sensor = i;
            }
        }
    }

    // Set partial data flag
    result->partial_data = (valid_sensors < MAX_RADAR_SENSORS);

    if (valid_sensors < 2)
    {
        result->void_detected      = false;
        result->confidence_percent = 0;
        strcpy(result->status_text, "Insufficient sensor data");
        return false;
    }

    result->void_size_mm         = max_deviation; // FIXED
    result->algorithm_used       = VOID_ALG_SIMPLE;
    result->baseline_diameter_mm = prv_void_system.config.baseline_diameter_mm;

    // Check if void detected
    bool void_detected = (max_deviation > prv_void_system.config.detection_threshold_mm); // FIXED

    if (void_detected)
    {
        result->void_detected      = true;
        result->confidence_percent = void_calculate_confidence(max_deviation,
                                                               prv_void_system.config.baseline_diameter_mm / 2,
                                                               prv_void_system.config.detection_threshold_mm); // FIXED
        void_characterise_detection(max_deviation, prv_void_system.config.baseline_diameter_mm / 2, result);

        // Remove void_sector assignment as it doesn't exist in struct
        // result->void_sector = dominant_sensor; // REMOVED

        snprintf(result->status_text, sizeof(result->status_text), "Void detected: %dmm deviation (sensor %d)", max_deviation, dominant_sensor);
    }
    else
    {
        result->void_detected      = false;
        result->confidence_percent = 0;
        result->severity           = VOID_SEVERITY_NONE; // FIXED
        strcpy(result->status_text, "No void detected");
    }

    return true;
}

static uint8_t prv_calculate_circle_confidence(const circle_fit_data_t *circle_data, uint16_t expected_radius)
{
    if (!circle_data->fit_successful)
    {
        return 0;
    }

    // Base confidence on fit error and radius deviation
    uint16_t radius_deviation = (circle_data->radius_mm > expected_radius) ? (circle_data->radius_mm - expected_radius) : (expected_radius - circle_data->radius_mm);

    // Lower confidence for high fit error or large center offset
    uint8_t confidence = 100;

    if (circle_data->fit_error_mm > 10)
    {
        confidence -= (circle_data->fit_error_mm - 10) * 2; // Reduce by 2% per mm of error above 10mm
    }

    uint16_t center_offset = (uint16_t)sqrt(circle_data->center_x_mm * circle_data->center_x_mm + circle_data->center_y_mm * circle_data->center_y_mm);
    if (center_offset > 50)
    {
        confidence -= (center_offset - 50) / 5; // Reduce by 1% per 5mm of center offset above 50mm
    }

    return (confidence < VOID_MIN_CONFIDENCE) ? VOID_MIN_CONFIDENCE : confidence;
}

// Original functions (mostly unchanged)
uint8_t void_calculate_confidence(uint16_t distance_mm, uint16_t expected_mm, uint16_t threshold_mm)
{
    if (distance_mm <= expected_mm)
    {
        return 0; // No void
    }

    uint16_t excess = distance_mm - expected_mm;

    if (excess <= threshold_mm)
    {
        return 0; // Below threshold
    }

    // Linear confidence calculation
    // Base confidence of 70% at threshold, up to 100% at 2x threshold
    uint16_t above_threshold = excess - threshold_mm;
    uint8_t  confidence      = 70 + (above_threshold * 30) / threshold_mm;

    return (confidence > 100) ? 100 : confidence;
}

void void_characterise_detection(uint16_t distance_mm, uint16_t expected_mm, void_status_t *result)
{
    uint16_t deviation = (distance_mm > expected_mm) ? (distance_mm - expected_mm) : (expected_mm - distance_mm);

    if (deviation < VOID_MINOR_THRESHOLD_MM)
    {
        result->severity = VOID_SEVERITY_MINOR; // FIXED
    }
    else if (deviation < VOID_MAJOR_THRESHOLD_MM)
    {
        result->severity = VOID_SEVERITY_MAJOR; // FIXED
    }
    else
    {
        result->severity = VOID_SEVERITY_CRITICAL; // FIXED
    }
}

// Configuration functions
void void_set_threshold(uint16_t threshold_mm)
{
    prv_void_system.config.detection_threshold_mm = threshold_mm;
    debug_send("Void threshold set to %d mm", threshold_mm);
}

void void_set_baseline(uint16_t baseline_mm)
{
    prv_void_system.config.baseline_diameter_mm = baseline_mm;
    debug_send("Void baseline set to %d mm", baseline_mm);
}

void void_set_confidence_threshold(uint8_t confidence_percent)
{
    if (confidence_percent <= 100)
    {
        prv_void_system.config.confidence_threshold = confidence_percent;
        debug_send("Void confidence threshold set to %d%%", confidence_percent);
    }
}

void void_set_range(uint16_t min_mm, uint16_t max_mm)
{
    if (min_mm < max_mm)
    {
        prv_void_system.config.range_min_mm = min_mm;
        prv_void_system.config.range_max_mm = max_mm;
        debug_send("Void range set to %d-%d mm", min_mm, max_mm);
    }
}

void void_set_median_filter(bool enabled)
{
    prv_void_system.config.median_filter_enabled = enabled;
    debug_send("Void median filter %s", enabled ? "enabled" : "disabled");
}

// Circle fitting configuration functions
void void_set_detection_algorithm(void_algorithm_t algorithm)
{
    prv_void_system.config.active_algorithm = algorithm;
    debug_send("Void algorithm set to %s", void_get_algorithm_string(algorithm));
}

void void_set_circle_tolerance(uint16_t tolerance_mm)
{
    prv_void_system.config.circle_fit_tolerance_mm = tolerance_mm;
    debug_send("Circle fit tolerance set to %d mm", tolerance_mm);
}

void void_set_min_sensors_circle(uint8_t min_sensors)
{
    if (min_sensors >= CIRCLE_FIT_MIN_SENSORS && min_sensors <= MAX_RADAR_SENSORS)
    {
        prv_void_system.config.min_sensors_for_circle = min_sensors;
        debug_send("Min sensors for circle fit set to %d", min_sensors);
    }
}

void void_set_auto_fallback(bool enabled)
{
    prv_void_system.config.auto_fallback_enabled = enabled;
    debug_send("Auto fallback %s", enabled ? "enabled" : "disabled");
}

void_algorithm_t void_get_current_algorithm(void)
{
    return prv_void_system.config.active_algorithm;
}

// Status and data retrieval functions
void void_get_latest_results(void_status_t *result)
{
    if (result && prv_void_system.system_initialized)
    {
        *result = prv_void_system.current_status;
    }
}

void void_get_current_config(void_config_t *config)
{
    if (config && prv_void_system.system_initialized)
    {
        *config = prv_void_system.config;
    }
}

bool void_get_measurement_data(void_measurement_t *measurement)
{
    if (!measurement || !prv_void_system.system_initialized)
    {
        return false;
    }

    *measurement = prv_void_system.latest_measurement;
    return true;
}

uint8_t void_get_history_count(void)
{
    return prv_void_system.history_count;
}

bool void_get_history_entry(uint8_t index, void_status_t *entry)
{
    if (!entry || index >= prv_void_system.history_count)
    {
        return false;
    }

    *entry = prv_void_system.history[index];
    return true;
}

void void_clear_history(void)
{
    prv_void_system.history_count = 0;
    memset(prv_void_system.history, 0, sizeof(prv_void_system.history));
}

bool void_is_system_ready(void)
{
    return prv_void_system.system_initialized;
}

const char *void_get_severity_string(void_severity_t severity)
{
    switch (severity)
    {
    case VOID_SEVERITY_NONE:
        return "None";
    case VOID_SEVERITY_MINOR:
        return "Minor";
    case VOID_SEVERITY_MAJOR:
        return "Major";
    case VOID_SEVERITY_CRITICAL:
        return "Critical";
    default:
        return "Unknown";
    }
}

const char *void_get_algorithm_string(void_algorithm_t algorithm)
{
    switch (algorithm)
    {
    case VOID_ALG_SIMPLE:
        return "Simple Threshold";
    case VOID_ALG_CIRCLEFIT:
        return "Circle Fitting";
    default:
        return "Unknown";
    }
}

// Private helper functions
static void prv_load_default_config(void)
{
    prv_void_system.config.baseline_diameter_mm   = VOID_DEFAULT_BASELINE_MM;
    prv_void_system.config.detection_threshold_mm = VOID_DEFAULT_THRESHOLD_MM; // FIXED
    prv_void_system.config.confidence_threshold   = VOID_MIN_CONFIDENCE;
    prv_void_system.config.median_filter_enabled  = true;
    prv_void_system.config.range_min_mm           = 10;
    prv_void_system.config.range_max_mm           = 500;

    // Circle fitting defaults
    prv_void_system.config.active_algorithm        = VOID_ALG_SIMPLE;
    prv_void_system.config.circle_fit_tolerance_mm = CIRCLE_FIT_DEFAULT_TOLERANCE_MM;
    prv_void_system.config.min_sensors_for_circle  = CIRCLE_FIT_MIN_SENSORS;
    prv_void_system.config.auto_fallback_enabled   = true;

    prv_void_system.current_status.measurement_time_ms = HAL_GetTick(); // FIXED
}

static bool prv_validate_sensor_data(uint16_t distance_mm, uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }

    // Check range limits
    if (distance_mm < prv_void_system.config.range_min_mm || distance_mm > prv_void_system.config.range_max_mm)
    {
        return false;
    }

    return true;
}

static void prv_update_measurement_data(void)
{
    prv_void_system.latest_measurement.measurement_time_ms = HAL_GetTick();

    // Get data from each radar sensor
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        radar_measurement_t *measurement = radar_get_measurement(i);
        if (measurement && measurement->data_valid)
        {
            prv_void_system.latest_measurement.distance_mm[i] = measurement->distance_mm;
            prv_void_system.latest_measurement.angle_deg[i]   = measurement->angle_deg;
            prv_void_system.latest_measurement.data_valid[i]  = true;
        }
        else
        {
            prv_void_system.latest_measurement.data_valid[i] = false;
        }
    }
}

static void prv_add_to_history(const void_status_t *status)
{
    if (prv_void_system.history_count < VOID_HISTORY_SIZE)
    {
        prv_void_system.history[prv_void_system.history_count] = *status;
        prv_void_system.history_count++;
    }
    else
    {
        // Shift history array and add new entry
        for (uint8_t i = 0; i < (VOID_HISTORY_SIZE - 1); i++)
        {
            prv_void_system.history[i] = prv_void_system.history[i + 1];
        }
        prv_void_system.history[VOID_HISTORY_SIZE - 1] = *status;
    }
}

static uint16_t prv_apply_median_filter(uint16_t distances[], uint8_t count)
{
    if (count == 0)
    {
        return 0;
    }
    if (count == 1)
    {
        return distances[0];
    }
    // Simple bubble sort for small arrays
    uint16_t sorted[MAX_RADAR_SENSORS];
    for (uint8_t i = 0; i < count; i++)
    {
        sorted[i] = distances[i];
    }

    for (uint8_t i = 0; i < count - 1; i++)
    {
        for (uint8_t j = 0; j < count - i - 1; j++)
        {
            if (sorted[j] > sorted[j + 1])
            {
                uint16_t temp = sorted[j];
                sorted[j]     = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    return sorted[count / 2]; // Return median
}

// COMPLETE: Automatic data transmission function
void void_send_data(void)
{
    void_measurement_t *latest = &prv_void_system.latest_measurement;

    // Calculate flags (0x0F = all sensors valid, 0x10 = void detected)
    uint8_t flags = 0x00;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (latest->data_valid[i])
        {
            flags |= (0x01 << i); // Set sensor valid bit
        }
    }
    if (prv_void_system.current_status.void_detected)
    {
        flags |= 0x10; // Set void detected bit
    }

    // Calculate void sizes (0 if no void)
    uint16_t void_sizes[MAX_RADAR_SENSORS] = { 0 };
    if (prv_void_system.current_status.void_detected)
    {
        // Set void size for detected void
        void_sizes[0] = prv_void_system.current_status.void_size_mm;
    }

    // AUTOMATIC: Send data to uphole (follows water detection pattern)
    uart_tx_channel_set(UART_UPHOLE);
    printf("&vd,0x%02X,%d,%d,%d,%d,%d,%d,%d\r\n",
           flags,
           latest->distance_mm[0],
           latest->distance_mm[1],
           latest->distance_mm[2],
           void_sizes[0],
           void_sizes[1],
           void_sizes[2],
           prv_void_system.current_status.confidence_percent);
    uart_tx_channel_undo();
}

// COMPLETE: Optional asynchronous event transmission
void void_send_event(void)
{
    if (prv_void_system.current_status.void_detected)
    {
        uart_tx_channel_set(UART_UPHOLE);
        printf("!vd,flag,%d,%d,%d\r\n",
               0, // sector (simplified for POC)
               prv_void_system.current_status.void_size_mm,
               prv_void_system.current_status.confidence_percent);
        uart_tx_channel_undo();
    }
}

// FIX: Complete the prv_update_measurement_data function
static void prv_update_measurement_data(void)
{
    prv_void_system.latest_measurement.measurement_time_ms = HAL_GetTick();

    // Get data from each radar sensor
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        radar_measurement_t *measurement = radar_get_measurement(i);
        if (measurement && measurement->data_valid)
        {
            prv_void_system.latest_measurement.distance_mm[i] = measurement->distance_mm;
            prv_void_system.latest_measurement.angle_deg[i]   = measurement->angle_deg;
            prv_void_system.latest_measurement.data_valid[i]  = true;
        }
        else
        {
            prv_void_system.latest_measurement.data_valid[i] = false;
        }
    }
}

// FIX: Complete missing implementation sections marked with {...}
// All the {...} placeholders need actual implementation
