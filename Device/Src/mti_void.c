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

    debug_send("Void detection system initialized (algorithm: %s)", void_get_algorithm_string(prv_void_system.config.active_algorithm));
}

void void_system_process(void)
{
    if (!prv_void_system.system_initialized)
    {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    // Check if it's time to process (every 100ms)
    if ((current_time - prv_void_system.last_process_time_ms) < VOID_PROCESS_INTERVAL_MS)
    {
        return;
    }

    prv_void_system.last_process_time_ms = current_time;

    // Update measurement data from radar system
    prv_update_measurement_data();

    // Reset detection status for this cycle
    void_status_t new_status        = { 0 };
    new_status.baseline_diameter_mm = prv_void_system.config.baseline_diameter_mm;
    new_status.detection_time_ms    = current_time;
    new_status.algorithm_used       = prv_void_system.config.active_algorithm;
    strcpy(new_status.status_text, "No void detected");

    // Choose detection algorithm
    bool void_found = false;

    if (prv_void_system.config.active_algorithm == VOID_ALG_CIRCLEFIT)
    {
        // Use circle fitting for multi-sensor analysis
        void_found = prv_circle_fit_void_detection(&new_status);
    }
    else
    {
        // Use simple algorithm - analyze each sensor individually
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            if (prv_void_system.latest_measurement.data_valid[i])
            {
                void_status_t sensor_result = { 0 };

                if (void_analyze_sensor_data(i, prv_void_system.latest_measurement.distance_mm[i], prv_void_system.latest_measurement.angle_deg[i], &sensor_result))
                {
                    if (sensor_result.confidence_percent >= prv_void_system.config.confidence_threshold)
                    {
                        new_status             = sensor_result;
                        new_status.void_sector = i;
                        void_found             = true;
                        break;
                    }
                }
            }
        }
    }

    // Update system status
    prv_void_system.current_status = new_status;

    // Add to history if void state changed
    static bool last_void_state = false;
    if (new_status.void_detected != last_void_state)
    {
        prv_add_to_history(&new_status);
        last_void_state = new_status.void_detected;

        // Send event notification
        if (new_status.void_detected)
        {
            debug_send("!vd,flag,%d,%d,%d,%s",
                       new_status.void_sector,
                       new_status.void_diameter_mm,
                       new_status.confidence_percent,
                       void_get_algorithm_string(new_status.algorithm_used));
        }
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
    result->detection_time_ms    = HAL_GetTick();
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
        result->void_diameter_mm   = (distance_mm - expected_distance) * 2; // Convert to diameter
        result->confidence_percent = void_calculate_confidence(distance_mm, expected_distance, threshold);

        // Characterize the detection
        void_characterise_detection(distance_mm, expected_distance, result);

        sprintf(result->status_text, "Void S%d: %dmm (simple)", sensor_idx, result->void_diameter_mm);
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
    if (!result)
    {
        return false;
    }

    // Get current measurement data
    uint16_t distances[MAX_RADAR_SENSORS];
    uint16_t angles[MAX_RADAR_SENSORS];
    bool     valid[MAX_RADAR_SENSORS];

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        distances[i] = prv_void_system.latest_measurement.distance_mm[i];
        angles[i]    = prv_void_system.latest_measurement.angle_deg[i];
        valid[i]     = prv_void_system.latest_measurement.data_valid[i];
    }

    // Perform circle fitting
    circle_fit_data_t circle_result;
    if (!prv_circle_fit_3_points(distances, angles, valid, &circle_result))
    {
        // Circle fitting failed - fallback to simple algorithm if enabled
        if (prv_void_system.config.auto_fallback_enabled)
        {
            debug_send("Circle fit failed, falling back to simple algorithm");
            return prv_simple_threshold_detection(result);
        }
        return false;
    }

    // Store circle fitting results
    result->circle_data    = circle_result;
    result->algorithm_used = VOID_ALG_CIRCLEFIT;

    // Calculate baseline parameters
    uint16_t expected_radius = prv_void_system.config.baseline_diameter_mm / 2;
    uint16_t fitted_diameter = circle_result.radius_mm * 2;

    // Void detection based on circle analysis
    if (fitted_diameter > (prv_void_system.config.baseline_diameter_mm + prv_void_system.config.detection_threshold_mm))
    {
        result->void_detected    = true;
        result->void_diameter_mm = fitted_diameter - prv_void_system.config.baseline_diameter_mm;

        // Calculate enhanced confidence for circle fitting
        result->confidence_percent = prv_calculate_circle_confidence(&circle_result, expected_radius);

        // Determine primary void sector (sensor with maximum deviation)
        uint16_t max_deviation  = 0;
        uint8_t  primary_sector = 0;

        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            if (valid[i])
            {
                uint16_t deviation = (distances[i] > expected_radius) ? (distances[i] - expected_radius) : 0;
                if (deviation > max_deviation)
                {
                    max_deviation  = deviation;
                    primary_sector = i;
                }
            }
        }

        result->void_sector = primary_sector;

        // Classify void severity based on circle analysis
        if (result->void_diameter_mm < 30)
        {
            result->void_severity = VOID_SEVERITY_MINOR;
        }
        else if (result->void_diameter_mm < 80)
        {
            result->void_severity = VOID_SEVERITY_MAJOR;
        }
        else
        {
            result->void_severity = VOID_SEVERITY_CRITICAL;
        }

        sprintf(result->status_text, "Void detected: %dmm dia, %d%% conf (circle fit)", result->void_diameter_mm, result->confidence_percent);

        return true;
    }

    strcpy(result->status_text, "No void detected (circle fit)");
    return false;
}

static bool prv_simple_threshold_detection(void_status_t *result)
{
    // Use existing simple algorithm implementation
    result->algorithm_used = VOID_ALG_SIMPLE;

    // Find the sensor with maximum deviation
    uint16_t expected_distance = prv_void_system.config.baseline_diameter_mm / 2;
    uint16_t threshold         = prv_void_system.config.detection_threshold_mm;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (prv_void_system.latest_measurement.data_valid[i])
        {
            uint16_t distance = prv_void_system.latest_measurement.distance_mm[i];

            if (distance > (expected_distance + threshold))
            {
                result->void_detected      = true;
                result->void_diameter_mm   = (distance - expected_distance) * 2;
                result->confidence_percent = void_calculate_confidence(distance, expected_distance, threshold);
                result->void_sector        = i;

                void_characterise_detection(distance, expected_distance, result);
                sprintf(result->status_text, "Void S%d: %dmm (fallback)", i, result->void_diameter_mm);

                return true;
            }
        }
    }

    strcpy(result->status_text, "No void detected (simple)");
    return false;
}

static uint8_t prv_calculate_circle_confidence(const circle_fit_data_t *circle_data, uint16_t expected_radius)
{
    if (!circle_data || !circle_data->fit_successful)
    {
        return 0;
    }

    uint8_t confidence = 0;

    // Factor 1: Geometric consistency (40% weight)
    uint16_t radius_deviation = (circle_data->radius_mm > expected_radius) ? (circle_data->radius_mm - expected_radius) : (expected_radius - circle_data->radius_mm);

    uint8_t geo_score = 0;
    if (radius_deviation < 10)
    {
        geo_score = 40;
    }
    else if (radius_deviation < 25)
    {
        geo_score = 30;
    }
    else if (radius_deviation < 50)
    {
        geo_score = 20;
    }
    else if (radius_deviation < 100)
    {
        geo_score = 10;
    }

    confidence += geo_score;

    // Factor 2: Fitting quality (35% weight)
    uint8_t fit_score = 0;
    if (circle_data->fit_error_mm < 5)
    {
        fit_score = 35;
    }
    else if (circle_data->fit_error_mm < 10)
    {
        fit_score = 25;
    }
    else if (circle_data->fit_error_mm < 20)
    {
        fit_score = 15;
    }
    else if (circle_data->fit_error_mm < 30)
    {
        fit_score = 5;
    }

    confidence += fit_score;

    // Factor 3: Sensor coverage (25% weight)
    uint8_t coverage_score = 0;
    if (circle_data->sensors_used >= 3)
    {
        coverage_score = 25;
    }
    else if (circle_data->sensors_used == 2)
    {
        coverage_score = 15;
    }
    else if (circle_data->sensors_used == 1)
    {
        coverage_score = 5;
    }

    confidence += coverage_score;

    return (confidence > 100) ? 100 : confidence;
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
    if (!result)
    {
        return;
    }

    uint16_t void_size = distance_mm - expected_mm;

    // Classify severity based on void size
    if (void_size < 20)
    {
        result->void_severity = VOID_SEVERITY_MINOR;
    }
    else if (void_size < 50)
    {
        result->void_severity = VOID_SEVERITY_MAJOR;
    }
    else
    {
        result->void_severity = VOID_SEVERITY_CRITICAL;
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
    if (result)
    {
        *result = prv_void_system.current_status;
    }
}

void void_get_current_config(void_config_t *config)
{
    if (config)
    {
        *config = prv_void_system.config;
    }
}

bool void_get_measurement_data(void_measurement_t *measurement)
{
    if (measurement)
    {
        *measurement = prv_void_system.latest_measurement;
        return true;
    }
    return false;
}

uint8_t void_get_history_count(void)
{
    return prv_void_system.history_count;
}

bool void_get_history_entry(uint8_t index, void_status_t *entry)
{
    if (entry && index < prv_void_system.history_count)
    {
        *entry = prv_void_system.history[index];
        return true;
    }
    return false;
}

void void_clear_history(void)
{
    prv_void_system.history_count = 0;
    memset(prv_void_system.history, 0, sizeof(prv_void_system.history));
    debug_send("Void history cleared");
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
        return "simple";
    case VOID_ALG_CIRCLEFIT:
        return "circlefit";
    default:
        return "unknown";
    }
}

// Private helper functions
static void prv_load_default_config(void)
{
    prv_void_system.config.baseline_diameter_mm   = VOID_DEFAULT_BASELINE_MM;
    prv_void_system.config.detection_threshold_mm = VOID_DEFAULT_THRESHOLD_MM;
    prv_void_system.config.confidence_threshold   = VOID_MIN_CONFIDENCE;
    prv_void_system.config.median_filter_enabled  = false;
    prv_void_system.config.range_min_mm           = 50;
    prv_void_system.config.range_max_mm           = 5000;

    // Circle fitting defaults
    prv_void_system.config.active_algorithm        = VOID_ALG_SIMPLE;
    prv_void_system.config.circle_fit_tolerance_mm = CIRCLE_FIT_DEFAULT_TOLERANCE_MM;
    prv_void_system.config.min_sensors_for_circle  = 3;
    prv_void_system.config.auto_fallback_enabled   = true;
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
        radar_measurement_t *radar_data = radar_get_measurement(i);
        if (radar_data && radar_data->data_valid)
        {
            prv_void_system.latest_measurement.distance_mm[i] = radar_data->distance_mm;
            prv_void_system.latest_measurement.angle_deg[i]   = radar_data->angle_deg;
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
    if (!status)
    {
        return;
    }

    // Add to circular buffer
    uint8_t index                  = prv_void_system.history_count % VOID_HISTORY_SIZE;
    prv_void_system.history[index] = *status;

    if (prv_void_system.history_count < VOID_HISTORY_SIZE)
    {
        prv_void_system.history_count++;
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
    memcpy(sorted, distances, count * sizeof(uint16_t));

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

    return sorted[count / 2];
}
