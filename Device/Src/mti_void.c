#include "mti_void.h"
#include "mti_radar.h"
#include "vmt_uart.h"
#include <string.h>
#include <stdio.h>

// Static system state
static void_system_state_t prv_void_system = { 0 };

// Private function prototypes
static void     prv_load_default_config(void);
static bool     prv_validate_sensor_data(uint16_t distance_mm, uint8_t sensor_idx);
static void     prv_update_measurement_data(void);
static void     prv_add_to_history(const void_status_t *status);
static uint16_t prv_apply_median_filter(uint16_t distances[], uint8_t count);

void void_system_init(void)
{
    // Initialize system state
    memset(&prv_void_system, 0, sizeof(prv_void_system));

    // Load default configuration
    prv_load_default_config();

    // Initialize status
    strcpy(prv_void_system.current_status.status_text, "System Ready");
    prv_void_system.current_status.baseline_diameter_mm = prv_void_system.config.baseline_diameter_mm;

    prv_void_system.system_initialized   = true;
    prv_void_system.last_process_time_ms = HAL_GetTick();

    debug_send("Void detection system initialized");
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
    strcpy(new_status.status_text, "No void detected");

    // Analyze each sensor
    bool void_found = false;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (prv_void_system.latest_measurement.data_valid[i])
        {
            void_status_t sensor_result = { 0 };

            if (void_analyze_sensor_data(i, prv_void_system.latest_measurement.distance_mm[i], prv_void_system.latest_measurement.angle_deg[i], &sensor_result))
            {
                // Void detected on this sensor
                if (sensor_result.confidence_percent >= prv_void_system.config.confidence_threshold)
                {
                    new_status             = sensor_result;
                    new_status.void_sector = i;
                    void_found             = true;

                    // Take the first high-confidence detection
                    break;
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
            debug_send("!vd,flag,%d,%d,%d", new_status.void_sector, new_status.void_diameter_mm, new_status.confidence_percent);
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

        sprintf(result->status_text, "Void S%d: %dmm", sensor_idx, result->void_diameter_mm);
        return true;
    }

    strcpy(result->status_text, "No void detected");
    return false;
}

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

// Private helper functions
static void prv_load_default_config(void)
{
    prv_void_system.config.baseline_diameter_mm   = VOID_DEFAULT_BASELINE_MM;
    prv_void_system.config.detection_threshold_mm = VOID_DEFAULT_THRESHOLD_MM;
    prv_void_system.config.confidence_threshold   = VOID_MIN_CONFIDENCE;
    prv_void_system.config.median_filter_enabled  = false;
    prv_void_system.config.range_min_mm           = 50;   // 5cm minimum
    prv_void_system.config.range_max_mm           = 5000; // 5m maximum
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
        prv_void_system.latest_measurement.data_valid[i] = radar_has_valid_data(i);

        if (prv_void_system.latest_measurement.data_valid[i])
        {
            prv_void_system.latest_measurement.distance_mm[i] = radar_get_distance_mm(i);
            prv_void_system.latest_measurement.angle_deg[i]   = radar_get_angle_deg(i);
        }
        else
        {
            prv_void_system.latest_measurement.distance_mm[i] = 0;
            prv_void_system.latest_measurement.angle_deg[i]   = 0;
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
