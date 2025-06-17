/**
 * @file mti_radar.c
 * @brief Radar data processing layer implementation
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#include "mti_radar.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include <string.h>
#include <math.h>

/*------------------------------------------------------------------------------
 * Static Variables and Configuration
 *----------------------------------------------------------------------------*/

/** @brief Latest processed measurements */
static radar_distance_t latest_measurements = { 0 };

/** @brief Processing configuration */
static struct
{
    float    snr_threshold;       // Minimum SNR for valid reading
    uint16_t min_distance_mm;     // Minimum valid distance
    uint16_t max_distance_mm;     // Maximum valid distance
    uint32_t last_process_time;   // Last processing timestamp
    bool     new_data_available;  // Flag for void detection layer
    uint16_t process_interval_ms; // Processing interval (configurable)
    uint8_t  process_rate_hz;     // Processing rate in Hz
    bool     sensors_started;     // Track if sensors are running
    uint8_t  sensor_profile;      // Current sensor profile
} radar_config = {
    .snr_threshold       = RADAR_MIN_SNR_THRESHOLD,
    .min_distance_mm     = RADAR_MIN_VALID_DISTANCE_MM,
    .max_distance_mm     = RADAR_MAX_VALID_DISTANCE_MM,
    .last_process_time   = 0,
    .new_data_available  = false,
    .process_interval_ms = RADAR_PROCESSING_INTERVAL_MS,
    .process_rate_hz     = 10, // Default 10Hz
    .sensors_started     = false,
    .sensor_profile      = 1 // Default to 50m single profile
};

/** @brief Sensor angle lookup table */
static const uint16_t sensor_angles[MAX_RADAR_SENSORS] = {
    SENSOR_0_ANGLE, // 0 degrees
    SENSOR_1_ANGLE, // 120 degrees
    SENSOR_2_ANGLE  // 240 degrees
};

/*------------------------------------------------------------------------------
 * Forward Declarations
 *----------------------------------------------------------------------------*/

static bool     process_raw_sensor_data(uint8_t sensor_idx);
static bool     validate_sensor_measurement(float distance_m, float snr);
static uint16_t convert_to_millimeters(float distance_m);
static void     update_system_health(void);

/*------------------------------------------------------------------------------
 * System Initialization
 *----------------------------------------------------------------------------*/

bool radar_system_init(void)
{
    debug_send("RADAR: Initializing processing system");

    // Initialize data structures
    memset(&latest_measurements, 0, sizeof(latest_measurements));

    // Set sensor angles
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        latest_measurements.angle_deg[i]  = sensor_angles[i];
        latest_measurements.data_valid[i] = false;
    }

    // Reset processing state
    radar_config.last_process_time  = HAL_GetTick();
    radar_config.new_data_available = false;

    debug_send("RADAR: System initialized");
    return true;
}

/*------------------------------------------------------------------------------
 * Main Processing Function
 *----------------------------------------------------------------------------*/

void radar_system_process(void)
{
    uint32_t current_time = HAL_GetTick();

    // Rate limiting - use configurable interval
    if ((current_time - radar_config.last_process_time) < radar_config.process_interval_ms)
    {
        return;
    }

    radar_config.last_process_time = current_time;

    bool new_data_processed = false;

    // Process each sensor that has new data
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (can_has_new_raw_data(i))
        {
            if (process_raw_sensor_data(i))
            {
                new_data_processed = true;
                can_mark_raw_data_processed(i);
            }
        }
    }

    if (new_data_processed)
    {
        // Update system health and sensor count
        update_system_health();

        // Update timestamp
        latest_measurements.timestamp_ms = current_time;

        // Mark new data available for void detection layer
        radar_config.new_data_available = true;

        debug_send("RADAR: Processed data - %d valid sensors", latest_measurements.valid_sensor_count);
    }
}

/*------------------------------------------------------------------------------
 * Data Processing Functions
 *----------------------------------------------------------------------------*/

static bool process_raw_sensor_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }

    // Get raw data from CAN layer
    radar_raw_t *raw_data = can_get_raw_data(sensor_idx);
    if (!raw_data || raw_data->num_points == 0)
    {
        latest_measurements.data_valid[sensor_idx] = false;
        return false;
    }

    // Find the closest valid point with highest SNR
    float best_distance     = 0.0f;
    float best_snr          = 0.0f;
    bool  valid_point_found = false;

    for (uint8_t point_idx = 0; point_idx < raw_data->num_points && point_idx < MAX_RADAR_DETECTED_POINTS; point_idx++)
    {
        float distance_m = raw_data->detected_points[point_idx][0];
        float snr        = raw_data->detected_points[point_idx][1];

        // Validate this measurement
        if (validate_sensor_measurement(distance_m, snr))
        {
            // Prefer closer points, but with good SNR
            if (!valid_point_found || (distance_m < best_distance && snr > (best_snr * 0.8f)) || (snr > best_snr * 1.2f))
            {
                best_distance     = distance_m;
                best_snr          = snr;
                valid_point_found = true;
            }
        }
    }

    if (valid_point_found)
    {
        // Store valid measurement
        latest_measurements.distance_mm[sensor_idx] = convert_to_millimeters(best_distance);
        latest_measurements.data_valid[sensor_idx]  = true;

        // Calculate confidence and quality based on SNR
        float confidence                              = fminf(best_snr / 100.0f, 1.0f); // Normalize SNR to 0-1
        latest_measurements.confidence[sensor_idx]    = confidence;
        latest_measurements.quality_score[sensor_idx] = (uint8_t)(confidence * 100.0f);

        debug_send("RADAR: S%d valid - %dmm, SNR=%.1f, conf=%.2f", sensor_idx, latest_measurements.distance_mm[sensor_idx], best_snr, confidence);
        return true;
    }
    else
    {
        // No valid points found
        latest_measurements.data_valid[sensor_idx]    = false;
        latest_measurements.confidence[sensor_idx]    = 0.0f;
        latest_measurements.quality_score[sensor_idx] = 0;

        debug_send("RADAR: S%d invalid - no valid points", sensor_idx);
        return false;
    }
}

static bool validate_sensor_measurement(float distance_m, float snr)
{
    // Check SNR threshold
    if (snr < radar_config.snr_threshold)
    {
        return false;
    }

    // Convert to millimeters for range check
    uint16_t distance_mm = convert_to_millimeters(distance_m);

    // Check distance range
    if (distance_mm < radar_config.min_distance_mm || distance_mm > radar_config.max_distance_mm)
    {
        return false;
    }

    return true;
}

static uint16_t convert_to_millimeters(float distance_m)
{
    // Convert meters to millimeters with bounds checking
    float distance_mm_f = distance_m * 1000.0f;

    if (distance_mm_f < 0.0f)
    {
        return 0;
    }
    else if (distance_mm_f > UINT16_MAX)
    {
        return UINT16_MAX;
    }
    else
    {
        return (uint16_t)distance_mm_f;
    }
}

static void update_system_health(void)
{
    // Count valid sensors
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (latest_measurements.data_valid[i])
        {
            valid_count++;
        }
    }

    latest_measurements.valid_sensor_count = valid_count;

    // System is healthy if we have at least 2 valid sensors and CAN system is healthy
    latest_measurements.system_healthy = (valid_count >= 2) && can_system_is_healthy();
}

/*------------------------------------------------------------------------------
 * Data Access Functions
 *----------------------------------------------------------------------------*/

bool radar_get_latest_measurements(radar_distance_t *measurements)
{
    if (!measurements)
    {
        return false;
    }

    // Copy latest measurements
    memcpy(measurements, &latest_measurements, sizeof(radar_distance_t));

    return radar_config.new_data_available;
}

bool radar_has_new_data(void)
{
    return radar_config.new_data_available;
}

void radar_mark_data_processed(void)
{
    radar_config.new_data_available = false;
}

bool radar_get_sensor_measurement(uint8_t sensor_idx, uint16_t *distance_mm, bool *valid)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !distance_mm || !valid)
    {
        return false;
    }

    *distance_mm = latest_measurements.distance_mm[sensor_idx];
    *valid       = latest_measurements.data_valid[sensor_idx];

    return true;
}

/*------------------------------------------------------------------------------
 * System Status Functions
 *----------------------------------------------------------------------------*/

bool radar_is_system_healthy(void)
{
    return latest_measurements.system_healthy;
}

uint8_t radar_get_valid_sensor_count(void)
{
    return latest_measurements.valid_sensor_count;
}

uint8_t radar_get_active_sensor_count(void)
{
    return get_active_sensor_count();
}

void radar_run_diagnostics(void)
{
    debug_send("=== RADAR System Diagnostics ===");
    debug_send("System healthy: %s", latest_measurements.system_healthy ? "YES" : "NO");
    debug_send("Valid sensors: %d/%d", latest_measurements.valid_sensor_count, MAX_RADAR_SENSORS);
    debug_send("SNR threshold: %.1f", radar_config.snr_threshold);
    debug_send("Distance range: %d-%dmm", radar_config.min_distance_mm, radar_config.max_distance_mm);
    debug_send("");

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        debug_send("Sensor %d (%d°): %s", i, latest_measurements.angle_deg[i], latest_measurements.data_valid[i] ? "VALID" : "INVALID");

        if (latest_measurements.data_valid[i])
        {
            debug_send("  Distance: %dmm, Confidence: %.2f, Quality: %d%%",
                       latest_measurements.distance_mm[i],
                       latest_measurements.confidence[i],
                       latest_measurements.quality_score[i]);
        }
    }

    debug_send("===============================");
}

/*------------------------------------------------------------------------------
 * Configuration Functions
 *----------------------------------------------------------------------------*/

void radar_set_snr_threshold(float snr_threshold)
{
    radar_config.snr_threshold = snr_threshold;
    debug_send("RADAR: SNR threshold set to %.1f", snr_threshold);
}

void radar_set_distance_range(uint16_t min_mm, uint16_t max_mm)
{
    if (min_mm < max_mm)
    {
        radar_config.min_distance_mm = min_mm;
        radar_config.max_distance_mm = max_mm;
        debug_send("RADAR: Distance range set to %d-%dmm", min_mm, max_mm);
    }
}

void radar_set_processing_rate(uint8_t rate_hz)
{
    if (rate_hz >= 1 && rate_hz <= 50)
    {
        radar_config.process_rate_hz     = rate_hz;
        radar_config.process_interval_ms = 1000 / rate_hz;
        debug_send("RADAR: Processing rate set to %dHz (%dms interval)", rate_hz, radar_config.process_interval_ms);
    }
}

uint8_t radar_get_processing_rate(void)
{
    return radar_config.process_rate_hz;
}

/*------------------------------------------------------------------------------
 * Sensor Control Functions
 *----------------------------------------------------------------------------*/

bool radar_start_sensors(void)
{
    debug_send("RADAR: Starting sensors...");

    // Send start command to all sensors
    if (!can_send_command_to_all_sensors(CAN_CMD_PAYLOAD_START))
    {
        debug_send("RADAR: Failed to send start commands");
        return false;
    }

    HAL_Delay(200); // Wait for sensors to respond

    // Configure sensors with default profile
    debug_send("RADAR: Configuring sensors (profile %d)...", radar_config.sensor_profile);
    uint8_t config_cmd[2] = { CAN_CMD_PAYLOAD_SELECT_PROFILE, radar_config.sensor_profile };

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (!can_send_array(CAN_COMMAND_BASE_ID + (i * 0x10), config_cmd, 2))
        {
            debug_send("RADAR: Failed to configure sensor %d", i);
        }
        HAL_Delay(50);
    }

    HAL_Delay(500); // Wait for configuration

    // Check if sensors are responding
    uint8_t online_sensors = can_get_online_sensor_count();

    if (online_sensors >= 2)
    {
        radar_config.sensors_started = true;
        debug_send("RADAR: Sensors started successfully (%d online)", online_sensors);
        return true;
    }
    else
    {
        debug_send("RADAR: Insufficient sensors online (%d/3)", online_sensors);
        return false;
    }
}

bool radar_stop_sensors(void)
{
    debug_send("RADAR: Stopping sensors...");

    // Send stop command to all sensors
    if (can_send_command_to_all_sensors(CAN_CMD_PAYLOAD_STOP))
    {
        radar_config.sensors_started = false;
        debug_send("RADAR: Sensors stopped");
        return true;
    }
    else
    {
        debug_send("RADAR: Failed to stop sensors");
        return false;
    }
}

bool radar_configure_sensors(uint8_t profile, uint8_t threshold)
{
    if (!radar_config.sensors_started)
    {
        debug_send("RADAR: Cannot configure - sensors not started");
        return false;
    }

    debug_send("RADAR: Configuring sensors (profile=%d, threshold=%d)", profile, threshold);

    // Set profile
    uint8_t profile_cmd[2]   = { CAN_CMD_PAYLOAD_SELECT_PROFILE, profile };
    uint8_t threshold_cmd[2] = { CAN_CMD_PAYLOAD_DET_THRESHOLD, threshold };

    bool success = true;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        // Send profile command
        if (!can_send_array(CAN_COMMAND_BASE_ID + (i * 0x10), profile_cmd, 2))
        {
            debug_send("RADAR: Failed to set profile for sensor %d", i);
            success = false;
        }
        HAL_Delay(50);

        // Send threshold command
        if (!can_send_array(CAN_COMMAND_BASE_ID + (i * 0x10), threshold_cmd, 2))
        {
            debug_send("RADAR: Failed to set threshold for sensor %d", i);
            success = false;
        }
        HAL_Delay(50);
    }

    if (success)
    {
        radar_config.sensor_profile = profile;
        debug_send("RADAR: Configuration complete");
    }

    return success;
}

bool radar_sensors_are_running(void)
{
    return radar_config.sensors_started && can_system_is_healthy();
}

/*------------------------------------------------------------------------------
 * Interface Functions (called by CAN layer)
 *----------------------------------------------------------------------------*/

void radar_notify_new_raw_data(uint8_t sensor_idx)
{
    // This function is called by process_complete_radar_frame() in mti_can.c
    // The actual processing happens in radar_system_process() to maintain
    // rate limiting and proper layer separation

    // We could add immediate processing here if needed, but for now
    // we rely on the main loop calling radar_system_process()
}

void radar_system_shutdown(void)
{
    debug_send("RADAR: Shutting down system...");

    // Stop sensors
    radar_stop_sensors();

    // Reset state
    memset(&latest_measurements, 0, sizeof(latest_measurements));
    radar_config.new_data_available = false;
    radar_config.sensors_started    = false;

    debug_send("RADAR: Shutdown complete");
}

/*------------------------------------------------------------------------------
 * Compatibility Functions (for existing integration)
 *----------------------------------------------------------------------------*/

bool radar_has_valid_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }
    return latest_measurements.data_valid[sensor_idx];
}

uint16_t radar_get_distance_mm(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return 0;
    }

    if (latest_measurements.data_valid[sensor_idx])
    {
        return latest_measurements.distance_mm[sensor_idx];
    }

    return 0;
}
