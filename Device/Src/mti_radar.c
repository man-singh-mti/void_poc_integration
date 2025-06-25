/**
 * @file mti_radar.c
 * @brief Radar system implementation - Data processing and sensor management
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#include "mti_radar.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include <string.h>
#include <math.h>

/*------------------------------------------------------------------------------
 * Static Variables and Configuration
 *----------------------------------------------------------------------------*/

/** @brief Latest processed measurements */
static radar_distance_t latest_measurements = { 0 };

/** @brief System configuration */
static struct
{
    float    snr_threshold;       // Minimum SNR for valid reading
    uint16_t min_distance_mm;     // Minimum valid distance
    uint16_t max_distance_mm;     // Maximum valid distance
    uint32_t last_process_time;   // Last processing timestamp
    bool     new_data_available;  // Flag for void detection layer
    uint16_t process_interval_ms; // Processing interval
    uint8_t  process_rate_hz;     // Processing rate in Hz
    bool     system_initialized;  // Initialization complete flag
    bool     firmware_verified;   // Firmware verification status
} radar_config = { .snr_threshold       = RADAR_MIN_SNR_THRESHOLD,
                   .min_distance_mm     = RADAR_MIN_VALID_DISTANCE_MM,
                   .max_distance_mm     = RADAR_MAX_VALID_DISTANCE_MM,
                   .last_process_time   = 0,
                   .new_data_available  = false,
                   .process_interval_ms = RADAR_PROCESSING_INTERVAL_MS,
                   .process_rate_hz     = 10,
                   .system_initialized  = false,
                   .firmware_verified   = false };

/** @brief Sensor angle lookup table */
static const uint16_t sensor_angles[MAX_RADAR_SENSORS] = {
    0,   // Sensor 0: 0 degrees
    120, // Sensor 1: 120 degrees
    240  // Sensor 2: 240 degrees
};

/** @brief Event-driven processing statistics */
static struct
{
    uint32_t events_processed;
    uint32_t last_event_time;
    bool     event_driven_mode;
} event_stats = { 0, 0, true };

/*------------------------------------------------------------------------------
 * Forward Declarations
 *----------------------------------------------------------------------------*/

static bool     process_sensor_data_from_can(uint8_t sensor_idx, can_sensor_t *sensor);
static bool     validate_sensor_measurement(uint8_t sensor_idx, float distance_m, float snr);
static uint16_t convert_to_millimeters(float distance_m);
static void     update_system_health(void);
static bool     verify_sensor_firmware(uint8_t sensor_idx);

/*------------------------------------------------------------------------------
 * System Initialization
 *----------------------------------------------------------------------------*/

bool radar_system_init(void)
{
    debug_send("=== RADAR SYSTEM INITIALIZATION START ===");

    // Initialize data structures
    memset(&latest_measurements, 0, sizeof(latest_measurements));

    // Set sensor angles
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        latest_measurements.angle_deg[i]  = sensor_angles[i]; // Should be 0, 120, 240
        latest_measurements.data_valid[i] = false;

        debug_send("RADAR: Sensor %d assigned angle %d deg", i, sensor_angles[i]);
    }

    // Reset processing state
    radar_config.last_process_time  = HAL_GetTick();
    radar_config.new_data_available = false;

    debug_send("RADAR: Data structures initialized");

    // Wait for CAN system to be ready
    if (!can_is_system_healthy())
    {
        debug_send("RADAR: Waiting for CAN system...");
        uint32_t wait_start = HAL_GetTick();
        while (!can_is_system_healthy() && (HAL_GetTick() - wait_start) < 5000)
        {
            HAL_Delay(100);
        }

        if (!can_is_system_healthy())
        {
            debug_send("RADAR: ERROR - CAN system not ready after 5 seconds");
            return false;
        }
    }

    debug_send("RADAR: CAN system ready, %d sensors online", can_get_online_count());

    // Verify firmware versions
    debug_send("RADAR: Verifying sensor firmware versions...");
    radar_config.firmware_verified = radar_verify_firmware_versions();

    if (!radar_config.firmware_verified)
    {
        debug_send("RADAR: WARNING - Firmware verification failed, continuing anyway");
    }

    // Set sensors to calibration mode initially
    debug_send("RADAR: Setting sensors to calibration mode");
    if (!radar_set_calibration_mode())
    {
        debug_send("RADAR: WARNING - Failed to set calibration mode");
    }

    HAL_Delay(500); // Allow sensors to configure

    // Mark system as initialized
    radar_config.system_initialized = true;

    // Report configuration to debug and uphole
    radar_report_configuration(UART_DEBUG);
    radar_report_configuration(UART_UPHOLE);

    debug_send("RADAR: System initialization complete");
    debug_send("=== RADAR SYSTEM INITIALIZATION END ===");

    return true;
}

/*------------------------------------------------------------------------------
 * Main Processing Function
 *----------------------------------------------------------------------------*/

// Enhanced radar_system_process to handle per-sensor flags
void radar_system_process(void)
{
    if (!radar_config.system_initialized)
    {
        return;
    }

    // Check if any CAN sensor has new data
    if (!can_has_new_radar_data())
    {
        return; // Exit immediately if no new data
    }

    uint32_t current_time       = HAL_GetTick();
    bool     new_data_processed = false;

    // debug_send("RADAR: Processing triggered by CAN data");

    // Process each sensor that has new data
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (can_sensor_has_new_data(i))
        {
            can_sensor_t *sensor = can_get_sensor(i);
            if (sensor && sensor->online)
            {
                if (process_sensor_data_from_can(i, sensor))
                {
                    new_data_processed = true;
                    // debug_send("RADAR: Processed sensor %d (frame #%lu)", i, can_get_sensor_frame_count(i));
                }
            }

            // Clear this sensor's flag
            can_clear_sensor_data_flag(i);
        }
        else
        {
            // Mark sensors without new data as potentially invalid
            can_sensor_t *sensor = can_get_sensor(i);
            if (!sensor || !sensor->online)
            {
                latest_measurements.data_valid[i]    = false;
                latest_measurements.confidence[i]    = 0.0f;
                latest_measurements.quality_score[i] = 0;
            }
        }
    }

    if (new_data_processed)
    {
        // Update system health and sensor count
        update_system_health();
        latest_measurements.timestamp_ms = current_time;
        radar_config.new_data_available  = true;

        // Update event statistics
        event_stats.events_processed++;
        event_stats.last_event_time = current_time;

        // debug_send("RADAR: Event #%lu processed - %d valid sensors", event_stats.events_processed, latest_measurements.valid_sensor_count);
    }

    // Update last process time
    radar_config.last_process_time = current_time;
}

/*------------------------------------------------------------------------------
 * Data Processing Functions
 *----------------------------------------------------------------------------*/

static bool process_sensor_data_from_can(uint8_t sensor_idx, can_sensor_t *sensor)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !sensor)
    {
        return false;
    }

    // Check if sensor has detection points - use current_point_count instead of num_points
    if (sensor->current_point_count == 0)
    {
        latest_measurements.data_valid[sensor_idx] = false;
        return false;
    }

    // Find the best detection point (closest with highest SNR)
    float best_distance     = 0.0f;
    float best_snr          = 0.0f;
    bool  valid_point_found = false;

    // Use current_point_count - this is the actual number of points received
    for (uint8_t point_idx = 0; point_idx < sensor->current_point_count && point_idx < MAX_RADAR_DETECTED_POINTS; point_idx++)
    {
        float distance_m = sensor->detection_points[point_idx][0];
        float snr        = sensor->detection_points[point_idx][1];

        if (validate_sensor_measurement(sensor_idx, distance_m, snr)) // Add sensor_idx parameter
        {
            // Prefer closer points with good SNR
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
        float confidence                              = fminf(best_snr / 100.0f, 1.0f);
        latest_measurements.confidence[sensor_idx]    = confidence;
        latest_measurements.quality_score[sensor_idx] = (uint8_t)(confidence * 100.0f);

        return true;
    }
    else
    {
        latest_measurements.data_valid[sensor_idx]    = false;
        latest_measurements.confidence[sensor_idx]    = 0.0f;
        latest_measurements.quality_score[sensor_idx] = 0;
        return false;
    }
}

static bool validate_sensor_measurement(uint8_t sensor_idx, float distance_m, float snr)
{
    // debug_send("RADAR: Validating S%d: dist=%.3fm, snr=%.1f, thresh=%.1f", sensor_idx, distance_m, snr, radar_config.snr_threshold);

    // Check SNR threshold
    if (snr < radar_config.snr_threshold)
    {
        //  debug_send("RADAR: S%d REJECTED - SNR %.1f < %.1f", sensor_idx, snr, radar_config.snr_threshold);
        return false;
    }

    // Convert to millimeters for range check
    uint16_t distance_mm = convert_to_millimeters(distance_m);

    // Check distance range
    if (distance_mm < radar_config.min_distance_mm || distance_mm > radar_config.max_distance_mm)
    {
        //  debug_send("RADAR: S%d REJECTED - distance %dmm out of range [%d-%dmm]", sensor_idx, distance_mm, radar_config.min_distance_mm,
        //  radar_config.max_distance_mm);
        return false;
    }

    //  debug_send("RADAR: S%d VALID - %.3fm, %.1f SNR", sensor_idx, distance_m, snr);
    return true;
}

static uint16_t convert_to_millimeters(float distance_m)
{
    float distance_mm_f = distance_m * 1000.0f;

    if (distance_mm_f < 0.0f)
        return 0;
    else if (distance_mm_f > UINT16_MAX)
        return UINT16_MAX;
    else
        return (uint16_t)distance_mm_f;
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
    latest_measurements.system_healthy = (valid_count >= 2) && can_is_system_healthy();
}

/*------------------------------------------------------------------------------
 * Sensor Management Functions
 *----------------------------------------------------------------------------*/

bool radar_set_calibration_mode(void)
{
    debug_send("RADAR: Setting calibration mode (profile %d)", RADAR_PROFILE_CALIBRATION);

    bool success = true;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint8_t profile_data[1] = { RADAR_PROFILE_CALIBRATION };
        if (!can_send_command_data(i, CAN_CMD_SELECT_PROFILE, profile_data, 1))
        {
            debug_send("RADAR: Failed to set calibration mode for sensor %d", i);
            success = false;
        }
        HAL_Delay(50);
    }

    return success;
}

bool radar_set_measurement_mode(void)
{
    debug_send("RADAR: Setting measurement mode (profile %d)", RADAR_PROFILE_MEASUREMENT);

    bool success = true;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint8_t profile_data[1] = { RADAR_PROFILE_MEASUREMENT };
        if (!can_send_command_data(i, CAN_CMD_SELECT_PROFILE, profile_data, 1))
        {
            debug_send("RADAR: Failed to set measurement mode for sensor %d", i);
            success = false;
        }
        HAL_Delay(50);
    }

    return success;
}

bool radar_start_sensors(void)
{
    debug_send("RADAR: Starting sensors");
    return can_send_to_all_sensors(CAN_CMD_START);
}

bool radar_stop_sensors(void)
{
    debug_send("RADAR: Stopping sensors");
    return can_send_to_all_sensors(CAN_CMD_STOP);
}

/*------------------------------------------------------------------------------
 * Firmware Verification
 *----------------------------------------------------------------------------*/

bool radar_verify_firmware_versions(void)
{
    debug_send("RADAR: Verifying firmware versions (expected: v%d.%d.%d)", RADAR_EXPECTED_FW_MAJOR, RADAR_EXPECTED_FW_MINOR, RADAR_EXPECTED_FW_PATCH);

    bool    all_verified = true;
    uint8_t online_count = can_get_online_count();

    if (online_count == 0)
    {
        debug_send("RADAR: No sensors online for firmware verification");
        return false;
    }

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        can_sensor_t *sensor = can_get_sensor(i);
        if (sensor && sensor->online)
        {
            if (!verify_sensor_firmware(i))
            {
                all_verified = false;
            }
        }
    }

    if (all_verified)
    {
        debug_send("RADAR: All online sensors have correct firmware");
    }
    else
    {
        debug_send("RADAR: WARNING - Some sensors have incorrect firmware");
    }

    return all_verified;
}

static bool verify_sensor_firmware(uint8_t sensor_idx)
{
    can_sensor_t *sensor = can_get_sensor(sensor_idx);
    if (!sensor || !sensor->online)
    {
        return false;
    }

    uint8_t major = sensor->fw_version[0];
    uint8_t minor = sensor->fw_version[1];
    uint8_t patch = sensor->fw_version[2];

    bool version_correct = (major == RADAR_EXPECTED_FW_MAJOR) && (minor == RADAR_EXPECTED_FW_MINOR) && (patch == RADAR_EXPECTED_FW_PATCH);

    if (version_correct)
    {
        debug_send("RADAR: Sensor %d firmware OK: v%d.%d.%d", sensor_idx, major, minor, patch);
    }
    else
    {
        debug_send("RADAR: Sensor %d firmware MISMATCH: v%d.%d.%d (expected v%d.%d.%d)",
                   sensor_idx,
                   major,
                   minor,
                   patch,
                   RADAR_EXPECTED_FW_MAJOR,
                   RADAR_EXPECTED_FW_MINOR,
                   RADAR_EXPECTED_FW_PATCH);
    }

    return version_correct;
}

bool radar_get_sensor_firmware(uint8_t sensor_idx, uint8_t *major, uint8_t *minor, uint8_t *patch)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !major || !minor || !patch)
    {
        return false;
    }

    can_sensor_t *sensor = can_get_sensor(sensor_idx);
    if (!sensor || !sensor->online)
    {
        return false;
    }

    *major = sensor->fw_version[0];
    *minor = sensor->fw_version[1];
    *patch = sensor->fw_version[2];

    return true;
}

/*------------------------------------------------------------------------------
 * Configuration and Status Reporting
 *----------------------------------------------------------------------------*/

void radar_report_configuration(uart_select_t channel)
{
    uart_tx_channel_set(channel);

    printf("@radar,config,initialized,%s\n", radar_config.system_initialized ? "1" : "0");
    printf("@radar,config,firmware_verified,%s\n", radar_config.firmware_verified ? "1" : "0");
    printf("@radar,config,online_sensors,%d\n", can_get_online_count());
    printf("@radar,config,valid_sensors,%d\n", latest_measurements.valid_sensor_count);
    printf("@radar,config,system_healthy,%s\n", latest_measurements.system_healthy ? "1" : "0");
    printf("@radar,config,snr_threshold,%.1f\n", radar_config.snr_threshold);
    printf("@radar,config,distance_range,%d,%d\n", radar_config.min_distance_mm, radar_config.max_distance_mm);
    printf("@radar,config,process_rate,%d\n", radar_config.process_rate_hz);

    // Report individual sensor firmware
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        can_sensor_t *sensor = can_get_sensor(i);
        if (sensor && sensor->online)
        {
            printf("@radar,sensor,%d,firmware,%d,%d,%d\n", i, sensor->fw_version[0], sensor->fw_version[1], sensor->fw_version[2]);
        }
        else
        {
            printf("@radar,sensor,%d,offline\n", i);
        }
    }

    uart_tx_channel_undo();
}

void radar_run_diagnostics(void)
{
    debug_send("=== RADAR SYSTEM DIAGNOSTICS ===");
    debug_send("System initialized: %s", radar_config.system_initialized ? "YES" : "NO");
    debug_send("Firmware verified: %s", radar_config.firmware_verified ? "YES" : "NO");
    debug_send("System healthy: %s", latest_measurements.system_healthy ? "YES" : "NO");
    debug_send("Valid sensors: %d/%d", latest_measurements.valid_sensor_count, MAX_RADAR_SENSORS);
    debug_send("Online sensors: %d/%d", can_get_online_count(), MAX_RADAR_SENSORS);
    debug_send("SNR threshold: %.1f", radar_config.snr_threshold);
    debug_send("Distance range: %d-%dmm", radar_config.min_distance_mm, radar_config.max_distance_mm);
    debug_send("Process rate: %dHz", radar_config.process_rate_hz);
    debug_send("");

    // Individual sensor diagnostics
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        can_sensor_t *sensor = can_get_sensor(i);
        debug_send("Sensor %d (%d d):", i, latest_measurements.angle_deg[i]);

        if (sensor && sensor->online)
        {
            debug_send("  Status: ONLINE");
            debug_send("  Firmware: v%d.%d.%d", sensor->fw_version[0], sensor->fw_version[1], sensor->fw_version[2]);
            debug_send("  Messages: %lu", sensor->msg_count);

            if (latest_measurements.data_valid[i])
            {
                debug_send("  Distance: %dmm", latest_measurements.distance_mm[i]);
                debug_send("  Confidence: %.2f", latest_measurements.confidence[i]);
                debug_send("  Quality: %d%%", latest_measurements.quality_score[i]);
            }
            else
            {
                debug_send("  Data: INVALID");
            }
        }
        else
        {
            debug_send("  Status: OFFLINE");
        }
        debug_send("");
    }

    debug_send("===============================");
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

/*------------------------------------------------------------------------------
 * System Status Functions
 *----------------------------------------------------------------------------*/

bool radar_is_system_healthy(void)
{
    return radar_config.system_initialized && latest_measurements.system_healthy;
}

uint8_t radar_get_valid_sensor_count(void)
{
    return latest_measurements.valid_sensor_count;
}

uint8_t radar_get_active_sensor_count(void)
{
    return can_get_online_count();
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

/*------------------------------------------------------------------------------
 * System Lifecycle
 *----------------------------------------------------------------------------*/

void radar_system_shutdown(void)
{
    debug_send("RADAR: Shutting down system");

    // Stop sensors
    radar_stop_sensors();

    // Reset state
    memset(&latest_measurements, 0, sizeof(latest_measurements));
    radar_config.new_data_available = false;
    radar_config.system_initialized = false;
    radar_config.firmware_verified  = false;

    debug_send("RADAR: Shutdown complete");
}

/*------------------------------------------------------------------------------
 * Test & Debug Functions
 *----------------------------------------------------------------------------*/

void radar_debug_measurements_periodic(void)
{
    static uint32_t last_debug_time = 0;
    static bool     sensors_started = false;
    static uint32_t debug_counter   = 0;

    uint32_t now = HAL_GetTick();

    if ((now - last_debug_time) >= 2000)
    {
        last_debug_time = now;
        debug_counter++;

        debug_send("=== RADAR MEASUREMENTS DEBUG #%lu ===", debug_counter);

        // First time: start sensors and enable event-driven mode
        if (!sensors_started)
        {
            debug_send("FIRST RUN: Starting sensors for event-driven processing");

            if (radar_start_sensors())
            {
                debug_send("Sensors started successfully");
                HAL_Delay(200);

                if (radar_set_measurement_mode())
                {
                    debug_send("Switched to measurement mode");
                    sensors_started = true;
                }
                else
                {
                    debug_send("ERROR: Failed to set measurement mode");
                }
            }
            else
            {
                debug_send("ERROR: Failed to start sensors");
            }

            debug_send("Waiting 1 second for sensors to stabilize...");
            HAL_Delay(1000);
        }

        // Show event-driven processing stats
        debug_send("Event Processing: CAN events=%s, Radar events=%lu, Void data=%s",
                   can_has_new_radar_data() ? "READY" : "idle",
                   event_stats.events_processed,
                   radar_has_new_data() ? "READY" : "idle");

        // Get latest processed measurements
        radar_distance_t measurements;
        bool             new_data = radar_get_latest_measurements(&measurements);

        debug_send("Data available: %s | System healthy: %s | Valid sensors: %d/%d",
                   new_data ? "YES" : "NO",
                   measurements.system_healthy ? "YES" : "NO",
                   measurements.valid_sensor_count,
                   MAX_RADAR_SENSORS);

        // Report individual sensor measurements
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            debug_send("Sensor %d (%d d):", i, measurements.angle_deg[i]);

            if (measurements.data_valid[i])
            {
                debug_send("  Distance: %dmm", measurements.distance_mm[i]);
                debug_send("  Confidence: %.2f", measurements.confidence[i]);
                debug_send("  Quality: %d%%", measurements.quality_score[i]);
                debug_send("  Status: VALID");
            }
            else
            {
                debug_send("  Status: INVALID - No valid detection points");
                debug_send("  Distance: N/A");
                debug_send("  Confidence: %.2f", measurements.confidence[i]);
                debug_send("  Quality: %d%%", measurements.quality_score[i]);
            }

            // Also show raw CAN sensor data for comparison
            can_sensor_t *can_sensor = can_get_sensor(i);
            if (can_sensor && can_sensor->online)
            {
                debug_send("  CAN Status: ONLINE - Points: %d - Frame: %lu",
                           can_sensor->current_point_count, // Use current_point_count
                           can_sensor->frame_number);

                // Show first few raw points
                for (uint8_t p = 0; p < can_sensor->current_point_count && p < 3; p++)
                {
                    debug_send("  Raw Point %d: %.3fm (%.1f SNR)", p, can_sensor->detection_points[p][0], can_sensor->detection_points[p][1]);
                }
            }
            else
            {
                debug_send("  CAN Status: OFFLINE");
            }
        }

        // Summary line for easy parsing
        debug_send("SUMMARY: [%d,%d,%d] valid=[%s,%s,%s] distances=[%d,%d,%d]mm",
                   measurements.distance_mm[0],
                   measurements.distance_mm[1],
                   measurements.distance_mm[2],
                   measurements.data_valid[0] ? "Y" : "N",
                   measurements.data_valid[1] ? "Y" : "N",
                   measurements.data_valid[2] ? "Y" : "N",
                   measurements.data_valid[0] ? measurements.distance_mm[0] : 0,
                   measurements.data_valid[1] ? measurements.distance_mm[1] : 0,
                   measurements.data_valid[2] ? measurements.distance_mm[2] : 0);

        debug_send("Timestamp: %lums | Processing healthy: %s", measurements.timestamp_ms, (HAL_GetTick() - measurements.timestamp_ms) < 1000 ? "YES" : "STALE");

        debug_send("=== END MEASUREMENTS DEBUG ===");
    }
}

void radar_get_event_stats(uint32_t *event_count, uint32_t *last_event_time)
{
    if (event_count)
    {
        *event_count = event_stats.events_processed;
    }
    if (last_event_time)
    {
        *last_event_time = event_stats.last_event_time;
    }
}
