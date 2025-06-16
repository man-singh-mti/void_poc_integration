/**
 * @file    mti_radar.c
 * @brief   Radar sensor control and data processing implementation
 * @author  MTi Group
 * @copyright 2025 MTi Group
 *
 * This file implements the interface defined in mti_radar.h for controlling
 * radar sensors in continuous mode and processing radar measurements.
 */

#include "mti_radar.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_void.h"
#include <string.h>

/**
 * @brief Internal storage for radar measurements
 *
 * The measurements are stored here but also accessible via the CAN module's
 * radar_system structure. This avoids duplicated storage while maintaining
 * the API contract for radar_get_measurement().
 */
static radar_measurement_t radar_measurements[MAX_RADAR_SENSORS] = { 0 };

/**
 * @brief Current operational status of the radar system
 */
static uint8_t radar_status = RADAR_INITIALISING;

/**
 * @brief Sensor angle lookup table in degrees
 * Maps sensor indices to their angular positions
 */
static const uint16_t sensor_angles[MAX_RADAR_SENSORS] = {
    SENSOR_0_ANGLE, /**< Sensor 0: 0 deg */
    SENSOR_1_ANGLE, /**< Sensor 1: 120 deg */
    SENSOR_2_ANGLE  /**< Sensor 2: 240 deg */
};

/**
 * @brief Initialize the radar system for continuous operation
 */
void radar_system_init(void)
{
    /* Initialize measurement storage */
    memset(radar_measurements, 0, sizeof(radar_measurements));

    debug_send("Radar system initializing in continuous mode");

    /* Set system status as initializing */
    radar_status = RADAR_INITIALISING;

    /* Initialize CAN for continuous mode operation */
    if (!can_initialize_continuous_mode())
    {
        debug_send("CAN initialization failed!");
        return;
    }

    /* Set proper angle values */
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        radar_measurements[i].angle_deg = sensor_angles[i];
    }

    /* Mark system as ready */
    radar_status = RADAR_READY;

    debug_send("Radar system initialized successfully");
}

/**
 * @brief Process radar system operations
 *
 * Handles periodic health checks and monitoring.
 * Should be called regularly from the main program loop.
 */
void radar_system_process(void)
{
    /* Skip processing if system isn't ready */
    if (radar_status == RADAR_INITIALISING)
    {
        return;
    }

    /* Check for stale data and invalidate if necessary */
    uint32_t current_time = HAL_GetTick();
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_measurements[i].data_valid)
        {
            /* Check if data is stale */
            if ((current_time - radar_measurements[i].timestamp_ms) > RADAR_DATA_TIMEOUT_MS)
            {
                radar_measurements[i].data_valid = false;
                debug_send("S%d: Data marked stale (age: %dms)", i, (current_time - radar_measurements[i].timestamp_ms));
            }
        }
    }

    /* Every 1 second, monitor sensor health */
    static uint32_t last_health_check = 0;
    if (current_time - last_health_check >= 1000)
    {
        /* Check sensor health and restart any offline sensors */
        monitor_sensor_health();
        last_health_check = current_time;
    }

    /* Adaptively adjust stale data timeout based on observed data rate */
    static uint32_t data_rate_check_time                  = 0;
    static uint32_t message_count[MAX_RADAR_SENSORS]      = { 0 };
    static uint32_t last_message_count[MAX_RADAR_SENSORS] = { 0 };
    static uint32_t adaptive_timeout_ms                   = RADAR_DATA_TIMEOUT_MS;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        // Count each valid message received
        if (radar_measurements[i].data_valid && radar_measurements[i].timestamp_ms > data_rate_check_time)
        {
            message_count[i]++;
        }
    }

    // Every 5 seconds, estimate data rate and adjust timeout
    if (current_time - data_rate_check_time >= 5000)
    {
        float   avg_messages_per_sec = 0;
        uint8_t active_sensors       = 0;

        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            if (message_count[i] > last_message_count[i])
            {
                float sensor_rate = (float)(message_count[i] - last_message_count[i]) / 5.0f;
                avg_messages_per_sec += sensor_rate;
                active_sensors++;
                last_message_count[i] = message_count[i];
            }
        }

        if (active_sensors > 0)
        {
            avg_messages_per_sec /= active_sensors;

            // Set timeout to approximately 3x the average message period
            if (avg_messages_per_sec > 0)
            {
                uint32_t new_timeout = (uint32_t)(3000.0f / avg_messages_per_sec);

                // Constrain between reasonable bounds (300ms - 5000ms)
                if (new_timeout < 300)
                    new_timeout = 300;
                if (new_timeout > 5000)
                    new_timeout = 5000;

                // Only report significant changes
                if (abs((int)new_timeout - (int)adaptive_timeout_ms) > 100)
                {
                    debug_send("Adjusting radar timeout: %dms → %dms (data rate: %.1fHz)", adaptive_timeout_ms, new_timeout, avg_messages_per_sec);
                    adaptive_timeout_ms = new_timeout;
                }
            }
        }

        data_rate_check_time = current_time;
    }

    // Use the adaptive timeout for stale data detection
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_measurements[i].data_valid)
        {
            if ((current_time - radar_measurements[i].timestamp_ms) > adaptive_timeout_ms)
            {
                radar_measurements[i].data_valid = false;
                debug_send("S%d: Data marked stale (age: %dms, timeout: %dms)", i, (current_time - radar_measurements[i].timestamp_ms), adaptive_timeout_ms);
            }
        }
    }
}

/**
 * @brief Start a specific radar sensor
 *
 * @param sensor_idx Index of the sensor to start (0-2)
 */
void radar_sensor_start(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    /* Send start command to specific sensor */
    can_send_to_sensor(sensor_idx, CAN_CMD_BASE, CAN_CMD_START);
    debug_send("Starting sensor %d", sensor_idx);

    /* Update system status */
    radar_status = RADAR_CHIRPING;
}

/**
 * @brief Stop a specific radar sensor
 *
 * @param sensor_idx Index of the sensor to stop (0-2)
 */
void radar_sensor_stop(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    /* Send stop command to specific sensor */
    can_send_to_sensor(sensor_idx, CAN_CMD_BASE, CAN_CMD_STOP);
    debug_send("Stopping sensor %d", sensor_idx);

    /* Mark data as invalid since sensor is stopped */
    radar_measurements[sensor_idx].data_valid = false;
}

/**
 * @brief Process raw radar measurement data
 *
 * Analyzes detected points from a radar sensor and determines the closest valid
 * point to represent the borehole wall. This function is called by the CAN
 * interrupt handler when a complete frame of data is received.
 *
 * @param sensor_idx      Index of the sensor providing the measurement (0-2)
 * @param detectedPoints  Array of points detected by radar [distance_m, SNR]
 * @param numPoints       Number of detected points in the array
 */
void radar_process_measurement(uint8_t sensor_idx, float detectedPoints[MAX_RADAR_DETECTED_POINTS][2], uint8_t numPoints)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        debug_send("ERROR: Invalid sensor index %d in radar_process_measurement", sensor_idx);
        return;
    }

    if (detectedPoints == NULL)
    {
        debug_send("ERROR: NULL detectedPoints in radar_process_measurement for sensor %d", sensor_idx);
        return;
    }

    radar_measurement_t *measurement = &radar_measurements[sensor_idx];

    /* Reset measurement */
    measurement->distance_mm  = 0;
    measurement->data_valid   = false;
    measurement->timestamp_ms = HAL_GetTick();

    /* Find the CLOSEST valid point (closest to borehole wall) */
    float closest_distance = 999.0f;
    bool  found_valid      = false;

    for (uint8_t i = 0; i < numPoints && i < MAX_RADAR_DETECTED_POINTS; i++)
    {
        float distance_m = detectedPoints[i][0];
        float snr        = detectedPoints[i][1];

        /* Apply filtering criteria for valid points */
        if (snr > RADAR_MIN_SNR_THRESHOLD && distance_m > RADAR_MIN_DISTANCE_M && distance_m < RADAR_MAX_DISTANCE_M)
        {
            /* Track the closest valid point */
            if (distance_m < closest_distance)
            {
                closest_distance = distance_m;
                found_valid      = true;
            }
        }
    }

    /* Store valid measurement result */
    if (found_valid)
    {
        measurement->distance_mm = (uint16_t)(closest_distance * 1000.0f);
        measurement->data_valid  = true;

        debug_send("S%d: Valid measurement: %d mm", sensor_idx, measurement->distance_mm);
    }
    else
    {
        debug_send("S%d: No valid points found", sensor_idx);
    }
}

/**
 * @brief Get the full measurement structure for a sensor
 *
 * @param sensor_idx Index of the sensor (0-2)
 * @return Pointer to the measurement structure, or NULL if sensor_idx is invalid
 */
radar_measurement_t *radar_get_measurement(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return NULL;
    }
    return &radar_measurements[sensor_idx];
}

/**
 * @brief Check if a sensor has valid measurement data
 *
 * @param sensor_idx Index of the sensor to check (0-2)
 * @return true if the sensor has valid data, false otherwise
 */
bool radar_has_valid_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }
    return radar_measurements[sensor_idx].data_valid;
}

/**
 * @brief Get the measured distance from a sensor
 *
 * @param sensor_idx Index of the sensor (0-2)
 * @return Distance in millimeters, or 0 if data is invalid
 */
uint16_t radar_get_distance_mm(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !radar_measurements[sensor_idx].data_valid)
    {
        return 0;
    }
    return radar_measurements[sensor_idx].distance_mm;
}

/**
 * @brief Get the angular position of a sensor
 *
 * @param sensor_idx Index of the sensor (0-2)
 * @return Angular position in degrees, or 0 if sensor_idx is invalid
 */
uint16_t radar_get_angle_deg(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return 0;
    }
    return sensor_angles[sensor_idx];
}

/**
 * @brief Set radar operational status
 *
 * @param status Radar system status (RADAR_INITIALISING, RADAR_READY, etc.)
 */
void radar_status_set(uint8_t status)
{
    radar_status = status;
}

/**
 * @brief Check if radar data is fresh (recently updated)
 *
 * @param sensor_idx Index of the sensor to check (0-2)
 * @return true if data is fresh (updated within timeout period)
 * @return false if data is stale or sensor index is invalid
 */
bool radar_data_is_fresh(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !radar_measurements[sensor_idx].data_valid)
    {
        return false;
    }

    uint32_t current_time = HAL_GetTick();
    uint32_t data_age     = current_time - radar_measurements[sensor_idx].timestamp_ms;

    return (data_age <= RADAR_DATA_TIMEOUT_MS);
}

/**
 * @brief Run comprehensive radar system diagnostics
 *
 * Performs several tests to verify radar system health and functionality
 */
void radar_diagnostics(void)
{
    debug_send("=== RADAR SYSTEM DIAGNOSTICS ===");

    // 1. Check overall system status
    debug_send("System Status: %s (%d)", radar_status_string(), radar_status);

    // 2. Test CAN connectivity
    test_sensor_responses();

    // 3. Check sensor data validity
    uint8_t valid_sensors = 0;
    uint8_t fresh_sensors = 0;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_has_valid_data(i))
        {
            valid_sensors++;
        }
        if (radar_data_is_fresh(i))
        {
            fresh_sensors++;
        }
    }

    debug_send("Valid sensors: %d/3, Fresh data: %d/3", valid_sensors, fresh_sensors);

    // 4. Calculate system health score
    uint8_t health = radar_system_health();
    debug_send("System health score: %d/100", health);

    // 5. Calculate overall data rate
    uint32_t        current_time    = HAL_GetTick();
    static uint32_t last_diag_time  = 0;
    static uint32_t last_total_msgs = 0;
    uint32_t        total_msgs      = 0;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        radar_data_t *sensor = &radar_system.sensors[i];
        total_msgs += sensor->frameNumber;
    }

    if (last_diag_time > 0)
    {
        float elapsed_sec  = (float)(current_time - last_diag_time) / 1000.0f;
        float msgs_per_sec = (float)(total_msgs - last_total_msgs) / elapsed_sec;
        debug_send("Current data rate: %.1f frames/sec total (%.1f per sensor)", msgs_per_sec, msgs_per_sec / MAX_RADAR_SENSORS);
    }

    last_diag_time  = current_time;
    last_total_msgs = total_msgs;

    debug_send("==========================");
}

/**
 * @brief Get radar status as human-readable string
 *
 * @return const char* Status description
 */
const char *radar_status_string(void)
{
    switch (radar_status)
    {
    case RADAR_INITIALISING:
        return "INITIALISING";
    case RADAR_READY:
        return "READY";
    case RADAR_CHIRPING:
        return "CHIRPING";
    case RADAR_STOPPED:
        return "STOPPED";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief Log radar data quality metrics
 *
 * Outputs the validity and freshness of radar data for each sensor
 */
void radar_log_quality_metrics(void)
{
    uint32_t        current_time     = HAL_GetTick();
    static uint32_t last_quality_log = 0;

    // Log once every 10 seconds
    if (current_time - last_quality_log < 10000)
    {
        return;
    }
    last_quality_log = current_time;

    debug_send("--- RADAR QUALITY METRICS ---");
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_measurements[i].data_valid)
        {
            debug_send("S%d: VALID data, distance: %dmm, age: %dms", i, radar_measurements[i].distance_mm, (current_time - radar_measurements[i].timestamp_ms));
        }
        else
        {
            debug_send("S%d: INVALID data, last update: %dms ago", i, (current_time - radar_measurements[i].timestamp_ms));
        }
    }
    debug_send("System status: %d", radar_status);
}
