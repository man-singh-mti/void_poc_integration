/**
 * @file    mti_radar.c
 * @brief   Radar sensor control and data processing implementation
 * @author  MTi Group
 * @copyright 2025 MTi Group
 *
 * This file implements the interface defined in mti_radar.h for controlling
 * radar sensors in a round-robin or staggered fashion and processing radar
 * measurements.
 */

#include "mti_radar.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_void.h"
#include <string.h>

/**
 * @brief Start a specific radar sensor in staggered mode
 *
 * @param sensor_idx Index of the sensor to start (0-2)
 */
static void radar_staggered_start_sensor(uint8_t sensor_idx);

/**
 * @brief Complete the current staggered radar cycle
 *
 * Stops all sensors and processes the collected data.
 */
static void radar_complete_staggered_cycle(void);

/** Global round-robin system state */
radar_round_robin_t radar_round_robin = { 0 };

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
 * @brief Initialize the radar system
 *
 * Sets up the round-robin state and prepares the system for operation.
 */
void radar_system_init(void)
{
    /* Initialize round-robin system */
    memset(&radar_round_robin, 0, sizeof(radar_round_robin));

    debug_send("Radar system init (staggered mode)");

    /* Start with staggered mode */
    radar_round_robin.system_running   = true;
    radar_round_robin.staggered_mode   = false; /* Will be set true on first cycle */
    radar_round_robin.last_switch_time = HAL_GetTick();
}

/**
 * @brief Process radar system operations
 *
 * Handles operation timing and initiates new radar cycles when appropriate.
 * Should be called regularly from the main program loop.
 */
void radar_system_process(void)
{
    if (!radar_round_robin.system_running)
    {
        return;
    }

    if (radar_round_robin.staggered_mode)
    {
        /* Process staggered cycle */
        radar_process_staggered_cycle();
    }
    else
    {
        /* Check if it's time to start a new cycle */
        uint32_t time_since_last = HAL_GetTick() - radar_round_robin.last_switch_time;

        if (time_since_last >= RADAR_STAGGERED_CYCLE_PAUSE_MS)
        {
            radar_start_staggered_cycle();
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
    can_send(CAN_CMD_BASE + sensor_idx, CAN_CMD_START);
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
    can_send(CAN_CMD_BASE + sensor_idx, CAN_CMD_STOP);
}

/**
 * @brief Process raw radar measurement data
 *
 * Analyzes detected points from a radar sensor and determines the closest valid
 * point to represent the borehole wall.
 *
 * @param sensor_idx      Index of the sensor providing the measurement (0-2)
 * @param detectedPoints  Array of points detected by radar [distance_m, SNR]
 * @param numPoints       Number of detected points in the array
 */
void radar_process_measurement(uint8_t sensor_idx, float detectedPoints[MAX_RADAR_DETECTED_POINTS][2], uint8_t numPoints)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    radar_measurement_t *measurement = &radar_round_robin.measurements[sensor_idx];

    /* Reset measurement */
    measurement->distance_mm = 0;
    measurement->data_valid  = false;
    measurement->angle_deg   = sensor_angles[sensor_idx];

    /* Find the CLOSEST valid point (closest to borehole wall) */
    float closest_distance = 999.0f;
    bool  found_valid      = false;

    for (uint8_t i = 0; i < numPoints && i < MAX_RADAR_DETECTED_POINTS; i++)
    {
        float distance_m = detectedPoints[i][0];
        float snr        = detectedPoints[i][1];

        if (snr > RADAR_MIN_SNR_THRESHOLD && distance_m > RADAR_MIN_DISTANCE_M && distance_m < RADAR_MAX_DISTANCE_M)
        {
            if (distance_m < closest_distance)
            {
                closest_distance = distance_m;
                found_valid      = true;
            }
        }
    }

    if (found_valid)
    {
        measurement->distance_mm = (uint16_t)(closest_distance * 1000.0f);
        measurement->data_valid  = true;
    }
}

/**
 * @brief Switch to the next sensor in round-robin mode
 *
 * Stops the current sensor and starts the next one in sequence.
 */
void radar_switch_to_next_sensor(void)
{
    /* Stop current sensor */
    if (radar_round_robin.current_sensor < MAX_RADAR_SENSORS)
    {
        radar_sensor_stop(radar_round_robin.current_sensor);
    }

    uint8_t previous_sensor = radar_round_robin.current_sensor;

    /* Move to next sensor */
    radar_round_robin.current_sensor++;
    if (radar_round_robin.current_sensor >= MAX_RADAR_SENSORS)
    {
        radar_round_robin.current_sensor = 0;
        /* Completed a full cycle - this could trigger additional processing if needed */
        debug_send("Radar cycle completed");
    }

    radar_sensor_start(radar_round_robin.current_sensor);
    radar_round_robin.last_switch_time = HAL_GetTick();
}

/**
 * @brief Start a new staggered radar cycle
 *
 * Initializes all state variables and starts the first sensor.
 */
void radar_start_staggered_cycle(void)
{
    uint32_t current_time = HAL_GetTick();

    /* Reset cycle state */
    radar_round_robin.cycle_start_time  = current_time;
    radar_round_robin.sensors_completed = 0;
    radar_round_robin.staggered_mode    = true;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        radar_round_robin.sensor_started[i]          = false;
        radar_round_robin.frame_complete[i]          = false;
        radar_round_robin.measurements[i].data_valid = false;
    }

    debug_send("Starting staggered radar cycle");

    /* Start first sensor immediately */
    radar_staggered_start_sensor(0);
}

/**
 * @brief Start a specific radar sensor in staggered mode
 *
 * @param sensor_idx Index of the sensor to start (0-2)
 */
static void radar_staggered_start_sensor(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || radar_round_robin.sensor_started[sensor_idx])
    {
        return;
    }

    radar_round_robin.sensor_started[sensor_idx] = true;
    can_send_to_sensor(sensor_idx, CAN_CMD_BASE, CAN_CMD_START);

    debug_send("Staggered start S%d at +%dms", sensor_idx, HAL_GetTick() - radar_round_robin.cycle_start_time);
}

/**
 * @brief Process the ongoing staggered cycle
 *
 * Starts additional sensors at appropriate intervals and monitors progress.
 */
void radar_process_staggered_cycle(void)
{
    if (!radar_round_robin.staggered_mode || !radar_round_robin.system_running)
    {
        return;
    }

    uint32_t elapsed_time = HAL_GetTick() - radar_round_robin.cycle_start_time;

    /* Start sensors at staggered intervals */
    if (elapsed_time >= RADAR_STAGGERED_START_INTERVAL_MS && !radar_round_robin.sensor_started[1])
    {
        radar_staggered_start_sensor(1);
    }

    if (elapsed_time >= (RADAR_STAGGERED_START_INTERVAL_MS * 2) && !radar_round_robin.sensor_started[2])
    {
        radar_staggered_start_sensor(2);
    }

    /* Check if all sensors have completed their frames */
    if (radar_round_robin.sensors_completed >= MAX_RADAR_SENSORS)
    {
        radar_complete_staggered_cycle();
    }

    /* Timeout fallback */
    if (elapsed_time >= RADAR_STAGGERED_TIMEOUT_MS)
    {
        debug_send("Staggered cycle timeout - completing with %d sensors", radar_round_robin.sensors_completed);
        radar_complete_staggered_cycle();
    }
}

/**
 * @brief Complete the current staggered radar cycle
 *
 * Stops all sensors and processes the collected data.
 */
static void radar_complete_staggered_cycle(void)
{
    uint32_t cycle_time = HAL_GetTick() - radar_round_robin.cycle_start_time;

    debug_send("Staggered cycle complete: %dms, %d/%d sensors", cycle_time, radar_round_robin.sensors_completed, MAX_RADAR_SENSORS);

    /* Stop all sensors */
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_round_robin.sensor_started[i])
        {
            radar_sensor_stop(i);
        }
    }

    /* Trigger void detection if we have sufficient data */
    if (radar_round_robin.sensors_completed >= 2)
    {
        debug_send("Triggering void analysis with %d sensors", radar_round_robin.sensors_completed);
        void_system_process();
    }
    else
    {
        debug_send("Insufficient sensor data for void analysis (%d sensors)", radar_round_robin.sensors_completed);
    }

    /* Reset staggered mode */
    radar_round_robin.staggered_mode = false;

    /* Schedule next cycle start after pause */
    HAL_Delay(RADAR_STAGGERED_CYCLE_PAUSE_MS);
    radar_start_staggered_cycle();
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
    return &radar_round_robin.measurements[sensor_idx];
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
    return radar_round_robin.measurements[sensor_idx].data_valid;
}

/**
 * @brief Get the measured distance from a sensor
 *
 * @param sensor_idx Index of the sensor (0-2)
 * @return Distance in millimeters, or 0 if data is invalid
 */
uint16_t radar_get_distance_mm(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return 0;
    }
    return radar_round_robin.measurements[sensor_idx].distance_mm;
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
