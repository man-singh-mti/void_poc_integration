#include "mti_radar.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include <string.h>

// Round-robin system state
radar_round_robin_t radar_round_robin = { 0 };

// Sensor angle lookup table
static const uint16_t sensor_angles[MAX_RADAR_SENSORS] = {
    SENSOR_0_ANGLE, // Sensor 0: 0°
    SENSOR_1_ANGLE, // Sensor 1: 120°
    SENSOR_2_ANGLE  // Sensor 2: 240°
};

void radar_system_init(void)
{
    // Initialize round-robin system
    radar_round_robin.current_sensor   = 0;
    radar_round_robin.last_switch_time = HAL_GetTick();
    radar_round_robin.system_running   = false;

    // Initialize measurements with sensor angles
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        radar_round_robin.measurements[i].distance_mm = 0;
        radar_round_robin.measurements[i].angle_deg   = sensor_angles[i];
        radar_round_robin.measurements[i].data_valid  = false;
    }

    debug_send("Radar system initialized - 3 sensors at 0°, 120°, 240°");

    // Start the first sensor
    radar_round_robin.system_running = true;
    radar_sensor_start(0);
}

void radar_system_process(void)
{
    if (!radar_round_robin.system_running)
    {
        return;
    }

    uint32_t current_time      = HAL_GetTick();
    uint32_t time_since_switch = current_time - radar_round_robin.last_switch_time;

    // Timeout fallback: switch if we've been on one sensor too long
    if (time_since_switch >= RADAR_CYCLE_TIME_MS)
    {
        debug_send("Sensor %d timeout - forcing switch", radar_round_robin.current_sensor);
        radar_switch_to_next_sensor();
    }
}

void radar_sensor_start(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
        return;

    debug_send("Starting radar sensor %d (angle %d°)", sensor_idx, sensor_angles[sensor_idx]);

    // Send start command to specific sensor
    can_send(CAN_CMD_BASE + sensor_idx, CAN_CMD_START);
}

void radar_sensor_stop(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
        return;

    // Send stop command to specific sensor
    can_send(CAN_CMD_BASE + sensor_idx, CAN_CMD_STOP);
}

void radar_process_measurement(uint8_t sensor_idx, float detectedPoints[MAX_RADAR_DETECTED_POINTS][2], uint8_t numPoints)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
        return;

    radar_measurement_t *measurement = &radar_round_robin.measurements[sensor_idx];

    debug_send("Processing %d points from sensor %d (%d°)", numPoints, sensor_idx, sensor_angles[sensor_idx]);

    // Reset measurement
    measurement->distance_mm = 0;
    measurement->data_valid  = false;

    // Find the CLOSEST valid point (closest to borehole wall)
    float closest_distance = 999.0f; // Start with large value
    bool  found_valid      = false;

    for (uint8_t i = 0; i < numPoints && i < MAX_RADAR_DETECTED_POINTS; i++)
    {
        float distance_m = detectedPoints[i][0];
        float snr        = detectedPoints[i][1];

        debug_send("S%d Point %d: %.3fm, SNR=%.1f", sensor_idx, i, distance_m, snr);

        // Simple filtering: minimum SNR and reasonable distance for borehole measurement
        if (snr > 100.0f && distance_m > 0.05f && distance_m < 5.0f) // 5cm to 5m range
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
        // Convert to millimeters and store single distance
        measurement->distance_mm = (uint16_t)(closest_distance * 1000.0f);
        measurement->data_valid  = true;

        debug_send("S%d (%d°): Wall distance = %u mm", sensor_idx, sensor_angles[sensor_idx], measurement->distance_mm);
    }
    else
    {
        debug_send("S%d (%d°): No valid wall measurement", sensor_idx, sensor_angles[sensor_idx]);
    }

    // IMMEDIATE SWITCH: Move to next sensor after processing frame
    uint32_t time_since_switch = HAL_GetTick() - radar_round_robin.last_switch_time;

    if (time_since_switch >= RADAR_MIN_CYCLE_MS)
    {
        radar_switch_to_next_sensor();
    }
}

void radar_switch_to_next_sensor(void)
{
    // Stop current sensor
    radar_sensor_stop(radar_round_robin.current_sensor);

    // Move to next sensor (round-robin)
    radar_round_robin.current_sensor   = (radar_round_robin.current_sensor + 1) % MAX_RADAR_SENSORS;
    radar_round_robin.last_switch_time = HAL_GetTick();

    debug_send("Switching to sensor %d (%d°)", radar_round_robin.current_sensor, sensor_angles[radar_round_robin.current_sensor]);

    // Start next sensor
    radar_sensor_start(radar_round_robin.current_sensor);
}

// Simplified getter functions
radar_measurement_t *radar_get_measurement(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
        return NULL;
    return &radar_round_robin.measurements[sensor_idx];
}

bool radar_has_valid_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }
    return radar_round_robin.measurements[sensor_idx].data_valid;
}

uint16_t radar_get_distance(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !radar_has_valid_data(sensor_idx))
        return 0;
    return radar_round_robin.measurements[sensor_idx].distance_mm;
}

uint16_t radar_get_angle(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
        return 0;
    return radar_round_robin.measurements[sensor_idx].angle_deg;
}
