#include "mti_radar.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_void.h" // ADD THIS INCLUDE
#include <string.h>


static void radar_staggered_start_sensor(uint8_t sensor_idx);
static void radar_complete_staggered_cycle(void);

// Round-robin system state
radar_round_robin_t radar_round_robin = { 0 };

// Sensor angle lookup table
static const uint16_t sensor_angles[MAX_RADAR_SENSORS] = {
    SENSOR_0_ANGLE, // Sensor 0: 0 deg
    SENSOR_1_ANGLE, // Sensor 1: 120 deg
    SENSOR_2_ANGLE  // Sensor 2: 240 deg
};

void radar_system_init(void)
{
    // Initialize round-robin system
    memset(&radar_round_robin, 0, sizeof(radar_round_robin));

    debug_send("Radar system init (staggered mode)");

    // Start with staggered mode
    radar_round_robin.system_running   = true;
    radar_round_robin.staggered_mode   = false; // Will be set true on first cycle
    radar_round_robin.last_switch_time = HAL_GetTick();
}

void radar_system_process(void)
{
    if (!radar_round_robin.system_running)
    {
        return;
    }

    if (radar_round_robin.staggered_mode)
    {
        // Process staggered cycle
        radar_process_staggered_cycle();
    }
    else
    {
        // Check if it's time to start a new cycle
        uint32_t time_since_last = HAL_GetTick() - radar_round_robin.last_switch_time;

        if (time_since_last >= RADAR_STAGGERED_CYCLE_PAUSE_MS)
        {
            radar_start_staggered_cycle();
        }
    }
}

void radar_sensor_start(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    // Remove this debug message - silent operation
    // debug_send("Starting radar sensor %d (angle %d deg)", sensor_idx, sensor_angles[sensor_idx]);

    // Send start command to specific sensor
    can_send(CAN_CMD_BASE + sensor_idx, CAN_CMD_START);
}

void radar_sensor_stop(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    // Send stop command to specific sensor
    can_send(CAN_CMD_BASE + sensor_idx, CAN_CMD_STOP);
}

void radar_process_measurement(uint8_t sensor_idx, float detectedPoints[MAX_RADAR_DETECTED_POINTS][2], uint8_t numPoints)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    radar_measurement_t *measurement = &radar_round_robin.measurements[sensor_idx];

    // Reset measurement
    measurement->distance_mm = 0;
    measurement->data_valid  = false;
    measurement->angle_deg   = sensor_angles[sensor_idx];

    // Find the CLOSEST valid point (closest to borehole wall)
    float closest_distance = 999.0f;
    bool  found_valid      = false;

    for (uint8_t i = 0; i < numPoints && i < MAX_RADAR_DETECTED_POINTS; i++)
    {
        float distance_m = detectedPoints[i][0];
        float snr        = detectedPoints[i][1];

        if (snr > 100.0f && distance_m > 0.05f && distance_m < 5.0f)
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

void radar_switch_to_next_sensor(void)
{
    // Stop current sensor
    if (radar_round_robin.current_sensor < MAX_RADAR_SENSORS)
    {
        radar_sensor_stop(radar_round_robin.current_sensor);
    }

    uint8_t previous_sensor = radar_round_robin.current_sensor;

    // Move to next sensor
    radar_round_robin.current_sensor++;
    if (radar_round_robin.current_sensor >= MAX_RADAR_SENSORS)
    {
        radar_round_robin.current_sensor = 0;
        // Completed a full cycle - this could trigger additional processing if needed
        debug_send("Radar cycle completed");
    }

    radar_sensor_start(radar_round_robin.current_sensor);
    radar_round_robin.last_switch_time = HAL_GetTick();
}

// Add these new functions after radar_switch_to_next_sensor()

void radar_start_staggered_cycle(void)
{
    uint32_t current_time = HAL_GetTick();

    // Reset cycle state
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

    // Start first sensor immediately
    radar_staggered_start_sensor(0);
}

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

void radar_process_staggered_cycle(void)
{
    if (!radar_round_robin.staggered_mode || !radar_round_robin.system_running)
    {
        return;
    }

    uint32_t elapsed_time = HAL_GetTick() - radar_round_robin.cycle_start_time;

    // Start sensors at staggered intervals
    if (elapsed_time >= 20 && !radar_round_robin.sensor_started[1])
    {
        radar_staggered_start_sensor(1);
    }

    if (elapsed_time >= 40 && !radar_round_robin.sensor_started[2])
    {
        radar_staggered_start_sensor(2);
    }

    // Check if all sensors have completed their frames
    if (radar_round_robin.sensors_completed >= MAX_RADAR_SENSORS)
    {
        radar_complete_staggered_cycle();
    }

    // Timeout fallback
    if (elapsed_time >= RADAR_STAGGERED_TIMEOUT_MS)
    {
        debug_send("Staggered cycle timeout - completing with %d sensors", radar_round_robin.sensors_completed);
        radar_complete_staggered_cycle();
    }
}

static void radar_complete_staggered_cycle(void)
{
    uint32_t cycle_time = HAL_GetTick() - radar_round_robin.cycle_start_time;

    debug_send("Staggered cycle complete: %dms, %d/%d sensors", cycle_time, radar_round_robin.sensors_completed, MAX_RADAR_SENSORS);

    // Stop all sensors
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_round_robin.sensor_started[i])
        {
            radar_sensor_stop(i);
        }
    }

    // Trigger void detection if we have sufficient data
    if (radar_round_robin.sensors_completed >= 2)
    {
        debug_send("Triggering void analysis with %d sensors", radar_round_robin.sensors_completed);

        // --- MODIFICATION START ---
        // Replace the non-existent function with the correct one.
        void_system_process(); // CHANGED: Use actual function name
        // --- MODIFICATION END ---
    }
    else
    {
        debug_send("Insufficient sensor data for void analysis (%d sensors)", radar_round_robin.sensors_completed);
    }

    // Reset staggered mode
    radar_round_robin.staggered_mode = false;

    // Schedule next cycle start after pause
    HAL_Delay(RADAR_STAGGERED_CYCLE_PAUSE_MS);
    radar_start_staggered_cycle();
}

// Simplified getter functions
radar_measurement_t *radar_get_measurement(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return NULL;
    }
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

uint16_t radar_get_distance_mm(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return 0;
    }
    return radar_round_robin.measurements[sensor_idx].distance_mm;
}

uint16_t radar_get_angle_deg(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return 0;
    }
    return sensor_angles[sensor_idx];
}
