#ifndef MTI_RADAR_H
#define MTI_RADAR_H

#include "stm32f7xx.h"
#include "mti_can.h"
#include <stdbool.h>
#include <stdint.h>

// Radar processing constants
#define RADAR_CYCLE_TIME_MS 100 // 100ms maximum per sensor cycle (fallback timeout)
#define RADAR_MIN_CYCLE_MS  50  // 50ms minimum per sensor cycle

// Sensor angular positions (degrees from reference)
#define SENSOR_0_ANGLE 0   // 0 degrees
#define SENSOR_1_ANGLE 120 // 120 degrees
#define SENSOR_2_ANGLE 240 // 240 degrees
// Alternative 4-sensor setup: 0, 90, 180, 270

// Simple measurement storage - ONE distance per sensor
typedef struct
{
    uint16_t distance_mm; // Single distance to borehole wall in millimeters
    uint16_t angle_deg;   // Angular position of this sensor (0, 120, 240)
    bool     data_valid;  // Is this measurement valid
} radar_measurement_t;

// System state for round-robin operation
typedef struct
{
    uint8_t             current_sensor;                  // Which sensor is active (0-2)
    uint32_t            last_switch_time;                // When we last switched sensors
    radar_measurement_t measurements[MAX_RADAR_SENSORS]; // Latest measurements from each sensor
    bool                system_running;                  // Is the system actively measuring
} radar_round_robin_t;

// Radar system management functions (implemented in mti_radar.c)
void radar_system_init(void);
void radar_system_process(void);
void radar_sensor_start(uint8_t sensor_idx);
void radar_sensor_stop(uint8_t sensor_idx);
void radar_switch_to_next_sensor(void);

// Radar data processing functions (implemented in mti_radar.c)
void radar_process_measurement(uint8_t sensor_idx, float detectedPoints[MAX_RADAR_DETECTED_POINTS][2], uint8_t numPoints);

// Getter functions for measurements (implemented in mti_radar.c)
radar_measurement_t *radar_get_measurement(uint8_t sensor_idx);
bool                 radar_has_valid_data(uint8_t sensor_idx);
uint16_t             radar_get_distance(uint8_t sensor_idx);
uint16_t             radar_get_angle(uint8_t sensor_idx);

// External access to round-robin state
extern radar_round_robin_t radar_round_robin;

#endif // MTI_RADAR_H
