#ifndef MTI_RADAR_H
#define MTI_RADAR_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include <stdint.h>

// Constants (define here to avoid circular dependency)
#ifndef MAX_RADAR_SENSORS
#define MAX_RADAR_SENSORS 3U
#endif

#ifndef MAX_RADAR_DETECTED_POINTS
#define MAX_RADAR_DETECTED_POINTS 20
#endif

// Radar processing constants
#define RADAR_CYCLE_TIME_MS 100 // 100ms maximum per sensor cycle (fallback timeout)
#define RADAR_MIN_CYCLE_MS  50  // 50ms minimum per sensor cycle

// Sensor angular positions (degrees from reference)
#define SENSOR_0_ANGLE 0   // 0 degrees
#define SENSOR_1_ANGLE 120 // 120 degrees
#define SENSOR_2_ANGLE 240 // 240 degrees

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
    bool                system_running;                  // Is the system actively measuring
    uint8_t             current_sensor;                  // Currently active sensor (0-2)
    uint32_t            last_switch_time;                // Timestamp of last sensor switch
    radar_measurement_t measurements[MAX_RADAR_SENSORS]; // Measurement storage for each sensor

    // NEW: Staggered operation fields
    bool     staggered_mode;                    // Are we using staggered start mode
    uint32_t cycle_start_time;                  // When current cycle started
    bool     sensor_started[MAX_RADAR_SENSORS]; // Track which sensors have been started
    bool     frame_complete[MAX_RADAR_SENSORS]; // Track which sensors have completed frames
    uint8_t  sensors_completed;                 // Count of completed sensors in current cycle
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
bool                 radar_has_valid_data(uint8_t sensor_idx);
uint16_t             radar_get_distance_mm(uint8_t sensor_idx);
uint16_t             radar_get_angle_deg(uint8_t sensor_idx);
radar_measurement_t *radar_get_measurement(uint8_t sensor_idx);

// Add missing constants
#define RADAR_MIN_SNR_THRESHOLD 100.0f // Minimum SNR for valid detection
#define RADAR_MIN_DISTANCE_M    0.05f  // 5cm minimum distance
#define RADAR_MAX_DISTANCE_M    5.0f   // 5m maximum distance
#define RADAR_INVALID_DISTANCE  0xFFFF // Invalid distance marker

// Add these constants after existing defines
#define RADAR_STAGGERED_START_INTERVAL_MS 20  // 20ms between sensor starts
#define RADAR_STAGGERED_TIMEOUT_MS        150 // Maximum time for staggered cycle
#define RADAR_STAGGERED_CYCLE_PAUSE_MS    50  // Pause between cycles

// External access to round-robin state
extern radar_round_robin_t radar_round_robin;

// Staggered operation functions
void radar_start_staggered_cycle(void);
void radar_process_staggered_cycle(void);
void radar_complete_staggered_cycle(void);

#endif // MTI_RADAR_H
