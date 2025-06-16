/**
 * @file    mti_radar.h
 * @brief   Radar sensor control and data processing interface
 * @author  MTi Group
 * @copyright 2025 MTi Group
 *
 * This file provides the interface for controlling radar sensors in a
 * round-robin or staggered fashion and processing the radar measurements.
 */

#ifndef MTI_RADAR_H
#define MTI_RADAR_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include <stdint.h>

/** @name Sensor Constants
 * Constants defining the number of sensors and data points (defined here to avoid circular dependency)
 * @{
 */
#ifndef MAX_RADAR_SENSORS
#define MAX_RADAR_SENSORS 3U /**< Maximum number of radar sensors in the system */
#endif

#ifndef MAX_RADAR_DETECTED_POINTS
#define MAX_RADAR_DETECTED_POINTS 20 /**< Maximum number of points detected by a radar sensor */
#endif
/** @} */

/** @name Radar Processing Constants
 * Constants defining radar processing timing parameters
 * @{
 */
#define RADAR_CYCLE_TIME_MS 100 /**< 100ms maximum per sensor cycle (fallback timeout) */
#define RADAR_MIN_CYCLE_MS  50  /**< 50ms minimum per sensor cycle */

/** @} */

/** @name Sensor Angular Positions
 * Angular positions of sensors in degrees from reference
 * @{
 */
#define SENSOR_0_ANGLE 0   /**< Sensor 0 position: 0 degrees */
#define SENSOR_1_ANGLE 120 /**< Sensor 1 position: 120 degrees */
#define SENSOR_2_ANGLE 240 /**< Sensor 2 position: 240 degrees */
/** @} */

/**
 * @brief Simple measurement storage structure for a single radar sensor
 *
 * Stores a single distance measurement from a radar sensor along with its
 * angular position and validity status.
 */
typedef struct
{
    uint16_t distance_mm; /**< Single distance to borehole wall in millimeters */
    uint16_t angle_deg;   /**< Angular position of this sensor (0, 120, 240) */
    bool     data_valid;  /**< Flag indicating if the measurement is valid */
} radar_measurement_t;

/**
 * @brief System state for round-robin and staggered radar sensor operation
 *
 * Contains all state information needed to manage the radar sensors in
 * both round-robin and staggered operation modes.
 */
typedef struct
{
    bool                system_running;                  /**< Flag indicating if the system is actively measuring */
    uint8_t             current_sensor;                  /**< Currently active sensor (0-2) */
    uint32_t            last_switch_time;                /**< Timestamp of last sensor switch */
    radar_measurement_t measurements[MAX_RADAR_SENSORS]; /**< Measurement storage for each sensor */

    /* Staggered operation fields */
    bool     staggered_mode;                    /**< Flag indicating if staggered start mode is active */
    uint32_t cycle_start_time;                  /**< Timestamp when current cycle started */
    bool     sensor_started[MAX_RADAR_SENSORS]; /**< Tracks which sensors have been started */
    bool     frame_complete[MAX_RADAR_SENSORS]; /**< Tracks which sensors have completed frames */
    uint8_t  sensors_completed;                 /**< Count of sensors that completed in current cycle */
} radar_round_robin_t;

/** @name Radar System Management Functions
 * Functions to initialize and control the radar system
 * @{
 */

/**
 * @brief Initialize the radar system
 *
 * Sets up the round-robin state and prepares the system for operation in staggered mode.
 */
void radar_system_init(void);

/**
 * @brief Process radar system operations
 *
 * Handles operation timing and initiates new radar cycles when appropriate.
 * Should be called regularly from the main program loop.
 */
void radar_system_process(void);

/**
 * @brief Start a specific radar sensor
 *
 * @param sensor_idx Index of the sensor to start (0-2)
 */
void radar_sensor_start(uint8_t sensor_idx);

/**
 * @brief Stop a specific radar sensor
 *
 * @param sensor_idx Index of the sensor to stop (0-2)
 */
void radar_sensor_stop(uint8_t sensor_idx);

/**
 * @brief Switch to the next sensor in round-robin mode
 *
 * Stops the current sensor and starts the next one in sequence.
 */
void radar_switch_to_next_sensor(void);
/** @} */

/** @name Radar Data Processing Functions
 * Functions to process radar sensor measurements
 * @{
 */

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
void radar_process_measurement(uint8_t sensor_idx, float detectedPoints[MAX_RADAR_DETECTED_POINTS][2], uint8_t numPoints);
/** @} */

/** @name Measurement Access Functions
 * Functions to retrieve radar measurement data
 * @{
 */

/**
 * @brief Check if a sensor has valid measurement data
 *
 * @param sensor_idx Index of the sensor to check (0-2)
 * @return true if the sensor has valid data, false otherwise
 */
bool radar_has_valid_data(uint8_t sensor_idx);

/**
 * @brief Get the measured distance from a sensor
 *
 * @param sensor_idx Index of the sensor (0-2)
 * @return Distance in millimeters, or 0 if data is invalid
 */
uint16_t radar_get_distance_mm(uint8_t sensor_idx);

/**
 * @brief Get the angular position of a sensor
 *
 * @param sensor_idx Index of the sensor (0-2)
 * @return Angular position in degrees, or 0 if sensor_idx is invalid
 */
uint16_t radar_get_angle_deg(uint8_t sensor_idx);

/**
 * @brief Get the full measurement structure for a sensor
 *
 * @param sensor_idx Index of the sensor (0-2)
 * @return Pointer to the measurement structure, or NULL if sensor_idx is invalid
 */
radar_measurement_t *radar_get_measurement(uint8_t sensor_idx);
/** @} */

/** @name Radar Detection Parameters
 * Constants for radar signal detection and filtering
 * @{
 */
#define RADAR_MIN_SNR_THRESHOLD 100.0f /**< Minimum SNR for valid detection */
#define RADAR_MIN_DISTANCE_M    0.05f  /**< 5cm minimum valid distance */
#define RADAR_MAX_DISTANCE_M    5.0f   /**< 5m maximum valid distance */
#define RADAR_INVALID_DISTANCE  0xFFFF /**< Marker for invalid distance */
/** @} */

/** @name Staggered Operation Parameters
 * Constants for the staggered sensor operation mode
 * @{
 */
#define RADAR_STAGGERED_START_INTERVAL_MS 20  /**< 20ms interval between sensor starts */
#define RADAR_STAGGERED_TIMEOUT_MS        150 /**< Maximum time for staggered cycle */
#define RADAR_STAGGERED_CYCLE_PAUSE_MS    50  /**< Pause between cycles */
/** @} */

/** @name External State Access
 * External variables for system state access
 * @{
 */
extern radar_round_robin_t radar_round_robin; /**< Round-robin system state */
/** @} */

/** @name Staggered Operation Functions
 * Functions for controlling sensors in staggered mode
 * @{
 */

/**
 * @brief Start a new staggered radar cycle
 *
 * Initializes all state variables and starts the first sensor.
 */
void radar_start_staggered_cycle(void);

/**
 * @brief Process the ongoing staggered cycle
 *
 * Starts additional sensors at appropriate intervals and monitors progress.
 */
void radar_process_staggered_cycle(void);

/**
 * @brief Complete the current staggered radar cycle (implemented as static)
 *
 * @note This function is declared in the header but implemented as static in the source file.
 * It should not be called directly from outside the module.
 * @deprecated Use radar_process_staggered_cycle() instead which will handle cycle completion.
 */
void radar_complete_staggered_cycle(void);
/** @} */

/**
 * @brief Process any completed radar measurements
 *
 * Processes data from any sensors that have new data ready,
 * for continuous operation mode.
 */
void radar_process_completed_measurements(void);

#endif /* MTI_RADAR_H */
