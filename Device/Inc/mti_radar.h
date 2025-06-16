/**
 * @file    mti_radar.h
 * @brief   Radar sensor control and data processing interface
 * @author  MTi Group
 * @copyright 2025 MTi Group
 *
 * This file provides the interface for controlling radar sensors in
 * continuous operation mode and processing the radar measurements.
 */

#ifndef MTI_RADAR_H
#define MTI_RADAR_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include <stdint.h>

/** @name Sensor Constants
 * Constants defining the number of sensors and data points
 * @{
 */
#ifndef MAX_RADAR_SENSORS
#define MAX_RADAR_SENSORS 3U /**< Maximum number of radar sensors in the system */
#endif

#ifndef MAX_RADAR_DETECTED_POINTS
#define MAX_RADAR_DETECTED_POINTS 20 /**< Maximum number of points detected by a radar sensor */
#endif
/** @} */

/** @name Sensor Angular Positions
 * Angular positions of sensors in degrees from reference
 * @{
 */
#define SENSOR_0_ANGLE 0   /**< Sensor 0 position: 0 degrees */
#define SENSOR_1_ANGLE 120 /**< Sensor 1 position: 120 degrees */
#define SENSOR_2_ANGLE 240 /**< Sensor 2 position: 240 degrees */
/** @} */

/** @name Radar Status Definitions
 * System-level radar operational status
 * @{
 */
#define RADAR_INITIALISING 0 /**< System is initializing */
#define RADAR_READY        1 /**< System is ready for operation */
#define RADAR_CHIRPING     2 /**< System is actively transmitting/receiving */
#define RADAR_STOPPED      3 /**< System operation is halted */
/** @} */

/** @name Radar Detection Parameters
 * Constants for radar signal detection and filtering
 * @{
 */
#define RADAR_MIN_SNR_THRESHOLD 100.0f /**< Minimum SNR for valid detection */
#define RADAR_MIN_DISTANCE_M    0.05f  /**< 5cm minimum valid distance */
#define RADAR_MAX_DISTANCE_M    5.0f   /**< 5m maximum valid distance */
#define RADAR_INVALID_DISTANCE  0xFFFF /**< Marker for invalid distance */
#define RADAR_DATA_TIMEOUT_MS   2000   /**< Data freshness timeout (2 seconds) */
/** @} */

/**
 * @brief Simple measurement storage structure for a single radar sensor
 *
 * Stores a single distance measurement from a radar sensor along with its
 * angular position and validity status.
 */
typedef struct
{
    uint16_t distance_mm;  /**< Single distance to borehole wall in millimeters */
    uint16_t angle_deg;    /**< Angular position of this sensor (0, 120, 240) */
    bool     data_valid;   /**< Flag indicating if the measurement is valid */
    uint32_t timestamp_ms; /**< Timestamp when measurement was taken */
} radar_measurement_t;

/** @name Radar System Management Functions
 * Functions to initialize and control the radar system
 * @{
 */

/**
 * @brief Initialize the radar system
 *
 * Sets up the system state and prepares the system for continuous operation.
 */
void radar_system_init(void);

/**
 * @brief Process radar system operations
 *
 * Handles operation timing and processes any new radar data.
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

/**
 * @brief Set radar operational status
 *
 * @param status Radar system status (RADAR_INITIALISING, RADAR_READY, etc.)
 */
void radar_status_set(uint8_t status);

/**
 * @brief Check if radar data is fresh (recently updated)
 *
 * @param sensor_idx Index of the sensor to check (0-2)
 * @return true if data is fresh (updated within timeout period)
 * @return false if data is stale or sensor index is invalid
 */
bool radar_data_is_fresh(uint8_t sensor_idx);

/** @} */

#endif /* MTI_RADAR_H */
