/**
 * @file mti_radar.h
 * @brief Radar data processing layer interface
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#ifndef MTI_RADAR_H
#define MTI_RADAR_H

#include "mti_radar_types.h"
#include <stdbool.h>
#include <stdint.h>

/** @name System Constants */
#define RADAR_PROCESSING_INTERVAL_MS 100   // Default 10Hz - configurable
#define RADAR_SENSOR_TIMEOUT_MS      2000  // 2 second sensor timeout
#define RADAR_MIN_VALID_DISTANCE_MM  50    // Minimum valid distance (50mm)
#define RADAR_MAX_VALID_DISTANCE_MM  5000  // Maximum valid distance (5m)
#define RADAR_MIN_SNR_THRESHOLD      10.0f // Minimum SNR for valid reading

/** @name Radar System Initialization */

/**
 * @brief Initialize radar processing system
 *
 * Sets up data structures and processing parameters
 *
 * @return true if initialization successful
 */
bool radar_system_init(void);

/**
 * @brief Main radar processing function
 *
 * Call this from main loop - processes new data from CAN layer
 * and produces clean radar_distance_t output
 */
void radar_system_process(void);

/**
 * @brief Shutdown radar system gracefully
 *
 * Stops sensors and cleans up resources
 */
void radar_system_shutdown(void);

/** @name Data Access Functions */

/**
 * @brief Get latest processed radar measurements
 *
 * @param measurements Pointer to structure to fill with latest data
 * @return true if new data is available
 */
bool radar_get_latest_measurements(radar_distance_t *measurements);

/**
 * @brief Check if radar system has new processed data
 *
 * @return true if new data is available for void detection layer
 */
bool radar_has_new_data(void);

/**
 * @brief Mark processed data as consumed by void detection layer
 */
void radar_mark_data_processed(void);

/**
 * @brief Get individual sensor measurement
 *
 * @param sensor_idx Sensor index (0-2)
 * @param distance_mm Pointer to store distance in millimeters
 * @param valid Pointer to store validity flag
 * @return true if sensor data is available
 */
bool radar_get_sensor_measurement(uint8_t sensor_idx, uint16_t *distance_mm, bool *valid);

/** @name System Status Functions */

/**
 * @brief Check if radar system is healthy and operational
 *
 * @return true if system is healthy (>=2 sensors online with valid data)
 */
bool radar_is_system_healthy(void);

/**
 * @brief Get number of sensors with valid current data
 *
 * @return Number of sensors (0-3) with valid measurements
 */
uint8_t radar_get_valid_sensor_count(void);

/**
 * @brief Run radar system diagnostics
 */
void radar_run_diagnostics(void);

/** @name Configuration Functions */

/**
 * @brief Set minimum SNR threshold for valid readings
 *
 * @param snr_threshold Minimum SNR value
 */
void radar_set_snr_threshold(float snr_threshold);

/**
 * @brief Set valid distance range
 *
 * @param min_mm Minimum valid distance in millimeters
 * @param max_mm Maximum valid distance in millimeters
 */
void radar_set_distance_range(uint16_t min_mm, uint16_t max_mm);

/**
 * @brief Set radar processing rate
 *
 * @param rate_hz Processing rate in Hz (1-50 Hz supported)
 */
void radar_set_processing_rate(uint8_t rate_hz);

/**
 * @brief Get current processing rate
 *
 * @return Current processing rate in Hz
 */
uint8_t radar_get_processing_rate(void);

/** @name Internal Functions (used by CAN layer) */

/**
 * @brief Notification from CAN layer that new raw data is available
 *
 * Called by process_complete_radar_frame() in mti_can.c
 *
 * @param sensor_idx Sensor index with new data
 */
void radar_notify_new_raw_data(uint8_t sensor_idx);

/** @name Sensor Control Functions */

/**
 * @brief Start all radar sensors
 *
 * Sends start commands and configures sensors for operation
 *
 * @return true if sensors started successfully
 */
bool radar_start_sensors(void);

/**
 * @brief Stop all radar sensors
 *
 * Sends stop commands to save power when not needed
 *
 * @return true if sensors stopped successfully
 */
bool radar_stop_sensors(void);

/**
 * @brief Configure sensor settings
 *
 * @param profile Radar profile (0=cal, 1=50m_single, 2=50m_multi, 3=100m_single, 4=100m_multi)
 * @param threshold Detection threshold
 * @return true if configuration successful
 */
bool radar_configure_sensors(uint8_t profile, uint8_t threshold);

/**
 * @brief Get sensor operational status
 *
 * @return true if sensors are started and operational
 */
bool radar_sensors_are_running(void);

/** @name Compatibility Functions (for existing integration) */

/**
 * @brief Check if specific sensor has valid data (compatibility function)
 *
 * @param sensor_idx Sensor index
 * @return true if sensor has valid current data
 */
bool radar_has_valid_data(uint8_t sensor_idx);

/**
 * @brief Get distance measurement for specific sensor (compatibility function)
 *
 * @param sensor_idx Sensor index
 * @return Distance in millimeters, or 0 if invalid
 */
uint16_t radar_get_distance_mm(uint8_t sensor_idx);

/**
 * @brief Get number of active sensors
 *
 * @return Number of active sensors
 */
uint8_t radar_get_active_sensor_count(void);

#endif // MTI_RADAR_H
