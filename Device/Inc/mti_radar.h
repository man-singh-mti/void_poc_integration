/**
 * @file mti_radar.h
 * @brief Radar system interface - Data processing and sensor management
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#ifndef MTI_RADAR_H
#define MTI_RADAR_H

#include "mti_radar_types.h"
#include "mti_can.h"
#include <stdbool.h>
#include <stdint.h>

/** @name System Constants */
#define RADAR_PROCESSING_INTERVAL_MS 100    // Default 10Hz processing
#define RADAR_SENSOR_TIMEOUT_MS      5000   // 5 second sensor timeout
#define RADAR_MIN_VALID_DISTANCE_MM  50     // Minimum valid distance (50mm)
#define RADAR_MAX_VALID_DISTANCE_MM  5000   // Maximum valid distance (5m)
#define RADAR_MIN_SNR_THRESHOLD      100.0f // Minimum SNR for valid reading (was 20.0f)

/** @name Expected Firmware Version */
#define RADAR_EXPECTED_FW_MAJOR 1
#define RADAR_EXPECTED_FW_MINOR 3
#define RADAR_EXPECTED_FW_PATCH 0

/** @name Sensor Profiles */
#define RADAR_PROFILE_CALIBRATION 0 // Calibration profile
#define RADAR_PROFILE_MEASUREMENT 1 // Standard measurement profile

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Defines the radar initialization status.
 */
typedef enum
{
    RADAR_INIT_NOT_STARTED, ///< Radar initialization has not started.
    RADAR_INIT_IN_PROGRESS, ///< Radar initialization is in progress.
    RADAR_INIT_OK,          ///< Radar initialization completed successfully.
    RADAR_INIT_ERROR        ///< Radar initialization failed.
} radar_init_status_t;


/** @name Processed Measurement Data */
typedef struct
{
    uint16_t distance_mm[MAX_RADAR_SENSORS];   // Clean distances in millimeters
    uint16_t angle_deg[MAX_RADAR_SENSORS];     // Sensor angles (0,120,240)
    uint16_t snr_value[MAX_RADAR_SENSORS];     // SNR values as integers
    bool     data_valid[MAX_RADAR_SENSORS];    // Validity flags per sensor
    float    confidence[MAX_RADAR_SENSORS];    // Confidence per sensor (0.0-1.0)
    uint8_t  quality_score[MAX_RADAR_SENSORS]; // Signal quality (0-100)
    uint8_t  valid_sensor_count;               // Number of sensors with valid data
    uint32_t timestamp_ms;                     // When processing completed
    bool     system_healthy;                   // Overall system health
} radar_distance_t;

/** @name System Initialization and Management */

/**
 * @brief Initialize radar system
 *
 * Performs complete initialization sequence:
 * - Verifies sensor firmware versions
 * - Sets up data processing parameters
 * - Configures sensors for calibration mode
 * - Reports configuration to debug and uphole
 *
 * @return true if initialization successful
 */
bool radar_system_init(void);

/**
 * @brief Main radar processing function
 *
 * Processes raw CAN data into clean measurements.
 * Call from main loop at regular intervals.
 */
void radar_system_process(void);

/**
 * @brief Shutdown radar system gracefully
 */
void radar_system_shutdown(void);

/** @name Data Processing Functions */

/**
 * @brief Get latest processed measurements
 *
 * @param measurements Pointer to structure to fill with latest data
 * @return true if new data is available
 */
bool radar_get_latest_measurements(radar_distance_t *measurements);

/**
 * @brief Check if new processed data is available
 *
 * @return true if new data ready for void detection layer
 */
bool radar_has_new_data(void);

/**
 * @brief Mark processed data as consumed
 */
void radar_mark_data_processed(void);

/** @name Sensor Management Functions */

/**
 * @brief Set sensors to calibration mode
 *
 * Sends CAN_CMD_SELECT_PROFILE with value 0 to all sensors
 *
 * @return true if command sent successfully
 */
bool radar_set_calibration_mode(void);

/**
 * @brief Set sensors to measurement mode
 *
 * Sends CAN_CMD_SELECT_PROFILE with value 1 to all sensors
 *
 * @return true if command sent successfully
 */
bool radar_set_measurement_mode(void);

/**
 * @brief Start sensor operation
 *
 * @return true if sensors started successfully
 */
bool radar_start_sensors(void);

/**
 * @brief Stop sensor operation
 *
 * @return true if sensors stopped successfully
 */
bool radar_stop_sensors(void);

/** @name System Status and Diagnostics */

/**
 * @brief Check if radar system is healthy
 *
 * @return true if system operational (>=2 sensors with valid data)
 */
bool radar_is_system_healthy(void);

/**
 * @brief Get number of sensors with valid data
 *
 * @return Number of sensors (0-3) with valid measurements
 */
uint8_t radar_get_valid_sensor_count(void);

/**
 * @brief Get number of online sensors from CAN layer
 *
 * @return Number of active sensors
 */
uint8_t radar_get_active_sensor_count(void);

/**
 * @brief Run comprehensive system diagnostics
 *
 * Reports to debug output:
 * - System health status
 * - Sensor firmware versions
 * - Data processing parameters
 * - Current measurements
 */
void radar_run_diagnostics(void);

/**
 * @brief Report system configuration
 *
 * Reports current configuration to specified UART channel
 *
 * @param channel UART channel for output
 */
void radar_report_configuration(uart_select_t channel);

/** @name Configuration Functions */

/**
 * @brief Set minimum SNR threshold
 *
 * @param snr_threshold Minimum SNR for valid readings
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
 * @brief Set processing rate
 *
 * @param rate_hz Processing rate in Hz (1-50 Hz)
 */
void radar_set_processing_rate(uint8_t rate_hz);

/** @name Firmware Verification */

/**
 * @brief Verify all sensor firmware versions
 *
 * @return true if all online sensors have expected firmware
 */
bool radar_verify_firmware_versions(void);

/**
 * @brief Get sensor firmware version
 *
 * @param sensor_idx Sensor index (0-2)
 * @param major Pointer to store major version
 * @param minor Pointer to store minor version
 * @param patch Pointer to store patch version
 * @return true if sensor is online and firmware data available
 */
bool radar_get_sensor_firmware(uint8_t sensor_idx, uint8_t *major, uint8_t *minor, uint8_t *patch);

/** @name Test & Debug Functions */

/**
 * @brief Periodic radar measurements debug output
 *
 * Runs every 2 seconds, starts sensors on first run and outputs
 * processed measurement data for testing without synchronization
 */
void radar_debug_measurements_periodic(void);

/** @name Event-Driven Processing */

/**
 * @brief Check if radar has new processed data ready for void layer
 * @return true if new processed data is available
 */
bool radar_has_new_processed_data(void);

/**
 * @brief Get event processing statistics
 * @param event_count Pointer to store number of events processed
 * @param last_event_time Pointer to store timestamp of last event
 */
void radar_get_event_stats(uint32_t *event_count, uint32_t *last_event_time);

#endif // MTI_RADAR_H
