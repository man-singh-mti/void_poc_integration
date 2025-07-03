/**
 * @file mti_void.h
 * @brief Void detection system interface
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#ifndef MTI_VOID_H
#define MTI_VOID_H

#include "mti_radar.h"
#include <stdbool.h>
#include <stdint.h>

/** @name System Constants */
#define VOID_PROCESSING_INTERVAL_MS  100 // 10Hz processing rate
#define VOID_DEFAULT_BASELINE_MM     200 // Default expected borehole diameter
#define VOID_DEFAULT_THRESHOLD_MM    100 // Default void detection threshold
#define VOID_DEFAULT_CONFIDENCE_MIN  70  // Default minimum confidence
#define VOID_HYSTERESIS_MM           10  // Hysteresis to prevent false events
#define VOID_CIRCLE_FIT_TOLERANCE_MM 20  // Circle fitting tolerance

/** @name Void Detection Character Codes */
#define VOID_CHAR_NONE    '0' // No void detected
#define VOID_CHAR_SENSOR0 'A' // Void detected by sensor 0 (0°)
#define VOID_CHAR_SENSOR1 'B' // Void detected by sensor 1 (120°)
#define VOID_CHAR_SENSOR2 'C' // Void detected by sensor 2 (240°)
#define VOID_CHAR_CIRCLE  'X' // Void detected by circle fitting
#define VOID_CHAR_TEST    'T' // Void detected in test/bypass mode
#define VOID_CHAR_ERROR   '?' // Unknown algorithm or error

/** @name Algorithm Types */
typedef enum
{
    VOID_ALGORITHM_BYPASS    = 0, // No processing
    VOID_ALGORITHM_SIMPLE    = 1, // Simple threshold detection
    VOID_ALGORITHM_CIRCLEFIT = 2  // Circle fitting detection
} void_algorithm_t;

/** @name Detection States */
typedef enum
{
    VOID_STATE_NO_DETECTION = 0,
    VOID_STATE_POSSIBLE     = 1,
    VOID_STATE_CONFIRMED    = 2
} void_detection_state_t;

/**
 * @brief Void detection results (Stage 3)
 */
typedef struct
{
    bool     void_detected;        // true if void found
    uint8_t  algorithm_used;       // 0=bypass,1=simple,2=circlefit
    uint8_t  confidence_percent;   // 0–100%
    uint16_t void_size_mm;         // estimated size
    uint8_t  sensor_count_used;    // sensors used
    uint32_t detection_time_ms;    // when done
    char     status_text[64];      // human-readable
    bool     new_result_available; // flagged for TX
} void_data_t;


/** @name Measurement Data Structure */
typedef struct
{
    uint16_t distance_mm[MAX_RADAR_SENSORS]; // Current distances
    uint16_t snr_value[MAX_RADAR_SENSORS];   // SNR values as integers
    bool     data_valid[MAX_RADAR_SENSORS];  // Data validity flags
    uint8_t  valid_sensor_count;             // Number of valid sensors
    uint32_t timestamp_ms;                   // Measurement timestamp
    uint8_t  flags;                          // Status flags for transmission
} void_measurement_t;

/** @name Configuration Structure */
typedef struct
{
    void_algorithm_t algorithm;              // Active detection algorithm
    uint16_t         baseline_diameter_mm;   // Expected borehole diameter
    uint16_t         threshold_mm;           // Void detection threshold
    uint8_t          confidence_min_percent; // Minimum confidence threshold
    uint16_t         min_distance_mm;        // Minimum valid distance
    uint16_t         max_distance_mm;        // Maximum valid distance
    bool             auto_fallback_enabled;  // Auto fallback to simple algorithm
    bool             median_filter_enabled;  // Enable median filtering
} void_config_t;

/** @name Circle Fitting Result */
typedef struct
{
    bool     fit_successful; // Circle fit was successful
    uint16_t center_x_mm;    // Circle center X coordinate
    uint16_t center_y_mm;    // Circle center Y coordinate
    uint16_t radius_mm;      // Circle radius
    uint16_t diameter_mm;    // Calculated diameter
    float    fit_quality;    // Fit quality (0.0-1.0)
} circle_fit_result_t;

/** @name System Initialization */

/**
 * @brief Initialize void detection system
 *
 * Sets up data structures and default configuration
 *
 * @return true if initialization successful
 */
bool void_system_init(void);

/**
 * @brief Main void detection processing function
 *
 * Call this from main loop - processes radar data and generates void events
 */
void void_system_process(void);

/**
 * @brief Check if void detection system is ready
 *
 * @return true if system is initialized and operational
 */
bool void_is_system_ready(void);

/** @name Data Access Functions */

/**
 * @brief Get latest void detection results
 *
 * @param result Pointer to structure to fill with latest data
 * @return true if new results are available
 */
bool void_get_latest_results(void_data_t *result);

/**
 * @brief Get measurement data for uphole transmission
 *
 * @param measurement Pointer to structure to fill with measurement data
 * @return true if new measurement data is available
 */
bool void_get_measurement_data(void_measurement_t *measurement);

/**
 * @brief Check if new void detection results are available
 *
 * @return true if new results are ready for transmission
 */
bool void_has_new_results(void);

/**
 * @brief Mark results as transmitted/processed
 */
void void_mark_results_processed(void);

/** @name Configuration Functions */

/**
 * @brief Set void detection algorithm
 *
 * @param algorithm Algorithm to use
 * @return true if algorithm was set successfully
 */
bool void_set_algorithm(void_algorithm_t algorithm);

/**
 * @brief Set baseline borehole diameter
 *
 * @param diameter_mm Expected borehole diameter in millimeters
 * @return true if value was set successfully
 */
bool void_set_baseline_diameter(uint16_t diameter_mm);

/**
 * @brief Set void detection threshold
 *
 * @param threshold_mm Threshold in millimeters above baseline
 * @return true if value was set successfully
 */
bool void_set_threshold(uint16_t threshold_mm);

/**
 * @brief Set minimum confidence threshold
 *
 * @param confidence_percent Minimum confidence (0-100)
 * @return true if value was set successfully
 */
bool void_set_confidence_threshold(uint8_t confidence_percent);

/**
 * @brief Enable/disable automatic fallback to simple algorithm
 *
 * @param enabled true to enable fallback
 */
void void_set_auto_fallback(bool enabled);

/**
 * @brief Enable/disable median filtering
 *
 * @param enabled true to enable median filtering
 */
void void_set_median_filter(bool enabled);

/**
 * @brief Get current configuration
 * @return Pointer to current configuration structure
 */
const void_config_t *void_get_config(void);

/**
 * @brief Get algorithm name as string
 *
 * @param algorithm Algorithm type
 * @return Pointer to algorithm name string
 */
const char *void_get_algorithm_string(void_algorithm_t algorithm);

/**
 * @brief Set baseline diameter
 *
 * @param baseline_mm Baseline diameter in millimeters
 */
bool void_set_baseline(uint16_t baseline_mm);

/**
 * @brief Set distance range
 *
 * @param min_mm Minimum distance in millimeters
 * @param max_mm Maximum distance in millimeters
 */
bool void_set_range(uint16_t min_mm, uint16_t max_mm);

/**
 * @brief Clear detection statistics
 */
void void_clear_statistics(void);

/** @name Algorithm-Specific Functions */

/**
 * @brief Test circle fitting with current data
 *
 * @param result Pointer to structure to fill with fit results
 * @return true if circle fitting was attempted
 */
bool void_test_circle_fit(circle_fit_result_t *result);

/**
 * @brief Get current detection state
 *
 * @return Current detection state
 */
void_detection_state_t void_get_detection_state(void);

/** @name System Status Functions */

/**
 * @brief Run void detection system diagnostics
 */
void void_run_diagnostics(void);

/**
 * @brief Get system statistics
 *
 * @param total_detections Pointer to store total detection count
 * @param algorithm_switches Pointer to store algorithm switch count
 * @param uptime_ms Pointer to store system uptime
 */
void void_get_statistics(uint32_t *total_detections, uint32_t *algorithm_switches, uint32_t *uptime_ms);

/** @name Event-Driven Processing */

/**
 * @brief Get event processing statistics
 * @param processing_count Pointer to store number of processing events
 * @param stream_count Pointer to store number of streams sent
 */
void void_get_event_stats(uint32_t *processing_count, uint32_t *stream_count);

/**
 * @brief Enable/disable automatic streaming mode
 * @param enabled true to enable automatic streaming to uphole
 */
void void_set_auto_streaming(bool enabled);

/**
 * @brief Check if automatic streaming is enabled
 * @return true if auto streaming is enabled
 */
bool void_is_auto_streaming_enabled(void);

/**
 * @brief Comprehensive void system test function (runs automatically)
 * @param test_interval_ms Test interval in milliseconds (e.g., 5000 for 5 seconds)
 */
void void_run_system_test(uint32_t test_interval_ms);

/**
 * @brief Print brief status of the void detection system
 *
 * Outputs current configuration and latest results
 */
void void_status_brief(void);

/** @name Simple System Control Functions (POC) */

/**
 * @brief Start void detection system (POC version)
 *
 * Simple startup: start radar sensors + enable auto streaming
 *
 * @return true if started successfully
 */
bool void_system_start(void);

/**
 * @brief Stop void detection system (POC version)
 *
 * Simple shutdown: stop radar sensors + disable auto streaming
 *
 * @return true if stopped successfully
 */
bool void_system_stop(void);

/**
 * @brief Check if void system is running
 *
 * @return true if system is active
 */
bool void_system_is_running(void);

/** @name Command Interface Functions */

/**
 * @brief Get current void detection status with character mapping
 * @param status_char Pointer to store detection character (0,A,B,C,X,T,?)
 * @param size_mm Pointer to store void size in mm
 * @param confidence_pct Pointer to store confidence percentage
 * @param system_state Pointer to store system state string
 * @return true if status is available
 */
bool void_get_status_compact(char *status_char, uint16_t *size_mm, uint8_t *confidence_pct, const char **system_state);

/**
 * @brief Get void detection character for current status
 * @return Character representing detection state (0,A,B,C,X,T,?)
 */
char void_get_detection_character(void);

/**
 * @brief Get description for a detection character
 * @param void_char Detection character
 * @return String description of the character
 */
const char *void_get_character_description(char void_char);

/**
 * @brief Check if a character is a valid detection character
 * @param void_char Character to validate
 * @return true if character is valid
 */
bool void_is_valid_detection_character(char void_char);

/**
 * @brief Send automatic data stream (called internally)
 */
void void_send_automatic_stream(void);

/**
 * @brief Get runtime diagnostics data
 * @param running Pointer to store running state
 * @param stats_detections Pointer to store detection count
 * @param stats_switches Pointer to store algorithm switches
 * @param stats_runtime_ms Pointer to store runtime in ms
 * @param ready Pointer to store ready state
 * @param streaming Pointer to store streaming state
 */
void void_get_diagnostics(bool *running, uint32_t *stats_detections, uint32_t *stats_switches, uint32_t *stats_runtime_ms, bool *ready, bool *streaming);

/** @name External Variables for POC */

/**
 * @brief Global void system running state (POC access)
 */
extern bool void_system_running;


#endif // MTI_VOID_H
