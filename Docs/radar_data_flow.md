# Radar System Data Flow Implementation Guide

## Overview

This document provides a complete step-by-step guide for implementing a clean, layered radar system data flow. The system consists of three distinct layers with clear data structures and minimal coupling between non-adjacent layers. The CAN layer handles both sensor configuration and continuous data reception.

## Target Architecture

```
CAN Bus → CAN Layer (radar_raw + config) → Radar Layer (radar_distance) → Void Detection Layer (void_data) → Reporting
```

### Design Principles

1. **Clear Layer Separation**: Each layer has distinct responsibilities and well-defined interfaces
2. **Consistent Naming**: Standardized struct naming (`radar_raw`, `radar_distance`, `void_data`)
3. **Unidirectional Data Flow**: Data flows forward through layers without circular dependencies
4. **Event-Driven Processing**: New data triggers processing cascades through the system
5. **Robust Error Handling**: Each layer validates inputs and handles errors gracefully
6. **Complete CAN Management**: CAN layer handles both sensor configuration and data reception

## CAN Layer Responsibilities

The CAN layer has two primary responsibilities:

### 1. Sensor Configuration and Control
- Initialize sensors via CAN commands
- Configure sensor parameters (power, FOV, detection thresholds)
- Monitor sensor status and health
- Handle sensor start/stop operations

### 2. Continuous Data Reception
- Receive raw radar data from multiple sensors
- Parse and validate incoming CAN messages
- Store raw data in `radar_raw_t` structures
- Notify radar layer of new data availability

## Current State Analysis

### What's Working Well

- ✅ CAN message reception and parsing
- ✅ Multi-sensor management and indexing  
- ✅ Basic data validation and filtering
- ✅ Sensor command transmission
- ✅ Void detection algorithms (bypass, threshold, circle fitting)
- ✅ Debug output and system monitoring

### What Needs Restructuring

- ❌ Mixed responsibilities in radar layer
- ❌ Direct void layer calls from CAN layer
- ❌ Inconsistent struct naming conventions
- ❌ No clear `radar_raw` → `radar_distance` → `void_data` flow
- ❌ Configuration and data handling mixed in interface
- ❌ Processing triggered from wrong layers

## Implementation Plan

---

## PHASE 1: CAN Layer Complete Restructuring (PRIORITY 1)

### File: `mti_radar_types.h` (NEW FILE - CREATE FIRST)

Create comprehensive type definitions for the entire pipeline:

```c
/**
 * @file mti_radar_types.h
 * @brief Common type definitions for radar data processing pipeline
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#ifndef MTI_RADAR_TYPES_H
#define MTI_RADAR_TYPES_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include <stdint.h>

/** @name System Configuration Constants */
#define MAX_RADAR_SENSORS 3U
#define MAX_RADAR_DETECTED_POINTS 20U

/** @name Sensor Angular Positions */
#define SENSOR_0_ANGLE 0
#define SENSOR_1_ANGLE 120
#define SENSOR_2_ANGLE 240

/** @name CAN Configuration Constants */
#define CAN_SENSOR_ID_OFFSET(sensor_idx) ((sensor_idx) * 0x10U)
#define CAN_CONFIG_RETRY_COUNT 3
#define CAN_CONFIG_TIMEOUT_MS 1000
#define CAN_INIT_SEQUENCE_TIMEOUT_MS 5000

/** @name Data Processing Constants */
#define RADAR_MIN_SNR_THRESHOLD 50.0f
#define RADAR_MIN_DISTANCE_M 0.05f
#define RADAR_MAX_DISTANCE_M 3.0f
#define RADAR_DATA_TIMEOUT_MS 2000
#define RADAR_INVALID_DISTANCE 0xFFFF

/**
 * @brief Radar hardware status enumeration
 */
typedef enum {
    RADAR_HW_NOT_INITIALISED = 0,
    RADAR_HW_READY = 1,
    RADAR_HW_CHIRPING = 2,
    RADAR_HW_STOPPED = 3,
    RADAR_HW_ERROR = 4
} radar_hw_status_t;

/**
 * @brief Radar firmware version information
 */
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint32_t build_number;
} radar_version_t;

/**
 * @brief Sensor configuration parameters
 */
typedef struct {
    uint8_t sensor_id;              // CAN sensor ID (0-2)
    uint8_t power_level;            // TX power (0-100%)
    uint8_t fov_degrees;            // Field of view (60-150°)
    float cfar_threshold;           // CFAR detection threshold
    uint8_t profile_id;             // Chirp profile (1-4)
    bool spread_spectrum_enabled;   // Spread spectrum on/off
} radar_sensor_config_t;

/**
 * @brief Sensor status and health information
 */
typedef struct {
    uint8_t sensor_id;              // Sensor index
    radar_hw_status_t hw_status;    // Hardware status from sensor
    radar_version_t version;        // Firmware version
    uint32_t last_response_ms;      // Last successful command response
    uint32_t message_count;         // Total messages received
    bool configuration_complete;    // Sensor fully configured
    bool online;                    // Currently responding
} radar_sensor_status_t;

/**
 * @brief Raw sensor data from CAN bus (Stage 1)
 * 
 * Contains unprocessed data exactly as received from radar sensors
 */
typedef struct {
    float detected_points[MAX_RADAR_DETECTED_POINTS][2];  // [distance_m, SNR]
    uint8_t num_points;                                   // Number of valid points
    uint32_t frame_number;                                // Frame counter from sensor
    uint16_t total_packet_length;                         // Total packet size
    uint8_t point_index;                                  // Current point being processed
    float max_snr;                                        // Maximum SNR in frame
    uint32_t timestamp_ms;                                // When data was received
    bool new_data_available;                              // Flag for event processing
    radar_sensor_status_t sensor_status;                  // Complete sensor status
} radar_raw_t;

/**
 * @brief Processed distance measurements (Stage 2)
 * 
 * Contains clean, validated measurements ready for void detection
 */
typedef struct {
    uint16_t distance_mm[MAX_RADAR_SENSORS];  // Clean distances in millimeters
    uint16_t angle_deg[MAX_RADAR_SENSORS];    // Sensor angles (0, 120, 240)
    bool data_valid[MAX_RADAR_SENSORS];       // Validity flags per sensor
    float confidence[MAX_RADAR_SENSORS];      // Confidence per sensor (0.0-1.0)
    uint8_t quality_score[MAX_RADAR_SENSORS]; // Signal quality (0-100)
    uint8_t valid_sensor_count;               // Number of sensors with valid data
    uint32_t timestamp_ms;                    // When processing completed
    bool system_healthy;                      // Overall system health
} radar_distance_t;

/**
 * @brief Void detection algorithm enumeration
 */
typedef enum {
    VOID_ALG_BYPASS = 0,
    VOID_ALG_SIMPLE_THRESHOLD = 1,
    VOID_ALG_CIRCLEFIT = 2
} void_algorithm_t;

/**
 * @brief Void severity levels
 */
typedef enum {
    VOID_SEVERITY_NONE = 0,
    VOID_SEVERITY_MINOR = 1,
    VOID_SEVERITY_MAJOR = 2,
    VOID_SEVERITY_CRITICAL = 3
} void_severity_t;

/**
 * @brief Void detection results (Stage 3)
 * 
 * Contains final analysis results ready for uphole transmission
 */
typedef struct {
    bool void_detected;                       // Primary detection result
    void_severity_t severity;                 // Severity level
    uint8_t confidence_percent;               // Detection confidence (0-100)
    void_algorithm_t algorithm_used;          // Algorithm that produced result
    uint16_t void_size_mm;                    // Estimated void size
    uint8_t sensor_count_used;                // Sensors used for detection
    uint32_t detection_time_ms;               // When analysis completed
    char status_text[64];                     // Human-readable status
    bool new_result_available;                // Flag for transmission
    
    // Individual sensor results
    struct {
        bool detected[MAX_RADAR_SENSORS];         // Per-sensor detection flags
        uint8_t confidence[MAX_RADAR_SENSORS];    // Per-sensor confidence
        uint16_t distance_mm[MAX_RADAR_SENSORS];  // Distances used for detection
    } sensor_data;
    
    // Algorithm-specific results
    union {
        struct {
            uint16_t threshold_used_mm;
            uint16_t baseline_used_mm;
        } threshold_data;
        
        struct {
            uint16_t center_x_mm;
            uint16_t center_y_mm;
            uint16_t radius_mm;
            uint8_t sensors_used;
            uint16_t fit_error_mm;
        } circle_data;
    } algorithm_result;
} void_data_t;

#endif // MTI_RADAR_TYPES_H
```

### File: `mti_can.h` (COMPLETE REWRITE)

```c
/**
 * @file mti_can.h
 * @brief CAN bus communication interface for radar sensors
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#ifndef MTI_CAN_H
#define MTI_CAN_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include "vmt_uart.h"
#include "mti_radar_types.h"

/** @name CAN Message ID Definitions
 * Based on actual IWR1843 AOP firmware implementation
 * @{
 */
#define CAN_CMD_BASE      0x80U  /**< Base address for command messages */
#define CAN_CMD_START     0x00U  /**< Command to start sensor operation */
#define CAN_CMD_STOP      0x01U  /**< Command to stop sensor operation */
#define CAN_CMD_CAL       0x02U  /**< Command to perform DC calibration */
#define CAN_CMD_POWER     0x03U  /**< Command to set TX power backoff */
#define CAN_CMD_STATUS    0x04U  /**< Command to request status and version */
#define CAN_CMD_THRESHOLD 0x05U  /**< Command to set CFAR detection threshold */
#define CAN_CMD_SPREAD    0x06U  /**< Command to enable/disable spread spectrum */
#define CAN_CMD_PROFILE   0x07U  /**< Command to set chirp configuration profile */
#define CAN_CMD_FOV       0x08U  /**< Command to set field of view */
#define CAN_CMD_INIT      0x09U  /**< Command to initialize sensor */

#define CAN_ID_HEADER_BASE   0xA0U  /**< Frame header messages */
#define CAN_ID_OBJECT_BASE   0xA1U  /**< Detected object data */
#define CAN_ID_STATUS_BASE   0xA8U  /**< Status response messages */
#define CAN_ID_VERSION_BASE  0xA9U  /**< Version information messages */
/** @} */

/** @name Power Level Definitions
 * @{
 */
#define RADAR_POWER_OFF    0     /**< Radar powered off */
#define RADAR_POWER_LOW    25    /**< Low power mode (25%) */
#define RADAR_POWER_MEDIUM 50    /**< Medium power mode (50%) */
#define RADAR_POWER_HIGH   75    /**< High power mode (75%) */
#define RADAR_POWER_MAX    100   /**< Maximum power (100%) */
/** @} */

/** @name Field of View Definitions
 * @{
 */
#define RADAR_FOV_NARROW 60      /**< Narrow field of view (60°) */
#define RADAR_FOV_MEDIUM 90      /**< Medium field of view (90°) */
#define RADAR_FOV_WIDE   120     /**< Wide field of view (120°) */
#define RADAR_FOV_MAX    150     /**< Maximum field of view (150°) */
/** @} */

/**
 * @brief Multi-sensor raw data system
 */
typedef struct {
    radar_raw_t sensors[MAX_RADAR_SENSORS];                    // Raw data from each sensor
    radar_sensor_status_t sensor_status[MAX_RADAR_SENSORS];    // Status for each sensor
    uint32_t last_message_time[MAX_RADAR_SENSORS];             // Last message timestamps
    uint32_t msgs_received[MAX_RADAR_SENSORS];                 // Message counters
    uint8_t active_sensor_count;                               // Cached count
    bool system_initialized;                                   // Initialization complete
} multi_radar_raw_system_t;

/** @name CAN System Initialization
 * @{
 */

/**
 * @brief Initialize complete CAN system with sensor configuration
 * 
 * Performs complete sensor initialization sequence:
 * 1. Set up CAN filters and interrupts
 * 2. Send initialization commands to all sensors
 * 3. Configure sensor parameters (power, FOV, etc.)
 * 4. Start continuous data streaming
 * 
 * @return true if initialization successful
 */
bool can_initialize_system(void);

/**
 * @brief Setup CAN hardware filters and interrupts
 * 
 * @return true if hardware setup successful
 */
bool can_setup_hardware(void);
/** @} */

/** @name CAN Sensor Configuration
 * @{
 */

/**
 * @brief Configure a specific sensor with parameters
 * 
 * @param sensor_idx Sensor index (0-2)
 * @param config Configuration parameters
 * @return true if configuration successful
 */
bool can_configure_sensor(uint8_t sensor_idx, const radar_sensor_config_t* config);

/**
 * @brief Start continuous data streaming from a sensor
 * 
 * @param sensor_idx Sensor index (0-2)
 * @return true if start command sent successfully
 */
bool can_start_sensor_streaming(uint8_t sensor_idx);

/**
 * @brief Stop data streaming from a sensor
 * 
 * @param sensor_idx Sensor index (0-2)
 * @return true if stop command sent successfully
 */
bool can_stop_sensor_streaming(uint8_t sensor_idx);

/**
 * @brief Send power level command to sensor
 * 
 * @param sensor_idx Sensor index (0-2)
 * @param power_percent Power level (0-100%)
 * @return true if command sent successfully
 */
bool can_set_sensor_power(uint8_t sensor_idx, uint8_t power_percent);

/**
 * @brief Set sensor field of view
 * 
 * @param sensor_idx Sensor index (0-2)
 * @param fov_degrees Field of view in degrees (60-150)
 * @return true if command sent successfully
 */
bool can_set_sensor_fov(uint8_t sensor_idx, uint8_t fov_degrees);

/**
 * @brief Set sensor detection threshold
 * 
 * @param sensor_idx Sensor index (0-2)
 * @param threshold CFAR threshold value
 * @return true if command sent successfully
 */
bool can_set_sensor_threshold(uint8_t sensor_idx, float threshold);

/**
 * @brief Set sensor chirp profile
 * 
 * @param sensor_idx Sensor index (0-2)
 * @param profile_id Profile ID (1-4)
 * @return true if command sent successfully
 */
bool can_set_sensor_profile(uint8_t sensor_idx, uint8_t profile_id);

/**
 * @brief Enable/disable spread spectrum
 * 
 * @param sensor_idx Sensor index (0-2)
 * @param enabled Enable spread spectrum
 * @return true if command sent successfully
 */
bool can_set_sensor_spread_spectrum(uint8_t sensor_idx, bool enabled);

/**
 * @brief Request sensor status and version information
 * 
 * @param sensor_idx Sensor index (0-2)
 * @return true if request sent successfully
 */
bool can_request_sensor_status(uint8_t sensor_idx);
/** @} */

/** @name CAN Data Access Functions
 * @{
 */

/**
 * @brief Get raw sensor data for processing by radar layer
 * 
 * @param sensor_idx Sensor index (0-2)
 * @return Pointer to raw sensor data or NULL if invalid
 */
radar_raw_t* can_get_raw_data(uint8_t sensor_idx);

/**
 * @brief Check if sensor has new unprocessed data
 * 
 * @param sensor_idx Sensor index (0-2)
 * @return true if new data is available
 */
bool can_has_new_raw_data(uint8_t sensor_idx);

/**
 * @brief Mark sensor data as processed by radar layer
 * 
 * @param sensor_idx Sensor index (0-2)
 */
void can_mark_raw_data_processed(uint8_t sensor_idx);

/**
 * @brief Get number of currently online sensors
 * 
 * @return Number of responding sensors (0-3)
 */
uint8_t can_get_online_sensor_count(void);

/**
 * @brief Check if specific sensor is online and responding
 * 
 * @param sensor_idx Sensor index (0-2)
 * @return true if sensor is online
 */
bool can_is_sensor_online(uint8_t sensor_idx);

/**
 * @brief Get complete sensor status information
 * 
 * @param sensor_idx Sensor index (0-2)
 * @return Pointer to sensor status or NULL if invalid
 */
radar_sensor_status_t* can_get_sensor_status(uint8_t sensor_idx);
/** @} */

/** @name CAN System Health and Diagnostics
 * @{
 */

/**
 * @brief Get overall CAN system health
 * 
 * @return true if system is healthy (>=2 sensors online)
 */
bool can_system_is_healthy(void);

/**
 * @brief Reset sensor data and status
 * 
 * @param sensor_idx Sensor index (0-2), or 0xFF for all sensors
 */
void can_reset_sensor_data(uint8_t sensor_idx);

/**
 * @brief Run CAN system diagnostics
 */
void can_run_diagnostics(void);

/**
 * @brief Get sensor index from CAN ID
 * 
 * @param can_id CAN message ID
 * @return Sensor index or 0xFF if invalid
 */
uint8_t get_sensor_index_from_can_id(uint32_t can_id);
/** @} */

/** @name Low-Level CAN Functions
 * @{
 */

/**
 * @brief Send single byte command
 * 
 * @param ID CAN message ID
 * @param message Command byte
 * @return true if sent successfully
 */
bool can_send(uint32_t ID, uint8_t message);

/**
 * @brief Send array of data
 * 
 * @param ID CAN message ID
 * @param message Data array
 * @param length Data length
 * @return true if sent successfully
 */
bool can_send_array(uint32_t ID, uint8_t *message, size_t length);

/**
 * @brief Send command to specific sensor
 * 
 * @param sensor_idx Sensor index (0-2)
 * @param base_id Base command ID
 * @param message Command data
 * @return true if sent successfully
 */
bool can_send_to_sensor(uint8_t sensor_idx, uint32_t base_id, uint8_t message);
/** @} */

#endif // MTI_CAN_H
```

### File: `mti_can.c` (COMPLETE REWRITE)

Since this is the priority, here's the complete implementation:

````c
/**
 * @file mti_can.c
 * @brief CAN bus communication implementation for radar sensors
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#include "can.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include <string.h>

/*------------------------------------------------------------------------------
 * Static Variables and Configuration
 *----------------------------------------------------------------------------*/

/** @brief Multi-sensor system instance */
static multi_radar_raw_system_t raw_radar_system = { 0 };

/** @brief Default sensor configurations */
static const radar_sensor_config_t default_sensor_configs[MAX_RADAR_SENSORS] = {
    { .sensor_id = 0, .power_level = 75, .fov_degrees = 120, .cfar_threshold = 100.0f, .profile_id = 2, .spread_spectrum_enabled = true },
    { .sensor_id = 1, .power_level = 75, .fov_degrees = 120, .cfar_threshold = 100.0f, .profile_id = 2, .spread_spectrum_enabled = true },
    { .sensor_id = 2, .power_level = 75, .fov_degrees = 120, .cfar_threshold = 100.0f, .profile_id = 2, .spread_spectrum_enabled = true }
};

/** @brief CAN hardware handles */
static CAN_RxHeaderTypeDef rxHeader;
static CAN_TxHeaderTypeDef txHeader;
static uint8_t canRX[8] = { 0 };
static CAN_FilterTypeDef canfil;
static uint32_t canMailbox;

/*------------------------------------------------------------------------------
 * Forward Declarations
 *----------------------------------------------------------------------------*/
static bool can_configure_sensor_sequence(uint8_t sensor_idx, const radar_sensor_config_t* config);
static bool can_wait_for_response(uint8_t sensor_idx, uint32_t timeout_ms);
static bool can_send_init_command(uint8_t sensor_idx);
static uint8_t get_active_sensor_count_internal(void);

/*------------------------------------------------------------------------------
 * CAN System Initialization
 *----------------------------------------------------------------------------*/

/**
 * @brief Initialize complete CAN system with sensor configuration
 */
bool can_initialize_system(void)
{
    debug_send("CAN: Starting system initialization");
    
    // Step 1: Initialize CAN hardware
    if (!can_setup_hardware()) {
        debug_send("CAN: Hardware setup failed");
        return false;
    }
    
    // Step 2: Initialize data structures
    memset(&raw_radar_system, 0, sizeof(raw_radar_system));
    
    // Step 3: Configure each sensor
    uint8_t configured_sensors = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        debug_send("CAN: Configuring sensor %d", i);
        
        if (can_configure_sensor_sequence(i, &default_sensor_configs[i])) {
            configured_sensors++;
            raw_radar_system.sensor_status[i].configuration_complete = true;
            debug_send("CAN: Sensor %d configured successfully", i);
        } else {
            debug_send("CAN: Sensor %d configuration failed", i);
        }
        
        // Small delay between sensor configurations
        HAL_Delay(100);
    }
    
    // Step 4: Start data streaming from configured sensors
    uint8_t streaming_sensors = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        if (raw_radar_system.sensor_status[i].configuration_complete) {
            if (can_start_sensor_streaming(i)) {
                streaming_sensors++;
                debug_send("CAN: Sensor %d streaming started", i);
            }
            HAL_Delay(50);
        }
    }
    
    // Step 5: Update system state
    raw_radar_system.active_sensor_count = streaming_sensors;
    raw_radar_system.system_initialized = (streaming_sensors >= 2);
    
    debug_send("CAN: System initialization complete - %d/%d sensors online", 
               streaming_sensors, MAX_RADAR_SENSORS);
    
    return raw_radar_system.system_initialized;
}

/**
 * @brief Setup CAN hardware filters and interrupts
 */
bool can_setup_hardware(void)
{
    debug_send("CAN: Setting up hardware");
    
    // Configure filter 0 for command acknowledgments (0x80-0x8F)
    canfil.FilterBank           = 0;
    canfil.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh         = ((0x00000080 << 3) | 0x4) >> 16;
    canfil.FilterIdLow          = ((0x00000080 << 3) | 0x4) & 0xFFFF;
    canfil.FilterMaskIdHigh     = ((0x000000F0 << 3) | 0x4) >> 16;
    canfil.FilterMaskIdLow      = ((0x000000F0 << 3) | 0x4) & 0xFFFF;
    canfil.FilterActivation     = ENABLE;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &canfil) != HAL_OK) {
        debug_send("CAN: Filter 0 configuration failed");
        return false;
    }
    
    // Configure filter 1 for data messages (0xA0-0xDF)
    canfil.FilterBank           = 1;
    canfil.FilterIdHigh         = ((0x000000A0 << 3) | 0x4) >> 16;
    canfil.FilterIdLow          = ((0x000000A0 << 3) | 0x4) & 0xFFFF;
    canfil.FilterMaskIdHigh     = ((0x000000E0 << 3) | 0x4) >> 16;
    canfil.FilterMaskIdLow      = ((0x000000E0 << 3) | 0x4) & 0xFFFF;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &canfil) != HAL_OK) {
        debug_send("CAN: Filter 1 configuration failed");
        return false;
    }
    
    // Start CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        debug_send("CAN: Start failed");
        return false;
    }
    
    // Enable receive interrupt
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        debug_send("CAN: Interrupt activation failed");
        return false;
    }
    
    debug_send("CAN: Hardware setup complete");
    return true;
}

/*------------------------------------------------------------------------------
 * Sensor Configuration Functions
 *----------------------------------------------------------------------------*/

/**
 * @brief Complete sensor configuration sequence
 */
static bool can_configure_sensor_sequence(uint8_t sensor_idx, const radar_sensor_config_t* config)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !config) {
        return false;
    }
    
    debug_send("CAN: Starting configuration sequence for sensor %d", sensor_idx);
    uint32_t start_time = HAL_GetTick();
    uint8_t retry_count = 0;
    
    // Step 1: Send initialization command with retries
    while (retry_count < CAN_CONFIG_RETRY_COUNT) {
        if (can_send_init_command(sensor_idx)) {
            if (can_wait_for_response(sensor_idx, CAN_CONFIG_TIMEOUT_MS)) {
                debug_send("CAN: Sensor %d init acknowledged", sensor_idx);
                break;
            } else {
                debug_send("CAN: Sensor %d init timeout (retry %d)", sensor_idx, retry_count + 1);
            }
        } else {
            debug_send("CAN: Sensor %d init send failed (retry %d)", sensor_idx, retry_count + 1);
        }
        retry_count++;
        HAL_Delay(200);
    }
    
    if (retry_count >= CAN_CONFIG_RETRY_COUNT) {
        debug_send("CAN: Sensor %d init failed after %d retries", sensor_idx, retry_count);
        return false;
    }
    
    // Step 2: Configure power level
    if (!can_set_sensor_power(sensor_idx, config->power_level)) {
        debug_send("CAN: Sensor %d power config failed", sensor_idx);
        return false;
    }
    HAL_Delay(100);
    
    // Step 3: Configure field of view
    if (!can_set_sensor_fov(sensor_idx, config->fov_degrees)) {
        debug_send("CAN: Sensor %d FOV config failed", sensor_idx);
        return false;
    }
    HAL_Delay(100);
    
    // Step 4: Set detection threshold
    if (!can_set_sensor_threshold(sensor_idx, config->cfar_threshold)) {
        debug_send("CAN: Sensor %d threshold config failed", sensor_idx);
        return false;
    }
    HAL_Delay(100);
    
    // Step 5: Select chirp profile
    if (!can_set_sensor_profile(sensor_idx, config->profile_id)) {
        debug_send("CAN: Sensor %d profile config failed", sensor_idx);
        return false;
    }
    HAL_Delay(100);
    
    // Step 6: Enable/disable spread spectrum
    if (!can_set_sensor_spread_spectrum(sensor_idx, config->spread_spectrum_enabled)) {
        debug_send("CAN: Sensor %d spread spectrum config failed", sensor_idx);
        return false;
    }
    HAL_Delay(100);
    
    // Step 7: Request final status
    can_request_sensor_status(sensor_idx);
    HAL_Delay(100);
    
    uint32_t total_time = HAL_GetTick() - start_time;
    debug_send("CAN: Sensor %d configured in %dms", sensor_idx, total_time);
    
    return true;
}

/**
 * @brief Wait for response from sensor
 */
static bool can_wait_for_response(uint8_t sensor_idx, uint32_t timeout_ms)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    
    uint32_t start_time = HAL_GetTick();
    uint32_t initial_msg_count = raw_radar_system.msgs_received[sensor_idx];
    
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (raw_radar_system.msgs_received[sensor_idx] > initial_msg_count) {
            return true;
        }
        HAL_Delay(10);
    }
    
    return false;
}

/**
 * @brief Send initialization command to sensor
 */
static bool can_send_init_command(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    
    uint32_t cmd_id = CAN_CMD_BASE + CAN_SENSOR_ID_OFFSET(sensor_idx);
    return can_send(cmd_id, CAN_CMD_INIT);
}

/*------------------------------------------------------------------------------
 * Individual Configuration Command Functions
 *----------------------------------------------------------------------------*/

bool can_configure_sensor(uint8_t sensor_idx, const radar_sensor_config_t* config)
{
    return can_configure_sensor_sequence(sensor_idx, config);
}

bool can_start_sensor_streaming(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    
    uint32_t cmd_id = CAN_CMD_BASE + CAN_SENSOR_ID_OFFSET(sensor_idx);
    
    if (can_send(cmd_id, CAN_CMD_START)) {
        debug_send("CAN: Start command sent to sensor %d", sensor_idx);
        return true;
    }
    
    debug_send("CAN: Failed to send start command to sensor %d", sensor_idx);
    return false;
}

bool can_stop_sensor_streaming(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    
    uint32_t cmd_id = CAN_CMD_BASE + CAN_SENSOR_ID_OFFSET(sensor_idx);
    
    if (can_send(cmd_id, CAN_CMD_STOP)) {
        raw_radar_system.sensor_status[sensor_idx].online = false;
        debug_send("CAN: Stop command sent to sensor %d", sensor_idx);
        return true;
    }
    
    return false;
}

bool can_set_sensor_power(uint8_t sensor_idx, uint8_t power_percent)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || power_percent > 100) {
        return false;
    }
    
    uint32_t cmd_id = CAN_CMD_BASE + CAN_SENSOR_ID_OFFSET(sensor_idx);
    uint8_t cmd_data[2] = { CAN_CMD_POWER, power_percent };
    
    return can_send_array(cmd_id, cmd_data, 2);
}

bool can_set_sensor_fov(uint8_t sensor_idx, uint8_t fov_degrees)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || fov_degrees < 60 || fov_degrees > 150) {
        return false;
    }
    
    uint32_t cmd_id = CAN_CMD_BASE + CAN_SENSOR_ID_OFFSET(sensor_idx);
    uint8_t cmd_data[2] = { CAN_CMD_FOV, fov_degrees };
    
    return can_send_array(cmd_id, cmd_data, 2);
}

bool can_set_sensor_threshold(uint8_t sensor_idx, float threshold)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    
    uint32_t cmd_id = CAN_CMD_BASE + CAN_SENSOR_ID_OFFSET(sensor_idx);
    uint16_t threshold_int = (uint16_t)(threshold * 10.0f); // Convert to fixed point
    uint8_t cmd_data[3] = { CAN_CMD_THRESHOLD, 
                           (uint8_t)(threshold_int & 0xFF), 
                           (uint8_t)((threshold_int >> 8) & 0xFF) };
    
    return can_send_array(cmd_id, cmd_data, 3);
}

bool can_set_sensor_profile(uint8_t sensor_idx, uint8_t profile_id)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || profile_id < 1 || profile_id > 4) {
        return false;
    }
    
    uint32_t cmd_id = CAN_CMD_BASE + CAN_SENSOR_ID_OFFSET(sensor_idx);
    uint8_t cmd_data[2] = { CAN_CMD_PROFILE, profile_id };
    
    return can_send_array(cmd_id, cmd_data, 2);
}

bool can_set_sensor_spread_spectrum(uint8_t sensor_idx, bool enabled)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    
    uint32_t cmd_id = CAN_CMD_BASE + CAN_SENSOR_ID_OFFSET(sensor_idx);
    uint8_t cmd_data[2] = { CAN_CMD_SPREAD, enabled ? 1 : 0 };
    
    return can_send_array(cmd_id, cmd_data, 2);
}

bool can_request_sensor_status(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    
    uint32_t cmd_id = CAN_CMD_BASE + CAN_SENSOR_ID_OFFSET(sensor_idx);
    return can_send(cmd_id, CAN_CMD_STATUS);
}

/*------------------------------------------------------------------------------
 * CAN Message Reception and Processing
 *----------------------------------------------------------------------------*/

/**
 * @brief CAN message reception interrupt handler
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX) != HAL_OK) {
        return;
    }
    
    uint8_t sensor_idx = get_sensor_index_from_can_id(rxHeader.StdId);
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return;
    }
    
    // Update sensor online status
    raw_radar_system.last_message_time[sensor_idx] = HAL_GetTick();
    raw_radar_system.sensor_status[sensor_idx].online = true;
    raw_radar_system.msgs_received[sensor_idx]++;
    
    // Process based on message type
    uint32_t base_msg_id = rxHeader.StdId & 0xF0; // Remove sensor offset
    
    switch (base_msg_id) {
        case CAN_ID_HEADER_BASE:
            process_header_message(sensor_idx, canRX);
            break;
            
        case CAN_ID_OBJECT_BASE:
            process_object_message(sensor_idx, canRX);
            break;
            
        case CAN_ID_STATUS_BASE:
            process_status_message(sensor_idx, canRX);
            break;
            
        case CAN_ID_VERSION_BASE:
            process_version_message(sensor_idx, canRX);
            break;
            
        default:
            debug_send("CAN: Unknown message ID 0x%03X from sensor %d", rxHeader.StdId, sensor_idx);
    }
}

/**
 * @brief Process header message from sensor
 */
void process_header_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data) {
        return;
    }
    
    radar_raw_t *sensor = &raw_radar_system.sensors[sensor_idx];
    
    // Extract header information
    sensor->frame_number = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | 
                          ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    sensor->total_packet_length = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    sensor->num_points = data[6];
    sensor->point_index = 0; // Reset for new frame
    sensor->max_snr = 0.0f;  // Reset for new frame
    
    debug_send("CAN: S%d header - frame %d, %d points, %d bytes", 
               sensor_idx, sensor->frame_number, sensor->num_points, sensor->total_packet_length);
}

/**
 * @brief Process object detection message from sensor
 */
void process_object_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data) {
        return;
    }
    
    radar_raw_t *sensor = &raw_radar_system.sensors[sensor_idx];
    
    if (sensor->point_index >= MAX_RADAR_DETECTED_POINTS) {
        debug_send("CAN: S%d point buffer overflow", sensor_idx);
        return;
    }
    
    // Extract distance and SNR (assuming specific data format)
    // This needs to match actual IWR1843 CAN message format
    uint16_t distance_mm = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    uint16_t snr_raw = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    
    // Convert to float values
    float distance_m = (float)distance_mm / 1000.0f;
    float snr = (float)snr_raw / 10.0f; // Assuming SNR is scaled by 10
    
    // Store in detected points array
    sensor->detected_points[sensor->point_index][0] = distance_m;
    sensor->detected_points[sensor->point_index][1] = snr;
    
    // Track maximum SNR
    if (snr > sensor->max_snr) {
        sensor->max_snr = snr;
    }
    
    sensor->point_index++;
    
    // Check if frame is complete
    if (sensor->point_index >= sensor->num_points) {
        process_complete_radar_frame(sensor_idx);
    }
}

/**
 * @brief Process status message from sensor
 */
void process_status_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data) {
        return;
    }
    
    radar_sensor_status_t *status = &raw_radar_system.sensor_status[sensor_idx];
    
    // Extract status information
    status->hw_status = (radar_hw_status_t)data[0];
    status->last_response_ms = HAL_GetTick();
    
    debug_send("CAN: S%d status - HW status: %d", sensor_idx, status->hw_status);
}

/**
 * @brief Process version message from sensor
 */
void process_version_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data) {
        return;
    }
    
    radar_sensor_status_t *status = &raw_radar_system.sensor_status[sensor_idx];
    
    // Extract version information
    status->version.major = data[0];
    status->version.minor = data[1];
    status->version.patch = data[2];
    status->version.build_number = (uint32_t)data[4] | ((uint32_t)data[5] << 8) | 
                                  ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
    
    debug_send("CAN: S%d version - %d.%d.%d build %d", 
               sensor_idx, status->version.major, status->version.minor, 
               status->version.patch, status->version.build_number);
}

/**
 * @brief Process complete radar frame and notify radar layer
 */
void process_complete_radar_frame(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return;
    }
    
    radar_raw_t *sensor = &raw_radar_system.sensors[sensor_idx];
    
    debug_send("CAN: S%d frame %d complete (%d points, max SNR=%.1f)", 
               sensor_idx, sensor->frame_number, sensor->num_points, sensor->max_snr);
    
    // Update system state
    sensor->timestamp_ms = HAL_GetTick();
    sensor->new_data_available = true;
    
    // Update active sensor count
    raw_radar_system.active_sensor_count = get_active_sensor_count_internal();
    
    // ONLY notify radar layer when both layers exist
    #ifdef MTI_RADAR_H  // Only call if radar layer is implemented
    radar_notify_new_raw_data(sensor_idx);
    #else
    debug_send("CAN: Radar layer not yet implemented - data stored in buffer");
    #endif
}

/*------------------------------------------------------------------------------
 * Data Access Functions
 *----------------------------------------------------------------------------*/

radar_raw_t* can_get_raw_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return NULL;
    }
    return &raw_radar_system.sensors[sensor_idx];
}

bool can_has_new_raw_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    return raw_radar_system.sensors[sensor_idx].new_data_available;
}

void can_mark_raw_data_processed(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return;
    }
    raw_radar_system.sensors[sensor_idx].new_data_available = false;
}

uint8_t can_get_online_sensor_count(void)
{
    return raw_radar_system.active_sensor_count;
}

bool can_is_sensor_online(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    
    uint32_t current_time = HAL_GetTick();
    uint32_t last_msg = raw_radar_system.last_message_time[sensor_idx];
    
    return (current_time - last_msg) < 3000; // 3 second timeout
}

radar_sensor_status_t* can_get_sensor_status(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return NULL;
    }
    return &raw_radar_system.sensor_status[sensor_idx];
}

bool can_system_is_healthy(void)
{
    return raw_radar_system.system_initialized && 
           (raw_radar_system.active_sensor_count >= 2);
}

/*------------------------------------------------------------------------------
 * System Management Functions
 *----------------------------------------------------------------------------*/

void can_reset_sensor_data(uint8_t sensor_idx)
{
    if (sensor_idx == 0xFF) {
        // Reset all sensors
        memset(&raw_radar_system, 0, sizeof(raw_radar_system));
        debug_send("CAN: All sensor data reset");
    } else if (sensor_idx < MAX_RADAR_SENSORS) {
        // Reset specific sensor
        memset(&raw_radar_system.sensors[sensor_idx], 0, sizeof(radar_raw_t));
        memset(&raw_radar_system.sensor_status[sensor_idx], 0, sizeof(radar_sensor_status_t));
        raw_radar_system.last_message_time[sensor_idx] = 0;
        raw_radar_system.msgs_received[sensor_idx] = 0;
        debug_send("CAN: Sensor %d data reset", sensor_idx);
    }
}

void can_run_diagnostics(void)
{
    debug_send("=== CAN System Diagnostics ===");
    debug_send("System initialized: %s", raw_radar_system.system_initialized ? "YES" : "NO");
    debug_send("Active sensors: %d/%d", raw_radar_system.active_sensor_count, MAX_RADAR_SENSORS);
    
    uint32_t current_time = HAL_GetTick();
    
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        radar_sensor_status_t* status = &raw_radar_system.sensor_status[i];
        uint32_t last_msg_age = current_time - raw_radar_system.last_message_time[i];
        
        debug_send("Sensor %d:", i);
        debug_send("  Online: %s", status->online ? "YES" : "NO");
        debug_send("  Configured: %s", status->configuration_complete ? "YES" : "NO");
        debug_send("  Messages: %d", raw_radar_system.msgs_received[i]);
        debug_send("  Last message: %dms ago", last_msg_age);
        debug_send("  HW Status: %d", status->hw_status);
        debug_send("  Version: %d.%d.%d", status->version.major, status->version.minor, status->version.patch);
    }
    
    debug_send("===============================");
}

uint8_t get_sensor_index_from_can_id(uint32_t can_id)
{
    // Extract sensor index from CAN ID
    // Assuming sensor offset of 0x10 per sensor
    uint8_t offset = (can_id & 0x0F);
    if (offset == 0) return 0;
    if (offset == 1) return 1;  
    if (offset == 2) return 2;
    
    return 0xFF; // Invalid sensor
}

/*------------------------------------------------------------------------------
 * Low-Level CAN Functions  
 *----------------------------------------------------------------------------*/

bool can_send(uint32_t ID, uint8_t message)
{
    uint8_t data[1] = { message };
    return can_send_array(ID, data, 1);
}

bool can_send_array(uint32_t ID, uint8_t *message, size_t length)
{
    if (!message || length == 0 || length > 8) {
        return false;
    }
    
    txHeader.StdId = ID;
    txHeader.ExtId = 0x00;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = length;
    txHeader.TransmitGlobalTime = DISABLE;
    
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, message, &canMailbox);
    
    if (status != HAL_OK) {
        debug_send("CAN: Send failed - ID=0x%03X, status=%d", ID, status);
        return false;
    }
    
    return true;
}

bool can_send_to_sensor(uint8_t sensor_idx, uint32_t base_id, uint8_t message)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    
    uint32_t sensor_id = base_id + CAN_SENSOR_ID_OFFSET(sensor_idx);
    return can_send(sensor_id, message);
}

/*------------------------------------------------------------------------------
 * Internal Helper Functions
 *----------------------------------------------------------------------------*/

static uint8_t get_active_sensor_count_internal(void)
{
    uint8_t count = 0;
    uint32_t current_time = HAL_GetTick();
    
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        if ((current_time - raw_radar_system.last_message_time[i]) < 3000) {
            count++;
        }
    }
    
    return count;
}
````
