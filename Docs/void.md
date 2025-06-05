# Void Detection System: Downhole Module Implementation Guide

**Version:** 1.3.1
**Date:** June 5, 2025
**Status:** Active Development

## Table of Contents

1. [Introduction](#1-introduction)
    1.1. [Purpose](#11-purpose)
    1.2. [Scope](#12-scope)
    1.3. [Target Audience](#13-target-audience)
    1.4. [Definitions and Acronyms](#14-definitions-and-acronyms)
2. [Executive Summary](#2-executive-summary)
3. [Void Detection Algorithm Requirements](#3-void-detection-algorithm-requirements)
    3.1. [Core Algorithm Design](#31-core-algorithm-design)
    3.2. [Implementation Requirements](#32-implementation-requirements)
    3.3. [Development Steps](#33-development-steps)
4. [Communication Protocols](#4-communication-protocols)
    4.1. [CAN Bus (Sensor Communication)](#41-can-bus-sensor-communication)
    4.2. [UART/RS485 (Uphole Communication)](#42-uartrs485-uphole-communication)
    4.2.1. [Command Format (Uphole/Debug TO Downhole)](#421-command-format-upholedebug-to-downhole)
    4.2.2. [Response/Event Format (Downhole TO Uphole/Debug)](#422-responseevent-format-downhole-to-upholedebug)
5. [Void Detection Methods](#5-void-detection-methods)
    5.1. [Simplified POC Architecture](#51-simplified-poc-architecture)
    5.2. [Simplified Processing Flow](#52-simplified-processing-flow)
6. [Detailed Design and Implementation](#6-detailed-design-and-implementation)
    6.1. [POC Data Structures and Flow](#61-poc-data-structures-and-flow)
    6.2. [Simplified Processing Modules](#62-simplified-processing-modules)
    6.3. [Basic Implementation Flow](#63-basic-implementation-flow)
7. [Embedded System Considerations](#7-embedded-system-considerations)
    7.1. [Real-Time Requirements](#71-real-time-requirements)
    7.2. [Memory Management](#72-memory-management)
    7.3. [Power Management](#73-power-management)
    7.4. [Hardware Abstraction](#74-hardware-abstraction)
8. [Implementation Status](#8-implementation-status)
    8.1. [Current POC Architecture Status](#81-current-poc-architecture-status)
    8.2. [Completed Implementation Details](#82-completed-implementation-details)
    8.3. [Updated POC Development Plan](#83-updated-poc-development-plan)
    8.4. [Detailed Implementation Specifications](#84-detailed-implementation-specifications)
9. [Next Steps](#12-next-steps)
10. [Current Implementation Status Summary](#13-current-implementation-status-summary)
11. [Appendix: Detailed Command/Response Timing Analysis](#14-appendix-detailed-commandresponse-timing-analysis)
12. [Appendix: Detailed Data Structure Definitions](#15-appendix-detailed-data-structure-definitions)

-----

## 1. Introduction

### 1.1. Purpose

This document provides a comprehensive technical guide for the design, implementation, and development of the void detection firmware for the downhole probe module. The system is built on an STM32F722 microcontroller following BARR-C embedded development guidelines and industry best practices for safety-critical applications.

### 1.2. Scope

This document covers the downhole module firmware implementation including:

* Real-time acquisition and processing of radar sensor data via CAN bus
* Implementation of void detection algorithms with configurable parameters
* Communication with uphole control system via UART/RS485
* System health monitoring, fault detection, and recovery mechanisms
* Temperature sensing and IMU integration (IMU integration for void detection is future work)
* Memory-efficient data structures and processing algorithms

### 1.3. Target Audience

* **Embedded Software Engineers:** Implementing and maintaining real-time firmware
* **System Architects:** Understanding embedded system interactions and constraints
* **Test Engineers:** Developing verification and validation procedures
* **Quality Assurance:** Ensuring compliance with safety and reliability standards

### 1.4. Definitions and Acronyms

| Term   | Definition                                         |
|:-------|:---------------------------------------------------|
| BARR-C | Barr Group's Embedded C Coding Standard            |
| CAN    | Controller Area Network - automotive standard bus  |
| mm     | Millimeters                                        |
| HAL    | Hardware Abstraction Layer (STM32 HAL)             |
| ISR    | Interrupt Service Routine                          |
| MCU    | Microcontroller Unit (STM32F722)                   |
| RTOS   | Real-Time Operating System (not used - bare metal) |
| SNR    | Signal-to-Noise Ratio                              |
| VMT    | Void Measurement Tool                              |
| WCET   | Worst Case Execution Time                          |

-----

## 2. Executive Summary

The void detection system is a safety-critical embedded application running on STM32F722 hardware. The system continuously monitors borehole wall distances using three radar sensors positioned at 120° intervals, processes this data in real-time using **millimeter (mm)** precision, and reports void detections to an uphole control system.

**Current Architecture Status:**

* ✅ **System Architecture:** Layered design with clear separation of concerns. Core modules include `mti_void` for detection, `mti_can` for sensor communication, `vmt_command` for uphole interface, and `mti_system` for overall control and initialization.
* ✅ **Hardware Initialization:** Comprehensive system startup sequence implemented in `mti_system.c`.
* ✅ **CAN Communication:** Robust radar sensor interface with error recovery detailed in `mti_can.c`.
* ✅ **UART Communication:** Debug and uphole communication channels functional (`vmt_uart.c`). Commands are prefixed with `@`. Responses/events from downhole use `&`, `!`, or `$` respectively.
* ✅ **Temperature Monitoring:** Complete temperature module implemented in [`mti_temp.c`](../Device/Src/mti_temp.c) with ADC-based sensing, smoothing, threshold detection, and command interface. **(COMPLETED)**
* ✅ **Radar System:** Complete round-robin sensor management implemented in [`mti_radar.c`](../Device/Src/mti_radar.c) with 3-sensor rotation, distance measurement processing, and status monitoring.
* ✅ **IMU System:** Functional accelerometer/gyroscope monitoring with dual-sensor validation and error handling.
* ✅ **Water Detection:** Basic water detection implemented with ADC monitoring and threshold detection.
* ❌ **Void Detection Logic:** Core algorithms using millimeter precision need implementation. The [`mti_void.c`](../Device/Src/mti_void.c) module exists but requires full algorithm development. This is the critical missing component.
* ✅ **Error Recovery:** Basic error handling (e.g., CAN sensor resets) is in place. Build system successfully compiles with 0 errors.

### Next Priority: Void Detection Implementation

With the temperature monitoring system now complete, the primary focus shifts to the core void detection algorithm implementation. All supporting infrastructure (radar data acquisition, communication) is functional. The void detection module ([`mti_void.c`](../Device/Src/mti_void.c)) needs to:

1. Process radar distance measurements from 3 sensors
2. Apply geometric calculations for borehole diameter analysis  
3. Detect significant void spaces using configurable thresholds
4. Generate void events with precise location data
5. Integrate with existing command and communication infrastructure

## 3. Void Detection Algorithm Requirements

### 3.1 Core Algorithm Design

The void detection system must implement the following algorithm stages:

#### Stage 1: Data Acquisition

* Collect distance measurements from 3 radar sensors positioned at 120° intervals
* Validate data quality and sensor health status
* Apply sensor-specific calibration offsets
* Filter out noise and invalid readings

#### Stage 2: Geometric Analysis

* Calculate borehole center position using triangulation
* Determine effective borehole diameter at measurement point
* Detect asymmetric wall conditions indicating potential voids
* Apply coordinate transformation for precise void location

#### Stage 3: Void Detection Logic

* Compare calculated diameter against baseline/expected diameter
* Apply configurable void detection thresholds (e.g., 20% diameter increase)
* Implement hysteresis to prevent false void start/end events
* Generate void severity classification (minor, major, critical)

#### Stage 4: Event Generation

* Generate void start events with precise location and severity
* Continuous void monitoring and size tracking
* Generate void end events when conditions return to normal
* Log void characteristics for uphole analysis

### 3.2 Implementation Requirements

**Data Structures:**

```c
typedef struct {
    uint16_t distance_mm[MAX_RADAR_SENSORS];  // Distance measurements in mm
    uint16_t angle_deg[MAX_RADAR_SENSORS];    // Sensor angular positions
    bool data_valid[MAX_RADAR_SENSORS];       // Data validity flags
    uint32_t timestamp_ms;                    // Measurement timestamp
} void_measurement_t;

typedef struct {
    bool void_detected;                       // Current void state
    uint16_t void_diameter_mm;               // Calculated void diameter
    uint16_t baseline_diameter_mm;           // Expected baseline diameter
    uint8_t void_severity;                   // 0=none, 1=minor, 2=major, 3=critical
    uint32_t void_start_time_ms;             // When void started
    uint32_t last_update_ms;                 // Last algorithm update
} void_status_t;
```

**Key Functions to Implement:**

* `void_system_init()` - Initialize void detection system
* `void_process_measurements()` - Main algorithm processing
* `void_get_status()` - Get current void detection status
* `void_set_thresholds()` - Configure detection parameters
* `void_calibrate_baseline()` - Set baseline diameter

**Integration Points:**

* Integrate with existing `radar_system_process()` in `vmt_device.c`
* Add void commands to `vmt_command.c` command handlers
* Use existing UART channels for void event reporting
* Leverage temperature data for environmental compensation

**Command Interface:**

* `@void,status` - Get current void detection status
* `@void,config,threshold,<value>` - Set void detection threshold
* `@void,config,baseline,<diameter>` - Set baseline borehole diameter
* `@void,calibrate` - Perform baseline calibration
* `!void,<detected>` - Void detection events (uphole reporting)

### 3.3 Development Steps

1. **Create `mti_void.h`** - Define data structures and function prototypes
2. **Implement `mti_void.c`** - Core void detection algorithms
3. **Integrate with `vmt_device.c`** - Add void processing to main loop
4. **Add void commands** - Extend `vmt_command.c` with void-specific commands
5. **Testing and Calibration** - Validate algorithm with test data
6. **Documentation** - Complete algorithm documentation and usage guide

-----

## 4. Communication Protocols

### 4.1. CAN Bus (Sensor Communication)

**Implementation in `mti_can.c`:**

* **Bus Speed:** 500 kbps.
* **Message Format:** Extended frame (29-bit ID).
* **Error Handling:** Automatic retransmission, bus-off recovery.
* **Real-Time:** Deterministic 100ms polling cycle for radar data requests.

```c
// Sensor addressing scheme (from mti_can.h)
#define CAN_MSG_ID_HEADER_SENSOR(sensor_idx)  (CAN_ID_HEADER_BASE + ((sensor_idx) * 0x10U)) //
#define CAN_MSG_ID_OBJECT_SENSOR(sensor_idx)  (CAN_ID_OBJECT_BASE + ((sensor_idx) * 0x10U)) //
#define CAN_MSG_ID_STATUS_SENSOR(sensor_idx)  (CAN_ID_STATUS_BASE + ((sensor_idx) * 0x10U)) //
#define CAN_CMD_ID_SENSOR(sensor_idx)         (CAN_CMD_BASE + (sensor_idx)) // For sending commands to sensors

// Real-time constraints (from vmt_common_defs.h)
#define RADAR_POLL_INTERVAL_MS    100   // 10Hz system update rate
#define CAN_COMM_TIMEOUT_MS      2000   // Communication timeout
#define SENSOR_RESET_DELAY_MS      10   // Reset recovery time
```

### 4.2. UART/RS485 (Uphole Communication)

**Implementation in `vmt_uart.c` and `vmt_command.c`:**

* **Baud Rate:** Configurable per UART instance (e.g., 115200 bps).
* **Protocol:** ASCII command/response with defined prefixes.
* **Buffer Management:** Circular RX/TX buffers.

#### 4.2.1. Command Format (Uphole/Debug TO Downhole)

All commands sent **TO** the downhole unit **MUST** be prefixed with `@`.
The general command structure is `@<command_family>,<action/target>,[param1],[param2],...<newline>`.
String parsing is handled by `string_decoder_by_end` in `vmt_string.c`.

```c
// --- System Control & Status Commands ---
/**
 * @brief Request connection and basic system status. Uphole sends this.
 * @command @connect
 * @response &status,down,<system_status_id>,ver,<fw_maj>,<fw_min>,<fw_sub>
 */

/**
 * @brief Command device to enter sleep mode.
 * @command @sleep
 * @response $db,Downhole sleep (debug) THEN $db,@wake (on wakeup, debug) AND !wake (uphole)
 */

/**
 * @brief Request current status of various subsystems or overall status.
 * @command @status?  // General status (similar to @connect response)
 * @command @status,imu?
 * @command @status,water?
 * @command @status,temp?
 * @command @status,void? // Same as @vd,status?
 * @response Depends on query, e.g., &status,imu,<active_imu_id>,<profile_id>
 */

// --- Void Detection Commands (@vd) ---

/**
 * @brief Get current operational status of the void detection subsystem.
 * @command @vd,status?
 * @response &vd,status,<op_mode>,<system_status_for_void>,<radar_hw_status>
 * <op_mode>: Current system_op_mode_t (e.g., 0 for STOPPED, 1 for MEASURE)
 * <system_status_for_void>: e.g., SYSTEM_STATUS_OK, SYSTEM_STATUS_ERROR_RADAR_HW
 * <radar_hw_status>: radar_hw_status_t from void_get_subsystem_hw_status()
 */

/**
 * @brief Get current void detection configuration.
 * @command @vd,config?
 * @response &vd,config,thresh_mm,<val>,conf_perc,<val>,diam_mm,<val>,range_min_mm,<val>,range_max_mm,<val>,mode_base,<val>,mode_multi,<val>,filter_median,<val>
 * (Values correspond to void_config_t members)
 */

/**
 * @brief Set void detection threshold.
 * @command @vd,config,thresh,<value_mm>
 * @param <value_mm>: Detection threshold in millimeters.
 * @response &vd,config,thresh,ack or &vd,config,thresh,nack,<error_code>
 */

/**
 * @brief Set void confidence threshold.
 * @command @vd,config,conf,<value_percent>
 * @param <value_percent>: Confidence threshold (0-100).
 * @response &vd,config,conf,ack or &vd,config,conf,nack,<error_code>
 */

/**
 * @brief Set expected borehole diameter. (Calls void_set_expected_diameter)
 * @command @vd,config,diam,<value_mm>
 * @param <value_mm>: Expected borehole diameter in millimeters.
 * @response &vd,config,diam,ack or &vd,config,diam,nack,<error_code>
 */

/**
 * @brief Set sensor operational range.
 * @command @vd,config,range,<min_mm>,<max_mm>
 * @param <min_mm>: Minimum valid sensor range in millimeters.
 * @param <max_mm>: Maximum valid sensor range in millimeters.
 * @response &vd,config,range,ack or &vd,config,range,nack,<error_code>
 */

/**
 * @brief Set baseline detection method.
 * @command @vd,config,mode,baseline,<method_id>
 * @param <method_id>: Integer ID for the baseline_method in void_config_t.
 * @response &vd,config,mode,baseline,ack or &vd,config,mode,baseline,nack,<error_code>
 */

/**
 * @brief Set multi-point processing algorithm.
 * @command @vd,config,mode,multipoint,<algorithm_id>
 * @param <algorithm_id>: Integer ID for void_multi_point_algorithm_t.
 * @response &vd,config,mode,multipoint,ack or &vd,config,mode,multipoint,nack,<error_code>
 */

/**
 * @brief Enable/disable median filter (placeholder for future enhancement).
 * @command @vd,config,filter,median,<0_or_1>
 * @param <0_or_1>: 0 to disable, 1 to enable.
 * @response &vd,config,filter,median,ack or &vd,config,filter,median,nack,<error_code>
 */

/**
 * @brief Set calibration factor for a specific radar sensor (Future Enhancement).
 * @command @vd,cal,sensor,<sensor_idx>,<factor_ppm>
 * @param <sensor_idx>: Sensor index (0-MAX_SENSORS-1).
 * @param <factor_ppm>: Calibration factor in parts per million (e.g., 1000000 for no cal, 1050000 for +5%).
 * @response &vd,cal,sensor,ack or &vd,cal,sensor,nack,<error_code>
 */

/**
 * @brief Request latest N wall profile data entries.
 * @command @vd,history,profile[,<count>]
 * @param <count> (optional): Number of latest profiles to retrieve. Default: 1.
 * @response Multiple lines of: &vd,history,profile,<timestamp_ms>,<center_x_mm>,<center_y_mm>,<radius_mm>,<s0_dist_mm>,<s0_status_id>,<s1_dist_mm>,<s1_status_id>,<s2_dist_mm>,<s2_status_id>
 * Followed by: &vd,history,profile,done
 */

/**
 * @brief Request latest N void detection data entries.
 * @command @vd,history,detection[,<count>]
 * @param <count> (optional): Number of latest detections to retrieve. Default: 1.
 * @response Multiple lines of: &vd,history,detection,<timestamp_ms>,<sector_idx>,<size_mm>,<confidence_%>
 * Followed by: &vd,history,detection,done
 */

/**
 * @brief Clear void detection and wall profile history.
 * @command @vd,clear,history
 * @response &vd,clear,history,ack or &vd,clear,history,nack,<error_code>
 */

/**
 * @brief Get detailed diagnostic information for the void detection subsystem.
 * @command @vd,diag?
 * @response &vd,diag,S0:[<dist_mm>,<snr>,<status_id>,<ts>],S1:[...],S2:[...],Profile:[<cx_mm>,<cy_mm>,<rad_mm>,<ts>],Config:[<thresh_mm>,<conf_perc>],HWStatus:<radar_hw_status_id>
 * (Content includes recent sensor readings, last profile, relevant config, and HW status)
 */

// --- Other existing commands (ensure @ prefix) ---
// @dt - Device detect (deprecated by @connect or @status?)
// @bt - Bottom detect status query (now part of @imu,status,bump?)
// @wt - Water detect status query (now part of @status,water?)
// @sensor,w,<value> or @sensor,g,<profile> - Sensor config (specific setters for water/IMU, potentially @vd,config for void sensors)
// @st - Start measurement mode (system level)
// @fn - Finish/Stop measurement mode (system level)
// @init,... - Initialization control/query (system level)
// @echo,... - Echo back parameters
// @spi,... - Low-level SPI access (debug)
// @log,... - Logging control (system level)
// @ff,... - Flash FIFO direct access (debug)
// @flash,... - Flash direct access (debug)
// @imu,... - IMU specific commands (e.g. @imu,config,param,value)
// @cmd,... - Generic debug/command execution
// @tp,... - Temperature sensor commands
```

> **Note:** The `@vd,history,…` and `@vd,diag?` commands are specified here but are not yet supported in the firmware. The calls to `prv_void_store_profile_to_history()` and `prv_void_store_detection_to_history()` remain disabled and no handlers exist. These will be implemented in a future release.

#### 4.2.2. Response/Event Format (Downhole TO Uphole/Debug)

Messages sent **FROM** the downhole unit use specific prefixes:

| Prefix | Direction       | Purpose                                  | Example (Illustrative for @vd)             |
|:-------|:----------------|:-----------------------------------------|:-------------------------------------------|
| `&`    | Downhole→Uphole | **Response** to a specific command       | `&vd,config,thresh,ack`                    |
| `!`    | Downhole→Uphole | **Asynchronous Alert/Event**             | `!vd,flag,1,120,85` (sector,size_mm,conf%) |
| `$`    | Downhole→Uphole | **Debug Message** (UART_DEBUG primarily) | `$db,Void: Sensor 0 processing data`       |

**Specific Void Detection Event:**

* `!vd,flag,<sector_idx>,<size_mm>,<confidence_%>` - Indicates a confirmed void detection. (prv\_void\_report\_detection)

-----

## 5. Void Detection Methods

### 5.1. Simplified POC Architecture

**Basic Data Flow for Void Detection:**

The void detection system follows a straightforward approach optimized for proof of concept implementation:

```c
// Basic data flow structures for POC
typedef struct {
    float raw_distance_m;     // Raw distance from radar sensor
    float raw_snr_db;         // Signal-to-noise ratio
    uint8_t sensor_id;        // Which sensor (0-2)
    uint32_t timestamp_ms;    // When data was received
} raw_radar_data_t;

typedef struct {
    uint16_t distance_mm;     // Cleaned distance in millimeters
    uint8_t snr;              // Cleaned SNR value
    uint8_t sensor_id;        // Sensor identifier
    bool is_valid;            // Data validity flag
    uint32_t timestamp_ms;    // Processing timestamp
} cleaned_radar_data_t;

typedef struct {
    bool void_detected;       // Simple void detection flag
    uint8_t affected_sensor;  // Which sensor detected the void
    uint16_t void_size_mm;    // Size of detected void
    uint8_t confidence;       // Detection confidence (0-100)
    uint32_t detection_time;  // When void was detected
} void_analysis_result_t;
```

### 5.2. Simplified Processing Flow

**POC Implementation Steps:**

1. **Data Input Stage:**
   * Raw radar data arrives via CAN bus
   * Store in `raw_radar_data_t` structure
   * Simple timestamp and sensor ID tracking

2. **Data Cleanup Stage:**
   * Convert meters to millimeters for precision
   * Apply basic range validation
   * Store in `cleaned_radar_data_t` structure
   * Flag invalid readings

3. **Void Analysis Stage:**
   * Compare cleaned distances against expected values
   * Apply simple threshold-based detection
   * Calculate basic confidence metrics
   * Store results in `void_analysis_result_t` structure

4. **Command Response Stage:**
   * Uphole sends `@vd` commands
   * Retrieve latest analysis results
   * Format and send response data

**Simplified Algorithm Logic:**

```c
// Basic void detection logic for POC
void simple_void_detection(cleaned_radar_data_t sensors[3], void_analysis_result_t* result) {
    uint16_t expected_distance = 150; // mm, configurable baseline
    uint16_t threshold = 50;          // mm, detection threshold
    
    for (int i = 0; i < 3; i++) {
        if (sensors[i].is_valid) {
            if (sensors[i].distance_mm > (expected_distance + threshold)) {
                result->void_detected = true;
                result->affected_sensor = i;
                result->void_size_mm = sensors[i].distance_mm - expected_distance;
                result->confidence = calculate_simple_confidence(sensors[i]);
                break;
            }
        }
    }
}
```

-----

## 6. Detailed Design and Implementation

### 6.1. POC Data Structures and Flow

**Simplified Data Architecture:**

The POC implementation uses a straightforward data flow with clearly defined stages and structures:

```c
// Stage 1: Raw Data Input (from CAN)
typedef struct {
    float raw_distance_m[3];    // Raw distances from 3 sensors
    float raw_snr_db[3];        // SNR values from 3 sensors
    uint32_t timestamp_ms;      // When data set was received
    bool sensor_active[3];      // Which sensors provided data
} radar_input_data_t;

// Stage 2: Cleaned Data (after radar module processing)
typedef struct {
    uint16_t clean_distance_mm[3];  // Cleaned distances in mm
    uint8_t clean_snr[3];           // Processed SNR values
    uint32_t process_time_ms;       // When cleaning was completed
    bool data_valid[3];             // Validity flags per sensor
} radar_cleaned_data_t;

// Stage 3: Void Analysis Results
typedef struct {
    bool void_present;              // Primary detection flag
    uint8_t void_sector;            // Which sector has void (0-2)
    uint16_t void_magnitude_mm;     // Size/severity of void
    uint8_t detection_confidence;   // Confidence level (0-100)
    uint32_t analysis_time_ms;      // When analysis completed
    char status_text[32];           // Human-readable status
} void_detection_result_t;
```

### 6.2. Simplified Processing Modules

#### Module 1: Data Input Handler

```c
// Function: Receive raw radar data from CAN
bool receive_radar_data(radar_input_data_t* input_data);

// Function: Store raw data in input structure
void store_raw_data(radar_input_data_t* data);
```

#### Module 2: Radar Data Cleanup

```c
// Function: Clean and validate raw sensor data
bool cleanup_radar_data(radar_input_data_t* raw, radar_cleaned_data_t* cleaned);

// Function: Convert meters to millimeters with validation
uint16_t convert_to_millimeters(float distance_m);

// Function: Validate sensor readings
bool validate_sensor_data(float distance, float snr);
```

#### Module 3: Void Analysis Engine

```c
// Function: Perform basic void detection analysis
bool analyze_for_voids(radar_cleaned_data_t* cleaned, void_detection_result_t* result);

// Function: Calculate detection confidence
uint8_t calculate_confidence(radar_cleaned_data_t* data, uint8_t sensor_idx);

// Function: Determine void characteristics
void characterize_void(uint16_t distance_mm, void_detection_result_t* result);
```

#### Module 4: Command Response Handler

```c
// Function: Handle @vd commands from uphole
void handle_void_command(const char* command);

// Function: Retrieve latest void analysis results
void get_latest_void_results(void_detection_result_t* result);

// Function: Format and send response to uphole
void send_void_response(void_detection_result_t* result);
```

### 6.3. Basic Implementation Flow

**Main Processing Loop:**

```c
void void_detection_main_loop(void) {
    static radar_input_data_t raw_data;
    static radar_cleaned_data_t cleaned_data;
    static void_detection_result_t void_results;
    
    // Step 1: Check for new radar data
    if (receive_radar_data(&raw_data)) {
        
        // Step 2: Clean and process the data
        if (cleanup_radar_data(&raw_data, &cleaned_data)) {
            
            // Step 3: Perform void analysis
            if (analyze_for_voids(&cleaned_data, &void_results)) {
                
                // Step 4: Results are stored and ready for commands
                // (Command handler will retrieve when needed)
            }
        }
    }
    
    // Step 5: Process any incoming commands
    // (Handled separately by command processor)
}
```

**Command Processing Flow:**

```c
void process_void_command(const char* cmd) {
    void_detection_result_t latest_results;
    
    // Get the most recent analysis results
    get_latest_void_results(&latest_results);
    
    // Format response based on command type
    if (strstr(cmd, "@vd,status")) {
        send_status_response(&latest_results);
    }
    else if (strstr(cmd, "@vd,data")) {
        send_data_response(&latest_results);
    }
    // Additional command handlers as needed
}
```

-----

## 7. Embedded System Considerations

### 7.1. Real-Time Requirements

**Deterministic Operation:**

* **Fixed polling cycle:** `RADAR_POLL_INTERVAL_MS` (100ms) in `vmt_common_defs.h` drives radar data requests.
* **Bounded execution time:** Algorithms primarily use integer arithmetic.
* **Interrupt priority management:** Standard ARM NVIC priorities (details not specified but assumed). CAN RX ISR (`HAL_CAN_RxFifo0MsgPendingCallback`) aims for minimal processing.
* **No blocking operations:** CAN operations are non-blocking (`HAL_CAN_AddTxMessage`, `HAL_CAN_GetRxMessage` used within callbacks or with polling).

```c
// Timing constraints (from vmt_common_defs.h and mti_void.h)
#define RADAR_POLL_INTERVAL_MS           100    // 10Hz update rate
// MAX_PROCESSING_TIME_MS, CAN_ISR_MAX_EXECUTION_TIME_US - Not explicitly defined, but implied by design.
#define CAN_COMM_TIMEOUT_MS              2000   // Used in mti_can.c for bus activity
```

### 7.2. Memory Management

**Static allocation following BARR-C guidelines:**

* Module variables like `prv_void_latest_measurements`, `prv_void_wall_profiles_history` are statically sized arrays.
* No evidence of `malloc` or `free` in the provided core logic files (`mti_*.c`, `vmt_*.c`).

```c
// Static arrays in mti_void.c
static wall_measurement_t prv_void_latest_measurements[MAX_SENSORS]; //
static wall_profile_t     prv_void_wall_profiles_history[WALL_HISTORY_SIZE]; //
// static void_detection_t   prv_void_detections_history[MAX_VOID_DETECTIONS]; // Defined but usage commented as TODO

// Memory usage constants not explicitly defined in code, but can be estimated from static array sizes.
```

### 7.3. Power Management

**Managed by `vmt_power.c` module:**

* `vmt_power_enter_sleep_mode()` handles peripheral shutdown (UARTs, IMU sampling via `vmt_imu_enable_sampling(false)`, ADC) and system clock adjustments for sleep.
* Wake-up reinitializes clocks and peripherals.
* `vmt_power_process()` can handle auto-sleep based on `last_activity_time`.

```c
// Example from vmt_power_enter_sleep_mode()
// HAL_SuspendTick();
// HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
// SystemClock_Config(); // On wake
// HAL_ResumeTick(); // On wake
```

### 7.4. Hardware Abstraction

* **STM32 HAL:** Used extensively for peripheral control (CAN, UART, SPI, ADC, GPIO, TIM).
* **VMT Drivers:** Modules like `vmt_adc.c`, `vmt_spi.c`, `vmt_uart.c` provide a higher level of abstraction over HAL for specific device needs.
* **Sensor Interface:** `mti_can.c` serves as the interface to the radar sensors. `vmt_icm20948.c` interfaces with the IMU hardware.
* **Temperature Interface:** `mti_temp.c` provides simplified integer-based temperature monitoring with basic threshold detection.

```c
// Temperature interface (simplified POC approach - integers only)
typedef struct {
    int16_t temperature_c;        // Temperature in Celsius (signed integer)
    bool data_valid;              // Data validity flag
    uint32_t timestamp_ms;        // When reading was taken
} temp_raw_data_t;

typedef struct {
    int16_t temperature_c;        // Cleaned temperature value in Celsius
    bool temp_high_flag;          // Over temperature warning
    bool temp_low_flag;           // Under temperature warning
    bool data_valid;              // Cleaned data validity
    uint32_t process_time_ms;     // When processing completed
} temp_processed_data_t;

typedef struct {
    int16_t current_temperature;  // Latest temperature reading in Celsius
    bool high_temp_alert;         // Combined high temperature alert
    bool low_temp_alert;          // Combined low temperature alert
    bool system_ready;            // Temperature system operational
    uint32_t last_update_ms;      // Last successful update
} temp_status_t;
```

-----

## 8. Implementation Status

### 8.1. Current POC Architecture Status

**Simplified Data Flow Implementation Progress:**

| Stage                    | Component                     | Status     | Description                                                              |
|:-------------------------|:------------------------------|:-----------|:-------------------------------------------------------------------------|
| **Stage 1: Data Input** | CAN Communication             | ✅ 98%     | Raw radar data reception functional                                      |
|                          | Raw Data Structures           | ✅ 100%    | Input data structures defined in `radar_data_t`                          |
|                          | Data Storage                  | ✅ 100%    | Raw data buffering implemented in `multi_radar_system_t`                 |
| **Stage 1.5: Temp Mod** | Temperature Module            | ✅ 100%    | **COMPLETED** - Full temperature sensing, processing, and command interface |
| **Stage 2: Cleanup**    | Radar Cleanup Module          | ✅ 95%     | **COMPLETED** - Data cleaning logic in `radar_process_measurement()`       |
|                          | Cleaned Data Structures       | ✅ 100%    | `radar_measurement_t` structure implemented                              |
|                          | Validation Logic              | ✅ 90%     | **COMPLETED** - SNR and distance validation implemented                  |
| **Stage 3: Analysis**   | Void Detection Module         | ❌ 5%      | **CRITICAL** - Skeleton [`mti_void.c`](../Device/Src/mti_void.c) exists, core logic missing       |
|                          | Analysis Result Structures    | ✅ 70%     | Result data structures defined, may need refinement                      |
|                          | Confidence Calculation       | ❌ 0%      | **NEEDS IMPLEMENTATION** - Confidence logic missing                      |
| **Stage 4: Commands**   | Command Framework             | ✅ 90%     | Basic command processing functional                                      |
|                          | Temp Command Handlers         | ✅ 100%    | **COMPLETED** - Temperature commands fully implemented                   |
|                          | Void Command Handlers         | ❌ 10%     | **NEEDS IMPLEMENTATION** - Placeholder exists in [`vmt_command.c`](../Device/Src/vmt_command.c)      |
|                          | Response Formatting           | ❌ 0%      | **NEEDS IMPLEMENTATION** - Void response logic missing                   |

### 8.2. Completed Implementation Details

#### ✅ Phase 1: Basic Data Flow - COMPLETED

The radar data cleanup and processing pipeline has been fully implemented:

#### Data Input & CAN Processing (`mti_can.c`)

```c
// Multi-sensor round-robin data collection
typedef struct {
    radar_data_t sensors[MAX_RADAR_SENSORS];           // Raw data from 3 sensors
    uint32_t     last_message_timestamp[MAX_RADAR_SENSORS];
    bool         sensor_online[MAX_RADAR_SENSORS];
} multi_radar_system_t;
```

**Radar Data Cleanup (`mti_radar.c`):**

```c
// Implemented in radar_process_measurement()
void radar_process_measurement(uint8_t sensor_idx, float detectedPoints[][2], uint8_t numPoints) {
    // 1. Reset measurement structure
    measurement->distance_mm = 0;
    measurement->data_valid = false;
    
    // 2. Find closest valid point with filtering
    for (uint8_t i = 0; i < numPoints; i++) {
        float distance_m = detectedPoints[i][0];
        float snr = detectedPoints[i][1];
        
        // 3. Validation: SNR > 100.0f and distance 5cm to 5m
        if (snr > 100.0f && distance_m > 0.05f && distance_m < 5.0f) {
            if (distance_m < closest_distance) {
                closest_distance = distance_m;
                found_valid = true;
            }
        }
    }
    
    // 4. Convert to millimeters and store
    if (found_valid) {
        measurement->distance_mm = (uint16_t)(closest_distance * 1000.0f);
        measurement->data_valid = true;
    }
}
```

**Cleaned Data Structures (`mti_radar.h`):**

```c
// Clean millimeter-precision storage
typedef struct {
    uint16_t distance_mm;  // Single wall distance in millimeters
    uint16_t angle_deg;    // Angular position (0°, 120°, 240°)
    bool     data_valid;   // Validation flag
} radar_measurement_t;

// Round-robin system state
typedef struct {
    uint8_t             current_sensor;                  // Active sensor (0-2)
    uint32_t            last_switch_time;                // Timing control
    radar_measurement_t measurements[MAX_RADAR_SENSORS]; // Clean data per sensor
    bool                system_running;                  // System state
} radar_round_robin_t;
```

**Data Access Functions:**

```c
// Clean getter functions for processed data
radar_measurement_t *radar_get_measurement(uint8_t sensor_idx);
bool                 radar_has_valid_data(uint8_t sensor_idx);
uint16_t             radar_get_distance_mm(uint8_t sensor_idx);
uint16_t             radar_get_angle_deg(uint8_t sensor_idx);
```

#### ✅ Phase 1.5: Temperature Module Implementation - COMPLETED

* [x] ✅ Created [`mti_temp.c`](../Device/Src/mti_temp.c) with complete temperature monitoring system.
* [x] ✅ Implemented `temp_get_raw_data()` function (ADC reading with conversion).
* [x] ✅ Implemented `temp_process_raw_data()` function (validation and smoothing).
* [x] ✅ Implemented `temp_update_flags()` function (threshold checking).
* [x] ✅ Created `temp_system_process()` main processing function.
* [x] ✅ Added temperature status structure management and configuration functions.
* [x] ✅ Integrated temperature module with system initialization ([`mti_system.c`](../Device/Src/mti_system.c)).
* [x] ✅ Implemented complete temperature command interface (`cmd_temp()` in [`vmt_command.c`](../Device/Src/vmt_command.c)).
* [x] ✅ Fixed linking issues and integrated into build system - builds with 0 errors.

### 8.3. Updated POC Development Plan

With Phase 1 and 1.5 complete, the focus is now on Void Detection.

#### ✅ Phase 1: Basic Data Flow Implementation Status - COMPLETED

#### ✅ Phase 1.5: Temperature Module Implementation Status - COMPLETED

#### Phase 2: Core Void Detection Module Implementation - CRITICAL PRIORITY

**Void Detection Module Implementation (Target: Weeks 1-2 of remaining schedule):**

* [ ] Populate [`mti_void.c`](../Device/Src/mti_void.c) with basic threshold-based void detection logic.
* [ ] Implement `void_get_cleaned_radar_data()` (or equivalent interface to [`mti_radar.c`](../Device/Src/mti_radar.c) data).
* [ ] Implement `void_analyze_sensor_data()` or `void_analyze_for_voids()` (simple threshold comparison per sensor, or combined).
* [ ] Implement `void_calculate_confidence()` (basic reliability scoring).
* [ ] Implement `void_characterize_detection()` (void size and sector identification).
* [ ] Create `void_system_process()` main processing function to orchestrate void detection.
* [ ] Integrate `void_system_process()` into the main system loop in [`mti_system.c`](../Device/Src/mti_system.c) or via [`vmt_device.c`](../Device/Src/vmt_device.c).
* [ ] Test core void detection logic with simulated or captured radar data.

#### Phase 3: Command Interface Implementation (Target: Week 2-3 of remaining schedule)

#### Temperature Commands - ✅ COMPLETED

* [x] ✅ Implemented `handle_temp_command()` function in [`vmt_command.c`](../Device/Src/vmt_command.c) (likely part of `cmd_temp`).
* [x] ✅ Added `@temp,status?` command handler.
* [x] ✅ Added `@temp,config,high,<value>` command handler.
* [x] ✅ Added `@temp,config,low,<value>` command handler.
* [x] ✅ Created temperature response formatting functions.
* [x] ✅ Tested temperature command integration.

#### Void Detection Commands

* [ ] Implement `handle_void_command` (or `cmd_void`) function in [`vmt_command.c`](../Device/Src/vmt_command.c).
* [ ] Add `@vd,status?` command handler (returns detection status and results from [`mti_void.c`](../Device/Src/mti_void.c)).
* [ ] Add `@vd,config,thresh,<value>` command handler (set detection threshold in [`mti_void.c`](../Device/Src/mti_void.c)).
* [ ] Add `@vd,config,baseline,<value>` command handler (set expected distance in [`mti_void.c`](../Device/Src/mti_void.c)).
* [ ] Create void detection response formatting functions.
* [ ] Test void detection command integration.

#### Phase 4: System Integration & Testing (Target: Week 3 of remaining schedule)

* [ ] Ensure void detection processing is correctly integrated into the radar data flow.
* [ ] Test combined system operation with all modules (Radar, Temp, Void, IMU, Water) active.
* [ ] Validate command responses from both temperature and void modules under various conditions.
* [ ] Perform end-to-end system testing with simulated and, if possible, live data.
* [ ] Document system behavior and performance characteristics.
* [ ] Address any remaining validation and testing tasks.

-----

## 8.4. Detailed Implementation Specifications

**Temperature Module (`mti_temp.c`) - ✅ COMPLETED**

```c
// Main temperature processing function (to be implemented)
void temp_system_process(void) {
    static temp_raw_data_t raw_data;
    static temp_processed_data_t processed_data;
    static temp_status_t current_status = {0};
    
    // Stage 1: Get temperature data (ADC conversion handled internally)
    if (temp_get_raw_data(&raw_data)) {
        
        // Stage 2: Process and clean data
        if (temp_process_raw_data(&raw_data, &processed_data)) {
            
            // Stage 3: Update flags and status
            temp_update_flags(&processed_data, &current_status);
            
            // Update current temperature for command access
            current_status.current_temperature = processed_data.temperature_c;
            current_status.last_update_ms = processed_data.process_time_ms;
            current_status.system_ready = true;
        }
    }
}

// Required implementation functions:
bool temp_get_raw_data(temp_raw_data_t* raw_data);
bool temp_process_raw_data(temp_raw_data_t* raw, temp_processed_data_t* processed);
void temp_update_flags(temp_processed_data_t* processed, temp_status_t* status);
void temp_get_latest_status(temp_status_t* status);

// Temperature command handlers:
void handle_temp_status_command(void);
void handle_temp_config_command(const char* params);
```

**Void Detection Module (`mti_void.c`) - Next Implementation Focus:**

```c
// Main void detection processing function (to be implemented)
void void_system_process(void) {
    static void_detection_result_t void_results = {0};
    
    // Get cleaned radar data from existing radar module
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        if (radar_has_valid_data(i)) {
            uint16_t distance_mm = radar_get_distance_mm(i);
            uint16_t angle_deg = radar_get_angle_deg(i);
            
            // Perform void analysis on this sensor's data
            if (void_analyze_sensor_data(i, distance_mm, angle_deg, &void_results)) {
                // Void detected - results stored in void_results structure
                // Command handlers can retrieve this data when requested
            }
        }
    }
}

// Required implementation functions:
bool void_analyze_sensor_data(uint8_t sensor_idx, uint16_t distance_mm, uint16_t angle_deg, void_detection_result_t* result);
uint8_t void_calculate_confidence(uint16_t distance_mm, uint16_t expected_mm, uint16_t threshold_mm);
void void_characterize_detection(uint16_t distance_mm, uint16_t expected_mm, void_detection_result_t* result);
void void_get_latest_results(void_detection_result_t* result);

// Void command handlers:
void handle_void_status_command(void);
void handle_void_config_command(const char* params);
```

**Simple Void Detection Algorithm (POC Implementation):**

```c
// Basic threshold-based void detection for POC
bool void_analyze_sensor_data(uint8_t sensor_idx, uint16_t distance_mm, uint16_t angle_deg, void_detection_result_t* result) {
    // Configuration values (to be made configurable via commands)
    static uint16_t expected_distance_mm = 150;  // Expected wall distance
    static uint16_t detection_threshold_mm = 50; // Void detection threshold
    
    // Simple threshold comparison
    if (distance_mm > (expected_distance_mm + detection_threshold_mm)) {
        // Void detected
        result->void_present = true;
        result->void_sector = sensor_idx;
        result->void_magnitude_mm = distance_mm - expected_distance_mm;
        result->detection_confidence = void_calculate_confidence(distance_mm, expected_distance_mm, detection_threshold_mm);
        result->analysis_time_ms = HAL_GetTick();
        
        // Generate alert event
        debug_send("!vd,flag,%d,%d,%d", sensor_idx, result->void_magnitude_mm, result->detection_confidence);
        return true;
    }
    
    return false;  // No void detected
}

// Simple confidence calculation
uint8_t void_calculate_confidence(uint16_t distance_mm, uint16_t expected_mm, uint16_t threshold_mm) {
    uint16_t excess_distance = distance_mm - expected_mm;
    
    if (excess_distance <= threshold_mm) {
        return 0;  // No detection
    }
    
    // Linear confidence scaling: more excess distance = higher confidence
    // Cap at 100% confidence
    uint32_t confidence = (excess_distance * 100) / (threshold_mm * 2);
    return (confidence > 100) ? 100 : (uint8_t)confidence;
}
```

### 8.6. Success Criteria and Validation

**Temperature Module Success Criteria:**

* ✅ Temperature readings in Celsius with ±2°C accuracy
* ✅ Configurable high/low temperature thresholds
* ✅ Alert flags functional for threshold violations
* ✅ Command interface responsive (`@temp,status?`, `@temp,config,*`)
* ✅ Integration with main system loop without timing issues

**Void Detection Module Success Criteria (Target):**

* ✅ Basic void detection using simple threshold algorithm
* ✅ Confidence calculation providing meaningful reliability scores
* ✅ Sector identification for multi-sensor void localization
* ✅ Command interface responsive (`@vd,status?`, `@vd,config,*`)
* ✅ Integration with existing radar data pipeline

**System Integration Success Criteria:**

* ✅ Both modules operating simultaneously without conflicts
* ✅ Command responses within 100ms for status queries
* ✅ System maintains 10Hz radar data processing rate
* ✅ No memory leaks or stack overflow issues
* ✅ Stable operation for extended periods (>1 hour continuous)

**POC Demonstration Goals:**

* ✅ Live temperature monitoring with threshold alerts
* ✅ Real-time void detection with configurable sensitivity
* ✅ Command-driven system configuration and status reporting
* ✅ Stable multi-sensor operation demonstrating system viability
* ✅ Foundation architecture ready for algorithm enhancement

### 8.7. Risk Assessment and Mitigation

**Implementation Risks:**

**High Risk:**

* **Void detection algorithms prove insufficient** - Mitigation: Start with simplest threshold approach, iterate
* **Integration timing issues with main loop** - Mitigation: Careful timing analysis, optimize critical paths
* **Command interface complexity** - Mitigation: Implement core commands first, add features incrementally

**Medium Risk:**

* **Temperature sensor calibration issues** - Mitigation: Use known sensor characteristics, add calibration offset
* **Memory usage exceeding constraints** - Mitigation: Monitor stack/heap usage, optimize data structures
* **CAN bus integration complications** - Mitigation: Leverage existing working radar infrastructure

**Low Risk:**

* **Minor timing variations** - Mitigation: Built-in timing margins, non-critical for POC
* **Command parsing edge cases** - Mitigation: Focus on primary use cases, handle errors gracefully

**Timeline Contingencies:**

* If void detection algorithm implementation takes longer than 2 weeks, prioritize basic threshold detection and status reporting over advanced features or full configuration command set for the initial POC.
* If command interface for void detection proves complex, implement status query (`@vd,status?`) first, then essential configuration commands

#### Estimated Timeline: 3 weeks to functional POC (from June 5, 2025)

* Week 1-2: Core Void Detection module implementation and unit testing.
* Week 2-3: Void Detection command interface implementation and integration.
* Week 3: Full system integration testing, optimization, and POC demonstration.

This plan provides a realistic, achievable path to a functional proof of concept for void detection, building upon the completed temperature module and existing infrastructure.

-----

## 12. Next Steps

With the temperature sensing module successfully implemented and integrated, the immediate priority is the development of the core void detection functionality.

### 12.1. Immediate POC Implementation Tasks (Next 3 Weeks)

The primary focus is to complete Phase 2, 3 (Void parts), and 4 of the Updated POC Development Plan (Section 8.3). Key tasks include:

#### Priority 1: Core Void Detection Algorithm (Target: Weeks 1-2)

* **Implement `mti_void.c` processing loop (`void_system_process`):**
  * Fetch cleaned radar data from the `mti_radar` module.
  * Call analysis functions for each relevant sensor reading.
  * Store detection results.
* **Implement `void_analyze_sensor_data` (or similar):**
  * Apply a simple threshold-based algorithm against a configurable baseline/expected distance.
  * Populate the `void_detection_result_t` structure.
* **Implement `void_calculate_confidence`:**
  * Develop a basic confidence score based on deviation from baseline and/or SNR (if available and relevant).
* **Implement `void_characterize_detection`:**
  * Determine void magnitude (size) and affected sensor/sector.
* **Data Structures:** Refine and utilize `void_measurement_t`, `void_status_t`, and `void_detection_result_t` as defined in [`mti_void.h`](../Device/Inc/mti_void.h) and section [6.1 POC Data Structures and Flow](#61-poc-data-structures-and-flow). Ensure proper initialization and state management.

#### Priority 2: Void Detection Command Interface (Target: Week 2-3)

* **Implement `handle_void_command` (or `cmd_void`) in `vmt_command.c`:**
  * Handler for `@vd,status?` to retrieve and format data from `mti_void.c`.
  * Handlers for `@vd,config,thresh,<value>` and `@vd,config,baseline,<value>` to update settings in `mti_void.c`.
* **Response Formatting:** Develop functions to send void detection status and configuration acknowledgements.
* **Event Reporting:** Implement `!vd,flag,...` asynchronous alert if a void is detected, as per section [4.2.2 Response/Event Format](#422-responseevent-format-downhole-to-upholedebug).

#### Priority 3: System Integration and Testing (Target: Week 3)

* Integrate `void_system_process()` into the main application loop in [`mti_system.c`](../Device/Src/mti_system.c) or [`vmt_device.c`](../Device/Src/vmt_device.c).
* Conduct thorough testing focusing on void detection specific scenarios.
* Verify interactions between the void detection module and other system components (CAN, UART, Radar, Temperature).

-----

## 13. Current Implementation Status Summary

### 13.1. Executive Summary

#### System Infrastructure: 95% Complete ✅

The STM32F722 void detection system has successfully implemented most of its foundational infrastructure:

* **CAN Communication System (98%)**: Fully functional with 3 radar sensor support, error handling, and recovery mechanisms
* **Radar Management (95%)**: Round-robin sensor polling, data acquisition, and processing framework
* **System Initialization (98%)**: Multi-step startup sequence with proper peripheral initialization
* **IMU Processing (70%)**: Dual ICM20948 sensors with motion detection and angle calculations
* **Water Detection (85%)**: Dual ADC sensors with adaptive thresholds and event reporting
* **Temperature Monitoring (100%)**: Complete ADC-based temperature module with smoothing, thresholds, and command interface - **FULLY IMPLEMENTED**
* **Hardware Abstraction (95%)**: BARR-C compliant peripheral wrappers for ADC, SPI, UART, Flash
* **Memory Management (100%)**: Static allocation throughout, no dynamic memory usage
* **Debug Infrastructure (100%)**: Comprehensive DEBUG_SEND logging system
* **Build System (100%)**: Successfully compiles with 0 errors, all linking issues resolved

#### Void Detection Algorithms: 5% Complete ⚠️

The core void detection functionality, which is the primary purpose of this system, requires significant implementation:

* **Core Algorithm File**: [`mti_void.c`](../Device/Src/mti_void.c) exists as a skeleton; core algorithms (thresholding, confidence, characterization) need to be implemented.
* **Circle Fitting**: 3-point geometric calculation algorithms not implemented (Advanced feature, likely post-POC).
* **Baseline Detection**: Primary void detection method (threshold against baseline) needs implementation.
* **Multi-Point Processing**: Sensor data fusion algorithms for void detection not implemented (Advanced feature, basic per-sensor analysis for POC).
* **Confidence Calculation**: Void detection reliability scoring needs implementation.
* **Data Conversion**: Radar sensor data to millimeter wall measurements is **COMPLETED** in [`mti_radar.c`](../Device/Src/mti_radar.c). [`mti_void.c`](../Device/Src/mti_void.c) will consume this.

#### Command Interface: 60% Complete ✅ (Overall - Temp commands are 100%, Void commands 10%)

* **Framework**: Extensible command system exists in [`vmt_command.c`](../Device/Src/vmt_command.c).
* **Temperature Commands**: Fully implemented and tested.
* **Void Commands**: `@vd` commands have placeholder implementation or are missing; need full implementation for status and basic configuration.
* **Design**: Full command specification exists in documentation, needs implementation for void commands.

### 13.2. Critical Path to POC Completion

**Immediate Blockers (CRITICAL PRIORITY - Next 3 Weeks):**

1. **Implement `mti_void.c` algorithms** - Threshold-based detection, confidence calculation, void characterization using data from `mti_radar`.
2. **Complete `@vd` command interface in `vmt_command.c`** - Implement handlers for status query and basic configuration of void detection parameters.
3. **Integrate `void_system_process`** into the main system loop.
4. **End-to-end testing** - Verify temperature and void systems work together, and validate void detection with simulated/real data.

**Estimated Development Effort (Remaining for POC):**

* Core void detection algorithms (`mti_void.c`): 1-2 weeks
* Void command interface completion (`vmt_command.c`): 0.5-1 week
* Integration and system testing: 0.5-1 week
* **Total: Approximately 3 weeks to POC completion**

### 13.3. System Readiness Assessment

**Ready for Integration ✅:**

* **CAN communication with radar sensors**
* **System initialization and peripheral management**
* **Command processing framework**
* **Debug and logging infrastructure**
* **Data structure definitions and interfaces**
* **Complete temperature monitoring system with ADC, processing, and commands**
* **Working build system with all linking issues resolved**
* **Cleaned radar data (mm precision) available from `mti_radar.c`**

**Requires Implementation ❌ (Focus for next 3 weeks):**

* All void detection processing algorithms in the [`mti_void.c`](../Device/Src/mti_void.c) module.
* Void detection confidence and reliability assessment logic.
* Full void-specific command handlers (`@vd` commands in [`vmt_command.c`](../Device/Src/vmt_command.c)).
* Integration of void processing into the main system loop and end-to-end testing.

**Conclusion:**

The system has excellent foundational infrastructure, including a fully functional temperature monitoring system and a robust radar data processing pipeline. The proof of concept is approximately **70% complete overall** (infrastructure and temp sensor are high, void detection is low). The primary remaining work is implementing the core void detection algorithms and their associated command interface to achieve a functional prototype. The path to POC completion is clear and estimated at 3 weeks.

-----

## 14. Appendix: Detailed Command/Response Timing Analysis

### 14.1. Timing Requirements Overview

**Key Timing Constraints:**

* `RADAR_POLL_INTERVAL_MS`: 100ms (10Hz) - Defines the maximum rate for radar data polling and processing.
* `CAN_COMM_TIMEOUT_MS`: 2000ms - Timeout for CAN communication before declaring a sensor as unresponsive.
* `KEEPALIVE_TIMEOUT_MS`: 500ms - Interval for sending keep-alive messages to uphole system.

**Critical Timing Considerations:**

* Void detection processing must complete within the `RADAR_POLL_INTERVAL_MS` to ensure timely responses to uphole commands.
* Any blocking operations or long computations in the main loop could delay the handling of incoming commands or sensor data, leading to missed deadlines.
* The system must be able to recover from errors (e.g., CAN bus off, sensor timeouts) within the `CAN_COMM_TIMEOUT_MS` to maintain communication integrity.

### 14.2. Command/Response Timing Analysis

**Command Processing Timing:**

* Command reception and parsing: < 10ms
* Response formulation: < 5ms
* Transmission via UART: Dependent on baud rate (e.g., 115200 bps → 1 byte = 69.44µs)

#### Example: @vd,status? Command

1. Received `@vd,status?` command via UART.
2. Parsed and identified as a request for void detection status.
3. Latest void detection results retrieved (assume latest processing cycle completed).
4. Formulated response: `&vd,status,1,0,radar_hw_status`.
5. Transmitted response via UART.

**Timing Considerations for Commands:**

* Commands that require immediate action (e.g., `@sleep`, `@st`) should be processed within 10ms.
* Status inquiry commands (e.g., `@vd,status?`) should have responses formulated within 5ms if the data is already available.
* Any command that triggers a state change (e.g., entering sleep mode) must ensure that the system can safely resume and process subsequent commands without manual intervention.

### 14.3. Data Processing Timing

**Data Processing Stages:**

1. Data reception from CAN: Interrupt-driven, should be processed in the ISR or immediately after.
2. Data cleanup and validation: Target < 20ms for all sensors.
3. Void analysis: Target < 50ms for basic analysis, < 100ms for extended analysis with confidence scoring.
4. Command response preparation: Target < 10ms.

#### Example: Data Processing Flow

1. Radar data received via CAN interrupt.
2. `receive_radar_data()` called, processing all active sensors.
3. For each sensor, `cleanup_radar_data()` is called:
    * Meter to millimeter conversion.
    * Basic range and SNR validation.
4. Validated data passed to `analyze_for_voids()`:
    * Threshold-based void detection.
    * Confidence calculation.
5. Results stored in void detection result structure.
6. If `@vd` command received, `send_void_response()` is called to format and send the response.

**Processing Timing Considerations:**

* The entire data processing flow from reception to void analysis should ideally complete within the `RADAR_POLL_INTERVAL_MS` to ensure real-time performance.
* Any delays in data processing directly impact the system's ability to respond to uphole commands in a timely manner.
* Optimization efforts should focus on reducing the processing time of the void detection algorithms and ensuring efficient data handling.

### 14.4. Error Handling and Recovery Timing

**Error Detection and Recovery:**

* CAN bus errors detected via `HAL_CAN_ErrorCallback`, recovery attempts made within the callback.
* Sensor timeouts detected in `can_process`, with recovery actions (e.g., sensor reset) triggered based on the timeout count.

#### Example: CAN Error Recovery

1. CAN bus error detected (e.g., bus-off state).
2. `HAL_CAN_ErrorCallback` invoked.
3. Error recovery sequence initiated:
    * Stop CAN transmission.
    * Reset CAN peripheral.
    * Restart CAN and re-enable notifications.
4. If recovery successful, resume normal operation.

**Timing Considerations for Error Handling:**

* Error recovery procedures should be designed to execute within a few milliseconds to minimize system downtime.
* The system should be able to recover from transient errors (e.g., single CAN bus-off) automatically.
* Persistent errors (e.g., repeated sensor timeouts) may require manual intervention, but the system should attempt to recover autonomously first.

-----

## 15. Appendix: Detailed Data Structure Definitions

### 15.1. Raw Radar Data Structure

```c
// Raw radar data structure (Stage 1)
typedef struct {
    float raw_distance_m[3];    // Raw distances from 3 sensors
    float raw_snr_db[3];        // SNR values from 3 sensors
    uint32_t timestamp_ms;      // When data set was received
    bool sensor_active[3];      // Which sensors provided data
} radar_input_data_t;
```

### 15.2. Cleaned Radar Data Structure

```c
// Cleaned radar data structure (Stage 2)
typedef struct {
    uint16_t clean_distance_mm[3];  // Cleaned distances in mm
    uint8_t clean_snr[3];           // Processed SNR values
    uint32_t process_time_ms;       // When cleaning was completed
    bool data_valid[3];             // Validity flags per sensor
} radar_cleaned_data_t;
```

### 15.3. Void Detection Result Structure

```c
// Void detection result structure (Stage 3)
typedef struct {
    bool void_present;              // Primary detection flag
    uint8_t void_sector;            // Which sector has void (0-2)
    uint16_t void_magnitude_mm;     // Size/severity of void
    uint8_t detection_confidence;   // Confidence level (0-100)
    uint32_t analysis_time_ms;      // When analysis completed
    char status_text[32];           // Human-readable status
} void_detection_result_t;
```

### 15.4. Multi-Sensor System State Structure

```c
// Multi-sensor round-robin system state
typedef struct {
    uint8_t             current_sensor;                  // Active sensor (0-2)
    uint32_t            last_switch_time;                // Timing control
    radar_measurement_t measurements[MAX_RADAR_SENSORS]; // Clean data per sensor
    bool                system_running;                  // System state
} radar_round_robin_t;
```

### 15.5. Temperature Data Structures

```c
// Raw temperature data structure
typedef struct {
    int16_t temperature_c;        // Temperature in Celsius (signed integer)
    bool data_valid;              // Data validity flag
    uint32_t timestamp_ms;        // When reading was taken
} temp_raw_data_t;

typedef struct {
    int16_t temperature_c;        // Cleaned temperature value in Celsius
    bool temp_high_flag;          // Over temperature warning
    bool temp_low_flag;           // Under temperature warning
    bool data_valid;              // Cleaned data validity
    uint32_t process_time_ms;     // When processing completed
} temp_processed_data_t;

typedef struct {
    int16_t current_temperature;  // Latest temperature reading in Celsius
    bool high_temp_alert;         // Combined high temperature alert
    bool low_temp_alert;          // Combined low temperature alert
    bool system_ready;            // Temperature system operational
    uint32_t last_update_ms;      // Last successful update
} temp_status_t;
```
