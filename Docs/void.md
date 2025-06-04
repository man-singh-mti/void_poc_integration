# Void Detection System: Downhole Module Implementation Guide

**Version:** 1.3.1
**Date:** May 27, 2025
**Status:** Active Development

## Table of Contents

1. [Introduction](#1-introduction)
    1.1. [Purpose](#11-purpose)
    1.2. [Scope](#12-scope)
    1.3. [Target Audience](#13-target-audience)
    1.4. [Definitions and Acronyms](#14-definitions-and-acronyms)
2. [Executive Summary](#2-executive-summary)
3. [System Overview](#3-system-overview)
    3.1. [Core Functionality](#31-core-functionality)
    3.2. [Simplified Data Flow Architecture](#32-simplified-data-flow-architecture)
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
9. [Safety and Reliability](#9-safety-and-reliability)
    9.1. [Fault Detection and Recovery](#91-fault-detection-and-recovery)
    9.2. [Watchdog and System Health](#92-watchdog-and-system-health)
    9.3. [Communication Robustness](#93-communication-robustness)
10. [Future Enhancements](#10-future-enhancements)
11. [Development Guidelines](#11-development-guidelines)
12. [Next Steps](#12-next-steps)
13. [Current Implementation Status Summary](#13-current-implementation-status-summary)

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
* ⚠️ **Void Detection Logic:** Core algorithms using millimeter precision are implemented (`mti_void.c`). Refinement and extended testing for edge cases are ongoing.
* ⚠️ **Error Recovery:** Basic error handling (e.g., CAN sensor resets) is in place. Advanced system-wide recovery mechanisms are under development.

**Design Principles:**

* **Deterministic Operation:** Fixed-time processing cycles for real-time requirements.
* **Fail-Safe Design:** Graceful degradation when sensors fail (e.g., `void_get_subsystem_hw_status`).
* **Memory Efficiency:** Primarily static allocation; no dynamic memory usage in critical paths.
* **Modularity:** Clear interfaces between hardware abstraction (e.g., `vmt_adc`, `vmt_spi`), drivers (`mti_can`), and application logic (`mti_void`, `mti_imu`).

-----

## 3. System Overview

### 3.1. Core Functionality

The system performs continuous real-time monitoring of borehole conditions through a simplified data flow architecture:

1. **Raw Data Input:** Radar sensor data flows into the system via CAN bus
2. **Data Storage:** Raw sensor data is stored in input data structures  
3. **Data Cleaning:** Radar modules process and clean the raw data
4. **Cleaned Data Storage:** Processed data is stored in cleaned data structures
5. **Void Analysis:** Void detection module performs logic analysis on cleaned data
6. **Results Storage:** Analysis results are stored in separate result structures
7. **Command Response:** When uphole commands arrive, functions retrieve latest data and respond

### 3.2. Simplified Data Flow Architecture

**Basic Structure for POC Implementation:**

```ascii
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Raw Radar     │    │    Radar        │    │   Void          │
│   Data Input    │───▶│   Cleanup       │───▶│   Analysis      │
│   (CAN Bus)     │    │   Module        │    │   Module        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Raw Data      │    │   Cleaned       │    │   Analysis      │
│   Struct        │    │   Data Struct   │    │   Results       │
│                 │    │                 │    │   Struct        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                        │
                                                        ▼
                                               ┌─────────────────┐
                                               │   Command       │
                                               │   Handler       │
                                               │   (Uphole)      │
                                               └─────────────────┘
```

**Key Design Principles for POC:**

* **Simple Data Flow:** Clear progression from input → cleanup → analysis → results
* **Structured Storage:** Each stage uses dedicated data structures
* **Modular Processing:** Separate modules handle distinct responsibilities
* **Command-Driven Output:** Results are retrieved only when requested
* **Basic Functionality:** Focus on core void detection without complex features

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

| Stage                    | Component                     | Status     | Description                                              |
|:-------------------------|:------------------------------|:-----------|:---------------------------------------------------------|
| **Stage 1: Data Input** | CAN Communication             | ✅ 98%     | Raw radar data reception functional                      |
|                          | Raw Data Structures           | ✅ 100%    | Input data structures defined in `radar_data_t`         |
|                          | Data Storage                  | ✅ 100%    | Raw data buffering implemented in `multi_radar_system_t`|
| **Stage 2: Cleanup**    | Radar Cleanup Module          | ✅ 95%     | **COMPLETED** - Data cleaning logic in `radar_process_measurement()` |
|                          | Cleaned Data Structures       | ✅ 100%    | `radar_measurement_t` structure implemented             |
|                          | Validation Logic              | ✅ 90%     | **COMPLETED** - SNR and distance validation implemented |
| **Stage 3: Analysis**   | Void Detection Module         | ❌ 0%      | **CRITICAL** - Core analysis logic missing              |
|                          | Analysis Result Structures    | ✅ 100%    | Result data structures defined                           |
|                          | Confidence Calculation       | ❌ 0%      | **NEEDS IMPLEMENTATION** - Confidence logic missing     |
| **Stage 4: Commands**   | Command Framework             | ✅ 90%     | Basic command processing functional                      |
|                          | Void Command Handlers         | ❌ 20%     | **NEEDS IMPLEMENTATION** - Only placeholder exists      |
|                          | Response Formatting           | ❌ 0%      | **NEEDS IMPLEMENTATION** - Response logic missing       |

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

### 8.3. Updated POC Development Plan

#### ✅ Phase 1: Basic Data Flow Implementation - COMPLETED

* [x] ✅ Implement basic radar data cleanup module (`radar_process_measurement()` in `mti_radar.c`)
* [x] ✅ Add meter-to-millimeter conversion with validation (integer conversion implemented)
* [x] ✅ Create simple data validation functions (SNR > 100.0f, distance 5cm-5m range)
* [x] ✅ Add cleaned data structure population (`radar_measurement_t` structures)

#### Phase 2: Core Module Implementation (Week 1-2) - CRITICAL PRIORITY

#### Temperature Module Implementation (Week 1)

* [ ] Create `mti_temp.c` with simplified integer-based temperature monitoring
* [ ] Implement `temp_get_raw_data()` function (ADC reading with internal conversion)
* [ ] Implement `temp_process_raw_data()` function (validation and cleaning)
* [ ] Implement `temp_update_flags()` function (threshold checking)
* [ ] Create `temp_system_process()` main processing function
* [ ] Add temperature status structure management
* [ ] Integrate with existing ADC infrastructure in `vmt_adc.c`

**Void Detection Module Implementation (Week 2):**

* [ ] Create `mti_void.c` with basic threshold-based void detection
* [ ] Implement `void_get_cleaned_radar_data()` (interface to radar module)
* [ ] Implement `void_analyze_for_voids()` (simple threshold comparison)
* [ ] Implement `void_calculate_confidence()` (basic reliability scoring)
* [ ] Implement `void_characterize_detection()` (void size and sector identification)
* [ ] Create `void_system_process()` main processing function
* [ ] Integrate with completed radar data cleanup pipeline

#### Phase 3: Command Interface Implementation (Week 3)

#### Temperature Commands

* [ ] Implement `handle_temp_command()` function in `vmt_command.c`
* [ ] Add `@temp,status?` command handler (returns temperature and alert flags)
* [ ] Add `@temp,config,high,<value>` command handler (set high threshold)
* [ ] Add `@temp,config,low,<value>` command handler (set low threshold)
* [ ] Create temperature response formatting functions
* [ ] Test temperature command integration with existing framework

#### Void Detection Commands

* [ ] Implement `handle_void_command()` function in `vmt_command.c`
* [ ] Add `@vd,status?` command handler (returns detection status and results)
* [ ] Add `@vd,config,thresh,<value>` command handler (set detection threshold)
* [ ] Add `@vd,config,baseline,<value>` command handler (set expected distance)
* [ ] Create void detection response formatting functions
* [ ] Test void detection command integration

#### Phase 4: System Integration & Testing (Week 4)

* [ ] Integrate temperature monitoring into main system loop (`mti_system.c`)
* [ ] Integrate void detection processing into radar data flow
* [ ] Test combined system operation with all modules active
* [ ] Validate command responses from both temperature and void modules
* [ ] Perform end-to-end system testing with simulated and live data
* [ ] Document system behavior and performance characteristics

### 8.4. Detailed Implementation Specifications

**Temperature Module (`mti_temp.c`) - Week 1 Implementation:**

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

**Void Detection Module (`mti_void.c`) - Week 2 Implementation:**

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

### 8.5. Integration Points and Dependencies

**Temperature Module Integration:**

* **Dependency:** `vmt_adc.c` - ADC reading functions
* **Integration Point:** `mti_system.c` - Add `temp_system_process()` to main loop
* **Command Integration:** `vmt_command.c` - Add temperature command handlers

**Void Detection Module Integration:**

* **Dependency:** `mti_radar.c` - Cleaned radar data access functions
* **Integration Point:** `mti_system.c` - Add `void_system_process()` to main loop  
* **Command Integration:** `vmt_command.c` - Add void detection command handlers

**System Integration Sequence:**

1. Implement and test temperature module independently
2. Implement and test void detection module independently
3. Integrate both modules into main system loop
4. Test command interfaces for both modules
5. Perform full system integration testing

### 8.6. Success Criteria and Validation

**Temperature Module Success Criteria:**

* ✅ Temperature readings in Celsius with ±2°C accuracy
* ✅ Configurable high/low temperature thresholds
* ✅ Alert flags functional for threshold violations
* ✅ Command interface responsive (`@temp,status?`, `@temp,config,*`)
* ✅ Integration with main system loop without timing issues

**Void Detection Module Success Criteria:**

* ✅ Basic void detection using threshold comparison
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

* If void detection implementation takes longer than planned, prioritize basic threshold detection over advanced algorithms
* If command interface proves complex, implement status queries first, configuration commands second
* If integration issues arise, implement modules independently first, then integrate incrementally

#### Estimated Timeline: 4 weeks to functional POC

* Week 1: Temperature module implementation and testing
* Week 2: Void detection module implementation and testing
* Week 3: Command interface implementation and integration
* Week 4: System testing, optimization, and POC demonstration

This plan provides a realistic, achievable path to a functional proof of concept while maintaining the flexibility to adjust priorities based on implementation challenges.

-----

## 9. Safety and Reliability

### 9.1. Fault Detection and Recovery

**Sensor and Communication Health:**

* **CAN Errors:** `mti_can.c` handles `HAL_CAN_ErrorCallback` for bus errors (BOF, EPV, EWG), attempting to stop, reset error, restart CAN, and re-enable notifications. `prv_can_total_error_count` tracks errors.
* **Sensor Timeouts:** `mti_can.c` (`can_process`) checks for `CAN_COMM_TIMEOUT_MS` and increments `prv_can_total_timeout_count`. It does not automatically trigger resets for individual sensors on global bus timeout but logs the possibility.
* **Individual Sensor Errors:** `can_handle_error()` in `mti_can.c` attempts to reset a specific sensor up to `SENSOR_MAX_RESET_ATTEMPTS` times before marking it as `RADAR_HW_STATUS_ERROR`.
* **Void Subsystem Status:** `void_get_subsystem_hw_status()` in `mti_void.c` aggregates status from individual sensors via `can_get_sensor_hw_status()` and CAN bus state.

```c
// In mti_can.c, for handling specific sensor errors:
// if (prv_sensor_reset_attempt_counts_array[sensor_idx] < SENSOR_MAX_RESET_ATTEMPTS) { //
//     if (can_reset_sensor(sensor_idx) == CAN_STATE_READY) return true; //
// } else {
//     prv_sensor_hw_status_array[sensor_idx] = RADAR_HW_STATUS_ERROR; //
// }
```

### 9.2. Watchdog and System Health

* **Independent Watchdog (IWDG):** Assumed to be configured at the HAL/main level, typical for STM32 projects. System must periodically "pet" the watchdog.
* **System Status:** `mti_system.c` manages `prv_system_current_status` which can reflect various error states (e.g., `SYSTEM_STATUS_ERROR_RADAR_HW`, `SYSTEM_STATUS_ERROR_IMU`).
* **Keep-Alive:** `mti_system.c` implements `system_check_and_send_keepalive()` which sends an `@status,down,A` message if in measure mode and `KEEPALIVE_TIMEOUT_MS` expires.

> **Protocol Note:** The `@status,down,A` prefix deviates from our usual response/event conventions (`&` for replies or `!` for async alerts). Update the code to emit `&status,down,A` (or `!keepalive,…`) for consistency with the ASCII protocol.

### 9.3. Communication Robustness

* **CAN:** Utilizes hardware CRC, message acknowledgments, and error frames provided by the CAN protocol. `mti_can.c` includes error counters and recovery mechanisms.
* **UART:** No explicit CRC mentioned for UART command/response protocol in `vmt_command.c`. Robustness relies on clear start/end delimiters (`@` for incoming, `\n` as end char) and parsing logic in `vmt_string.c`. Retransmission logic for UART commands is typically handled by the uphole system if no response or NACK is received.

-----

## 10. Future Enhancements

### 10.1. Advanced Signal Processing

* **Median Filtering:** `prv_void_current_config.enable_median_filter` flag exists in `mti_void.h`, but the `MULTI_POINT_MEDIAN` option in `prv_void_process_multi_point_detection` is a simple sort-and-pick median, not a running historical filter. True median filtering on time-series data is pending.
* **Kalman Filtering:** Not implemented.
* **Sensor Fusion:** Basic data aggregation from multiple radars occurs. IMU correction for tilt and temperature compensation are pending.

### 10.2. Machine Learning Integration

* Not currently implemented or planned in the provided codebase.

### 10.3. Performance Optimization

* **Integer Arithmetic:** Transition to millimeter units for distances is a significant step. `prv_void_fit_circle_integer` uses integer math.
* **Fixed-Point:** Broader use of fixed-point arithmetic for other calculations (angles, etc.) is largely pending.
* **Lookup Tables:** Not observed for trigonometric functions; `sqrtf`, `cosf`, `sinf`, `atanf`, `asinf`, `acosf` are used from `math.h`.

-----

## 11. Development Guidelines

### 11.1. Coding Standards

The codebase generally attempts to follow BARR-C style (snake_case, `prv_` prefix for static/private, header guards, etc.).

```c
// Function naming example from mti_void.c:
// static void prv_void_analyze_for_voids(void); //
// bool void_init(void); //

// Variable naming from mti_void.c:
// static bool prv_void_is_initialized; //
// static wall_measurement_t prv_void_latest_measurements[MAX_SENSORS]; //
// #define DEFAULT_VOID_DETECTION_THRESHOLD_MM 50U //

// Error handling often uses return enums (e.g., can_state_t, flash_fifo_res_t)
// typedef enum { CAN_STATE_UNINIT, CAN_STATE_READY, ... } can_state_t;
```

### 11.2. Testing Requirements

* **Unit Testing:** No dedicated unit test framework visible in the provided files. Testing appears to be done via debug messages and observed behavior.
* **Hardware-in-the-loop (HIL):** The setup implies HIL testing via UART commands and CAN bus monitoring.

### 11.3. Documentation Standards

* **Doxygen:** Used in header files (`.h`) for functions and data structures. Consistency varies.
* **File Headers:** Most `.c` and `.h` files have a standard comment header with file purpose, author, version, date.

-----

## 12. Next Steps

### 12.1. Immediate POC Implementation Tasks

#### Priority 1: Implement Data Flow Stages (CRITICAL)

**Stage 2 - Radar Cleanup Module:**

* [ ] Create `radar_cleanup.c` module for data processing
* [ ] Implement `cleanup_radar_data()` function for Stage 1→2 conversion
* [ ] Add meter-to-millimeter conversion with range validation
* [ ] Implement basic sensor data validation logic
* [ ] Add cleaned data structure population

**Stage 3 - Void Analysis Module:**

* [ ] Create `void_analysis.c` module for detection logic
* [ ] Implement `analyze_for_voids()` function for Stage 2→3 conversion
* [ ] Add basic threshold-based void detection algorithm
* [ ] Implement simple confidence calculation
* [ ] Add void characterization and result storage

**Stage 4 - Command Response System:**

* [ ] Enhance `vmt_command.c` with full `@vd` command handling
* [ ] Implement `get_latest_void_results()` for data retrieval
* [ ] Add response formatting for different command types
* [ ] Create uphole communication functions

#### Priority 2: Data Structure Implementation

* [ ] Define and implement all POC data structures in headers
* [ ] Add proper structure initialization functions
* [ ] Implement data validation and bounds checking
* [ ] Add timestamp management throughout data flow

#### Priority 3: Basic Algorithm Implementation

* [ ] Implement simple distance-based void detection
* [ ] Add configurable threshold values for detection
* [ ] Create basic confidence scoring based on SNR and distance
* [ ] Add sector identification for multi-sensor analysis

### 12.2. POC Validation and Testing

#### Data Flow Testing

* [ ] Test Stage 1→2: Raw data to cleaned data conversion
* [ ] Test Stage 2→3: Cleaned data to analysis results
* [ ] Test Stage 3→4: Results retrieval via commands
* [ ] Validate end-to-end data flow integrity

#### Algorithm Testing

* [ ] Test void detection with simulated sensor data
* [ ] Validate threshold-based detection logic
* [ ] Test confidence calculation accuracy
* [ ] Verify response formatting and uphole communication

#### Integration Testing

* [ ] Test with live CAN bus data (if available)
* [ ] Validate command processing under various conditions
* [ ] Test system behavior with invalid/missing sensor data
* [ ] Verify memory usage and performance characteristics

### 12.3. Expected POC Outcomes

**Functional Deliverables:**

* ✅ Complete data flow from CAN input to uphole response
* ✅ Basic void detection using simple threshold algorithm
* ✅ Working `@vd` command interface for monitoring and control
* ✅ Confidence scoring for detection reliability assessment
* ✅ Multi-sensor data processing and sector identification

**Technical Achievements:**

* ✅ Millimeter precision data handling throughout system
* ✅ Modular architecture with clear separation of concerns
* ✅ Simple but effective void detection suitable for proof of concept
* ✅ Integration with existing CAN and command infrastructure
* ✅ Foundation for future algorithm enhancement and optimization

#### Timeline: 4 weeks to functional POC

This simplified approach focuses on demonstrating core void detection capability while maintaining clean architecture for future enhancement. The basic data flow structure provides a solid foundation that can be extended with more sophisticated algorithms once the POC proves the concept viability.

-----

## 13. Current Implementation Status Summary

### 13.1. Executive Summary

#### System Infrastructure: 85% Complete ✅

The STM32F722 void detection system has successfully implemented most of its foundational infrastructure:

* **CAN Communication System (98%)**: Fully functional with 3 radar sensor support, error handling, and recovery mechanisms
* **Radar Management (90%)**: Round-robin sensor polling, data acquisition, and processing framework
* **System Initialization (98%)**: Multi-step startup sequence with proper peripheral initialization
* **IMU Processing (70%)**: Dual ICM20948 sensors with motion detection and angle calculations
* **Water Detection (85%)**: Dual ADC sensors with adaptive thresholds and event reporting
* **Temperature Sensing (85%)**: Basic ADC reading and status reporting (needs completion)
* **Hardware Abstraction (90%)**: BARR-C compliant peripheral wrappers for ADC, SPI, UART, Flash
* **Memory Management (100%)**: Static allocation throughout, no dynamic memory usage
* **Debug Infrastructure (100%)**: Comprehensive DEBUG_SEND logging system

#### Void Detection Algorithms: 0% Complete ❌

The core void detection functionality, which is the primary purpose of this system, has not yet been implemented:

* **Core Algorithm File**: `mti_void.c` is missing from the current codebase
* **Circle Fitting**: 3-point geometric calculation algorithms not implemented
* **Baseline Detection**: Primary void detection method not implemented  
* **Multi-Point Processing**: Sensor data fusion algorithms not implemented
* **Confidence Calculation**: Void detection reliability scoring not implemented
* **Data Conversion**: Radar sensor data to millimeter wall measurements not implemented

#### Command Interface: 20% Complete ⚠️

* **Framework**: Extensible command system exists in `vmt_command.c`
* **Void Commands**: `@vd` commands have placeholder implementation only
* **Design**: Full command specification exists in documentation but needs implementation

### 13.2. Critical Path to POC Completion

**Immediate Blockers (CRITICAL PRIORITY):**

1. **Implement `mti_void.c`** - Create the missing core void detection module
2. **Implement void detection algorithms** - Circle fitting, baseline detection, confidence calculation
3. **Implement sensor data conversion** - Radar data to millimeter precision wall measurements
4. **Complete `@vd` command interface** - Replace placeholder with functional commands
5. **Complete temperature module** - Finish temperature sensing and integration

**Estimated Development Effort:**

* Core void detection algorithms: 2-3 weeks
* Command interface completion: 1 week  
* Temperature module completion: 3-5 days
* Integration and testing: 1 week
* **Total: 4-5 weeks to POC completion**

### 13.3. System Readiness Assessment

**Ready for Integration ✅:**

* CAN communication with radar sensors
* System initialization and peripheral management
* Command processing framework
* Debug and logging infrastructure
* Data structure definitions and interfaces

**Requires Implementation ❌:**

* All void detection processing algorithms
* Sensor data conversion to millimeter precision
* Void detection confidence and reliability assessment
* Complete temperature monitoring integration
* Full void-specific command handlers

**Conclusion:**

The system has excellent foundational infrastructure but is missing its core functionality. The proof of concept is approximately 70% complete overall, with strong supporting systems but requiring immediate implementation of the primary void detection algorithms to achieve a functional prototype.

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

```
