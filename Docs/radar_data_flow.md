# Radar System Data Flow Implementation Guide

## Overview

This document provides a complete step-by-step guide for implementing a clean, layered radar system data flow. The system consists of three distinct layers with clear data structures and minimal coupling between non-adjacent layers.

## Target Architecture

```
CAN Bus → CAN Layer (radar_raw) → Radar Layer (radar_distance) → Void Detection Layer (void_data) → Reporting
```

### Design Principles

1. **Clear Layer Separation**: Each layer has distinct responsibilities and well-defined interfaces
2. **Consistent Naming**: Standardized struct naming (`radar_raw`, `radar_distance`, `void_data`)
3. **Unidirectional Data Flow**: Data flows forward through layers without circular dependencies
4. **Event-Driven Processing**: New data triggers processing cascades through the system
5. **Robust Error Handling**: Each layer validates inputs and handles errors gracefully

## Current State Analysis

### What's Working Well

- ✅ CAN message reception and parsing
- ✅ Multi-sensor management and indexing
- ✅ Basic data validation and filtering
- ✅ Void detection algorithms (bypass, threshold, circle fitting)
- ✅ Debug output and system monitoring

### What Needs Restructuring

- ❌ Mixed responsibilities in radar layer
- ❌ Direct void layer calls from CAN layer
- ❌ Inconsistent struct naming conventions
- ❌ No clear `radar_raw` → `radar_distance` → `void_data` flow
- ❌ Processing triggered from wrong layers

## Implementation Plan

---

## PHASE 1: CAN Layer Restructuring

### File: `mti_can.h`

#### Changes Required

1. **Rename data structures for clarity**

```c
// BEFORE: Inconsistent naming
typedef struct radar_data_s {
    float detectedPoints[MAX_RADAR_DETECTED_POINTS][2];
    uint8_t numDetPoints;
    // ... other fields
} radar_data_t;

// AFTER: Clear naming convention
typedef struct radar_raw_s {
    float detectedPoints[MAX_RADAR_DETECTED_POINTS][2];  // Raw sensor data [distance_m, SNR]
    uint8_t numDetPoints;                                // Number of valid detection points
    uint32_t frameNumber;                                // Frame sequence number
    uint16_t totalPacketLength;                          // Total packet size in bytes
    uint8_t pointIndex;                                  // Current point being processed
    float maxSNR;                                        // Maximum SNR in this frame
    bool new_data_ready;                                 // Flag indicating new frame complete
    radar_hw_status_t status;                            // Hardware status from sensor
    radar_version_t version;                             // Firmware version information
    uint32_t timestamp_ms;                               // When this data was received
} radar_raw_t;

typedef struct multi_radar_system_s {
    radar_raw_t sensors[MAX_RADAR_SENSORS];              // Raw data from each sensor
    bool sensor_online[MAX_RADAR_SENSORS];               // Online status flags
    uint32_t last_message_timestamp[MAX_RADAR_SENSORS];  // Last message received time
    uint32_t msgs_received[MAX_RADAR_SENSORS];           // Message counters for diagnostics
    uint8_t active_sensor_count;                         // Cached count of online sensors
} multi_radar_system_t;
```

**Why**: Establishes clear naming convention and eliminates confusion between raw sensor data and processed measurements.

2. **Add new accessor functions**

```c
// New function declarations
radar_raw_t* can_get_raw_sensor_data(uint8_t sensor_idx);
bool can_has_new_data(uint8_t sensor_idx);
void can_mark_data_processed(uint8_t sensor_idx);
bool can_is_sensor_online(uint8_t sensor_idx);
uint8_t can_get_active_sensor_count(void);
```

**Why**: Provides clean interface for radar layer to access raw data without exposing internal CAN structures.

### File: `mti_can.c`

#### Changes Required

1. **Update global variable declaration**

```c
// BEFORE:
multi_radar_system_t radar_system = { 0 };

// AFTER: (no change in declaration, but usage clarified)
multi_radar_system_t radar_system = { 0 };  // This contains radar_raw_t data
```

2. **Modify `process_complete_radar_frame()` function**

```c
// BEFORE: Direct coupling to void layer
void process_complete_radar_frame(uint8_t sensor_idx)
{
    radar_data_t *sensor = &radar_system.sensors[sensor_idx];
    
    debug_send("S%d: Frame %d complete with %d points", sensor_idx, sensor->frameNumber, sensor->numDetPoints);
    
    radar_system.last_message_timestamp[sensor_idx] = HAL_GetTick();
    radar_system.sensor_online[sensor_idx] = true;
    
    // PROBLEM: Direct call to radar layer AND void layer
    radar_process_measurement(sensor_idx, sensor->detectedPoints, sensor->numDetPoints);
    void_process_new_sensor_data(sensor_idx);  // <- REMOVE THIS
    
    sensor->new_data_ready = false;
}

// AFTER: Clean separation - only call radar layer
void process_complete_radar_frame(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        debug_send("Error: Invalid sensor index %d", sensor_idx);
        return;
    }
    
    radar_raw_t *sensor = &radar_system.sensors[sensor_idx];
    
    debug_send("S%d: Frame %d complete with %d points", sensor_idx, sensor->frameNumber, sensor->numDetPoints);
    
    // Update system state
    radar_system.last_message_timestamp[sensor_idx] = HAL_GetTick();
    radar_system.sensor_online[sensor_idx] = true;
    sensor->timestamp_ms = HAL_GetTick();
    sensor->new_data_ready = true;
    
    // Update active sensor count cache
    radar_system.active_sensor_count = get_active_sensor_count();
    
    // ONLY call radar layer - let it handle void layer notification
    radar_notify_new_raw_data(sensor_idx);
}
```

**Why**: Eliminates tight coupling between CAN and void layers. CAN layer now only knows about radar layer.

3. **Add new accessor functions**

```c
// Add these new functions to mti_can.c

/**
 * @brief Get raw sensor data for processing by radar layer
 * @param sensor_idx Index of sensor (0 to MAX_RADAR_SENSORS-1)
 * @return Pointer to raw sensor data or NULL if invalid
 */
radar_raw_t* can_get_raw_sensor_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return NULL;
    }
    return &radar_system.sensors[sensor_idx];
}

/**
 * @brief Check if sensor has new unprocessed data
 * @param sensor_idx Index of sensor
 * @return true if new data is available
 */
bool can_has_new_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    return radar_system.sensors[sensor_idx].new_data_ready;
}

/**
 * @brief Mark sensor data as processed by radar layer
 * @param sensor_idx Index of sensor
 */
void can_mark_data_processed(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return;
    }
    radar_system.sensors[sensor_idx].new_data_ready = false;
}

/**
 * @brief Check if sensor is currently online
 * @param sensor_idx Index of sensor
 * @return true if sensor is responding
 */
bool can_is_sensor_online(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return false;
    }
    return is_sensor_online(sensor_idx);  // Use existing function
}

/**
 * @brief Get number of currently active sensors
 * @return Number of online sensors (0 to MAX_RADAR_SENSORS)
 */
uint8_t can_get_active_sensor_count(void)
{
    return radar_system.active_sensor_count;
}
```

**Why**: Provides clean interface for radar layer to interact with CAN data without accessing internal structures directly.

---

## PHASE 2: Radar Layer Restructuring

### File: `mti_radar.h`

#### Changes Required

1. **Define `radar_distance_t` structure**

```c
// BEFORE: Single measurement structure
typedef struct
{
    uint16_t distance_mm;    // Single distance to borehole wall in millimeters
    uint16_t angle_deg;      // Angular position of this sensor (0, 120, 240)
    bool     data_valid;     // Flag indicating if the measurement is valid
    uint32_t timestamp_ms;   // Timestamp when measurement was taken
} radar_measurement_t;

// AFTER: Add clean data structure for processed measurements
typedef struct
{
    uint16_t distance_mm;    // Single distance to borehole wall in millimeters
    uint16_t angle_deg;      // Angular position of this sensor (0, 120, 240)
    bool     data_valid;     // Flag indicating if the measurement is valid
    uint32_t timestamp_ms;   // Timestamp when measurement was taken
    float    confidence;     // Confidence in measurement (0.0 to 1.0)
    uint8_t  quality_score;  // Signal quality score (0-100)
} radar_measurement_t;

// New structure for multi-sensor clean data
typedef struct
{
    uint16_t distance_mm[MAX_RADAR_SENSORS];  // Clean distances in millimeters
    uint16_t angle_deg[MAX_RADAR_SENSORS];    // Angular positions
    bool     data_valid[MAX_RADAR_SENSORS];   // Validity flags per sensor
    float    confidence[MAX_RADAR_SENSORS];   // Confidence per sensor
    uint8_t  quality_score[MAX_RADAR_SENSORS]; // Quality scores
    uint32_t timestamp_ms;                    // When this set was processed
    uint8_t  valid_sensor_count;              // Number of sensors with valid data
    bool     system_healthy;                  // Overall system health status
} radar_distance_t;
```

**Why**: Creates clear distinction between individual sensor measurements and system-wide processed data ready for void detection.

2. **Update function declarations**

```c
// BEFORE: Mixed responsibilities
void radar_process_measurement(uint8_t sensor_idx, float detectedPoints[MAX_RADAR_DETECTED_POINTS][2], uint8_t numPoints);
radar_measurement_t *radar_get_measurement(uint8_t sensor_idx);

// AFTER: Clear layer responsibilities
// Raw data processing functions
void radar_notify_new_raw_data(uint8_t sensor_idx);
bool radar_process_raw_to_distance(radar_distance_t* output_data);
void radar_update_single_sensor(uint8_t sensor_idx);

// Clean data access functions  
radar_distance_t* radar_get_distance_data(void);
radar_measurement_t* radar_get_measurement(uint8_t sensor_idx);
bool radar_has_valid_distance_data(void);

// System status functions
uint8_t radar_get_valid_sensor_count(void);
bool radar_is_system_ready(void);
```

**Why**: Separates raw data processing from clean data access, making the API clearer.

### File: `mti_radar.c`

#### Changes Required

1. **Add new static variables**

```c
// BEFORE: Only individual measurements
static radar_measurement_t radar_measurements[MAX_RADAR_SENSORS] = { 0 };

// AFTER: Add system-wide clean data structure
static radar_measurement_t radar_measurements[MAX_RADAR_SENSORS] = { 0 };
static radar_distance_t radar_clean_data = { 0 };  // NEW: System-wide clean data
static bool sensors_updated[MAX_RADAR_SENSORS] = { false };  // NEW: Track updates
```

**Why**: Maintains both individual sensor data and system-wide processed data for different use cases.

2. **Replace `radar_process_measurement()` with new functions**

```c
// REMOVE: Old function with mixed responsibilities
void radar_process_measurement(uint8_t sensor_idx, float detectedPoints[MAX_RADAR_DETECTED_POINTS][2], uint8_t numPoints)
{
    // OLD IMPLEMENTATION - REMOVE ENTIRELY
}

// ADD: New notification function called by CAN layer
void radar_notify_new_raw_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        debug_send("Error: Invalid sensor index %d", sensor_idx);
        return;
    }
    
    debug_send("Radar: New raw data available from sensor %d", sensor_idx);
    
    // Process this sensor's raw data immediately
    radar_update_single_sensor(sensor_idx);
    
    // Check if we should update system-wide clean data
    uint8_t updated_count = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        if (sensors_updated[i]) {
            updated_count++;
        }
    }
    
    // If we have updates from multiple sensors, process system-wide data
    if (updated_count >= 2) {
        if (radar_process_raw_to_distance(&radar_clean_data)) {
            // Notify void detection layer of new clean data
            if (system_is_operational_mode()) {
                void_process_new_distance_data(&radar_clean_data);
            }
        }
        
        // Reset update flags
        memset(sensors_updated, false, sizeof(sensors_updated));
    }
}

// ADD: Process single sensor raw data
void radar_update_single_sensor(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS) {
        return;
    }
    
    // Get raw data from CAN layer
    radar_raw_t* raw_data = can_get_raw_sensor_data(sensor_idx);
    if (!raw_data || !can_has_new_data(sensor_idx)) {
        return;
    }
    
    radar_measurement_t* measurement = &radar_measurements[sensor_idx];
    
    // Process the raw detected points
    float closest_distance = 100.0f;  // Initialize with large value
    float best_snr = 0.0f;
    bool valid_point_found = false;
    
    // Find closest valid point with good SNR
    for (uint8_t i = 0; i < raw_data->numDetPoints; i++) {
        float distance = raw_data->detectedPoints[i][0];  // meters
        float snr = raw_data->detectedPoints[i][1];       // SNR value
        
        // Validate using thresholds
        if (snr > RADAR_MIN_SNR_THRESHOLD && 
            distance > RADAR_MIN_DISTANCE_M && 
            distance < RADAR_MAX_DISTANCE_M &&
            distance < closest_distance) {
            
            closest_distance = distance;
            best_snr = snr;
            valid_point_found = true;
        }
    }
    
    // Update measurement structure
    if (valid_point_found) {
        measurement->distance_mm = (uint16_t)(closest_distance * 1000.0f);
        measurement->angle_deg = sensor_angles[sensor_idx];
        measurement->data_valid = true;
        measurement->timestamp_ms = HAL_GetTick();
        measurement->confidence = (best_snr > 200.0f) ? 1.0f : (best_snr / 200.0f);
        measurement->quality_score = (uint8_t)((best_snr / 300.0f) * 100.0f);
        if (measurement->quality_score > 100) measurement->quality_score = 100;
        
        debug_send("Radar %d: %u mm, conf=%.2f, qual=%d", 
                   sensor_idx, measurement->distance_mm, 
                   measurement->confidence, measurement->quality_score);
    } else {
        measurement->data_valid = false;
        measurement->confidence = 0.0f;
        measurement->quality_score = 0;
        debug_send("Radar %d: No valid measurement", sensor_idx);
    }
    
    // Mark this sensor as updated
    sensors_updated[sensor_idx] = true;
    
    // Mark CAN data as processed
    can_mark_data_processed(sensor_idx);
}

// ADD: Process system-wide clean data
bool radar_process_raw_to_distance(radar_distance_t* output_data)
{
    if (!output_data) {
        return false;
    }
    
    uint32_t current_time = HAL_GetTick();
    uint8_t valid_count = 0;
    
    // Copy individual measurements to system structure
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        radar_measurement_t* meas = &radar_measurements[i];
        
        // Check if data is still fresh (within last 2 seconds)
        bool data_fresh = (current_time - meas->timestamp_ms) < RADAR_DATA_TIMEOUT_MS;
        bool data_valid = meas->data_valid && data_fresh;
        
        output_data->distance_mm[i] = data_valid ? meas->distance_mm : RADAR_INVALID_DISTANCE;
        output_data->angle_deg[i] = meas->angle_deg;
        output_data->data_valid[i] = data_valid;
        output_data->confidence[i] = data_valid ? meas->confidence : 0.0f;
        output_data->quality_score[i] = data_valid ? meas->quality_score : 0;
        
        if (data_valid) {
            valid_count++;
        }
    }
    
    output_data->timestamp_ms = current_time;
    output_data->valid_sensor_count = valid_count;
    output_data->system_healthy = (valid_count >= 2);
    
    debug_send("Radar: System update - %d valid sensors", valid_count);
    
    return (valid_count >= 2);  // Return true if system has enough data
}
```

**Why**: Completely separates raw data processing from system-wide data aggregation. Creates clear pipeline: CAN → individual sensor processing → system-wide processing → void detection.

3. **Update `radar_system_process()` for maintenance only**

```c
// BEFORE: Mixed data processing and maintenance
void radar_system_process(void)
{
    // Complex processing code mixed with maintenance
}

// AFTER: Pure maintenance function
void radar_system_process(void)
{
    /* Skip processing if system isn't ready */
    if (radar_status == RADAR_INITIALISING) {
        return;
    }

    uint32_t current_time = HAL_GetTick();
    
    /* Check for stale data and invalidate if necessary */
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        radar_measurement_t* meas = &radar_measurements[i];
        
        if (meas->data_valid && 
            (current_time - meas->timestamp_ms) > RADAR_DATA_TIMEOUT_MS) {
            meas->data_valid = false;
            debug_send("Radar %d: Data timeout, marking invalid", i);
        }
    }

    /* Monitor sensor health every 1 second */
    static uint32_t last_health_check = 0;
    if (current_time - last_health_check >= 1000) {
        last_health_check = current_time;
        monitor_sensor_health();  // Calls CAN layer function
    }
    
    /* Update system health status */
    uint8_t valid_count = radar_get_valid_sensor_count();
    radar_clean_data.system_healthy = (valid_count >= 2);
    
    /* No data processing here - that's event-driven now */
}
```

**Why**: Separates maintenance tasks from data processing. Data processing is now event-driven through `radar_notify_new_raw_data()`.

4. **Add new accessor functions**

```c
// ADD: New clean data accessor
radar_distance_t* radar_get_distance_data(void)
{
    return &radar_clean_data;
}

// ADD: System readiness check
bool radar_has_valid_distance_data(void)
{
    return radar_clean_data.system_healthy && 
           (radar_clean_data.valid_sensor_count >= 2);
}

// ADD: Valid sensor count
uint8_t radar_get_valid_sensor_count(void)
{
    uint8_t count = 0;
    uint32_t current_time = HAL_GetTick();
    
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        radar_measurement_t* meas = &radar_measurements[i];
        bool data_fresh = (current_time - meas->timestamp_ms) < RADAR_DATA_TIMEOUT_MS;
        
        if (meas->data_valid && data_fresh) {
            count++;
        }
    }
    
    return count;
}

// ADD: System readiness
bool radar_is_system_ready(void)
{
    return (radar_status == RADAR_READY || radar_status == RADAR_CHIRPING) &&
           radar_has_valid_distance_data();
}
```

**Why**: Provides clean interface for void detection layer to access processed radar data.

---

## PHASE 3: Void Detection Layer Enhancement

### File: `mti_void.h`

#### Changes Required

1. **Update data structures**

```c
// BEFORE: Unclear naming
typedef void_status_t void_data_t;

// AFTER: Clear naming and enhanced structure
typedef struct {
    bool void_detected;                    // Primary detection result
    void_severity_t severity;              // Severity level (minor/major/critical)
    uint8_t confidence_percent;            // Overall confidence (0-100)
    void_algorithm_t algorithm_used;       // Which algorithm produced this result
    uint32_t detection_timestamp_ms;      // When detection was made
    
    // Individual sensor results
    struct {
        bool detected[MAX_RADAR_SENSORS];      // Per-sensor detection flags
        uint8_t confidence[MAX_RADAR_SENSORS]; // Per-sensor confidence
        uint16_t distance_mm[MAX_RADAR_SENSORS]; // Distances used for detection
    } sensor_data;
    
    // Algorithm-specific data
    union {
        struct {
            uint16_t threshold_used_mm;        // Threshold that was applied
            uint16_t baseline_used_mm;         // Baseline that was used
        } threshold_data;
        
        struct {
            uint16_t center_x_mm;              // Circle center X coordinate
            uint16_t center_y_mm;              // Circle center Y coordinate  
            uint16_t radius_mm;                // Fitted circle radius
            uint8_t sensors_used;              // Number of sensors in fit
            uint16_t fit_error_mm;             // RMS fit error
        } circle_data;
    } algorithm_result;
} void_data_t;
```

**Why**: Creates comprehensive result structure that contains all information needed for reporting and debugging.

2. **Add new function declarations**

```c
// BEFORE: Mixed function responsibilities
void void_process_new_sensor_data(uint8_t sensor_idx);

// AFTER: Clear data flow functions
void void_process_new_distance_data(const radar_distance_t* distance_data);
void_data_t* void_get_latest_result(void);
bool void_has_new_result(void);
void void_mark_result_processed(void);
```

**Why**: Aligns function names with new data flow where void layer receives `radar_distance_t` instead of individual sensor updates.

### File: `mti_void.c`

#### Changes Required

1. **Update static variables**

```c
// BEFORE: Mixed naming
static void_system_state_t prv_void_system = { 0 };

// AFTER: Clear naming and enhanced state
static struct {
    bool system_initialized;
    void_config_t config;
    void_data_t latest_result;
    bool new_result_available;
    uint32_t last_processing_time_ms;
    uint32_t last_distance_update_ms;
    
    // Current measurement data (copy of radar_distance_t)
    struct {
        uint16_t distance_mm[MAX_RADAR_SENSORS];
        uint16_t angle_deg[MAX_RADAR_SENSORS];
        bool data_valid[MAX_RADAR_SENSORS];
        float confidence[MAX_RADAR_SENSORS];
        uint32_t timestamp_ms;
        uint8_t valid_sensor_count;
    } current_measurements;
} prv_void_system = { 0 };
```

**Why**: Simplifies state management and clearly separates configuration, current data, and results.

2. **Replace `void_process_new_sensor_data()` with new function**

```c
// REMOVE: Old sensor-specific function
void void_process_new_sensor_data(uint8_t sensor_idx) {
    // REMOVE ENTIRELY
}

// ADD: New distance data processing function
void void_process_new_distance_data(const radar_distance_t* distance_data)
{
    if (!prv_void_system.system_initialized || !distance_data) {
        debug_send("Void: System not ready or invalid data");
        return;
    }
    
    // Rate limiting - don't process too frequently
    uint32_t current_time = HAL_GetTick();
    if (current_time - prv_void_system.last_processing_time_ms < VOID_MIN_PROCESS_INTERVAL_MS) {
        return;
    }
    
    // Copy distance data to internal structure
    memcpy(&prv_void_system.current_measurements, distance_data, 
           sizeof(prv_void_system.current_measurements));
    
    prv_void_system.last_distance_update_ms = current_time;
    
    debug_send("Void: Processing new distance data (%d valid sensors)", 
               distance_data->valid_sensor_count);
    
    // Only process if we have enough valid sensors
    if (distance_data->valid_sensor_count < prv_void_system.config.min_sensors_circle) {
        prv_send_insufficient_sensor_fault(distance_data->valid_sensor_count, 
                                           prv_void_system.config.algorithm);
        return;
    }
    
    // Process the detection
    void_data_t result = {0};
    bool detection_successful = false;
    
    // Run the configured algorithm
    switch (prv_void_system.config.algorithm) {
        case VOID_ALG_BYPASS:
            detection_successful = prv_bypass_detection(&result);
            break;
            
        case VOID_ALG_SIMPLE_THRESHOLD:
            detection_successful = prv_simple_threshold_detection(&result);
            break;
            
        case VOID_ALG_CIRCLEFIT:
            detection_successful = prv_circle_fit_void_detection(&result);
            break;
            
        default:
            debug_send("Void: Unknown algorithm %d", prv_void_system.config.algorithm);
            return;
    }
    
    if (detection_successful) {
        // Store result and mark as new
        prv_void_system.latest_result = result;
        prv_void_system.new_result_available = true;
        prv_void_system.last_processing_time_ms = current_time;
        
        // Send automatic streams if enabled
        void_send_automatic_stream();
        
        // Send detection events if state changed
        static bool previous_detection = false;
        if (result.void_detected != previous_detection) {
            void_send_detection_events(result.void_detected, previous_detection);
            previous_detection = result.void_detected;
        }
    }
}
```

**Why**: Completely changes the interface to accept processed radar data instead of individual sensor updates. This aligns with the new data flow architecture.

3. **Update algorithm functions to use new data structure**

```c
// UPDATE: Modify existing algorithm functions to use current_measurements
static bool prv_simple_threshold_detection(void_data_t *result)
{
    if (!result) {
        return false;
    }
    
    // Initialize result structure
    memset(result, 0, sizeof(void_data_t));
    result->algorithm_used = VOID_ALG_SIMPLE_THRESHOLD;
    result->detection_timestamp_ms = HAL_GetTick();
    result->algorithm_result.threshold_data.threshold_used_mm = prv_void_system.config.threshold_mm;
    result->algorithm_result.threshold_data.baseline_used_mm = prv_void_system.config.baseline_mm;
    
    uint8_t detections = 0;
    uint8_t total_confidence = 0;
    
    // Check each sensor against threshold
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++) {
        if (!prv_void_system.current_measurements.data_valid[i]) {
            continue;
        }
        
        uint16_t distance = prv_void_system.current_measurements.distance_mm[i];
        uint16_t expected = prv_void_system.config.baseline_mm;
        uint16_t threshold = prv_void_system.config.threshold_mm;
        
        // Copy sensor data to result
        result->sensor_data.distance_mm[i] = distance;
        
        // Check for void detection
        if (distance > (expected + threshold)) {
            result->sensor_data.detected[i] = true;
            detections++;
            
            // Calculate confidence based on how far beyond threshold
            uint16_t excess = distance - (expected + threshold);
            uint8_t confidence = void_calculate_confidence(distance, expected, threshold);
            result->sensor_data.confidence[i] = confidence;
            total_confidence += confidence;
            
            debug_send("Void: S%d detected void - %dmm > %dmm+%dmm (conf=%d)", 
                      i, distance, expected, threshold, confidence);
        } else {
            result->sensor_data.detected[i] = false;
            result->sensor_data.confidence[i] = 0;
        }
    }
    
    // Determine overall result
    result->void_detected = (detections > 0);
    result->confidence_percent = detections > 0 ? (total_confidence / detections) : 0;
    
    // Determine severity based on confidence and number of detections
    if (result->void_detected) {
        if (detections >= 2 && result->confidence_percent >= 80) {
            result->severity = VOID_SEVERITY_CRITICAL;
        } else if (detections >= 2 || result->confidence_percent >= 60) {
            result->severity = VOID_SEVERITY_MAJOR;
        } else {
            result->severity = VOID_SEVERITY_MINOR;
        }
    }
    
    debug_send("Void: Threshold detection - %s (%d sensors, %d%% confidence)", 
               result->void_detected ? "DETECTED" : "none", 
               detections, result->confidence_percent);
    
    return true;
}

// Similarly update prv_circle_fit_void_detection() to use current_measurements
static bool prv_circle_fit_void_detection(void_data_t *result)
{
    // Similar pattern - use prv_void_system.current_measurements instead of 
    // accessing individual sensor data
    
    // ... implementation using current_measurements structure
}
```

**Why**: Updates algorithms to work with the new centralized measurement data instead of accessing sensors individually.

4. **Add new accessor functions**

```c
// ADD: Result access functions
void_data_t* void_get_latest_result(void)
{
    return &prv_void_system.latest_result;
}

bool void_has_new_result(void)
{
    return prv_void_system.new_result_available;
}

void void_mark_result_processed(void)
{
    prv_void_system.new_result_available = false;
}

// UPDATE: System readiness check
bool void_is_system_ready(void)
{
    if (!prv_void_system.system_initialized) {
        return false;
    }
    
    // Check if we have recent measurement data
    uint32_t current_time = HAL_GetTick();
    bool data_fresh = (current_time - prv_void_system.last_distance_update_ms) < VOID_SENSOR_TIMEOUT_MS;
    
    return data_fresh && 
           (prv_void_system.current_measurements.valid_sensor_count >= CIRCLE_FIT_MIN_SENSORS);
}
```

**Why**: Provides clean interface for command layer and reporting systems to access void detection results.

---

## PHASE 4: System Integration Updates

### File: `mti_system.c`

#### Changes Required

1. **Update `module_init()` to follow new initialization order**

```c
// UPDATE: Step radar case in module_init()
case STEP_RADAR:
    if (radar_init_status == RADAR_INIT_NOT_STARTED) {
        debug_send("Starting radar initialization...");
        radar_system_init();  // This calls can_initialize_continuous_mode()
        radar_init_status = RADAR_INIT_IN_PROGRESS;
        return false;
    }

    if (radar_init_status == RADAR_INIT_IN_PROGRESS) {
        // Check if CAN layer has enough active sensors
        uint8_t active_sensors = can_get_active_sensor_count();
        if (active_sensors >= 2) {
            debug_send("Radar initialization complete (%d sensors active)", active_sensors);
            radar_init_status = RADAR_INIT_OK;
            init_step = STEP_VOID;
        } else if (HAL_GetTick() > 15000) {  // 15 second timeout
            debug_send("Radar initialization timeout (only %d sensors)", active_sensors);
            radar_init_status = RADAR_INIT_ERROR;
            module_status = STATUS_RADAR_ERROR;
            init_step = STEP_TEMP;  // Skip void init if radar failed
        }
        return false;
    }
    init_step = STEP_VOID;
    
case STEP_VOID:
    debug_send("Initializing void detection system...");
    void_system_init();
    
    // Void system doesn't need sensors to initialize - just configuration
    if (void_is_system_ready()) {
        debug_send("Void detection system ready with live data");
    } else {
        debug_send("Void detection system initialized (waiting for radar data)");
    }
    
    init_step = STEP_FINISH;
    return false;
```

**Why**: Ensures proper initialization order and allows void system to initialize even if radar data isn't immediately available.

### File: `vmt_device.c`

#### Changes Required

1. **Update main processing loop**

```c
// UPDATE: device_process() function
void device_process(void)
{
    /* Existing IMU and other processing */
    
    // Add radar system processing if initialized
    if (initialised_get() && state_get() >= measure_state) {
        // Process CAN health monitoring and timeouts
        // This is maintenance only - data processing is event-driven
        radar_system_process();
        
        // Void detection processing is triggered by radar layer
        // No need to call void_system_process() here
    }
    
    /* Continue with existing processing */
}
```

**Why**: Removes unnecessary polling of void detection - it's now event-driven from radar layer.

### File: `vmt_command.c`

#### Changes Required

1. **Update command handlers to use new data structures**

```c
// UPDATE: cmd_void() function to use new accessor
static void cmd_void(h_str_pointers_t *str_p)
{
    // ... existing parameter parsing ...
    
    if (strcmp(str_p->str[1], "status") == 0) {
        void_data_t* result = void_get_latest_result();
        if
