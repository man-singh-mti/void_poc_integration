# Radar System Data Flow Implementation Guide

## Overview

This document provides a complete step-by-step guide for implementing a clean, layered radar system data flow. The system consists of three distinct layers with clear data structures and minimal coupling between non-adjacent layers.

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

## Implementation Status

### ✅ PHASE 1 COMPLETED: CAN Layer Complete Restructuring

**Implementation Status: COMPLETE**

- ✅ `mti_radar_types.h` - Complete type definitions for entire pipeline
- ✅ `mti_can.h` - Clean interface with proper separation of concerns
- ✅ `mti_can.c` - Full implementation with Extended CAN IDs and proper message processing
- ✅ CAN sensor configuration and control functions
- ✅ Continuous data reception with proper buffering
- ✅ Event-driven processing with `process_complete_radar_frame()`
- ✅ Comprehensive testing functions (`test_sensor_indexing()`, `test_sensor_responses()`)

**Key Achievements:**
- Fixed Extended CAN ID usage (critical for sensor communication)
- Implemented 0x60 command base with 0xA0 data base (no conflicts)
- Added proper sensor indexing with 0x10 offsets
- Created robust message processing pipeline
- Integrated with existing system architecture

---

## 🔄 PHASE 2: RADAR LAYER IMPLEMENTATION (IN PROGRESS)

### File: `mti_radar.h` (CREATE NEW)

Clean interface for radar data processing layer:

```c
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
#define RADAR_PROCESSING_INTERVAL_MS 100    // 10Hz processing rate
#define RADAR_SENSOR_TIMEOUT_MS 2000        // 2 second sensor timeout
#define RADAR_MIN_VALID_DISTANCE_MM 50      // Minimum valid distance (50mm)
#define RADAR_MAX_VALID_DISTANCE_MM 5000    // Maximum valid distance (5m)
#define RADAR_MIN_SNR_THRESHOLD 10.0f       // Minimum SNR for valid reading

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

/** @name Internal Functions (used by CAN layer) */

/**
 * @brief Notification from CAN layer that new raw data is available
 * 
 * Called by process_complete_radar_frame() in mti_can.c
 * 
 * @param sensor_idx Sensor index with new data
 */
void radar_notify_new_raw_data(uint8_t sensor_idx);

#endif // MTI_RADAR_H
```
