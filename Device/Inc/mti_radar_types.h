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

/**
 * @brief Raw sensor data from CAN bus (Stage 1)
 * 
 * Contains unprocessed data exactly as received from radar sensors
 */
typedef struct {
    float detected_points[MAX_RADAR_DETECTED_POINTS][2];  // [distance_m, SNR]
    uint8_t num_points;                                   // Number of valid points
    uint32_t frame_number;                                // Frame counter from sensor
    uint32_t timestamp_ms;                                // When data was received
    bool new_data_available;                              // Flag for event processing
    uint8_t sensor_status;                                // Hardware status from sensor
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
    uint8_t valid_sensor_count;               // Number of sensors with valid data
    uint32_t timestamp_ms;                    // When processing completed
    bool system_healthy;                      // Overall system health
} radar_distance_t;

/**
 * @brief Void detection results (Stage 3)
 * 
 * Contains final analysis results ready for uphole transmission
 */
typedef struct {
    bool void_detected;                       // Primary detection result
    uint8_t algorithm_used;                   // 0=bypass, 1=simple, 2=circlefit
    uint8_t confidence_percent;               // Detection confidence (0-100)
    uint16_t void_size_mm;                    // Estimated void size
    uint8_t sensor_count_used;                // Sensors used for detection
    uint32_t detection_time_ms;               // When analysis completed
    char status_text[64];                     // Human-readable status
    bool new_result_available;                // Flag for transmission
} void_data_t;

#endif // MTI_RADAR_TYPES_H
