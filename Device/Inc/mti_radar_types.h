/**
 * @file mti_radar_types.h
 * @brief Common type definitions for radar data processing pipeline
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#ifndef MTI_RADAR_TYPES_H
#define MTI_RADAR_TYPES_H

#include <stdbool.h>
#include <stdint.h>


/**
 * @brief Void detection results (Stage 3)
 *
 * Contains final analysis results ready for uphole transmission
 */
typedef struct
{
    bool     void_detected;        // Primary detection result
    uint8_t  algorithm_used;       // 0=bypass,1=simple,2=circlefit
    uint8_t  confidence_percent;   // Detection confidence (0-100)
    uint16_t void_size_mm;         // Estimated void size
    uint8_t  sensor_count_used;    // Sensors used for detection
    uint32_t detection_time_ms;    // When analysis completed
    char     status_text[64];      // Human-readable status
    bool     new_result_available; // Flag for transmission
} void_data_t;

#endif // MTI_RADAR_TYPES_H
