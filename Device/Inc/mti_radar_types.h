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
#define MAX_RADAR_SENSORS         3U
#define MAX_RADAR_DETECTED_POINTS 20U
#define SENSOR_TIMEOUT_MS         3000U // Sensor online timeout in milliseconds

/** @name Sensor Angular Positions */
#define SENSOR_0_ANGLE 0U
#define SENSOR_1_ANGLE 120U
#define SENSOR_2_ANGLE 240U

/** @name CAN Communication Constants */
#define CAN_SENSOR_OFFLINE_TIMEOUT_MS 3000U ///< Sensor offline timeout
#define CAN_STARTUP_DELAY_MS          500U  ///< Startup delay
#define CAN_COMMAND_DELAY_MS          50U   ///< Delay between commands
#define CAN_STATUS_BOOT               0x01U ///< Boot status
#define CAN_STATUS_CHIRP              0x02U ///< Chirping status
#define CAN_STATUS_STOPPED            0x03U ///< Stopped status

#ifndef M_PI
#define M_PI 3.14159265358979323846f // Pi constant
#endif

/** @name Radar Hardware Status */
typedef enum
{
    RADAR_HW_NOT_INITIALISED = 0,
    RADAR_HW_READY           = 1,
    RADAR_HW_CHIRPING        = 2,
    RADAR_HW_STOPPED         = 3
} radar_hw_status_t;

/** @name Radar Initialization Status */
typedef enum
{
    RADAR_INIT_NOT_STARTED = 0,
    RADAR_INIT_IN_PROGRESS = 1,
    RADAR_INIT_OK          = 2,
    RADAR_INIT_ERROR       = 3
} radar_init_status_t;

/**
 * @brief Raw sensor data from CAN bus (Stage 1)
 *
 * Contains unprocessed data exactly as received from radar sensors
 */
typedef struct
{
    float    detected_points[MAX_RADAR_DETECTED_POINTS][2]; ///< [distance_m, SNR]
    uint8_t  num_points;                                    ///< Number of valid points
    uint32_t frame_number;                                  ///< Frame counter from sensor
    uint32_t timestamp_ms;                                  ///< When data was received
    bool     new_data_available;                            ///< Flag for event processing
    bool     new_frame;                                     ///< Flag indicating new frame started
    bool     frame_complete;                                ///< Flag indicating frame is complete
    uint8_t  sensor_status;                                 ///< Hardware status from sensor
} radar_raw_t;

/**
 * @brief Processed distance measurements (Stage 2)
 *
 * Contains clean, validated measurements ready for void detection
 */
typedef struct
{
    uint16_t distance_mm[MAX_RADAR_SENSORS];   ///< Clean distances in millimeters
    uint16_t angle_deg[MAX_RADAR_SENSORS];     ///< Sensor angles (0, 120, 240)
    bool     data_valid[MAX_RADAR_SENSORS];    ///< Validity flags per sensor
    float    confidence[MAX_RADAR_SENSORS];    ///< Confidence per sensor (0.0-1.0)
    uint8_t  quality_score[MAX_RADAR_SENSORS]; ///< Signal quality (0-100)
    uint8_t  valid_sensor_count;               ///< Number of sensors with valid data
    uint32_t timestamp_ms;                     ///< When processing completed
    bool     system_healthy;                   ///< Overall system health
} radar_distance_t;

/**
 * @brief Void detection results (Stage 3)
 *
 * Contains final analysis results ready for uphole transmission
 */
typedef struct
{
    bool     void_detected;        ///< Primary detection result
    uint8_t  algorithm_used;       ///< 0=bypass, 1=simple, 2=circlefit
    uint8_t  confidence_percent;   ///< Detection confidence (0-100)
    uint16_t void_size_mm;         ///< Estimated void size
    uint8_t  sensor_count_used;    ///< Sensors used for detection
    uint32_t detection_time_ms;    ///< When analysis completed
    char     status_text[64];      ///< Human-readable status
    bool     new_result_available; ///< Flag for transmission
} void_data_t;

/**
 * @brief Multi-sensor raw data system
 *
 * Complete system state for all radar sensors including version and status tracking
 */
typedef struct
{
    // Main sensor data array
    radar_raw_t sensors[MAX_RADAR_SENSORS]; ///< Primary sensor data array

    // Communication tracking
    uint32_t last_message_time[MAX_RADAR_SENSORS]; ///< Last message timestamp per sensor
    uint32_t last_status_time[MAX_RADAR_SENSORS];  ///< Last status message timestamp per sensor
    uint32_t msgs_received[MAX_RADAR_SENSORS];     ///< Message count per sensor

    // Sensor version information
    uint8_t sensor_version_major[MAX_RADAR_SENSORS];    ///< Version major per sensor
    uint8_t sensor_version_minor[MAX_RADAR_SENSORS];    ///< Version minor per sensor
    uint8_t sensor_version_patch[MAX_RADAR_SENSORS];    ///< Version patch per sensor
    bool    sensor_version_received[MAX_RADAR_SENSORS]; ///< Version received flag per sensor

    // Sensor status information
    uint8_t sensor_status[MAX_RADAR_SENSORS]; ///< Current status per sensor

    // System state
    uint8_t active_sensor_count; ///< Number of active sensors
    bool    system_initialized;  ///< System initialization flag
} multi_radar_raw_system_t;

#endif // MTI_RADAR_TYPES_H
