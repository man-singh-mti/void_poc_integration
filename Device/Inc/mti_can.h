/**
 * @file mti_can.h
 * @brief MTI Radar CAN communication interface
 * @author [Author Name]
 * @date [Current Date]
 *
 * @details This module provides the CAN communication interface for the MTI radar system.
 * It includes command definitions, radar profiles, status enumerations, and communication
 * functions for sending commands and data over the CAN bus. The interface supports
 * various radar operating modes including calibration, single/multi-target detection
 * at different ranges (50m/100m).
 *
 * Hardware Interface:
 * - CAN controller: STM32F7xx integrated CAN peripheral
 * - CAN transceiver: External CAN transceiver chip
 * - Baud rate: Configurable (typically 500kbps or 1Mbps)
 *
 * @note This implementation assumes STM32F7xx HAL library is available
 * @note All CAN IDs follow the radar system specification
 */

#ifndef MTI_CAN_H
#define MTI_CAN_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include <stddef.h>

/* CAN Command Definitions */
#define CAN_CMD       0x80 /**< Base command identifier */
#define CAN_START     0x00 /**< Start radar command */
#define CAN_STOP      0x01 /**< Stop radar command */
#define CAN_CAL       0x02 /**< Calibration command */
#define CAN_POWER     0x03 /**< Power control command */
#define CAN_STATUS    0x04 /**< Status request command */
#define CAN_THRESHOLD 0x05 /**< Threshold setting command */
#define CAN_SPREAD    0x06 /**< Spread parameter command */
#define CAN_PROFILE   0x07 /**< Profile selection command */
#define CAN_FOV       0x08 /**< Field of view command */

/* CAN Message ID Definitions */
#define CAN_ID_HEADER  0xA0 /**< Header message ID */
#define CAN_ID_OBJECT  0xA1 /**< Object detection message ID */
#define CAN_ID_PROFILE 0xA2 /**< Profile configuration message ID */
#define CAN_ID_STATUS  0xA3 /**< Status report message ID */
#define CAN_ID_VERSION 0xA4 /**< Version information message ID */

/* Radar Profile Definitions */
#define RADAR_PROFILE_CAL         0 /**< Calibration profile */
#define RADAR_PROFILE_50M_SINGLE  1 /**< 50m range, single target detection */
#define RADAR_PROFILE_50M_MULTI   2 /**< 50m range, multi-target detection */
#define RADAR_PROFILE_100M_SINGLE 3 /**< 100m range, single target detection */
#define RADAR_PROFILE_100M_MULTI  4 /**< 100m range, multi-target detection */

/**
 * @brief Radar system operational states
 *
 * State transitions are controlled by CAN commands and system events.
 */
typedef enum
{
    RADAR_INITIALISING, /**< System is initializing hardware and calibrating */
    RADAR_READY,        /**< System is ready to receive commands */
    RADAR_CHIRPING,     /**< System is actively transmitting and receiving */
    RADAR_STOPPED,      /**< System is stopped and in low power mode */
} radar_status_t;

/**
 * @brief Individual radar detection data
 * @details Contains detection parameters for a single radar target
 */
typedef struct
{
    float    distance_m;   /**< Target distance in meters */
    float    snr_db;       /**< Signal‐to‐noise ratio in dB */
    float    velocity_mps; /**< Target velocity in m/s (if available) */
    float    angle_deg;    /**< Target angle in degrees (if available) */
    uint32_t timestamp_ms; /**< Detection timestamp in milliseconds */
    uint8_t  confidence;   /**< Detection confidence level (0–100) */
} radar_detection_t;

/**
 * @brief Complete radar frame data
 * @details Contains all detections and metadata for a single radar frame
 */
typedef struct
{
    uint32_t          frame_number;             /**< Sequential frame identifier */
    uint32_t          total_packet_length;      /**< Total CAN packet length */
    uint32_t          frame_timestamp_ms;       /**< Frame reception timestamp */
    uint8_t           num_detections;           /**< Number of valid detections */
    uint8_t           profile_mode;             /**< Active radar profile */
    radar_detection_t detections[20];           /**< Array of detections (max 20) */
    bool              frame_complete;           /**< Frame completion status */
    float             max_snr_db;               /**< Maximum SNR in this frame */
    float             frame_processing_time_ms; /**< Processing time for this frame */
} radar_frame_t;

/**
 * @brief Radar system state and statistics
 * @details Manages current radar data and system status
 */
typedef struct
{
    radar_frame_t  current_frame;       /**< Frame currently being assembled */
    radar_frame_t  last_complete_frame; /**< Most recent complete frame */
    radar_status_t status;              /**< Current radar operational status */
    uint32_t       frames_received;     /**< Total frames received counter */
    uint32_t       frames_dropped;      /**< Dropped frames counter */
    uint32_t       last_frame_time_ms;  /**< Timestamp of last complete frame */
    uint32_t       error_count;         /**< CAN communication error counter */
    bool           new_frame_available; /**< Flag for new frame availability */
} radar_data_t;

/**
 * @brief Radar configuration parameters
 * @details System configuration and calibration data
 */
typedef struct
{
    uint8_t  active_profile;      /**< Current radar profile (0–4) */
    float    detection_threshold; /**< Minimum detection threshold */
    float    max_range_m;         /**< Maximum detection range */
    uint16_t update_rate_hz;      /**< Frame update rate */
    bool     calibration_active;  /**< Calibration mode status */
} radar_config_t;

/**
 * @brief Radar performance metrics
 * @details System performance tracking
 */
typedef struct
{
    uint32_t total_frames;                 /**< Total frames processed */
    uint32_t valid_detections;             /**< Total valid detections */
    float    average_frame_rate;           /**< Average frame processing rate */
    float    average_detections_per_frame; /**< Average detections per frame */
    uint32_t last_reset_time_ms;           /**< Last statistics reset time */
} radar_metrics_t;

/**
 * @brief Initialize and configure the CAN interface
 *
 * This function initializes the STM32F7xx CAN peripheral with the appropriate
 * settings for radar communication. It configures the bit timing, filters,
 * and interrupt handlers.
 *
 * @return true if CAN setup was successful, false otherwise
 *
 * @note Must be called before any other CAN communication functions
 * @note Requires system clock to be configured before calling
 */
bool can_setup(void);

/**
 * @brief Send a single byte message over CAN
 *
 * Transmits a single byte message with the specified CAN ID. This function
 * is typically used for simple command messages that don't require payload data.
 *
 * @param[in] ID CAN message identifier (11-bit standard format)
 * @param[in] message Single byte message to transmit
 *
 * @return true if message was queued for transmission successfully, false otherwise
 *
 * @note Function is non-blocking; actual transmission occurs asynchronously
 * @note CAN ID should be within valid range (0x000 to 0x7FF for standard format)
 */
bool can_send(uint32_t ID, uint8_t message);

/**
 * @brief Send a multi-byte message array over CAN
 *
 * Transmits a message containing multiple bytes with the specified CAN ID.
 * This function is used for complex messages that require structured data.
 *
 * @param[in] ID CAN message identifier (11-bit standard format)
 * @param[in] message Pointer to byte array containing message data
 * @param[in] length Number of bytes to transmit (must be 1-8 for standard CAN)
 *
 * @return true if message was queued for transmission successfully, false otherwise
 *
 * @note Function is non-blocking; actual transmission occurs asynchronously
 * @note Message pointer must remain valid until transmission completes
 * @note Length must not exceed 8 bytes for standard CAN frames
 * @note Null pointer check is performed on message parameter
 */
bool can_send_array(uint32_t ID, uint8_t *message, size_t length);

/**
 * @brief Get the latest complete radar frame
 * @param frame Pointer to store the frame data
 * @return true if valid frame available, false otherwise
 */
bool radar_get_latest_frame(radar_frame_t *frame);

/**
 * @brief Get current radar status and statistics
 * @param data Pointer to store radar data
 * @return true if data is valid
 */
bool radar_get_data(radar_data_t *data);

/**
 * @brief Check if new frame data is available
 * @return true if new complete frame since last read
 */
bool radar_frame_available(void);

/**
 * @brief Get detection count from latest frame
 * @return Number of detections in latest complete frame
 */
uint8_t radar_get_detection_count(void);

/**
 * @brief Get radar configuration
 * @param[out] config Pointer to store configuration data
 * @return true if successful, false otherwise
 */
bool radar_get_config(radar_config_t *config);

/**
 * @brief Set radar configuration
 * @param[in] config Pointer to new configuration
 * @return true if successful, false otherwise
 */
bool radar_set_config(const radar_config_t *config);

/**
 * @brief Get radar performance metrics
 * @param[out] metrics Pointer to store metrics data
 * @return true if successful, false otherwise
 */
bool radar_get_metrics(radar_metrics_t *metrics);

/**
 * @brief Reset radar statistics
 * @return true if successful, false otherwise
 */
bool radar_reset_statistics(void);

/**
 * @brief Get detection by index from latest frame
 * @param[in] index Detection index (0 to num_detections-1)
 * @param[out] detection Pointer to store detection data
 * @return true if valid detection found, false otherwise
 */
bool radar_get_detection_by_index(uint8_t index, radar_detection_t *detection);

/**
 * @brief Find closest detection in latest frame
 * @param[out] detection Pointer to store closest detection
 * @return true if detection found, false otherwise
 */
bool radar_get_closest_detection(radar_detection_t *detection);

#endif /* MTI_CAN_H */
