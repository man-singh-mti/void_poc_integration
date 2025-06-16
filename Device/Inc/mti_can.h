/**
 * @file mti_can.h
 * @brief CAN bus communication interface for multi-radar sensor system
 *
 * This file provides interface functions and definitions for CAN communication
 * with radar sensors, supporting command and data exchange in a multi-sensor
 * configuration.
 *
 * File organization:
 * - Configuration and constant definitions
 * - Type definitions (enums, structs, unions)
 * - Public function declarations
 * - External variable declarations
 *
 * @author MTi Group
 * @copyright Copyright (c) 2025 MTi Group
 */

#ifndef MTI_CAN_H
#define MTI_CAN_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include "vmt_uart.h" // For debug_send function
#include "mti_void.h"

/**
 * @name System Configuration Constants
 * @{
 */
/** @brief Maximum number of radar sensors in the system */
#define MAX_RADAR_SENSORS 3U
/** @brief Maximum number of detected points per radar frame */
#define MAX_RADAR_DETECTED_POINTS 20
/** @} */

/**
 * @name CAN Command Definitions
 * @brief Command codes for radar sensor control
 * @{
 */
#define CAN_CMD_BASE      0x80U /**< Base address for command messages */
#define CAN_CMD_START     0x00U /**< Command to start sensor operation */
#define CAN_CMD_STOP      0x01U /**< Command to stop sensor operation */
#define CAN_CMD_CAL       0x02U /**< Command to calibrate sensor */
#define CAN_CMD_POWER     0x03U /**< Command to set power level */
#define CAN_CMD_STATUS    0x04U /**< Command to request status information */
#define CAN_CMD_THRESHOLD 0x05U /**< Command to set detection threshold */
#define CAN_CMD_SPREAD    0x06U /**< Command to set spreading factor */
#define CAN_CMD_PROFILE   0x07U /**< Command to set operation profile */
#define CAN_CMD_FOV       0x08U /**< Command to set field of view */
#define CAN_CMD_INIT      0x09U /**< Command to initialize sensor */
/** @} */

/**
 * @name Radar Power Level Definitions
 * @brief Power level settings as percentage of maximum
 * @{
 */
#define RADAR_POWER_OFF    0   /**< Radar powered off */
#define RADAR_POWER_LOW    25  /**< Low power mode (25%) */
#define RADAR_POWER_MEDIUM 50  /**< Medium power mode (50%) */
#define RADAR_POWER_HIGH   75  /**< High power mode (75%) */
#define RADAR_POWER_MAX    100 /**< Maximum power (100%) */
/** @} */

/**
 * @name Radar Field of View Definitions
 * @brief Field of view settings in degrees
 * @{
 */
#define RADAR_FOV_NARROW 60  /**< Narrow field of view (60°) */
#define RADAR_FOV_MEDIUM 90  /**< Medium field of view (90°) */
#define RADAR_FOV_WIDE   120 /**< Wide field of view (120°) */
#define RADAR_FOV_MAX    150 /**< Maximum field of view (150°) */
/** @} */

/**
 * @name CAN Message Type Base IDs
 * @brief Base identifiers for different CAN message types
 * @{
 */
#define CAN_ID_HEADER_BASE  0xA0U /**< Base ID for frame header messages */
#define CAN_ID_OBJECT_BASE  0xA1U /**< Base ID for detected object data */
#define CAN_ID_PROFILE_BASE 0xA2U /**< Base ID for profile information */
#define CAN_ID_STATUS_BASE  0xA3U /**< Base ID for status messages */
#define CAN_ID_VERSION_BASE 0xA4U /**< Base ID for version information */
/** @} */

/**
 * @brief Calculate sensor-specific CAN offset
 *
 * Generates offset value based on sensor index (0x00, 0x10, 0x20 for 3 sensors)
 *
 * @param sensor_idx Sensor index (0-2)
 * @return Offset value for the specific sensor
 */
#define CAN_SENSOR_OFFSET(sensor_idx) ((sensor_idx) * 0x10U)

/**
 * @name Sensor-Specific Message ID Macros
 * @brief Macros to generate sensor-specific message IDs
 * @{
 */
#define CAN_MSG_ID_HEADER_SENSOR(idx)  (CAN_ID_HEADER_BASE + CAN_SENSOR_OFFSET(idx))  /**< Header message ID for specific sensor */
#define CAN_MSG_ID_OBJECT_SENSOR(idx)  (CAN_ID_OBJECT_BASE + CAN_SENSOR_OFFSET(idx))  /**< Object data message ID for specific sensor */
#define CAN_MSG_ID_PROFILE_SENSOR(idx) (CAN_ID_PROFILE_BASE + CAN_SENSOR_OFFSET(idx)) /**< Profile message ID for specific sensor */
#define CAN_MSG_ID_STATUS_SENSOR(idx)  (CAN_ID_STATUS_BASE + CAN_SENSOR_OFFSET(idx))  /**< Status message ID for specific sensor */
#define CAN_MSG_ID_VERSION_SENSOR(idx) (CAN_ID_VERSION_BASE + CAN_SENSOR_OFFSET(idx)) /**< Version message ID for specific sensor */
/** @} */

/**
 * @name Sensor Base Addresses
 * @brief Base address constants for each sensor
 * @{
 */
#define SENSOR_0_BASE_ADDR 0x00 /**< Base address for sensor 0 */
#define SENSOR_1_BASE_ADDR 0x10 /**< Base address for sensor 1 */
#define SENSOR_2_BASE_ADDR 0x20 /**< Base address for sensor 2 */
/** @} */

/**
 * @brief Generate command ID for a specific sensor
 *
 * @param sensor_idx Sensor index (0-2)
 * @return Command ID for the specified sensor
 */
#define CAN_CMD_SENSOR_ID(sensor_idx) (CAN_CMD_BASE + (sensor_idx))

/**
 * @name Radar Operation Profile Constants
 * @brief Predefined radar operation profiles
 * @{
 */
#define RADAR_PROFILE_CAL         0 /**< Calibration profile */
#define RADAR_PROFILE_50M_SINGLE  1 /**< Single target, 50m range profile */
#define RADAR_PROFILE_50M_MULTI   2 /**< Multiple targets, 50m range profile */
#define RADAR_PROFILE_100M_SINGLE 3 /**< Single target, 100m range profile */
#define RADAR_PROFILE_100M_MULTI  4 /**< Multiple targets, 100m range profile */
/** @} */

/**
 * @brief System-level radar operational status
 */
typedef enum
{
    RADAR_INITIALISING, /**< System is initializing */
    RADAR_READY,        /**< System is ready for operation */
    RADAR_CHIRPING,     /**< System is actively transmitting/receiving */
    RADAR_STOPPED,      /**< System operation is halted */
} radar_status_t;

/**
 * @brief Hardware-level radar sensor status
 */
typedef enum
{
    RADAR_HW_INITIALISING = 0, /**< Hardware initialization in progress */
    RADAR_HW_READY        = 1, /**< Hardware ready for operation */
    RADAR_HW_CHIRPING     = 2, /**< Hardware actively transmitting/receiving */
    RADAR_HW_STOPPED      = 3, /**< Hardware operation halted */
    RADAR_HW_ERROR        = 4, /**< Hardware error detected */
    RADAR_HW_CALIBRATING  = 5  /**< Hardware calibration in progress */
} radar_hw_status_t;

/**
 * @brief Union for CAN message data interpretation
 *
 * Provides convenient access to CAN data payload in different formats
 */
typedef union
{
    uint8_t  bytes[8];  /**< Access as 8 individual bytes */
    uint32_t words[2];  /**< Access as 2 32-bit words */
    float    floats[2]; /**< Access as 2 floating point values */
} can_data_union_t;

/**
 * @brief Radar sensor data structure
 *
 * Contains all data related to a single radar sensor including detected points,
 * status, and version information
 */
typedef struct
{
    float             detectedPoints[MAX_RADAR_DETECTED_POINTS][2]; /**< Array of detected points [distance, SNR] */
    uint32_t          frameNumber;                                  /**< Frame counter for this sensor */
    uint32_t          totalPacketLength;                            /**< Raw value from header, protocol-specific */
    uint8_t           numDetPoints;                                 /**< Number of points expected/received for the frame */
    uint8_t           pointIndex;                                   /**< Current index for point reception */
    float             maxSNR;                                       /**< Calculated maximum SNR value */
    radar_hw_status_t status;                                       /**< Hardware status of this specific radar sensor */
    bool              new_data_ready;                               /**< Flag indicating new data is ready for processing */
    struct
    {
        uint8_t major; /**< Major version number */
        uint8_t minor; /**< Minor version number */
        uint8_t sub;   /**< Sub version number */
    } version;         /**< Sensor firmware version information */
} radar_data_t;

/**
 * @brief Multi-sensor system structure
 *
 * Contains data for all connected radar sensors and system-wide information
 */
typedef struct
{
    radar_data_t sensors[MAX_RADAR_SENSORS];                /**< Array of sensor data structures */
    uint8_t      active_sensor_count;                       /**< Number of currently active sensors */
    uint32_t     last_message_timestamp[MAX_RADAR_SENSORS]; /**< Last message time for each sensor */
    bool         sensor_online[MAX_RADAR_SENSORS];          /**< Online status flag for each sensor */
} multi_radar_system_t;

/**
 * @name CAN Communication Functions
 * @{
 */

/**
 * @brief Initialize CAN interface and set up filters
 *
 * @return true if setup was successful
 * @return false if any error occurred during setup
 */
bool can_setup(void);

/**
 * @brief Send a single byte CAN message
 *
 * @param ID CAN message identifier
 * @param message Single byte payload to send
 * @return true if message was successfully queued
 * @return false if transmission failed
 */
bool can_send(uint32_t ID, uint8_t message);

/**
 * @brief Send a multi-byte CAN message
 *
 * @param ID CAN message identifier
 * @param message Pointer to message data
 * @param length Length of message data (max 8 bytes)
 * @return true if message was successfully queued
 * @return false if transmission failed
 */
bool can_send_array(uint32_t ID, uint8_t *message, size_t length);

/**
 * @brief Send a command to a specific sensor
 *
 * @param sensor_idx Index of the target sensor (0 to MAX_RADAR_SENSORS-1)
 * @param base_id Base CAN ID for the message type
 * @param message Command code to send
 * @return true if message was successfully queued
 * @return false if invalid sensor index or transmission failed
 */
bool can_send_to_sensor(uint8_t sensor_idx, uint32_t base_id, uint8_t message);

/**
 * @brief Send a command to all connected sensors
 *
 * @param base_id Base CAN ID for the message type
 * @param message Command code to send
 */
void broadcast_to_all_sensors(uint32_t base_id, uint8_t message);

/**
 * @brief Extract sensor index from a CAN message ID
 *
 * @param can_id CAN message identifier
 * @return uint8_t Sensor index (0 to MAX_RADAR_SENSORS-1) or 0xFF if invalid
 */
uint8_t get_sensor_index_from_can_id(uint32_t can_id);
/** @} */

/**
 * @name CAN Message Processing Functions
 * @{
 */

/**
 * @brief Process a command received for a specific sensor
 *
 * @param sensor_idx Index of the target sensor
 * @param command Command code received
 * @param data Pointer to the complete CAN message data
 */
void process_sensor_command(uint8_t sensor_idx, uint8_t command, can_data_union_t *data);

/**
 * @brief Process a complete radar data frame
 *
 * Called when all expected points for a frame have been received
 *
 * @param sensor_idx Index of the sensor that completed a frame
 */
void process_complete_radar_frame(uint8_t sensor_idx);
/** @} */

/**
 * @name Sensor Status Functions
 * @{
 */

/**
 * @brief Check if a sensor is currently online
 *
 * A sensor is considered online if a message has been received
 * within the last 2 seconds
 *
 * @param sensor_idx Index of the sensor to check
 * @return true if sensor is online
 * @return false if sensor is offline or index is invalid
 */
bool is_sensor_online(uint8_t sensor_idx);

/**
 * @brief Get the number of currently active sensors
 *
 * @return uint8_t Number of online sensors (0 to MAX_RADAR_SENSORS)
 */
uint8_t get_active_sensor_count(void);

/**
 * @brief Reset all data for a specific sensor
 *
 * @param sensor_idx Index of the sensor to reset
 */
void reset_sensor_data(uint8_t sensor_idx);
/** @} */

/**
 * @name Debug and Test Functions
 * @{
 */

/**
 * @brief Run tests on sensor index extraction functionality
 */
void test_sensor_indexing(void);

/**
 * @brief Test connectivity with physical sensors
 *
 * Sends status requests to all configured sensors and checks
 * for responses
 */
void test_sensor_responses(void);

/**
 * @brief Send debug message to the debug output
 *
 * @param format Printf-style format string
 * @param ... Format arguments
 */
void debug_send(const char *format, ...);
/** @} */

/**
 * @brief Global radar system instance
 *
 * Central data structure containing information about all sensors in the system
 */
extern multi_radar_system_t radar_system;

#endif // MTI_CAN_H
