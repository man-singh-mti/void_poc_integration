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
#include <stddef.h>
#include "vmt_uart.h"
#include "mti_radar_types.h"

/** @name CAN Base ID Definitions */
#define CAN_COMMAND_BASE_ID            0x60U ///< Base for outgoing commands to sensors
#define CAN_DATA_HEADER_BASE_ID        0xA0U ///< Base for Header data from sensors
#define CAN_DATA_OBJECT_BASE_ID        0xA1U ///< Base for Object/Detected Points data
#define CAN_DATA_PROFILE_BASE_ID       0xA2U ///< Base for Range Profile data
#define CAN_DATA_STATUS_REPLY_BASE_ID  0xA3U ///< Base for Status reply from sensor
#define CAN_DATA_VERSION_REPLY_BASE_ID 0xA4U ///< Base for Version reply from sensor

/** @name Sensor Command IDs (0x60 base + 0x10 offsets) */
#define CAN_S0_COMMAND_ID 0x60U
#define CAN_S1_COMMAND_ID 0x70U
#define CAN_S2_COMMAND_ID 0x80U
#define CAN_S3_COMMAND_ID 0x90U

/** @name Sensor Data IDs (0xA0 base + 0x10 offsets) */
#define CAN_S0_DATA_HEADER_ID        0xA0U
#define CAN_S0_DATA_OBJECT_ID        0xA1U
#define CAN_S0_DATA_STATUS_REPLY_ID  0xA3U
#define CAN_S0_DATA_VERSION_REPLY_ID 0xA4U

#define CAN_S1_DATA_HEADER_ID        0xB0U
#define CAN_S1_DATA_OBJECT_ID        0xB1U
#define CAN_S1_DATA_STATUS_REPLY_ID  0xB3U
#define CAN_S1_DATA_VERSION_REPLY_ID 0xB4U

#define CAN_S2_DATA_HEADER_ID        0xC0U
#define CAN_S2_DATA_OBJECT_ID        0xC1U
#define CAN_S2_DATA_STATUS_REPLY_ID  0xC3U
#define CAN_S2_DATA_VERSION_REPLY_ID 0xC4U

#define CAN_S3_DATA_HEADER_ID        0xD0U
#define CAN_S3_DATA_OBJECT_ID        0xD1U
#define CAN_S3_DATA_STATUS_REPLY_ID  0xD3U
#define CAN_S3_DATA_VERSION_REPLY_ID 0xD4U

/** @name CAN Command Payloads */
#define CAN_START         0x00U      ///< Start command
#define CAN_STOP          0x01U      ///< Stop command
#define CAN_CAL           0x02U      ///< Calibration command
#define CAN_POWER         0x03U      ///< Power command
#define CAN_STATUS        0x04U      ///< Status query command
#define CAN_THRESHOLD     0x05U      ///< Threshold setting
#define CAN_SPREAD        0x06U      ///< Spread setting
#define CAN_PROFILE       0x07U      ///< Profile selection
#define CAN_FOV           0x08U      ///< Field of view setting
#define CAN_VERSION_QUERY CAN_STATUS ///< Version is queried with status command

#define CAN_CMD_PAYLOAD_START          CAN_START
#define CAN_CMD_PAYLOAD_STOP           CAN_STOP
#define CAN_CMD_PAYLOAD_QUERY_STATUS   CAN_STATUS
#define CAN_CMD_PAYLOAD_SELECT_PROFILE CAN_PROFILE
#define CAN_CMD_PAYLOAD_DET_THRESHOLD  CAN_THRESHOLD


/** @name System Functions */

/**
 * @brief Initialize CAN hardware
 * @return true if successful
 */
bool can_setup(void);

/**
 * @brief Initialize complete CAN sensor system
 * @return true if initialization successful
 */
bool can_initialize_system(void);

/** @name Communication Functions */

/**
 * @brief Send single byte CAN message
 * @param ID CAN identifier
 * @param message Single byte message
 * @return true if successful
 */
bool can_send(uint32_t ID, uint8_t message);

/**
 * @brief Send multi-byte CAN message
 * @param ID CAN identifier
 * @param message Message array
 * @param length Message length (1-8 bytes)
 * @return true if successful
 */
bool can_send_array(uint32_t ID, uint8_t *message, size_t length);

/**
 * @brief Send command to specific sensor
 * @param sensor_idx Sensor index (0-2)
 * @param command Command byte
 * @return true if successful
 */
bool can_send_to_sensor(uint8_t sensor_idx, uint8_t command);

/**
 * @brief Send command to all sensors
 * @param command Command byte
 * @return true if all sends successful
 */
bool can_send_command_to_all_sensors(uint8_t command);

/** @name Data Access Functions */

/**
 * @brief Get raw sensor data
 * @param sensor_idx Sensor index (0-2)
 * @return Pointer to raw data or NULL if invalid
 */
radar_raw_t *can_get_raw_data(uint8_t sensor_idx);

/**
 * @brief Check if sensor has new data
 * @param sensor_idx Sensor index (0-2)
 * @return true if new data available
 */
bool can_has_new_raw_data(uint8_t sensor_idx);

/**
 * @brief Mark sensor data as processed
 * @param sensor_idx Sensor index (0-2)
 */
void can_mark_raw_data_processed(uint8_t sensor_idx);

/**
 * @brief Get count of online sensors
 * @return Number of online sensors (0-3)
 */
uint8_t can_get_online_sensor_count(void);

/**
 * @brief Check if specific sensor is online
 * @param sensor_idx Sensor index (0-2)
 * @return true if sensor is online
 */
bool can_is_sensor_online(uint8_t sensor_idx);

/** @name System Health */

/**
 * @brief Check if CAN system is healthy
 * @return true if system healthy (>=2 sensors online)
 */
bool can_system_is_healthy(void);

/**
 * @brief Run comprehensive system diagnostics
 */
void can_run_diagnostics(void);

/** @name Testing Functions */

/**
 * @brief Test sensor CAN ID indexing logic
 */
void test_sensor_indexing(void);

/**
 * @brief Test basic CAN communication and sensor responses
 */
void test_basic_can_communication(void);

/**
 * @brief Test sensor start/stop sequence
 */
void test_sensor_start_stop_sequence(void);

/**
 * @brief Run complete CAN system test suite
 */
void test_complete_can_system(void);

/** @name Internal Functions */

/**
 * @brief Map CAN ID to sensor index
 * @param can_id CAN message ID
 * @return Sensor index or 0xFF if invalid
 */
uint8_t get_sensor_index_from_can_id(uint32_t can_id);

/**
 * @brief Process completed radar frame
 * @param sensor_idx Sensor index
 */
void process_complete_radar_frame(uint8_t sensor_idx);

/** @name Compatibility Functions */

/**
 * @brief Get active sensor count (legacy compatibility)
 * @return Number of active sensors
 */
uint8_t get_active_sensor_count(void);

#endif // MTI_CAN_H
