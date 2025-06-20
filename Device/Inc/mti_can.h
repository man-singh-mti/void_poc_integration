/**
 * @file mti_can.h
 * @brief Simplified CAN bus communication for radar sensors
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#ifndef MTI_CAN_H
#define MTI_CAN_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include "vmt_uart.h"
#include "mti_radar_types.h"

// --- Base CAN ID Definitions ---
#define CAN_COMMAND_BASE_ID            0x60 // Base for outgoing commands to sensors
#define CAN_DATA_HEADER_BASE_ID        0xA0 // Base for Header data from sensors
#define CAN_DATA_OBJECT_BASE_ID        0xA1 // Base for Object/Detected Points data
#define CAN_DATA_PROFILE_BASE_ID       0xA2 // Base for Range Profile data
#define CAN_DATA_STATUS_REPLY_BASE_ID  0xA3 // Base for Status reply from sensor
#define CAN_DATA_VERSION_REPLY_BASE_ID 0xA4 // Base for Version reply from sensor

// --- Sensor Command IDs (0x60 base + 0x10 offsets) ---
#define CAN_S0_COMMAND_ID 0x60
#define CAN_S1_COMMAND_ID 0x70
#define CAN_S2_COMMAND_ID 0x80
#define CAN_S3_COMMAND_ID 0x90

// --- Sensor Data IDs (0xA0 base + 0x10 offsets) ---
#define CAN_S0_DATA_HEADER_ID        0xA0
#define CAN_S0_DATA_OBJECT_ID        0xA1
#define CAN_S0_DATA_STATUS_REPLY_ID  0xA3
#define CAN_S0_DATA_VERSION_REPLY_ID 0xA4

#define CAN_S1_DATA_HEADER_ID        0xB0
#define CAN_S1_DATA_OBJECT_ID        0xB1
#define CAN_S1_DATA_STATUS_REPLY_ID  0xB3
#define CAN_S1_DATA_VERSION_REPLY_ID 0xB4

#define CAN_S2_DATA_HEADER_ID        0xC0
#define CAN_S2_DATA_OBJECT_ID        0xC1
#define CAN_S2_DATA_STATUS_REPLY_ID  0xC3
#define CAN_S2_DATA_VERSION_REPLY_ID 0xC4

#define CAN_S3_DATA_HEADER_ID        0xD0
#define CAN_S3_DATA_OBJECT_ID        0xD1
#define CAN_S3_DATA_STATUS_REPLY_ID  0xD3
#define CAN_S3_DATA_VERSION_REPLY_ID 0xD4

// --- CAN Command Payloads (from working code) ---
#define CAN_CMD           0x80       // Command ID from working code
#define CAN_START         0x00       // Start command
#define CAN_STOP          0x01       // Stop command
#define CAN_CAL           0x02       // Calibration command
#define CAN_POWER         0x03       // Power command
#define CAN_STATUS        0x04       // Status query command
#define CAN_THRESHOLD     0x05       // Threshold setting
#define CAN_SPREAD        0x06       // Spread setting
#define CAN_PROFILE       0x07       // Profile selection
#define CAN_FOV           0x08       // Field of view setting
#define CAN_VERSION_QUERY CAN_STATUS // Version is queried with status command (0x04)

// Aliases for compatibility with your code
#define CAN_CMD_PAYLOAD_START          CAN_START
#define CAN_CMD_PAYLOAD_STOP           CAN_STOP
#define CAN_CMD_PAYLOAD_QUERY_STATUS   CAN_STATUS
#define CAN_CMD_PAYLOAD_SELECT_PROFILE CAN_PROFILE
#define CAN_CMD_PAYLOAD_DET_THRESHOLD  CAN_THRESHOLD

/** @name CAN System Functions */
bool can_setup(void);
bool can_initialize_system(void);

/** @name CAN Communication Functions */
bool can_send(uint32_t ID, uint8_t message);
bool can_send_array(uint32_t ID, uint8_t *message, size_t length);
bool can_send_to_sensor(uint8_t sensor_idx, uint8_t command);
bool can_send_command_to_all_sensors(uint8_t command);

/** @name CAN Data Access Functions */
radar_raw_t *can_get_raw_data(uint8_t sensor_idx);
bool         can_has_new_raw_data(uint8_t sensor_idx);
void         can_mark_raw_data_processed(uint8_t sensor_idx);
uint8_t      can_get_online_sensor_count(void);
bool         can_is_sensor_online(uint8_t sensor_idx);

/** @name CAN System Health */
bool can_system_is_healthy(void);
void can_run_diagnostics(void);

/** @name Compatibility Functions (for existing code) */
uint8_t get_active_sensor_count(void); // Alias for can_get_online_sensor_count()

/** @name Testing Functions */
void test_complete_can_system(void);
void test_basic_can_communication(void);
void test_sensor_start_stop_sequence(void);
void test_sensor_indexing(void);
void test_sensor_responses(void);

/** @name Message Processing (Internal) */
uint8_t get_sensor_index_from_can_id(uint32_t can_id);
void    process_complete_radar_frame(uint8_t sensor_idx);

// Internal message processing functions
static void process_header_message(uint8_t sensor_idx, uint8_t *data);
static void process_object_message(uint8_t sensor_idx, uint8_t *data);
static void process_status_message(uint8_t sensor_idx, uint8_t *data);
static void process_version_message(uint8_t sensor_idx, uint8_t *data);

#endif // MTI_CAN_H
