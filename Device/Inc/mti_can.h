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

// --- Sensor ID Offsets ---
#define CAN_SENSOR0_OFFSET 0x00
#define CAN_SENSOR1_OFFSET 0x10
#define CAN_SENSOR2_OFFSET 0x20
#define CAN_SENSOR3_OFFSET 0x30

// --- CAN Command Payload Codes (1st byte of data in a command message) ---
#define CAN_CMD_PAYLOAD_START           0x00
#define CAN_CMD_PAYLOAD_STOP            0x01
#define CAN_CMD_PAYLOAD_DC_CALIB        0x02
#define CAN_CMD_PAYLOAD_TX_BACKOFF      0x03
#define CAN_CMD_PAYLOAD_QUERY_STATUS    0x04
#define CAN_CMD_PAYLOAD_DET_THRESHOLD   0x05
#define CAN_CMD_PAYLOAD_SPREAD_SPECTRUM 0x06
#define CAN_CMD_PAYLOAD_SELECT_PROFILE  0x07
#define CAN_CMD_PAYLOAD_SET_FOV         0x08

// --- Sensor Command IDs ---
#define CAN_S0_CMD_ID (CAN_COMMAND_BASE_ID + CAN_SENSOR0_OFFSET) // 0x60
#define CAN_S1_CMD_ID (CAN_COMMAND_BASE_ID + CAN_SENSOR1_OFFSET) // 0x70
#define CAN_S2_CMD_ID (CAN_COMMAND_BASE_ID + CAN_SENSOR2_OFFSET) // 0x80
#define CAN_S3_CMD_ID (CAN_COMMAND_BASE_ID + CAN_SENSOR3_OFFSET) // 0x90

// --- Sensor Data IDs ---
#define CAN_S0_DATA_HEADER_ID        (CAN_DATA_HEADER_BASE_ID + CAN_SENSOR0_OFFSET)        // 0xA0
#define CAN_S0_DATA_OBJECT_ID        (CAN_DATA_OBJECT_BASE_ID + CAN_SENSOR0_OFFSET)        // 0xA1
#define CAN_S0_DATA_STATUS_REPLY_ID  (CAN_DATA_STATUS_REPLY_BASE_ID + CAN_SENSOR0_OFFSET)  // 0xA3
#define CAN_S0_DATA_VERSION_REPLY_ID (CAN_DATA_VERSION_REPLY_BASE_ID + CAN_SENSOR0_OFFSET) // 0xA4

#define CAN_S1_DATA_HEADER_ID        (CAN_DATA_HEADER_BASE_ID + CAN_SENSOR1_OFFSET)        // 0xB0
#define CAN_S1_DATA_OBJECT_ID        (CAN_DATA_OBJECT_BASE_ID + CAN_SENSOR1_OFFSET)        // 0xB1
#define CAN_S1_DATA_STATUS_REPLY_ID  (CAN_DATA_STATUS_REPLY_BASE_ID + CAN_SENSOR1_OFFSET)  // 0xB3
#define CAN_S1_DATA_VERSION_REPLY_ID (CAN_DATA_VERSION_REPLY_BASE_ID + CAN_SENSOR1_OFFSET) // 0xB4

#define CAN_S2_DATA_HEADER_ID        (CAN_DATA_HEADER_BASE_ID + CAN_SENSOR2_OFFSET)        // 0xC0
#define CAN_S2_DATA_OBJECT_ID        (CAN_DATA_OBJECT_BASE_ID + CAN_SENSOR2_OFFSET)        // 0xC1
#define CAN_S2_DATA_STATUS_REPLY_ID  (CAN_DATA_STATUS_REPLY_BASE_ID + CAN_SENSOR2_OFFSET)  // 0xC3
#define CAN_S2_DATA_VERSION_REPLY_ID (CAN_DATA_VERSION_REPLY_BASE_ID + CAN_SENSOR2_OFFSET) // 0xC4

#define CAN_S3_DATA_HEADER_ID        (CAN_DATA_HEADER_BASE_ID + CAN_SENSOR3_OFFSET)        // 0xD0
#define CAN_S3_DATA_OBJECT_ID        (CAN_DATA_OBJECT_BASE_ID + CAN_SENSOR3_OFFSET)        // 0xD1
#define CAN_S3_DATA_STATUS_REPLY_ID  (CAN_DATA_STATUS_REPLY_BASE_ID + CAN_SENSOR3_OFFSET)  // 0xD3
#define CAN_S3_DATA_VERSION_REPLY_ID (CAN_DATA_VERSION_REPLY_BASE_ID + CAN_SENSOR3_OFFSET) // 0xD4

/**
 * @brief Multi-sensor raw data system
 */
typedef struct
{
    radar_raw_t sensors[MAX_RADAR_SENSORS];
    uint32_t    last_message_time[MAX_RADAR_SENSORS];
    uint32_t    msgs_received[MAX_RADAR_SENSORS];
    uint8_t     active_sensor_count;
    bool        system_initialized;
} multi_radar_raw_system_t;

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

/** @name Testing Functions */
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
