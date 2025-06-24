/**
 * @file mti_can.h
 * @brief CAN bus communication for radar sensors using Extended 29-bit IDs
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#ifndef MTI_CAN_H
#define MTI_CAN_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include "vmt_uart.h"
#include "mti_radar_types.h"

// --- CAN Command Payload Codes (1st byte of data in a command message) ---
#define CAN_CMD_PAYLOAD_START           0x00 // Start sensor
#define CAN_CMD_PAYLOAD_STOP            0x01 // Stop sensor
#define CAN_CMD_PAYLOAD_DC_CALIB        0x02 // DC Calibration
#define CAN_CMD_PAYLOAD_TX_BACKOFF      0x03 // TX Backoff
#define CAN_CMD_PAYLOAD_QUERY_STATUS    0x04 // Query Status
#define CAN_CMD_PAYLOAD_DET_THRESHOLD   0x05 // Detection Threshold
#define CAN_CMD_PAYLOAD_SPREAD_SPECTRUM 0x06 // Spread Spectrum
#define CAN_CMD_PAYLOAD_SELECT_PROFILE  0x07 // Select Profile
#define CAN_CMD_PAYLOAD_SET_FOV         0x08 // Set Field of View

// --- Sensor Command Extended 29-bit IDs (what we transmit) ---
#define CAN_S0_CMD_ID 0x00000060 // 96 decimal  - Sensor 0 command
#define CAN_S1_CMD_ID 0x00000070 // 112 decimal - Sensor 1 command
#define CAN_S2_CMD_ID 0x00000080 // 128 decimal - Sensor 2 command

// --- Sensor Data Extended 29-bit IDs (what we receive) ---
// Sensor 0 Data IDs (160-164 decimal range)
#define CAN_S0_DATA_HEADER_ID        0x000000A0 // 160 - Header data
#define CAN_S0_DATA_OBJECT_ID        0x000000A1 // 161 - Object/Detection data
#define CAN_S0_DATA_PROFILE_ID       0x000000A2 // 162 - Range Profile data
#define CAN_S0_DATA_STATUS_REPLY_ID  0x000000A3 // 163 - Status reply
#define CAN_S0_DATA_VERSION_REPLY_ID 0x000000A4 // 164 - Version reply

// Sensor 1 Data IDs (176-180 decimal range)
#define CAN_S1_DATA_HEADER_ID        0x000000B0 // 176 - Header data
#define CAN_S1_DATA_OBJECT_ID        0x000000B1 // 177 - Object/Detection data
#define CAN_S1_DATA_PROFILE_ID       0x000000B2 // 178 - Range Profile data
#define CAN_S1_DATA_STATUS_REPLY_ID  0x000000B3 // 179 - Status reply
#define CAN_S1_DATA_VERSION_REPLY_ID 0x000000B4 // 180 - Version reply

// Sensor 2 Data IDs (192-196 decimal range)
#define CAN_S2_DATA_HEADER_ID        0x000000C0 // 192 - Header data
#define CAN_S2_DATA_OBJECT_ID        0x000000C1 // 193 - Object/Detection data
#define CAN_S2_DATA_PROFILE_ID       0x000000C2 // 194 - Range Profile data
#define CAN_S2_DATA_STATUS_REPLY_ID  0x000000C3 // 195 - Status reply
#define CAN_S2_DATA_VERSION_REPLY_ID 0x000000C4 // 196 - Version reply

// --- System Configuration ---
#define MAX_RADAR_SENSORS_ACTIVE 3 // Only using sensors 0, 1, 2

// --- Base ID Definitions for Pattern Recognition (Extended IDs) ---
#define CAN_COMMAND_BASE_ID            0x00000060 // Base for outgoing commands
#define CAN_DATA_HEADER_BASE_ID        0x000000A0 // Base for Header data
#define CAN_DATA_OBJECT_BASE_ID        0x000000A1 // Base for Object data
#define CAN_DATA_PROFILE_BASE_ID       0x000000A2 // Base for Profile data
#define CAN_DATA_STATUS_REPLY_BASE_ID  0x000000A3 // Base for Status replies
#define CAN_DATA_VERSION_REPLY_BASE_ID 0x000000A4 // Base for Version replies

// --- Sensor ID Offset Pattern ---
#define CAN_SENSOR_ID_OFFSET 0x10 // 16 decimal offset between sensors
#define CAN_SENSOR0_OFFSET   0x00 // S0: +0  (A0-A4)
#define CAN_SENSOR1_OFFSET   0x10 // S1: +16 (B0-B4)
#define CAN_SENSOR2_OFFSET   0x20 // S2: +32 (C0-C4)

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

/** @name Compatibility Functions (for existing code) */
uint8_t get_active_sensor_count(void); // Alias for can_get_online_sensor_count()

/** @name Testing Functions */
void test_sensor_indexing(void);
void test_sensor_responses(void);

void test_can_message_processing(void);

void can_periodic_sensor_status_debug(void);
void can_periodic_version_request(void);
bool can_init_debug_timers(void);

/** @name Message Processing (Internal) */
uint8_t get_sensor_index_from_can_id(uint32_t can_id);
void    process_complete_radar_frame(uint8_t sensor_idx);

// Note: These functions are now implemented in the .c file, not static
void process_header_message(uint8_t sensor_idx, uint8_t *data);
void process_object_message(uint8_t sensor_idx, uint8_t *data);
void process_status_message(uint8_t sensor_idx, uint8_t *data);
void process_version_message(uint8_t sensor_idx, uint8_t *data);

#endif // MTI_CAN_H
