/**
 * @file mti_can.h
 * @brief Clean CAN middleware for radar sensors - Event-driven data collection
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#ifndef MTI_CAN_H
#define MTI_CAN_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include <stdint.h>
#include "vmt_uart.h"

/** @name Command Payload Codes */
#define CAN_CMD_START           0x00 // Start sensor
#define CAN_CMD_STOP            0x01 // Stop sensor
#define CAN_CMD_DC_CALIB        0x02 // DC Calibration
#define CAN_CMD_TX_BACKOFF      0x03 // TX Backoff
#define CAN_CMD_QUERY_STATUS    0x04 // Query Status
#define CAN_CMD_DET_THRESHOLD   0x05 // Detection Threshold
#define CAN_CMD_SPREAD_SPECTRUM 0x06 // Spread Spectrum
#define CAN_CMD_SELECT_PROFILE  0x07 // Select Profile
#define CAN_CMD_SET_FOV         0x08 // Set Field of View

/** @name Command IDs (what we transmit) */
#define CAN_S0_CMD_ID 0x00000060 // Sensor 0 command
#define CAN_S1_CMD_ID 0x00000070 // Sensor 1 command
#define CAN_S2_CMD_ID 0x00000080 // Sensor 2 command

/** @name Data IDs (what we receive) */
// Sensor 0
#define CAN_S0_HEADER_ID  0x000000A0 // Header data
#define CAN_S0_OBJECT_ID  0x000000A1 // Object/Detection data
#define CAN_S0_PROFILE_ID 0x000000A2 // Range Profile data
#define CAN_S0_STATUS_ID  0x000000A3 // Status reply
#define CAN_S0_VERSION_ID 0x000000A4 // Version reply

// Sensor 1
#define CAN_S1_HEADER_ID  0x000000B0 // Header data
#define CAN_S1_OBJECT_ID  0x000000B1 // Object/Detection data
#define CAN_S1_PROFILE_ID 0x000000B2 // Range Profile data
#define CAN_S1_STATUS_ID  0x000000B3 // Status reply
#define CAN_S1_VERSION_ID 0x000000B4 // Version reply

// Sensor 2
#define CAN_S2_HEADER_ID  0x000000C0 // Header data
#define CAN_S2_OBJECT_ID  0x000000C1 // Object/Detection data
#define CAN_S2_PROFILE_ID 0x000000C2 // Range Profile data
#define CAN_S2_STATUS_ID  0x000000C3 // Status reply
#define CAN_S2_VERSION_ID 0x000000C4 // Version reply

/** @name System Status */
typedef enum
{
    RADAR_INITIALISING,
    RADAR_READY,
    RADAR_CHIRPING,
    RADAR_STOPPED,
} can_status_t;

/** @name Sensor Data Structure - Event Driven */
typedef struct
{
    // Live data (auto-updated by CAN interrupt)
    uint32_t frame_number;            // From header message
    uint8_t  num_points;              // Number of detection points
    float    detection_points[20][2]; // [distance_m, SNR] from object messages
    uint8_t  status_code;             // From status reply
    uint8_t  fw_version[3];           // [major, minor, patch] from version reply

    // Status tracking (auto-updated)
    uint32_t     last_msg_time; // Timestamp of last message
    uint32_t     msg_count;     // Total messages received
    bool         online;        // Sensor responding
    can_status_t status;        // Current sensor status
} can_sensor_t;

/** @name System Data Structure */
typedef struct
{
    can_sensor_t sensors[3];         // All sensor data
    bool         system_initialized; // System ready flag
    uint8_t      online_count;       // Number of online sensors
    can_status_t system_status;      // Overall system status
    uint32_t     init_start_time;    // For timing measurements
} can_system_t;

/*------------------------------------------------------------------------------
 * Core API - Simple and Clean
 *----------------------------------------------------------------------------*/

/** @name Initialization */
bool can_init(void); // Complete initialization sequence

/** @name Commands */
bool can_send_command(uint8_t sensor_id, uint8_t command);                                  // Single command
bool can_send_command_data(uint8_t sensor_id, uint8_t command, uint8_t *data, uint8_t len); // Command + parameters
bool can_send_to_all_sensors(uint8_t command);                                              // Broadcast command

/** @name Data Access - Direct */
can_sensor_t *can_get_sensor(uint8_t sensor_id); // Direct access to live data
can_system_t *can_get_system(void);              // Complete system data
uint8_t       can_get_online_count(void);        // Online sensor count

/** @name Status */
bool         can_is_system_healthy(void); // System health check
can_status_t can_get_system_status(void); // Current system status

/** @name Processing */
void can_process_timeouts(void); // Timeout management (main loop)

/** @name Test & Debug Functions */
void can_test_periodic(void);         // Periodic test (every 10s)
void can_debug_system_status(void);   // Print complete status
void can_debug_sensor_indexing(void); // Test CAN ID mapping

#endif // MTI_CAN_H
