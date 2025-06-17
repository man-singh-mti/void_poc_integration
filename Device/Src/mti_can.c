/**
 * @file mti_can.c
 * @brief Simplified CAN bus communication implementation
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#include "can.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include <string.h>

/*------------------------------------------------------------------------------
 * Static Variables
 *----------------------------------------------------------------------------*/

static multi_radar_raw_system_t raw_radar_system = { 0 };
static CAN_RxHeaderTypeDef      rxHeader;
static CAN_TxHeaderTypeDef      txHeader;
static uint8_t                  canRX[8] = { 0 };
static CAN_FilterTypeDef        canfil;
static uint32_t                 canMailbox;

/*------------------------------------------------------------------------------
 * Static Function Declarations
 *----------------------------------------------------------------------------*/

static void process_header_message(uint8_t sensor_idx, uint8_t *data);
static void process_object_message(uint8_t sensor_idx, uint8_t *data);
static void process_status_message(uint8_t sensor_idx, uint8_t *data);
static void process_version_message(uint8_t sensor_idx, uint8_t *data);

/*------------------------------------------------------------------------------
 * CAN System Initialization
 *----------------------------------------------------------------------------*/

bool can_setup(void)
{
    debug_send("CAN: Setting up hardware");

    // Use simpler filter like working code - accept all messages
    canfil.FilterBank           = 0;
    canfil.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh         = 0; // Accept all
    canfil.FilterIdLow          = 0; // Accept all
    canfil.FilterMaskIdHigh     = 0; // Mask = 0 (accept all)
    canfil.FilterMaskIdLow      = 0; // Mask = 0 (accept all)
    canfil.FilterScale          = CAN_FILTERSCALE_32BIT;
    canfil.FilterActivation     = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &canfil) != HAL_OK)
    {
        debug_send("CAN: Filter failed");
        return false;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        debug_send("CAN: Start failed");
        return false;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        debug_send("CAN: Interrupt activation failed");
        return false;
    }

    debug_send("CAN: Hardware setup complete");
    return true;
}

bool can_initialize_system(void)
{
    debug_send("CAN: Starting system initialization");

    if (!can_setup())
    {
        return false;
    }

    memset(&raw_radar_system, 0, sizeof(raw_radar_system));

    // Start all sensors
    uint8_t online_sensors = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        debug_send("CAN: Starting sensor %d", i);

        if (can_send_to_sensor(i, CAN_CMD_PAYLOAD_START))
        {
            HAL_Delay(100);
            // Check if sensor responds (we'll get status messages)
            if (raw_radar_system.msgs_received[i] > 0)
            {
                online_sensors++;
                debug_send("CAN: Sensor %d online", i);
            }
        }
    }

    raw_radar_system.active_sensor_count = online_sensors;
    raw_radar_system.system_initialized  = (online_sensors >= 2);

    debug_send("CAN: System init complete - %d/%d sensors online", online_sensors, MAX_RADAR_SENSORS);

    return raw_radar_system.system_initialized;
}

/*------------------------------------------------------------------------------
 * CAN Communication Functions
 *----------------------------------------------------------------------------*/

bool can_send(uint32_t ID, uint8_t message)
{
    txHeader.StdId              = 0;  // Not used for extended
    txHeader.ExtId              = ID; // Use Extended ID like working code
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.IDE                = CAN_ID_EXT; // CRITICAL: Use Extended IDs
    txHeader.DLC                = 1;
    txHeader.TransmitGlobalTime = DISABLE;

    uint8_t           data[1] = { message };
    HAL_StatusTypeDef status  = HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &canMailbox);

    if (status != HAL_OK)
    {
        debug_send("CAN: Send failed - ID=0x%03X, status=%d", ID, status);
        return false;
    }

    debug_send("CAN TX (0x%02X) 0x%02X", ID, message);
    return true;
}

bool can_send_array(uint32_t ID, uint8_t *message, size_t length)
{
    if (!message || length == 0 || length > 8)
    {
        return false;
    }

    txHeader.StdId              = 0;  // Not used for extended
    txHeader.ExtId              = ID; // Use Extended ID
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.IDE                = CAN_ID_EXT; // CRITICAL: Use Extended IDs
    txHeader.DLC                = length;
    txHeader.TransmitGlobalTime = DISABLE;

    // Debug output like working code
    char data_hex[17];
    memset(data_hex, 0, 17);
    for (int i = 0; i < length; i++)
    {
        sprintf(data_hex, "%s%02X", data_hex, message[i]);
    }
    debug_send("CAN TX (0x%02X) 0x%s", ID, data_hex);

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, message, &canMailbox);
    return (status == HAL_OK);
}

bool can_send_to_sensor(uint8_t sensor_idx, uint8_t command)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }

    uint32_t command_id = CAN_COMMAND_BASE_ID + (sensor_idx * 0x10);
    return can_send(command_id, command);
}

bool can_send_command_to_all_sensors(uint8_t command)
{
    bool all_success = true;

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (!can_send_to_sensor(i, command))
        {
            debug_send("CAN: Failed to send command 0x%02X to sensor %d", command, i);
            all_success = false;
        }
        HAL_Delay(10); // Small delay between commands
    }

    return all_success;
}

/*------------------------------------------------------------------------------
 * CAN Message Reception
 *----------------------------------------------------------------------------*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX) != HAL_OK)
    {
        debug_send("CAN RX ERROR");
        return;
    }

    // Use ExtId like working code
    uint32_t can_id     = rxHeader.ExtId;
    uint8_t  sensor_idx = get_sensor_index_from_can_id(can_id);

    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        debug_send("CAN: Unknown sensor for ID 0x%02X", can_id);
        return;
    }

    // Update sensor online status
    raw_radar_system.last_message_time[sensor_idx] = HAL_GetTick();
    raw_radar_system.msgs_received[sensor_idx]++;

    // Debug output like working code
    char data_hex[17];
    memset(data_hex, 0, 17);
    for (int i = 0; i < rxHeader.DLC; i++)
    {
        sprintf(data_hex, "%s%02X", data_hex, canRX[i]);
    }
    debug_send("CAN RX: (0x%02X) 0x%s", can_id, data_hex);

    // Process based on message type (using lower 4 bits)
    uint8_t message_type = can_id & 0x0F;

    switch (message_type)
    {
    case 0x00: // Header (0xA0, 0xB0, 0xC0, 0xD0)
        process_header_message(sensor_idx, canRX);
        break;

    case 0x01: // Object data (0xA1, 0xB1, 0xC1, 0xD1)
        process_object_message(sensor_idx, canRX);
        break;

    case 0x03: // Status reply (0xA3, 0xB3, 0xC3, 0xD3)
        process_status_message(sensor_idx, canRX);
        break;

    case 0x04: // Version reply (0xA4, 0xB4, 0xC4, 0xD4)
        process_version_message(sensor_idx, canRX);
        break;

    default:
        debug_send("CAN: Unknown message type 0x%02X from sensor %d", message_type, sensor_idx);
        break;
    }
}

/*------------------------------------------------------------------------------
 * Message Processing Functions
 *----------------------------------------------------------------------------*/

static void process_header_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data)
    {
        return;
    }

    radar_raw_t *sensor = &raw_radar_system.sensors[sensor_idx];

    // Match working code format: totalPacketLength + frameNumber
    uint32_t totalPacketLength;
    memcpy(&totalPacketLength, data, 4);
    memcpy(&sensor->frame_number, data + 4, 4);

    sensor->num_points = totalPacketLength - 129; // Like working code

    // Reset for new frame
    memset(sensor->detected_points, 0, sizeof(sensor->detected_points));

    debug_send("#frame,%d,%d", sensor->frame_number, sensor->num_points);
    debug_send("CAN: S%d header - frame %d, %d points", sensor_idx, sensor->frame_number, sensor->num_points);
}

static void process_object_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data)
    {
        return;
    }

    radar_raw_t *sensor = &raw_radar_system.sensors[sensor_idx];

    // Match working code: copy 8 bytes directly to detected point
    static uint8_t point_index[MAX_RADAR_SENSORS] = { 0 };

    if (point_index[sensor_idx] < MAX_RADAR_DETECTED_POINTS)
    {
        memcpy(&sensor->detected_points[point_index[sensor_idx]], data, 8);
        debug_send("0xA1: %f (%.0f)", sensor->detected_points[point_index[sensor_idx]][0], sensor->detected_points[point_index[sensor_idx]][1]);

        point_index[sensor_idx]++;

        // Check if frame is complete
        if (point_index[sensor_idx] >= sensor->num_points)
        {
            point_index[sensor_idx] = 0; // Reset for next frame
            process_complete_radar_frame(sensor_idx);
        }
    }
}

static void process_status_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data)
    {
        return;
    }

    debug_send("CAN: S%d status = %d", sensor_idx, data[0]);

    // Set radar status like working code
    // system_status_set(MODULE_RADAR, data[0]); // If this function exists
}

static void process_version_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data)
    {
        return;
    }

    debug_send("CAN: S%d version = %d.%d.%d", sensor_idx, data[0], data[1], data[2]);

    // Set version like working code
    // version_set(MODULE_RADAR, data[0], data[1], data[2]); // If this function exists
}

void process_complete_radar_frame(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    radar_raw_t *sensor = &raw_radar_system.sensors[sensor_idx];

    sensor->timestamp_ms       = HAL_GetTick();
    sensor->new_data_available = true;

    debug_send("CAN: S%d frame complete", sensor_idx);

// Notify radar layer when it exists
#ifdef MTI_RADAR_H
    radar_notify_new_raw_data(sensor_idx);
#endif
}

/*------------------------------------------------------------------------------
 * Data Access Functions
 *----------------------------------------------------------------------------*/

radar_raw_t *can_get_raw_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return NULL;
    }
    return &raw_radar_system.sensors[sensor_idx];
}

bool can_has_new_raw_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }
    return raw_radar_system.sensors[sensor_idx].new_data_available;
}

void can_mark_raw_data_processed(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }
    raw_radar_system.sensors[sensor_idx].new_data_available = false;
}

uint8_t can_get_online_sensor_count(void)
{
    uint8_t  count        = 0;
    uint32_t current_time = HAL_GetTick();

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if ((current_time - raw_radar_system.last_message_time[i]) < 3000)
        {
            count++;
        }
    }

    raw_radar_system.active_sensor_count = count;
    return count;
}

bool can_is_sensor_online(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }

    uint32_t current_time = HAL_GetTick();
    uint32_t last_msg     = raw_radar_system.last_message_time[sensor_idx];

    return (current_time - last_msg) < 3000; // 3 second timeout
}

bool can_system_is_healthy(void)
{
    return raw_radar_system.system_initialized && (can_get_online_sensor_count() >= 2);
}

/*------------------------------------------------------------------------------
 * System Management
 *----------------------------------------------------------------------------*/

void can_run_diagnostics(void)
{
    debug_send("=== CAN System Diagnostics ===");
    debug_send("Command Base: 0x60, Data Base: 0xA0");
    debug_send("System initialized: %s", raw_radar_system.system_initialized ? "YES" : "NO");
    debug_send("Active sensors: %d/%d", can_get_online_sensor_count(), MAX_RADAR_SENSORS);
    debug_send("");

    uint32_t current_time = HAL_GetTick();

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint32_t last_msg_age = current_time - raw_radar_system.last_message_time[i];
        bool     online       = can_is_sensor_online(i);
        uint32_t cmd_id       = CAN_COMMAND_BASE_ID + (i * 0x10);
        uint32_t header_id    = CAN_DATA_HEADER_BASE_ID + (i * 0x10);

        debug_send("Sensor %d: %s (%d msgs, %dms ago)", i, online ? "ONLINE" : "OFFLINE", raw_radar_system.msgs_received[i], last_msg_age);
        debug_send("  Command ID: 0x%02X, Data Header ID: 0x%02X", cmd_id, header_id);
    }

    debug_send("===============================");
}

uint8_t get_sensor_index_from_can_id(uint32_t can_id)
{
    // Command messages: 0x60, 0x70, 0x80, 0x90 (0x60 base + 0x10 offsets)
    if (can_id >= 0x60 && can_id <= 0x90 && ((can_id & 0x0F) == 0x00))
    {
        uint8_t sensor_idx = (can_id - 0x60) / 0x10;
        if (sensor_idx < MAX_RADAR_SENSORS)
        {
            return sensor_idx;
        }
    }

    // Data messages: 0xA0-0xAF, 0xB0-0xBF, 0xC0-0xCF, 0xD0-0xDF (0xA0 base + 0x10 offsets)
    if (can_id >= 0xA0 && can_id <= 0xDF)
    {
        uint8_t sensor_idx = (can_id - 0xA0) / 0x10;
        if (sensor_idx < MAX_RADAR_SENSORS)
        {
            return sensor_idx;
        }
    }

    return 0xFF; // Invalid
}

/*------------------------------------------------------------------------------
 * Testing Functions
 *----------------------------------------------------------------------------*/

void test_sensor_indexing(void)
{
    debug_send("=== Testing Sensor Indexing (0x60 Command Base) ===");

    struct
    {
        uint32_t    can_id;
        uint8_t     expected;
        const char *desc;
    } tests[] = { // Command messages (0x60 base)
                  { 0x60, 0, "S0 Command (0x60)" },
                  { 0x70, 1, "S1 Command (0x70)" },
                  { 0x80, 2, "S2 Command (0x80)" },
                  { 0x90, 3, "S3 Command (0x90)" },

                  // Data Header messages (0xA0 base) - Key identification
                  { CAN_S0_DATA_HEADER_ID, 0, "S0 Header (0xA0)" },
                  { CAN_S1_DATA_HEADER_ID, 1, "S1 Header (0xB0)" },
                  { CAN_S2_DATA_HEADER_ID, 2, "S2 Header (0xC0)" },
                  { CAN_S3_DATA_HEADER_ID, 3, "S3 Header (0xD0)" },

                  // Other data messages
                  { CAN_S0_DATA_OBJECT_ID, 0, "S0 Object (0xA1)" },
                  { CAN_S1_DATA_OBJECT_ID, 1, "S1 Object (0xB1)" },
                  { CAN_S2_DATA_OBJECT_ID, 2, "S2 Object (0xC1)" },
                  { CAN_S3_DATA_OBJECT_ID, 3, "S3 Object (0xD1)" },

                  { CAN_S0_DATA_STATUS_REPLY_ID, 0, "S0 Status (0xA3)" },
                  { CAN_S1_DATA_STATUS_REPLY_ID, 1, "S1 Status (0xB3)" },
                  { CAN_S2_DATA_STATUS_REPLY_ID, 2, "S2 Status (0xC3)" },
                  { CAN_S3_DATA_STATUS_REPLY_ID, 3, "S3 Status (0xD3)" },

                  { CAN_S0_DATA_VERSION_REPLY_ID, 0, "S0 Version (0xA4)" },
                  { CAN_S1_DATA_VERSION_REPLY_ID, 1, "S1 Version (0xB4)" },
                  { CAN_S2_DATA_VERSION_REPLY_ID, 2, "S2 Version (0xC4)" },
                  { CAN_S3_DATA_VERSION_REPLY_ID, 3, "S3 Version (0xD4)" },

                  // Invalid cases
                  { 0x50, 0xFF, "Invalid Low" },
                  { 0xE0, 0xFF, "Invalid High" },
                  { 0x61, 0xFF, "Invalid Command (0x61)" },
                  { 0xA5, 0xFF, "Invalid Data (0xA5)" }
    };

    uint8_t passed = 0;
    uint8_t total  = sizeof(tests) / sizeof(tests[0]);

    debug_send("Command Base: 0x60, Data Base: 0x A0");
    debug_send("Sensor Offsets: S0=+0x00, S1=+0x10, S2=+0x20, S3=+0x30");
    debug_send("");

    for (uint8_t i = 0; i < total; i++)
    {
        uint8_t result = get_sensor_index_from_can_id(tests[i].can_id);
        if (result == tests[i].expected)
        {
            passed++;
            debug_send("PASS: %s -> S%d", tests[i].desc, result);
        }
        else
        {
            debug_send("FAIL: %s -> S%d (expected S%d)", tests[i].desc, result, tests[i].expected);
        }
    }

    debug_send("");
    debug_send("Indexing Tests: %d/%d passed", passed, total);

    if (passed == total)
    {
        debug_send("ALL TESTS PASSED - Addressing scheme is correct!");
    }
    else
    {
        debug_send("SOME TESTS FAILED - Check addressing scheme!");
    }
}

void test_sensor_responses(void)
{
    debug_send("=== Testing Sensor Responses ===");

    // Send status query to all sensors
    debug_send("Sending status queries to all sensors...");
    can_send_command_to_all_sensors(CAN_CMD_PAYLOAD_QUERY_STATUS);

    HAL_Delay(200); // Wait for responses

    // Check results
    uint8_t responding = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        bool online = can_is_sensor_online(i);
        debug_send("Sensor %d (Header ID 0x%02X): %s", i, CAN_DATA_HEADER_BASE_ID + (i * 0x10), online ? "RESPONDING" : "NO RESPONSE");
        if (online)
            responding++;
    }

    debug_send("Response Test: %d/%d sensors responding", responding, MAX_RADAR_SENSORS);
}
