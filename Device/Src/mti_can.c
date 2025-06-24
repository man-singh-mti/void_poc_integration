/**
 * @file mti_can.c
 * @brief CAN bus communication implementation using Extended IDs (corrected)
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

static uint32_t last_debug_print_time     = 0;
static uint32_t last_version_request_time = 0;
static bool     debug_timers_initialized  = false;

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
    debug_send("CAN: Setting up hardware with extended IDs");

    // Configure filter to accept ALL extended ID messages (no filtering)
    canfil.FilterBank           = 0;
    canfil.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfil.FilterScale          = CAN_FILTERSCALE_32BIT;
    canfil.FilterIdHigh         = 0x0000; // Accept all
    canfil.FilterIdLow          = 0x0000; // Accept all
    canfil.FilterMaskIdHigh     = 0x0000; // No masking (accept all)
    canfil.FilterMaskIdLow      = 0x0000; // No masking (accept all)
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterActivation     = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &canfil) != HAL_OK)
    {
        debug_send("CAN: Filter configuration failed");
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

    debug_send("CAN: Hardware setup complete - accepting ALL extended ID messages");
    debug_send("CAN: Command IDs: 0x%08X, 0x%08X, 0x%08X", 0x00000060, 0x00000070, 0x00000080);
    debug_send("CAN: Data IDs: 0x%08X-0x%08X, 0x%08X-0x%08X, 0x%08X-0x%08X", 0x000000A0, 0x000000A4, 0x000000B0, 0x000000B4, 0x000000C0, 0x000000C4);

    return true;
}

bool can_initialize_system(void)
{
    debug_send("CAN: Starting system initialization");

    // CRITICAL: Wait for CAN sensors to power up and initialize
    debug_send("CAN: Waiting 5 seconds for sensor power-up...");
    HAL_Delay(5000); // 5 second delay for sensor initialization

    // Step 1: Setup CAN hardware
    if (!can_setup())
    {
        debug_send("CAN: Hardware setup failed");
        return false;
    }

    // Step 2: Initialize system state
    memset(&raw_radar_system, 0, sizeof(raw_radar_system));
    debug_send("CAN: System state cleared");

    // Step 3: Sequential sensor initialization (only sensors 0, 1, 2)
    debug_send("CAN: Starting sensors sequentially with command 0x00");

    for (uint8_t i = 0; i < 3; i++) // Changed from MAX_RADAR_SENSORS to 3
    {
        debug_send("CAN: Starting sensor %d...", i);

        // Send start command (0x00)
        if (!can_send_to_sensor(i, CAN_CMD_PAYLOAD_START))
        {
            debug_send("CAN: Failed to send start command to sensor %d", i);
            continue;
        }

        // Wait for initial response (increased timeout)
        uint32_t wait_start        = HAL_GetTick();
        uint32_t initial_msg_count = raw_radar_system.msgs_received[i];

        while ((HAL_GetTick() - wait_start) < 5000) // 5 seconds
        {
            if (raw_radar_system.msgs_received[i] > initial_msg_count)
            {
                debug_send("CAN: Sensor %d responded after %dms", i, HAL_GetTick() - wait_start);
                break;
            }
            HAL_Delay(100);
        }

        if (raw_radar_system.msgs_received[i] == initial_msg_count)
        {
            debug_send("CAN: Sensor %d no response to start command", i);
        }

        // Delay between sensors to prevent bus flooding
        HAL_Delay(500);
    }

    // Step 4: Query status from responding sensors
    debug_send("CAN: Querying status from all sensors with command 0x04");

    for (uint8_t i = 0; i < 3; i++) // Changed from MAX_RADAR_SENSORS to 3
    {
        if (raw_radar_system.msgs_received[i] > 0)
        {
            debug_send("CAN: Querying status from sensor %d", i);
            can_send_to_sensor(i, CAN_CMD_PAYLOAD_QUERY_STATUS);
            HAL_Delay(100);
        }
    }

    // Step 5: Monitor for ongoing responses (8 seconds)
    debug_send("CAN: Monitoring for sensor data for 8 seconds...");
    debug_send("CAN: Expected data on IDs: 0xA0-0xA4, 0xB0-0xB4, 0xC0-0xC4");

    uint32_t       start_time          = HAL_GetTick();
    uint32_t       last_status_time    = start_time;
    const uint32_t MONITOR_DURATION_MS = 8000; // 8 seconds
    const uint32_t STATUS_INTERVAL_MS  = 2000; // Status update every 2 seconds

    while ((HAL_GetTick() - start_time) < MONITOR_DURATION_MS)
    {
        // Print status every 2 seconds
        if ((HAL_GetTick() - last_status_time) >= STATUS_INTERVAL_MS)
        {
            uint8_t responding_sensors = 0;
            for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
            {
                if (raw_radar_system.msgs_received[i] > 0)
                {
                    responding_sensors++;
                }
            }

            uint32_t elapsed = (HAL_GetTick() - start_time) / 1000;
            debug_send("CAN: Monitor [%ds] - %d/%d sensors active", elapsed, responding_sensors, MAX_RADAR_SENSORS);

            last_status_time = HAL_GetTick();
        }

        HAL_Delay(100);
    }

    // Step 6: Final assessment
    uint8_t online_sensors = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (raw_radar_system.msgs_received[i] > 0)
        {
            online_sensors++;
            debug_send("CAN: Sensor %d ONLINE - %d messages received", i, raw_radar_system.msgs_received[i]);
        }
        else
        {
            debug_send("CAN: Sensor %d OFFLINE - No messages received", i);
        }
    }

    // Step 7: Update system state
    raw_radar_system.active_sensor_count = online_sensors;
    raw_radar_system.system_initialized  = (online_sensors >= 2); // Need at least 2 sensors

    // Step 8: Final status
    debug_send("CAN: ========================================");
    debug_send("CAN: Sequential initialization complete");
    debug_send("CAN: Sensors online: %d/%d", online_sensors, MAX_RADAR_SENSORS);
    debug_send("CAN: System status: %s", raw_radar_system.system_initialized ? "READY" : "INSUFFICIENT SENSORS");
    debug_send("CAN: ========================================");

    return raw_radar_system.system_initialized;
}

/*------------------------------------------------------------------------------
 * CAN Communication Functions
 *----------------------------------------------------------------------------*/

bool can_send(uint32_t ID, uint8_t message)
{
    // Wait for available mailbox with timeout
    uint32_t timeout_start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        if ((HAL_GetTick() - timeout_start) > 100) // 100ms timeout
        {
            debug_send("CAN: Send timeout - no free mailboxes");
            return false;
        }
        HAL_Delay(1);
    }

    txHeader.DLC                = 1;
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = 0;
    txHeader.ExtId              = ID;
    txHeader.TransmitGlobalTime = DISABLE;

    uint8_t           data[1] = { message };
    HAL_StatusTypeDef status  = HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &canMailbox);

    if (status != HAL_OK)
    {
        debug_send("CAN: Send failed - ExtID=0x%08X, status=%d", ID, status);
        return false;
    }

    // debug_send("CAN TX (0x%08X) 0x%02X", ID, message);

    // Small delay to prevent bus flooding
    HAL_Delay(10);

    return true;
}

bool can_send_array(uint32_t ID, uint8_t *message, size_t length)
{
    if (!message || length == 0 || length > 8)
    {
        return false;
    }

    txHeader.DLC                = length;
    txHeader.IDE                = CAN_ID_EXT; // Use Extended ID
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = 0;  // Clear standard ID
    txHeader.ExtId              = ID; // Use extended ID field
    txHeader.TransmitGlobalTime = DISABLE;

    // Debug output
    char data_hex[25];
    memset(data_hex, 0, 25);
    for (int i = 0; i < length; i++)
    {
        sprintf(data_hex, "%s%02X", data_hex, message[i]);
    }
    debug_send("CAN TX (0x%08X) 0x%s", ID, data_hex);

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, message, &canMailbox);
    return (status == HAL_OK);
}

// Update the sensor validation function

bool can_send_to_sensor(uint8_t sensor_idx, uint8_t command)
{
    if (sensor_idx >= 3) // Changed from MAX_RADAR_SENSORS to 3
    {
        debug_send("CAN: Invalid sensor index %d (only 0-2 supported)", sensor_idx);
        return false;
    }

    uint32_t command_id = CAN_COMMAND_BASE_ID + (sensor_idx * CAN_SENSOR_ID_OFFSET);
    return can_send(command_id, command);
}

// Update the all sensors command function

bool can_send_command_to_all_sensors(uint8_t command)
{
    bool all_success = true;

    for (uint8_t i = 0; i < 3; i++) // Changed from MAX_RADAR_SENSORS to 3
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
    // Check if FIFO is about to overflow
    uint32_t fill_level = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
    if (fill_level > 2) // If more than 2 messages pending
    {
        debug_send("CAN: RX FIFO overload (%d msgs)", fill_level);
        // Clear excess messages to prevent overflow
        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 1)
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX);
        }
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX) != HAL_OK)
    {
        debug_send("CAN RX ERROR");
        return;
    }

    // Use ExtId for extended ID messages
    uint32_t can_id = rxHeader.ExtId;

    // Simplified debug output - just show the CAN ID
    // debug_send("RX:%02X", (can_id & 0xFF));

    // Process messages based on extended IDs
    // Sensor 0 (0x000000A0-0x000000A4)
    if (can_id == 0x000000A0) // CAN_S0_DATA_HEADER_ID
    {
        raw_radar_system.last_message_time[0] = HAL_GetTick();
        raw_radar_system.msgs_received[0]++;
        process_header_message(0, canRX);
    }
    else if (can_id == 0x000000A1) // CAN_S0_DATA_OBJECT_ID
    {
        raw_radar_system.last_message_time[0] = HAL_GetTick();
        raw_radar_system.msgs_received[0]++;
        process_object_message(0, canRX);
    }
    else if (can_id == 0x000000A3) // CAN_S0_DATA_STATUS_REPLY_ID
    {
        raw_radar_system.last_message_time[0] = HAL_GetTick();
        raw_radar_system.msgs_received[0]++;
        process_status_message(0, canRX);
    }
    else if (can_id == 0x000000A4) // CAN_S0_DATA_VERSION_REPLY_ID
    {
        raw_radar_system.last_message_time[0] = HAL_GetTick();
        raw_radar_system.msgs_received[0]++;
        process_version_message(0, canRX);
    }
    // Sensor 1 (0x000000B0-0x000000B4)
    else if (can_id == 0x000000B0) // CAN_S1_DATA_HEADER_ID
    {
        raw_radar_system.last_message_time[1] = HAL_GetTick();
        raw_radar_system.msgs_received[1]++;
        process_header_message(1, canRX);
    }
    else if (can_id == 0x000000B1) // CAN_S1_DATA_OBJECT_ID
    {
        raw_radar_system.last_message_time[1] = HAL_GetTick();
        raw_radar_system.msgs_received[1]++;
        process_object_message(1, canRX);
    }
    else if (can_id == 0x000000B3) // CAN_S1_DATA_STATUS_REPLY_ID
    {
        raw_radar_system.last_message_time[1] = HAL_GetTick();
        raw_radar_system.msgs_received[1]++;
        process_status_message(1, canRX);
    }
    else if (can_id == 0x000000B4) // CAN_S1_DATA_VERSION_REPLY_ID
    {
        raw_radar_system.last_message_time[1] = HAL_GetTick();
        raw_radar_system.msgs_received[1]++;
        process_version_message(1, canRX);
    }
    // Sensor 2 (0x000000C0-0x000000C4)
    else if (can_id == 0x000000C0) // CAN_S2_DATA_HEADER_ID
    {
        raw_radar_system.last_message_time[2] = HAL_GetTick();
        raw_radar_system.msgs_received[2]++;
        process_header_message(2, canRX);
    }
    else if (can_id == 0x000000C1) // CAN_S2_DATA_OBJECT_ID
    {
        raw_radar_system.last_message_time[2] = HAL_GetTick();
        raw_radar_system.msgs_received[2]++;
        process_object_message(2, canRX);
    }
    else if (can_id == 0x000000C3) // CAN_S2_DATA_STATUS_REPLY_ID
    {
        raw_radar_system.last_message_time[2] = HAL_GetTick();
        raw_radar_system.msgs_received[2]++;
        process_status_message(2, canRX);
    }
    else if (can_id == 0x000000C4) // CAN_S2_DATA_VERSION_REPLY_ID
    {
        raw_radar_system.last_message_time[2] = HAL_GetTick();
        raw_radar_system.msgs_received[2]++;
        process_version_message(2, canRX);
    }
    else
    {
        debug_send("UNK:%08X", can_id);
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

    // Extract header data - First byte indicates data type
    uint8_t data_type = data[0]; // 0x1A, 0x1B, 0x1C etc.

    // Frame number is in bytes 4-7 (little endian)
    memcpy(&sensor->frame_number, data + 4, 4);

    // Calculate number of detection points
    if (data_type == 0x1A)
    {
        sensor->num_points = 16; // Sensor 0
    }
    else if (data_type == 0x1B)
    {
        sensor->num_points = 17; // Sensor 1
    }
    else if (data_type == 0x1C)
    {
        sensor->num_points = 18; // Sensor 2
    }
    else
    {
        sensor->num_points = data_type - 10; // Fallback
    }

    // Reset for new frame
    memset(sensor->detected_points, 0, sizeof(sensor->detected_points));

    // Minimal debug output
    // debug_send("F%d:%d", sensor->frame_number, sensor->num_points);
}

static void process_object_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data)
    {
        return;
    }

    radar_raw_t *sensor = &raw_radar_system.sensors[sensor_idx];

    static uint8_t frame_point_count[MAX_RADAR_SENSORS] = { 0 };

    // Process first detection point (bytes 0-3)
    if (frame_point_count[sensor_idx] < MAX_RADAR_DETECTED_POINTS)
    {
        memcpy(&sensor->detected_points[frame_point_count[sensor_idx]], data, 4);
        frame_point_count[sensor_idx]++;
    }

    // Process second detection point (bytes 4-7)
    if (frame_point_count[sensor_idx] < MAX_RADAR_DETECTED_POINTS)
    {
        memcpy(&sensor->detected_points[frame_point_count[sensor_idx]], data + 4, 4);
        frame_point_count[sensor_idx]++;
    }

    // Check if frame is complete
    if (frame_point_count[sensor_idx] >= sensor->num_points)
    {
        frame_point_count[sensor_idx] = 0; // Reset for next frame
        process_complete_radar_frame(sensor_idx);
    }
}

static void process_status_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data)
    {
        return;
    }

    // Status data is 4 bytes: [status, 00, 00, 00]
    uint8_t status = data[0];

    debug_send("CAN: S%d status = %d (0x%02X %02X %02X %02X)", sensor_idx, status, data[0], data[1], data[2], data[3]);

    // Store status for system health monitoring
    raw_radar_system.sensors[sensor_idx].system_status = status;
}

static void process_version_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data)
    {
        return;
    }

    // Version data is 3 bytes: [major, minor, patch]
    uint8_t major = data[0];
    uint8_t minor = data[1];
    uint8_t patch = data[2];

    debug_send("CAN: S%d version = %d.%d.%d", sensor_idx, major, minor, patch);

    // Store version information
    raw_radar_system.sensors[sensor_idx].fw_version_major = major;
    raw_radar_system.sensors[sensor_idx].fw_version_minor = minor;
    raw_radar_system.sensors[sensor_idx].fw_version_patch = patch;
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

    // debug_send("CAN: S%d frame complete - %d points", sensor_idx, sensor->num_points);

// Notify radar layer when it exists
#ifdef MTI_RADAR_H
    radar_notify_new_raw_data(sensor_idx);
#endif
}

/*------------------------------------------------------------------------------
 * Data Access Functions (unchanged)
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
    debug_send("=== CAN System Diagnostics (Extended IDs) ===");
    debug_send("Command IDs: 0x%08X, 0x%08X, 0x%08X", 0x00000060, 0x00000070, 0x00000080);
    debug_send("System initialized: %s", raw_radar_system.system_initialized ? "YES" : "NO");
    debug_send("Active sensors: %d/%d", can_get_online_sensor_count(), MAX_RADAR_SENSORS);
    debug_send("");

    uint32_t current_time = HAL_GetTick();

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint32_t last_msg_age = current_time - raw_radar_system.last_message_time[i];
        bool     online       = can_is_sensor_online(i);
        uint32_t cmd_id       = 0x00000060 + (i * 0x10);
        uint32_t header_id    = 0x000000A0 + (i * 0x10);

        debug_send("Sensor %d: %s (%d msgs, %dms ago)", i, online ? "ONLINE" : "OFFLINE", raw_radar_system.msgs_received[i], last_msg_age);
        debug_send("  Command ID: 0x%08X, Data Header ID: 0x%08X", cmd_id, header_id);
    }

    debug_send("===============================");
}

uint8_t get_sensor_index_from_can_id(uint32_t can_id)
{
    // Command messages: 0x00000060, 0x00000070, 0x00000080
    if (can_id >= 0x00000060 && can_id <= 0x00000080)
    {
        if ((can_id - 0x00000060) % 0x10 == 0)
        {
            uint8_t sensor_idx = (can_id - 0x00000060) / 0x10;
            if (sensor_idx < MAX_RADAR_SENSORS)
            {
                return sensor_idx;
            }
        }
    }

    // Data messages: 0x000000A0-0x000000C4
    if (can_id >= 0x000000A0 && can_id <= 0x000000C4)
    {
        uint8_t sensor_idx = (can_id - 0x000000A0) / 0x10;
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
    debug_send("=== Testing Sensor Indexing (Extended IDs) ===");

    struct
    {
        uint32_t    can_id;
        uint8_t     expected;
        const char *desc;
    } tests[] = { // Command messages
                  { 0x00000060, 0, "S0 Command" },
                  { 0x00000070, 1, "S1 Command" },
                  { 0x00000080, 2, "S2 Command" },

                  // Data Header messages
                  { 0x000000A0, 0, "S0 Header" },
                  { 0x000000B0, 1, "S1 Header" },
                  { 0x000000C0, 2, "S2 Header" },

                  // Other data messages
                  { 0x000000A1, 0, "S0 Object" },
                  { 0x000000B1, 1, "S1 Object" },
                  { 0x000000C1, 2, "S2 Object" },

                  { 0x000000A3, 0, "S0 Status" },
                  { 0x000000B3, 1, "S1 Status" },
                  { 0x000000C3, 2, "S2 Status" },

                  { 0x000000A4, 0, "S0 Version" },
                  { 0x000000B4, 1, "S1 Version" },
                  { 0x000000C4, 2, "S2 Version" },

                  // Invalid cases
                  { 0x00000050, 0xFF, "Invalid Low" },
                  { 0x000000E0, 0xFF, "Invalid High" },
                  { 0x00000061, 0xFF, "Invalid Command" },
                  { 0x000000A5, 0xFF, "Invalid Data" }
    };

    uint8_t passed = 0;
    uint8_t total  = sizeof(tests) / sizeof(tests[0]);

    for (uint8_t i = 0; i < total; i++)
    {
        uint8_t result = get_sensor_index_from_can_id(tests[i].can_id);
        if (result == tests[i].expected)
        {
            passed++;
            debug_send("PASS: %s (0x%08X) -> S%d", tests[i].desc, tests[i].can_id, result);
        }
        else
        {
            debug_send("FAIL: %s (0x%08X) -> S%d (expected S%d)", tests[i].desc, tests[i].can_id, result, tests[i].expected);
        }
    }

    debug_send("Indexing Tests: %d/%d passed", passed, total);

    if (passed == total)
    {
        debug_send("ALL TESTS PASSED - Extended ID addressing scheme is correct!");
    }
    else
    {
        debug_send("SOME TESTS FAILED - Check extended ID addressing scheme!");
    }
}

void test_sensor_responses(void)
{
    debug_send("=== Testing Sensor Responses (Extended IDs) ===");

    // Send status query to all sensors
    debug_send("Sending status queries to all sensors...");
    can_send_command_to_all_sensors(CAN_CMD_PAYLOAD_QUERY_STATUS);

    HAL_Delay(200); // Wait for responses

    // Check results
    uint8_t responding = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        bool online = can_is_sensor_online(i);
        debug_send("Sensor %d (Header ID 0x%08X): %s", i, 0x000000A0 + (i * 0x10), online ? "RESPONDING" : "NO RESPONSE");
        if (online)
        {
            responding++;
        }
    }

    debug_send("Response Test: %d/%d sensors responding", responding, MAX_RADAR_SENSORS);
}

/*------------------------------------------------------------------------------
 * Compatibility Functions (for existing code)
 *----------------------------------------------------------------------------*/

uint8_t get_active_sensor_count(void)
{
    // Alias for existing function name used in mti_system.c
    return can_get_online_sensor_count();
}

/**
 * @brief Initialize debug timing system
 */
bool can_init_debug_timers(void)
{
    last_debug_print_time     = HAL_GetTick();
    last_version_request_time = HAL_GetTick();
    debug_timers_initialized  = true;
    return true;
}

/**
 * @brief Print sensor status every 5 seconds (non-blocking)
 * Call this in your main loop
 */
void can_periodic_sensor_status_debug(void)
{
    static uint32_t status_counter = 0; // Counter for status messages

    if (!debug_timers_initialized)
    {
        can_init_debug_timers();
    }

    uint32_t current_time = HAL_GetTick();

    // Print status every 10 seconds (10000ms)
    if ((current_time - last_debug_print_time) >= 10000) // 10 seconds
    {
        last_debug_print_time = current_time;
        status_counter++; // Increment counter each time message is sent

        uint8_t online_count = 0;

        // Count online sensors
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS_ACTIVE; i++)
        {
            if (can_is_sensor_online(i))
            {
                online_count++;
            }
        }

        // Single debug print - minimal data using debug_send with counter
        debug_send("CAN Status [%d]: S0:%s S1:%s S2:%s Total:%d/3",
                   status_counter,
                   can_is_sensor_online(0) ? "ON" : "OFF",
                   can_is_sensor_online(1) ? "ON" : "OFF",
                   can_is_sensor_online(2) ? "ON" : "OFF",
                   online_count);
    }
}

/**
 * @brief Send version request to all sensors every 10 seconds
 * Call this in your main loop for stress testing
 */
void can_periodic_version_request(void)
{
    if (!debug_timers_initialized)
    {
        can_init_debug_timers();
    }

    uint32_t current_time = HAL_GetTick();

    // Send version request every 10 seconds (10000ms)
    if ((current_time - last_version_request_time) >= 10000)
    {
        last_version_request_time = current_time;

        // Send version query command to all sensors
        bool success = can_send_command_to_all_sensors(CAN_CMD_PAYLOAD_QUERY_STATUS);

        if (success)
        {
            debug_send("Version request sent to all sensors");
        }
        else
        {
            debug_send("ERROR: Version request failed");
        }
    }
}
