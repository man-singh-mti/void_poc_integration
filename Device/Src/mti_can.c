/**
 * @file mti_can.c
 * @brief CAN bus communication implementation for radar sensors
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

static void        process_header_message(uint8_t sensor_idx, uint8_t *data);
static void        process_object_message(uint8_t sensor_idx, uint8_t *data);
static void        process_status_message(uint8_t sensor_idx, uint8_t *data);
static void        process_version_message(uint8_t sensor_idx, uint8_t *data);
static void        update_sensor_online_status(uint8_t sensor_idx);
static const char *get_status_string(uint8_t status);
static bool        validate_sensor_message(uint8_t sensor_idx, uint8_t *data, uint8_t length);

/*------------------------------------------------------------------------------
 * CAN System Initialization
 *----------------------------------------------------------------------------*/

bool can_setup(void)
{
    debug_send("CAN: Setting up hardware");

    // Use the same filter configuration as working code
    const uint32_t FILTER_ID   = ((0x00000085UL << 3) | 0x4UL);
    const uint32_t FILTER_MASK = 0UL;

    canfil.FilterBank           = 0;
    canfil.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh         = (uint16_t)(FILTER_ID >> 16);
    canfil.FilterIdLow          = (uint16_t)(FILTER_ID & 0xFFFFUL);
    canfil.FilterMaskIdHigh     = (uint16_t)(FILTER_MASK >> 16);
    canfil.FilterMaskIdLow      = (uint16_t)(FILTER_MASK & 0xFFFFUL);
    canfil.FilterScale          = CAN_FILTERSCALE_32BIT;
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

    // Step 1: Send START command to each sensor individually
    debug_send("CAN: Starting all sensors...");

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint32_t sensor_cmd_id = CAN_COMMAND_BASE_ID + (i * 0x10); // 0x60, 0x70, 0x80
        debug_send("CAN: Starting sensor %d", i);

        if (!can_send(sensor_cmd_id, CAN_START))
        {
            debug_send("CAN: Failed to start sensor %d", i);
        }
        HAL_Delay(100); // Increased delay for startup
    }

    // Wait for sensors to start up
    HAL_Delay(500); // Increased wait time

    // Step 2: Query version from each sensor
    debug_send("CAN: Querying sensor versions...");

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint32_t sensor_cmd_id = CAN_COMMAND_BASE_ID + (i * 0x10);
        debug_send("CAN: Querying sensor %d version", i);

        if (!can_send(sensor_cmd_id, CAN_VERSION_QUERY))
        {
            debug_send("CAN: Failed to query sensor %d version", i);
        }
        HAL_Delay(50); // Small delay between commands
    }

    // Wait for version responses
    HAL_Delay(300);

    // Step 3: Query status from each sensor
    debug_send("CAN: Querying sensor status...");

    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint32_t sensor_cmd_id = CAN_COMMAND_BASE_ID + (i * 0x10);
        debug_send("CAN: Querying sensor %d status", i);

        if (!can_send(sensor_cmd_id, CAN_STATUS))
        {
            debug_send("CAN: Failed to query sensor %d status", i);
        }
        HAL_Delay(50); // Small delay between commands
    }

    // Wait for status responses
    HAL_Delay(300);

    // Step 4: Report initialization results
    uint8_t responding_sensors = can_get_online_sensor_count();

    raw_radar_system.active_sensor_count = responding_sensors;
    raw_radar_system.system_initialized  = (responding_sensors >= 1);

    debug_send("CAN: System init complete - %d/%d sensors online", responding_sensors, MAX_RADAR_SENSORS);

    // Show detailed sensor information including versions
    debug_send("CAN: === SENSOR INITIALIZATION REPORT ===");
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        bool online = can_is_sensor_online(i);
        debug_send("CAN: Sensor %d (CMD:0x%02X, DATA:0x%02X): %s",
                   i,
                   CAN_COMMAND_BASE_ID + (i * 0x10),
                   CAN_DATA_HEADER_BASE_ID + (i * 0x10),
                   online ? "ONLINE" : "OFFLINE");

        if (online)
        {
            // Report version if available
            if (raw_radar_system.sensor_version_major[i] != 0 || raw_radar_system.sensor_version_minor[i] != 0 || raw_radar_system.sensor_version_patch[i] != 0)
            {
                debug_send("  Version: v%02d.%02d.%02d",
                           raw_radar_system.sensor_version_major[i],
                           raw_radar_system.sensor_version_minor[i],
                           raw_radar_system.sensor_version_patch[i]);
            }
            else
            {
                debug_send("  Version: NOT REPORTED");
            }

            debug_send("  Status: %d, Messages: %d", raw_radar_system.sensor_status[i], raw_radar_system.msgs_received[i]);
        }
    }
    debug_send("CAN: === END SENSOR REPORT ===");

    return raw_radar_system.system_initialized;
}

/*------------------------------------------------------------------------------
 * CAN Communication Functions
 *----------------------------------------------------------------------------*/

bool can_send(uint32_t ID, uint8_t message)
{
    txHeader.DLC                = 1;
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID; // CRITICAL: Set BOTH like working code
    txHeader.ExtId              = ID; // CRITICAL: Set BOTH like working code
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

    txHeader.DLC                = length;
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID; // CRITICAL: Set BOTH like working code
    txHeader.ExtId              = ID; // CRITICAL: Set BOTH like working code
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

    uint32_t can_id = rxHeader.ExtId;

    // Debug output first (like working code)
    char data_hex[17];
    memset(data_hex, 0, 17);
    for (int i = 0; i < rxHeader.DLC; i++)
    {
        sprintf(data_hex, "%s%02X", data_hex, canRX[i]);
    }
    debug_send("CAN RX: (0x%02X) 0x%s", can_id, data_hex);

    // Map CAN ID to sensor index using confirmed mapping
    uint8_t sensor_idx = 0xFF; // Invalid by default

    // Data messages: 0xA0-0xAF=S0, 0xB0-0xBF=S1, 0xC0-0xCF=S2, 0xD0-0xDF=S3
    if (can_id >= 0xA0 && can_id <= 0xAF)
    {
        sensor_idx = 0; // Sensor 0
    }
    else if (can_id >= 0xB0 && can_id <= 0xBF)
    {
        sensor_idx = 1; // Sensor 1
    }
    else if (can_id >= 0xC0 && can_id <= 0xCF)
    {
        sensor_idx = 2; // Sensor 2
    }
    else if (can_id >= 0xD0 && can_id <= 0xDF)
    {
        sensor_idx = 3; // Sensor 3 (future use)
    }

    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        debug_send("CAN: Unknown/unsupported sensor for ID 0x%02X", can_id);
        return;
    }

    // Update sensor online status
    raw_radar_system.last_message_time[sensor_idx] = HAL_GetTick();
    raw_radar_system.msgs_received[sensor_idx]++;

    debug_send("CAN RX: S%d (0x%02X) 0x%s", sensor_idx, can_id, data_hex);

    // Process based on message type (using lower 4 bits of CAN ID)
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
 * Message Processing Functions (Fixed for consistency)
 *----------------------------------------------------------------------------*/

static void process_header_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    // Header message indicates start of new frame
    uint32_t frame_number = 0;

    // Extract frame info (assuming little-endian format)
    if (data[0] != 0x1A && data[0] != 0x1B) // Check for valid header
    {
        uint32_t total_length = 0;
        memcpy(&total_length, &data[0], 4);
        if (data[4] != 0) // Frame number might be in bytes 4-7
        {
            memcpy(&frame_number, &data[4], 4);
        }
    }
    else
    {
        // Alternative format - adjust based on your data
        frame_number = data[0]; // First byte might be frame number
    }

    debug_send("CAN: S%d Header - Frame=%lu", sensor_idx, frame_number);

    // Use consistent sensor array access
    radar_raw_t *sensor_data  = &raw_radar_system.sensors[sensor_idx];
    sensor_data->num_points   = 0;
    sensor_data->frame_number = frame_number;
    sensor_data->new_frame    = true;
}

static void process_object_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    // Extract distance and SNR from 8-byte object data
    // Bytes 0-3: Distance (float, little-endian)
    // Bytes 4-7: SNR (float, little-endian)
    float distance_m, snr;
    memcpy(&distance_m, &data[0], sizeof(float));
    memcpy(&snr, &data[4], sizeof(float));

    debug_send("CAN: S%d Object - Distance=%.3fm, SNR=%.1f", sensor_idx, distance_m, snr);

    // Use consistent sensor array access
    radar_raw_t *sensor_data = &raw_radar_system.sensors[sensor_idx];

    if (sensor_data->num_points < MAX_RADAR_DETECTED_POINTS)
    {
        sensor_data->detected_points[sensor_data->num_points][0] = distance_m;
        sensor_data->detected_points[sensor_data->num_points][1] = snr;
        sensor_data->num_points++;

        // Update metadata
        sensor_data->timestamp_ms       = HAL_GetTick();
        sensor_data->new_data_available = true;
    }
}

static void process_status_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    uint8_t status = data[0];

    // Store the status
    raw_radar_system.sensor_status[sensor_idx]    = status;
    raw_radar_system.last_status_time[sensor_idx] = HAL_GetTick(); // Status meanings based on constants:
    const char *status_str                        = "UNKNOWN";
    switch (status)
    {
    case CAN_STATUS_BOOT:
        status_str = "BOOT";
        break;
    case CAN_STATUS_CHIRP:
        status_str = "CHIRP";
        break;
    case CAN_STATUS_STOPPED:
        status_str = "STOPPED";
        break;
    default:
        status_str = "UNKNOWN";
        break;
    }

    debug_send("CAN: S%d status: %s (0x%02X)", sensor_idx, status_str, status);

    // If sensor is chirping, it's considered online and healthy
    if (status == CAN_STATUS_CHIRP) // CHIRP
    {
        raw_radar_system.last_message_time[sensor_idx] = HAL_GetTick();
    }
}

static void process_version_message(uint8_t sensor_idx, uint8_t *data)
{
    if (sensor_idx >= MAX_RADAR_SENSORS || !data)
    {
        return;
    }

    // Extract semantic version from 3-byte format: AABBCC = vAA.BB.CC
    uint8_t major = data[0];
    uint8_t minor = data[1];
    uint8_t patch = data[2];

    // Store version information
    raw_radar_system.sensor_version_major[sensor_idx]    = major;
    raw_radar_system.sensor_version_minor[sensor_idx]    = minor;
    raw_radar_system.sensor_version_patch[sensor_idx]    = patch;
    raw_radar_system.sensor_version_received[sensor_idx] = true;

    debug_send("CAN: S%d version = v%02d.%02d.%02d", sensor_idx, major, minor, patch);

    // Optional: Check for compatible versions
    const uint8_t MIN_MAJOR = 1;
    const uint8_t MIN_MINOR = 3;
    const uint8_t MIN_PATCH = 0;

    bool version_compatible = (major > MIN_MAJOR) || (major == MIN_MAJOR && minor > MIN_MINOR) || (major == MIN_MAJOR && minor == MIN_MINOR && patch >= MIN_PATCH);

    if (version_compatible)
    {
        debug_send("CAN: S%d version COMPATIBLE", sensor_idx);
    }
    else
    {
        debug_send("CAN: S%d version WARNING - Minimum required: v%02d.%02d.%02d", sensor_idx, MIN_MAJOR, MIN_MINOR, MIN_PATCH);
    }

// Set version in system module if function exists
#ifdef MTI_SYSTEM_H
// version_set(MODULE_RADAR_S0 + sensor_idx, major, minor, patch);
#endif
}

void process_complete_radar_frame(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    // Use consistent sensor array access
    radar_raw_t *sensor_data = &raw_radar_system.sensors[sensor_idx];

    if (sensor_data->num_points > 0)
    {
        debug_send("CAN: S%d Frame complete - %d points detected", sensor_idx, sensor_data->num_points);

        // Mark frame as complete for radar processing layer
        sensor_data->frame_complete = true;

        // Notify radar layer that new data is available
        radar_notify_new_raw_data(sensor_idx);
    }
}

/*------------------------------------------------------------------------------
 * Data Access Functions (Cleaned up)
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
        if ((current_time - raw_radar_system.last_message_time[i]) < CAN_SENSOR_OFFLINE_TIMEOUT_MS)
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
    } // A sensor is online if:
    // 1. We've received messages from it
    // 2. Recent activity (within timeout)
    // 3. Status is CHIRP - actively working

    uint32_t current_time    = HAL_GetTick();
    bool     has_messages    = (raw_radar_system.msgs_received[sensor_idx] > 0);
    bool     recent_activity = (current_time - raw_radar_system.last_message_time[sensor_idx] < SENSOR_TIMEOUT_MS);
    bool     is_chirping     = (raw_radar_system.sensor_status[sensor_idx] == CAN_STATUS_CHIRP);

    return has_messages && recent_activity && is_chirping;
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

        // Add version information
        if (raw_radar_system.sensor_version_received[i])
        {
            debug_send("  Version: v%02d.%02d.%02d",
                       raw_radar_system.sensor_version_major[i],
                       raw_radar_system.sensor_version_minor[i],
                       raw_radar_system.sensor_version_patch[i]);
        }
        else
        {
            debug_send("  Version: NOT RECEIVED");
        }
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

    return 0xFF; // Invalid sensor index
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

    debug_send("Command Base: 0x60, Data Base: 0xA0");
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

void test_basic_can_communication(void)
{
    debug_send("=== Testing Basic CAN Communication ===");

    // Clear counters
    memset(&raw_radar_system, 0, sizeof(raw_radar_system));

    debug_send("Step 1: Testing CAN transmission to all sensors...");

    // Test transmission to each sensor individually
    bool all_tx_success = true;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint32_t sensor_cmd_id = CAN_COMMAND_BASE_ID + (i * 0x10); // 0x60, 0x70, 0x80
        bool     tx_result     = can_send(sensor_cmd_id, CAN_STATUS);
        debug_send("TX Test S%d (0x%02X): %s", i, sensor_cmd_id, tx_result ? "PASS" : "FAIL");

        if (!tx_result)
        {
            all_tx_success = false;
        }
        HAL_Delay(50); // Small delay between commands
    }

    if (!all_tx_success)
    {
        debug_send("CAN transmission failed - check hardware");
        return;
    }

    debug_send("Step 2: Waiting for responses...");

    // Wait and check for any CAN messages
    HAL_Delay(500);

    uint32_t total_messages = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        total_messages += raw_radar_system.msgs_received[i];
    }

    debug_send("Step 3: Results");
    debug_send("  Total messages received: %d", total_messages);

    // Show per-sensor results
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint32_t cmd_id  = CAN_COMMAND_BASE_ID + (i * 0x10);
        uint32_t data_id = CAN_DATA_HEADER_BASE_ID + (i * 0x10);
        bool     online  = can_is_sensor_online(i);

        debug_send("  S%d: CMD=0x%02X, DATA=0x%02X, %s (%d msgs)", i, cmd_id, data_id, online ? "RESPONDING" : "NO RESPONSE", raw_radar_system.msgs_received[i]);
    }

    uint8_t online_count = can_get_online_sensor_count();
    debug_send("  Sensors online: %d/%d", online_count, MAX_RADAR_SENSORS);

    if (total_messages > 0)
    {
        debug_send("SUCCESS: CAN communication working");
    }
    else
    {
        debug_send("FAIL: No responses received");
        debug_send("Check:");
        debug_send("  - CAN bus wiring");
        debug_send("  - Sensor power");
        debug_send("  - CAN bus termination");
        debug_send("  - Baud rate (probably 500kbps)");
        debug_send("  - Each sensor should respond to its specific CMD ID");
    }

    debug_send("=========================================");
}

void test_sensor_start_stop_sequence(void)
{
    debug_send("=== Testing Sensor Start/Stop Sequence ===");

    // Clear counters
    memset(&raw_radar_system, 0, sizeof(raw_radar_system));

    // Step 1: Send START to all sensors
    debug_send("Step 1: Starting all sensors...");
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint32_t sensor_cmd_id = CAN_COMMAND_BASE_ID + (i * 0x10);
        bool     result        = can_send(sensor_cmd_id, CAN_START);
        debug_send("  S%d START (0x%02X): %s", i, sensor_cmd_id, result ? "SENT" : "FAILED");
        HAL_Delay(50);
    }

    HAL_Delay(300); // Wait for startup

    // Step 2: Query status from all sensors
    debug_send("Step 2: Querying all sensor status...");
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        uint32_t sensor_cmd_id = CAN_COMMAND_BASE_ID + (i * 0x10);
        bool     result        = can_send(sensor_cmd_id, CAN_STATUS);
        debug_send("  S%d STATUS (0x%02X): %s", i, sensor_cmd_id, result ? "SENT" : "FAILED");
        HAL_Delay(50);
    }

    HAL_Delay(300); // Wait for responses

    // Step 3: Check results
    debug_send("Step 3: Results after START+STATUS sequence");
    uint8_t responding = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        bool online = can_is_sensor_online(i);
        if (online)
        {
            responding++;
        }
        debug_send("  S%d: %s (%d messages received)", i, online ? "RESPONDING" : "NO RESPONSE", raw_radar_system.msgs_received[i]);
    }

    debug_send("Total responding sensors: %d/%d", responding, MAX_RADAR_SENSORS);

    // Step 4: Optional STOP test
    if (responding > 0)
    {
        debug_send("Step 4: Testing STOP command...");
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            if (can_is_sensor_online(i))
            {
                uint32_t sensor_cmd_id = CAN_COMMAND_BASE_ID + (i * 0x10);
                bool     result        = can_send(sensor_cmd_id, CAN_STOP);
                debug_send("  S%d STOP (0x%02X): %s", i, sensor_cmd_id, result ? "SENT" : "FAILED");
                HAL_Delay(50);
            }
        }
    }

    debug_send("=======================================");
}

/*------------------------------------------------------------------------------
 * Compatibility Functions
 *----------------------------------------------------------------------------*/

uint8_t get_active_sensor_count(void)
{
    // Alias for existing function name used in mti_system.c
    return can_get_online_sensor_count();
}

void test_complete_can_system(void)
{
    debug_send("========== COMPLETE CAN SYSTEM TEST ==========");

    // Test 1: Basic hardware
    test_basic_can_communication();
    HAL_Delay(1000);

    // Test 2: Full sequence
    test_sensor_start_stop_sequence();
    HAL_Delay(1000);

    // Test 3: Final system check
    can_run_diagnostics();
    debug_send("============= TEST COMPLETE =============");
}

/*------------------------------------------------------------------------------
 * Helper Functions
 *----------------------------------------------------------------------------*/

static void update_sensor_online_status(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    raw_radar_system.last_message_time[sensor_idx] = HAL_GetTick();
    raw_radar_system.msgs_received[sensor_idx]++;
}

static const char *get_status_string(uint8_t status)
{
    switch (status)
    {
    case CAN_STATUS_BOOT:
        return "BOOT";
    case CAN_STATUS_CHIRP:
        return "CHIRP";
    case CAN_STATUS_STOPPED:
        return "STOPPED";
    default:
        return "UNKNOWN";
    }
}

static bool validate_sensor_message(uint8_t sensor_idx, uint8_t *data, uint8_t length)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }

    if (!data || length == 0 || length > 8)
    {
        return false;
    }

    return true;
}
