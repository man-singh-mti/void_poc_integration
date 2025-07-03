/**
 * @file mti_can.c
 * @brief Clean CAN middleware implementation - Event-driven data collection
 * @author MTi Group
 * @copyright 2025 MTi Group
 */

#include "mti_can.h"
#include "can.h"
#include <string.h>

/*------------------------------------------------------------------------------
 * Private Variables
 *----------------------------------------------------------------------------*/

static can_system_t        can_system = { 0 };
static CAN_RxHeaderTypeDef rx_header;
static CAN_TxHeaderTypeDef tx_header;
static uint8_t             rx_data[8];
static uint32_t            tx_mailbox;

/** @brief Event flags for immediate processing */
volatile bool     can_radar_data_ready                      = false;
volatile bool     can_sensor_data_ready[MAX_RADAR_SENSORS]  = { false, false, false };
volatile bool     can_new_object_data                       = false;
volatile uint32_t can_sensor_frame_count[MAX_RADAR_SENSORS] = { 0, 0, 0 };
static uint32_t   frame_completion_count                    = 0;

/*------------------------------------------------------------------------------
 * Private Functions
 *----------------------------------------------------------------------------*/

static bool setup_can_hardware(void)
{
    CAN_FilterTypeDef filter = { .FilterBank           = 0,
                                 .FilterMode           = CAN_FILTERMODE_IDMASK,
                                 .FilterScale          = CAN_FILTERSCALE_32BIT,
                                 .FilterIdHigh         = 0x0000,
                                 .FilterIdLow          = 0x0000,
                                 .FilterMaskIdHigh     = 0x0000,
                                 .FilterMaskIdLow      = 0x0000,
                                 .FilterFIFOAssignment = CAN_RX_FIFO0,
                                 .FilterActivation     = ENABLE };

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        debug_send("CAN:init, Filter config failed");
        return false;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        debug_send("CAN:init, Start failed");
        return false;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        debug_send("CAN:init, Interrupt activation failed");
        return false;
    }

    debug_send("CAN:init, Hardware ready");
    return true;
}

static bool can_transmit(uint32_t can_id, uint8_t *data, uint8_t length)
{
    // Wait for free mailbox with timeout
    uint32_t start_time = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        if ((HAL_GetTick() - start_time) > 100)
        { // 100ms timeout
            debug_send("CAN:tx, Timeout - no free mailbox");
            return false;
        }
        HAL_Delay(1);
    }

    // Setup transmission
    tx_header.DLC                = length;
    tx_header.IDE                = CAN_ID_EXT;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.StdId              = 0;
    tx_header.ExtId              = can_id;
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);

    if (status == HAL_OK)
    {
        // Remove verbose success message
        // debug_send("[C]TX %lX [%02X]", can_id, data[0]);
        return true;
    }
    else
    {
        debug_send("CAN:tx, Failed - status %d", status);
        return false;
    }
}

/*------------------------------------------------------------------------------
 * Event-Driven CAN Callback - Just Copy Data to Structs
 *----------------------------------------------------------------------------*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        debug_send("CAN:rx, Message read failed");
        return;
    }

    uint32_t timestamp = HAL_GetTick();
    uint32_t can_id    = rx_header.ExtId;

    switch (can_id)
    {
    // Sensor 0 messages
    case CAN_S0_HEADER_ID:
        memcpy(&can_system.sensors[0].frame_number, &rx_data[4], 4);
        can_system.sensors[0].num_points          = rx_data[0] - 0x1A + 1;
        can_system.sensors[0].current_point_count = 0;
        can_system.sensors[0].last_msg_time       = timestamp;
        can_system.sensors[0].msg_count++;
        can_system.sensors[0].online = true;
        can_system.sensors[0].status = RADAR_CHIRPING;
        break;

    case CAN_S0_OBJECT_ID:
        if (can_system.sensors[0].current_point_count < MAX_RADAR_DETECTED_POINTS)
        {
            uint8_t point_idx = can_system.sensors[0].current_point_count;

            float range, snr;
            memcpy(&range, &rx_data[0], 4);
            memcpy(&snr, &rx_data[4], 4);

            can_system.sensors[0].detection_points[point_idx][0] = range;
            can_system.sensors[0].detection_points[point_idx][1] = snr;

            can_system.sensors[0].current_point_count++;

            // NEW: Set per-sensor flags
            can_sensor_data_ready[0] = true;
            can_radar_data_ready     = true;
            can_new_object_data      = true;
            can_sensor_frame_count[0]++;
            frame_completion_count++;
        }
        can_system.sensors[0].last_msg_time = timestamp;
        can_system.sensors[0].msg_count++;
        break;

    case CAN_S0_STATUS_ID:
        can_system.sensors[0].status_code   = rx_data[0];
        can_system.sensors[0].last_msg_time = timestamp;
        can_system.sensors[0].msg_count++;
        break;

    case CAN_S0_VERSION_ID:
        memcpy(can_system.sensors[0].fw_version, rx_data, 3);
        can_system.sensors[0].last_msg_time = timestamp;
        can_system.sensors[0].msg_count++;
        break;

    // Sensor 1 messages
    case CAN_S1_HEADER_ID:
        memcpy(&can_system.sensors[1].frame_number, &rx_data[4], 4);
        can_system.sensors[1].num_points          = rx_data[0] - 0x1A + 1;
        can_system.sensors[1].current_point_count = 0; // Add this line
        can_system.sensors[1].last_msg_time       = timestamp;
        can_system.sensors[1].msg_count++;
        can_system.sensors[1].online = true;
        can_system.sensors[1].status = RADAR_CHIRPING;
        break;

    case CAN_S1_OBJECT_ID:
        if (can_system.sensors[1].current_point_count < MAX_RADAR_DETECTED_POINTS)
        {
            uint8_t point_idx = can_system.sensors[1].current_point_count;

            float range, snr;
            memcpy(&range, &rx_data[0], 4);
            memcpy(&snr, &rx_data[4], 4);

            can_system.sensors[1].detection_points[point_idx][0] = range;
            can_system.sensors[1].detection_points[point_idx][1] = snr;

            can_system.sensors[1].current_point_count++;

            // NEW: Set per-sensor flags
            can_sensor_data_ready[1] = true;
            can_radar_data_ready     = true;
            can_new_object_data      = true;
            can_sensor_frame_count[1]++;
            frame_completion_count++;
        }
        can_system.sensors[1].last_msg_time = timestamp;
        can_system.sensors[1].msg_count++;
        break;

    case CAN_S1_STATUS_ID:
        can_system.sensors[1].status_code   = rx_data[0];
        can_system.sensors[1].last_msg_time = timestamp;
        can_system.sensors[1].msg_count++;
        break;

    case CAN_S1_VERSION_ID:
        memcpy(can_system.sensors[1].fw_version, rx_data, 3);
        can_system.sensors[1].last_msg_time = timestamp;
        can_system.sensors[1].msg_count++;
        break;

    // Sensor 2 messages
    case CAN_S2_HEADER_ID:
        memcpy(&can_system.sensors[2].frame_number, &rx_data[4], 4);
        can_system.sensors[2].num_points          = rx_data[0] - 0x1A + 1;
        can_system.sensors[2].current_point_count = 0; // Add this line
        can_system.sensors[2].last_msg_time       = timestamp;
        can_system.sensors[2].msg_count++;
        can_system.sensors[2].online = true;
        can_system.sensors[2].status = RADAR_CHIRPING;
        break;

    case CAN_S2_OBJECT_ID:
        if (can_system.sensors[2].current_point_count < MAX_RADAR_DETECTED_POINTS)
        {
            uint8_t point_idx = can_system.sensors[2].current_point_count;

            float range, snr;
            memcpy(&range, &rx_data[0], 4);
            memcpy(&snr, &rx_data[4], 4);

            can_system.sensors[2].detection_points[point_idx][0] = range;
            can_system.sensors[2].detection_points[point_idx][1] = snr;

            can_system.sensors[2].current_point_count++;

            // NEW: Set per-sensor flags
            can_sensor_data_ready[2] = true;
            can_radar_data_ready     = true;
            can_new_object_data      = true;
            can_sensor_frame_count[2]++;
            frame_completion_count++;
        }
        can_system.sensors[2].last_msg_time = timestamp;
        can_system.sensors[2].msg_count++;
        break;

    case CAN_S2_STATUS_ID:
        can_system.sensors[2].status_code   = rx_data[0];
        can_system.sensors[2].last_msg_time = timestamp;
        can_system.sensors[2].msg_count++;
        break;

    case CAN_S2_VERSION_ID:
        memcpy(can_system.sensors[2].fw_version, rx_data, 3);
        can_system.sensors[2].last_msg_time = timestamp;
        can_system.sensors[2].msg_count++;
        break;

    default:
        debug_send("CAN:rx, Unknown ID 0x%08lX", can_id);
        break;
    }
}

/*------------------------------------------------------------------------------
 * Public API Implementation
 *----------------------------------------------------------------------------*/

bool can_init(void)
{
    debug_send("CAN:init, Starting");

    memset(&can_system, 0, sizeof(can_system));
    can_system.init_start_time = HAL_GetTick();
    can_system.system_status   = RADAR_INITIALISING;

    for (uint8_t i = 0; i < 3; i++)
    {
        can_system.sensors[i].status = RADAR_INITIALISING;
    }

    debug_send("CAN:init, Setting up hardware");
    if (!setup_can_hardware())
    {
        debug_send("CAN:init, Hardware setup failed");
        return false;
    }

    debug_send("CAN:init, Waiting 5s for sensor boot");
    HAL_Delay(5000);

    debug_send("CAN:init, Sending START to all sensors");
    can_send_to_all_sensors(CAN_CMD_START);
    HAL_Delay(200);

    debug_send("CAN:init, Sending VERSION query to all sensors");
    can_send_to_all_sensors(CAN_CMD_QUERY_STATUS);

    debug_send("CAN:init, Waiting 1s for sensor responses");
    HAL_Delay(1000);

    debug_send("CAN:init, Checking sensor responses");
    for (uint8_t i = 0; i < 3; i++)
    {
        can_sensor_t *sensor = &can_system.sensors[i];
        if (sensor->online)
        {
            sensor->status = RADAR_READY;
            debug_send("CAN:init, S%d ONLINE, FW v%d.%d.%d, Status 0x%02X, Msgs %lu",
                       i,
                       sensor->fw_version[0],
                       sensor->fw_version[1],
                       sensor->fw_version[2],
                       sensor->status_code,
                       sensor->msg_count);
        }
        else
        {
            sensor->status = RADAR_STOPPED;
            debug_send("CAN:init, S%d OFFLINE", i);
        }
    }

    can_system.online_count = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        if (can_system.sensors[i].online)
        {
            can_system.online_count++;
        }
    }

    debug_send("CAN:init, Active sensors: %d/3", can_system.online_count);

    debug_send("CAN:init, Turning OFF all sensors after init");
    can_send_to_all_sensors(CAN_CMD_STOP);

    for (uint8_t i = 0; i < 3; i++)
    {
        if (can_system.sensors[i].online)
        {
            can_system.sensors[i].status = RADAR_STOPPED;
        }
    }

    can_system.system_initialized = true;
    can_system.system_status      = (can_system.online_count > 0) ? RADAR_READY : RADAR_STOPPED;

    uint32_t init_time = HAL_GetTick() - can_system.init_start_time;
    debug_send("CAN:init, Complete in %lums", init_time);
    debug_send("CAN:init, END");

    return true;
}

bool can_send_command(uint8_t sensor_id, uint8_t command)
{
    if (sensor_id >= 3)
    {
        return false;
    }
    uint32_t can_id;
    switch (sensor_id)
    {
    case 0:
        can_id = CAN_S0_CMD_ID;
        break;
    case 1:
        can_id = CAN_S1_CMD_ID;
        break;
    case 2:
        can_id = CAN_S2_CMD_ID;
        break;
    default:
        return false;
    }

    uint8_t data[1] = { command };
    return can_transmit(can_id, data, 1);
}

bool can_send_command_data(uint8_t sensor_id, uint8_t command, uint8_t *data, uint8_t len)
{
    if (sensor_id >= 3 || len > 7)
    {
        return false; // Max 7 data bytes + 1 command byte
    }
    uint32_t can_id;
    switch (sensor_id)
    {
    case 0:
        can_id = CAN_S0_CMD_ID;
        break;
    case 1:
        can_id = CAN_S1_CMD_ID;
        break;
    case 2:
        can_id = CAN_S2_CMD_ID;
        break;
    default:
        return false;
    }

    uint8_t tx_data[8];
    tx_data[0] = command;
    memcpy(&tx_data[1], data, len);

    return can_transmit(can_id, tx_data, len + 1);
}

bool can_send_to_all_sensors(uint8_t command)
{
    bool all_success = true;

    for (uint8_t i = 0; i < 3; i++)
    {
        if (!can_send_command(i, command))
        {
            all_success = false;
        }
        HAL_Delay(50); // Small delay between sensors
    }

    return all_success;
}

can_sensor_t *can_get_sensor(uint8_t sensor_id)
{
    if (sensor_id >= 3)
    {
        return NULL;
    }
    return &can_system.sensors[sensor_id];
}

can_system_t *can_get_system(void)
{
    return &can_system;
}

uint8_t can_get_online_count(void)
{
    return can_system.online_count;
}

bool can_is_system_healthy(void)
{
    // System is healthy if CAN hardware was initialized successfully
    // Sensors may be offline between commands, but the CAN system itself is functional
    return can_system.system_initialized;
}

bool can_has_sufficient_sensors(void)
{
    // Check if we have enough sensors currently online for operation
    return can_system.online_count >= 2;
}

can_status_t can_get_system_status(void)
{
    return can_system.system_status;
}

void can_process_timeouts(void)
{
    uint32_t        now                   = HAL_GetTick();
    uint8_t         previous_online_count = can_system.online_count;
    static uint32_t last_timeout_check    = 0;

    // Only check timeouts every 500ms to reduce noise
    if ((now - last_timeout_check) < 500)
    {
        return;
    }
    last_timeout_check = now;

    can_system.online_count = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        bool has_timed_out      = (now - can_system.sensors[i].last_msg_time) > 3000;
        bool was_expecting_data = (can_system.sensors[i].status == RADAR_CHIRPING);

        if (has_timed_out)
        {
            if (can_system.sensors[i].online)
            {
                can_system.sensors[i].online = false;

                // Only report as timeout if we were expecting data
                if (was_expecting_data)
                {
                    debug_send("CAN:timeout, S%d offline (was chirping)", i);
                    can_system.sensors[i].status = RADAR_STOPPED;
                }
                else
                {
                    // Sensor was stopped - this is expected silence, not a timeout
                    debug_send("CAN:quiet, S%d stopped (expected)", i);
                }
            }
        }
        else if (can_system.sensors[i].online)
        {
            can_system.online_count++;
        }
    }

    // Only report significant status changes
    if (abs(previous_online_count - can_system.online_count) >= 1)
    {
        debug_send("CAN:status, Online sensors: %d/3", can_system.online_count);
    }

    can_system.system_status = (can_system.online_count > 0) ? RADAR_READY : RADAR_STOPPED;
}

/*------------------------------------------------------------------------------
 * Status String Table
 *----------------------------------------------------------------------------*/
static const char *can_status_str[] = { "INIT", "READY", "CHIRP", "STOP" };

/*------------------------------------------------------------------------------
 * Test & Debug Functions
 *----------------------------------------------------------------------------*/

void can_test_periodic(void)
{
    static uint32_t last_test_time = 0;
    static uint32_t test_counter   = 0;

    uint32_t now = HAL_GetTick();

    // Run every 10 seconds
    if ((now - last_test_time) >= 10000)
    {
        last_test_time = now;
        test_counter++;

        debug_send("CAN:test, Periodic test #%lu", test_counter);
        debug_send("CAN:test, Test run count: %lu", test_counter);

        // 1. Send start command to all sensors
        debug_send("CAN:test, Sending START to all sensors");
        can_send_to_all_sensors(CAN_CMD_START);
        HAL_Delay(200);

        // 2. Send version command
        debug_send("CAN:test, Sending VERSION query to all sensors");
        can_send_to_all_sensors(CAN_CMD_QUERY_STATUS);
        HAL_Delay(500); // Wait for responses

        // 3. Retrieve and output status and version
        debug_send("CAN:test, Sensor Status:");
        for (uint8_t i = 0; i < 3; i++)
        {
            can_sensor_t *sensor = can_get_sensor(i);

            if (sensor->online)
            {
                debug_send("  S%d: ONLINE - %s - FW v%d.%d.%d - Status: 0x%02X - Msgs: %lu",
                           i,
                           can_status_str[sensor->status],
                           sensor->fw_version[0],
                           sensor->fw_version[1],
                           sensor->fw_version[2],
                           sensor->status_code,
                           sensor->msg_count);
            }
            else
            {
                debug_send("  S%d: OFFLINE - %s", i, can_status_str[sensor->status]);
            }
        }

        // 4. Turn off sensors
        debug_send("CAN:test, Turning OFF all sensors");
        can_send_to_all_sensors(CAN_CMD_STOP);

        debug_send("CAN:test, TEST COMPLETE");
    }
}

void can_debug_system_status(void)
{
    debug_send("CAN:init, System initialized: %s", can_system.system_initialized ? "YES" : "NO");
    debug_send("CAN:init, Status: %s, Online: %d/3, Healthy: %s",
               can_status_str[can_system.system_status],
               can_system.online_count,
               can_is_system_healthy() ? "YES" : "NO");

    for (uint8_t i = 0; i < 3; i++)
    {
        can_sensor_t *s = &can_system.sensors[i];
        debug_send("CAN:init, S%d: %s, %s, FW v%d.%d.%d, 0x%02X, Msgs: %lu",
                   i,
                   s->online ? "ON" : "OFF",
                   can_status_str[s->status],
                   s->fw_version[0],
                   s->fw_version[1],
                   s->fw_version[2],
                   s->status_code,
                   s->msg_count);
    }
}

void can_debug_sensor_indexing(void)
{
    debug_send("CAN:index, Command IDs: S0=0x%08X, S1=0x%08X, S2=0x%08X", CAN_S0_CMD_ID, CAN_S1_CMD_ID, CAN_S2_CMD_ID);
    debug_send("CAN:index, S0 Data: 0x%08X-0x%08X", CAN_S0_HEADER_ID, CAN_S0_VERSION_ID);
    debug_send("CAN:index, S1 Data: 0x%08X-0x%08X", CAN_S1_HEADER_ID, CAN_S1_VERSION_ID);
    debug_send("CAN:index, S2 Data: 0x%08X-0x%08X", CAN_S2_HEADER_ID, CAN_S2_VERSION_ID);
}

// Add flag management functions
bool can_has_new_radar_data(void)
{
    return can_radar_data_ready;
}

void can_clear_radar_data_flag(void)
{
    can_radar_data_ready = false;
    can_new_object_data  = false;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        can_sensor_data_ready[i] = false;
    }
}

uint32_t can_get_frame_completion_count(void)
{
    return frame_completion_count;
}

bool can_sensor_has_new_data(uint8_t sensor_id)
{
    if (sensor_id >= MAX_RADAR_SENSORS)
    {
        return false;
    }
    return can_sensor_data_ready[sensor_id];
}

void can_clear_sensor_data_flag(uint8_t sensor_id)
{
    if (sensor_id < MAX_RADAR_SENSORS)
    {
        can_sensor_data_ready[sensor_id] = false;
    }

    // Check if any sensors still have new data
    can_radar_data_ready = false;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (can_sensor_data_ready[i])
        {
            can_radar_data_ready = true;
            break;
        }
    }

    if (!can_radar_data_ready)
    {
        can_new_object_data = false;
    }
}

uint32_t can_get_sensor_frame_count(uint8_t sensor_id)
{
    if (sensor_id >= MAX_RADAR_SENSORS)
    {
        return 0;
    }
    return can_sensor_frame_count[sensor_id];
}

bool can_is_system_initialized(void)
{
    return can_system.system_initialized;
}
