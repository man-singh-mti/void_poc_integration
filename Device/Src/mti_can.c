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

/*------------------------------------------------------------------------------
 * Private Functions
 *----------------------------------------------------------------------------*/

static bool setup_can_hardware(void)
{
    debug_send("CAN: Setting up hardware (no filtering)");

    // Accept all extended IDs - no filtering
    CAN_FilterTypeDef filter = { .FilterBank           = 0,
                                 .FilterMode           = CAN_FILTERMODE_IDMASK,
                                 .FilterScale          = CAN_FILTERSCALE_32BIT,
                                 .FilterIdHigh         = 0x0000, // Accept all
                                 .FilterIdLow          = 0x0000,
                                 .FilterMaskIdHigh     = 0x0000, // No filtering
                                 .FilterMaskIdLow      = 0x0000,
                                 .FilterFIFOAssignment = CAN_RX_FIFO0,
                                 .FilterActivation     = ENABLE };

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        debug_send("CAN: ERROR - Filter config failed");
        return false;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        debug_send("CAN: ERROR - Start failed");
        return false;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        debug_send("CAN: ERROR - Interrupt activation failed");
        return false;
    }

    debug_send("CAN: Hardware ready");
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
            debug_send("CAN: TX timeout - no free mailbox");
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
        debug_send("CAN: TX 0x%08lX [%02X]", can_id, data[0]);
        return true;
    }
    else
    {
        debug_send("CAN: TX failed - status %d", status);
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
        return;
    }

    uint32_t timestamp = HAL_GetTick();
    uint32_t can_id    = rx_header.ExtId;

    // Copy incoming data to structs - no complex processing
    switch (can_id)
    {
    // Sensor 0 messages
    case CAN_S0_HEADER_ID:
        memcpy(&can_system.sensors[0].frame_number, &rx_data[4], 4);
        can_system.sensors[0].num_points    = rx_data[0] - 0x1A + 1; // Simple conversion
        can_system.sensors[0].last_msg_time = timestamp;
        can_system.sensors[0].msg_count++;
        can_system.sensors[0].online = true;
        can_system.sensors[0].status = RADAR_CHIRPING;
        break;

    case CAN_S0_OBJECT_ID:
        // Copy detection points directly
        memcpy(&can_system.sensors[0].detection_points[0], &rx_data[0], 4);
        memcpy(&can_system.sensors[0].detection_points[1], &rx_data[4], 4);
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
        can_system.sensors[1].num_points    = rx_data[0] - 0x1A + 1;
        can_system.sensors[1].last_msg_time = timestamp;
        can_system.sensors[1].msg_count++;
        can_system.sensors[1].online = true;
        can_system.sensors[1].status = RADAR_CHIRPING;
        break;

    case CAN_S1_OBJECT_ID:
        memcpy(&can_system.sensors[1].detection_points[0], &rx_data[0], 4);
        memcpy(&can_system.sensors[1].detection_points[1], &rx_data[4], 4);
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
        can_system.sensors[2].num_points    = rx_data[0] - 0x1A + 1;
        can_system.sensors[2].last_msg_time = timestamp;
        can_system.sensors[2].msg_count++;
        can_system.sensors[2].online = true;
        can_system.sensors[2].status = RADAR_CHIRPING;
        break;

    case CAN_S2_OBJECT_ID:
        memcpy(&can_system.sensors[2].detection_points[0], &rx_data[0], 4);
        memcpy(&can_system.sensors[2].detection_points[1], &rx_data[4], 4);
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
        debug_send("CAN: Unknown ID 0x%08lX", can_id);
        break;
    }
}

/*------------------------------------------------------------------------------
 * Public API Implementation
 *----------------------------------------------------------------------------*/

bool can_init(void)
{
    debug_send("=== CAN INITIALIZATION START ===");

    // 1. Clear system data
    memset(&can_system, 0, sizeof(can_system));
    can_system.init_start_time = HAL_GetTick();
    can_system.system_status   = RADAR_INITIALISING;

    // Initialize sensor status
    for (uint8_t i = 0; i < 3; i++)
    {
        can_system.sensors[i].status = RADAR_INITIALISING;
    }

    // 2. Setup CAN hardware
    debug_send("CAN: Setting up hardware");
    if (!setup_can_hardware())
    {
        debug_send("CAN: ERROR - Hardware setup failed");
        return false;
    }

    // 3. Wait 5 seconds for sensor boot
    debug_send("CAN: Waiting 5 seconds for sensor boot...");
    HAL_Delay(5000);

    // 4. Send start command to all sensors
    debug_send("CAN: Sending START command to all sensors");
    can_send_to_all_sensors(CAN_CMD_START);
    HAL_Delay(200);

    // 5. Send version query to all sensors
    debug_send("CAN: Sending VERSION query to all sensors");
    can_send_to_all_sensors(CAN_CMD_QUERY_STATUS);

    // 6. Wait 1 second for responses
    debug_send("CAN: Waiting 1 second for sensor responses...");
    HAL_Delay(1000);

    // 7. Check and report sensor status
    debug_send("CAN: Checking sensor responses:");
    for (uint8_t i = 0; i < 3; i++)
    {
        can_sensor_t *sensor = &can_system.sensors[i];
        if (sensor->online)
        {
            sensor->status = RADAR_READY;
            debug_send("  Sensor %d: ONLINE - FW v%d.%d.%d - Status: 0x%02X - Messages: %lu",
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
            debug_send("  Sensor %d: OFFLINE", i);
        }
    }

    // 8. Update system status
    can_system.online_count = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        if (can_system.sensors[i].online)
        {
            can_system.online_count++;
        }
    }

    debug_send("CAN: Active sensors: %d/3", can_system.online_count);

    // 9. Turn off sensors after initialization
    debug_send("CAN: Turning OFF all sensors after initialization");
    can_send_to_all_sensors(CAN_CMD_STOP);

    // Mark sensors as stopped
    for (uint8_t i = 0; i < 3; i++)
    {
        if (can_system.sensors[i].online)
        {
            can_system.sensors[i].status = RADAR_STOPPED;
        }
    }

    // 10. Mark system as initialized
    can_system.system_initialized = true;
    can_system.system_status      = (can_system.online_count > 0) ? RADAR_READY : RADAR_STOPPED;

    uint32_t init_time = HAL_GetTick() - can_system.init_start_time;
    debug_send("CAN: Initialization complete in %lu ms", init_time);
    debug_send("=== CAN INITIALIZATION END ===");

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
    return can_system.system_initialized && (can_system.online_count >= 2);
}

can_status_t can_get_system_status(void)
{
    return can_system.system_status;
}

void can_process_timeouts(void)
{
    uint32_t now                   = HAL_GetTick();
    uint8_t  previous_online_count = can_system.online_count;

    can_system.online_count = 0;

    // Check each sensor for timeout
    for (uint8_t i = 0; i < 3; i++)
    {
        if ((now - can_system.sensors[i].last_msg_time) > 3000)
        { // 3 second timeout
            if (can_system.sensors[i].online)
            {
                can_system.sensors[i].online = false;
                can_system.sensors[i].status = RADAR_STOPPED;
                debug_send("CAN: Sensor %d timeout", i);
            }
        }
        else if (can_system.sensors[i].online)
        {
            can_system.online_count++;
        }
    }

    // Update system status if online count changed
    if (previous_online_count != can_system.online_count)
    {
        if (can_system.online_count == 0)
        {
            can_system.system_status = RADAR_STOPPED;
        }
        else
        {
            can_system.system_status = RADAR_READY;
        }
    }
}

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

        debug_send("=== CAN PERIODIC TEST #%lu ===", test_counter);
        debug_send("Test run count: %lu", test_counter);

        // 1. Send start command to all sensors
        debug_send("Sending START to all sensors");
        can_send_to_all_sensors(CAN_CMD_START);
        HAL_Delay(200);

        // 2. Send version command
        debug_send("Sending VERSION query to all sensors");
        can_send_to_all_sensors(CAN_CMD_QUERY_STATUS);
        HAL_Delay(500); // Wait for responses

        // 3. Retrieve and output status and version
        debug_send("Sensor Status:");
        for (uint8_t i = 0; i < 3; i++)
        {
            can_sensor_t *sensor       = can_get_sensor(i);
            const char   *status_str[] = { "INIT", "READY", "CHIRP", "STOP" };

            if (sensor->online)
            {
                debug_send("  S%d: ONLINE - %s - FW v%d.%d.%d - Status: 0x%02X - Msgs: %lu",
                           i,
                           status_str[sensor->status],
                           sensor->fw_version[0],
                           sensor->fw_version[1],
                           sensor->fw_version[2],
                           sensor->status_code,
                           sensor->msg_count);
            }
            else
            {
                debug_send("  S%d: OFFLINE - %s", i, status_str[sensor->status]);
            }
        }

        // 4. Turn off sensors
        debug_send("Turning OFF all sensors");
        can_send_to_all_sensors(CAN_CMD_STOP);

        debug_send("=== TEST COMPLETE ===");
    }
}

void can_debug_system_status(void)
{
    const char *status_strings[] = { "INITIALISING", "READY", "CHIRPING", "STOPPED" };

    debug_send("=== CAN SYSTEM STATUS ===");
    debug_send("System initialized: %s", can_system.system_initialized ? "YES" : "NO");
    debug_send("System status: %s", status_strings[can_system.system_status]);
    debug_send("Online sensors: %d/3", can_system.online_count);
    debug_send("System healthy: %s", can_is_system_healthy() ? "YES" : "NO");

    for (uint8_t i = 0; i < 3; i++)
    {
        can_sensor_t *s = &can_system.sensors[i];
        debug_send("Sensor %d: %s - %s - FW v%d.%d.%d - Status: 0x%02X - Msgs: %lu",
                   i,
                   s->online ? "ONLINE" : "OFFLINE",
                   status_strings[s->status],
                   s->fw_version[0],
                   s->fw_version[1],
                   s->fw_version[2],
                   s->status_code,
                   s->msg_count);
    }
    debug_send("========================");
}

void can_debug_sensor_indexing(void)
{
    debug_send("=== CAN SENSOR ID MAPPING TEST ===");
    debug_send("Command IDs:");
    debug_send("  Sensor 0: 0x%08X", CAN_S0_CMD_ID);
    debug_send("  Sensor 1: 0x%08X", CAN_S1_CMD_ID);
    debug_send("  Sensor 2: 0x%08X", CAN_S2_CMD_ID);

    debug_send("Data IDs:");
    debug_send("  S0: 0x%08X-0x%08X", CAN_S0_HEADER_ID, CAN_S0_VERSION_ID);
    debug_send("  S1: 0x%08X-0x%08X", CAN_S1_HEADER_ID, CAN_S1_VERSION_ID);
    debug_send("  S2: 0x%08X-0x%08X", CAN_S2_HEADER_ID, CAN_S2_VERSION_ID);
    debug_send("=================================");
}
