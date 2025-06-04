#include "can.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include <string.h>

CAN_RxHeaderTypeDef rxHeader;
CAN_TxHeaderTypeDef txHeader;
uint8_t             canRX[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
CAN_FilterTypeDef   canfil;
uint32_t            canMailbox;

// Multi-sensor system instance
multi_radar_system_t radar_system = { 0 };

bool can_setup(void)
{
#ifdef PCB_CANBUS
// Configure filter to accept both command (0x80-0x8F) and data (0xA0-0xBF) messages
#define FILTER_ID_CMD    ((0x00000080 << 3) | 0x4)
#define FILTER_MASK_CMD  ((0x000000F0 << 3) | 0x4)
#define FILTER_ID_DATA   ((0x000000A0 << 3) | 0x4)
#define FILTER_MASK_DATA ((0x000000F0 << 3) | 0x4)

    // Filter 0: Command messages (0x80-0x8F)
    canfil.FilterBank           = 0;
    canfil.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh         = FILTER_ID_CMD >> 16;
    canfil.FilterIdLow          = FILTER_ID_CMD & 0xFFFF;
    canfil.FilterMaskIdHigh     = FILTER_MASK_CMD >> 16;
    canfil.FilterMaskIdLow      = FILTER_MASK_CMD & 0xFFFF;
    canfil.FilterScale          = CAN_FILTERSCALE_32BIT;
    canfil.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &canfil);

    // Filter 1: Data messages (0xA0-0xBF)
    canfil.FilterBank       = 1;
    canfil.FilterIdHigh     = FILTER_ID_DATA >> 16;
    canfil.FilterIdLow      = FILTER_ID_DATA & 0xFFFF;
    canfil.FilterMaskIdHigh = FILTER_MASK_DATA >> 16;
    canfil.FilterMaskIdLow  = FILTER_MASK_DATA & 0xFFFF;
    HAL_CAN_ConfigFilter(&hcan1, &canfil);

    HAL_CAN_Start(&hcan1);
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
    {
        return true;
    }
#endif
    return false;
}

bool can_send(uint32_t ID, uint8_t message)
{
#ifdef PCB_CANBUS
    txHeader.DLC                = 1;
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID;
    txHeader.ExtId              = ID;
    txHeader.TransmitGlobalTime = DISABLE;

    debug_send("CAN TX (0x%02X) 0x%02X", ID, message);

    uint8_t csend[1] = { message };
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox) == HAL_OK)
        return true;
#endif
    return false;
}

bool can_send_array(uint32_t ID, uint8_t *message, size_t length)
{
#ifdef PCB_CANBUS
    txHeader.DLC                = length;
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID;
    txHeader.ExtId              = ID;
    txHeader.TransmitGlobalTime = DISABLE;

    char data_hex[17];
    memset(data_hex, 0, 17);
    for (int i = 0; i < length; i++)
    {
        sprintf(data_hex, "%s%02X", data_hex, message[i]);
    }
    debug_send("CAN TX (0x%02X) 0x%s", ID, data_hex);

    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, message, &canMailbox) == HAL_OK)
        return true;
#endif
    return false;
}

// Send command to specific sensor
bool can_send_to_sensor(uint8_t sensor_idx, uint32_t base_id, uint8_t message)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
        return false;

    uint32_t sensor_id = base_id + CAN_SENSOR_OFFSET(sensor_idx);
    return can_send(sensor_id, message);
}

// Broadcast command to all sensors
void broadcast_to_all_sensors(uint32_t base_id, uint8_t message)
{
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        can_send_to_sensor(i, base_id, message);
    }
}

// Corrected sensor index extraction
uint8_t get_sensor_index_from_can_id(uint32_t can_id)
{
    // Handle command messages (0x80-0x82)
    if ((can_id & 0xF0) == 0x80)
    {
        uint8_t sensor_idx = can_id & 0x0F;
        if (sensor_idx < MAX_RADAR_SENSORS)
            return sensor_idx;
    }

    // Handle data messages (0xA0-0xC4 range)
    if ((can_id & 0xF0) >= 0xA0 && (can_id & 0xF0) <= 0xC0)
    {
        uint8_t offset     = can_id & 0x0F;
        uint8_t sensor_idx = offset >> 4; // Extract sensor index from upper nibble of offset
        if (sensor_idx < MAX_RADAR_SENSORS)
            return sensor_idx;
    }

    return 0xFF; // Invalid sensor index
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX) == HAL_OK)
    {
        can_data_union_t rx_data;
        memcpy(rx_data.bytes, canRX, rxHeader.DLC);

        // Handle command messages
        if ((rxHeader.ExtId & 0xF0) == 0x80)
        {
            uint8_t sensor_idx = rxHeader.ExtId & 0x0F;
            uint8_t command    = rx_data.bytes[0];

            debug_send("CAN CMD S%d: 0x%02X Data: 0x%02X", sensor_idx, rxHeader.ExtId, command);

            if (sensor_idx < MAX_RADAR_SENSORS)
            {
                process_sensor_command(sensor_idx, command, &rx_data);
            }
            return;
        }

        // Handle data messages
        uint8_t sensor_idx = get_sensor_index_from_can_id(rxHeader.ExtId);

        if (sensor_idx >= MAX_RADAR_SENSORS)
        {
            debug_send("Invalid sensor index for ID: 0x%02X", rxHeader.ExtId);
            return;
        }

        radar_data_t *sensor  = &radar_system.sensors[sensor_idx];
        uint8_t       base_id = rxHeader.ExtId & 0xF0;

        // Debug output
        char data_hex[17];
        memset(data_hex, 0, 17);
        for (int i = 0; i < rxHeader.DLC; i++)
        {
            sprintf(data_hex, "%s%02X", data_hex, rx_data.bytes[i]);
        }
        debug_send("CAN RX S%d: (0x%02X) 0x%s", sensor_idx, rxHeader.ExtId, data_hex);

        switch (base_id)
        {
        case 0xA0: // Header
            sensor->totalPacketLength = rx_data.words[0];
            sensor->frameNumber       = rx_data.words[1];
            sensor->numDetPoints      = (sensor->totalPacketLength > 129) ? (sensor->totalPacketLength - 129) : 0;
            sensor->pointIndex        = 0;
            sensor->maxSNR            = 0;
            sensor->new_data_ready    = false;

            debug_send("S%d Frame: %d, Points: %d", sensor_idx, sensor->frameNumber, sensor->numDetPoints);
            break;

        case 0xA1: // Detected point
            if (sensor->pointIndex < MAX_RADAR_DETECTED_POINTS && sensor->pointIndex < sensor->numDetPoints)
            {
                sensor->detectedPoints[sensor->pointIndex][0] = rx_data.floats[0];
                sensor->detectedPoints[sensor->pointIndex][1] = rx_data.floats[1];

                debug_send("S%d Point %d: %.2f, %.2f",
                           sensor_idx,
                           sensor->pointIndex,
                           sensor->detectedPoints[sensor->pointIndex][0],
                           sensor->detectedPoints[sensor->pointIndex][1]);

                sensor->pointIndex++;

                if (sensor->pointIndex >= sensor->numDetPoints)
                {
                    sensor->new_data_ready = true;
                    // Process complete frame for this sensor
                    process_complete_radar_frame(sensor_idx);
                }
            }
            break;

        case 0xA3: // Status
            sensor->status = (radar_hw_status_t)rx_data.bytes[0];
            debug_send("S%d Status: %d", sensor_idx, rx_data.bytes[0]);
            break;

        case 0xA4: // Version
            sensor->version.major = rx_data.bytes[0];
            sensor->version.minor = rx_data.bytes[1];
            sensor->version.sub   = rx_data.bytes[2];
            debug_send("S%d Version: %d.%d.%d", sensor_idx, sensor->version.major, sensor->version.minor, sensor->version.sub);
            break;
        }

        radar_system.last_message_timestamp[sensor_idx] = HAL_GetTick();
        radar_system.sensor_online[sensor_idx]          = true;
    }
    else
    {
        debug_send("CAN RX ERROR");
    }
}

void process_sensor_command(uint8_t sensor_idx, uint8_t command, can_data_union_t *data)
{
    radar_data_t *sensor = &radar_system.sensors[sensor_idx];

    switch (command)
    {
    case CAN_CMD_START:
        debug_send("S%d: Start command received", sensor_idx);
        break;

    case CAN_CMD_STOP:
        debug_send("S%d: Stop command received", sensor_idx);
        break;

    case CAN_CMD_STATUS:
        can_send(CAN_MSG_ID_STATUS_SENSOR(sensor_idx), (uint8_t)sensor->status);
        // Set radar initialization to OK when we get response
        radar_init_status_set(RADAR_INIT_OK);
        radar_status_set(RADAR_READY);
        break;

    case CAN_CMD_CAL:
        debug_send("S%d: Calibration command received", sensor_idx);
        break;

    case CAN_CMD_POWER:
        if (data && data->bytes[1]) // Check if there's additional data
        {
            uint8_t power_level = data->bytes[1];
            debug_send("S%d: Power level set to %d%%", sensor_idx, power_level);
        }
        break;

    default:
        debug_send("S%d: Unknown command 0x%02X", sensor_idx, command);
        break;
    }
}

void process_complete_radar_frame(uint8_t sensor_idx)
{
    radar_data_t *sensor = &radar_system.sensors[sensor_idx];

    debug_send("S%d: Frame %d complete with %d points", sensor_idx, sensor->frameNumber, sensor->numDetPoints);

    // Call the simplified radar processing function
    radar_process_measurement(sensor_idx, sensor->detectedPoints, sensor->numDetPoints);
}
