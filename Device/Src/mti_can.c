#include "can.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include <math.h>
#include "mti_imu.h"

CAN_RxHeaderTypeDef rxHeader;                              // CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader;                              // CAN Bus Receive Header
uint8_t             canRX[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // CAN Bus Receive Buffer
CAN_FilterTypeDef   canfil;                                // CAN Bus Filter
uint32_t            canMailbox;                            // CAN Bus Mail box variable

static radar_data_t    radar_data      = { 0 };
static radar_config_t  current_config  = { 0 };
static radar_metrics_t current_metrics = { 0 };

static uint8_t pointIndex = 0; /**< current write‐index into detections[] */

bool can_setup(void)
{
#ifdef PCB_CANBUS
#define FILTER_ID ((0x00000085 << 3) | 0x4)
// #define FILTER_MASK ((0x000000D2 << 3) | 0x4)
#define FILTER_MASK 0

    canfil.FilterBank           = 0;
    canfil.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh         = FILTER_ID >> 16;
    canfil.FilterIdLow          = FILTER_ID & 0xFFFF;
    canfil.FilterMaskIdHigh     = FILTER_MASK >> 16;
    canfil.FilterMaskIdLow      = FILTER_MASK & 0xFFFF;
    canfil.FilterScale          = CAN_FILTERSCALE_32BIT;
    canfil.FilterActivation     = ENABLE;
    // canfil.SlaveStartFilterBank = 14;

    // 0x80 1000 0000
    // 0xA0 1010 0000
    // 0xA1 1010 0001
    // 0xA2 1010 0010
    // 0xA3 1010 0011
    // 0xA4 1010 0100

    HAL_CAN_ConfigFilter(&hcan1, &canfil); // Initialize CAN Filter
    HAL_CAN_Start(&hcan1);                 // Initialize CAN Bus
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
    {
        return true;
    } // Initialize CAN Bus Rx Interrupt
#endif
    return false;
}

bool can_send(uint32_t ID, uint8_t message)
{
#ifdef PCB_CANBUS
    if (ID == CAN_CMD)
    {
        if (message == CAN_START)
        {
            syn_radar_set(RADAR_CHIRPING);
        }
        else if (message == CAN_STOP)
        {
            syn_radar_set(RADAR_STOPPED);
        }
    }
    txHeader.DLC                = 1; // Number of bites to be transmitted max- 8
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID;
    txHeader.ExtId              = ID;
    txHeader.TransmitGlobalTime = DISABLE;
    DEBUG_SEND("CAN TX (0x%02X) 0x%02X", ID, message);
    uint8_t csend[1];
    csend[0] = message;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox) == HAL_OK)
    {
        return true; // Send Message
    }
#endif
    return false;
}

bool can_send_array(uint32_t ID, uint8_t *message, size_t length)
{
#ifdef PCB_CANBUS
    txHeader.DLC                = length; // Number of bites to be transmitted max- 8
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
    DEBUG_SEND("CAN TX (0x%02X) 0x%s", ID, data_hex);
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, message, &canMailbox) == HAL_OK)
        return true; // Send Message
#endif
    return false;
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan1)
{
    DEBUG_SEND("CAN FIFO FULL");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX) != HAL_OK)
    {
        DEBUG_SEND("CAN RX ERROR");
        return;
    }

    switch (rxHeader.ExtId)
    {
    case CAN_ID_HEADER:
        if (radar_data.current_frame.frame_complete)
            radar_data.last_complete_frame = radar_data.current_frame;

        memcpy(&radar_data.current_frame.total_packet_length, canRX, 4);
        memcpy(&radar_data.current_frame.frame_number, canRX + 4, 4);
        radar_data.current_frame.num_detections     = radar_data.current_frame.total_packet_length - 129;
        radar_data.current_frame.frame_timestamp_ms = HAL_GetTick();
        radar_data.current_frame.frame_complete     = false;
        radar_data.current_frame.max_snr_db         = 0;
        radar_data.frames_received++;
        radar_data.new_frame_available = false;
        pointIndex                     = 0;
        break;

    case CAN_ID_OBJECT:
        if (pointIndex < radar_data.current_frame.num_detections && pointIndex < 20)
        {
            // bytes 0–3: distance, 4–7: snr
            memcpy(&radar_data.current_frame.detections[pointIndex].distance_m, canRX, 4);
            memcpy(&radar_data.current_frame.detections[pointIndex].snr_db, canRX + 4, 4);
            // bytes 8–11: velocity, 12–15: angle, byte 16: confidence
            memcpy(&radar_data.current_frame.detections[pointIndex].velocity_mps, canRX + 8, 4);
            memcpy(&radar_data.current_frame.detections[pointIndex].angle_deg, canRX + 12, 4);
            radar_data.current_frame.detections[pointIndex].confidence   = canRX[16];
            radar_data.current_frame.detections[pointIndex].timestamp_ms = HAL_GetTick();

            if (radar_data.current_frame.detections[pointIndex].snr_db > radar_data.current_frame.max_snr_db)
            {
                radar_data.current_frame.max_snr_db = radar_data.current_frame.detections[pointIndex].snr_db;
            }

            pointIndex++;
            if (pointIndex >= radar_data.current_frame.num_detections)
            {
                radar_data.current_frame.frame_complete = true;
                radar_data.last_frame_time_ms           = HAL_GetTick();
                radar_data.new_frame_available          = true;
                // radar_frame_complete_callback(&radar_data.current_frame);
            }
        }
        break;

    case CAN_ID_PROFILE:
        // PDF §3.2: radar reports active profile
        radar_data.current_frame.profile_mode = canRX[0];
        radar_profile_switch(canRX[0]);
        break;

    case CAN_ID_VERSION:
        // PDF §3.5: version info
        version_set(canRX);
        break;

    case CAN_ID_STATUS:
        radar_data.status = canRX[0];
        break;

    // PCB self‐test (if still required)
    case CAN_CMD:
        if (canRX[0] == CAN_STATUS)
        {
            pcb_self_test();
        }
        break;

    default:
        break;
    }
}

bool radar_frame_available(void)
{
    return radar_data.new_frame_available;
}

bool radar_get_latest_frame(radar_frame_t *frame)
{
    if (!frame || !radar_data.new_frame_available)
        return false;
    *frame                         = radar_data.last_complete_frame;
    radar_data.new_frame_available = false;
    return true;
}

/**
 * @brief Get radar configuration
 * @param[out] config Pointer to store configuration data
 * @return true if successful, false otherwise
 */
bool radar_get_config(radar_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    // Return current cached configuration
    // In a full implementation, this might request config from radar via CAN
    *config = current_config;
    return true;
}

/**
 * @brief Set radar configuration
 * @param[in] config Pointer to new configuration
 * @return true if successful, false otherwise
 */
bool radar_set_config(const radar_config_t *cfg)
{
    if (!cfg || cfg->active_profile > RADAR_PROFILE_100M_MULTI)
        return false;

    current_config = *cfg;
    // per PDF: use CAN_CMD ID, payload = CAN_PROFILE + profile index
    return can_send(CAN_CMD, (uint8_t)(CAN_PROFILE << 4 | cfg->active_profile));
}

/**
 * @brief Get radar performance metrics
 * @param[out] metrics Pointer to store metrics data
 * @return true if successful, false otherwise
 */
bool radar_get_metrics(radar_metrics_t *metrics)
{
    if (metrics == NULL)
    {
        return false;
    }

    // Calculate metrics from current radar data
    current_metrics.total_frames     = radar_data.frames_received;
    current_metrics.valid_detections = 0;

    // Count valid detections from last complete frame
    if (radar_data.last_complete_frame.frame_complete)
    {
        for (int i = 0; i < radar_data.last_complete_frame.num_detections; i++)
        {
            if (radar_data.last_complete_frame.detections[i].distance_m > 0)
            {
                current_metrics.valid_detections++;
            }
        }

        // Calculate average detections per frame
        if (current_metrics.total_frames > 0)
        {
            current_metrics.average_detections_per_frame = (float)current_metrics.valid_detections / current_metrics.total_frames;
        }
    }

    *metrics = current_metrics;
    return true;
}

/**
 * @brief Reset radar statistics
 * @return true if successful, false otherwise
 */
bool radar_reset_statistics(void)
{
    // Reset radar data counters
    radar_data.frames_received    = 0;
    radar_data.frames_dropped     = 0;
    radar_data.error_count        = 0;
    radar_data.last_frame_time_ms = 0;

    // Reset metrics
    memset(&current_metrics, 0, sizeof(radar_metrics_t));
    current_metrics.last_reset_time_ms = HAL_GetTick();

    return true;
}

/**
 * @brief Get detection by index from latest frame
 * @param[in] index Detection index (0 to num_detections-1)
 * @param[out] detection Pointer to store detection data
 * @return true if valid detection found, false otherwise
 */
bool radar_get_detection_by_index(uint8_t index, radar_detection_t *detection)
{
    if (detection == NULL || !radar_data.last_complete_frame.frame_complete)
    {
        return false;
    }

    if (index >= radar_data.last_complete_frame.num_detections)
    {
        return false;
    }

    *detection = radar_data.last_complete_frame.detections[index];
    return true;
}

/**
 * @brief Find closest detection in latest frame
 * @param[out] detection Pointer to store closest detection
 * @return true if detection found, false otherwise
 */
bool radar_get_closest_detection(radar_detection_t *detection)
{
    if (detection == NULL || !radar_data.last_complete_frame.frame_complete)
    {
        return false;
    }

    if (radar_data.last_complete_frame.num_detections == 0)
    {
        return false;
    }

    float   min_distance  = radar_data.last_complete_frame.detections[0].distance_m;
    uint8_t closest_index = 0;

    // Find detection with minimum distance
    for (uint8_t i = 1; i < radar_data.last_complete_frame.num_detections; i++)
    {
        if (radar_data.last_complete_frame.detections[i].distance_m < min_distance)
        {
            min_distance  = radar_data.last_complete_frame.detections[i].distance_m;
            closest_index = i;
        }
    }

    *detection = radar_data.last_complete_frame.detections[closest_index];
    return true;
}
