/**
 * @file    mti_can.c
 * @author  Mti Group
 * @brief   CAN interface implementation for radar sensor communication
 * @version 1.2.0 (Updated for naming and status conventions)
 * @date    2025-05-22
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "can.h"
#include "vmt_common_defs.h"
#include "vmt_uart.h"
#include "mti_can.h"

#define CAN_TX_DEBUG_BUFFER_SIZE 17

static struct
{
    uint32_t filter_id;
    uint8_t  threshold;
    uint8_t  fov;
    uint8_t  mode;
    uint8_t  power;
} prv_can_default_sensor_params = { .filter_id = DEFAULT_RADAR_FILTER_ID_BASE,
                                    .threshold = DEFAULT_RADAR_DETECTION_THRESHOLD,
                                    .fov       = DEFAULT_RADAR_FOV,
                                    .mode      = DEFAULT_RADAR_OPERATING_MODE,
                                    .power     = DEFAULT_RADAR_POWER_PERCENT };

static CAN_TxHeaderTypeDef prv_can_tx_header;
static uint32_t            prv_can_tx_mailbox;
static can_state_t         prv_can_current_state = CAN_STATE_UNINIT;

static radar_data_t prv_radar_sensor_data_array[MAX_SENSORS];
static uint32_t     prv_can_total_error_count   = 0;
static uint32_t     prv_can_total_timeout_count = 0;
static uint32_t     prv_can_last_msg_rx_time    = 0;

static radar_hw_status_t prv_sensor_hw_status_array[MAX_SENSORS];
static uint8_t           prv_sensor_signal_strength_array[MAX_SENSORS];
static uint8_t           prv_sensor_reset_attempt_counts_array[MAX_SENSORS];

static radar_data_callback_t prv_global_radar_data_callback = NULL;

static HAL_StatusTypeDef
prv_can_hal_configure_filter(CAN_FilterTypeDef *filter_config, uint8_t filter_bank_offset, uint32_t id1, uint32_t mask_or_id2, bool is_mask_mode);
static HAL_StatusTypeDef prv_can_hal_enable_notifications(void);
static can_state_t       prv_can_validate_tx_params(const uint8_t *data, size_t length);
static void              prv_can_configure_hal_tx_header(CAN_TxHeaderTypeDef *tx_hdr, uint32_t msg_id, size_t data_len);
static void              prv_can_format_data_for_debug(char *hex_buf, const uint8_t *data, size_t data_len);
static bool              prv_can_rx_get_message(CAN_HandleTypeDef *hcan_ptr, CAN_RxHeaderTypeDef *rx_hdr, uint8_t *data_buf);
static bool              prv_can_rx_validate_dlc(const CAN_RxHeaderTypeDef *rx_hdr, uint8_t min_dlc);
static void              prv_can_rx_convert_bytes_to_union(can_data_union_t *data_union, const uint8_t *bytes, uint8_t dlc);
static void              prv_can_rx_process_header_msg(uint8_t sensor_idx, const CAN_RxHeaderTypeDef *rx_hdr, const uint8_t *data);
static void              prv_can_rx_process_object_msg(uint8_t sensor_idx, const CAN_RxHeaderTypeDef *rx_hdr, const uint8_t *data);
static void              prv_can_rx_process_status_msg(uint8_t sensor_idx, const CAN_RxHeaderTypeDef *rx_hdr, const uint8_t *data);
static void              prv_can_rx_process_version_msg(uint8_t sensor_idx, const CAN_RxHeaderTypeDef *rx_hdr, const uint8_t *data);
static void              prv_can_debug_print_radar_hw_status(radar_hw_status_t status);
static uint8_t           prv_can_get_sensor_idx_from_id(uint32_t can_id);
static void              prv_can_calculate_snr_for_sensor_data(uint8_t sensor_idx, radar_data_t *radar_data);


static HAL_StatusTypeDef
prv_can_hal_configure_filter(CAN_FilterTypeDef *filter_config, uint8_t filter_bank_offset, uint32_t id1, uint32_t mask_or_id2, bool is_mask_mode)
{
    memset(filter_config, 0, sizeof(CAN_FilterTypeDef));
    filter_config->FilterBank  = filter_bank_offset;
    filter_config->FilterMode  = is_mask_mode ? CAN_FILTERMODE_IDMASK : CAN_FILTERMODE_IDLIST;
    filter_config->FilterScale = CAN_FILTERSCALE_32BIT;

    filter_config->FilterIdHigh = (uint16_t)((id1 >> 13) & 0xFFFF);
    filter_config->FilterIdLow  = (uint16_t)(((id1 << 3) & 0xFFF8) | CAN_ID_EXT);

    if (is_mask_mode)
    {
        filter_config->FilterMaskIdHigh = (uint16_t)((mask_or_id2 >> 13) & 0xFFFF);
        filter_config->FilterMaskIdLow  = (uint16_t)(((mask_or_id2 << 3) & 0xFFF8) | CAN_ID_EXT);
    }
    else
    {
        filter_config->FilterMaskIdHigh = (uint16_t)((mask_or_id2 >> 13) & 0xFFFF);
        filter_config->FilterMaskIdLow  = (uint16_t)(((mask_or_id2 << 3) & 0xFFF8) | CAN_ID_EXT);
    }

    filter_config->FilterFIFOAssignment = CAN_RX_FIFO0;
    filter_config->FilterActivation     = ENABLE;
    filter_config->SlaveStartFilterBank = 14;

    return HAL_CAN_ConfigFilter(&hcan1, filter_config);
}

static HAL_StatusTypeDef prv_can_hal_enable_notifications(void)
{
    return HAL_CAN_ActivateNotification(&hcan1,
                                        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
                                            CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);
}

can_state_t can_init(void)
{
    if (prv_can_current_state == CAN_STATE_READY)
    {
        return CAN_STATE_READY;
    }

    CAN_FilterTypeDef default_filter;
    if (prv_can_hal_configure_filter(&default_filter, 0, 0x00000000, 0x00000000, true) != HAL_OK)
    {
        DEBUG_SEND("CAN Init: Default filter config failed.");
        prv_can_current_state = CAN_STATE_ERROR;
        return CAN_STATE_ERROR;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        DEBUG_SEND("CAN Init: HAL_CAN_Start failed (Error: 0x%08lX).", HAL_CAN_GetError(&hcan1));
        prv_can_current_state = CAN_STATE_ERROR;
        return CAN_STATE_ERROR;
    }

    if (prv_can_hal_enable_notifications() != HAL_OK)
    {
        DEBUG_SEND("CAN Init: Notification activation failed.");
        prv_can_current_state = CAN_STATE_ERROR;
        return CAN_STATE_ERROR;
    }

    for (uint8_t i = 0; i < MAX_SENSORS; i++)
    {
        memset(&prv_radar_sensor_data_array[i], 0, sizeof(radar_data_t));
        prv_sensor_hw_status_array[i]            = RADAR_HW_STATUS_INITIALISING;
        prv_sensor_signal_strength_array[i]      = 0;
        prv_sensor_reset_attempt_counts_array[i] = 0;
        prv_radar_sensor_data_array[i].status    = RADAR_HW_STATUS_INITIALISING;
    }

    prv_can_current_state    = CAN_STATE_READY;
    prv_can_last_msg_rx_time = HAL_GetTick();
    DEBUG_SEND("CAN: Interface initialized.");
    return CAN_STATE_READY;
}

can_state_t can_get_state(void)
{
    return prv_can_current_state;
}

static can_state_t prv_can_validate_tx_params(const uint8_t *data, size_t length)
{
    if (data == NULL && length > 0)
    {
        DEBUG_SEND("CAN TX Error: Null data with non-zero length %u.", (unsigned int)length);
        return CAN_STATE_ERROR;
    }
    if (length > 8)
    {
        DEBUG_SEND("CAN TX Error: Data length %u exceeds 8 bytes.", (unsigned int)length);
        return CAN_STATE_ERROR;
    }
    return CAN_STATE_READY;
}

static void prv_can_configure_hal_tx_header(CAN_TxHeaderTypeDef *tx_hdr, uint32_t msg_id, size_t data_len)
{
    memset(tx_hdr, 0, sizeof(CAN_TxHeaderTypeDef));
    tx_hdr->ExtId              = msg_id;
    tx_hdr->IDE                = CAN_ID_EXT;
    tx_hdr->RTR                = CAN_RTR_DATA;
    tx_hdr->DLC                = data_len;
    tx_hdr->TransmitGlobalTime = DISABLE;
}

static void prv_can_format_data_for_debug(char *hex_buf, const uint8_t *data, size_t data_len)
{
    memset(hex_buf, 0, CAN_TX_DEBUG_BUFFER_SIZE);
    for (size_t i = 0; i < data_len && i < 8; ++i)
    {
        snprintf(&hex_buf[i * 2], 3, "%02X", data[i]);
    }
}

can_state_t can_send_byte(uint32_t id, uint8_t message)
{
    return can_send_array(id, &message, 1);
}

can_state_t can_send_array(uint32_t id, const uint8_t *message, size_t length)
{
    if (prv_can_current_state != CAN_STATE_READY && prv_can_current_state != CAN_STATE_BUSY)
    {
        DEBUG_SEND("CAN TX Error: Interface not ready (State: %d).", prv_can_current_state);
        return prv_can_current_state;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        return CAN_STATE_BUSY;
    }
    if (prv_can_validate_tx_params(message, length) != CAN_STATE_READY)
    {
        return CAN_STATE_ERROR;
    }

    prv_can_configure_hal_tx_header(&prv_can_tx_header, id, length);

    if (g_debug_enabled)
    {
        char data_hex_str[CAN_TX_DEBUG_BUFFER_SIZE];
        prv_can_format_data_for_debug(data_hex_str, message, length);
        DEBUG_SEND("CAN TX (ID: 0x%08lX, DLC: %u) Data: 0x%s", id, (unsigned int)length, data_hex_str);
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &prv_can_tx_header, (uint8_t *)message, &prv_can_tx_mailbox) != HAL_OK)
    {
        DEBUG_SEND("CAN TX Error: HAL_CAN_AddTxMessage failed (Error: 0x%08lX).", HAL_CAN_GetError(&hcan1));
        prv_can_total_error_count++;
        return CAN_STATE_ERROR;
    }
    return CAN_STATE_READY;
}

can_state_t can_send_to_sensor(uint8_t sensor_idx, uint8_t cmd, const uint8_t *data, size_t data_len)
{
    if (sensor_idx >= MAX_SENSORS)
    {
        DEBUG_SEND("CAN Send Error: Sensor index %u out of range.", sensor_idx);
        return CAN_STATE_ERROR;
    }
    if (data_len > 7)
    {
        DEBUG_SEND("CAN Send Error: Cmd data length %u too large for sensor %u cmd %u.", (unsigned int)data_len, sensor_idx, cmd);
        return CAN_STATE_ERROR;
    }

    uint32_t message_id = CAN_CMD_ID_SENSOR(sensor_idx);
    uint8_t  payload[8];
    size_t   payload_len = 0;

    payload[0]  = cmd;
    payload_len = 1;

    if (data != NULL && data_len > 0)
    {
        memcpy(&payload[1], data, data_len);
        payload_len += data_len;
    }
    return can_send_array(message_id, payload, payload_len);
}

can_state_t can_request_radar_data(void)
{
    can_state_t overall_send_status = CAN_STATE_READY;
    bool        any_busy            = false;

    for (uint8_t i = 0; i < MAX_SENSORS; ++i)
    {
        if (prv_sensor_hw_status_array[i] == RADAR_HW_STATUS_READY || prv_sensor_hw_status_array[i] == RADAR_HW_STATUS_CHIRPING ||
            prv_sensor_hw_status_array[i] == RADAR_HW_STATUS_STOPPED)
        {
            can_state_t sensor_send_status = can_send_to_sensor(i, CAN_CMD_START, NULL, 0);
            if (sensor_send_status == CAN_STATE_ERROR)
            {
                overall_send_status = CAN_STATE_ERROR;
                can_handle_error(i, CAN_ERROR_TYPE_UNKNOWN);
            }
            else if (sensor_send_status == CAN_STATE_BUSY)
            {
                any_busy = true;
            }
        }
    }
    if (overall_send_status == CAN_STATE_READY && any_busy)
    {
        return CAN_STATE_BUSY;
    }
    return overall_send_status;
}

uint32_t can_get_error_count(void)
{
    return prv_can_total_error_count;
}

uint32_t can_get_timeout_count(void)
{
    return prv_can_total_timeout_count;
}

uint32_t can_get_time_since_last_msg(void)
{
    uint32_t current_time = HAL_GetTick();
    if (current_time >= prv_can_last_msg_rx_time)
    {
        return current_time - prv_can_last_msg_rx_time;
    }
    return (0xFFFFFFFFUL - prv_can_last_msg_rx_time) + current_time + 1;
}

uint8_t can_get_radar_signal(uint8_t sensor_idx)
{
    if (sensor_idx < MAX_SENSORS)
    {
        return prv_sensor_signal_strength_array[sensor_idx];
    }
    return 0;
}

bool can_handle_error(uint8_t sensor_idx, can_error_type_t error_type)
{
    if (sensor_idx < MAX_SENSORS)
    {
        DEBUG_SEND("CAN Error: Handling type %d for sensor %u.", error_type, sensor_idx);
        if (prv_sensor_hw_status_array[sensor_idx] != RADAR_HW_STATUS_ERROR)
        {
            if (prv_sensor_reset_attempt_counts_array[sensor_idx] < SENSOR_MAX_RESET_ATTEMPTS)
            { // Use SENSOR_MAX_RESET_ATTEMPTS for resets
                DEBUG_SEND("CAN Error: Attempting reset for sensor %u (reset attempt %u/%u).",
                           sensor_idx,
                           prv_sensor_reset_attempt_counts_array[sensor_idx] + 1,
                           SENSOR_MAX_RESET_ATTEMPTS);
                if (can_reset_sensor(sensor_idx) == CAN_STATE_READY)
                {
                    return true;
                }
                else
                {
                    DEBUG_SEND("CAN Error: Reset initiation failed for sensor %u.", sensor_idx);
                    return false;
                }
            }
            else
            { // Exceeded SENSOR_MAX_RESET_ATTEMPTS
                DEBUG_SEND("CAN Error: Sensor %u max reset attempts (%u) reached. Marking as ERROR.", sensor_idx, SENSOR_MAX_RESET_ATTEMPTS);
                prv_sensor_hw_status_array[sensor_idx]         = RADAR_HW_STATUS_ERROR;
                prv_radar_sensor_data_array[sensor_idx].status = RADAR_HW_STATUS_ERROR;
                return false;
            }
        }
        else
        {
            DEBUG_SEND("CAN Error: Sensor %u already in RADAR_HW_STATUS_ERROR.", sensor_idx);
            return false;
        }
    }
    else if (sensor_idx == INVALID_SENSOR_INDEX)
    {
        DEBUG_SEND("CAN Error: Handling generic bus error type %d.", error_type);
        // Generic bus errors usually handled by HAL_CAN_ErrorCallback
        prv_can_total_error_count++;
        return false;
    }
    return false;
}

void can_process(void)
{
    if (prv_can_current_state == CAN_STATE_UNINIT)
    {
        can_init();
        return;
    }
    if (prv_can_current_state == CAN_STATE_ERROR)
    {
        return;
    }

    if (can_get_time_since_last_msg() > CAN_COMM_TIMEOUT_MS)
    {
        prv_can_total_timeout_count++;
        DEBUG_SEND("CAN Timeout: No bus activity for >%lu ms. Count: %lu", (unsigned long)CAN_COMM_TIMEOUT_MS, prv_can_total_timeout_count);
        prv_can_last_msg_rx_time = HAL_GetTick();

        for (uint8_t i = 0; i < MAX_SENSORS; ++i)
        {
            if (prv_sensor_hw_status_array[i] != RADAR_HW_STATUS_ERROR && prv_sensor_hw_status_array[i] != RADAR_HW_STATUS_INITIALISING)
            {
                DEBUG_SEND("CAN Timeout: Sensor %u may be affected by bus timeout.", i);
                // If a global timeout occurs, it might imply individual sensors need checking or resetting.
                // This could call can_handle_error(i, CAN_ERROR_TYPE_TIMEOUT);
                // However, this should be handled carefully to avoid cascading resets if the bus is truly down.
            }
        }
    }

    for (uint8_t i = 0; i < MAX_SENSORS; ++i)
    {
        if (prv_radar_sensor_data_array[i].new_data_ready)
        {
            prv_can_calculate_snr_for_sensor_data(i, &prv_radar_sensor_data_array[i]);
            if (prv_global_radar_data_callback != NULL)
            {
                prv_global_radar_data_callback(i, &prv_radar_sensor_data_array[i]);
            }
            prv_radar_sensor_data_array[i].new_data_ready = false;
        }
    }
}

radar_data_t *can_get_radar_data_ptr(uint8_t sensor_idx)
{
    if (sensor_idx < MAX_SENSORS)
    {
        return &prv_radar_sensor_data_array[sensor_idx];
    }
    return NULL;
}

can_state_t can_reset_sensor(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_SENSORS)
    {
        return CAN_STATE_ERROR;
    }

    // This counter is for reset attempts for THIS sensor
    prv_sensor_reset_attempt_counts_array[sensor_idx]++;
    if (prv_sensor_reset_attempt_counts_array[sensor_idx] > SENSOR_MAX_RESET_ATTEMPTS)
    {
        DEBUG_SEND("CAN Reset: Sensor %u max reset attempts. Marking ERROR.", sensor_idx);
        prv_sensor_hw_status_array[sensor_idx]         = RADAR_HW_STATUS_ERROR;
        prv_radar_sensor_data_array[sensor_idx].status = RADAR_HW_STATUS_ERROR;
        return CAN_STATE_ERROR;
    }

    DEBUG_SEND("CAN Reset: Sensor %u (Attempt %u/%u)", sensor_idx, prv_sensor_reset_attempt_counts_array[sensor_idx], SENSOR_MAX_RESET_ATTEMPTS);
    memset(&prv_radar_sensor_data_array[sensor_idx], 0, sizeof(radar_data_t));
    prv_sensor_hw_status_array[sensor_idx]         = RADAR_HW_STATUS_INITIALISING;
    prv_radar_sensor_data_array[sensor_idx].status = RADAR_HW_STATUS_INITIALISING;
    prv_sensor_signal_strength_array[sensor_idx]   = 0;

    can_state_t cmd_status;
    cmd_status = can_send_to_sensor(sensor_idx, CAN_CMD_STOP, NULL, 0);
    HAL_Delay(SENSOR_RESET_DELAY_MS);
    cmd_status = can_send_to_sensor(sensor_idx, CAN_CMD_INIT, NULL, 0);

    if (cmd_status == CAN_STATE_ERROR)
    {
        DEBUG_SEND("CAN Reset: Failed to send INIT to sensor %u.", sensor_idx);
        return CAN_STATE_ERROR;
    }
    return CAN_STATE_READY;
}

bool can_all_sensors_ready(void)
{
    for (uint8_t i = 0; i < MAX_SENSORS; ++i)
    {
        if (prv_sensor_hw_status_array[i] != RADAR_HW_STATUS_READY && prv_sensor_hw_status_array[i] != RADAR_HW_STATUS_CHIRPING)
        {
            return false;
        }
    }
    return true;
}

radar_hw_status_t can_get_sensor_hw_status(uint8_t sensor_idx)
{
    if (sensor_idx < MAX_SENSORS)
    {
        return prv_sensor_hw_status_array[sensor_idx];
    }
    return RADAR_HW_STATUS_ERROR;
}

bool can_setup_sensor_filter(uint8_t sensor_idx, uint32_t filter_id)
{
    if (sensor_idx >= MAX_SENSORS)
    {
        DEBUG_SEND("CAN Filter Error: Sensor index %u invalid.", sensor_idx);
        return false;
    }
    uint8_t target_filter_bank = sensor_idx + 1;
    if (target_filter_bank >= 14)
    {
        DEBUG_SEND("CAN Filter Error: Target bank %u out of range for sensor %u.", target_filter_bank, sensor_idx);
        return false;
    }

    if (prv_can_current_state != CAN_STATE_READY)
    {
        if (can_init() != CAN_STATE_READY)
        {
            DEBUG_SEND("CAN Filter Error: CAN init failed for sensor %u filter.", sensor_idx);
            return false;
        }
    }

    CAN_FilterTypeDef filter_s;
    if (prv_can_hal_configure_filter(&filter_s, target_filter_bank, filter_id, 0x1FFFFFFF, true) != HAL_OK)
    { // Mask for exact ID match
        DEBUG_SEND("CAN Filter Error: Config failed for S%u, ID 0x%08lX, Bank %u.", sensor_idx, filter_id, target_filter_bank);
        return false;
    }
    DEBUG_SEND("CAN Filter: S%u, ID 0x%08lX, Bank %u configured.", sensor_idx, filter_id, target_filter_bank);
    return true;
}

bool can_setup_all_sensors(void)
{
    if (prv_can_current_state != CAN_STATE_READY)
    {
        if (can_init() != CAN_STATE_READY)
        {
            DEBUG_SEND("CAN SetupAll: CAN init failed. Cannot setup filters.");
            return false;
        }
    }
    DEBUG_SEND("CAN SetupAll: Configuring filters for %d sensors.", MAX_SENSORS);
    bool success = true;
    for (uint8_t i = 0; i < MAX_SENSORS; ++i)
    {
        if (!can_setup_sensor_filter(i, CAN_MSG_ID_HEADER_SENSOR(i)))
        { // Setup for header ID
            DEBUG_SEND("CAN SetupAll: Failed filter for Header ID for sensor %u.", i);
            success = false;
        }
        // Add more calls to can_setup_sensor_filter here if other IDs need specific filters,
        // ensuring filter banks are managed correctly.
    }
    return success;
}

can_state_t can_register_data_callback(radar_data_callback_t callback)
{
    if (callback == NULL)
    {
        prv_global_radar_data_callback = NULL;
        return CAN_STATE_ERROR;
    }
    prv_global_radar_data_callback = callback;
    return CAN_STATE_READY;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == hcan1.Instance)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t             rx_data[8];
        if (prv_can_rx_get_message(hcan, &rx_header, rx_data))
        {
            if (rx_header.IDE == CAN_ID_EXT)
            {
                uint8_t sensor_idx = prv_can_get_sensor_idx_from_id(rx_header.ExtId);
                if (sensor_idx < MAX_SENSORS)
                {
                    if (rx_header.ExtId == CAN_MSG_ID_HEADER_SENSOR(sensor_idx))
                    {
                        prv_can_rx_process_header_msg(sensor_idx, &rx_header, rx_data);
                    }
                    else if (rx_header.ExtId == CAN_MSG_ID_OBJECT_SENSOR(sensor_idx))
                    {
                        prv_can_rx_process_object_msg(sensor_idx, &rx_header, rx_data);
                    }
                    else if (rx_header.ExtId == CAN_MSG_ID_STATUS_SENSOR(sensor_idx))
                    {
                        prv_can_rx_process_status_msg(sensor_idx, &rx_header, rx_data);
                    }
                    else if (rx_header.ExtId == CAN_MSG_ID_VERSION_SENSOR(sensor_idx))
                    {
                        prv_can_rx_process_version_msg(sensor_idx, &rx_header, rx_data);
                    }
                }
            }
        }
    }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == hcan1.Instance)
    {
        DEBUG_SEND("CAN RX Error: FIFO0 Full! Messages lost.");
        prv_can_total_error_count++;
        can_handle_error(INVALID_SENSOR_INDEX, CAN_ERROR_TYPE_FIFO);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == hcan1.Instance)
    {
        uint32_t errorcode = HAL_CAN_GetError(hcan);
        DEBUG_SEND("CAN HAL Error Callback. Code: 0x%08lX", errorcode);
        prv_can_total_error_count++;

        if (errorcode & (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_EWG))
        {
            DEBUG_SEND("CAN: Severe bus error. Attempting recovery...");
            HAL_CAN_Stop(hcan);
            HAL_CAN_ResetError(hcan);
            if (HAL_CAN_Start(hcan) != HAL_OK)
            {
                DEBUG_SEND("CAN Error: HAL_CAN_Start failed post-error. State: ERROR.");
                prv_can_current_state = CAN_STATE_ERROR;
            }
            else
            {
                if (prv_can_hal_enable_notifications() != HAL_OK)
                {
                    DEBUG_SEND("CAN Error: Failed to re-enable notifications post-error. State: ERROR.");
                    prv_can_current_state = CAN_STATE_ERROR;
                }
                else
                {
                    DEBUG_SEND("CAN: HAL recovery successful. Resumed.");
                    prv_can_current_state = CAN_STATE_READY;
                }
            }
        }
        else
        {
            HAL_CAN_ResetError(hcan);
        }
    }
}

static void prv_can_rx_convert_bytes_to_union(can_data_union_t *data_union, const uint8_t *bytes, uint8_t dlc)
{
    memset(data_union, 0, sizeof(can_data_union_t));
    for (uint8_t i = 0; i < dlc && i < 8; ++i)
    {
        data_union->bytes[i] = bytes[i];
    }
}

static bool prv_can_rx_get_message(CAN_HandleTypeDef *hcan_ptr, CAN_RxHeaderTypeDef *rx_hdr, uint8_t *data_buf)
{
    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, rx_hdr, data_buf) != HAL_OK)
    {
        DEBUG_SEND("CAN RX Error: GetRxMessage failed (Error: 0x%08lX)", HAL_CAN_GetError(hcan_ptr));
        prv_can_total_error_count++;
        return false;
    }
    prv_can_last_msg_rx_time = HAL_GetTick();
    if (g_debug_enabled)
    {
        char dbg_buf[CAN_TX_DEBUG_BUFFER_SIZE];
        prv_can_format_data_for_debug(dbg_buf, data_buf, rx_hdr->DLC);
        DEBUG_SEND("CAN RX (ID: 0x%0*lX, DLC: %u, %s) Data: %s",
                   (rx_hdr->IDE == CAN_ID_EXT ? 8 : 3),
                   (rx_hdr->IDE == CAN_ID_EXT ? rx_hdr->ExtId : rx_hdr->StdId),
                   rx_hdr->DLC,
                   (rx_hdr->IDE == CAN_ID_EXT ? "Ext" : "Std"),
                   dbg_buf);
    }
    return true;
}

static bool prv_can_rx_validate_dlc(const CAN_RxHeaderTypeDef *rx_hdr, uint8_t min_dlc)
{
    if (rx_hdr->DLC < min_dlc)
    {
        DEBUG_SEND("CAN RX Error: ID 0x%08lX DLC %u < min %u", (rx_hdr->IDE == CAN_ID_EXT ? rx_hdr->ExtId : (uint32_t)rx_hdr->StdId), rx_hdr->DLC, min_dlc);
        return false;
    }
    return true;
}

static void prv_can_rx_process_header_msg(uint8_t sensor_idx, const CAN_RxHeaderTypeDef *rx_hdr, const uint8_t *data)
{
    if (!prv_can_rx_validate_dlc(rx_hdr, 8))
    {
        return;
    }

    radar_data_t    *radar = &prv_radar_sensor_data_array[sensor_idx];
    can_data_union_t d_union;
    prv_can_rx_convert_bytes_to_union(&d_union, data, rx_hdr->DLC);

    radar->totalPacketLength = d_union.words[0];
    radar->frameNumber       = d_union.words[1];
    radar->numDetPoints      = (radar->totalPacketLength > MAX_RADAR_DETECTED_POINTS) ? MAX_RADAR_DETECTED_POINTS : (uint8_t)radar->totalPacketLength;
    radar->pointIndex        = 0;
    radar->maxSNR            = 0.0f;
    radar->new_data_ready    = false;

    prv_sensor_reset_attempt_counts_array[sensor_idx] = 0;
    prv_sensor_hw_status_array[sensor_idx]            = RADAR_HW_STATUS_CHIRPING;
    radar->status                                     = RADAR_HW_STATUS_CHIRPING;
    DEBUG_SEND("CAN RX: S%u Frame %lu Header. Expect %u pts.", sensor_idx, radar->frameNumber, radar->numDetPoints);
}

static void prv_can_rx_process_object_msg(uint8_t sensor_idx, const CAN_RxHeaderTypeDef *rx_hdr, const uint8_t *data)
{
    if (!prv_can_rx_validate_dlc(rx_hdr, 8))
    {
        return;
    }

    radar_data_t *radar = &prv_radar_sensor_data_array[sensor_idx];
    if (radar->pointIndex >= radar->numDetPoints || radar->pointIndex >= MAX_RADAR_DETECTED_POINTS)
    {
        if (radar->pointIndex >= MAX_RADAR_DETECTED_POINTS)
        {
            radar->numDetPoints = 0;
        }
        return;
    }

    can_data_union_t d_union;
    prv_can_rx_convert_bytes_to_union(&d_union, data, rx_hdr->DLC);
    radar->detectedPoints[radar->pointIndex][0] = d_union.floats[0];
    radar->detectedPoints[radar->pointIndex][1] = d_union.floats[1];
    radar->pointIndex++;

    if (radar->pointIndex == radar->numDetPoints)
    {
        radar->new_data_ready = true;
        DEBUG_SEND("CAN RX: S%u Frame %lu All %u pts recvd.", sensor_idx, radar->frameNumber, radar->numDetPoints);
        prv_sensor_hw_status_array[sensor_idx] = RADAR_HW_STATUS_CHIRPING;
        radar->status                          = RADAR_HW_STATUS_CHIRPING;
    }
}

static void prv_can_rx_process_status_msg(uint8_t sensor_idx, const CAN_RxHeaderTypeDef *rx_hdr, const uint8_t *data)
{
    if (!prv_can_rx_validate_dlc(rx_hdr, 1))
    {
        return;
    }
    radar_hw_status_t new_hw_status = (radar_hw_status_t)data[0];
    if (new_hw_status >= RADAR_HW_STATUS_COUNT)
    {
        DEBUG_SEND("CAN RX Error: S%u invalid status code %u", sensor_idx, new_hw_status);
        return;
    }
    if (prv_sensor_hw_status_array[sensor_idx] != new_hw_status)
    {
        prv_sensor_hw_status_array[sensor_idx]         = new_hw_status;
        prv_radar_sensor_data_array[sensor_idx].status = new_hw_status;
        DEBUG_SEND("CAN RX: S%u HW Status -> %d.", sensor_idx, new_hw_status);
        if (g_debug_enabled)
        {
            prv_can_debug_print_radar_hw_status(new_hw_status);
        }
    }
    prv_sensor_reset_attempt_counts_array[sensor_idx] = 0;
}

static void prv_can_rx_process_version_msg(uint8_t sensor_idx, const CAN_RxHeaderTypeDef *rx_hdr, const uint8_t *data)
{
    if (!prv_can_rx_validate_dlc(rx_hdr, 3))
    {
        return;
    }
    prv_radar_sensor_data_array[sensor_idx].version.major = data[0];
    prv_radar_sensor_data_array[sensor_idx].version.minor = data[1];
    prv_radar_sensor_data_array[sensor_idx].version.sub   = data[2];
    DEBUG_SEND("CAN RX: S%u Ver: %u.%u.%u", sensor_idx, data[0], data[1], data[2]);
    prv_sensor_reset_attempt_counts_array[sensor_idx] = 0;
}

static void prv_can_debug_print_radar_hw_status(radar_hw_status_t status)
{
    if (status < RADAR_HW_STATUS_COUNT && RADAR_HW_TO_SYSTEM_STATUS_MAP[status].description)
    { // Check array bounds
        DEBUG_SEND("  HW Status Detail: %s", RADAR_HW_TO_SYSTEM_STATUS_MAP[status].description);
    }
    else
    {
        DEBUG_SEND("  HW Status Detail: Unknown value %d", status);
    }
}

static uint8_t prv_can_get_sensor_idx_from_id(uint32_t can_id)
{
    for (uint8_t i = 0; i < MAX_SENSORS; ++i)
    {
        // Check against all known message ID patterns for this sensor
        if (can_id == CAN_MSG_ID_HEADER_SENSOR(i) || can_id == CAN_MSG_ID_OBJECT_SENSOR(i) || can_id == CAN_MSG_ID_PROFILE_SENSOR(i) || // If used
            can_id == CAN_MSG_ID_STATUS_SENSOR(i) || can_id == CAN_MSG_ID_VERSION_SENSOR(i) || can_id == CAN_CMD_ID_SENSOR(i) /* For command ACKs */)
        {
            return i;
        }
    }
    return INVALID_SENSOR_INDEX;
}

static void prv_can_calculate_snr_for_sensor_data(uint8_t sensor_idx, radar_data_t *radar_data)
{
    if (radar_data->numDetPoints == 0)
    {
        radar_data->maxSNR                           = 0.0f;
        prv_sensor_signal_strength_array[sensor_idx] = 0;
        return;
    }
    float   max_amp = 0.0f, sum_amp = 0.0f;
    uint8_t N = 0;
    for (uint8_t k = 0; k < radar_data->numDetPoints && k < MAX_RADAR_DETECTED_POINTS; ++k)
    {
        float x   = radar_data->detectedPoints[k][0];
        float y   = radar_data->detectedPoints[k][1];
        float amp = sqrtf(x * x + y * y);
        if (amp > max_amp)
        {
            max_amp = amp;
        }
        sum_amp += amp;
        N++;
    }
    if (N > 0)
    {
        float avg_amp                                = sum_amp / N;
        radar_data->maxSNR                           = (avg_amp > 0.001f) ? (max_amp / avg_amp) : (max_amp > 0 ? 999.0f : 0.0f); // Simplified SNR
        float scaled_sig                             = radar_data->maxSNR * 2.55f;
        prv_sensor_signal_strength_array[sensor_idx] = (scaled_sig > 255.f) ? 255 : (uint8_t)scaled_sig;
    }
    else
    {
        radar_data->maxSNR                           = 0.0f;
        prv_sensor_signal_strength_array[sensor_idx] = 0;
    }
}
