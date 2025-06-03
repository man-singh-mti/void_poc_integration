/**
 * @file    mti_can.h
 * @author  Mti Group
 * @brief   CAN interface for radar sensor communication
 * @version 1.2.0 (Updated for naming and status conventions)
 * @date    2025-05-22
 */

#ifndef MTI_CAN_H
#define MTI_CAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32f7xx_hal.h"
#include "vmt_common_defs.h" // For MAX_SENSORS, radar_hw_status_t, etc.

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_CMD_BASE      0x80U
#define CAN_CMD_START     0x00U
#define CAN_CMD_STOP      0x01U
#define CAN_CMD_CAL       0x02U
#define CAN_CMD_POWER     0x03U
#define CAN_CMD_STATUS    0x04U
#define CAN_CMD_THRESHOLD 0x05U
#define CAN_CMD_SPREAD    0x06U
#define CAN_CMD_PROFILE   0x07U
#define CAN_CMD_FOV       0x08U
#define CAN_CMD_INIT      0x09U

#define CAN_ID_HEADER_BASE  0xA0U
#define CAN_ID_OBJECT_BASE  0xA1U
#define CAN_ID_PROFILE_BASE 0xA2U
#define CAN_ID_STATUS_BASE  0xA3U
#define CAN_ID_VERSION_BASE 0xA4U

#define CAN_SENSOR_OFFSET_0 0x00U
#define CAN_SENSOR_OFFSET_1 0x10U
#define CAN_SENSOR_OFFSET_2 0x20U

#define CAN_CMD_ID_SENSOR(sensor_idx)         (CAN_CMD_BASE + (sensor_idx))
#define CAN_MSG_ID_HEADER_SENSOR(sensor_idx)  (CAN_ID_HEADER_BASE + ((sensor_idx) * 0x10U))
#define CAN_MSG_ID_OBJECT_SENSOR(sensor_idx)  (CAN_ID_OBJECT_BASE + ((sensor_idx) * 0x10U))
#define CAN_MSG_ID_PROFILE_SENSOR(sensor_idx) (CAN_ID_PROFILE_BASE + ((sensor_idx) * 0x10U))
#define CAN_MSG_ID_STATUS_SENSOR(sensor_idx)  (CAN_ID_STATUS_BASE + ((sensor_idx) * 0x10U))
#define CAN_MSG_ID_VERSION_SENSOR(sensor_idx) (CAN_ID_VERSION_BASE + ((sensor_idx) * 0x10U))

#define DEFAULT_RADAR_FILTER_ID_BASE      CAN_CMD_BASE
#define DEFAULT_RADAR_DETECTION_THRESHOLD 150
#define DEFAULT_RADAR_FOV                 100
#define DEFAULT_RADAR_OPERATING_MODE      0
#define DEFAULT_RADAR_POWER_PERCENT       100

#define CAN_ID_MSG_TYPE_MASK         0x0FU
#define CAN_ID_BASE_MASK             0xF0U
#define CAN_ID_TYPE_STATUS_DATA_BASE 0xA0U
#define CAN_ID_SENSOR_OFFSET_MASK    0x30U
#define CAN_ID_SENSOR_OFFSET_SHIFT   4
#define CAN_ID_TYPE_COMMAND_BASE     0x80U
#define CAN_ID_SENSOR_INDEX_MASK     0x0FU
#define INVALID_SENSOR_INDEX         0xFFU

#define MIN_RADAR_PACKET_LENGTH   129U
#define MAX_RADAR_DETECTED_POINTS 20

typedef enum
{
    CAN_STATE_UNINIT,
    CAN_STATE_READY,
    CAN_STATE_ERROR,
    CAN_STATE_BUSY
} can_state_t;

typedef struct
{
    float             detectedPoints[MAX_RADAR_DETECTED_POINTS][2];
    uint32_t          frameNumber;
    uint32_t          totalPacketLength; // Raw value from header, interpretation is protocol-specific
    uint8_t           numDetPoints;      // Actual number of points expected/received for the frame
    uint8_t           pointIndex;
    float             maxSNR; // Calculated/estimated SNR or signal quality
    radar_hw_status_t status; // Hardware status of this specific radar sensor
    bool              new_data_ready;
    struct
    {
        uint8_t major;
        uint8_t minor;
        uint8_t sub;
    } version;
} radar_data_t;

typedef union
{
    uint8_t  bytes[8];
    uint32_t words[2];
    float    floats[2];
} can_data_union_t;

can_state_t       can_init(void);
can_state_t       can_get_state(void);
can_state_t       can_send_to_sensor(uint8_t sensor_idx, uint8_t cmd, const uint8_t *data, size_t data_len);
can_state_t       can_request_radar_data(void); // Renamed from can_request_all_radar_data for brevity if only one type of request
uint32_t          can_get_error_count(void);
uint32_t          can_get_timeout_count(void);
uint32_t          can_get_time_since_last_msg(void);
uint8_t           can_get_radar_signal(uint8_t sensor_idx); // Renamed from can_get_sensor_signal
bool              can_handle_error(uint8_t sensor_idx, can_error_type_t error_type);
void              can_process(void);
radar_data_t     *can_get_radar_data_ptr(uint8_t sensor_idx); // Renamed from can_get_radar_data
can_state_t       can_reset_sensor(uint8_t sensor_idx);
bool              can_all_sensors_ready(void);
radar_hw_status_t can_get_sensor_hw_status(uint8_t sensor_idx); // Renamed from can_get_sensor_status

can_state_t can_send_byte(uint32_t id, uint8_t message);
can_state_t can_send_array(uint32_t id, const uint8_t *message, size_t length);
can_state_t can_send_sensor_command(uint8_t sensor_idx, uint8_t cmd); // Add this declaration

bool can_setup_all_sensors(void);
bool can_setup_sensor_filter(uint8_t sensor_idx, uint32_t filter_id); // Renamed from can_setup_sensor

typedef void (*radar_data_callback_t)(uint8_t sensor_idx, radar_data_t *radar_data_ptr); // Parameter name consistency
can_state_t can_register_data_callback(radar_data_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* MTI_CAN_H */
