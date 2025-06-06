#ifndef MTI_CAN_H
#define MTI_CAN_H

#include "stm32f7xx.h"
#include <stdbool.h>
#include "vmt_uart.h" // For debug_send function
#include "mti_void.h"

// Update for 3 sensors
#define MAX_RADAR_SENSORS         3U
#define MAX_RADAR_DETECTED_POINTS 20

// Command definitions - Updated to match design document
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

// Power level definitions
#define RADAR_POWER_OFF    0
#define RADAR_POWER_LOW    25
#define RADAR_POWER_MEDIUM 50
#define RADAR_POWER_HIGH   75
#define RADAR_POWER_MAX    100

// Field of View definitions (degrees)
#define RADAR_FOV_NARROW 60
#define RADAR_FOV_MEDIUM 90
#define RADAR_FOV_WIDE   120
#define RADAR_FOV_MAX    150

// Base IDs for different message types
#define CAN_ID_HEADER_BASE  0xA0U
#define CAN_ID_OBJECT_BASE  0xA1U
#define CAN_ID_PROFILE_BASE 0xA2U
#define CAN_ID_STATUS_BASE  0xA3U
#define CAN_ID_VERSION_BASE 0xA4U

// Sensor offsets (0x00, 0x10, 0x20 for 3 sensors)
#define CAN_SENSOR_OFFSET(sensor_idx) ((sensor_idx) * 0x10U)

// Macros to get sensor-specific CAN IDs
#define CAN_MSG_ID_HEADER_SENSOR(idx)  (CAN_ID_HEADER_BASE + CAN_SENSOR_OFFSET(idx))
#define CAN_MSG_ID_OBJECT_SENSOR(idx)  (CAN_ID_OBJECT_BASE + CAN_SENSOR_OFFSET(idx))
#define CAN_MSG_ID_PROFILE_SENSOR(idx) (CAN_ID_PROFILE_BASE + CAN_SENSOR_OFFSET(idx))
#define CAN_MSG_ID_STATUS_SENSOR(idx)  (CAN_ID_STATUS_BASE + CAN_SENSOR_OFFSET(idx))
#define CAN_MSG_ID_VERSION_SENSOR(idx) (CAN_ID_VERSION_BASE + CAN_SENSOR_OFFSET(idx))

// Base addresses for each sensor type
#define SENSOR_0_BASE_ADDR 0x00
#define SENSOR_1_BASE_ADDR 0x10
#define SENSOR_2_BASE_ADDR 0x20

// Command ID generation for specific sensors
#define CAN_CMD_SENSOR_ID(sensor_idx) (CAN_CMD_BASE + (sensor_idx))

// Radar profiles
#define RADAR_PROFILE_CAL         0
#define RADAR_PROFILE_50M_SINGLE  1
#define RADAR_PROFILE_50M_MULTI   2
#define RADAR_PROFILE_100M_SINGLE 3
#define RADAR_PROFILE_100M_MULTI  4

typedef enum
{
    RADAR_INITIALISING,
    RADAR_READY,
    RADAR_CHIRPING,
    RADAR_STOPPED,
} radar_status_t;

typedef enum
{
    RADAR_HW_INITIALISING = 0,
    RADAR_HW_READY        = 1,
    RADAR_HW_CHIRPING     = 2,
    RADAR_HW_STOPPED      = 3,
    RADAR_HW_ERROR        = 4,
    RADAR_HW_CALIBRATING  = 5
} radar_hw_status_t;

typedef union
{
    uint8_t  bytes[8];
    uint32_t words[2];
    float    floats[2];
} can_data_union_t;

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

// Multi-sensor system
typedef struct
{
    radar_data_t sensors[MAX_RADAR_SENSORS];
    uint8_t      active_sensor_count;
    uint32_t     last_message_timestamp[MAX_RADAR_SENSORS];
    bool         sensor_online[MAX_RADAR_SENSORS];
} multi_radar_system_t;

// CAN communication functions (implemented in mti_can.c)
bool    can_setup(void);
bool    can_send(uint32_t ID, uint8_t message);
bool    can_send_array(uint32_t ID, uint8_t *message, size_t length);
bool    can_send_to_sensor(uint8_t sensor_idx, uint32_t base_id, uint8_t message);
void    broadcast_to_all_sensors(uint32_t base_id, uint8_t message);
uint8_t get_sensor_index_from_can_id(uint32_t can_id);

// CAN message processing functions (implemented in mti_can.c)
void process_sensor_command(uint8_t sensor_idx, uint8_t command, can_data_union_t *data);
void process_complete_radar_frame(uint8_t sensor_idx);

// Sensor status functions (implemented in mti_can.c)
bool    is_sensor_online(uint8_t sensor_idx);
uint8_t get_active_sensor_count(void);
void    reset_sensor_data(uint8_t sensor_idx);

// Debug and test functions
void test_sensor_indexing(void);
void test_sensor_responses(void);
void debug_send(const char *format, ...);

// Global radar system instance
extern multi_radar_system_t radar_system;

#endif // MTI_CAN_H
