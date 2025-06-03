#include "stm32f7xx.h"
#include <stdbool.h>

#define CAN_CMD        0x80
#define CAN_START      0x00
#define CAN_STOP       0x01
#define CAN_CAL        0x02
#define CAN_POWER      0x03
#define CAN_STATUS     0x04
#define CAN_THRESHOLD  0x05
#define CAN_SPREAD     0x06
#define CAN_PROFILE    0x07
#define CAN_FOV        0x08
#define CAN_ID_HEADER  0xA0
#define CAN_ID_OBJECT  0xA1
#define CAN_ID_PROFILE 0xA2
#define CAN_ID_STATUS  0xA3
#define CAN_ID_VERSION 0xA4

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

bool can_setup(void);
bool can_send(uint32_t ID, uint8_t message);
bool can_send_array(uint32_t ID, uint8_t *message, size_t length);
