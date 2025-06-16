#include "main.h"
#include "vmt_device.h"
#include "mti_can.h"

#define HW_VER 3
#if HW_VER >= 3
#define PCB_CANBUS
#endif

enum state_enum
{
    initialising_state,
    stopped_state,
    measure_state,
    saving_state,
    debug_log_state
};

typedef enum
{
    STATUS_SYNC,
    STATUS_OK,
    STATUS_IMU_ERROR,
    STATUS_RADAR_ERROR, // Add this line
    STATUS_TEMP_ERROR,
    STATUS_VOID_ERROR,
    STATUS_VOID_DETECTION_ERROR // Add void-specific error
} status_t;

typedef enum step_
{
    STEP_START,      // 0
    STEP_VER_SYNC,   // 1
    STEP_WATER_SYNC, // 2
    STEP_IMU_SYNC,   // 3
    STEP_IMU_TEST,   // 4
    STEP_RADAR,      // 5
    STEP_TEMP,       // 6
    STEP_VOID,       // 7 - Add void initialization step
    STEP_FINISH,     // 8
} init_step_t;

// Add radar init status tracking
typedef enum
{
    RADAR_INIT_NOT_STARTED,
    RADAR_INIT_IN_PROGRESS,
    RADAR_INIT_OK,
    RADAR_INIT_ERROR
} radar_init_status_t;

void     imu_validate(h_imu_t *h_imu);
bool     debug_get(void);
bool     initialised_get(void);
bool     water_synced_get(void);
uint8_t  state_get(void);
void     state_set(uint8_t);
status_t module_status_get(void);
void     radar_status_set(radar_hw_status_t status);
void     module_status_set(status_t status);
bool     module_init(void);
void     version_ack(bool received);
bool     keepalive_check(void);
void     keepalive_reset(void);
// Add these missing function declarations
radar_init_status_t radar_init_status_get(void);
void                radar_init_status_set(radar_init_status_t status);
bool                system_is_operational_mode(void); // Add this function declaration
