#include "main.h"
#include "vmt_device.h"
#include "mti_can.h"
#include "mti_radar_types.h" // Add this line for radar types

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
    STEP_START      = 0,
    STEP_VER_SYNC   = 1,
    STEP_WATER_SYNC = 2,
    STEP_IMU_SYNC   = 3,
    STEP_IMU_TEST   = 4,
    STEP_RADAR      = 5, // Updated name
    STEP_TEMP       = 6,
    STEP_VOID       = 7, // Add void step
    STEP_FINISH     = 8
} init_step_t;

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
bool                system_is_operational_mode(void);

// Add forward declarations for diagnostic functions
static void debug_init_status(void);
static void debug_can_diagnostics(void);
static void debug_radar_diagnostics(void);
static void debug_void_diagnostics(void);

// Add missing declarations
extern bool              debug;
extern bool              initialised;
extern bool              water_synced;
extern radar_hw_status_t radar_status;
extern status_t          module_status;
extern bool              version_sent;
extern init_step_t       init_step;
extern uint8_t           retries_ver;
extern uint8_t           retries_water;
extern uint8_t           retries_imu;
extern uint8_t           retries_radar;
extern uint8_t           state;
extern uint32_t          keepalive_timer;
