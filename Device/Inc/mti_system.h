/**
 * @file mti_system.h
 * @brief System-level definitions, state management, and initialization for the MTI device.
 *
 * This file contains the core enumerations, status types, and function
 * declarations that govern the overall behavior and initialization sequence
 * of the MTI device. It includes state machine definitions, module status
 * tracking, and hardware-specific configurations.
 */

#ifndef MTI_SYSTEM_H
#define MTI_SYSTEM_H

#include "main.h"
#include "vmt_device.h"
#include "mti_can.h"
#include "mti_radar.h" // For can_status_t, radar_init_status_t

#define HW_VER 3
#if HW_VER >= 3
#define PCB_CANBUS
#endif

/**
 * @brief Defines the possible operational states of the MTI system.
 */
enum state_enum
{
    initialising_state, ///< System is currently initializing modules.
    stopped_state,      ///< System is initialized but not actively measuring.
    measure_state,      ///< System is actively performing measurements.
    saving_state,       ///< System is in the process of saving data.
    debug_log_state     ///< System is in a debug logging mode.
};

/**
 * @brief Defines the possible status codes for modules and the overall system.
 */
typedef enum
{
    STATUS_SYNC,                ///< System or module is synchronizing.
    STATUS_OK,                  ///< System or module is operating normally.
    STATUS_IMU_ERROR,           ///< An error occurred with the IMU.
    STATUS_RADAR_ERROR,         ///< An error occurred with the Radar.
    STATUS_TEMP_ERROR,          ///< An error occurred with the Temperature sensor.
    STATUS_VOID_ERROR,          ///< An error occurred with the Void detection system.
    STATUS_VOID_DETECTION_ERROR ///< A void-specific detection error occurred.
} status_t;

/**
 * @brief Defines the steps in the system initialization sequence.
 */
typedef enum step_
{
    STEP_START      = 0, ///< Initial step, beginning of initialization.
    STEP_VER_SYNC   = 1, ///< Version synchronization step.
    STEP_WATER_SYNC = 2, ///< Water level/sensor synchronization step.
    STEP_IMU_SYNC   = 3, ///< IMU synchronization step.
    STEP_IMU_TEST   = 4, ///< IMU self-test step.
    STEP_RADAR      = 5, ///< Radar system initialization step.
    STEP_TEMP       = 6, ///< Temperature sensor initialization step.
    STEP_VOID       = 7, ///< Void detection system initialization step.
    STEP_FINISH     = 8  ///< Final step, initialization complete.
} init_step_t;

// Function Declarations

/**
 * @brief Validates the IMU data.
 * @param h_imu Pointer to the IMU data structure.
 */
void imu_validate(h_imu_t *h_imu);

/**
 * @brief Gets the current debug mode status.
 * @return True if debug mode is active, false otherwise.
 */
bool debug_get(void);

/**
 * @brief Gets the system initialization status.
 * @return True if the system has completed initialization, false otherwise.
 */
bool initialised_get(void);

/**
 * @brief Gets the water sensor synchronization status.
 * @return True if water sensor is synced, false otherwise.
 */
bool water_synced_get(void);

/**
 * @brief Gets the current system state.
 * @return The current state_enum value.
 */
uint8_t state_get(void);

/**
 * @brief Sets the system state.
 * @param state The state_enum value to set.
 */
void state_set(uint8_t state);

/**
 * @brief Gets the overall module status.
 * @return The current status_t value for the system.
 */
status_t module_status_get(void);

/**
 * @brief Sets the radar hardware status.
 * @param status The can_status_t value to set.
 */
void radar_status_set(can_status_t status);

/**
 * @brief Sets the overall module status.
 * @param status The status_t value to set for the system.
 */
void module_status_set(status_t status);

/**
 * @brief Initializes all system modules.
 * @return True if initialization was successful or is already complete, false if in progress.
 */
bool module_init(void);

/**
 * @brief Acknowledges the version information reception.
 * @param received True if the version was acknowledged by the uphole, false otherwise.
 */
void version_ack(bool received);

/**
 * @brief Checks the keepalive timer.
 * @return True if the keepalive check passed (or is not applicable), false if timeout occurred.
 */
bool keepalive_check(void);

/**
 * @brief Resets the keepalive timer.
 */
void keepalive_reset(void);

/**
 * @brief Gets the radar initialization status.
 * @return The current radar_init_status_t value.
 */
radar_init_status_t radar_init_status_get(void);

/**
 * @brief Sets the radar initialization status.
 * @param status The radar_init_status_t value to set.
 */
void radar_init_status_set(radar_init_status_t status);

/**
 * @brief Checks if the system is in an operational mode (measure state and initialized).
 * @return True if the system is operational, false otherwise.
 */
bool system_is_operational_mode(void);

/**
 * @brief Wrapper for void system test (called from device_process)
 */
void debug_void_system_test_wrapper(void);

// Forward declarations for static diagnostic functions (implementation in .c file)
static void debug_init_status(void);
static void debug_radar_diagnostics(void);
static void debug_void_diagnostics(void);

// External variable declarations

/// @brief Global flag for debug mode. True if active, false otherwise.
extern bool debug;
/// @brief Global flag indicating if system initialization is complete.
extern bool initialised;
/// @brief Global flag indicating if water sensor synchronization is complete.
extern bool water_synced;
/// @brief Current hardware status of the radar.
extern can_status_t radar_status;
/// @brief Overall status of the system modules.
extern status_t module_status;
/// @brief Flag indicating if the version information has been sent.
extern bool version_sent;
/// @brief Current step in the system initialization sequence.
extern init_step_t init_step;
/// @brief Retry counter for version synchronization.
extern uint8_t retries_ver;
/// @brief Retry counter for water sensor synchronization.
extern uint8_t retries_water;
/// @brief Retry counter for IMU synchronization.
extern uint8_t retries_imu;
/// @brief Retry counter for radar initialization.
extern uint8_t retries_radar;
/// @brief Current operational state of the system.
extern uint8_t state;
/// @brief Timer for keepalive mechanism.
extern uint32_t keepalive_timer;

#endif /* MTI_SYSTEM_H */
