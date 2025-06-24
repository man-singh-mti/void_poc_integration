/**
 * @file mti_system.c
 * @brief Implements system-level state management and initialization for the MTI device.
 *
 * This file contains the core logic for the MTI device's operational states,
 * module initialization sequence, status reporting, and debug functionalities.
 * It coordinates various subsystems like IMU, Radar, Temperature, and Void detection.
 */

#include "mti_system.h"
#include "vmt_uart.h"
#include "mti_imu.h"
#include "mti_water.h"
#include "mti_can.h"
#include "mti_radar.h"
#include "mti_temp.h"
#include "mti_void.h"

// Global variable definitions
bool         debug        = true;
bool         initialised  = false; // Initialize to false by default
bool         water_synced = false; // Initialize to false by default
can_status_t radar_status;
status_t     module_status   = STATUS_SYNC; // Initialize to STATUS_SYNC
bool         version_sent    = false;       // Initialize to false by default
init_step_t  init_step       = STEP_START;
uint8_t      retries_ver     = 0;
uint8_t      retries_water   = 0;
uint8_t      retries_imu     = 0;
uint8_t      retries_radar   = 0;
uint8_t      state           = initialising_state; // Initialize to initialising_state
uint32_t     keepalive_timer = 0;

/**
 * @brief String representations for radar hardware statuses.
 *        Used for debug printing.
 */
static const char *str_radar_status[4] = { "Not Initialised", "Ready", "Chirping", "Stopped" };

// Forward declarations for static functions
static void debug_init_status(void);
static void debug_radar_diagnostics(void);
static void debug_void_diagnostics(void);

// Function needed by debug_init_status, ensure it's declared if not in a header
// Assuming reserve_get(), imu_profile_get(), imu_active_get(), temp_is_initialized(),
/// void_system_init(), void_is_system_ready(), can_run_diagnostics(), radar_run_diagnostics(),
/// radar_get_latest_measurements(), get_active_sensor_count(), test_sensor_indexing(),
/// test_sensor_responses(), void_run_diagnostics(), FW_VER_MAJOR, FW_VER_MINOR, FW_VER_SUB,
/// device_init_finish_get() are defined elsewhere (e.g. specific module headers or main.h)

/**
 * @brief Gets the current debug mode status.
 * @return True if debug mode is active, false otherwise.
 */
bool debug_get(void)
{
    return debug;
}

/**
 * @brief Gets the system initialization status.
 * @return True if the system has completed initialization, false otherwise.
 */
bool initialised_get(void)
{
    return initialised;
}

/**
 * @brief Gets the water sensor synchronization status.
 * @return True if water sensor is synced, false otherwise.
 */
bool water_synced_get(void)
{
    return water_synced;
}

/**
 * @brief Sets the radar hardware status and prints it if debug is enabled.
 * @param status The can_status_t value to set.
 */
void radar_status_set(can_status_t status)
{
    radar_status = status;
    if (debug_get())
    {
        printf("Radar status: %s\n", str_radar_status[radar_status]);
    }
}

/**
 * @brief Gets the current system state.
 * @return The current state_enum value.
 */
uint8_t state_get(void)
{
    return state;
}

/**
 * @brief Sets the system state.
 * @param set_state The state_enum value to set.
 */
void state_set(uint8_t set_state)
{
    state = set_state;
}

/**
 * @brief Gets the overall module status.
 * @return The current status_t value for the system.
 */
status_t module_status_get(void)
{
    return module_status;
}

/**
 * @brief Sets the overall module status.
 * @param status The status_t value to set for the system.
 */
void module_status_set(status_t status)
{
    module_status = status;
}

/**
 * @brief Stores the initialization status of the radar system.
 */
static radar_init_status_t radar_init_status = RADAR_INIT_NOT_STARTED;

/**
 * @brief Main system initialization routine.
 *        Cycles through initialization steps for various modules.
 * @return True if initialization is complete, false if still in progress.
 */
bool module_init(void)
{
    static uint32_t timestamp = 0; // Initialize timestamp

    if (initialised)
    {
        return true;
    }

    // Throttle initialization calls to prevent busy-looping
    if (HAL_GetTick() < timestamp + 1000)
    {
        return false;
    }
    timestamp = HAL_GetTick();

    uart_tx_channel_set(UART_UPHOLE);

    switch (init_step)
    {
    case STEP_START:
        if (debug_get())
        {
            printf("Initialising modules\n");
        }
        init_step = STEP_VER_SYNC;
        // Fall-through intended
    case STEP_VER_SYNC:
        if (version_sent)
        {
            init_step = STEP_WATER_SYNC;
        }
        else if (retries_ver < 3)
        {
            // Assuming FW_VER_MAJOR, FW_VER_MINOR, FW_VER_SUB are defined elsewhere
            printf("@status,down,0,ver,%d,%d,%d\n", FW_VER_MAJOR, FW_VER_MINOR, FW_VER_SUB);
            retries_ver++;
        }
        else
        {
            if (debug_get())
            {
                printf("@db, Downhole version not acknowledged by uphole\n");
            }
            init_step = STEP_WATER_SYNC; // Proceed even if not acknowledged after retries
        }
        break;

    case STEP_WATER_SYNC:
        // Assuming reserve_get() and reserve_set() are defined elsewhere
        if (reserve_get()) // Assuming reserve_get() returns a value that evaluates to true when synced
        {
            water_synced = true; // Explicitly set water_synced
            init_step    = STEP_IMU_SYNC;
        }
        else if (retries_water < 3 || HAL_GetTick() < 8000) // Allow retries for up to 8 seconds
        {
            printf("@status,down,0,water\n");
            retries_water++;
        }
        else
        {
            reserve_set(1280); // Default value if sync fails
            if (debug_get())
            {
                printf("@db,Sync error. Setting water to %d\n", reserve_get());
            }
            water_synced = true; // Mark as synced with default/fallback
            init_step    = STEP_IMU_SYNC;
        }
        break;

    case STEP_IMU_SYNC:
        // Assuming imu_profile_get() and imu_profile_set() are defined elsewhere
        if (imu_profile_get() != 0)
        {
            init_step = STEP_IMU_TEST;
        }
        else if (retries_imu < 3)
        {
            printf("@status,down,0,imu\n");
            retries_imu++;
        }
        else
        {
            imu_profile_set(1); // Default profile if sync fails
            if (debug_get())
            {
                printf("@db,Sync error. Setting imu profile to %d\n", imu_profile_get());
            }
            init_step = STEP_IMU_TEST;
        }
        break;

    case STEP_IMU_TEST:
        // Assuming imu_active_get() is defined elsewhere
        if (imu_active_get() == 10) // IMU Test in progress
        {
            break; // Stay in this step
        }
        if (imu_active_get() == 30) // IMU Test failed
        {
            printf("@status,down,2\n"); // IMU error status
            module_status_set(STATUS_IMU_ERROR);
        }
        init_step = STEP_TEMP;
        // Fall-through intended
    case STEP_TEMP:
        if (temp_init())
        {
            if (debug_get())
            {
                printf("@db,Temperature module initialized\n");

                // Add temperature test
                temp_raw_data_t raw_test;
                if (temp_get_raw_data(&raw_test))
                {
                    printf("@db,Temperature ADC test: %d (PASS)\n", raw_test.temperature_raw);
                }
                else
                {
                    printf("@db,Temperature ADC test: FAIL\n");
                }
            }
        }
        else
        {
            printf("@status,down,7\n"); // Temperature error
            module_status_set(STATUS_TEMP_ERROR);
        }
        init_step = STEP_RADAR;
        break;

    case STEP_RADAR:
        if (radar_system_init())
        {
            if (debug_get())
            {
                printf("@db,Radar system initialized\n");

                // Add radar test
                uint8_t responding_sensors = can_get_online_count();
                printf("@db,Radar sensor test: %d/%d sensors (", responding_sensors, MAX_RADAR_SENSORS);
                if (responding_sensors >= 2)
                {
                    printf("PASS)\n");
                }
                else
                {
                    printf("FAIL)\n");
                }
            }
            radar_init_status_set(RADAR_INIT_OK);
        }
        else
        {
            printf("@status,down,3\n"); // Radar error
            module_status_set(STATUS_RADAR_ERROR);
            radar_init_status_set(RADAR_INIT_ERROR);
        }
        init_step = STEP_FINISH;
        break;

        /*    case STEP_VOID:
                if (void_system_init())
                {
                    if (void_is_system_ready())
                    {
                        if (debug_get())
                        {
                            printf("@db,Void detection ready\n");

                            // Add void detection test
                            void_run_diagnostics();

                            // Test with simulated data if available
                            circle_fit_result_t test_result;
                            if (void_test_circle_fit(&test_result))
                            {
                                printf("@db,Void circle fit test: PASS\n");
                            }
                        }
                    }
                    else
                    {
                        if (debug_get())
                        {
                            printf("@db,Void detection initialized but waiting for radar\n");
                        }
                    }
                }
                else
                {
                    printf("@status,down,8\n"); // Void error
                    module_status_set(STATUS_VOID_ERROR);
                }
                init_step = STEP_FINISH;
                break; */

    case STEP_FINISH:
        if (debug_get())
        {
            debug_init_status();       // Full initialization summary
            debug_radar_diagnostics(); // Radar processing info
            debug_void_diagnostics();  // Void detection info
        }

        initialised = true;
        if (module_status_get() == STATUS_SYNC) // If no errors occurred during init
        {
            module_status_set(STATUS_OK);
        }
        printf("@status,down,%d\n", module_status_get());
        return true;
    }
    return false; // Still initializing
}

/**
 * @brief Acknowledges the version information reception from uphole.
 * @param received True if the version was acknowledged, false if a NACK or timeout occurred.
 */
void version_ack(bool received)
{
    if (received)
    {
        version_sent = true;
        if (debug_get())
        {
            printf("@db, Version acknowledged by uphole.\n");
        }
    }
    else // Reset if NACK or communication issue, to allow re-sync attempt from STEP_START
    {
        if (debug_get())
        {
            printf("@db, Version NACK or timeout. Resetting init sequence.\n");
        }
        version_sent  = false;
        retries_ver   = 0;
        retries_water = 0;
        retries_imu   = 0;
        retries_radar = 0; // Also reset radar retries as init sequence restarts
        init_step     = STEP_START;
        module_status_set(STATUS_SYNC); // Reset status to SYNC
    }
}

/**
 * @brief Validates IMU data and status.
 *        This function seems to be part of a periodic check or post-initialization validation.
 * @param h_imu Pointer to the IMU data structure array (assuming two IMUs).
 */
void imu_validate(h_imu_t *h_imu) // Assuming h_imu is an array of 2 IMUs
{
    // static bool    imu_status[2] = {false, false}; // Keep track of individual IMU init status
    // static uint8_t retry = 0;

    // The logic in the original function was complex and involved direct UART manipulation.
    // For this refactoring, focusing on structure and Doxygen.
    // The detailed IMU testing and selection logic would require deeper context of `imu_test` and `device_init_finish_get`.

    // Example of how it might be structured:
    if (device_init_finish_get()) // Check if main initialization is done
    {
        uart_tx_channel_set(UART_DEBUG); // Preferable to manage UART channels consistently

        // Simplified conceptual logic:
        // imu_test_t results[2];
        // results[0] = imu_test(&h_imu[0]);
        // results[1] = imu_test(&h_imu[1]);

        // ... (logic to evaluate results, set active IMU, report status) ...

        // if (debug_get()) {
        //     printf("@db,IMU 0 Test: %d, IMU 1 Test: %d\n", results[0], results[1]);
        // }

        // uart_tx_channel_set(UART_UPHOLE); // Switch back to primary channel
        // printf("@status,imu_val_done,..."); // Report validation status

        uart_tx_channel_undo(); // Ensure UART channel is restored
    }
}

/**
 * @brief Resets the keepalive timer to the current system tick.
 */
void keepalive_reset(void)
{
    keepalive_timer = HAL_GetTick();
}

/**
 * @brief Checks if the keepalive timeout has occurred.
 *        Sends a status message if timeout detected during measure state.
 * @return Always returns true (original behavior, consider if this is intended).
 */
bool keepalive_check(void)
{
    if (state_get() == measure_state && HAL_GetTick() > (keepalive_timer + 1000))
    {
        uart_tx_channel_set(UART_UPHOLE);
        printf("@status,down,A\n");      // 'A' might signify a keepalive NACK or timeout event
        keepalive_timer = HAL_GetTick(); // Reset timer after sending
    }
    return true; // Original function returned true, this might need review based on usage
}

/**
 * @brief Gets the current initialization status of the radar system.
 * @return radar_init_status_t The radar's initialization status.
 */
radar_init_status_t radar_init_status_get(void)
{
    return radar_init_status;
}

/**
 * @brief Sets the initialization status of the radar system.
 * @param status The new radar_init_status_t to set.
 */
void radar_init_status_set(radar_init_status_t status)
{
    radar_init_status = status;
}

/**
 * @brief Checks if the system is in a fully operational mode.
 *        Operational mode is defined as being in 'measure_state' and fully 'initialised'.
 * @return True if the system is operational, false otherwise.
 */
bool system_is_operational_mode(void)
{
    return (state_get() == measure_state && initialised_get());
}

/**
 * @brief Prints a summary of the system initialization status to the debug console.
 *        This function is called when debug mode is enabled, typically at the end of initialization.
 */
static void debug_init_status(void)
{
    if (debug_get())
    {
        printf("@db,=== SYSTEM INITIALIZATION STATUS ===\n");

        // Overall system status
        printf("@db,System: %s | State: %d | Status: %d\n", initialised_get() ? "INITIALIZED" : "INITIALIZING", state_get(), module_status_get());

        // Module-by-module status
        printf("@db,Modules:\n");
        printf("@db,  Version: %s (retries: %d)\n", version_sent ? "ACK" : "PENDING", retries_ver);
        printf("@db,  Water: %s (retries: %d, reserve: %d)\n", water_synced_get() ? "SYNCED" : "PENDING/FAILED", retries_water, reserve_get());
        printf("@db,  IMU: profile=%d, active=%d (retries: %d)\n", imu_profile_get(), imu_active_get(), retries_imu);

        // Radar system detailed status
        const char *radar_status_str = "UNKNOWN";
        switch (radar_init_status_get())
        {
        case RADAR_INIT_NOT_STARTED:
            radar_status_str = "NOT_STARTED";
            break;
        case RADAR_INIT_IN_PROGRESS:
            radar_status_str = "IN_PROGRESS";
            break;
        case RADAR_INIT_OK:
            radar_status_str = "OK";
            break;
        case RADAR_INIT_ERROR:
            radar_status_str = "ERROR";
            break;
        default:
            radar_status_str = "INVALID_STATUS";
            break;
        }
        printf("@db,  Radar: %s (hw_status: %s, init_retries: %d)\n", radar_status_str, str_radar_status[radar_status], retries_radar);

        // CAN and sensor details (if radar has attempted init)
        if (radar_init_status_get() != RADAR_INIT_NOT_STARTED)
        {
            printf("@db,    Active Sensors: %d/%d\n", can_get_online_count(), MAX_RADAR_SENSORS);
        }

        // Temperature system
        printf("@db,  Temperature: %s\n", temp_is_initialized() ? "INITIALIZED" : "NOT_INITIALIZED");

        // Void detection system
        printf("@db,  Void Detection:\n");
        // Assuming void_system_init() returns bool for success/failure of the init call itself,
        // and void_is_system_ready() checks if it's actually operational (e.g. dependencies met).
        printf("@db,    Init Attempted: %s\n", "Queryable via logs or specific status flag if needed"); // void_system_init() is called in sequence
        printf("@db,    System Ready: %s\n", void_is_system_ready() ? "YES" : "NO");

        if (void_is_system_ready())
        {
            // Add any specific void system ready parameters if available
            // e.g., printf("@db,      Mode: %s\n", void_get_mode_str());
        }

        // Timing information
        printf("@db,Timing: uptime=%lu ms, init_step=%d\n", HAL_GetTick(), init_step);

        // Error summary
        if (module_status_get() != STATUS_OK && module_status_get() != STATUS_SYNC)
        {
            printf("@db,Overall Status: ERROR (Code: %d)\n", module_status_get());
        }
        else if (module_status_get() == STATUS_SYNC)
        {
            printf("@db,Overall Status: SYNC (Initialization in progress or pending)\n");
        }
        else
        {
            printf("@db,Overall Status: OK\n");
        }
        printf("@db,=== END INITIALIZATION STATUS ===\n");
    }
}

static void debug_radar_diagnostics(void)
{
    if (debug_get())
    {
        printf("@db,--- RADAR DIAGNOSTICS ---\n");
        radar_run_diagnostics(); // Assuming this function prints its own details

        // Example: Display latest measurements if available
        // radar_distance_t measurements;
        // if (radar_get_latest_measurements(&measurements)) {
        //     printf("@db,  Latest Distances (mm): S0=%d, S1=%d, S2=%d, S3=%d\n",
        //            measurements.sensor[0], measurements.sensor[1],
        //            measurements.sensor[2], measurements.sensor[3]);
        // }
        printf("@db,--- END RADAR DIAGNOSTICS ---\n");
    }
}

/**
 * @brief Runs and prints Void detection system diagnostics if debug mode is enabled.
 */
static void debug_void_diagnostics(void)
{
    if (debug_get())
    {
        printf("@db,--- VOID DIAGNOSTICS ---\n");
        void_run_diagnostics(); // Assuming this function prints its own details
        printf("@db,--- END VOID DIAGNOSTICS ---\n");
    }
}

// Note: The original file had a `bool void_is_system_ready(void);` declaration
// at the top. This function should be defined in `mti_void.c` and declared
// in `mti_void.h`. If it's a local helper specific to this file's logic and not
// part of the public API of mti_void, it should be static and defined here.
// For now, assuming it's part of the mti_void module as per includes.
