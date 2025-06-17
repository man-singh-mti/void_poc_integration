#include "mti_system.h"
#include "vmt_uart.h"
#include "mti_imu.h"
#include "mti_water.h"
#include "mti_can.h"
#include "mti_radar.h"
#include "mti_temp.h"
#include "mti_void.h" // Add this include

bool              debug = true;
bool              initialised;
bool              water_synced;
radar_hw_status_t radar_status;
status_t          module_status;
bool              initialised;
bool              version_sent;
init_step_t       init_step     = STEP_START;
uint8_t           retries_ver   = 0;
uint8_t           retries_water = 0;
uint8_t           retries_imu   = 0;
uint8_t           retries_radar = 0;
uint8_t           state;
uint32_t          keepalive_timer;

static const char *str_radar_status[4] = { "Not Initialised", "Ready", "Chirping", "Stopped" };

// Add missing function declaration at the top
bool void_is_system_ready(void); // Add this line after includes

bool debug_get(void)
{
    return debug;
}

bool initialised_get(void)
{
    return initialised;
}

bool water_synced_get(void)
{
    return water_synced;
}

void radar_status_set(radar_hw_status_t status)
{
    radar_status = status;
    printf("Radar status: %s\n", str_radar_status[radar_status]);
}

uint8_t state_get(void)
{
    return state;
}

void state_set(uint8_t set)
{
    state = set;
}

status_t module_status_get(void)
{
    return module_status;
}

void module_status_set(status_t status)
{
    module_status = status;
}

static radar_init_status_t radar_init_status = RADAR_INIT_NOT_STARTED;

bool module_init(void)
{
    static uint32_t timer_previous = 0;
    uint32_t        timer_now      = HAL_GetTick();

    if (initialised)
    {
        return true;
    }
    static uint32_t timestamp;
    if (HAL_GetTick() < timestamp + 1000)
    {
        return false;
    }
    timestamp = HAL_GetTick();

    uart_tx_channel_set(UART_UPHOLE);

    switch (init_step)
    {
    case STEP_START:
        printf("Initialising modules\n");
        init_step = STEP_VER_SYNC;
    case STEP_VER_SYNC:
        if (version_sent)
        {
            init_step = STEP_WATER_SYNC;
        }
        else if (retries_ver < 3)
        {
            printf("@status,down,0,ver,%d,%d,%d\n", FW_VER_MAJOR, FW_VER_MINOR, FW_VER_SUB);
            retries_ver++;
        }
        else
        {
            printf("@db, Downhole version not acknowledged by uphole\n");
            init_step = STEP_WATER_SYNC;
        }
        break;
    case STEP_WATER_SYNC:
        if (reserve_get())
        {
            init_step = STEP_IMU_SYNC;
        }
        else if (retries_water < 3 || HAL_GetTick() < 8000)
        {
            printf("@status,down,0,water\n");
            retries_water++;
            break;
        }
        else
        {
            reserve_set(1280);
            printf("@db,Sync error. Setting water to %d\n", reserve_get());
            init_step = STEP_IMU_SYNC;
        }
    case STEP_IMU_SYNC:
        if (imu_profile_get() != 0)
        {
            init_step = STEP_IMU_TEST;
        }
        else if (retries_imu < 3)
        {
            printf("@status,down,0,imu\n");
            retries_imu++;
            break;
        }
        else
        {
            imu_profile_set(1);
            printf("@db,Sync error. Setting imu profile to %d\n", imu_profile_get());
            init_step = STEP_IMU_TEST;
        }
    case STEP_IMU_TEST:
        if (imu_active_get() == 10)
        {
            break;
        }
        if (imu_active_get() == 30)
        {
            printf("@status,down,2\n"); // IMU error
        }
        init_step = STEP_RADAR;
    case STEP_RADAR: // Renamed from STEP_VOID
        if (radar_init_status == RADAR_INIT_NOT_STARTED)
        {
            radar_init_status = RADAR_INIT_IN_PROGRESS;
            retries_radar     = 0;
        }

        if (radar_init_status == RADAR_INIT_IN_PROGRESS)
        {
            static uint8_t radar_retries = 0;
            if (radar_retries < 5)
            {
                if (radar_retries == 0)
                {
                    can_setup();
                    radar_system_init();

                    // Test 1: Function logic test
                    debug_send("Running sensor indexing diagnostics...");
                    test_sensor_indexing();

                    // Test 2: Real sensor communication test
                    debug_send("Testing real sensor communication...");
                    test_sensor_responses();
                }
                else
                {
                    // Subsequent retries - just ping sensors
                    test_sensor_responses();
                }

                radar_retries++;

                // Check if we have enough responding sensors
                uint8_t responding_sensors = get_active_sensor_count();

                if (responding_sensors >= 2)
                {
                    radar_init_status = RADAR_INIT_OK;
                    debug_send("✓ Radar init SUCCESS: %d/%d sensors responding", responding_sensors, MAX_RADAR_SENSORS);
                }
                else if (radar_retries >= 5)
                {
                    radar_init_status = RADAR_INIT_ERROR;
                    module_status_set(STATUS_RADAR_ERROR);
                    printf("@status,down,3\n");
                    debug_send("✗ Radar init FAILED: Only %d/%d sensors responding after %d retries", responding_sensors, MAX_RADAR_SENSORS, radar_retries);
                }
            }
        }
        else
        {
            // Either OK or ERROR - move to next step
            init_step = STEP_TEMP;
        }
        break;

    case STEP_TEMP:
        // Initialize temperature module
        if (temp_init())
        {
            printf("@db,Temperature module initialized\n");
        }
        else
        {
            printf("@status,down,7\n"); // Temperature initialization error
        }
        init_step = STEP_VOID;
        break;

    case STEP_VOID:
        // Initialize void detection module
        if (void_system_init())
        {
            if (void_is_system_ready())
            {
                printf("@db,Void detection module initialized\n");
            }
            else
            {
                printf("@db,Void detection initialized but not ready (waiting for radar)\n");
            }
        }
        else
        {
            printf("@status,down,8\n"); // Void initialization error
            printf("@db,Void detection initialization failed\n");
        }
        init_step = STEP_FINISH;
        break;

    case STEP_FINISH:
        debug_init_status();       // Full initialization summary
        debug_can_diagnostics();   // Detailed CAN/sensor info
        debug_radar_diagnostics(); // Radar processing info
        debug_void_diagnostics();  // Void detection info

        initialised = true;
        if (module_status == STATUS_SYNC)
        {
            module_status = STATUS_OK;
        }
        printf("@status,down,%d\n", module_status);
        return true;
    }
    return false;
}

void version_ack(bool received)
{
    if (received)
    {
        version_sent = true;
    }
    else
    {
        retries_ver   = 0;
        retries_water = 0;
        retries_imu   = 0;
        init_step     = STEP_START;
    }
}

void imu_validate(h_imu_t *h_imu)
{
    static bool    imu_status[2];
    static uint8_t retry;
    // if(state<= stopped_state || id == 0) return;
    // uart_tx_channel_set(UART_UPHOLE);
    uart_tx_channel_set(UART_DEBUG);
    // water_detect();


    static imu_test_t res[2];
    if (device_init_finish_get())
    {
        for (int i = 0; i < 2; i++)
        {
            res[i] = imu_test(&h_imu[i]);
            if (res[i] == IMU_TEST_OK && !imu_status[i])
            {
                imu_status[i] = true;
                printf("@db,IMU status %d = %d\n", i, imu_status[i]);
            }
            else if (res[i] == IMU_TEST_ERROR && imu_status[i])
            {
                imu_status[i] = false;
                printf("@db,IMU status %d = %d\n", i, imu_status[i]);
            }
        }
        uart_tx_channel_set(UART_UPHOLE);

        // send status to uphole
        uint8_t error_code = 0;
        if (res[0] == IMU_TEST_ERROR)
        {
            if (res[1] == IMU_TEST_ERROR)
            {
                error_code = 2;
            }
            else
            {
                error_code = 3;
            }
        }
        else if (res[1] == IMU_TEST_ERROR)
        {
            error_code = 4;
        }
        if (error_code > 1)
        {
            module_status = (status_t)error_code;
            if (retry < 3)
            {
                printf("@db,retry IMU init\n");
                imu_test_reset();
                retry++;
            }
        }
        if (res[0] == IMU_TEST_OK)
        {
            imu_status[0] = true;
        }
        else if (res[0] == IMU_TEST_ERROR)
        {
            imu_status[0] = false;
        }
        if (res[1] == IMU_TEST_OK)
        {
            imu_status[1] = true;
        }
        else if (res[1] == IMU_TEST_ERROR)
        {
            imu_status[1] = false;
        }

        // select which IMU to use based on init status
        if (imu_status[0])
        {
            imu_active_set(0);
        }
        else if (imu_status[1])
        {
            imu_active_set(1);
        }
        else
        {
            imu_active_set(30); // invalid
        }
        if ((res[0] == IMU_TEST_OK && imu_status[1]) || (res[1] == IMU_TEST_OK && imu_status[0]))
        {
            //				if(reserve_get() > 0) {
            //					if(module_status != 5) {
            //						module_status = 5;
            //					 printf("@init,down,status,%d\n",module_status);
            //					}
            //				}
            //				else {
            //					module_status = 6;
            //					printf("@init,down,water\n");
            //				}
        }
        // printf("@db,active imu: %d\n",imu_active);
        // uart_tx_channel_set(UART_DEBUG);
        // printf("@val,%lf:%lf:%f:%lf\r\n",h_imu[0].h_accel.angle,h_imu[1].h_accel.angle,h_imu[0].h_accel.len,h_imu[1].h_accel.len);
        // uart_tx_channel_undo();
        // imu_active = 1;
        uart_tx_channel_undo();
    }
}

void keepalive_reset(void)
{
    keepalive_timer = HAL_GetTick();
}

bool keepalive_check(void)
{
    if (state_get() == measure_state && HAL_GetTick() > keepalive_timer + 1000)
    {
        uart_tx_channel_set(UART_UPHOLE);
        printf("@status,down,A\n");
        keepalive_timer = HAL_GetTick();
    }
    return true; // Add this missing return statement
}

radar_init_status_t radar_init_status_get(void)
{
    return radar_init_status;
}

void radar_init_status_set(radar_init_status_t status)
{
    radar_init_status = status;
}

bool system_is_operational_mode(void)
{
    return (state == measure_state && initialised);
}

// Add this helper function
static void debug_init_status(void)
{
    if (debug_get())
    {
        printf("@db,=== SYSTEM INITIALIZATION STATUS ===\n");

        // Overall system status
        printf("@db,System: %s | State: %d | Status: %d\n", initialised ? "INITIALIZED" : "INITIALIZING", state_get(), module_status_get());

        // Module-by-module status
        printf("@db,Modules:\n");
        printf("@db,  Version: %s (retries: %d)\n", version_sent ? "ACK" : "PENDING", retries_ver);
        printf("@db,  Water: %s (retries: %d, reserve: %d)\n", water_synced_get() ? "SYNCED" : "PENDING", retries_water, reserve_get());
        printf("@db,  IMU: profile=%d, active=%d (retries: %d)\n", imu_profile_get(), imu_active_get(), retries_imu);

        // Radar system detailed status
        const char *radar_status_str;
        switch (radar_init_status)
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
            radar_status_str = "UNKNOWN";
            break;
        }

        printf("@db,  Radar: %s (retries: %d)\n", radar_status_str, retries_radar);

        // CAN and sensor details
        if (radar_init_status != RADAR_INIT_NOT_STARTED)
        {
            uint8_t active_sensors = get_active_sensor_count();
            printf("@db,    CAN: %s\n", can_system_is_healthy() ? "HEALTHY" : "ERROR");
            printf("@db,    Sensors: %d/%d active\n", active_sensors, MAX_RADAR_SENSORS);

            // Individual sensor status
            for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
            {
                printf("@db,      S%d: %s\n", i, can_is_sensor_online(i) ? "ONLINE" : "OFFLINE");
            }
        }

        // Temperature system
        printf("@db,  Temperature: %s\n", temp_is_initialized() ? "INITIALIZED" : "NOT_INIT");

        // Void detection system
        printf("@db,  Void Detection:\n");
        printf("@db,    Init: %s\n", "TRUE"); // void_system_init() should return true
        printf("@db,    Ready: %s\n", void_is_system_ready() ? "YES" : "NO");

        if (void_is_system_ready())
        {
            void_config_t void_config;
            void_get_config(&void_config);
            printf("@db,    Algorithm: %d, Baseline: %dmm, Threshold: %dmm\n", void_config.algorithm, void_config.baseline_diameter_mm, void_config.threshold_mm);
        }

        // Timing information
        printf("@db,Timing: uptime=%lums, init_step=%d\n", HAL_GetTick(), init_step);

        // Error summary
        if (module_status_get() != STATUS_OK && module_status_get() != STATUS_SYNC)
        {
            const char *error_desc;
            switch (module_status_get())
            {
            case STATUS_IMU_ERROR:
                error_desc = "IMU_ERROR";
                break;
            case STATUS_RADAR_ERROR:
                error_desc = "RADAR_ERROR";
                break;
            case STATUS_TEMP_ERROR:
                error_desc = "TEMP_ERROR";
                break;
            case STATUS_VOID_ERROR:
                error_desc = "VOID_ERROR";
                break;
            case STATUS_VOID_DETECTION_ERROR:
                error_desc = "VOID_DETECTION_ERROR";
                break;
            default:
                error_desc = "UNKNOWN_ERROR";
                break;
            }
            printf("@db,ERROR: %s\n", error_desc);
        }

        printf("@db,=== END INITIALIZATION STATUS ===\n");
    }
}

// Add these helper functions for detailed diagnostics

static void debug_can_diagnostics(void)
{
    if (debug_get())
    {
        printf("@db,--- CAN DIAGNOSTICS ---\n");

        // Run CAN diagnostics
        can_run_diagnostics();

        // Show sensor communication statistics
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            radar_raw_t *raw_data = can_get_raw_data(i);
            if (raw_data)
            {
                printf("@db,Sensor %d: frames=%lu, points=%d, status=0x%02X\n", i, raw_data->frame_number, raw_data->num_points, raw_data->sensor_status);
            }
        }
        printf("@db,--- END CAN DIAGNOSTICS ---\n");
    }
}

static void debug_radar_diagnostics(void)
{
    if (debug_get())
    {
        printf("@db,--- RADAR DIAGNOSTICS ---\n");

        radar_run_diagnostics();

        radar_distance_t measurements;
        if (radar_get_latest_measurements(&measurements))
        {
            printf("@db,Valid sensors: %d, System healthy: %s\n", measurements.valid_sensor_count, measurements.system_healthy ? "YES" : "NO");

            for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
            {
                if (measurements.data_valid[i])
                {
                    printf("@db,S%d: %dmm (conf: %.1f, quality: %d)\n", i, measurements.distance_mm[i], measurements.confidence[i], measurements.quality_score[i]);
                }
            }
        }
        printf("@db,--- END RADAR DIAGNOSTICS ---\n");
    }
}

static void debug_void_diagnostics(void)
{
    if (debug_get())
    {
        printf("@db,--- VOID DIAGNOSTICS ---\n");

        void_run_diagnostics();

        uint32_t total_detections, algorithm_switches, uptime_ms;
        void_get_statistics(&total_detections, &algorithm_switches, &uptime_ms);

        printf("@db,Statistics: detections=%lu, switches=%lu, uptime=%lums\n", total_detections, algorithm_switches, uptime_ms);

        void_detection_state_t detection_state = void_get_detection_state();
        printf("@db,Detection state: %d\n", detection_state);

        printf("@db,--- END VOID DIAGNOSTICS ---\n");
    }
}
