/**
 * @file mti_can.c
 * @brief Implementation of CAN bus communication for multi-radar sensor system
 *
 * Provides functions for initializing and managing CAN communication with
 * multiple radar sensors, including command handling and data processing.
 *
 * File organization:
 * - Global variables
 * - CAN initialization functions
 * - Message transmission functions
 * - Message reception and processing
 * - Sensor status management
 * - Testing and diagnostic functions
 *
 * @author MTi Group
 * @copyright Copyright (c) 2025 MTi Group
 */

#include "can.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include "mti_radar.h"
#include "mti_void.h"
#include <string.h>

/** @brief CAN receive header structure */
CAN_RxHeaderTypeDef rxHeader;

/** @brief CAN transmit header structure */
CAN_TxHeaderTypeDef txHeader;

/** @brief Buffer for CAN received data */
uint8_t canRX[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/** @brief CAN filter configuration structure */
CAN_FilterTypeDef canfil;

/** @brief CAN transmit mailbox indicator */
uint32_t canMailbox;

/** @brief Multi-sensor system instance */
multi_radar_system_t radar_system = { 0 };

/*------------------------------------------------------------------------------
 * CAN Initialization
 *----------------------------------------------------------------------------*/

/**
 * @brief Initialize CAN interface and set up filters
 *
 * Configures CAN hardware with appropriate filters for command and data messages,
 * then starts the CAN peripheral.
 *
 * @return true if setup was successful
 * @return false if any error occurred during setup
 */
/**
 * @brief Initialize CAN interface and set up filters
 *
 * Configures two filters:
 * 1. Filter 0 for command messages (0x80-0x8F)
 * 2. Filter 1 for data messages (0xA0-0xBF)
 *
 * @return true if setup was successful
 * @return false if any error occurred during setup
 */
bool can_setup(void)
{
/* Configure filter to accept both command (0x80-0x8F) and data (0xA0-0xBF) messages */
#define FILTER_ID_CMD    ((0x00000080 << 3) | 0x4)
#define FILTER_MASK_CMD  ((0x000000F0 << 3) | 0x4)
#define FILTER_ID_DATA   ((0x000000A0 << 3) | 0x4)
#define FILTER_MASK_DATA ((0x000000F0 << 3) | 0x4)

    // Filter 0: Command messages (0x80-0x8F)
    canfil.FilterBank           = 0;
    canfil.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh         = FILTER_ID_CMD >> 16;
    canfil.FilterIdLow          = FILTER_ID_CMD & 0xFFFF;
    canfil.FilterMaskIdHigh     = FILTER_MASK_CMD >> 16;
    canfil.FilterMaskIdLow      = FILTER_MASK_CMD & 0xFFFF;
    canfil.FilterScale          = CAN_FILTERSCALE_32BIT;
    canfil.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &canfil);

    // Filter 1: Data messages (0xA0-0xBF)
    canfil.FilterBank       = 1;
    canfil.FilterIdHigh     = FILTER_ID_DATA >> 16;
    canfil.FilterIdLow      = FILTER_ID_DATA & 0xFFFF;
    canfil.FilterMaskIdHigh = FILTER_MASK_DATA >> 16;
    canfil.FilterMaskIdLow  = FILTER_MASK_DATA & 0xFFFF;
    HAL_CAN_ConfigFilter(&hcan1, &canfil);

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return false;
    }

    debug_send("CAN setup complete for %d sensors", MAX_RADAR_SENSORS);

    return true;
}

/* Undefine local macros to prevent namespace pollution */
#undef FILTER_ID_CMD
#undef FILTER_MASK_CMD
#undef FILTER_ID_DATA
#undef FILTER_MASK_DATA

/*------------------------------------------------------------------------------
 * Message Transmission Functions
 *----------------------------------------------------------------------------*/

/**
 * @brief Send a single byte CAN message
 *
 * @param ID CAN message identifier
 * @param message Single byte payload to send
 * @return true if message was successfully queued
 * @return false if transmission failed
 */
bool can_send(uint32_t ID, uint8_t message)
{
    txHeader.DLC                = 1;
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID;
    txHeader.ExtId              = ID;
    txHeader.TransmitGlobalTime = DISABLE;

    debug_send("CAN TX (0x%02X) 0x%02X", ID, message);
    uint8_t csend[1] = { message };
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox) == HAL_OK)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Send a multi-byte CAN message
 *
 * @param ID CAN message identifier
 * @param message Pointer to message data
 * @param length Length of message data (max 8 bytes)
 * @return true if message was successfully queued
 * @return false if transmission failed
 */
bool can_send_array(uint32_t ID, uint8_t *message, size_t length)
{
    txHeader.DLC                = length;
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
    debug_send("CAN TX (0x%02X) 0x%s", ID, data_hex);

    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, message, &canMailbox) == HAL_OK)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Send a command to a specific sensor
 *
 * @param sensor_idx Index of the target sensor (0 to MAX_RADAR_SENSORS-1)
 * @param base_id Base CAN ID for the message type
 * @param message Command code to send
 * @return true if message was successfully queued
 * @return false if invalid sensor index or transmission failed
 */
bool can_send_to_sensor(uint8_t sensor_idx, uint32_t base_id, uint8_t message)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }

    uint32_t sensor_id = base_id + CAN_SENSOR_OFFSET(sensor_idx);
    return can_send(sensor_id, message);
}

/**
 * @brief Send a command to all connected sensors
 *
 * @param base_id Base CAN ID for the message type
 * @param message Command code to send
 */
void broadcast_to_all_sensors(uint32_t base_id, uint8_t message)
{
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        can_send_to_sensor(i, base_id, message);
    }
}

/*------------------------------------------------------------------------------
 * Message Reception and Processing
 *----------------------------------------------------------------------------*/

/**
 * @brief Extract sensor index from a CAN message ID
 *
 * Analyzes the CAN message ID to determine which sensor it belongs to.
 * For command messages (0x80-0x82), extracts index from the lower 4 bits.
 * For data messages (0xA0-0xC4), determines index from the message type range.
 *
 * @param can_id CAN message identifier
 * @return uint8_t Sensor index (0 to MAX_RADAR_SENSORS-1) or 0xFF if invalid
 */
uint8_t get_sensor_index_from_can_id(uint32_t can_id)
{
    // Handle command messages (0x80-0x82)
    if ((can_id & 0xF0) == 0x80)
    {
        uint8_t sensor_idx = can_id & 0x0F;
        if (sensor_idx < MAX_RADAR_SENSORS)
        {
            return sensor_idx;
        }
    }

    // Handle data messages (0xA0-0xC4 range)
    if ((can_id & 0xF0) >= 0xA0 && (can_id & 0xF0) <= 0xC0)
    {
        // Extract sensor index from the lower 4 bits
        // 0xA0 = sensor 0, 0xA1 = sensor 0, etc.
        // 0xB0 = sensor 1, 0xB1 = sensor 1, etc.
        // 0xC0 = sensor 2, 0xC1 = sensor 2, etc.

        uint8_t base_id    = can_id & 0xF0;
        uint8_t sensor_idx = 0;

        switch (base_id)
        {
        case 0xA0:
            sensor_idx = 0;
            break; // Sensor 0 data messages
        case 0xB0:
            sensor_idx = 1;
            break; // Sensor 1 data messages
        case 0xC0:
            sensor_idx = 2;
            break; // Sensor 2 data messages
        default:
            debug_send("Unknown base ID for sensor indexing: 0x%02X", base_id);
            return 0xFF; // Invalid
        }

        if (sensor_idx < MAX_RADAR_SENSORS)
        {
            return sensor_idx;
        }
    }

    debug_send("Invalid CAN ID for sensor indexing: 0x%02X", can_id);
    return 0xFF; // Invalid sensor index
}

/**
 * @brief CAN receive callback function
 *
 * Called by the HAL when a CAN message is received in FIFO0.
 * Processes both command and data messages from connected radar sensors.
 *
 * @param hcan1 Pointer to the CAN handle structure
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX) == HAL_OK)
    {
        can_data_union_t rx_data;
        memcpy(rx_data.bytes, canRX, rxHeader.DLC);

        // Handle command messages
        if ((rxHeader.ExtId & 0xF0) == 0x80)
        {
            uint8_t sensor_idx = rxHeader.ExtId & 0x0F;
            uint8_t command    = rx_data.bytes[0];

            debug_send("CAN CMD S%d: 0x%02X Data: 0x%02X", sensor_idx, rxHeader.ExtId, command);

            if (sensor_idx < MAX_RADAR_SENSORS)
            {
                process_sensor_command(sensor_idx, command, &rx_data);
            }
            return;
        }

        // Handle data messages
        uint8_t sensor_idx = get_sensor_index_from_can_id(rxHeader.ExtId);

        if (sensor_idx >= MAX_RADAR_SENSORS)
        {
            debug_send("Invalid sensor index for ID: 0x%02X", rxHeader.ExtId);
            return;
        }

        radar_data_t *sensor  = &radar_system.sensors[sensor_idx];
        uint8_t       base_id = rxHeader.ExtId & 0xF0;

        // Debug output
        char data_hex[17];
        memset(data_hex, 0, 17);
        for (int i = 0; i < rxHeader.DLC; i++)
        {
            sprintf(data_hex, "%s%02X", data_hex, rx_data.bytes[i]);
        }
        debug_send("CAN RX S%d: (0x%02X) 0x%s", sensor_idx, rxHeader.ExtId, data_hex);

        switch (base_id)
        {
        case 0xA0: // Header
            sensor->totalPacketLength = rx_data.words[0];
            sensor->frameNumber       = rx_data.words[1];

            // FIXED: Proper packet size calculation
            // Each detected point is 8 bytes (2 floats: distance + SNR)
            // Header is 8 bytes, so subtract header size and divide by point size
            if (sensor->totalPacketLength > 8)
            {
                sensor->numDetPoints = (sensor->totalPacketLength - 8) / 8;
            }
            else
            {
                sensor->numDetPoints = 0;
            }

            // Sanity check
            if (sensor->numDetPoints > MAX_RADAR_DETECTED_POINTS)
            {
                debug_send("S%d: Too many points (%d), limiting to %d", sensor_idx, sensor->numDetPoints, MAX_RADAR_DETECTED_POINTS);
                sensor->numDetPoints = MAX_RADAR_DETECTED_POINTS;
            }

            sensor->pointIndex     = 0;
            sensor->maxSNR         = 0;
            sensor->new_data_ready = false;

            debug_send("S%d Frame: %d, Points: %d (total packet: %d bytes)", sensor_idx, sensor->frameNumber, sensor->numDetPoints, sensor->totalPacketLength);
            break;

        case 0xA1: // Detected point
            if (sensor->pointIndex < MAX_RADAR_DETECTED_POINTS && sensor->pointIndex < sensor->numDetPoints)
            {
                sensor->detectedPoints[sensor->pointIndex][0] = rx_data.floats[0];
                sensor->detectedPoints[sensor->pointIndex][1] = rx_data.floats[1];

                debug_send("S%d Point %d: %.2f, %.2f",
                           sensor_idx,
                           sensor->pointIndex,
                           sensor->detectedPoints[sensor->pointIndex][0],
                           sensor->detectedPoints[sensor->pointIndex][1]);

                sensor->pointIndex++;

                if (sensor->pointIndex >= sensor->numDetPoints)
                {
                    sensor->new_data_ready = true;
                    // Process complete frame for this sensor
                    process_complete_radar_frame(sensor_idx);
                }
            }
            break;

        case 0xA3: // Status
            sensor->status = (radar_hw_status_t)rx_data.bytes[0];
            debug_send("S%d Status: %d", sensor_idx, rx_data.bytes[0]);
            break;

        case 0xA4: // Version
            sensor->version.major = rx_data.bytes[0];
            sensor->version.minor = rx_data.bytes[1];
            sensor->version.sub   = rx_data.bytes[2];
            debug_send("S%d Version: %d.%d.%d", sensor_idx, sensor->version.major, sensor->version.minor, sensor->version.sub);
            break;
        }

        radar_system.last_message_timestamp[sensor_idx] = HAL_GetTick();
        radar_system.sensor_online[sensor_idx]          = true;
    }
    else
    {
        debug_send("CAN RX ERROR");
    }
}

/**
 * @brief Process a command received for a specific sensor
 *
 * Handles various commands like start, stop, status, and calibration
 * for a specific radar sensor.
 *
 * @param sensor_idx Index of the target sensor
 * @param command Command code received
 * @param data Pointer to the complete CAN message data
 */
void process_sensor_command(uint8_t sensor_idx, uint8_t command, can_data_union_t *data)
{
    radar_data_t *sensor = &radar_system.sensors[sensor_idx];

    switch (command)
    {
    case CAN_CMD_START:
        radar_status_set(RADAR_CHIRPING);
        sensor->status = RADAR_HW_CHIRPING; //  Correct
        debug_send("S%d: Start command received - radar chirping", sensor_idx);
        break;

    case CAN_CMD_STOP:
        // FIXED: Use correct radar hardware status
        radar_status_set(RADAR_STOPPED);
        sensor->status = RADAR_HW_STOPPED; //  Correct
        debug_send("S%d: Stop command received - radar stopped", sensor_idx);
        break;

    case CAN_CMD_STATUS:
        // Respond with current status
        can_send(CAN_MSG_ID_STATUS_SENSOR(sensor_idx), (uint8_t)sensor->status);
        radar_init_status_set(RADAR_INIT_OK);
        debug_send("S%d: Status response sent", sensor_idx);
        break;

    case CAN_CMD_CAL:
        debug_send("S%d: Calibration command received", sensor_idx);
        break;

    default:
        // Keep only error messages
        debug_send("S%d: Unknown cmd 0x%02X", sensor_idx, command);
        break;
    }
}

/**
 * @brief Process a complete radar data frame
 *
 * Called when all expected points for a frame have been received.
 * In continuous operation mode, immediately processes the data and
 * triggers void detection when enough sensors have reported data.
 *
 * @param sensor_idx Index of the sensor that completed a frame
 */
void process_complete_radar_frame(uint8_t sensor_idx)
{
    radar_data_t *sensor = &radar_system.sensors[sensor_idx];

    debug_send("S%d: Frame %d complete with %d points", sensor_idx, sensor->frameNumber, sensor->numDetPoints);

    /* Mark sensor as online/active */
    radar_system.last_message_timestamp[sensor_idx] = HAL_GetTick();
    radar_system.sensor_online[sensor_idx]          = true;

    /* Process this sensor's measurement data immediately */
    radar_process_measurement(sensor_idx, sensor->detectedPoints, sensor->numDetPoints);

    /* Mark data as processed */
    sensor->new_data_ready = false;

    /* In continuous mode, check if we should trigger void detection */
    static uint32_t last_void_process_time = 0;
    uint32_t        current_time           = HAL_GetTick();

    /* Process void detection no more than once every 200ms */
    if (current_time - last_void_process_time >= 200)
    {
        /* Count active sensors with valid data */
        uint8_t valid_sensors = 0;
        for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
        {
            if (radar_has_valid_data(i))
            {
                valid_sensors++;
            }
        }

        /* Only trigger void detection if we have enough data */
        if (valid_sensors >= 2)
        {
            debug_send("Continuous mode: Triggering void detection with %d sensors", valid_sensors);
            void_system_process();
            last_void_process_time = current_time;
        }
    }
}

/**
 * @brief Get the number of currently active sensors
 *
 * Counts how many sensors are currently marked as online
 * in the radar system structure.
 *
 * @return uint8_t Number of online sensors (0 to MAX_RADAR_SENSORS)
 */
uint8_t get_active_sensor_count(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        if (radar_system.sensor_online[i])
        {
            count++;
        }
    }
    return count;
}

/**
 * @brief Check if a sensor is currently online
 *
 * A sensor is considered online if a message has been received
 * within the last 2 seconds.
 *
 * @param sensor_idx Index of the sensor to check
 * @return true if sensor is online
 * @return false if sensor is offline or index is invalid
 */
bool is_sensor_online(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return false;
    }

    // Consider sensor online if we've received a message in the last 2 seconds
    uint32_t current_time = HAL_GetTick();
    uint32_t last_message = radar_system.last_message_timestamp[sensor_idx];

    return (current_time - last_message) < 2000; // 2 second timeout
}

/**
 * @brief Reset all data for a specific sensor
 *
 * Clears all data associated with the specified sensor
 * and marks it as offline.
 *
 * @param sensor_idx Index of the sensor to reset
 */
void reset_sensor_data(uint8_t sensor_idx)
{
    if (sensor_idx >= MAX_RADAR_SENSORS)
    {
        return;
    }

    radar_data_t *sensor = &radar_system.sensors[sensor_idx];
    memset(sensor, 0, sizeof(radar_data_t));
    radar_system.sensor_online[sensor_idx] = false;
}

/*------------------------------------------------------------------------------
 * Testing and Diagnostic Functions
 *----------------------------------------------------------------------------*/

/**
 * @brief Run tests on sensor index extraction functionality
 *
 * Tests the sensor index extraction function against various
 * CAN ID values to verify correct operation.
 */
void test_sensor_indexing(void)
{
    debug_send("=== Testing Sensor Indexing Fix ===");

    // Test various CAN IDs to verify sensor indexing works correctly
    struct
    {
        uint32_t    can_id;
        uint8_t     expected_sensor;
        const char *message_type;
    } test_cases[] = {
        // Sensor 0 messages (0xA0-0xAF range)
        { 0xA0, 0, "S0 Header" },
        { 0xA1, 0, "S0 Object" },
        { 0xA2, 0, "S0 Profile" },
        { 0xA3, 0, "S0 Status" },
        { 0xA4, 0, "S0 Version" },

        // Sensor 1 messages (0xB0-0xBF range)
        { 0xB0, 1, "S1 Header" },
        { 0xB1, 1, "S1 Object" },
        { 0xB2, 1, "S1 Profile" },
        { 0xB3, 1, "S1 Status" },
        { 0xB4, 1, "S1 Version" },

        // Sensor 2 messages (0xC0-0xCF range)
        { 0xC0, 2, "S2 Header" },
        { 0xC1, 2, "S2 Object" },
        { 0xC2, 2, "S2 Profile" },
        { 0xC3, 2, "S2 Status" },
        { 0xC4, 2, "S2 Version" },

        // Command messages (should work too)
        { 0x80, 0, "S0 Command" },
        { 0x81, 1, "S1 Command" },
        { 0x82, 2, "S2 Command" },

        // Invalid cases
        { 0x70, 0xFF, "Invalid ID" },
        { 0xD0, 0xFF, "Out of range" },
    };

    uint8_t test_count = sizeof(test_cases) / sizeof(test_cases[0]);
    uint8_t passed     = 0;

    for (uint8_t i = 0; i < test_count; i++)
    {
        uint8_t result = get_sensor_index_from_can_id(test_cases[i].can_id);
        bool    pass   = (result == test_cases[i].expected_sensor);

        debug_send("Test %2d: ID 0x%02X (%s) -> Sensor %d (expected %d) %s",
                   i + 1,
                   test_cases[i].can_id,
                   test_cases[i].message_type,
                   result,
                   test_cases[i].expected_sensor,
                   pass ? "PASS" : "FAIL");

        if (pass)
        {
            passed++;
        }
    }

    debug_send("=== Sensor Indexing Test Results: %d/%d PASSED ===", passed, test_count);

    if (passed == test_count)
    {
        debug_send("✓ All sensor indexing tests PASSED - system ready");
    }
    else
    {
        debug_send("✗ %d sensor indexing tests FAILED - check implementation", test_count - passed);
    }
}

// Add this function after test_sensor_indexing()

/**
 * @brief Test connectivity with physical sensors
 *
 * Sends status requests to all configured sensors and checks
 * for responses to determine which sensors are online.
 */
void test_sensor_responses(void)
{
    debug_send("=== Testing Real Sensor Responses ===");

    // Send status command to each sensor individually
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        debug_send("Pinging sensor %d...", i);
        can_send_to_sensor(i, CAN_CMD_BASE, CAN_CMD_STATUS);

        // Small delay between sensor pings
        HAL_Delay(100);
    }

    // Wait a bit for responses
    HAL_Delay(500);

    // Check results
    uint8_t responding_sensors = 0;
    for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
    {
        bool online = is_sensor_online(i);
        debug_send("Sensor %d: %s", i, online ? "ONLINE" : "OFFLINE");
        if (online)
        {
            responding_sensors++;
        }
    }

    debug_send("=== Sensor Response Test: %d/%d sensors responding ===", responding_sensors, MAX_RADAR_SENSORS);
}
