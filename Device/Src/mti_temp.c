#include "mti_temp.h"
#include "vmt_adc.h"
#include "vmt_uart.h"
#include "main.h"
#include "mti_system.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private variables */
static temp_status_t prv_temp_current_status = { 0 };
static int16_t       prv_temp_high_threshold = TEMP_DEFAULT_HIGH_THRESHOLD_C;
static int16_t       prv_temp_low_threshold  = TEMP_DEFAULT_LOW_THRESHOLD_C;

/* Smoothing buffer for temperature readings */
static int16_t prv_temp_smoothing_buffer[TEMP_SMOOTHING_SAMPLES];
static uint8_t prv_temp_buffer_index = 0;
static bool    prv_temp_buffer_full  = false;

/* Private function declarations */
static int16_t prv_temp_convert_adc_to_celsius(uint16_t adc_value);
static int16_t prv_temp_apply_smoothing(int16_t new_temp);
static bool    prv_temp_validate_reading(int16_t temperature);
static void    prv_temp_send_alert_events(bool high_alert, bool low_alert, bool previous_high, bool previous_low);

/* Main temperature processing function - enhanced with automatic streaming */
void temp_system_process(void)
{
    static temp_raw_data_t       raw_data;
    static temp_processed_data_t processed_data;

    // Stage 1: Get temperature data (ADC conversion handled internally)
    if (temp_get_raw_data(&raw_data))
    {
        // Stage 2: Process and clean data
        if (temp_process_raw_data(&raw_data, &processed_data))
        {
            // Store previous state for change detection
            int16_t previous_temp       = prv_temp_current_status.current_temperature;
            bool    previous_high_alert = prv_temp_current_status.high_temp_alert;
            bool    previous_low_alert  = prv_temp_current_status.low_temp_alert;

            // Stage 3: Update flags and status
            temp_update_flags(&processed_data, &prv_temp_current_status);

            // Update current temperature for command access
            prv_temp_current_status.current_temperature = processed_data.temperature_c;
            prv_temp_current_status.last_update_ms      = processed_data.process_time_ms;
            prv_temp_current_status.system_ready        = true;

            // Stage 4: Check for significant changes
            prv_temp_current_status.significant_change = temp_significant_change_detected();

            // Stage 5: Send automatic streams based on void.md requirements
            bool alert_change = (previous_high_alert != prv_temp_current_status.high_temp_alert) || (previous_low_alert != prv_temp_current_status.low_temp_alert);

            bool should_stream = alert_change || prv_temp_current_status.significant_change || temp_max_interval_exceeded() ||
                                 (prv_temp_current_status.last_streamed_temperature == TEMP_INVALID_READING);

            // Only send automatic streams in operational mode (like water module)
            if (should_stream && state_get() == measure_state) // ← Changed this line
            {
                // Send alert events first if needed (like water module pattern)
                if (alert_change)
                {
                    prv_temp_send_alert_events(prv_temp_current_status.high_temp_alert, prv_temp_current_status.low_temp_alert, previous_high_alert, previous_low_alert);
                }

                // Send automatic change-based stream
                temp_send_change_based_stream();
            }
        }
    }
}

/* Send automatic change-based stream (per void.md specification) */
void temp_send_change_based_stream(void)
{
    // Only send in operational mode (like water module)
    if (state_get() != measure_state)
    {
        return;
    }

    uart_tx_channel_set(UART_UPHOLE);

    // Format: &temp,status,<temp_c>,<alert> (from void.md table)
    uint8_t alert_flag = 0;
    if (prv_temp_current_status.high_temp_alert)
    {
        alert_flag |= 0x01;
    }
    if (prv_temp_current_status.low_temp_alert)
    {
        alert_flag |= 0x02;
    }
    printf("&temp,status,%d,%d\r\n", prv_temp_current_status.current_temperature, alert_flag);

    uart_tx_channel_undo();

    // Update streaming state
    prv_temp_current_status.last_streamed_temperature = prv_temp_current_status.current_temperature;
    prv_temp_current_status.last_stream_time_ms       = HAL_GetTick();
    prv_temp_current_status.significant_change        = false;
}

/* Send alert events (like water module !water,messageID,state pattern) */
static void prv_temp_send_alert_events(bool high_alert, bool low_alert, bool previous_high, bool previous_low)
{
    static uint8_t temp_message_id = 30; // Start at 30 (water uses 10-19, void might use 20-29)

    uart_tx_channel_set(UART_UPHOLE);

    // Send high temperature alert events
    if (high_alert && !previous_high)
    {
        printf("!temp,%d,high\r\n", temp_message_id);
        printf("!temp,%d,high\r\n", temp_message_id); // Send twice like water module
    }
    else if (!high_alert && previous_high)
    {
        printf("!temp,%d,normal_high\r\n", temp_message_id);
        printf("!temp,%d,normal_high\r\n", temp_message_id);
    }

    // Send low temperature alert events
    if (low_alert && !previous_low)
    {
        printf("!temp,%d,low\r\n", temp_message_id);
        printf("!temp,%d,low\r\n", temp_message_id); // Send twice like water module
    }
    else if (!low_alert && previous_low)
    {
        printf("!temp,%d,normal_low\r\n", temp_message_id);
        printf("!temp,%d,normal_low\r\n", temp_message_id);
    }

    uart_tx_channel_undo();

    // Increment message ID (cycle 30-39)
    if (temp_message_id == 39)
    {
        temp_message_id = 30;
    }
    else
    {
        temp_message_id++;
    }
}

/* Check for significant temperature change (1°C threshold per void.md) */
bool temp_significant_change_detected(void)
{
    if (prv_temp_current_status.last_streamed_temperature == TEMP_INVALID_READING)
    {
        return true; // First reading is always significant
    }

    int16_t change = prv_temp_current_status.current_temperature - prv_temp_current_status.last_streamed_temperature;
    if (change < 0)
        change = -change; // Absolute value

    return (change >= TEMP_CHANGE_THRESHOLD_C);
}

/* Check if max interval exceeded (10 seconds per void.md) */
bool temp_max_interval_exceeded(void)
{
    uint32_t elapsed = HAL_GetTick() - prv_temp_current_status.last_stream_time_ms;
    return (elapsed >= TEMP_MAX_INTERVAL_MS);
}

/* Get raw temperature data from ADC */
bool temp_get_raw_data(temp_raw_data_t *raw_data)
{
    if (raw_data == NULL)
    {
        return false;
    }

    // Get ADC reading from existing infrastructure
    uint16_t adc_reading = (uint16_t)adc_value_get(ADC_SEQ_TEMP);

    raw_data->temperature_raw = adc_reading;
    raw_data->timestamp_ms    = HAL_GetTick();
    raw_data->data_valid      = (adc_reading > 0); // Basic validity check

    return raw_data->data_valid;
}

/* Process and clean raw temperature data */
bool temp_process_raw_data(temp_raw_data_t *raw, temp_processed_data_t *processed)
{
    if (raw == NULL || processed == NULL || !raw->data_valid)
    {
        return false;
    }

    // Convert ADC reading to Celsius
    int16_t temp_celsius = prv_temp_convert_adc_to_celsius(raw->temperature_raw);

    // Validate the reading
    if (!prv_temp_validate_reading(temp_celsius))
    {
        processed->data_valid    = false;
        processed->temperature_c = TEMP_INVALID_READING;
        return false;
    }

    // Apply smoothing
    processed->temperature_c   = prv_temp_apply_smoothing(temp_celsius);
    processed->data_valid      = true;
    processed->process_time_ms = HAL_GetTick();

    return true;
}

/* Update temperature flags and status */
void temp_update_flags(temp_processed_data_t *processed, temp_status_t *status)
{
    if (processed == NULL || status == NULL || !processed->data_valid)
    {
        return;
    }

    // Check temperature thresholds
    processed->temp_high_flag = (processed->temperature_c >= prv_temp_high_threshold);
    processed->temp_low_flag  = (processed->temperature_c <= prv_temp_low_threshold);

    // Update status alerts
    status->high_temp_alert = processed->temp_high_flag;
    status->low_temp_alert  = processed->temp_low_flag;
}

/* Get latest temperature status */
void temp_get_latest_status(temp_status_t *status)
{
    if (status != NULL)
    {
        memcpy(status, &prv_temp_current_status, sizeof(temp_status_t));
    }
}

/* Configuration functions */
void temp_set_high_threshold(int16_t threshold_c)
{
    prv_temp_high_threshold = threshold_c;
}

void temp_set_low_threshold(int16_t threshold_c)
{
    prv_temp_low_threshold = threshold_c;
}

int16_t temp_get_high_threshold(void)
{
    return prv_temp_high_threshold;
}

int16_t temp_get_low_threshold(void)
{
    return prv_temp_low_threshold;
}

/* Command interface functions */
void handle_temp_status_command(void)
{
    // Remove hardcoded UART channel - let caller handle channel selection
    printf("&temp,status,%d,%d,%d,%d\n",
           prv_temp_current_status.current_temperature,
           prv_temp_current_status.high_temp_alert ? 1 : 0,
           prv_temp_current_status.low_temp_alert ? 1 : 0,
           prv_temp_current_status.system_ready ? 1 : 0);
}

void handle_temp_config_command(const char *params)
{
    if (params == NULL)
    {
        // Show current thresholds when no parameters provided
        printf("&temp,thresholds,%d,%d\n", prv_temp_high_threshold, prv_temp_low_threshold);
        return;
    }

    // Parse the parameters string
    // Expected format: "high,85" or "low,-10"
    char param_copy[64];
    strncpy(param_copy, params, sizeof(param_copy) - 1);
    param_copy[sizeof(param_copy) - 1] = '\0';

    char *command   = strtok(param_copy, ",");
    char *value_str = strtok(NULL, ",");

    if (command == NULL)
    {
        printf("&temp,config,error,invalid_format\n");
        return;
    }

    if (strcmp(command, "high") == 0 && value_str != NULL)
    {
        int16_t threshold = atoi(value_str);
        temp_set_high_threshold(threshold);
        printf("&temp,config,high,ack,%d\n", threshold);
    }
    else if (strcmp(command, "low") == 0 && value_str != NULL)
    {
        int16_t threshold = atoi(value_str);
        temp_set_low_threshold(threshold);
        printf("&temp,config,low,ack,%d\n", threshold);
    }
    else if (strcmp(command, "get") == 0)
    {
        printf("&temp,thresholds,%d,%d\n", prv_temp_high_threshold, prv_temp_low_threshold);
    }
    else
    {
        printf("&temp,config,nack,unknown_command\n");
    }
}

/* Initialize temperature system - enhanced for streaming */
bool temp_init(void)
{
    // Initialize status structure
    memset(&prv_temp_current_status, 0, sizeof(temp_status_t));
    prv_temp_current_status.current_temperature = TEMP_INVALID_READING;

    // Initialize streaming fields
    prv_temp_current_status.last_streamed_temperature = TEMP_INVALID_READING;
    prv_temp_current_status.last_stream_time_ms       = HAL_GetTick();
    prv_temp_current_status.significant_change        = false;

    // Initialize smoothing buffer
    memset(prv_temp_smoothing_buffer, 0, sizeof(prv_temp_smoothing_buffer));
    prv_temp_buffer_index = 0;
    prv_temp_buffer_full  = false;

    // Set default thresholds
    prv_temp_high_threshold = TEMP_DEFAULT_HIGH_THRESHOLD_C;
    prv_temp_low_threshold  = TEMP_DEFAULT_LOW_THRESHOLD_C;

    // Test ADC reading
    uint16_t test_adc = (uint16_t)adc_value_get(ADC_SEQ_TEMP);
    if (test_adc == 0)
    {
        printf("@db,Temperature ADC initialization failed\n");
        return false;
    }

    printf("@db,Temperature module ready with automatic streaming\n");
    return true;
}

/* Check if temperature system is initialized */
bool temp_is_initialized(void)
{
    return prv_temp_current_status.system_ready;
}

/* Convert ADC reading to Celsius (integer-based) */
static int16_t prv_temp_convert_adc_to_celsius(uint16_t adc_value)
{
    // Simple linear conversion for POC
    // This would need calibration for actual hardware
    // Formula: temp = (adc_value * scale_factor) + offset
    // For POC: assume 25°C at mid-range ADC (2048)
    int32_t temp_raw = ((int32_t)adc_value - 2048) / 20; // Rough conversion
    return (int16_t)(temp_raw + 25);                     // Offset to ~25°C baseline
}

/* Apply smoothing to temperature reading */
static int16_t prv_temp_apply_smoothing(int16_t new_temp)
{
    // Add new reading to circular buffer
    prv_temp_smoothing_buffer[prv_temp_buffer_index] = new_temp;
    prv_temp_buffer_index                            = (prv_temp_buffer_index + 1) % TEMP_SMOOTHING_SAMPLES;

    if (!prv_temp_buffer_full && prv_temp_buffer_index == 0)
    {
        prv_temp_buffer_full = true;
    }

    // Calculate average
    int32_t sum   = 0;
    uint8_t count = prv_temp_buffer_full ? TEMP_SMOOTHING_SAMPLES : prv_temp_buffer_index;

    for (uint8_t i = 0; i < count; i++)
    {
        sum += prv_temp_smoothing_buffer[i];
    }

    return (int16_t)(sum / count);
}

/* Validate temperature reading */
static bool prv_temp_validate_reading(int16_t temperature)
{
    // Basic range check for reasonable temperature values
    return (temperature >= -50 && temperature <= 150);
}

/* Private debug printing function */
static void prv_temp_debug_print(const char *message, ...)
{
    h_dev_debug_t debug_flags;
    dev_printf_debug_get(&debug_flags);

    if (debug_flags.b_temp_sample)
    {
        uart_tx_channel_set(UART_DEBUG);
        printf("@temp_debug,");

        va_list args;
        va_start(args, message);
        vprintf(message, args);
        va_end(args);

        printf("\r\n");
        uart_tx_channel_undo();
    }
}
