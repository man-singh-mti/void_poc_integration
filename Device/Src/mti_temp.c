#include "mti_temp.h"
#include "vmt_adc.h"
#include "vmt_uart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

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

/* Main temperature processing function */
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
            // Stage 3: Update flags and status
            temp_update_flags(&processed_data, &prv_temp_current_status);

            // Update current temperature for command access
            prv_temp_current_status.current_temperature = processed_data.temperature_c;
            prv_temp_current_status.last_update_ms      = processed_data.process_time_ms;
            prv_temp_current_status.system_ready        = true;
        }
    }
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
    uart_tx_channel_set(UART_DEBUG);
    printf("&temp,status,%d,%d,%d,%d\n",
           prv_temp_current_status.current_temperature,
           prv_temp_current_status.high_temp_alert ? 1 : 0,
           prv_temp_current_status.low_temp_alert ? 1 : 0,
           prv_temp_current_status.system_ready ? 1 : 0);
    uart_tx_channel_undo();
}

void handle_temp_config_command(const char *params)
{
    // Implementation for configuration commands
    uart_tx_channel_set(UART_DEBUG);
    printf("&temp,config,ack\n");
    uart_tx_channel_undo();
}

/* Initialize temperature system */
bool temp_init(void)
{
    // Initialize status structure
    memset(&prv_temp_current_status, 0, sizeof(temp_status_t));
    prv_temp_current_status.current_temperature = TEMP_INVALID_READING;

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

    printf("@db,Temperature module ready, ADC test: %u\n", test_adc);
    return true;
}

/* Private function implementations */

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
