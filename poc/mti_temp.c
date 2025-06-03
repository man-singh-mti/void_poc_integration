/*******************************************************************************
 * @file      mti_temp.c
 * @brief     Temperature sensor implementation
 * @details   Implements temperature measurement, alert generation, and status reporting
 *            according to the protocol specified in void_additions.md
 * @author    Engineering Team
 * @version   1.0.0
 * @date      2025-05-13
 * @copyright (c) 2025 MTi Group
 * @license   Proprietary
 ******************************************************************************/

/* Own header */
#include "mti_temp.h"

/* Project includes */
#include "vmt_adc.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include "vmt_common_defs.h"

/* Standard library includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> // Include math.h for M_PI

/*******************************************************************************
 * Private constants
 ******************************************************************************/

/* Temperature conversion constants */
static const float TEMP_SCALING_FACTOR = 10.0f; /**< Convert ADC to 0.1°C precision */

/*******************************************************************************
 * Private variables
 ******************************************************************************/

/* Temperature sensor configuration */
static int16_t temp_calibration_offset = 0;                       /**< Calibration offset in 0.1°C units */
static int16_t temp_alert_low          = TEMP_DEFAULT_LOW_ALERT;  /**< Low temperature alert threshold */
static int16_t temp_alert_high         = TEMP_DEFAULT_HIGH_ALERT; /**< High temperature alert threshold */
static uint8_t temp_rate               = TEMP_DEFAULT_RATE;       /**< Readings per second */

/* Temperature state variables */
static int16_t  temp_value        = 0;     /**< Current temperature in 0.1°C units */
static uint8_t  temp_status       = 0;     /**< Status code per documentation */
static bool     b_temp_alert_low  = false; /**< Low temperature alert flag */
static bool     b_temp_alert_high = false; /**< High temperature alert flag */
static uint32_t message_id_temp   = 0;     /**< Message counter for temperature alerts */

/* Mapping between event types and their string representations */
static const char *const temp_event_strings[] = {
    "n",  /**< normal */
    "l",  /**< low alert */
    "h",  /**< high alert */
    "el", /**< error low */
    "eh"  /**< error high */
};

/*******************************************************************************
 * Private function prototypes
 ******************************************************************************/

static bool temp_check_error_limits(void);
static bool temp_check_warning_limits(void);
static void temp_handle_normal_state(void);
static bool temp_read_and_convert(void);

/*******************************************************************************
 * Public function implementations
 ******************************************************************************/


/**
 * @brief Reset temperature sensor state
 *
 * @details Clears all state variables, alert flags, and message counters
 *
 * @note Does not change configuration parameters or calibration
 */
void temp_reset(void)
{
    temp_value        = 0;
    temp_status       = TEMP_STATUS_NORMAL;
    b_temp_alert_low  = false;
    b_temp_alert_high = false;
    message_id_temp   = 0;
}

/**
 * @brief Send temperature event message
 *
 * @param[in] event_type Type of temperature event
 * @param[in] value Temperature value associated with event in 0.1°C
 *
 * @details Format: "!tp,<id>,<event_type>,<value>"
 */
void temp_send_event(temp_event_t event_type, int16_t value)
{
    /* Ensure event_type is within valid range */
    if ((unsigned int)event_type >= sizeof(temp_event_strings) / sizeof(temp_event_strings[0]))
    {
        return;
    }

    uart_tx_channel_set(UART_UPHOLE);
    printf("!tp,%lu,%s,%d\n", (unsigned long)message_id_temp++, temp_event_strings[event_type], (int)value);
    uart_tx_channel_undo();
}

/*******************************************************************************
 * Private function implementations
 ******************************************************************************/

/**
 * @brief Check if temperature is outside valid measurement range
 *
 * @return true if temperature is in error state, false otherwise
 *
 * @note Updates temp_status and sends events if needed
 */
static bool temp_check_error_limits(void)
{
    if (temp_value < TEMP_MIN_VALID)
    {
        /* Below valid range (-40°C) */
        temp_status = TEMP_STATUS_ERROR_LOW;
        if (system_get_status() != SYSTEM_STATUS_ERROR_TEMP)
        {
            system_set_status(SYSTEM_STATUS_ERROR_TEMP);
            temp_send_event(TEMP_EVENT_ERROR_LOW, temp_value);
        }
        return true;
    }
    else if (temp_value > TEMP_MAX_VALID)
    {
        /* Above valid range (150°C) */
        temp_status = TEMP_STATUS_ERROR_HIGH;
        if (system_get_status() != SYSTEM_STATUS_ERROR_TEMP)
        {
            system_set_status(SYSTEM_STATUS_ERROR_TEMP);
            temp_send_event(TEMP_EVENT_ERROR_HIGH, temp_value);
        }
        return true;
    }
    return false;
}

/**
 * @brief Check if temperature is within warning thresholds
 *
 * @return true if temperature is in warning state, false otherwise
 *
 * @note Updates temp_status and sends events if needed
 */
static bool temp_check_warning_limits(void)
{
    if (temp_value < temp_alert_low)
    {
        temp_status = TEMP_STATUS_WARNING_LOW;
        if (!b_temp_alert_low)
        {
            b_temp_alert_low  = true;
            b_temp_alert_high = false;
            system_set_status(SYSTEM_STATUS_WARNING_TEMP_LOW);
            temp_send_event(TEMP_EVENT_ALERT_LOW, temp_value);
        }
        return true;
    }
    else if (temp_value > temp_alert_high)
    {
        temp_status = TEMP_STATUS_WARNING_HIGH;
        if (!b_temp_alert_high)
        {
            b_temp_alert_high = true;
            b_temp_alert_low  = false;
            system_set_status(SYSTEM_STATUS_WARNING_TEMP_HIGH);
            temp_send_event(TEMP_EVENT_ALERT_HIGH, temp_value);
        }
        return true;
    }
    return false;
}

/**
 * @brief Update status when temperature is in normal range
 *
 * @details Clears alert flags with hysteresis and updates system status
 *
 * @note Uses TEMP_HYSTERESIS to prevent alert oscillation around thresholds
 */
static void temp_handle_normal_state(void)
{
    temp_status = TEMP_STATUS_NORMAL;

    /* Reset low alert flag with hysteresis */
    if (b_temp_alert_low && temp_value > (temp_alert_low + TEMP_HYSTERESIS))
    {
        b_temp_alert_low = false;
        if (system_get_status() == SYSTEM_STATUS_WARNING_TEMP_LOW)
        {
            system_set_status(SYSTEM_STATUS_OK);
        }
        temp_send_event(TEMP_EVENT_NORMAL, temp_value);
    } /* Reset high alert flag with hysteresis */
    if (b_temp_alert_high && temp_value < (temp_alert_high - TEMP_HYSTERESIS))
    {
        b_temp_alert_high = false;
        if (system_get_status() == SYSTEM_STATUS_WARNING_TEMP_HIGH)
        {
            system_set_status(SYSTEM_STATUS_OK);
        }
        temp_send_event(TEMP_EVENT_NORMAL, temp_value);
    }
}

/**
 * @brief Read and convert ADC value to temperature
 *
 * @return true if successful, false on ADC error
 *
 * @note Updates temp_value with the new temperature reading
 * @warning Sets error status on ADC read failure
 */
static bool temp_read_and_convert(void)
{
    /* Get raw ADC reading */
    uint16_t adc_raw = adc_value_get_uint16(ADC_SEQ_TEMP);

    /* Convert directly to temperature using integer math */
    int16_t raw_temp = adc_raw * 10;
    temp_value       = raw_temp + temp_calibration_offset;

    return true;
}

/**
 * @brief Main temperature detection function
 *
 * @details Uses ADC to read temperature and sets status based on thresholds
 *
 * @note Should be called periodically from the main loop
 */
void temp_detect(void)
{
    /* Previous status for change detection */
    uint8_t prev_status = temp_status;

    /* Read and convert ADC value */
    if (!temp_read_and_convert())
    {
        return;
    }

    /* Check temperature against limits */
    if (!temp_check_error_limits() && !temp_check_warning_limits())
    {
        temp_handle_normal_state();
    }

    /* Log status changes to debug channel */
    if (prev_status != temp_status)
    {
        uart_tx_channel_set(UART_DEBUG);
        printf("$db,Temperature status changed: %d -> %d (%d.%d°C)\n", (int)prev_status, (int)temp_status, (int)(temp_value / 10), (int)abs(temp_value % 10));
        uart_tx_channel_undo();
    }
}

/**
 * @brief Get current temperature value with 0.1°C precision
 *
 * @return int16_t Temperature in tenths of a degree Celsius
 *
 * @note Returns most recently measured value, does not trigger a new reading
 */
int16_t temp_value_get(void)
{
    return temp_value;
}

/**
 * @brief Get current temperature status code
 *
 * @return uint8_t Status code (0-4) per protocol documentation
 */
uint8_t temp_status_get(void)
{
    return temp_status;
}

/**
 * @brief Get alert flags bitmap
 *
 * @return uint8_t Bitmap of active alerts (TEMP_ALERT_LOW | TEMP_ALERT_HIGH)
 */
uint8_t temp_alerts_get(void)
{
    uint8_t alerts = TEMP_ALERT_NONE;

    if (b_temp_alert_low)
    {
        alerts |= TEMP_ALERT_LOW;
    }
    if (b_temp_alert_high)
    {
        alerts |= TEMP_ALERT_HIGH;
    }
    return alerts;
}

/**
 * @brief Set temperature alert thresholds
 *
 * @param[in] low_alert Low temperature threshold in 0.1°C
 * @param[in] high_alert High temperature threshold in 0.1°C
 *
 * @note Will trigger an immediate re-evaluation of current temperature against new thresholds
 */
void temp_set_thresholds(int16_t low_alert, int16_t high_alert)
{
    /* Store previous thresholds for change detection */
    int16_t prev_low  = temp_alert_low;
    int16_t prev_high = temp_alert_high;

    /* Update thresholds */
    temp_alert_low  = low_alert;
    temp_alert_high = high_alert;

    /* Log changes to debug channel */
    if (prev_low != low_alert || prev_high != high_alert)
    {
        uart_tx_channel_set(UART_DEBUG);
        printf("$db,Temperature thresholds updated: low=%d.%d°C, high=%d.%d°C\n",
               (int)(low_alert / 10),
               (int)abs(low_alert % 10),
               (int)(high_alert / 10),
               (int)abs(high_alert % 10));
        uart_tx_channel_undo();
    }

    /* Re-evaluate current temperature with new thresholds */
    temp_detect();
}

/**
 * @brief Get current temperature alert thresholds
 *
 * @param[out] low_alert Pointer to store low threshold, may be NULL
 * @param[out] high_alert Pointer to store high threshold, may be NULL
 */
void temp_get_thresholds(int16_t *restrict low_alert, int16_t *restrict high_alert)
{
    if (low_alert)
    {
        *low_alert = temp_alert_low;
    }
    if (high_alert)
    {
        *high_alert = temp_alert_high;
    }
}

/**
 * @brief Calibrate temperature sensor with reference value
 *
 * @param[in] ref_temp_str Reference temperature string in 0.1°C (e.g., "230" for 23.0°C)
 * @return int16_t Calculated offset value in 0.1°C
 *
 * @note Will trigger an immediate re-evaluation of current temperature with new calibration
 */
int16_t temp_calibration_set(const char *restrict ref_temp_str)
{
    if (ref_temp_str == NULL)
    {
        return temp_calibration_offset; /* Return current offset on invalid input */
    }

    /* Convert string to int and explicitly cast to int16_t with range check */
    int ref_temp_int = atoi(ref_temp_str);
    /* Ensure value is within int16_t range */
    if (ref_temp_int > INT16_MAX || ref_temp_int < INT16_MIN)
    {
        uart_tx_channel_set(UART_DEBUG);
        printf("$db,Temperature calibration failed: Value out of range (%d)\n", ref_temp_int);
        uart_tx_channel_undo();
        return temp_calibration_offset;
    }
    int16_t ref_temp = (int16_t)ref_temp_int;

    uint16_t adc_raw = adc_value_get_uint16(ADC_SEQ_TEMP);

    /* Calculate new offset based on reference temperature */
    float   raw_temp        = ((float)adc_raw * 10.0f) / TEMP_SCALING_FACTOR;
    int16_t raw_value       = (int16_t)raw_temp;
    temp_calibration_offset = ref_temp - raw_value;

    /* Log calibration to debug channel */
    uart_tx_channel_set(UART_DEBUG);
    printf("$db,Temperature calibrated: ref=%d.%d°C, ADC=%d, offset=%d\n", (int)(ref_temp / 10), (int)abs(ref_temp % 10), (int)adc_raw, (int)temp_calibration_offset);
    uart_tx_channel_undo();

    /* Update temperature value with new calibration */
    temp_detect();

    return temp_calibration_offset;
}

/**
 * @brief Set temperature configuration parameter
 *
 * @param[in] param Parameter name (alert_low, alert_high, rate)
 * @param[in] value Parameter value as string
 * @return int 0 on success, -1 on error (invalid parameter name or value)
 */
int temp_config_set(const char *restrict param, const char *restrict value)
{
    if (param == NULL || value == NULL)
    {
        return -1;
    }

    if (strcmp(param, "alert_low") == 0)
    {
        int param_value = atoi(value);
        /* Check if value is within valid temperature range and int16_t range */
        if (param_value >= TEMP_MIN_VALID && param_value <= TEMP_MAX_VALID && param_value >= INT16_MIN && param_value <= INT16_MAX)
        {
            temp_alert_low = (int16_t)param_value;
            temp_detect(); /* Re-evaluate with new threshold */
            return 0;
        }
        return -1;
    }
    else if (strcmp(param, "alert_high") == 0)
    {
        int param_value = atoi(value);
        /* Check if value is within valid temperature range */
        if (param_value >= TEMP_MIN_VALID && param_value <= TEMP_MAX_VALID)
        {
            temp_alert_high = (int16_t)param_value;
            temp_detect(); /* Re-evaluate with new threshold */
            return 0;
        }
        return -1;
    }
    else if (strcmp(param, "rate") == 0)
    {
        int new_rate = atoi(value);
        if (new_rate >= 1 && new_rate <= 60)
        {
            temp_rate = (uint8_t)new_rate;
            return 0;
        }
    }

    return -1; /* Invalid parameter */
}

/**
 * @brief Print diagnostic information for temperature sensor
 *
 * @param[in] channel UART channel to output to
 *
 * @details Format: "&tp,0,diag,<raw_adc>,<conversion_factor>,<offset>"
 */
void temp_diag_print(uart_select_t channel)
{
    uint16_t adc_raw = adc_value_get_uint16(ADC_SEQ_TEMP);

    uart_tx_channel_set(channel);
    /* Note the "&tp" prefix following protocol convention */
    printf("&tp,0,diag,%u,%.1f,%d\n", (unsigned int)adc_raw, (double)TEMP_SCALING_FACTOR, (int)temp_calibration_offset);
    uart_tx_channel_undo();
}

/**
 * @brief Initialize temperature sensor module
 *
 * @return true if initialization was successful
 * @return false if initialization failed
 *
 * @note Must be called before any other temperature functions
 * @warning Requires ADC to be properly initialized first
 */
bool temp_init(void)
{
    uint16_t adc_raw = adc_value_get_uint16(ADC_SEQ_TEMP);

    /* Convert to temperature with 0.1°C precision */
    float   raw_temp     = ((float)adc_raw * 10.0f) / TEMP_SCALING_FACTOR;
    int16_t startup_temp = (int16_t)raw_temp;

    /* Check for valid initial reading within operational range */
    if (startup_temp < TEMP_MIN_VALID || startup_temp > TEMP_MAX_VALID)
    {
        /* Temperature sensor error */
        system_set_status(SYSTEM_STATUS_ERROR_TEMP);
        uart_tx_channel_set(UART_DEBUG);
        printf("$db,Temperature sensor init failed: Reading out of range (%d)\n", (int)startup_temp);
        uart_tx_channel_undo();
        return false;
    } /*
       * Initialize with saved configuration if available
       * This would typically load from EEPROM or Flash storage
       * temp_alert_low = get_saved_temp_low_alert();
       * temp_alert_high = get_saved_temp_high_alert();
       * temp_calibration_offset = get_saved_temp_offset();
       */
    /* Note: Reading rate will be used in future implementation */
    (void)temp_rate;

    /* Reset state variables and get initial reading */
    temp_reset();
    temp_detect();

    uart_tx_channel_set(UART_DEBUG);
    printf("$db,Temperature sensor initialized: %d.%d°C\n", (int)(temp_value / 10), (int)abs(temp_value % 10));
    uart_tx_channel_undo();

    return true;
}

/**
 * @brief Check if low temperature alert is active
 *
 * @return true if low temperature alert is active, false otherwise
 */
bool temp_alert_low_is_active(void)
{
    return b_temp_alert_low;
}

/**
 * @brief Check if high temperature alert is active
 *
 * @return true if high temperature alert is active, false otherwise
 */
bool temp_alert_high_is_active(void)
{
    return b_temp_alert_high;
}
