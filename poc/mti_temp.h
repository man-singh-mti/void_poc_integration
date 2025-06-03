#ifndef MTI_TEMP_H
#define MTI_TEMP_H

/*******************************************************************************
 * @file      mti_temp.h
 * @brief     Temperature sensor interface for the MTi downhole system
 * @details   Provides a consistent interface for temperature measurement
 *            with enhanced precision (0.1°C units) and configurable alert thresholds.
 *            Implements the protocol described in void_additions.md.
 * @author    Engineering Team
 * @version   1.0.0
 * @date      2025-05-13
 * @copyright (c) 2025 MTi Group
 * @license   Proprietary
 *
 * @note      This module is not thread-safe and should only be called from
 *            a single execution context.
 ******************************************************************************/

/* Standard includes */
#include <stdbool.h>
#include <stdint.h>

/* Project includes */
#include "vmt_uart.h"

/**
 * @brief Temperature range constants in tenths of degree Celsius
 * @note These define the operational limits of the temperature sensor
 */
#define TEMP_MIN_VALID  (-400) /**< -40.0°C: Minimum valid temperature */
#define TEMP_MAX_VALID  1500   /**< 150.0°C: Maximum valid temperature */
#define TEMP_HYSTERESIS 20     /**< 2.0°C: Hysteresis for clearing alerts */

/**
 * @brief Temperature status codes matching documentation
 * @note Values correspond directly to protocol specification
 */
typedef enum
{
    TEMP_STATUS_NORMAL       = 0, /**< 0: Temperature within expected operating range */
    TEMP_STATUS_WARNING_LOW  = 1, /**< 1: Warning - Low temperature approaching limit */
    TEMP_STATUS_WARNING_HIGH = 2, /**< 2: Warning - High temperature approaching limit */
    TEMP_STATUS_ERROR_LOW    = 3, /**< 3: Error - Below valid measurement range */
    TEMP_STATUS_ERROR_HIGH   = 4  /**< 4: Error - Above valid measurement range */
} temp_status_t;

/**
 * @brief Temperature alert events
 * @note Used when sending event messages via UART
 */
typedef enum
{
    TEMP_EVENT_NORMAL,     /**< Temperature returned to normal range */
    TEMP_EVENT_ALERT_LOW,  /**< Low temperature alert triggered */
    TEMP_EVENT_ALERT_HIGH, /**< High temperature alert triggered */
    TEMP_EVENT_ERROR_LOW,  /**< Temperature below valid range */
    TEMP_EVENT_ERROR_HIGH  /**< Temperature above valid range */
} temp_event_t;

/**
 * @brief Alert flags bit masks
 * @note Used for retrieving bitmap of active alerts
 */
#define TEMP_ALERT_NONE 0x00U /**< No alerts active */
#define TEMP_ALERT_LOW  0x01U /**< Low temperature alert active */
#define TEMP_ALERT_HIGH 0x02U /**< High temperature alert active */

/**
 * @brief Default configuration values
 * @note Can be adjusted using temp_config_set
 */
#define TEMP_DEFAULT_LOW_ALERT  50  /**< 5.0°C: Default low temperature alert */
#define TEMP_DEFAULT_HIGH_ALERT 800 /**< 80.0°C: Default high temperature alert */
#define TEMP_DEFAULT_RATE       10  /**< 10 Hz: Default reading rate */


/* Temperature alert status flags */
extern bool b_temp_alert_low;  /* Flag indicating temperature below low threshold */
extern bool b_temp_alert_high; /* Flag indicating temperature above high threshold */

/**
 * @brief Initialize temperature sensor module
 *
 * @return true if initialization was successful
 * @return false if initialization failed
 *
 * @note Must be called before any other temperature functions
 * @warning Requires ADC to be properly initialized first
 */
bool temp_init(void);

/**
 * @brief Reset temperature sensor state
 *
 * @details Clears all state variables, alert flags, and message counters
 *
 * @note Does not change configuration parameters or calibration
 */
void temp_reset(void);

/**
 * @brief Main temperature detection function
 *
 * @details Uses ADC to read temperature and sets status based on thresholds
 *
 * @note Should be called periodically from the main loop
 * @warning If ADC read fails, will set error status
 */
void temp_detect(void);

/**
 * @brief Get current temperature value with 0.1°C precision
 *
 * @return int16_t Temperature in tenths of a degree Celsius
 *
 * @note Returns most recently measured value, does not trigger a new reading
 */
int16_t temp_value_get(void);

/**
 * @brief Get current temperature status code
 *
 * @return uint8_t Status code (0-4) per protocol documentation
 */
uint8_t temp_status_get(void);

/**
 * @brief Get alert flags bitmap
 *
 * @return uint8_t Bitmap of active alerts (TEMP_ALERT_LOW | TEMP_ALERT_HIGH)
 */
uint8_t temp_alerts_get(void);

/**
 * @brief Set temperature alert thresholds
 *
 * @param[in] low_alert Low temperature threshold in 0.1°C
 * @param[in] high_alert High temperature threshold in 0.1°C
 *
 * @note Will trigger an immediate re-evaluation of current temperature against new thresholds
 * @note Low alert must be lower than high alert for proper operation
 */
void temp_set_thresholds(int16_t low_alert, int16_t high_alert);

/**
 * @brief Get current temperature alert thresholds
 *
 * @param[out] low_alert Pointer to store low threshold, may be NULL
 * @param[out] high_alert Pointer to store high threshold, may be NULL
 *
 * @note If either pointer is NULL, that value will not be returned
 */
void temp_get_thresholds(int16_t *restrict low_alert, int16_t *restrict high_alert);

/**
 * @brief Calibrate temperature sensor with reference value
 *
 * @param[in] ref_temp_str Reference temperature string in 0.1°C (e.g., "230" for 23.0°C)
 * @return int16_t Calculated offset value in 0.1°C
 *
 * @note Will trigger an immediate re-evaluation of current temperature with new calibration
 * @note Calibration persists until power cycle or explicit reset
 */
int16_t temp_calibration_set(const char *restrict ref_temp_str);

/**
 * @brief Set temperature configuration parameter
 *
 * @param[in] param Parameter name (alert_low, alert_high, rate)
 * @param[in] value Parameter value as string
 * @return int 0 on success, -1 on error (invalid parameter name or value)
 *
 * @note For "rate", valid values are 1-60 (readings per second)
 * @note For alert thresholds, values are in 0.1°C units
 */
int temp_config_set(const char *restrict param, const char *restrict value);

/**
 * @brief Print diagnostic information for temperature sensor
 *
 * @param[in] channel UART channel to output to
 *
 * @details Format: "&tp,0,diag,<raw_adc>,<conversion_factor>,<offset>"
 * @note Does not trigger a new temperature reading
 */
void temp_diag_print(uart_select_t channel);

/**
 * @brief Send temperature event message
 *
 * @param[in] event_type Type of temperature event
 * @param[in] value Temperature value associated with event in 0.1°C
 *
 * @details Format: "!tp,<id>,<event_type>,<value>"
 * @note Messages are sent to UART_UPHOLE channel
 */
void temp_send_event(temp_event_t event_type, int16_t value);

bool temp_alert_low_is_active(void);
bool temp_alert_high_is_active(void);


#endif /* MTI_TEMP_H */
