/**
 * @file vmt_device.h
 * @brief Header file for the VMT device, defining structures, constants, and function prototypes
 *        for device-specific operations, including IMU, ADC, SPI communication, and data logging.
 */
#ifndef __VMT_DEVICE_H__
#define __VMT_DEVICE_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "vmt_icm20948.h" // ICM20948 IMU driver
#include "vmt_adc.h"      // ADC handling
#include "vmt_uart.h"     // UART communication

#pragma anon_unions

/* Defines */
#define FW_VER_MAJOR (1) ///< Firmware version major number
#define FW_VER_MINOR (2) ///< Firmware version minor number
#define FW_VER_SUB   (4) ///< Firmware version sub number

#define DEV_IMU_NUM    (2)           ///< Number of IMU units
#define DEV_SPI_CH_NUM (DEV_IMU_NUM) ///< Number of SPI channels, typically one per IMU

#define IMU_SAMPLE_HAL_TIMER (htim7) ///< HAL Timer instance used for IMU sampling

#define DEV_ADC_SAMPLE_RATE_DEFAULT (400)                          ///< Default ADC sample rate in Hz
#define DEV_IMU_SAMPLE_RATE         (1000.0)                       ///< IMU sample rate in Hz
#define DEV_IMU_SAMPLE_PERIOD       (1000.0 / DEV_IMU_SAMPLE_RATE) ///< IMU sample period in ms

#define DEV_IMU_AVG_TIME       (200.0)                                                     ///< Time window for IMU averaging in ms
#define DEV_IMU_AVG_SAMPLE_NUM ((size_t)(DEV_IMU_SAMPLE_RATE * DEV_IMU_AVG_TIME / 1000.0)) ///< Number of samples for IMU averaging
#define DEV_IMU_AVG_BUFF_SIZE  ((size_t)DEV_IMU_AVG_SAMPLE_NUM + 1)                        ///< Buffer size for IMU averaging

#define DEV_IMU_VECTOR_AXIS_N (3) ///< Number of axes for an IMU vector (X, Y, Z)


extern const double math_pi; ///< Mathematical constant PI

/* Type Defines */

/**
 * @brief Structure for managing a moving average of int16_t values.
 */
typedef struct h_moving_avg_i16_
{
    int16_t *p_buff;   ///< Pointer to the data buffer.
    size_t   buff_len; ///< Length of the buffer.
    size_t   data_len; ///< Number of valid data points currently in the buffer.
    size_t   data_n;   ///< Current index in the circular buffer.
    double   sum;      ///< Sum of the current data points in the buffer.
    double   avg;      ///< Current average of the data points.
} h_moving_avg_i16_t;

/**
 * @brief Structure for managing a moving average of float values.
 */
typedef struct h_moving_avg_f_
{
    float *p_buff;   ///< Pointer to the data buffer.
    size_t buff_len; ///< Length of the buffer.
    size_t data_len; ///< Number of valid data points currently in the buffer.
    size_t data_n;   ///< Current index in the circular buffer.
    double sum;      ///< Sum of the current data points in the buffer.
    double avg;      ///< Current average of the data points.
} h_moving_avg_f_t;

/**
 * @brief Structure for ADC sensor data with Schmitt trigger logic.
 */
typedef struct sensor_adc_
{
    double volt;    ///< Current voltage reading.
    double thre_h;  ///< Schmitt trigger high threshold.
    double thre_l;  ///< Schmitt trigger low threshold.
    bool   b_level; ///< Current level state based on thresholds.
} h_sensor_adc_t;

/**
 * @brief Structure for holding all sensor data, primarily ADC readings.
 */
typedef struct h_sensor_
{
    union
    {
        uint16_t adc_value[ADC_SEQ_NUM]; ///< Array of raw ADC values.
        struct
        {
            uint16_t power; ///< Power ADC reading.
            union
            {
                uint16_t water[ADC_SEQ_WATER_NUM]; ///< Water sensor ADC readings.
                struct
                {
                    uint16_t water_1; ///< Water sensor 1 ADC reading.
                    uint16_t water_2; ///< Water sensor 2 ADC reading.
                };
            };
            uint16_t temp_wall; ///< Wall temperature ADC reading.
            union
            {
                uint16_t dist[ADC_SEQ_DIST_NUM]; ///< Distance sensor ADC readings.
                struct
                {
                    uint16_t dist_1; ///< Distance sensor 1 ADC reading.
                    uint16_t dist_2; ///< Distance sensor 2 ADC reading.
                    uint16_t dist_3; ///< Distance sensor 3 ADC reading.
                };
            };
        };
    };
    h_sensor_adc_t h_adc[ADC_SEQ_NUM]; ///< Array of processed ADC sensor data.
} h_sensor_t;

/**
 * @brief Function pointer type for getting clock source frequency.
 * @return Clock frequency in Hz.
 */
typedef uint32_t (*clk_source_get_t)(void);

/**
 * @brief Enumeration for IMU vector types.
 */
typedef enum imu_vector_
{
    IMU_VECTOR_ACCEL, ///< Accelerometer vector.
    IMU_VECTOR_GYRO,  ///< Gyroscope vector.
    IMU_VECTOR_MAG,   ///< Magnetometer vector.
    IMU_VECTOR_NUM,   ///< Number of IMU vector types.
} imu_vector_t;

/**
 * @brief Structure for representing an IMU vector (e.g., acceleration, gyro).
 */
typedef struct h_vector_
{
    int16_t  data_org[DEV_IMU_VECTOR_AXIS_N]; ///< Original raw data for X, Y, Z axes.
    uint32_t squ;                             ///< Square of the vector length.
    double   len;                             ///< Length (magnitude) of the vector.
    double   angle;                           ///< Angle of the vector (context-dependent).
} h_vector_t;

/**
 * @brief Structure for managing an IMU device.
 */
typedef struct h_imu_
{
    uint8_t       id;           ///< IMU identifier.
    h_icm20948_t *p_h_icm20948; ///< Pointer to the ICM20948 driver handle.
    union
    {
        int16_t data[ICM20948_DATA_NUM]; ///< Raw data array from IMU.
        struct
        {
            int16_t accel_x, accel_y, accel_z; ///< Accelerometer data (X, Y, Z).
            int16_t gyro_x, gyro_y, gyro_z;    ///< Gyroscope data (X, Y, Z).
            int16_t mag_x, mag_y, mag_z;       ///< Magnetometer data (X, Y, Z).
        };
    };

    uint8_t             step;                                 ///< Current step in the IMU processing state machine.
    bool                b_g_log_en : 1;                       ///< Flag to enable gravity vector logging.
    bool                b_g_log_busy : 1;                     ///< Flag indicating gravity logging is busy.
    bool                b_update : 1;                         ///< Flag indicating new data is available.
    bool                b_overflow : 1;                       ///< Flag indicating data overflow.
    h_moving_avg_i16_t *p_h_g_squ_avg[DEV_IMU_VECTOR_AXIS_N]; ///< Pointers to moving average for gravity square components.
    int16_t             gravity[DEV_IMU_VECTOR_AXIS_N];       ///< Calculated gravity vector.
    double              gravity_len;                          ///< Length of the gravity vector.
    h_vector_t          h_accel;                              ///< Processed accelerometer vector data.
    h_vector_t          h_gyro;                               ///< Processed gyroscope vector data.
} h_imu_t;

/**
 * @brief Structure for SPI send operation parameters.
 */
typedef struct h_dev_spi_send_ h_dev_spi_send_t;
struct h_dev_spi_send_
{
    uint8_t  channel;                             ///< SPI channel number.
    uint8_t *p_tx_buff;                           ///< Pointer to the transmit buffer.
    uint8_t *p_rx_buff;                           ///< Pointer to the receive buffer.
    size_t   data_size;                           ///< Size of the data to transmit/receive.
    void (*finish_cb)(h_dev_spi_send_t *p_h_spi); ///< Callback function upon completion.
};

/**
 * @brief Structure for IMU data within a device log entry.
 */
typedef struct h_dev_log_imu_
{
    union
    {
        int16_t accel[DEV_IMU_VECTOR_AXIS_N]; ///< Accelerometer data array.
        struct
        {
            int16_t acc_x; ///< X-axis acceleration.
            int16_t acc_y; ///< Y-axis acceleration.
            int16_t acc_z; ///< Z-axis acceleration.
        };
    };
    int16_t angle_acc;  ///< Accelerometer-derived angle (scaled by 100).
    int16_t angle_gyro; ///< Gyroscope-derived angle (scaled by 100).
} h_dev_log_imu_t;

/**
 * @brief Structure for a device log entry.
 */
typedef struct h_dev_log_
{
    uint32_t        time;               ///< Timestamp of the log entry (ms).
    h_dev_log_imu_t h_imu[DEV_IMU_NUM]; ///< IMU data for each IMU.
} h_dev_log_t;

/**
 * @brief Union for device debug flags.
 */
typedef union h_dev_debug_
{
    uint8_t byte; ///< All debug flags as a single byte.
    struct
    {
        bool b_init : 1;           ///< Enable init debug messages.
        bool b_spi_init : 1;       ///< Enable SPI init debug messages.
        bool b_imu_sample_set : 1; ///< Enable IMU sample set debug messages.
        bool b_adc_sample : 1;     ///< Enable ADC sample debug messages.
        bool b_imu_sample : 1;     ///< Enable IMU sample debug messages.
        bool b_radar_sample : 1;   ///< Enable radar sample debug messages.
        bool b_radar_verbose : 1;  ///< Enable detailed radar debug messages.
        bool b_temp_sample : 1;    ///< Enable temperature sample debug messages.
        bool b_void_sample : 1;    ///< Enable void detection sample debug messages.
    };
} h_dev_debug_t;

/**
 * @brief Enumeration for device operation results.
 */
typedef enum dev_res_
{
    DEV_RES_ERROR,      ///< General error.
    DEV_RES_SUCCESS,    ///< Operation successful.
    DEV_RES_BUSY,       ///< Device or resource is busy.
    DEV_RES_ERROR_PARA, ///< Input parameter error.
    DEV_RES_NUM,        ///< Number of device result types.
} dev_res_t;

/**
 * @brief Enumeration for log operation results.
 */
typedef enum log_res_
{
    LOG_RES_ERROR, ///< Logging error.
    LOG_RES_OK,    ///< Logging operation successful.
    LOG_RES_BUSY,  ///< Logging system is busy.
} log_res_t;

/**
 * @brief Structure to hold values to be stored in flash memory, particularly max accelerations.
 */
typedef struct
{
    int16_t acc_x;         ///< Current X-axis acceleration.
    int16_t acc_y;         ///< Current Y-axis acceleration.
    int16_t acc_z;         ///< Current Z-axis acceleration.
    int16_t acc_x_max;     ///< Maximum X-axis acceleration recorded.
    int16_t acc_y_max;     ///< Maximum Y-axis acceleration recorded.
    int16_t acc_z_max;     ///< Maximum Z-axis acceleration recorded.
    int16_t acc_y_bottom;  ///< Y-axis acceleration at bottom detection (context-specific).
    int16_t amplitude;     ///< Current acceleration amplitude (context-specific).
    int16_t amplitude_max; ///< Maximum acceleration amplitude recorded (context-specific).
} flash_values_t;


/* Function Prototypes */

/**
 * @brief Resets the maximum acceleration values stored in flash_value.
 */
void flash_data_reset(void);

/**
 * @brief Callback function called when IMU data update is finished.
 *        Processes IMU data for dump detection and logging.
 */
void imu_update_finish_cb(void);

/**
 * @brief Starts the device initialization process.
 */
void device_init_start(void);

/**
 * @brief Gets the device initialization completion status.
 * @return True if initialization is complete, false otherwise.
 */
bool device_init_finish_get(void);

/**
 * @brief Main processing loop for the device. Handles initialization, command processing,
 *        sensor data processing, and logging.
 */
void device_process(void);

/**
 * @brief Restarts device components, resetting IMU and water detection.
 */
void device_restart(void);

/**
 * @brief Sets the device debug print flags.
 * @param p_h_flag Pointer to the debug flag structure to set.
 */
void dev_printf_debug_set(h_dev_debug_t *p_h_flag);

/**
 * @brief Gets the current device debug print flags.
 * @param p_h_flag Pointer to a structure to store the current debug flags.
 */
void dev_printf_debug_get(h_dev_debug_t *p_h_flag);

/**
 * @brief Sends data over SPI.
 * @param p_h_spi Pointer to the SPI send parameters structure.
 * @return Result of the SPI send operation (dev_res_t).
 */
dev_res_t dev_spi_send(h_dev_spi_send_t *p_h_spi);

/**
 * @brief Enables or disables dump detection printf messages.
 * @param b_enable True to enable, false to disable.
 */
void dev_dump_printf_set(bool b_enable);

/**
 * @brief Gets the current dump detection status.
 * @return True if dump is detected, false otherwise.
 */
bool dev_dump_get(void);

/**
 * @brief Enables or disables water detection printf messages.
 * @param b_enable True to enable, false to disable.
 */
void dev_water_det_printf_set(bool b_enable);

/**
 * @brief Gets the current water detection status.
 * @return True if water is detected, false otherwise.
 */
bool dev_water_det_get(void);

/**
 * @brief Sets the IMU sample rate.
 * @param sample_rate The desired sample rate in Hz.
 */
void imu_sample_rate_set(double sample_rate);

/**
 * @brief Enables or disables gravity vector logging for a specific IMU.
 * @param imu_select IMU index (0 or 1).
 * @param b_enable True to enable, false to disable.
 */
void imu_g_log_en_set(uint8_t imu_select, bool b_enable);

/**
 * @brief Checks if gravity vector logging is busy for a specific IMU.
 * @param imu_select IMU index (0 or 1).
 * @return True if logging is busy or enabled, false otherwise.
 */
bool imu_g_log_busy_get(uint8_t imu_select);

/**
 * @brief Checks if an IMU data overflow has occurred.
 * @param imu_select IMU index (0 or 1).
 * @return True if no overflow, false if overflow occurred.
 */
bool imu_overflow_get(uint8_t imu_select);

/**
 * @brief Clears the IMU data overflow flag for a specific IMU.
 * @param imu_select IMU index (0 or 1).
 */
void imu_overflow_clear(uint8_t imu_select);

/**
 * @brief Initiates erasing all logs from flash.
 * @return True if erase command was accepted, false if busy.
 */
bool dev_log_erase_all_set(void);

/**
 * @brief Checks if the log erase operation is busy.
 * @return True if erase is in progress, false otherwise.
 */
bool dev_log_erase_busy_get(void);

/**
 * @brief Enables or disables saving logs to flash.
 * @param b_enable True to enable, false to disable.
 * @return True if the new state was set, false if enabling failed (e.g., no space).
 */
bool dev_log_save_enable_set(bool b_enable);

/**
 * @brief Checks if the log saving operation is busy.
 * @return True if saving is active or flash write is busy, false otherwise.
 */
bool dev_log_save_busy_get(void);

/**
 * @brief Checks if a log save overflow has occurred.
 * @return True if an overflow occurred, false otherwise.
 */
bool dev_log_save_overflow_get(void);

/**
 * @brief Sets the UART channel for log reporting.
 * @param channel The UART channel to use for reporting.
 */
void dev_log_report_uart_ch_set(uart_select_t channel);

/**
 * @brief Enables or disables reporting logs via UART.
 * @param b_enable True to enable, false to disable.
 * @return True if the new state was set, false if enabling failed (e.g., no logs).
 */
bool dev_log_report_enable_set(bool b_enable);

/**
 * @brief Checks if the log reporting operation is busy.
 * @return True if reporting is active, false otherwise.
 */
bool dev_log_report_busy_get(void);

/**
 * @brief Gets the currently loaded log data (used during report).
 * @param p_h_log Pointer to a structure to store the log data.
 */
void dev_log_data_get(h_dev_log_t *p_h_log);

/**
 * @brief Puts the device into a low-power sleep mode.
 */
void dev_sleep(void);

#endif //__VMT_DEVICE_H__
