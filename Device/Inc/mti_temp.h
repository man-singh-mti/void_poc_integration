#ifndef __MTI_TEMP_H__
#define __MTI_TEMP_H__

#include <stdint.h>
#include <stdbool.h>
#include "vmt_adc.h"

/* Temperature system configuration */
#define TEMP_DEFAULT_HIGH_THRESHOLD_C 85   // 85°C high threshold
#define TEMP_DEFAULT_LOW_THRESHOLD_C  -10  // -10°C low threshold
#define TEMP_SMOOTHING_SAMPLES        10   // Number of samples for smoothing
#define TEMP_INVALID_READING          -999 // Invalid temperature marker

/* Temperature data structures */
typedef struct
{
    int16_t temperature_raw; // Raw ADC reading
    int32_t timestamp_ms;    // When reading was taken
    bool    data_valid;      // Raw data validity flag
} temp_raw_data_t;

typedef struct
{
    int16_t  temperature_c;   // Cleaned temperature value in Celsius
    bool     temp_high_flag;  // Over temperature warning
    bool     temp_low_flag;   // Under temperature warning
    bool     data_valid;      // Cleaned data validity
    uint32_t process_time_ms; // When processing completed
} temp_processed_data_t;

typedef struct
{
    int16_t  current_temperature; // Latest temperature reading in Celsius
    bool     high_temp_alert;     // Combined high temperature alert
    bool     low_temp_alert;      // Combined low temperature alert
    bool     system_ready;        // Temperature system operational
    uint32_t last_update_ms;      // Last successful update
} temp_status_t;

/* Function declarations */
void temp_system_process(void);
bool temp_get_raw_data(temp_raw_data_t *raw_data);
bool temp_process_raw_data(temp_raw_data_t *raw, temp_processed_data_t *processed);
void temp_update_flags(temp_processed_data_t *processed, temp_status_t *status);
void temp_get_latest_status(temp_status_t *status);
bool temp_init(void);

/* Configuration functions */
void    temp_set_high_threshold(int16_t threshold_c);
void    temp_set_low_threshold(int16_t threshold_c);
int16_t temp_get_high_threshold(void);
int16_t temp_get_low_threshold(void);

/* Command interface functions */
void handle_temp_status_command(void);
void handle_temp_config_command(const char *params);

#endif /* __MTI_TEMP_H__ */
