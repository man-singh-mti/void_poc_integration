#ifndef MTI_VOID_H
#define MTI_VOID_H

#include "stm32f7xx.h"
#include "mti_radar.h"
#include <stdbool.h>
#include <stdint.h>

// Void detection constants
#define MAX_RADAR_SENSORS         3U
#define VOID_HISTORY_SIZE         10U
#define VOID_DEFAULT_BASELINE_MM  150U // Default expected borehole diameter (mm)
#define VOID_DEFAULT_THRESHOLD_MM 50U  // Default void detection threshold (mm)
#define VOID_MIN_CONFIDENCE       70U  // Minimum confidence for void detection
#define VOID_PROCESS_INTERVAL_MS  100U // Process void detection every 100ms

// Void severity levels
typedef enum
{
    VOID_SEVERITY_NONE     = 0,
    VOID_SEVERITY_MINOR    = 1,
    VOID_SEVERITY_MAJOR    = 2,
    VOID_SEVERITY_CRITICAL = 3
} void_severity_t;

// Void detection configuration
typedef struct
{
    uint16_t baseline_diameter_mm;   // Expected borehole diameter
    uint16_t detection_threshold_mm; // Threshold above baseline for void detection
    uint8_t  confidence_threshold;   // Minimum confidence required (0-100)
    bool     median_filter_enabled;  // Enable median filtering
    uint16_t range_min_mm;           // Minimum valid distance
    uint16_t range_max_mm;           // Maximum valid distance
} void_config_t;

// Single void measurement from one sensor
typedef struct
{
    uint16_t distance_mm[MAX_RADAR_SENSORS]; // Distances in mm from each sensor
    uint16_t angle_deg[MAX_RADAR_SENSORS];   // Angles (0, 120, 240 degrees)
    bool     data_valid[MAX_RADAR_SENSORS];  // Valid flags for each sensor
    uint32_t measurement_time_ms;            // Timestamp when measurement taken
} void_measurement_t;

// Void detection result
typedef struct
{
    bool            void_detected;        // Current void state
    uint8_t         void_sector;          // Which sensor detected void (0-2)
    uint16_t        void_diameter_mm;     // Calculated void diameter
    uint16_t        baseline_diameter_mm; // Expected baseline diameter
    void_severity_t void_severity;        // Severity classification
    uint8_t         confidence_percent;   // Detection confidence (0-100)
    uint32_t        detection_time_ms;    // When void was detected
    char            status_text[32];      // Human-readable status
} void_status_t;

// System-wide void detection state
typedef struct
{
    void_config_t      config;                     // Current configuration
    void_measurement_t latest_measurement;         // Latest sensor data
    void_status_t      current_status;             // Current detection status
    void_status_t      history[VOID_HISTORY_SIZE]; // Historical detections
    uint8_t            history_count;              // Number of entries in history
    uint32_t           last_process_time_ms;       // Last processing timestamp
    bool               system_initialized;         // Initialization flag
} void_system_state_t;

// Core void detection functions
void void_system_init(void);
void void_system_process(void);

// Data analysis functions
bool    void_analyze_sensor_data(uint8_t sensor_idx, uint16_t distance_mm, uint16_t angle_deg, void_status_t *result);
uint8_t void_calculate_confidence(uint16_t distance_mm, uint16_t expected_mm, uint16_t threshold_mm);
void    void_characterise_detection(uint16_t distance_mm, uint16_t expected_mm, void_status_t *result);

// Configuration functions
void void_set_threshold(uint16_t threshold_mm);
void void_set_baseline(uint16_t baseline_mm);
void void_set_confidence_threshold(uint8_t confidence_percent);
void void_set_range(uint16_t min_mm, uint16_t max_mm);
void void_set_median_filter(bool enabled);

// Status and data retrieval functions
void    void_get_latest_results(void_status_t *result);
void    void_get_current_config(void_config_t *config);
bool    void_get_measurement_data(void_measurement_t *measurement);
uint8_t void_get_history_count(void);
bool    void_get_history_entry(uint8_t index, void_status_t *entry);

// Utility functions
void        void_clear_history(void);
bool        void_is_system_ready(void);
const char *void_get_severity_string(void_severity_t severity);

#endif // MTI_VOID_H
