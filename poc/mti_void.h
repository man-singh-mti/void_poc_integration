/**
 * @file    mti_void.h
 * @brief   Void Detection System Driver
 * @author  Engineering Team
 * @version 1.2.0 (Converted to millimeter-based integer arithmetic)
 * @date    2025-05-26
 */

#ifndef MTI_VOID_H
#define MTI_VOID_H

#include "main.h"
#include "mti_can.h"         // For radar_data_t
#include "vmt_common_defs.h" // For MAX_SENSORS, radar_hw_status_t, system_status_t etc.
#include <stdint.h>
#include <stdbool.h>

// Sensor configuration constants
#define SENSOR_SPACING_DEGREES    120
#define WALL_HISTORY_SIZE         10
#define MIN_VALID_SAMPLES         3
#define OUTLIER_THRESHOLD_STD_DEV 2
#define MAX_VOID_DETECTIONS       10
#define MAX_GEO_MODEL_POINTS      50

// Distance limits in millimeters
#define MIN_BOREHOLE_DIAMETER_MM 1000U // 1.0m minimum borehole diameter
#define MAX_BOREHOLE_DIAMETER_MM 5000U // 5.0m maximum borehole diameter

// Default configuration constants
#define DEFAULT_BASELINE_METHOD              0U
#define DEFAULT_VOID_DETECTION_THRESHOLD_MM  50U
#define DEFAULT_MIN_SNR_THRESHOLD            30U
#define DEFAULT_CONFIDENCE_THRESHOLD_PERCENT 70U
#define DEFAULT_NOMINAL_WALL_DISTANCE_MM     2000U
#define DEFAULT_MIN_VALID_RANGE_MM           200U
#define DEFAULT_MAX_VALID_RANGE_MM           5000U
#define DEFAULT_VOID_BASELINE_METHOD         0U

// Void detection modes
typedef enum
{
    VOID_DETECTION_MODE_SINGLE     = 0U,
    VOID_DETECTION_MODE_CONTINUOUS = 1U,
    VOID_DETECTION_MODE_TRIGGERED  = 2U
} void_detection_mode_t;

// Multi-point processing algorithms
typedef enum
{
    MULTI_POINT_CLOSEST              = 0U,
    MULTI_POINT_STRONGEST            = 1U,
    MULTI_POINT_MEDIAN               = 2U,
    MULTI_POINT_AVERAGE              = 3U,
    MULTI_POINT_ALGORITHM_SEQUENTIAL = 0U, // Alias for CLOSEST
    MULTI_POINT_COUNT                = 4U
} multi_point_algorithm_t;

// Integer arithmetic helper constants
#define COLLINEARITY_THRESHOLD_SCALED 1000 // Scaled threshold for integer arithmetic
#define SQRT_APPROX_INITIAL_B         0x8000U
#define SQRT_APPROX_INITIAL_BSHIFT    15

// Communication constants
#define VOID_REPORT_MESSAGE_MAX_LENGTH 64


/**
 * @brief Status of a wall measurement from a single radar sensor.
 */
typedef enum
{
    WALL_STATUS_NO_DETECTION = 0, /**< No wall detected */
    WALL_STATUS_VALID,            /**< Valid wall detection */
    WALL_STATUS_ERROR,            /**< Hardware or measurement error */
    WALL_STATUS_OUT_OF_RANGE,     /**< Detection outside valid range */
    WALL_STATUS_SUSPICIOUS,       /**< Suspicious reading (large change) */
    WALL_STATUS_COUNT             /**< Number of status values */
} wall_status_t;

/**
 * @brief Void detection configuration parameters (all distances in millimeters)
 */
typedef struct
{
    uint8_t                 baseline_method;       /**< Baseline detection method */
    uint16_t                detection_threshold;   /**< Void detection threshold in mm */
    uint8_t                 min_snr;               /**< Minimum SNR for valid reading */
    uint8_t                 confidence_threshold;  /**< Confidence threshold (0-100%) */
    uint16_t                default_wall_distance; /**< Default wall distance in mm */
    uint16_t                range_min;             /**< Minimum valid range in mm */
    uint16_t                range_max;             /**< Maximum valid range in mm */
    uint8_t                 mode;                  /**< Operating mode */
    bool                    enable_median_filter;  /**< Enable median filtering */
    multi_point_algorithm_t multi_point_algorithm; /**< Algorithm for multi-point data */
} void_config_t;

/**
 * @brief Processed wall measurement data from a single radar (all distances in millimeters)
 */
typedef struct
{
    uint16_t      distance;  /**< Distance in millimeters */
    uint8_t       snr;       /**< Signal-to-noise ratio (0-255) */
    wall_status_t status;    /**< Measurement status */
    uint32_t      timestamp; /**< HAL_GetTick() timestamp */
} wall_measurement_t;

/**
 * @brief Data for a detected void (all measurements in millimeters)
 */
typedef struct
{
    uint8_t  sector;     /**< Sensor index/sector (0-2) */
    uint16_t size_mm;    /**< Void size/depth in millimeters */
    uint8_t  confidence; /**< Confidence percentage (0-100%) */
    uint32_t timestamp;  /**< HAL_GetTick() timestamp */
} void_detection_t;

typedef struct
{
    int32_t       center_x_mm;                   /**< Center X offset in millimeters */
    int32_t       center_y_mm;                   /**< Center Y offset in millimeters */
    uint16_t      borehole_radius_mm;            /**< Calculated borehole radius in millimeters */
    uint16_t      wall_distance_mm[MAX_SENSORS]; /**< Wall distances in millimeters */
    wall_status_t wall_status[MAX_SENSORS];      /**< Status for each sensor measurement */
    uint32_t      timestamp;                     /**< HAL_GetTick() timestamp */
} wall_profile_t;

/**
 * @brief Geometric model point (distances in millimeters)
 */
typedef struct
{
    uint16_t radius_mm; /**< Radius in millimeters */
} geo_model_point_t;

/**
 * @brief Geometric model for the borehole (distances in millimeters)
 */
typedef struct
{
    bool              loaded;                       /**< True if model is loaded and valid */
    uint16_t          point_count;                  /**< Number of valid points in model */
    geo_model_point_t points[MAX_GEO_MODEL_POINTS]; /**< Model data points */
} geo_model_t;

// Public function declarations

/**
 * @brief Initialize the void detection subsystem
 * @return true if initialization successful, false otherwise
 */
bool void_init(void);

/**
 * @brief Process void detection (called from main loop)
 */
void void_process(void);

/**
 * @brief Set expected borehole diameter for detection algorithm
 * @param[in] diameter_mm Expected borehole diameter in millimeters
 * @return true if diameter is valid and set, false otherwise
 */
bool void_set_expected_diameter(uint16_t diameter_mm);

/**
 * @brief Get latest wall measurement from specified sensor
 * @param[in] sensor_idx Sensor index (0-2)
 * @return Latest wall measurement structure
 */
wall_measurement_t void_get_latest_measurement(uint8_t sensor_idx);

/**
 * @brief Update wall measurement for specified sensor
 * @param[in] sensor_idx Sensor index (0-2)
 * @param[in] measurement Wall measurement data
 */
void void_update_measurement(uint8_t sensor_idx, wall_measurement_t measurement);

/**
 * @brief Callback function for processing radar data from CAN module
 * @param[in] sensor_idx Sensor index (0-2)
 * @param[in] radar_data_ptr Pointer to radar data structure
 */
void void_process_radar_data_callback(uint8_t sensor_idx, radar_data_t *radar_data_ptr);

/**
 * @brief Get overall void subsystem hardware status
 * @return Hardware status reflecting overall radar sensor health
 */
radar_hw_status_t void_get_subsystem_hw_status(void);

/**
 * @brief Update void detection configuration.
 *        The new configuration is validated before being applied.
 * @param[in] new_config Pointer to a void_config_t structure containing the new configuration settings.
 * @return true if the configuration was valid and applied successfully, false otherwise.
 * @note Input parameters are validated (e.g., thresholds, ranges).
 */
bool void_update_config(const void_config_t *new_config);

/**
 * @brief Get a copy of the current void detection configuration.
 * @return void_config_t The current configuration settings.
 */
void_config_t void_get_config(void);

/**
 * @brief Get the most recently reported void detection from the internal history.
 * @param[out] last_detection Pointer to a void_detection_t structure where the last
 *                            detection data will be copied.
 * @return true if a detection was available in history and copied successfully,
 *         false otherwise (e.g., history is empty or last_detection is NULL).
 * @note This relies on the detection history being enabled and populated.
 */
bool void_get_last_detection(void_detection_t *last_detection);

/**
 * @brief Retrieve a specified number of recent void detection entries from history.
 *        Entries are copied newest first.
 * @param[out] history_buffer Pointer to an array of void_detection_t where historical
 *                            data will be copied.
 * @param[in] buffer_size The maximum number of void_detection_t entries that can be
 *                        stored in history_buffer.
 * @return uint8_t The actual number of detection entries copied to history_buffer.
 *                 This can be less than buffer_size if history contains fewer entries
 *                 or if buffer_size is 0. Returns 0 if history_buffer is NULL.
 */
uint8_t void_get_detection_history(void_detection_t *history_buffer, uint8_t buffer_size);

/**
 * @brief Clears all entries from the void detection history buffer.
 * @note This action is irreversible.
 */
void void_clear_detection_history(void);

/**
 * @brief Get wall profile history
 * @param[out] profile_buffer Buffer to store profile history
 * @param[in] buffer_size Size of the buffer (max entries)
 * @return Number of profiles copied to buffer
 */
uint8_t void_get_profile_history(wall_profile_t *profile_buffer, uint8_t buffer_size);

#endif /* MTI_VOID_H */
