#ifndef RADAR_SENSOR_H
#define RADAR_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define RADAR_MAX_POINTS    128
#define RADAR_MAX_TARGETS   5

#define RADAR_UART_NUM      UART_NUM_1
#define RADAR_UART_TXD_PIN  38
#define RADAR_UART_RXD_PIN  39
#define RADAR_UART_BAUD     115200
#define RADAR_UART_BUF_SIZE 1024

typedef struct {
    float x;
    float y;
    float z;
    float velocity;
    float snr;
    float abs_val;
    float dpk;
} radar_point_t;

typedef enum {
    POSTURE_UNKNOWN = 0,
    POSTURE_STANDING,
    POSTURE_SITTING,
    POSTURE_LYING,
    POSTURE_SLEEPING,
    POSTURE_FALL,
    POSTURE_NO_PRESENCE,
} human_posture_t;

typedef struct {
    bool present;
    human_posture_t posture;
    float center_x;
    float center_y;
    float center_z;
    float avg_velocity;
    float confidence;
    int point_count;
} human_target_t;

typedef void (*radar_detection_cb_t)(const human_target_t *targets, int target_count);

typedef struct {
    float eps;
    int min_samples;
    float human_conf_threshold;
    float v_move_threshold;
    float fall_v_threshold;
    float standing_z;
    float sitting_z;
    float lying_z;
    float fall_z_drop_threshold;
    float fall_accel_threshold;
    float fall_height_collapse;
    float fall_xy_spread_lying;
    float fall_z_range_lying;
    float fall_hold_time_us;
    int   fall_recovery_frames;
    float track_gate_radius;
    int   track_miss_limit;
    radar_detection_cb_t detection_cb;
} radar_config_t;

#define RADAR_CONFIG_DEFAULT() { \
    .eps = 0.55f, \
    .min_samples = 5, \
    .human_conf_threshold = 0.3f, \
    .v_move_threshold = 0.05f, \
    .fall_v_threshold = -0.3f, \
    .standing_z = 1.0f, \
    .sitting_z = 0.6f, \
    .lying_z = 0.25f, \
    .fall_z_drop_threshold = 0.10f, \
    .fall_accel_threshold = -1.0f, \
    .fall_height_collapse = 0.25f, \
    .fall_xy_spread_lying = 0.8f, \
    .fall_z_range_lying = 0.25f, \
    .fall_hold_time_us = 5000000, \
    .fall_recovery_frames = 5, \
    .track_gate_radius = 0.8f, \
    .track_miss_limit = 5, \
    .detection_cb = NULL, \
}

esp_err_t radar_sensor_init(const radar_config_t *config);
esp_err_t radar_send_command(const char *cmd, char *response, size_t resp_size, int timeout_ms);
esp_err_t radar_start(void);
esp_err_t radar_stop(void);
esp_err_t radar_configure(void);
const human_target_t *radar_get_targets(int *count);
human_posture_t radar_get_primary_posture(void);
const char *radar_posture_to_string(human_posture_t posture);
void radar_sensor_deinit(void);

#endif
