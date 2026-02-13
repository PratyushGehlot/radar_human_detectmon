/**
 * @file radar_sensor.c
 * @brief Radar Human Detection & Fall Monitor
 * @details ESP32-S3-BOX-3 based real-time human presence detection and fall
 *          monitoring system using LD6001 mmWave radar sensor.
 * @author PratyushGehlot
 * @see https://github.com/PratyushGehlot/radar_human_detectmon
 */

/* 
   Radar sensor handling code for ESP32-S3 Box3 application.
   This module interfaces with the radar sensor over UART, processes the raw data to detect human presence and posture, 
   and provides a callback mechanism to report detected targets.

   The radar sensor is expected to provide point cloud data with x,y,z coordinates, velocity, SNR, absolute value, and DPK for each detected point. 
   The processing includes filtering based on confidence, clustering points into targets, and classifying human posture based on height and movement.

   The configuration parameters allow tuning of the detection algorithm for better accuracy in different environments. 
*/

#include "radar_sensor.h"
#include "wifi_stream.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"

#define GRID_X 6.0f
#define GRID_Y 6.0f
#define GRID_Z 2.5f//3.0f

#define FALL_CONFIRMATION_THRESHOLD 2

static const char *TAG = "radar_sensor";

#define RADAR_UART_NUM UART_NUM_1

static radar_config_t s_config;
static radar_point_t s_frame_points[RADAR_MAX_POINTS];
static int s_frame_point_count = 0;
static int s_frame_counter = 0;
static human_target_t s_targets[RADAR_MAX_TARGETS];
static int s_target_count = 0;
static SemaphoreHandle_t s_data_mutex = NULL;
static TaskHandle_t s_radar_task_handle = NULL;
static bool s_running = false;

typedef enum {
    FALL_NONE = 0,
    FALL_SUSPECT,
    FALL_CONFIRMED,
    FALL_COOLDOWN,
} fall_state_t;

typedef struct {
    bool active;
    uint8_t id;
    int missed_frames;

    float x, y, z;
    float z_max;
    float z_range;
    float xy_span;
    float v;
    float v_var;

    float prev_z;
    float prev_v;
    int64_t prev_t_us;

    fall_state_t fall_state;
    uint8_t fall_counter;
    int64_t fall_confirmed_t_us;
    int recovery_counter;

    human_posture_t prev_posture;
} target_track_t;

static target_track_t s_tracks[RADAR_MAX_TARGETS];
static uint8_t s_next_track_id = 1;

typedef struct {
    float cx, cy, cz;
    float z_min, z_max, z_range;
    float xy_span_x, xy_span_y, xy_span;
    float v_mean, v_var;
    float confidence;
    int point_count;
} cluster_features_t;

esp_err_t radar_send_command(const char *cmd, char *response, size_t resp_size, int timeout_ms)
{
    int len = strlen(cmd);
    uart_write_bytes(RADAR_UART_NUM, cmd, len);
    ESP_LOGD(TAG, "Sent command: %s", cmd);

    if (response != NULL && resp_size > 0) {
        int rx_len = uart_read_bytes(RADAR_UART_NUM, (uint8_t *)response, resp_size - 1,
                                     timeout_ms / portTICK_PERIOD_MS);
        if (rx_len < 0) {
            rx_len = 0;
        }
        response[rx_len] = '\0';
        ESP_LOGD(TAG, "Response: %s", response);

        if (strstr(response, "OK") != NULL) {
            return ESP_OK;
        }
        return ESP_FAIL;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t radar_configure(void)
{
    char resp[64];
    esp_err_t ret;

#if 0
    ret = radar_send_command("AT+STOP\n", resp, sizeof(resp), 200);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AT+STOP failed");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);

    ret = radar_send_command("AT+PROG=02\n", resp, sizeof(resp), 200);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AT+PROG=02 failed");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);

    ret = radar_send_command("AT+DEBUG=0\n", resp, sizeof(resp), 200);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AT+DEBUG=0 failed");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);

    ret = radar_send_command("AT+HEATIME=60\n", resp, sizeof(resp), 200);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AT+HEATIME=60 failed");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);

    ret = radar_send_command("AT+SENS=3\n", resp, sizeof(resp), 200);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AT+SENS=3 failed");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
#endif
    return ESP_OK;
}

esp_err_t radar_start(void)
{
    esp_err_t ret = radar_configure();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "radar_configure returned error, continuing anyway");
    }

    ESP_LOGW(TAG, "AT+START\n");
    char resp[64];
    ret = radar_send_command("AT+START\n", resp, sizeof(resp), 500);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AT+START failed");
    }
    else 
    {
        ESP_LOGI(TAG, "AT+START succeeded");
    }
    return ret;
}

esp_err_t radar_stop(void)
{
    char resp[64];
    esp_err_t ret = radar_send_command("AT+STOP\n", resp, sizeof(resp), 500);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AT+STOP failed");
    }
    else 
    {
        ESP_LOGI(TAG, "AT+STOP succeeded");
    }
    return ret;
}

static float point_confidence(float snr, float abs_val, float dpk)
{
    float snr_c = fminf(snr / 40.0f, 1.0f);
    float abs_c = fminf(abs_val / 15.0f, 1.0f);
    float dpk_c = fminf(dpk / 10.0f, 1.0f);
    return 0.45f * snr_c + 0.40f * abs_c + 0.15f * dpk_c;
}

static float distance_2d(float x1, float y1, float x2, float y2)
{
    float dx = x1 - x2;
    float dy = y1 - y2;
    return sqrtf(dx * dx + dy * dy);
}

static void compute_cluster_features(int cluster_id[], int filtered_idx[], int filtered_count,
                                     int cid, cluster_features_t *feat)
{
    float sum_x = 0, sum_y = 0, sum_z = 0, sum_conf = 0;
    float x_min = 1e9f, x_max = -1e9f;
    float y_min = 1e9f, y_max = -1e9f;
    float z_min = 1e9f, z_max = -1e9f;
    float v_mean = 0, v_M2 = 0;
    int n = 0;

    for (int i = 0; i < filtered_count; i++) {
        if (cluster_id[i] != cid) continue;
        radar_point_t *p = &s_frame_points[filtered_idx[i]];

        sum_x += p->x;
        sum_y += p->y;
        sum_z += p->z;
        sum_conf += point_confidence(p->snr, p->abs_val, p->dpk);

        if (p->x < x_min) x_min = p->x;
        if (p->x > x_max) x_max = p->x;
        if (p->y < y_min) y_min = p->y;
        if (p->y > y_max) y_max = p->y;
        if (p->z < z_min) z_min = p->z;
        if (p->z > z_max) z_max = p->z;

        n++;
        float dv = p->velocity - v_mean;
        v_mean += dv / n;
        v_M2 += dv * (p->velocity - v_mean);
    }

    feat->point_count = n;
    if (n == 0) return;

    feat->cx = sum_x / n;
    feat->cy = sum_y / n;
    feat->cz = sum_z / n;
    feat->z_min = z_min;
    feat->z_max = z_max;
    feat->z_range = z_max - z_min;
    feat->xy_span_x = x_max - x_min;
    feat->xy_span_y = y_max - y_min;
    feat->xy_span = fmaxf(feat->xy_span_x, feat->xy_span_y);
    feat->v_mean = v_mean;
    feat->v_var = (n > 1) ? (v_M2 / (n - 1)) : 0.0f;

    float mean_conf = sum_conf / n;
    feat->confidence = 0.6f * mean_conf + 0.4f * fminf((float)n / 15.0f, 1.0f);
}

static human_posture_t classify_posture(const cluster_features_t *f, const radar_config_t *cfg)
{
    float height = f->z_max;

    if (height >= cfg->standing_z && f->z_range > 0.4f && f->xy_span < 1.2f) {
        return POSTURE_STANDING;
    }
    if (height >= cfg->sitting_z && f->z_range > 0.2f) {
        return POSTURE_SITTING;
    }

    bool is_lying = (height < cfg->lying_z + 0.2f) ||
                    (f->z_range < cfg->fall_z_range_lying && f->xy_span > cfg->fall_xy_spread_lying);

    if (is_lying) {
        if (fabsf(f->v_mean) < cfg->v_move_threshold && f->v_var < cfg->v_move_threshold * cfg->v_move_threshold) {
            return POSTURE_SLEEPING;
        }
        return POSTURE_LYING;
    }

    if (height >= cfg->sitting_z) {
        return POSTURE_SITTING;
    }
    return POSTURE_LYING;
}

static int allocate_track(void)
{
    for (int i = 0; i < RADAR_MAX_TARGETS; i++) {
        if (!s_tracks[i].active) return i;
    }
    return -1;
}

static void update_track_fall_state(target_track_t *tr, const cluster_features_t *f,
                                    human_posture_t base_posture, int64_t now)
{
    float dt = 0.0f;
    float accel = 0.0f;
    float z_drop = 0.0f;
    float height_drop = 0.0f;

    if (tr->prev_t_us > 0) {
        dt = (float)(now - tr->prev_t_us) * 1e-6f;
        if (dt > 0.0f && dt < 2.0f) {
            z_drop = tr->prev_z - f->cz;
            accel = (f->v_mean - tr->prev_v) / dt;
            height_drop = tr->z_max - f->z_max;
        }
    }

    ESP_LOGD(TAG, "Fall[%d]: st=%d z=%.2f zmax=%.2f zdrop=%.2f hdrop=%.2f v=%.2f a=%.2f base=%d",
             tr->id, tr->fall_state, f->cz, f->z_max, z_drop, height_drop, f->v_mean, accel, base_posture);

    switch (tr->fall_state) {
    case FALL_NONE: {
        int evidence = 0;
        if (f->v_mean < s_config.fall_v_threshold) evidence++;
        if (z_drop > s_config.fall_z_drop_threshold) evidence++;
        if (accel < s_config.fall_accel_threshold) evidence++;
        if (height_drop > s_config.fall_height_collapse) evidence++;
        bool was_upright = (tr->prev_posture == POSTURE_STANDING || tr->prev_posture == POSTURE_SITTING);
        if (was_upright && (base_posture == POSTURE_LYING || base_posture == POSTURE_SLEEPING)) evidence++;

        if (evidence >= 2) {
            tr->fall_state = FALL_SUSPECT;
            tr->fall_counter = 1;
            ESP_LOGI(TAG, "Fall[%d]: NONE->SUSPECT evidence=%d", tr->id, evidence);
        }
        break;
    }
    case FALL_SUSPECT: {
        int evidence = 0;
        if (f->v_mean < s_config.fall_v_threshold) evidence++;
        if (z_drop > s_config.fall_z_drop_threshold * 0.5f) evidence++;
        if (f->z_max < s_config.sitting_z) evidence++;
        if (base_posture == POSTURE_LYING || base_posture == POSTURE_SLEEPING) evidence++;

        if (evidence >= 1) {
            tr->fall_counter++;
            if (tr->fall_counter >= FALL_CONFIRMATION_THRESHOLD) {
                tr->fall_state = FALL_CONFIRMED;
                tr->fall_confirmed_t_us = now;
                tr->recovery_counter = 0;
                ESP_LOGW(TAG, "Fall[%d]: CONFIRMED counter=%d", tr->id, tr->fall_counter);
            }
        } else {
            if (tr->fall_counter > 0) tr->fall_counter--;
            if (tr->fall_counter == 0) {
                tr->fall_state = FALL_NONE;
            }
        }
        break;
    }
    case FALL_CONFIRMED: {
        int64_t elapsed = now - tr->fall_confirmed_t_us;
        if (elapsed < s_config.fall_hold_time_us) break;

        if (base_posture == POSTURE_STANDING || base_posture == POSTURE_SITTING) {
            tr->recovery_counter++;
            if (tr->recovery_counter >= s_config.fall_recovery_frames) {
                tr->fall_state = FALL_COOLDOWN;
                tr->fall_counter = 0;
                tr->recovery_counter = 0;
            }
        } else {
            tr->recovery_counter = 0;
        }
        break;
    }
    case FALL_COOLDOWN:
        if (base_posture == POSTURE_STANDING || base_posture == POSTURE_SITTING) {
            tr->fall_state = FALL_NONE;
        }
        break;
    }
}

static void detect_humans(void)
{
    int filtered_idx[RADAR_MAX_POINTS];
    int filtered_count = 0;

    for (int i = 0; i < s_frame_point_count; i++) {
        radar_point_t *p = &s_frame_points[i];
        float conf = point_confidence(p->snr, p->abs_val, p->dpk);
        if (conf >= 0.4f) {
            filtered_idx[filtered_count++] = i;
        }
    }

    int cluster_id[RADAR_MAX_POINTS];
    for (int i = 0; i < filtered_count; i++) {
        cluster_id[i] = -1;
    }

    int next_cluster = 0;

    for (int i = 0; i < filtered_count; i++) {
        if (cluster_id[i] != -1) continue;

        int neighbors[RADAR_MAX_POINTS];
        int neighbor_count = 0;

        radar_point_t *pi = &s_frame_points[filtered_idx[i]];
        for (int j = 0; j < filtered_count; j++) {
            radar_point_t *pj = &s_frame_points[filtered_idx[j]];
            if (distance_2d(pi->x, pi->y, pj->x, pj->y) <= s_config.eps) {
                neighbors[neighbor_count++] = j;
            }
        }

        if (neighbor_count < s_config.min_samples) continue;

        int cid = next_cluster++;
        for (int n = 0; n < neighbor_count; n++) {
            cluster_id[neighbors[n]] = cid;
        }

        for (int iter = 0; iter < 3; iter++) {
            for (int k = 0; k < filtered_count; k++) {
                if (cluster_id[k] != cid) continue;
                radar_point_t *pk = &s_frame_points[filtered_idx[k]];
                for (int m = 0; m < filtered_count; m++) {
                    if (cluster_id[m] != -1) continue;
                    radar_point_t *pm = &s_frame_points[filtered_idx[m]];
                    if (distance_2d(pk->x, pk->y, pm->x, pm->y) <= s_config.eps) {
                        cluster_id[m] = cid;
                    }
                }
            }
        }
    }

    cluster_features_t feats[RADAR_MAX_TARGETS];
    int feat_count = 0;
    for (int c = 0; c < next_cluster && feat_count < RADAR_MAX_TARGETS; c++) {
        memset(&feats[feat_count], 0, sizeof(cluster_features_t));
        compute_cluster_features(cluster_id, filtered_idx, filtered_count, c, &feats[feat_count]);
        if (feats[feat_count].point_count > 0 && feats[feat_count].confidence >= s_config.human_conf_threshold) {
            feat_count++;
        }
    }

    float gate_sq = s_config.track_gate_radius * s_config.track_gate_radius;
    bool track_matched[RADAR_MAX_TARGETS] = {false};
    int feat_to_track[RADAR_MAX_TARGETS];
    for (int i = 0; i < RADAR_MAX_TARGETS; i++) feat_to_track[i] = -1;

    for (int fi = 0; fi < feat_count; fi++) {
        int best = -1;
        float best_d2 = gate_sq;
        for (int ti = 0; ti < RADAR_MAX_TARGETS; ti++) {
            if (!s_tracks[ti].active || track_matched[ti]) continue;
            float dx = s_tracks[ti].x - feats[fi].cx;
            float dy = s_tracks[ti].y - feats[fi].cy;
            float d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best = ti;
            }
        }
        if (best >= 0) {
            track_matched[best] = true;
            feat_to_track[fi] = best;
        }
    }

    for (int ti = 0; ti < RADAR_MAX_TARGETS; ti++) {
        if (!s_tracks[ti].active) continue;
        if (!track_matched[ti]) {
            s_tracks[ti].missed_frames++;
            if (s_tracks[ti].missed_frames > s_config.track_miss_limit) {
                s_tracks[ti].active = false;
            }
        }
    }

    int64_t now = esp_timer_get_time();
    human_target_t new_targets[RADAR_MAX_TARGETS];
    int new_target_count = 0;

    for (int fi = 0; fi < feat_count; fi++) {
        cluster_features_t *f = &feats[fi];
        int ti = feat_to_track[fi];

        if (ti < 0) {
            ti = allocate_track();
            if (ti < 0) continue;
            memset(&s_tracks[ti], 0, sizeof(target_track_t));
            s_tracks[ti].active = true;
            s_tracks[ti].id = s_next_track_id++;
            s_tracks[ti].prev_posture = POSTURE_UNKNOWN;
        }

        target_track_t *tr = &s_tracks[ti];
        tr->missed_frames = 0;

        human_posture_t base_posture = classify_posture(f, &s_config);
        update_track_fall_state(tr, f, base_posture, now);

        human_posture_t final_posture = base_posture;
        if (tr->fall_state == FALL_CONFIRMED) {
            final_posture = POSTURE_FALL;
        } else if (tr->fall_state == FALL_SUSPECT) {
            final_posture = base_posture;
        }

        #define EMA_ALPHA 0.4f
        tr->x  = (tr->prev_t_us > 0) ? (EMA_ALPHA * f->cx + (1.0f - EMA_ALPHA) * tr->x)  : f->cx;
        tr->y  = (tr->prev_t_us > 0) ? (EMA_ALPHA * f->cy + (1.0f - EMA_ALPHA) * tr->y)  : f->cy;
        tr->z  = (tr->prev_t_us > 0) ? (EMA_ALPHA * f->cz + (1.0f - EMA_ALPHA) * tr->z)  : f->cz;
        tr->v  = (tr->prev_t_us > 0) ? (EMA_ALPHA * f->v_mean + (1.0f - EMA_ALPHA) * tr->v) : f->v_mean;
        tr->z_max   = (tr->prev_t_us > 0) ? (EMA_ALPHA * f->z_max + (1.0f - EMA_ALPHA) * tr->z_max) : f->z_max;
        tr->z_range = (tr->prev_t_us > 0) ? (EMA_ALPHA * f->z_range + (1.0f - EMA_ALPHA) * tr->z_range) : f->z_range;
        tr->xy_span = (tr->prev_t_us > 0) ? (EMA_ALPHA * f->xy_span + (1.0f - EMA_ALPHA) * tr->xy_span) : f->xy_span;
        tr->v_var   = (tr->prev_t_us > 0) ? (EMA_ALPHA * f->v_var + (1.0f - EMA_ALPHA) * tr->v_var) : f->v_var;

        tr->prev_z = f->cz;
        tr->prev_v = f->v_mean;
        tr->prev_t_us = now;
        tr->prev_posture = final_posture;

        human_target_t *t = &new_targets[new_target_count++];
        t->present = true;
        t->posture = final_posture;
        t->center_x = tr->x;
        t->center_y = tr->y;
        t->center_z = tr->z;
        t->avg_velocity = tr->v;
        t->confidence = f->confidence;
        t->point_count = f->point_count;
    }

    ESP_LOGD(TAG, "Detection: %d filtered from %d points, %d clusters, %d targets",
             filtered_count, s_frame_point_count, next_cluster, new_target_count);
    for (int i = 0; i < new_target_count; i++) {
        ESP_LOGI(TAG, "Target[%d]: %s (%.2f,%.2f,%.2f) v=%.2f conf=%.2f pts=%d",
                 i, radar_posture_to_string(new_targets[i].posture),
                 new_targets[i].center_x, new_targets[i].center_y, new_targets[i].center_z,
                 new_targets[i].avg_velocity, new_targets[i].confidence, new_targets[i].point_count);
    }

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(s_targets, new_targets, sizeof(human_target_t) * new_target_count);
    s_target_count = new_target_count;
    xSemaphoreGive(s_data_mutex);

    if (s_config.detection_cb != NULL && new_target_count > 0) {
        s_config.detection_cb(new_targets, new_target_count);
    }
}

static bool parse_point_line(const char *line, radar_point_t *point)
{
    const char *px = strstr(line, "x=");
    const char *py = strstr(line, "y=");
    const char *pz = strstr(line, "z=");
    const char *pv = strstr(line, "v=");
    const char *psnr = strstr(line, "snr=");
    const char *pabs = strstr(line, "abs=");
    const char *pdpk = strstr(line, "dpk=");

    if (!px || !py || !pz || !pv || !psnr || !pabs || !pdpk) {
        return false;
    }

    //Parse raw values 
    float raw_x = (float)atof(px + 2);
    float raw_y = (float)atof(py + 2);
    float raw_z = (float)atof(pz + 2);

    //Transform coordinates 
    point->x = raw_x + (GRID_X / 2.0f);// center x-axis
    point->y = raw_y + (GRID_Y / 2.0f); // Center y-axis
    point->z = GRID_Z - raw_z; // z is already height from ground, invert Z axis

    point->velocity = (float)atof(pv + 2);
    point->snr = (float)atof(psnr + 4);
    point->abs_val = (float)atof(pabs + 4);
    point->dpk = (float)atof(pdpk + 4);

    return true;
}

static void radar_rx_task(void *arg)
{
    uint8_t *rx_buf = (uint8_t *)malloc(256);
    char *line_buf = (char *)malloc(512);
    int line_pos = 0;

    if (!rx_buf || !line_buf) {
        ESP_LOGE(TAG, "Failed to allocate radar rx buffers");
        free(rx_buf);
        free(line_buf);
        vTaskDelete(NULL);
        return;
    }

    while (s_running) {
        int rx_len = uart_read_bytes(RADAR_UART_NUM, rx_buf, 255, 100 / portTICK_PERIOD_MS);
        if (rx_len <= 0) {
            continue;
        }

        for (int i = 0; i < rx_len; i++) {
            char ch = (char)rx_buf[i];
            if (ch == '\n') {
                line_buf[line_pos] = '\n';
                line_buf[line_pos + 1] = '\0';
                wifi_stream_send(line_buf, line_pos + 1);
                line_buf[line_pos] = '\0';

                if (strstr(line_buf, "-----PointNum") != NULL) {
                    s_frame_counter++;
                    ESP_LOGD(TAG, "Frame #%d received, %d points", s_frame_counter, s_frame_point_count);
                    detect_humans();
                    s_frame_point_count = 0;
                } else {
                    radar_point_t point;
                    if (parse_point_line(line_buf, &point) && s_frame_point_count < RADAR_MAX_POINTS) {
                        s_frame_points[s_frame_point_count++] = point;
                        ESP_LOGD(TAG, "Point[%d]: x=%.2f y=%.2f z=%.2f v=%.2f snr=%.1f abs=%.1f dpk=%.1f",
                                 s_frame_point_count - 1, point.x, point.y, point.z,
                                 point.velocity, point.snr, point.abs_val, point.dpk);
                    }
                }

                line_pos = 0;
            } else if (line_pos < 511) {
                line_buf[line_pos++] = ch;
            }
        }
    }

    free(rx_buf);
    free(line_buf);
    vTaskDelete(NULL);
}

static esp_err_t radar_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = RADAR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(RADAR_UART_NUM, RADAR_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(RADAR_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(RADAR_UART_NUM, RADAR_UART_TXD_PIN, RADAR_UART_RXD_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART initialized on UART%d (TX:%d, RX:%d)",
             RADAR_UART_NUM, RADAR_UART_TXD_PIN, RADAR_UART_RXD_PIN);
    return ESP_OK;
}

esp_err_t radar_sensor_init(const radar_config_t *config)
{
    if (config != NULL) {
        s_config = *config;
    } else {
        radar_config_t defaults = RADAR_CONFIG_DEFAULT();
        s_config = defaults;
    }

    esp_err_t err = radar_uart_init();
    if (err != ESP_OK) {
        return err;
    }

    s_data_mutex = xSemaphoreCreateMutex();
    if (s_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    s_running = true;

    BaseType_t ret = xTaskCreate(radar_rx_task, "radar_rx_task", 8192, NULL, 10, &s_radar_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create radar rx task");
        vSemaphoreDelete(s_data_mutex);
        s_data_mutex = NULL;
        s_running = false;
        return ESP_FAIL;
    }

    esp_err_t err1 = radar_start();
    if (err1 != ESP_OK) {
        ESP_LOGW(TAG, "radar_start returned error: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Radar sensor initialization complete");
    return ESP_OK;
}

const human_target_t *radar_get_targets(int *count)
{
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    if (count != NULL) {
        *count = s_target_count;
    }
    xSemaphoreGive(s_data_mutex);
    return s_targets;
}

human_posture_t radar_get_primary_posture(void)
{
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    if (s_target_count == 0) {
        xSemaphoreGive(s_data_mutex);
        return POSTURE_NO_PRESENCE;
    }

    int best = 0;
    for (int i = 1; i < s_target_count; i++) {
        if (s_targets[i].confidence > s_targets[best].confidence) {
            best = i;
        }
    }
    human_posture_t posture = s_targets[best].posture;
    xSemaphoreGive(s_data_mutex);
    return posture;
}

const char *radar_posture_to_string(human_posture_t posture)
{
    switch (posture) {
    case POSTURE_STANDING:    return "Standing";
    case POSTURE_SITTING:     return "Sitting";
    case POSTURE_LYING:       return "Lying";
    case POSTURE_SLEEPING:    return "Sleeping";
    case POSTURE_FALL:        return "Fall Detected";
    case POSTURE_NO_PRESENCE: return "No Presence";
    default:                  return "Unknown";
    }
}

void radar_sensor_deinit(void)
{
    s_running = false;
    vTaskDelay(500 / portTICK_PERIOD_MS);

    radar_stop();

    if (s_data_mutex != NULL) {
        vSemaphoreDelete(s_data_mutex);
        s_data_mutex = NULL;
    }

    s_radar_task_handle = NULL;

    uart_driver_delete(RADAR_UART_NUM);

    ESP_LOGI(TAG, "Radar sensor deinitialized");
}
