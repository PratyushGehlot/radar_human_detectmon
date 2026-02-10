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
typedef struct {
    float prev_z;
    float prev_v;
    int64_t last_update_time;
    bool tracking;
} fall_tracker_t;

typedef struct {
    human_posture_t current_posture;
    human_posture_t candidate_posture;
    int confirmation_frames;
} posture_tracker_t;

typedef struct {
    bool human_present;
    int no_human_frames;
    int human_present_frames;
    int64_t last_human_time;
} presence_tracker_t;

static const char *TAG = "radar_sensor";

static radar_config_t s_config;
static radar_point_t s_frame_points[RADAR_MAX_POINTS];
static int s_frame_point_count = 0;
static int s_frame_counter = 0;
static human_target_t s_targets[RADAR_MAX_TARGETS];
static int s_target_count = 0;
static SemaphoreHandle_t s_data_mutex = NULL;
static TaskHandle_t s_radar_task_handle = NULL;
static bool s_running = false;

static fall_tracker_t s_fall_tracker = {0};
static posture_tracker_t s_posture_tracker = {
    .current_posture = POSTURE_NO_PRESENCE,
    .candidate_posture = POSTURE_NO_PRESENCE,
    .confirmation_frames = 0
};
static presence_tracker_t s_presence_tracker = {
    .human_present = false,
    .no_human_frames = 0,
    .human_present_frames = 0,
    .last_human_time = 0
};

static int s_fall_confirmation_frames = 0;
#define FALL_CONFIRMATION_THRESHOLD 2
#define FALL_DROP_RATE_THRESHOLD 0.4f
#define POSTURE_CONFIRMATION_THRESHOLD 3
#define POSTURE_HYSTERESIS 0.05f
#define NO_HUMAN_CONFIRMATION_THRESHOLD 5
#define HUMAN_PRESENT_CONFIRMATION_THRESHOLD 2
#define HUMAN_PRESENCE_TIMEOUT_MS 3000
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
        if (cluster_id[i] != -1) {
            continue;
        }

        int neighbors[RADAR_MAX_POINTS];
        int neighbor_count = 0;

        radar_point_t *pi = &s_frame_points[filtered_idx[i]];
        for (int j = 0; j < filtered_count; j++) {
            radar_point_t *pj = &s_frame_points[filtered_idx[j]];
            if (distance_2d(pi->x, pi->y, pj->x, pj->y) <= s_config.eps) {
                neighbors[neighbor_count++] = j;
            }
        }

        if (neighbor_count < s_config.min_samples) {
            continue;
        }

        int cid = next_cluster++;
        for (int n = 0; n < neighbor_count; n++) {
            cluster_id[neighbors[n]] = cid;
        }

        for (int iter = 0; iter < 3; iter++) {
            for (int k = 0; k < filtered_count; k++) {
                if (cluster_id[k] != cid) {
                    continue;
                }
                radar_point_t *pk = &s_frame_points[filtered_idx[k]];
                for (int m = 0; m < filtered_count; m++) {
                    if (cluster_id[m] != -1) {
                        continue;
                    }
                    radar_point_t *pm = &s_frame_points[filtered_idx[m]];
                    if (distance_2d(pk->x, pk->y, pm->x, pm->y) <= s_config.eps) {
                        cluster_id[m] = cid;
                    }
                }
            }
        }
    }

    human_target_t new_targets[RADAR_MAX_TARGETS];
    int new_target_count = 0;

    for (int c = 0; c < next_cluster && new_target_count < RADAR_MAX_TARGETS; c++) {
        float sum_x = 0, sum_y = 0, sum_z = 0, sum_v = 0, sum_conf = 0;
        int cnt = 0;

        for (int i = 0; i < filtered_count; i++) {
            if (cluster_id[i] != c) {
                continue;
            }
            radar_point_t *p = &s_frame_points[filtered_idx[i]];
            sum_x += p->x;
            sum_y += p->y;
            sum_z += p->z;
            sum_v += p->velocity;
            sum_conf += point_confidence(p->snr, p->abs_val, p->dpk);
            cnt++;
        }

        if (cnt == 0) {
            continue;
        }

        float avg_x = sum_x / cnt;
        float avg_y = sum_y / cnt;
        float avg_z = sum_z / cnt;
        float avg_v = sum_v / cnt;
        float mean_conf = sum_conf / cnt;

        float cluster_conf = 0.6f * mean_conf + 0.4f * fminf((float)cnt / 15.0f, 1.0f);

        if (cluster_conf < s_config.human_conf_threshold) {
            continue;
        }
        // ADVANCED FALL DETECTION
        // Track Z-height changes over time to detect rapid drops
        int64_t now = esp_timer_get_time();
        
        human_posture_t posture;
        
        if (s_fall_tracker.tracking) {
            float z_drop = s_fall_tracker.prev_z - avg_z;  // Positive = dropping, Negative = rising
            int64_t time_diff_ms = (now - s_fall_tracker.last_update_time) / 1000;
            
            // Calculate drop rate (meters per second)
            // Positive = falling down, Negative = moving up
            float drop_rate = (time_diff_ms > 0) ? (z_drop / (time_diff_ms / 1000.0f)) : 0.0f;
            
            ESP_LOGD(TAG, "Fall tracking: prev_z=%.2f, curr_z=%.2f, z_drop=%.2f, drop_rate=%.2f m/s",
                     s_fall_tracker.prev_z, avg_z, z_drop, drop_rate);
            // Additional check: velocity should be getting more negative (accelerating downward)
            float v_change = avg_v - s_fall_tracker.prev_v;
            bool accelerating_down = (avg_v < -0.2f && v_change < 0);  // Getting faster downward
            
            // Fall criteria (ONLY detect DOWNWARD falls):
            // 1. Z is DROPPING (z_drop > 0, drop_rate > 0)
            // 2. Rapid drop rate (> 0.5 m/s)
            // 3. Significant absolute drop (> 0.3m)
            // 4. Downward velocity (v < 0)
            // 5. Final Z position is low (< 0.5m)
            // 6. Accelerating downward
            if (z_drop > 0.3f &&                          // MUST be dropping (positive value)
                drop_rate > FALL_DROP_RATE_THRESHOLD &&   // MUST be rapid downward
                avg_v < -0.2f &&                          // MUST have downward velocity
                avg_z < 0.5f &&                           // MUST end up low
                accelerating_down) {                      // MUST be accelerating downward
                
                s_fall_confirmation_frames++;
                if (s_fall_confirmation_frames >= FALL_CONFIRMATION_THRESHOLD) {
                    posture = POSTURE_FALL;
                    ESP_LOGW(TAG, "FALL DETECTED: drop=%.2fm in %lldms, rate=%.2fm/s, final_z=%.2fm, v=%.2f",
                                          z_drop, time_diff_ms, drop_rate, avg_z, avg_v);
                } else {
                    // Potential fall, but not confirmed yet
                    posture = (avg_z >= s_config.standing_z) ? POSTURE_STANDING :
                              (avg_z >= s_config.sitting_z) ? POSTURE_SITTING : POSTURE_LYING;
                    ESP_LOGD(TAG, "Potential fall: frames=%d/%d", 
                                          s_fall_confirmation_frames, FALL_CONFIRMATION_THRESHOLD);
                }
            } else {
                // No fall detected, use normal posture classification
                
                // IMPORTANT: Reset fall counter if person is rising (standing up)
                if (z_drop < -0.2f) {  // Rising significantly (standing up)
                    if (s_fall_confirmation_frames > 0) {
                        ESP_LOGD(TAG, "Rising detected (z_drop=%.2f), resetting fall counter", z_drop);
                    }
                    s_fall_confirmation_frames = 0;
                } else if (z_drop <= 0) {  // Any upward or stable movement
                    s_fall_confirmation_frames = 0;
                }
                
                if (avg_z >= s_config.standing_z) {
                    posture = POSTURE_STANDING;
                } else if (avg_z >= s_config.sitting_z) {
                    posture = POSTURE_SITTING;
                } else if (avg_z >= s_config.lying_z) {
                    posture = POSTURE_LYING;
                } else {
                    posture = POSTURE_LYING;  // Very low but no rapid drop = lying
                }
                
                // Refine LYING vs SLEEPING
                if (posture == POSTURE_LYING && fabsf(avg_v) < s_config.v_move_threshold) {
                    posture = POSTURE_SLEEPING;
                }
            }
        } else {
            // First frame, no tracking history yet
            if (avg_z >= s_config.standing_z) {
                posture = POSTURE_STANDING;
            } else if (avg_z >= s_config.sitting_z) {
                posture = POSTURE_SITTING;
            } else if (avg_z >= s_config.lying_z) {
                posture = POSTURE_LYING;
            } else {
                posture = POSTURE_LYING;
            }
            
            if (posture == POSTURE_LYING && fabsf(avg_v) < s_config.v_move_threshold) {
                posture = POSTURE_SLEEPING;
            }
        }
        
        // Update tracker
        s_fall_tracker.prev_z = avg_z;
        s_fall_tracker.last_update_time = now;
        s_fall_tracker.tracking = true;

        human_target_t *t = &new_targets[new_target_count++];
        t->present = true;
        t->posture = posture;
        t->center_x = avg_x;
        t->center_y = avg_y;
        t->center_z = avg_z;
        t->avg_velocity = avg_v;
        t->confidence = cluster_conf;
        t->point_count = cnt;
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

    point->x = (float)atof(px + 2);
    point->y = (float)atof(py + 2);
    point->z = (float)atof(pz + 2);
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

