/**
 * @file wifi_stream.h
 * @brief Radar Human Detection & Fall Monitor
 * @details ESP32-S3-BOX-3 based real-time human presence detection and fall
 *          monitoring system using LD6001 mmWave radar sensor.
 * @author PratyushGehlot
 * @see https://github.com/PratyushGehlot/radar_human_detectmon
 */

#ifndef WIFI_STREAM_H
#define WIFI_STREAM_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

#define WIFI_STREAM_DEFAULT_PORT 3333
#define WIFI_STREAM_MAX_CLIENTS 3

typedef struct {
    const char *ssid;
    const char *password;
    uint16_t port;
    bool ap_mode;
} wifi_stream_config_t;

#define WIFI_STREAM_CONFIG_DEFAULT() { \
    .ssid = "ESP32S3BOX3_RadarSensor", \
    .password = "esp32radar1234", \
    .port = WIFI_STREAM_DEFAULT_PORT, \
    .ap_mode = true, \
}

esp_err_t wifi_stream_init(const wifi_stream_config_t *config);
esp_err_t wifi_stream_send(const char *data, size_t len);
void wifi_stream_deinit(void);

/* Get the IP address string of the first connected client, or NULL if none */
const char *wifi_stream_get_client_ip(void);

#endif
