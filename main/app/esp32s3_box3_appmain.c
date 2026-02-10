/*
 * ESP32s3 Box3 App Main - Radar Human presense and fall detection for home safeety with Privecy protection.
 * This is the main application file for the ESP32s3 Box3, which initializes the system, mounts the SPIFFS file system, sets up the display and LVGL, and starts the main application loop. The application includes features such as radar-based human presence detection and fall detection for home safety, with a focus on privacy protection. The code is structured to ensure efficient use of resources and responsiveness of the user interface.
*/

#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "app_play_wav.h"
#include "radar_sensor.h"
#include "wifi_stream.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "gui/ui.h"
#include "gui/ui_events.h"


static const char *TAG = "main";

/*******************************************************************************
* Private functions
*******************************************************************************/

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 Box3 Fall Detection and Presence Detection Application Starting...");
    ESP_LOGI(TAG, "Compile time: %s %s", __DATE__, __TIME__);

    /* Initialize and mount SPIFFS */
    ESP_LOGI(TAG, "Mounting SPIFFS...");
    bsp_spiffs_mount();

    /* Initialize I2C (for touch and audio) */
    ESP_LOGI(TAG, "Initializing I2C...");
    bsp_i2c_init();

    /* Initialize display and LVGL */
    ESP_LOGI(TAG, "Initializing display and LVGL...");
    bsp_display_start();

    /* Set default display brightness */
    bsp_display_brightness_set(APP_DISP_DEFAULT_BRIGHTNESS);

    /* Initialize audio */
    ESP_LOGI(TAG, "Initializing audio...");
    app_audio_init();

    /* Lock LVGL mutex */
    lvgl_port_lock(0);

    /* Initialize UI and navigation */
    ESP_LOGI(TAG, "Initializing UI...");
    ui_init();
    ui_navigation_init();

    /* Unlock LVGL mutex after UI initialization */
    lvgl_port_unlock();

    /* Initialize SPI flash file system and show list of files on display */
    //app_disp_fs_init();

    /* Play boot animation and audio (non-blobking)*/
    ESP_LOGI(TAG, "Playing boot animation and audio...");
    load_progressbar_and_playaudio();

    /* Initialize WiFi streaming server */
    ESP_LOGI(TAG, "Initializing WiFi stream...");
    wifi_stream_config_t wifi_cfg = WIFI_STREAM_CONFIG_DEFAULT();
    wifi_stream_init(&wifi_cfg);

    /* Initializing radar sensor (UART + detection) */
    ESP_LOGI(TAG, "Initializing radar sensor...");
    radar_config_t radar_cfg = RADAR_CONFIG_DEFAULT();
    radar_sensor_init(&radar_cfg);

    /* Start Screen3 detection + posture monitor */
    lvgl_port_lock(0);
    ui_screen3_start_monitor();
    lvgl_port_unlock();

    ESP_LOGI(TAG, "App initialization done.");

}
