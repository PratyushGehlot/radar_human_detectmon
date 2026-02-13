/**
 * @file app_play_wav.h
 * @brief Radar Human Detection & Fall Monitor
 * @details ESP32-S3-BOX-3 based real-time human presence detection and fall
 *          monitoring system using LD6001 mmWave radar sensor.
 * @author PratyushGehlot
 * @see https://github.com/PratyushGehlot/radar_human_detectmon
 */

/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once


/* Default screen brightness */
#define APP_DISP_DEFAULT_BRIGHTNESS  (50)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Add and show LVGL objects on display
 */
void app_disp_lvgl_show(void);

/**
 * @brief Initialize SPI Flash File System and show list of files on display
 */
void app_disp_fs_init(void);

/**
 * @brief Initialize audio
 */
void app_audio_init(void);


/* Play recorded audio file */
void play_file(void *arg);

/* Request current playback to stop */
void play_file_request_stop(void);

/* Set speaker output volume (0-100) */
void set_speaker_volume(int volume);

#ifdef __cplusplus
}
#endif
