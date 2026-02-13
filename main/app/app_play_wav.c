/**
 * @file app_play_wav.c
 * @brief Radar Human Detection & Fall Monitor
 * @details ESP32-S3-BOX-3 based real-time human presence detection and fall
 *          monitoring system using LD6001 mmWave radar sensor.
 * @author PratyushGehlot
 * @see https://github.com/PratyushGehlot/radar_human_detectmon
 */

/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "app_play_wav.h"
#include "jpeg_decoder.h"



/* SPIFFS mount root */
#define FS_MNT_PATH  BSP_SPIFFS_MOUNT_POINT

/* Buffer for reading/writing to I2S driver. Same length as SPIFFS buffer and I2S buffer, for optimal read/write performance.
   Recording audio data path:
   I2S peripheral -> I2S buffer (DMA) -> App buffer (RAM) -> SPIFFS buffer -> External SPI Flash.
   Vice versa for playback. */
#define BUFFER_SIZE     (1024)
#define SAMPLE_RATE     (22050)
#define DEFAULT_VOLUME  (100)
/* The recording will be RECORDING_LENGTH * BUFFER_SIZE long (in bytes)
   With sampling frequency 22050 Hz and 16bit mono resolution it equals to ~3.715 seconds */
#define RECORDING_LENGTH (160)

#define REC_FILENAME    FS_MNT_PATH"/recording.wav"

static const char *TAG = "DISP";

static esp_codec_dev_handle_t spk_codec_dev = NULL;
static esp_codec_dev_handle_t mic_codec_dev = NULL;

/*******************************************************************************
* Types definitions
*******************************************************************************/
typedef enum {
    APP_FILE_TYPE_UNKNOWN,
    APP_FILE_TYPE_TXT,
    APP_FILE_TYPE_IMG,
    APP_FILE_TYPE_WAV,
} app_file_type_t;

typedef struct __attribute__((packed))
{
    uint16_t num_channels;
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint32_t data_size;
    long data_offset;
} wav_info_t;

typedef struct __attribute__((packed))
{
    char riff_tag[4];
    uint32_t file_size;
    char wave_tag[4];
    char fmt_tag[4];
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char data_tag[4];
    uint32_t data_size;
} wav_header_write_t;

static esp_err_t wav_parse(FILE *f, wav_info_t *info)
{
    uint8_t hdr[12];
    if (fread(hdr, 1, 12, f) != 12) return ESP_FAIL;
    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) return ESP_FAIL;

    while (1) {
        uint8_t chunk_hdr[8];
        if (fread(chunk_hdr, 1, 8, f) != 8) return ESP_FAIL;
        uint32_t chunk_size = chunk_hdr[4] | (chunk_hdr[5] << 8) | (chunk_hdr[6] << 16) | (chunk_hdr[7] << 24);

        if (memcmp(chunk_hdr, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            if (chunk_size < 16 || fread(fmt, 1, 16, f) != 16) return ESP_FAIL;
            info->num_channels   = fmt[2] | (fmt[3] << 8);
            info->sample_rate    = fmt[4] | (fmt[5] << 8) | (fmt[6] << 16) | (fmt[7] << 24);
            info->bits_per_sample = fmt[14] | (fmt[15] << 8);
            if (chunk_size > 16) fseek(f, chunk_size - 16, SEEK_CUR);
        } else if (memcmp(chunk_hdr, "data", 4) == 0) {
            info->data_size = chunk_size;
            info->data_offset = ftell(f);
            return ESP_OK;
        } else {
            fseek(f, chunk_size, SEEK_CUR);
        }
    }
    return ESP_FAIL;
}

/*******************************************************************************
* Function definitions
*******************************************************************************/
/*******************************************************************************
* Local variables
*******************************************************************************/

/* Audio */
static SemaphoreHandle_t audio_mux;
static bool play_file_repeat = false;
static bool play_file_stop = false;

/*******************************************************************************
* Public API functions
*******************************************************************************/

void app_audio_init(void)
{
    /*create audio mutex once at startup*/
    if (audio_mux == NULL) {
        audio_mux = xSemaphoreCreateMutex();
        assert(audio_mux);
        ESP_LOGI(TAG, "Audio mutex created.");
    }

    /* Initialize speaker */
    spk_codec_dev = bsp_audio_codec_speaker_init();
    assert(spk_codec_dev);
    /* Speaker output volume */
    esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);

    /* Initialize microphone */
    mic_codec_dev = bsp_audio_codec_microphone_init();
    assert(mic_codec_dev);
    /* Microphone input gain */
    esp_codec_dev_set_in_gain(mic_codec_dev, 50.0);
}


/*******************************************************************************
* Private API function
*******************************************************************************/

void play_file_request_stop(void)
{
    play_file_stop = true;
}

void play_file(void *arg)
{
    play_file_stop = false;
    char *path = arg;
    FILE *file = NULL;
    int16_t *wav_bytes = NULL;
    esp_err_t ret;

    /*Ensure mutex exisit*/
    if (audio_mux == NULL || spk_codec_dev == NULL) {
        ESP_LOGE(TAG,"Audio not initialized! call app_audio_init() before playing audio.");
        return;
    }

    wav_bytes = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DEFAULT);
    if (wav_bytes == NULL) {
        ESP_LOGE(TAG, "Not enough memory for playing!");
        //goto END;
        return;
    }

    /* Open file */
    file = fopen(path, "rb");
    if (file == NULL) {
        ESP_LOGE(TAG, "%s file does not exist!", path);
        //goto END;
        free(wav_bytes);
        return;
    }

    /* Parse WAV file chunks */
    wav_info_t wav_info;
    if (wav_parse(file, &wav_info) != ESP_OK) {
        ESP_LOGW(TAG, "Error parsing WAV file");
        free(wav_bytes);
        fclose(file);
        return;
    }

    ESP_LOGI(TAG, "WAV file info: Sample Rate: %" PRIu32 ", Channels: %" PRIu16 ", Bits per sample: %" PRIu16 ", Data size: %" PRIu32,
             wav_info.sample_rate, wav_info.num_channels, wav_info.bits_per_sample, wav_info.data_size);

    /* Configure codec device according to WAV header */
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = wav_info.sample_rate,
        .channel = wav_info.num_channels,
        .bits_per_sample = wav_info.bits_per_sample,
    };

    /* Take mutex before opening codec device */
    xSemaphoreTake(audio_mux, portMAX_DELAY);
    ret = esp_codec_dev_open(spk_codec_dev, &fs);
    xSemaphoreGive(audio_mux);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open codec device for playback %s", esp_err_to_name(ret));
        //goto END;
        free(wav_bytes);
        fclose(file);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // Short delay to ensure codec device is ready

    /* playback loop*/
    uint32_t bytes_send_to_i2s = 0;
    do {
        bytes_send_to_i2s = 0;
        fseek(file, wav_info.data_offset, SEEK_SET);
        while (bytes_send_to_i2s < wav_info.data_size) {
            if (play_file_stop) {
                //goto END;
                break;
            }
            
            /* Get data from SPIFFS */
            size_t bytes_read_from_spiffs = fread(wav_bytes, 1, BUFFER_SIZE, file);

            /* Send it to I2S */
            xSemaphoreTake(audio_mux, portMAX_DELAY);
            esp_codec_dev_write(spk_codec_dev, wav_bytes, bytes_read_from_spiffs);
            xSemaphoreGive(audio_mux);

            bytes_send_to_i2s += bytes_read_from_spiffs;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    } while (play_file_repeat && !play_file_stop);


    /* Clean up and free resources with mutex protection */
    xSemaphoreTake(audio_mux, portMAX_DELAY);
    esp_codec_dev_close(spk_codec_dev);
    xSemaphoreGive(audio_mux);

    if(file) {
        fclose(file);
    }

    if(wav_bytes) {
        free(wav_bytes);
    }

    ESP_LOGI(TAG, "Finished playing file: %s", path);
}

void set_speaker_volume(int volume)
{
    if (spk_codec_dev == NULL) return;
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    esp_codec_dev_set_out_vol(spk_codec_dev, volume);
}
