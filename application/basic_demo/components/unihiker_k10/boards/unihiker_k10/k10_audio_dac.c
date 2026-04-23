/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "dev_audio_codec.h"
#include "driver/gpio.h"
#include "dummy_codec.h"
#include "esp_board_manager.h"
#include "esp_board_manager_includes.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "gen_board_device_custom.h"

#define TAG "k10_audio_dac"

#define K10_AUDIO_DAC_I2S_NAME "i2s_audio_out"

static const dev_audio_codec_config_t s_k10_audio_dac_cfg = {
    .name = "audio_dac",
    .chip = "dummy",
    .type = "audio_codec",
    .data_if_type = DEV_AUDIO_CODEC_DATA_IF_TYPE_I2S,
    .adc_enabled = false,
    .adc_max_channel = 0,
    .adc_channel_mask = 0,
    .adc_channel_labels = "",
    .adc_init_gain = 0,
    .dac_enabled = true,
    .dac_max_channel = 1,
    .dac_channel_mask = 0x1,
    .dac_init_gain = 0,
    .pa_cfg = {
        .name = NULL,
        .port = -1,
        .active_level = 0,
        .gain = 0.0f,
    },
    .i2c_cfg = {0},
    .i2s_cfg = {
        .name = K10_AUDIO_DAC_I2S_NAME,
        .port = 0,
        .clk_src = 0,
        .tx_aux_out_io = -1,
        .tx_aux_out_line = 0,
        .tx_aux_out_invert = false,
    },
    .adc_cfg = {
        .periph_name = NULL,
        .sample_rate_hz = 0,
        .max_store_buf_size = 0,
        .conv_frame_size = 0,
        .conv_mode = 0,
        .format = 0,
        .pattern_num = 0,
        .cfg_mode = 0,
        .cfg = {
            .single_unit = {
                .unit_id = 0,
                .atten = 0,
                .bit_width = 0,
                .channel_id = {},
            },
        },
    },
    .metadata = NULL,
    .metadata_size = 0,
    .mclk_enabled = true,
    .aec_enabled = false,
    .eq_enabled = false,
    .alc_enabled = false,
};

static int audio_dac_init(void *config, int cfg_size, void **device_handle)
{
    (void)cfg_size;
    ESP_RETURN_ON_FALSE(config != NULL && device_handle != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid K10 audio_dac config");

    const dev_custom_audio_dac_config_t *cfg = (const dev_custom_audio_dac_config_t *)config;
    esp_err_t ret = ESP_OK;
    dev_audio_codec_handles_t *codec_handles = calloc(1, sizeof(*codec_handles));
    audio_codec_i2s_cfg_t i2s_cfg = {0};
    dummy_codec_cfg_t dummy_cfg = {0};
    esp_codec_dev_cfg_t dev_cfg = {0};
    void *tx_handle = NULL;

    ESP_RETURN_ON_FALSE(codec_handles != NULL, ESP_ERR_NO_MEM, TAG, "no memory for audio_dac handle");
    ESP_GOTO_ON_ERROR(esp_board_periph_ref_handle(K10_AUDIO_DAC_I2S_NAME, &tx_handle), err, TAG,
                      "failed to ref I2S TX handle");

    i2s_cfg.port = I2S_NUM_0;
    i2s_cfg.tx_handle = tx_handle;
    i2s_cfg.rx_handle = NULL;
    codec_handles->data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_GOTO_ON_FALSE(codec_handles->data_if != NULL, ESP_ERR_NO_MEM, err, TAG,
                      "failed to create I2S data interface");

    codec_handles->gpio_if = audio_codec_new_gpio();
    ESP_GOTO_ON_FALSE(codec_handles->gpio_if != NULL, ESP_ERR_NO_MEM, err, TAG,
                      "failed to create GPIO codec interface");

    dummy_cfg.gpio_if = codec_handles->gpio_if;
    dummy_cfg.pa_pin = GPIO_NUM_NC;
    dummy_cfg.pa_reverted = false;
    codec_handles->codec_if = dummy_codec_new(&dummy_cfg);
    ESP_GOTO_ON_FALSE(codec_handles->codec_if != NULL, ESP_ERR_NO_MEM, err, TAG,
                      "failed to create dummy codec");

    dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_OUT;
    dev_cfg.codec_if = codec_handles->codec_if;
    dev_cfg.data_if = codec_handles->data_if;
    codec_handles->codec_dev = esp_codec_dev_new(&dev_cfg);
    ESP_GOTO_ON_FALSE(codec_handles->codec_dev != NULL, ESP_ERR_NO_MEM, err, TAG,
                      "failed to create output codec device");

    esp_codec_set_disable_when_closed(codec_handles->codec_dev, false);

    *device_handle = codec_handles;
    esp_board_device_update_config("audio_dac", (void *)&s_k10_audio_dac_cfg);
    ESP_LOGI(TAG, "K10 I2S audio_dac initialized: codec_dev=%p sample_rate=%" PRIu32,
             codec_handles->codec_dev, cfg->sample_rate_hz);
    return ESP_OK;

err:
    if (codec_handles != NULL) {
        if (codec_handles->codec_dev != NULL) {
            esp_codec_dev_delete(codec_handles->codec_dev);
        }
        if (codec_handles->codec_if != NULL) {
            audio_codec_delete_codec_if(codec_handles->codec_if);
        }
        if (codec_handles->gpio_if != NULL) {
            audio_codec_delete_gpio_if(codec_handles->gpio_if);
        }
        if (codec_handles->data_if != NULL) {
            audio_codec_delete_data_if(codec_handles->data_if);
        }
        free(codec_handles);
    }
    if (tx_handle != NULL) {
        esp_board_periph_unref_handle(K10_AUDIO_DAC_I2S_NAME);
    }
    return ret;
}

static int audio_dac_deinit(void *device_handle)
{
    dev_audio_codec_handles_t *codec_handles = (dev_audio_codec_handles_t *)device_handle;

    if (codec_handles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (codec_handles->codec_dev != NULL) {
        esp_codec_dev_close(codec_handles->codec_dev);
        esp_codec_dev_delete(codec_handles->codec_dev);
    }
    if (codec_handles->codec_if != NULL) {
        audio_codec_delete_codec_if(codec_handles->codec_if);
    }
    if (codec_handles->gpio_if != NULL) {
        audio_codec_delete_gpio_if(codec_handles->gpio_if);
    }
    if (codec_handles->data_if != NULL) {
        audio_codec_delete_data_if(codec_handles->data_if);
    }
    esp_board_periph_unref_handle(K10_AUDIO_DAC_I2S_NAME);
    free(codec_handles);
    return ESP_OK;
}

CUSTOM_DEVICE_IMPLEMENT(audio_dac, audio_dac_init, audio_dac_deinit);
