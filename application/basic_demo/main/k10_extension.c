/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "k10_extension.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "dev_camera.h"
#include "esp_board_manager.h"
#include "esp_board_manager_includes.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_io_expander_tca95xx_16bit.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "led_strip_types.h"

#define TAG "k10_ext"

#define K10_I2C_NAME "i2c_master"
#define K10_CAMERA_NAME "camera"
#define K10_AUDIO_ADC_NAME "audio_adc"
#define K10_AUDIO_DAC_NAME "audio_dac"

#define K10_EXPANDER_ADDR ESP_IO_EXPANDER_I2C_TCA9555_ADDRESS_000
#define K10_LCD_BACKLIGHT IO_EXPANDER_PIN_NUM_0
#define K10_CAMERA_RESET  IO_EXPANDER_PIN_NUM_1
#define K10_AMP_GAIN      IO_EXPANDER_PIN_NUM_15

#define K10_RGB_GPIO 46
#define K10_RGB_COUNT 3

#define AHT20_ADDR 0x38
#define LTR303_ADDR 0x29
#define SC7A20H_ADDR 0x19

#define LTR303_REG_ALS_CONTR 0x80
#define LTR303_REG_ALS_MEAS_RATE 0x85
#define LTR303_REG_ALS_STATUS 0x8C
#define LTR303_REG_ALS_DATA_CH1_0 0x88

#define SC7A20H_REG_WHO_AM_I 0x0F
#define SC7A20H_REG_CTRL1 0x20
#define SC7A20H_REG_CTRL4 0x23
#define SC7A20H_REG_OUT_X_L 0x28

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    esp_io_expander_handle_t io_expander;
    i2c_master_dev_handle_t aht20;
    i2c_master_dev_handle_t ltr303;
    i2c_master_dev_handle_t sc7a20h;
    led_strip_handle_t rgb;
    bool i2c_ready;
} k10_extension_state_t;

static k10_extension_state_t s_state;

static esp_err_t k10_i2c_bus_ensure(void)
{
    if (s_state.i2c_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(esp_board_manager_get_periph_handle(K10_I2C_NAME, (void **)&s_state.i2c_bus),
                        TAG, "Failed to get board I2C bus");
    s_state.i2c_ready = true;
    return ESP_OK;
}

static esp_err_t k10_i2c_add_device(i2c_master_dev_handle_t *out_dev, uint16_t addr, uint32_t speed_hz)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = speed_hz,
    };

    ESP_RETURN_ON_ERROR(k10_i2c_bus_ensure(), TAG, "I2C bus unavailable");
    return i2c_master_bus_add_device(s_state.i2c_bus, &dev_cfg, out_dev);
}

static esp_err_t k10_io_expander_ensure(void)
{
    if (s_state.io_expander != NULL) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(k10_i2c_bus_ensure(), TAG, "I2C bus unavailable");
    ESP_RETURN_ON_ERROR(esp_io_expander_new_i2c_tca95xx_16bit(s_state.i2c_bus, K10_EXPANDER_ADDR,
                                                              &s_state.io_expander),
                        TAG, "Failed to create IO expander");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_dir(s_state.io_expander, K10_LCD_BACKLIGHT | K10_CAMERA_RESET | K10_AMP_GAIN,
                                                IO_EXPANDER_OUTPUT),
                        TAG, "Failed to set IO expander direction");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(s_state.io_expander, K10_AMP_GAIN, 0),
                        TAG, "Failed to set amplifier gain");
    return ESP_OK;
}

static esp_err_t k10_camera_reset_pulse(void)
{
    ESP_RETURN_ON_ERROR(k10_io_expander_ensure(), TAG, "IO expander unavailable");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(s_state.io_expander, K10_CAMERA_RESET, 0),
                        TAG, "Failed to pull camera reset low");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(s_state.io_expander, K10_CAMERA_RESET, 1),
                        TAG, "Failed to release camera reset");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t k10_rgb_init(void)
{
    if (s_state.rgb != NULL) {
        return ESP_OK;
    }

    led_strip_config_t strip_cfg = {
        .strip_gpio_num = K10_RGB_GPIO,
        .max_leds = K10_RGB_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = true,
        },
    };

    ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_state.rgb),
                        TAG, "Failed to create RGB strip");
    ESP_RETURN_ON_ERROR(led_strip_clear(s_state.rgb), TAG, "Failed to clear RGB strip");
    ESP_RETURN_ON_ERROR(led_strip_set_pixel(s_state.rgb, 0, 0, 0, 24), TAG, "Failed to set RGB self-test");
    ESP_RETURN_ON_ERROR(led_strip_set_pixel(s_state.rgb, 1, 0, 24, 0), TAG, "Failed to set RGB self-test");
    ESP_RETURN_ON_ERROR(led_strip_set_pixel(s_state.rgb, 2, 24, 0, 0), TAG, "Failed to set RGB self-test");
    ESP_RETURN_ON_ERROR(led_strip_refresh(s_state.rgb), TAG, "Failed to refresh RGB strip");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(led_strip_clear(s_state.rgb), TAG, "Failed to clear RGB strip");
    ESP_LOGI(TAG, "WS2812 strip ready on GPIO%d (%d LEDs)", K10_RGB_GPIO, K10_RGB_COUNT);
    return ESP_OK;
}

static void k10_release_probe_resources(void)
{
    if (s_state.rgb != NULL) {
        led_strip_del(s_state.rgb);
        s_state.rgb = NULL;
    }
    if (s_state.aht20 != NULL) {
        i2c_master_bus_rm_device(s_state.aht20);
        s_state.aht20 = NULL;
    }
    if (s_state.ltr303 != NULL) {
        i2c_master_bus_rm_device(s_state.ltr303);
        s_state.ltr303 = NULL;
    }
    if (s_state.sc7a20h != NULL) {
        i2c_master_bus_rm_device(s_state.sc7a20h);
        s_state.sc7a20h = NULL;
    }
    if (s_state.io_expander != NULL) {
        esp_io_expander_del(s_state.io_expander);
        s_state.io_expander = NULL;
    }
}

static esp_err_t k10_aht20_sample(float *humidity, float *temperature_c)
{
    uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};
    uint8_t trigger_cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t rx[6] = {0};
    uint32_t raw_humidity = 0;
    uint32_t raw_temperature = 0;

    if (s_state.aht20 == NULL) {
        ESP_RETURN_ON_ERROR(k10_i2c_add_device(&s_state.aht20, AHT20_ADDR, 400000),
                            TAG, "Failed to add AHT20");
    }

    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_state.aht20, init_cmd, sizeof(init_cmd), 100),
                        TAG, "Failed to init AHT20");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_state.aht20, trigger_cmd, sizeof(trigger_cmd), 100),
                        TAG, "Failed to trigger AHT20 measurement");
    vTaskDelay(pdMS_TO_TICKS(90));
    ESP_RETURN_ON_ERROR(i2c_master_receive(s_state.aht20, rx, sizeof(rx), 100),
                        TAG, "Failed to read AHT20 sample");

    raw_humidity = ((uint32_t)rx[1] << 12) | ((uint32_t)rx[2] << 4) | (rx[3] >> 4);
    raw_temperature = ((uint32_t)(rx[3] & 0x0F) << 16) | ((uint32_t)rx[4] << 8) | rx[5];

    if (humidity != NULL) {
        *humidity = (raw_humidity * 100.0f) / 1048576.0f;
    }
    if (temperature_c != NULL) {
        *temperature_c = ((raw_temperature * 200.0f) / 1048576.0f) - 50.0f;
    }
    return ESP_OK;
}

static esp_err_t k10_ltr303_sample(uint16_t *ch0, uint16_t *ch1)
{
    uint8_t config[][2] = {
        {LTR303_REG_ALS_CONTR, 0x01},
        {LTR303_REG_ALS_MEAS_RATE, 0x03},
    };
    uint8_t raw[4] = {0};
    uint8_t status = 0;
    uint8_t reg = LTR303_REG_ALS_DATA_CH1_0;

    if (s_state.ltr303 == NULL) {
        ESP_RETURN_ON_ERROR(k10_i2c_add_device(&s_state.ltr303, LTR303_ADDR, 400000),
                            TAG, "Failed to add LTR303");
    }

    for (size_t i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
        ESP_RETURN_ON_ERROR(i2c_master_transmit(s_state.ltr303, config[i], sizeof(config[i]), 100),
                            TAG, "Failed to configure LTR303");
    }
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_state.ltr303, &reg, 1, raw, sizeof(raw), 100),
                        TAG, "Failed to read LTR303 sample");
    reg = LTR303_REG_ALS_STATUS;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_state.ltr303, &reg, 1, &status, 1, 100),
                        TAG, "Failed to read LTR303 status");

    if (ch1 != NULL) {
        *ch1 = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    }
    if (ch0 != NULL) {
        *ch0 = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
    }
    ESP_LOGI(TAG, "LTR303 status=0x%02x", status);
    return ESP_OK;
}

static esp_err_t k10_sc7a20h_sample(uint8_t *who_am_i, int16_t xyz[3])
{
    uint8_t reg = SC7A20H_REG_WHO_AM_I;
    uint8_t id = 0;
    uint8_t init[][2] = {
        {SC7A20H_REG_CTRL1, 0x57},
        {SC7A20H_REG_CTRL4, 0x88},
    };
    uint8_t raw[6] = {0};

    if (s_state.sc7a20h == NULL) {
        ESP_RETURN_ON_ERROR(k10_i2c_add_device(&s_state.sc7a20h, SC7A20H_ADDR, 400000),
                            TAG, "Failed to add SC7A20H");
    }

    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_state.sc7a20h, &reg, 1, &id, 1, 100),
                        TAG, "Failed to read SC7A20H id");
    for (size_t i = 0; i < sizeof(init) / sizeof(init[0]); ++i) {
        ESP_RETURN_ON_ERROR(i2c_master_transmit(s_state.sc7a20h, init[i], sizeof(init[i]), 100),
                            TAG, "Failed to configure SC7A20H");
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    reg = SC7A20H_REG_OUT_X_L | 0x80;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_state.sc7a20h, &reg, 1, raw, sizeof(raw), 100),
                        TAG, "Failed to read SC7A20H axes");

    if (who_am_i != NULL) {
        *who_am_i = id;
    }
    if (xyz != NULL) {
        xyz[0] = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
        xyz[1] = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
        xyz[2] = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
    }
    return ESP_OK;
}

static void k10_log_device_handle(const char *name)
{
    void *handle = NULL;
    esp_err_t err = esp_board_manager_get_device_handle(name, &handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s handle=%p", name, handle);
    } else {
        ESP_LOGW(TAG, "%s unavailable: %s", name, esp_err_to_name(err));
    }
}

static void k10_probe_camera(void)
{
    dev_camera_handle_t *camera = NULL;
    esp_err_t err = esp_board_manager_init_device_by_name(K10_CAMERA_NAME);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "camera init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_board_manager_get_device_handle(K10_CAMERA_NAME, (void **)&camera);
    if (err != ESP_OK || camera == NULL) {
        ESP_LOGW(TAG, "camera handle unavailable: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "camera ready: dev_path=%s meta_path=%s",
             camera->dev_path ? camera->dev_path : "(none)",
             camera->meta_path ? camera->meta_path : "(none)");
}

esp_err_t k10_extension_init(void)
{
    float humidity = 0.0f;
    float temperature_c = 0.0f;
    uint16_t light_ch0 = 0;
    uint16_t light_ch1 = 0;
    uint8_t accel_id = 0;
    int16_t accel_xyz[3] = {0};

    ESP_RETURN_ON_ERROR(k10_rgb_init(), TAG, "Failed to init RGB strip");
    ESP_RETURN_ON_ERROR(k10_camera_reset_pulse(), TAG, "Failed to reset camera");
    k10_probe_camera();

    if (k10_aht20_sample(&humidity, &temperature_c) == ESP_OK) {
        ESP_LOGI(TAG, "AHT20 temperature=%.2fC humidity=%.2f%%RH", temperature_c, humidity);
    }
    if (k10_ltr303_sample(&light_ch0, &light_ch1) == ESP_OK) {
        ESP_LOGI(TAG, "LTR303 ch0=%" PRIu16 " ch1=%" PRIu16, light_ch0, light_ch1);
    }
    if (k10_sc7a20h_sample(&accel_id, accel_xyz) == ESP_OK) {
        ESP_LOGI(TAG, "SC7A20H who_am_i=0x%02x accel_raw=[%d,%d,%d]",
                 accel_id, accel_xyz[0], accel_xyz[1], accel_xyz[2]);
    }

    if (esp_board_manager_init_device_by_name(K10_AUDIO_DAC_NAME) != ESP_OK) {
        ESP_LOGW(TAG, "audio_dac init failed; continuing without audio output");
    }
    k10_log_device_handle(K10_AUDIO_ADC_NAME);
    k10_log_device_handle(K10_AUDIO_DAC_NAME);
    k10_release_probe_resources();
    return ESP_OK;
}
