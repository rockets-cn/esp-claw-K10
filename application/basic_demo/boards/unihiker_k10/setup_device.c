/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_io_expander_tca95xx_16bit.h"
#include "esp_lcd_ili9341.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "K10_BOARD_SETUP"

#define K10_EXPANDER_ADDR ESP_IO_EXPANDER_I2C_TCA9555_ADDRESS_000
#define K10_LCD_BACKLIGHT IO_EXPANDER_PIN_NUM_0
#define K10_CAMERA_RESET  IO_EXPANDER_PIN_NUM_1
#define K10_AMP_GAIN      IO_EXPANDER_PIN_NUM_15

static esp_io_expander_handle_t s_io_expander;

static esp_err_t k10_io_expander_ensure_ready(void)
{
    if (s_io_expander != NULL) {
        return ESP_OK;
    }

    i2c_master_bus_handle_t i2c_bus = NULL;
    ESP_RETURN_ON_ERROR(esp_board_periph_get_handle("i2c_master", (void **)&i2c_bus),
                        TAG, "Failed to get board I2C bus");
    ESP_RETURN_ON_ERROR(esp_io_expander_new_i2c_tca95xx_16bit(i2c_bus, K10_EXPANDER_ADDR, &s_io_expander),
                        TAG, "Failed to create IO expander");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_dir(s_io_expander, K10_LCD_BACKLIGHT | K10_CAMERA_RESET | K10_AMP_GAIN,
                                                IO_EXPANDER_OUTPUT),
                        TAG, "Failed to set IO expander direction");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(s_io_expander, K10_AMP_GAIN, 0),
                        TAG, "Failed to set amplifier gain");
    return ESP_OK;
}

static esp_err_t k10_prepare_display_power(void)
{
    ESP_RETURN_ON_ERROR(k10_io_expander_ensure_ready(), TAG, "Failed to prepare IO expander");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(s_io_expander, K10_LCD_BACKLIGHT | K10_CAMERA_RESET, 0),
                        TAG, "Failed to pull display rails low");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(s_io_expander, K10_LCD_BACKLIGHT | K10_CAMERA_RESET, 1),
                        TAG, "Failed to enable display rails");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

esp_err_t lcd_panel_factory_entry_t(esp_lcd_panel_io_handle_t io,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    esp_lcd_panel_handle_t *ret_panel)
{
    esp_lcd_panel_dev_config_t panel_cfg = {0};
    memcpy(&panel_cfg, panel_dev_config, sizeof(panel_cfg));

    ESP_RETURN_ON_ERROR(k10_prepare_display_power(), TAG, "Failed to power display");
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9341(io, &panel_cfg, ret_panel),
                        TAG, "Failed to create ILI9341 panel");
    return ESP_OK;
}
