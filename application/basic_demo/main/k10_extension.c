/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "k10_extension.h"

#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>

#include "dev_audio_codec.h"
#include "driver/i2c_master.h"
#include "dev_camera.h"
#include "esp_board_manager.h"
#include "esp_board_manager_includes.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_err.h"
#include "esp_io_expander_tca95xx_16bit.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "led_strip_types.h"

#define TAG "k10_ext"

#define K10_I2C_NAME "i2c_master"
#define K10_CAMERA_NAME "camera"
#define K10_AUDIO_ADC_NAME "audio_adc"
#define K10_AUDIO_DAC_NAME "audio_dac"
#define K10_DVP_VIDEO_OBJECT_NAME "DVP"

#define K10_CAMERA_CAPTURE_TIMEOUT_MS 3000
#define K10_CAMERA_BUFFER_COUNT 3
#define K10_AUDIO_CHUNK_BYTES 512
#define K10_AUDIO_TONE_VOLUME 100

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
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    char pixel_format_str[5];
} k10_camera_stream_info_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    char pixel_format_str[5];
    size_t frame_bytes;
    int64_t timestamp_us;
} k10_camera_frame_info_t;

typedef TAILQ_ENTRY(k10_esp_video_buffer_element) k10_esp_video_buffer_node_t;

struct esp_video;
struct k10_esp_video_buffer;

typedef struct k10_esp_video_buffer_element {
    bool free;
    k10_esp_video_buffer_node_t node;
    struct k10_esp_video_buffer *video_buffer;
    uint32_t index;
    uint8_t *buffer;
    uint32_t valid_size;
    void *priv_data;
} k10_esp_video_buffer_element_t;

esp_err_t camera_open(const char *dev_path);
esp_err_t camera_get_stream_info(k10_camera_stream_info_t *out_info);
esp_err_t camera_capture_jpeg(int timeout_ms, uint8_t **jpeg_data, size_t *jpeg_bytes,
                              k10_camera_frame_info_t *out_info);
esp_err_t camera_close(void);
void camera_free_buffer(void *buffer);
esp_err_t esp_video_open(const char *name, struct esp_video **video_ret);
esp_err_t esp_video_close(struct esp_video *video);
esp_err_t esp_video_get_format(struct esp_video *video, struct v4l2_format *format);
esp_err_t esp_video_setup_buffer(struct esp_video *video, uint32_t type, uint32_t memory_type, uint32_t count);
esp_err_t esp_video_start_capture(struct esp_video *video, uint32_t type);
esp_err_t esp_video_stop_capture(struct esp_video *video, uint32_t type);
esp_err_t esp_video_queue_element_index(struct esp_video *video, uint32_t type, int index);
k10_esp_video_buffer_element_t *esp_video_recv_element(struct esp_video *video, uint32_t type, uint32_t ticks);
esp_err_t esp_video_done_element(struct esp_video *video, uint32_t type, k10_esp_video_buffer_element_t *element);

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

static void k10_verify_camera_capture(void)
{
    dev_camera_handle_t *camera = NULL;
    k10_camera_stream_info_t stream_info = {0};
    k10_camera_frame_info_t frame_info = {0};
    uint8_t *jpeg_data = NULL;
    size_t jpeg_bytes = 0;
    esp_err_t err = esp_board_manager_get_device_handle(K10_CAMERA_NAME, (void **)&camera);

    if (err != ESP_OK || camera == NULL || camera->dev_path == NULL) {
        ESP_LOGW(TAG, "camera capture skipped: handle unavailable: %s", esp_err_to_name(err));
        return;
    }

    err = camera_open(camera->dev_path);
    if (err == ESP_OK) {
        err = camera_get_stream_info(&stream_info);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "camera stream info failed: %s", esp_err_to_name(err));
            goto cleanup_vfs;
        }

        ESP_LOGI(TAG, "camera stream verified: %ux%u format=%s",
                 stream_info.width, stream_info.height, stream_info.pixel_format_str);

        err = camera_capture_jpeg(K10_CAMERA_CAPTURE_TIMEOUT_MS, &jpeg_data, &jpeg_bytes, &frame_info);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "camera frame capture failed: %s", esp_err_to_name(err));
            goto cleanup_vfs;
        }

        ESP_LOGI(TAG,
                 "camera frame captured via VFS: jpeg_bytes=%u frame=%ux%u format=%s raw_bytes=%u timestamp_us=%" PRId64,
                 (unsigned int)jpeg_bytes,
                 frame_info.width,
                 frame_info.height,
                 frame_info.pixel_format_str,
                 (unsigned int)frame_info.frame_bytes,
                 frame_info.timestamp_us);
        goto cleanup_vfs;
    }

    ESP_LOGW(TAG, "camera VFS capture unavailable (%s), trying direct esp_video path",
             esp_err_to_name(err));

    {
        struct esp_video *video = NULL;
        k10_esp_video_buffer_element_t *element = NULL;
        struct v4l2_format format = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        };
        bool started = false;

        err = esp_video_open(K10_DVP_VIDEO_OBJECT_NAME, &video);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "camera direct open failed: %s", esp_err_to_name(err));
            return;
        }

        err = esp_video_get_format(video, &format);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "camera direct get_format failed: %s", esp_err_to_name(err));
            goto cleanup_direct;
        }

        err = esp_video_setup_buffer(video, format.type, V4L2_MEMORY_MMAP, K10_CAMERA_BUFFER_COUNT);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "camera direct setup_buffer failed: %s", esp_err_to_name(err));
            goto cleanup_direct;
        }

        for (int i = 0; i < K10_CAMERA_BUFFER_COUNT; ++i) {
            err = esp_video_queue_element_index(video, format.type, i);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "camera direct queue buffer %d failed: %s", i, esp_err_to_name(err));
                goto cleanup_direct;
            }
        }

        err = esp_video_start_capture(video, format.type);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "camera direct start_capture failed: %s", esp_err_to_name(err));
            goto cleanup_direct;
        }
        started = true;

        element = esp_video_recv_element(video, format.type, pdMS_TO_TICKS(K10_CAMERA_CAPTURE_TIMEOUT_MS));
        if (element == NULL) {
            ESP_LOGW(TAG, "camera direct receive frame timed out");
            err = ESP_ERR_TIMEOUT;
            goto cleanup_direct;
        }

        ESP_LOGI(TAG,
                 "camera frame captured via esp_video: bytes=%u frame=%ux%u pixfmt=0x%08" PRIx32 " buffer_index=%u",
                 (unsigned int)element->valid_size,
                 format.fmt.pix.width,
                 format.fmt.pix.height,
                 format.fmt.pix.pixelformat,
                 element->index);

cleanup_direct:
        if (element != NULL) {
            esp_err_t done_err = esp_video_done_element(video, format.type, element);
            if (done_err != ESP_OK) {
                ESP_LOGW(TAG, "camera direct done_element failed: %s", esp_err_to_name(done_err));
            }
        }
        if (started) {
            esp_err_t stop_err = esp_video_stop_capture(video, format.type);
            if (stop_err != ESP_OK) {
                ESP_LOGW(TAG, "camera direct stop_capture failed: %s", esp_err_to_name(stop_err));
            }
        }
        esp_err_t close_err = esp_video_close(video);
        if (close_err != ESP_OK) {
            ESP_LOGW(TAG, "camera direct close failed: %s", esp_err_to_name(close_err));
        }
    }

cleanup_vfs:
    if (jpeg_data != NULL) {
        camera_free_buffer(jpeg_data);
    }
    err = camera_close();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "camera close failed after capture test: %s", esp_err_to_name(err));
    }
}

static void k10_camera_capture_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2500));
    k10_verify_camera_capture();
    vTaskDelete(NULL);
}

static esp_err_t k10_get_i2s_audio_format(const periph_i2s_config_t *i2s_cfg,
                                          uint32_t *sample_rate,
                                          uint8_t *channels,
                                          uint8_t *bits_per_sample)
{
    if (i2s_cfg == NULL || sample_rate == NULL || channels == NULL || bits_per_sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (i2s_cfg->mode == I2S_COMM_MODE_STD) {
        *sample_rate = i2s_cfg->i2s_cfg.std.clk_cfg.sample_rate_hz;
        *channels = (i2s_cfg->i2s_cfg.std.slot_cfg.slot_mode == I2S_SLOT_MODE_STEREO) ? 2 : 1;
        *bits_per_sample = (uint8_t)i2s_cfg->i2s_cfg.std.slot_cfg.data_bit_width;
        return ESP_OK;
    }

#if CONFIG_SOC_I2S_SUPPORTS_TDM
    if (i2s_cfg->mode == I2S_COMM_MODE_TDM) {
        *sample_rate = i2s_cfg->i2s_cfg.tdm.clk_cfg.sample_rate_hz;
        *channels = (uint8_t)i2s_cfg->i2s_cfg.tdm.slot_cfg.total_slot;
        *bits_per_sample = (uint8_t)i2s_cfg->i2s_cfg.tdm.slot_cfg.data_bit_width;
        return ESP_OK;
    }
#endif

#if CONFIG_SOC_I2S_SUPPORTS_PDM_TX
    if (i2s_cfg->mode == I2S_COMM_MODE_PDM && (i2s_cfg->direction & I2S_DIR_TX)) {
        *sample_rate = i2s_cfg->i2s_cfg.pdm_tx.clk_cfg.sample_rate_hz;
        *channels = (i2s_cfg->i2s_cfg.pdm_tx.slot_cfg.slot_mode == I2S_SLOT_MODE_STEREO) ? 2 : 1;
        *bits_per_sample = (uint8_t)i2s_cfg->i2s_cfg.pdm_tx.slot_cfg.data_bit_width;
        return ESP_OK;
    }
#endif

#if CONFIG_SOC_I2S_SUPPORTS_PDM_RX
    if (i2s_cfg->mode == I2S_COMM_MODE_PDM && (i2s_cfg->direction & I2S_DIR_RX)) {
        *sample_rate = i2s_cfg->i2s_cfg.pdm_rx.clk_cfg.sample_rate_hz;
        *channels = (i2s_cfg->i2s_cfg.pdm_rx.slot_cfg.slot_mode == I2S_SLOT_MODE_STEREO) ? 2 : 1;
        *bits_per_sample = (uint8_t)i2s_cfg->i2s_cfg.pdm_rx.slot_cfg.data_bit_width;
        return ESP_OK;
    }
#endif

    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t k10_play_audio_tone(esp_codec_dev_handle_t codec_dev,
                                     uint32_t sample_rate,
                                     uint8_t channels,
                                     uint8_t bits_per_sample,
                                     uint32_t freq_hz,
                                     uint32_t duration_ms,
                                     int volume_pct)
{
    uint8_t bytes_per_sample = (uint8_t)(bits_per_sample / 8);
    uint32_t chunk_frames = 0;
    uint32_t total_frames = 0;
    uint32_t frames_written = 0;
    float gain_scale = 0.0f;
    float amplitude = 0.0f;
    float phase = 0.0f;
    float phase_step = 0.0f;
    int16_t *buf = NULL;

    if (codec_dev == NULL || sample_rate == 0 || channels == 0 || bits_per_sample != 16 ||
        bytes_per_sample != sizeof(int16_t)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (freq_hz >= sample_rate / 2) {
        return ESP_ERR_INVALID_ARG;
    }

    chunk_frames = K10_AUDIO_CHUNK_BYTES / (channels * bytes_per_sample);
    if (chunk_frames == 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    total_frames = (uint32_t)(((uint64_t)sample_rate * duration_ms) / 1000);

    if (volume_pct > 0) {
        float gain_db = ((float)volume_pct - 100.0f) * 0.5f;
        gain_scale = powf(10.0f, gain_db / 20.0f);
    }
    amplitude = 32767.0f * gain_scale;
    phase_step = 2.0f * (float)M_PI * (float)freq_hz / (float)sample_rate;

    buf = (int16_t *)malloc(chunk_frames * channels * sizeof(int16_t));
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    while (frames_written < total_frames) {
        uint32_t frames_this_chunk = total_frames - frames_written;
        if (frames_this_chunk > chunk_frames) {
            frames_this_chunk = chunk_frames;
        }

        for (uint32_t i = 0; i < frames_this_chunk; ++i) {
            int16_t sample = (int16_t)(sinf(phase) * amplitude);
            for (uint8_t ch = 0; ch < channels; ++ch) {
                buf[i * channels + ch] = sample;
            }
            phase += phase_step;
            if (phase >= 2.0f * (float)M_PI) {
                phase -= 2.0f * (float)M_PI;
            }
        }

        if (esp_codec_dev_write(codec_dev, buf,
                                (int)(frames_this_chunk * channels * sizeof(int16_t))) != ESP_CODEC_DEV_OK) {
            free(buf);
            return ESP_FAIL;
        }
        frames_written += frames_this_chunk;
    }

    free(buf);
    return ESP_OK;
}

static void k10_verify_audio_output(void)
{
    dev_audio_codec_handles_t *codec_handles = NULL;
    dev_audio_codec_config_t *codec_cfg = NULL;
    periph_i2s_config_t *i2s_cfg = NULL;
    esp_codec_dev_sample_info_t sample_info = {0};
    void *config = NULL;
    void *periph_config = NULL;
    uint32_t sample_rate = 0;
    uint8_t channels = 0;
    uint8_t bits_per_sample = 0;
    esp_err_t err;
    int ret;

    err = esp_board_manager_init_device_by_name(K10_AUDIO_DAC_NAME);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "audio output test skipped: init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_board_manager_get_device_handle(K10_AUDIO_DAC_NAME, (void **)&codec_handles);
    if (err != ESP_OK || codec_handles == NULL || codec_handles->codec_dev == NULL) {
        ESP_LOGW(TAG, "audio output test skipped: handle unavailable: %s", esp_err_to_name(err));
        return;
    }

    err = esp_board_manager_get_device_config(K10_AUDIO_DAC_NAME, &config);
    if (err != ESP_OK || config == NULL) {
        ESP_LOGW(TAG, "audio output test skipped: config unavailable: %s", esp_err_to_name(err));
        return;
    }
    codec_cfg = (dev_audio_codec_config_t *)config;
    if (!codec_cfg->dac_enabled || codec_cfg->i2s_cfg.name == NULL) {
        ESP_LOGW(TAG, "audio output test skipped: invalid DAC config");
        return;
    }

    err = esp_board_manager_get_periph_config(codec_cfg->i2s_cfg.name, &periph_config);
    if (err != ESP_OK || periph_config == NULL) {
        ESP_LOGW(TAG, "audio output test skipped: periph config unavailable: %s", esp_err_to_name(err));
        return;
    }
    i2s_cfg = (periph_i2s_config_t *)periph_config;
    err = k10_get_i2s_audio_format(i2s_cfg, &sample_rate, &channels, &bits_per_sample);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "audio output test skipped: unsupported I2S format: %s", esp_err_to_name(err));
        return;
    }

    sample_info.sample_rate = sample_rate;
    sample_info.channel = channels;
    sample_info.bits_per_sample = bits_per_sample;
    ret = esp_codec_dev_open(codec_handles->codec_dev, &sample_info);
    if (ret != ESP_CODEC_DEV_OK) {
        ESP_LOGW(TAG, "audio output open failed: ret=%d", ret);
        return;
    }

    ret = esp_codec_dev_set_out_vol(codec_handles->codec_dev, K10_AUDIO_TONE_VOLUME);
    if (ret != ESP_CODEC_DEV_OK && ret != ESP_CODEC_DEV_NOT_SUPPORT) {
        ESP_LOGW(TAG, "audio output set volume failed: ret=%d", ret);
        goto cleanup;
    }

    ESP_LOGI(TAG, "audio output verified: sample_rate=%" PRIu32 " channels=%u bits=%u",
             sample_rate, channels, bits_per_sample);

    err = k10_play_audio_tone(codec_handles->codec_dev, sample_rate, channels, bits_per_sample,
                              523, 180, K10_AUDIO_TONE_VOLUME);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
        err = k10_play_audio_tone(codec_handles->codec_dev, sample_rate, channels, bits_per_sample,
                                  659, 180, K10_AUDIO_TONE_VOLUME);
    }
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
        err = k10_play_audio_tone(codec_handles->codec_dev, sample_rate, channels, bits_per_sample,
                                  784, 240, K10_AUDIO_TONE_VOLUME);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "audio tone sequence failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    ESP_LOGI(TAG, "audio tone sequence submitted successfully");
    vTaskDelay(pdMS_TO_TICKS(300));

cleanup:
    ret = esp_codec_dev_close(codec_handles->codec_dev);
    if (ret != ESP_CODEC_DEV_OK) {
        ESP_LOGW(TAG, "audio output close failed: ret=%d", ret);
    }
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
    if (xTaskCreate(k10_camera_capture_task, "k10_cam_chk", 6144, NULL, 4, NULL) != pdPASS) {
        ESP_LOGW(TAG, "failed to create deferred camera capture task");
    }

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
    k10_verify_audio_output();
    k10_release_probe_resources();
    return ESP_OK;
}
