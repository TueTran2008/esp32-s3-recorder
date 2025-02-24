#include "audio_error.h"

#include "audio_hal.h"
#include "audio_mem.h"
#include "audio_pipeline.h"
#include "board.h"
#include "board_def.h"
// #include "board_recorder.h"
#include "des_encoder.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "fatfs_stream.h"
#include "freertos/idf_additions.h"
#include "i2s_stream.h"
#include "nvs_flash.h"
#include "periph_sdcard.h"
#include "raw_stream.h"
#include "wav_encoder.h"
#include "wifi_login.h"
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

typedef enum { BOARD_EVENT_RECORD = 0, BOARD_EVENT_STOP_RECORD = 1 } board_event_t;
#define CONFIG_GPIO_SOUND_TRIG (17)
#define GPIO_INPUT_PIN_SEL ((1ULL << CONFIG_GPIO_SOUND_TRIG))
#define TIMER_WAIT_THRESHOLD 10
#define CONFIG_SERVER_URI "ws://103.252.136.73:8000/ws/send/"

extern audio_hal_func_t AUDIO_CODEC_ES8388_DEFAULT_HANDLE;

static bool m_board_is_recording = false;
static const char *TAG = "PIPELINR_REC_WAV_AMR_SDCARD";
static bool board_init = false;
static audio_board_handle_t m_board_handler = 0;
static bool pwm_init = false;
static QueueHandle_t gpio_evt_queue = NULL;
static uint32_t timer_signal_off_count = 0;

static bool count_signal_off = false;

static void pwm_pin_init(void) {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                      .duty_resolution = LEDC_DUTY_RES,
                                      .timer_num = LEDC_TIMER,
                                      .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
                                      .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_MODE,
                                          .channel = LEDC_CHANNEL,
                                          .timer_sel = LEDC_TIMER,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .gpio_num = LEDC_OUTPUT_IO,
                                          .duty = 0, // Set duty to 0%
                                          .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    pwm_init = true;
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t sound_trig_level = gpio_get_level(CONFIG_GPIO_SOUND_TRIG);
    if (sound_trig_level == false) {
        count_signal_off = false;
        if (m_board_is_recording == false) {
            static board_event_t event = BOARD_EVENT_RECORD;
            xQueueSendFromISR(gpio_evt_queue, &event, NULL);
        }
    } else {
        if (m_board_is_recording == true) {
            count_signal_off = true;
            timer_signal_off_count = 0;
        }
    }
}

static void pwm_update_output(uint32_t duty) {

    if (pwm_init == false) {
        pwm_pin_init();
    }
    if (duty > 100) {
        duty = 100;
        ESP_LOGW(TAG, "Set duty %u > 100 -> Duty = 100", (unsigned int)duty);
    }
    uint32_t l_duty = (duty * PWM_RESOLUTION) / 100;
    ESP_LOGW(TAG, "ESP PWM value %u", (unsigned int)l_duty);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, l_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

static void gpio_init(void) {
    gpio_config_t io_conf = {};

    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 0;

    io_conf.intr_type = GPIO_INTR_ANYEDGE;

    gpio_config(&io_conf);

    // change gpio interrupt type for one pin
    gpio_set_intr_type(CONFIG_GPIO_SOUND_TRIG, GPIO_INTR_ANYEDGE);

    // install gpio isr service
    // gpio_install_isr_service(0); // need because the SD card already did this
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(CONFIG_GPIO_SOUND_TRIG, gpio_isr_handler, NULL);
    ESP_LOGI(TAG, "Custom board Sound Trigger has been initialized");
}

static audio_board_handle_t esp_custom_board_handle_init(void) {
    if (board_init) {
        ESP_LOGW(TAG, "Custom board audio hal has been initialized!");
        return m_board_handler;
    }
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG(); // config from board_def.h
    m_board_handler = (audio_board_handle_t)audio_calloc(1, sizeof(struct audio_board_handle));

    AUDIO_MEM_CHECK(TAG, m_board_handler, return NULL);
    m_board_handler->audio_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES8388_DEFAULT_HANDLE);

    AUDIO_MEM_CHECK(TAG, m_board_handler->audio_hal, return NULL);
    board_init = true;
    return m_board_handler;
}

static void log_init(void) {
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("wifi", ESP_LOG_INFO);
    esp_log_level_set("FFS", ESP_LOG_INFO);
}
///////////////////////////
#define EXAMPLE_AUDIO_SAMPLE_RATE (16000)
#define EXAMPLE_AUDIO_BITS (16)
#define EXAMPLE_AUDIO_CHANNELS (1)
audio_element_handle_t http_stream_writer;
#include "esp_http_client.h"
#include "http_stream.h"

esp_err_t _http_stream_event_handle(http_stream_event_msg_t *msg) {
    esp_http_client_handle_t http = (esp_http_client_handle_t)msg->http_client;
    char len_buf[16];
    static int total_write = 0;

    if (msg->event_id == HTTP_STREAM_PRE_REQUEST) {
        // set header
        ESP_LOGI(TAG, "[ + ] HTTP client HTTP_STREAM_PRE_REQUEST, lenght=%d", msg->buffer_len);
        esp_http_client_set_method(http, HTTP_METHOD_POST);
        char dat[10] = {0};
        snprintf(dat, sizeof(dat), "%d", EXAMPLE_AUDIO_SAMPLE_RATE);
        esp_http_client_set_header(http, "x-audio-sample-rates", dat);
        memset(dat, 0, sizeof(dat));
        snprintf(dat, sizeof(dat), "%d", EXAMPLE_AUDIO_BITS);
        esp_http_client_set_header(http, "x-audio-bits", dat);
        memset(dat, 0, sizeof(dat));
        snprintf(dat, sizeof(dat), "%d", EXAMPLE_AUDIO_CHANNELS);
        esp_http_client_set_header(http, "x-audio-channel", dat);
        total_write = 0;
        return ESP_OK;
    }

    if (msg->event_id == HTTP_STREAM_ON_REQUEST) {
        // write data
        int wlen = sprintf(len_buf, "%x\r\n", msg->buffer_len);
        if (esp_http_client_write(http, len_buf, wlen) <= 0) {
            return ESP_FAIL;
        }
        if (esp_http_client_write(http, msg->buffer, msg->buffer_len) <= 0) {
            return ESP_FAIL;
        }
        if (esp_http_client_write(http, "\r\n", 2) <= 0) {
            return ESP_FAIL;
        }
        total_write += msg->buffer_len;
        printf("\033[A\33[2K\rTotal bytes written: %d\n", total_write);
        return msg->buffer_len;
    }

    if (msg->event_id == HTTP_STREAM_POST_REQUEST) {
        ESP_LOGI(TAG, "[ + ] HTTP client HTTP_STREAM_POST_REQUEST, write end chunked marker");
        if (esp_http_client_write(http, "0\r\n\r\n", 5) <= 0) {
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    if (msg->event_id == HTTP_STREAM_FINISH_REQUEST) {
        ESP_LOGI(TAG, "[ + ] HTTP client HTTP_STREAM_FINISH_REQUEST");
        char *buf = calloc(1, 64);
        assert(buf);
        int read_len = esp_http_client_read(http, buf, 64);
        if (read_len <= 0) {
            free(buf);
            return ESP_FAIL;
        }
        buf[read_len] = 0;
        ESP_LOGI(TAG, "Got HTTP Response = %s", (char *)buf);
        free(buf);
        return ESP_OK;
    }
    return ESP_OK;
}
void app_main() {

    board_event_t event;
    int channel_format = I2S_CHANNEL_TYPE_RIGHT_LEFT;
    int sample_rate = 16000;
    audio_pipeline_handle_t pipeline_wav, pipeline_http;
    audio_element_handle_t wav_fatfs_stream_writer, i2s_stream_reader, wav_encoder, http_stream_writer;
    uint32_t record_time = 0;
    int volume = 0;
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    log_init();
    pwm_pin_init();
    pwm_update_output(10);

    wifi_login_init();
    ESP_LOGI(TAG, "[1.0] Mount sdcard");
    // Initialize peripherals management

    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    // Initialize SD Card peripheral
    audio_board_sdcard_init(set, SD_MODE_SPI);

    // Init audio hal to communicate with codec
    ESP_LOGI(TAG, "[2.0] Start codec chip");
    audio_board_handle_t board_handle = esp_custom_board_handle_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

    audio_hal_set_volume(board_handle->audio_hal, 80);
    audio_hal_get_volume(board_handle->audio_hal, &volume);

    ESP_LOGI(TAG, "[3.0] Create audio pipeline_wav for recording");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline_wav = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline_wav);

    pipeline_http = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline_http);

    ESP_LOGI(TAG, "[3.1] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_READER;
    i2s_cfg.multi_out_num = 1;
    i2s_cfg.task_core = 1;
    sample_rate = 16000;

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    i2s_cfg.chan_cfg.id = CODEC_ADC_I2S_PORT;
    // i2s_cfg.std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    // i2s_cfg.std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    i2s_cfg.std_cfg.clk_cfg.sample_rate_hz = sample_rate;
#else
    i2s_cfg.i2s_port = CODEC_ADC_I2S_PORT;
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_cfg.i2s_config.sample_rate = sample_rate;
#endif // (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))

    i2s_stream_set_channel_type(&i2s_cfg, channel_format);
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);
    /*Config http stream writer*/

    /////////////////
    http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
    http_cfg.type = AUDIO_STREAM_WRITER;
    http_cfg.event_handle = _http_stream_event_handle;
    http_stream_writer = http_stream_init(&http_cfg);
    ////////////////
    ESP_LOGI(TAG, "[3.2] Create wav encoder to encode wav format");
    wav_encoder_cfg_t wav_cfg = DEFAULT_WAV_ENCODER_CONFIG();
    wav_encoder = wav_encoder_init(&wav_cfg);

    ESP_LOGI(TAG, "[3.3] Create fatfs stream to write data to sdcard");
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_WRITER;
    wav_fatfs_stream_writer = fatfs_stream_init(&fatfs_cfg);

    audio_element_info_t info = AUDIO_ELEMENT_INFO_DEFAULT();
    audio_element_getinfo(i2s_stream_reader, &info);
    audio_element_setinfo(wav_fatfs_stream_writer, &info);

    ESP_LOGI(TAG, "[3.4] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline_wav, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline_wav, wav_encoder, "wav");
    audio_pipeline_register(pipeline_wav, wav_fatfs_stream_writer, "wav_file");

    audio_pipeline_register(pipeline_http, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline_http, http_stream_writer, "http");

    ESP_LOGI(TAG, "[3.5] Link it together "
                  "[codec_chip]-->i2s_stream-->wav_encoder-->fatfs_stream-->[sdcard]");
    const char *link_wav[3] = {"i2s", "wav", "wav_file"};
    const char *link_http[2] = {"i2s", "http"};

    audio_pipeline_link(pipeline_http, &link_http[0], 2);
    // audio_pipeline_link(pipeline_wav, &link_wav[0], 3);

    ESP_LOGI(TAG, "[3.6] Set up  uri (file as fatfs_stream, wav as wav encoder)");
    audio_element_info_t music_info = {0};
    audio_element_getinfo(i2s_stream_reader, &music_info);
    ESP_LOGI(TAG, "[ * ] Save the recording info to the fatfs stream writer, sample_rates=%d, bits=%d, ch=%d", music_info.sample_rates, music_info.bits, music_info.channels);
    audio_element_setinfo(wav_fatfs_stream_writer, &music_info);

    audio_element_set_uri(wav_fatfs_stream_writer, "/sdcard/rec_out.wav");

    ESP_LOGI(TAG, "Get board volume :%d", volume);
    gpio_evt_queue = xQueueCreate(10, sizeof(board_event_t));
    gpio_init(); // initialized sound trigger

    board_event_t test_event = BOARD_EVENT_RECORD;
    xQueueSend(gpio_evt_queue, &test_event, 0);

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &event, 0)) {
            if (event == BOARD_EVENT_RECORD && m_board_is_recording == false) {
                m_board_is_recording = true;

                ESP_LOGI(TAG, "[4.7] Set up  uri (file as fatfs_stream, wav as wav encoder)");
                vTaskDelay(3000 / portTICK_RATE_MS);

                while (wifi_login_connect_status() == false) {
                    ESP_LOGI(TAG, "WiFi is not connected");
                    vTaskDelay(500 / portTICK_RATE_MS);
                }
                ESP_LOGI(TAG, "[6.0] start audio_pipeline");
                audio_element_set_uri(http_stream_writer, CONFIG_SERVER_URI);
                audio_pipeline_run(pipeline_wav);
                audio_pipeline_run(pipeline_http);
            } else if (event == BOARD_EVENT_STOP_RECORD && m_board_is_recording == true) {
                if (pipeline_wav) {
                    audio_pipeline_stop(pipeline_wav);
                    audio_pipeline_wait_for_stop(pipeline_wav);
                    audio_pipeline_terminate(pipeline_wav);
                    audio_pipeline_reset_ringbuffer(pipeline_wav);
                    audio_pipeline_reset_elements(pipeline_wav);

                    audio_pipeline_stop(pipeline_http);
                    audio_pipeline_wait_for_stop(pipeline_http);
                    audio_pipeline_terminate(pipeline_http);
                    audio_pipeline_reset_ringbuffer(pipeline_http);
                    audio_pipeline_reset_elements(pipeline_http);

                    encrypt_wav("/sdcard/rec_out.wav", "/sdcard/rec_des.wav");
                    ESP_LOGI(TAG, "[8.0] Stop audio_pipeline");
                    vTaskDelay(portTICK_RATE_MS * 5000); // wait 5 second before next record
                } else {
                    ESP_LOGW(TAG, "[8.0] Stop audio pipeline but pipeline is empty");
                }
                m_board_is_recording = false;
            }
        }
        if (m_board_is_recording == true) {
            record_time++;
            if (record_time >= (RECORD_TIME_SECONDS)) {
                ESP_LOGW(TAG, "Record more than 30 seconds -> stopping");
                board_event_t event = BOARD_EVENT_STOP_RECORD;
                xQueueSend(gpio_evt_queue, &event, 0);
                record_time = 0;
            } else {
                ESP_LOGI(TAG, "Record for %u milliseconds", (unsigned int)record_time * 100);
            }
        } else {
            record_time = 0;
        }
        // read gpio event

        if (count_signal_off == true) {
            if (m_board_is_recording == true) {
                timer_signal_off_count++;
                if (timer_signal_off_count > SOUND_TRIGGER_OFF_WAIT) {
                    ESP_LOGW(TAG, "Sound trigger off for 5 seconds -> stopping");
                    // m_board_is_recording = false;
                    board_event_t event = BOARD_EVENT_STOP_RECORD;
                    xQueueSend(gpio_evt_queue, &event, 0);

                } // wait 5s of of sound trigger off to kill
            }
        } else {
            timer_signal_off_count = 0;
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}
