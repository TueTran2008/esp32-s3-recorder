/* Record wav and amr to SD card
 * g

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

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
#include "periph_sdcard.h"
#include "raw_stream.h"
#include "wav_encoder.h"
#include <inttypes.h>
#include <mbedtls/des.h>
#include <stdbool.h>
#include <string.h>

typedef enum { BOARD_EVENT_RECORD = 0, BOARD_EVENT_STOP_RECORD = 1 } board_event_t;
#define CONFIG_GPIO_SOUND_TRIG (17)
#define GPIO_INPUT_PIN_SEL ((1ULL << CONFIG_GPIO_SOUND_TRIG))
#define TIMER_WAIT_THRESHOLD 10

extern audio_hal_func_t AUDIO_CODEC_ES8388_DEFAULT_HANDLE;

static bool m_board_is_recording = false;
static const char *TAG = "PIPELINR_REC_WAV_AMR_SDCARD";
static bool board_init = false;
static audio_board_handle_t m_board_handler = 0;
static bool pwm_init = false;
static QueueHandle_t gpio_evt_queue = NULL;

// Dummy key (must be 8 bytes for DES)
const unsigned char dummy_key[8] = "hihehahu";

// Encrypt WAV file
void encrypt_wav(const char *input_path, const char *output_path) {
    FILE *input = fopen(input_path, "rb");
    FILE *output = fopen(output_path, "wb");
    if (!input || !output) {
        ESP_LOGE(TAG, "Failed to open files.");
        return;
    }

    unsigned char header[HEADER_SIZE];
    fread(header, 1, HEADER_SIZE, input);   // Read WAV header
    fwrite(header, 1, HEADER_SIZE, output); // Write WAV header to output

    unsigned char input_block[BLOCK_SIZE];
    unsigned char output_block[BLOCK_SIZE];
    size_t read_size;

    mbedtls_des_context ctx;
    mbedtls_des_init(&ctx);
    mbedtls_des_setkey_enc(&ctx, dummy_key);

    while ((read_size = fread(input_block, 1, BLOCK_SIZE, input)) > 0) {
        if (read_size < BLOCK_SIZE) { // Padding if not a full block
            memset(input_block + read_size, BLOCK_SIZE - read_size, BLOCK_SIZE - read_size);
        }
        mbedtls_des_crypt_ecb(&ctx, input_block, output_block); // Encrypt
        fwrite(output_block, 1, BLOCK_SIZE, output);
    }

    fclose(input);
    fclose(output);
    mbedtls_des_free(&ctx);
    ESP_LOGI(TAG, "Encryption complete. Saved to %s", output_path);
}

// Decrypt WAV file
void decrypt_wav(const char *input_path, const char *output_path) {
    FILE *input = fopen(input_path, "rb");
    FILE *output = fopen(output_path, "wb");
    if (!input || !output) {
        ESP_LOGE(TAG, "Failed to open files.");
        return;
    }

    unsigned char header[HEADER_SIZE];
    fread(header, 1, HEADER_SIZE, input);   // Read WAV header
    fwrite(header, 1, HEADER_SIZE, output); // Write WAV header to output

    unsigned char input_block[BLOCK_SIZE];
    unsigned char output_block[BLOCK_SIZE];
    size_t read_size;

    mbedtls_des_context ctx;
    mbedtls_des_init(&ctx);
    mbedtls_des_setkey_dec(&ctx, dummy_key);

    while ((read_size = fread(input_block, 1, BLOCK_SIZE, input)) > 0) {
        mbedtls_des_crypt_ecb(&ctx, input_block, output_block); // Decrypt
        fwrite(output_block, 1, BLOCK_SIZE, output);
    }

    fclose(input);
    fclose(output);
    mbedtls_des_free(&ctx);
    ESP_LOGI(TAG, "Decryption complete. Saved to %s", output_path);
}
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

    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

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
}

void app_main() {

    board_event_t event;
    int sample_rate = 0;
    audio_pipeline_handle_t pipeline_wav;
    audio_element_handle_t wav_fatfs_stream_writer, i2s_stream_reader, wav_encoder /*des_fatfs_stream_writer, des_encrypter*/;
    uint32_t record_time = 0;
    int volume = 0;
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    uint32_t timer_signal_off_count = 0;

    log_init();
    gpio_init();
    pwm_pin_init();
    pwm_update_output(14);

    ESP_LOGI(TAG, "[1.0] Mount sdcard");
    // Initialize peripherals management

    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    // Initialize SD Card peripheral
    audio_board_sdcard_init(set, SD_MODE_SPI);

    // Init audio hal to communicate with codec
    ESP_LOGI(TAG, "[2.0] Start codec chip");
    audio_board_handle_t board_handle = esp_custom_board_handle_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

    audio_hal_set_volume(board_handle->audio_hal, 100);
    audio_hal_get_volume(board_handle->audio_hal, &volume);
    gpio_evt_queue = xQueueCreate(10, sizeof(board_event_t));

    ESP_LOGI(TAG, "[3.0] Create audio pipeline_wav for recording");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline_wav = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline_wav);

    ESP_LOGI(TAG, "[3.1] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_READER;
    i2s_cfg.multi_out_num = 1;
    i2s_cfg.task_core = 1;
    sample_rate = 16000;

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    i2s_cfg.chan_cfg.id = CODEC_ADC_I2S_PORT;
    i2s_cfg.std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    i2s_cfg.std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    i2s_cfg.std_cfg.clk_cfg.sample_rate_hz = sample_rate;
#else
    i2s_cfg.i2s_port = CODEC_ADC_I2S_PORT;
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_cfg.i2s_config.sample_rate = sample_rate;
#endif // (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))

    i2s_stream_reader = i2s_stream_init(&i2s_cfg);

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

    ESP_LOGI(TAG, "[3.5] Link it together "
                  "[codec_chip]-->i2s_stream-->wav_encoder-->fatfs_stream-->[sdcard]");
    const char *link_wav[3] = {"i2s", "wav", "wav_file"};
    audio_pipeline_link(pipeline_wav, &link_wav[0], 3);

    ESP_LOGI(TAG, "[3.6] Set up  uri (file as fatfs_stream, wav as wav encoder)");
    audio_element_set_uri(wav_fatfs_stream_writer, "/sdcard/rec_out.wav");

    ESP_LOGI(TAG, "Get board volume :%d", volume);

    ////////////////////////////////////////////////////////////
    // ESP_LOGI(TAG, "[4.0] Create audio pipeline_des for recording");
    // audio_pipeline_cfg_t pipeline_cfg_dup = DEFAULT_AUDIO_PIPELINE_CONFIG();
    // pipeline_des = audio_pipeline_init(&pipeline_cfg_dup);
    // mem_assert(pipeline_des);
    //
    // ESP_LOGI(TAG, "[4.1] Create des encrypt to encrypt wav format");
    //
    // ESP_LOGI(TAG, "[3.2] Create wav encoder to encode wav format");
    // wav_encoder_cfg_t wav_cfg_copy = DEFAULT_WAV_ENCODER_CONFIG();
    // audio_element_handle_t wav_encoder_copy = wav_encoder_init(&wav_cfg_copy);
    //
    // ESP_LOGI(TAG, "[4.1] Create raw stream to write data");
    // raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    // raw_cfg.type = AUDIO_STREAM_READER;
    // audio_element_handle_t el_raw_reader = raw_stream_init(&raw_cfg);
    //
    // des_encrypt_cfg_t des_cfg = DEFAULT_DES_ENCODER_CONFIG();
    // des_cfg.block_size = BLOCK_SIZE;
    // des_cfg.header_size = HEADER_SIZE;
    // des_cfg.key = (uint8_t *)dummy_key;
    // des_encrypter = des_encoder_init(&des_cfg);
    //
    // ESP_LOGI(TAG, "[4.2] Create fatfs stream to write des data to sdcard");
    // fatfs_stream_cfg_t des_fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    // des_fatfs_cfg.type = AUDIO_STREAM_WRITER;
    // des_fatfs_cfg.task_core = 1;
    // des_fatfs_stream_writer = fatfs_stream_init(&des_fatfs_cfg);
    //
    // audio_pipeline_register(pipeline_des, el_raw_reader, "des_raw");
    // audio_pipeline_register(pipeline_des, des_encrypter, "des_encrypt");
    // audio_pipeline_register(pipeline_des, des_fatfs_stream_writer, "des_file");
    // audio_pipeline_register(pipeline_des, wav_encoder_copy, "wav_copy");
    //
    // const char *link_amr[4] = {"des_raw", "des_encrypt", "wav_copy", "des_file"};
    // audio_pipeline_link(pipeline_des, &link_amr[0], 4);
    // ESP_LOGI(TAG, "[4.3] Create ringbuf to link  i2s");
    // ringbuf_handle_t rb = audio_element_get_output_ringbuf(el_raw_reader);
    // audio_element_set_multi_output_ringbuf(i2s_stream_reader, rb, 0);
    //
    // ESP_LOGI(TAG, "[4.4] Set up  uri (file as fatfs_stream, wav as wav encoder)");
    // audio_element_set_uri(des_fatfs_stream_writer, "/sdcard/rec_des.wav");
    // ESP_LOGI(TAG, "[4.5] Link it together raw_stream-->wav_encoder-->des encrypt-->fatfs_stream-->[sdcard]");
    /////////////////////////////////////////////////////////////////////
    audio_pipeline_run(pipeline_wav);
    // audio_pipeline_run(pipeline_des);
    m_board_is_recording = true;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &event, portTICK_RATE_MS * 0)) {
            if (event == BOARD_EVENT_RECORD && m_board_is_recording == false) {
                m_board_is_recording = true;

                ESP_LOGI(TAG, "[4.7] Set up  uri (file as fatfs_stream, wav as wav encoder)");

                ESP_LOGI(TAG, "[6.0] start audio_pipeline");
                audio_pipeline_run(pipeline_wav);
                // audio_pipeline_run(pipeline_des);

            } else if (event == BOARD_EVENT_STOP_RECORD && m_board_is_recording == true) {
                if (pipeline_wav) {
                    audio_pipeline_stop(pipeline_wav);
                    audio_pipeline_wait_for_stop(pipeline_wav);
                    audio_pipeline_terminate(pipeline_wav);
                    audio_pipeline_reset_ringbuffer(pipeline_wav);
                    audio_pipeline_reset_elements(pipeline_wav);
                    //////
                    // audio_pipeline_stop(pipeline_des);
                    // audio_pipeline_wait_for_stop(pipeline_des);
                    // audio_pipeline_terminate(pipeline_des);
                    // audio_pipeline_reset_ringbuffer(pipeline_des);
                    // audio_pipeline_reset_elements(pipeline_des);
                    ESP_LOGI(TAG, "[8.0] Stop audio_pipeline");
                    // encrypt file
                    encrypt_wav("/sdcard/rec_out.wav", "/sdcard/des.wav");
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
            } else {
                ESP_LOGI(TAG, "Record for %d milliseconds", (unsigned int)record_time);
            }
        }
        // read gpio event
        uint32_t sound_trig_level = gpio_get_level(CONFIG_GPIO_SOUND_TRIG);
        if (sound_trig_level == 0) {
            if (m_board_is_recording == false) {
                ESP_LOGW(TAG, "Detect sound trigger value low -> send event start recording");
                // m_board_is_recording = false;
                board_event_t event = BOARD_EVENT_RECORD;
                xQueueSend(gpio_evt_queue, &event, 0);
            } else {
                timer_signal_off_count = 0;
            }
        } else {
            if (m_board_is_recording == true) {
                timer_signal_off_count++;
                if (timer_signal_off_count > SOUND_TRIGGER_OFF_WAIT) {
                    ESP_LOGW(TAG, "Sound trigger off for 5 seconds -> stopping");
                    // m_board_is_recording = false;
                    board_event_t event = BOARD_EVENT_STOP_RECORD;
                    xQueueSend(gpio_evt_queue, &event, 0);

                } // wait 5s of of sound trigger off to kill
            }
        }
        ESP_LOGI(TAG, "Sound trigger level: %u", (unsigned int)sound_trig_level);
        vTaskDelay(portTICK_RATE_MS * 100);
    }
}
