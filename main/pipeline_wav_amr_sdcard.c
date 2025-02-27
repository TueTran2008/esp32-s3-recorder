#include "audio_error.h"

#include "audio_hal.h"
#include "audio_mem.h"
#include "audio_pipeline.h"
#include "board.h"
#include "board_def.h"
#include "des_encoder.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "fatfs_stream.h"
#include "freertos/idf_additions.h"
#include "i2s_stream.h"
#include "nvs_flash.h"
#include "opus_encoder.h"
#include "periph_sdcard.h"
#include "periph_wifi.h"
#include "tcp_client_stream.h"
#include "wav_encoder.h"
#include "wifi_login.h"
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
// #include "esp_event_loop.h"

/**
 * @brief Base event for LED 1.
 */
ESP_EVENT_DECLARE_BASE(BOARD_EVENT_BASE);

/**
 * @brief Definition of the base event for LED 1.
 */
ESP_EVENT_DEFINE_BASE(BOARD_EVENT_BASE);

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
static esp_event_loop_handle_t event_loop_handle;
static bool count_signal_off = false;
static audio_pipeline_handle_t pipeline_wav, pipeline_tcp;
static audio_element_handle_t wav_fatfs_stream_writer, i2s_stream_reader, wav_encoder, tcp_stream_writer, opus_encoder;
static esp_timer_handle_t timer_handle;

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
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("wifi", ESP_LOG_INFO);
    esp_log_level_set("FFS", ESP_LOG_INFO);
    esp_log_level_set("TCP_STREAM", ESP_LOG_DEBUG);
}
///////////////////////////

static void board_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id, void *event_data) {
    static board_state_t state = BOARD_STATE_IDLE;
    switch (state) {

    case BOARD_STATE_IDLE:
        if (base == BOARD_EVENT_BASE && event_id == BOARD_EVENT_RECORD) {
            esp_timer_stop(timer_handle);
            ESP_LOGI(TAG, "[4.7] Set up  uri (file as fatfs_stream, wav as wav encoder)");
            ESP_LOGI(TAG, "[6.0] start audio_pipeline");
            // audio_pipeline_run(pipeline_wav);
            audio_pipeline_run(pipeline_tcp);
            state = BOARD_STATE_RECORDING;
        }
        break;
    case BOARD_STATE_RECORDING:
        if (base == BOARD_EVENT_BASE && event_id == BOARD_EVENT_STOP_RECORD) {
            // audio_pipeline_stop(pipeline_wav);
            // audio_pipeline_wait_for_stop(pipeline_wav);
            // audio_pipeline_terminate(pipeline_wav);
            // audio_pipeline_reset_ringbuffer(pipeline_wav);
            // audio_pipeline_reset_elements(pipeline_wav);

            audio_pipeline_stop(pipeline_tcp);
            audio_pipeline_wait_for_stop(pipeline_tcp);
            audio_pipeline_terminate(pipeline_tcp);
            audio_pipeline_reset_ringbuffer(pipeline_tcp);
            audio_pipeline_reset_elements(pipeline_tcp);
            state = BOARD_STATE_IDLE;
            esp_timer_start_once(timer_handle, 3000000);
        }
        break;
    default:
        break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // esp_event_post_to(event_loop_handle, BOARD_EVENT_BASE, BOARD_EVENT_RECORD, NULL, 0,
        //               portMAX_DELAY)
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGE(TAG, "DARWINN got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void timer_board_callback(void *arg) {
    // ESP_LOGI(TAG,"Timer expired! Posting event...\n");
    esp_event_post_to(event_loop_handle, BOARD_EVENT_BASE, BOARD_EVENT_RECORD, NULL, 0, portMAX_DELAY);
    ESP_LOGI(TAG, "TIMER 10S callback stop streaming");
}

void app_main() {
    int channel_format = I2S_CHANNEL_TYPE_RIGHT_LEFT;
    int sample_rate = 16000;
    int volume = 0;
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0))
    ESP_ERROR_CHECK(esp_netif_init());
#else
    tcpip_adapter_init();
#endif
    esp_event_loop_args_t loop_args = {.queue_size = 5, .task_name = "event_task", .task_priority = uxTaskPriorityGet(NULL), .task_stack_size = 3072, .task_core_id = tskNO_AFFINITY};

    esp_event_loop_create(&loop_args, &event_loop_handle);

    log_init();
    // pwm_pin_init();
    // pwm_update_output(10);

    // wifi_login_init();
    ESP_LOGI(TAG, "[1.0] Mount sdcard");
    // Initialize peripherals management

    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    periph_wifi_cfg_t wifi_cfg = {
        .wifi_config.sta.ssid = CONFIG_WIFI_SSID,
        .wifi_config.sta.password = CONFIG_WIFI_PASSWORD,
    };

    esp_periph_handle_t wifi_handle = periph_wifi_init(&wifi_cfg);

    // Start wifi & button peripheral
    esp_periph_start(set, wifi_handle);
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, &instance_got_ip));

    periph_wifi_wait_for_connected(wifi_handle, portMAX_DELAY);
    // Initialize SD Card peripheral
    audio_board_sdcard_init(set, SD_MODE_SPI);

    // Init audio hal to communicate with codec
    ESP_LOGI(TAG, "[2.0] Start codec chip");
    audio_board_handle_t board_handle = esp_custom_board_handle_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

    audio_hal_set_volume(board_handle->audio_hal, 60);
    audio_hal_get_volume(board_handle->audio_hal, &volume);

    ESP_LOGI(TAG, "[3.0] Create audio pipeline_wav for recording");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    // pipeline_wav = audio_pipeline_init(&pipeline_cfg);
    // mem_assert(pipeline_wav);

    pipeline_tcp = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline_tcp);

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
    // i2s_cfg.i2s_port = CODEC_ADC_I2S_PORT;
    // i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    // i2s_cfg.i2s_config.sample_rate = sample_rate;
#endif // (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))

    i2s_stream_set_channel_type(&i2s_cfg, channel_format);
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);
    /*Config http stream writer*/

    /////////////////
    ESP_LOGI(TAG, "[2.2] Create tcp client stream to read data");
    tcp_stream_cfg_t tcp_cfg = TCP_STREAM_CFG_DEFAULT();
    tcp_cfg.type = AUDIO_STREAM_WRITER;
    tcp_cfg.port = CONFIG_TCP_PORT;
    tcp_cfg.host = CONFIG_TCP_URL;
    tcp_cfg.ext_stack = false;
    tcp_stream_writer = tcp_stream_init(&tcp_cfg);
    AUDIO_NULL_CHECK(TAG, tcp_stream_writer, return);
    ////////////////
    // ESP_LOGI(TAG, "[3.2] Create wav encoder to encode wav format");
    // wav_encoder_cfg_t wav_cfg = DEFAULT_WAV_ENCODER_CONFIG();
    // wav_encoder = wav_encoder_init(&wav_cfg);

    // ESP_LOGI(TAG, "[3.3] Create fatfs stream to write data to sdcard");
    // fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    // fatfs_cfg.type = AUDIO_STREAM_WRITER;
    // wav_fatfs_stream_writer = fatfs_stream_init(&fatfs_cfg);
    // Setup Opus Encoder
    opus_encoder_cfg_t opus_cfg = DEFAULT_OPUS_ENCODER_CONFIG();
    opus_cfg.task_stack = 4096 * 4 * 4;
    opus_cfg.sample_rate = 16000; // Set sample rate
    opus_cfg.channel = 2;         // Stereo
    opus_encoder = encoder_opus_init(&opus_cfg);
    audio_element_info_t info = AUDIO_ELEMENT_INFO_DEFAULT();
    audio_element_getinfo(i2s_stream_reader, &info);
    // audio_element_setinfo(wav_fatfs_stream_writer, &info);

    ESP_LOGI(TAG, "[3.4] Register all elements to audio pipeline");
    // audio_pipeline_register(pipeline_wav, i2s_stream_reader, "i2s");
    // audio_pipeline_register(pipeline_tcp, wav_encoder, "wav");
    // audio_pipeline_register(pipeline_wav, wav_fatfs_stream_writer, "wav_file");

    audio_pipeline_register(pipeline_tcp, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline_tcp, opus_encoder, "opus");
    audio_pipeline_register(pipeline_tcp, tcp_stream_writer, "tcp");

    ESP_LOGI(TAG, "[3.5] Link it together "
                  "[codec_chip]-->i2s_stream-->wav_encoder-->fatfs_stream-->[sdcard]");
    // const char *link_wav[3] = {"i2s", "wav", "wav_file"};
    const char *link_tcp[3] = {"i2s", "opus", "tcp"};

    audio_pipeline_link(pipeline_tcp, &link_tcp[0], 3);
    // audio_pipeline_link(pipeline_wav, &link_wav[0], 3);

    ESP_LOGI(TAG, "[3.6] Set up  uri (file as fatfs_stream, wav as wav encoder)");
    audio_element_info_t music_info = {0};
    audio_element_getinfo(i2s_stream_reader, &music_info);
    ESP_LOGI(TAG, "[ * ] Save the recording info to the fatfs stream writer, sample_rates=%d, bits=%d, ch=%d", music_info.sample_rates, music_info.bits, music_info.channels);
    // opus_encoder_get_music_info
    // audio_element_setinfo(wav_fatfs_stream_writer, &music_info);
    // audio_element_set_uri(wav_fatfs_stream_writer, "/sdcard/rec_out.wav");

    ESP_LOGI(TAG, "Get board volume :%d", volume);

    const esp_timer_create_args_t timer_args = {.callback = &timer_board_callback, .name = "my_timer"};
    esp_timer_create(&timer_args, &timer_handle);

    esp_event_handler_instance_register_with(event_loop_handle, BOARD_EVENT_BASE, ESP_EVENT_ANY_ID, board_event_handler, NULL, NULL);

    esp_event_post_to(event_loop_handle, BOARD_EVENT_BASE, BOARD_EVENT_RECORD, NULL, 0, portMAX_DELAY);

    // esp_timer_start_once(timer_handle, 10000000);
    // const char *link_wav[3] = {"i2s", "wav", "wav_file"};
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline_tcp, evt);

    ESP_LOGI(TAG, "[4.2] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }
        ESP_LOGI(TAG, "[ * ] Event received:");
        ESP_LOGI(TAG, "    Source Type: %d", msg.source_type);
        ESP_LOGI(TAG, "    Command: %d", msg.cmd);
        ESP_LOGI(TAG, "    Source Handle: %p", msg.source);
        ESP_LOGI(TAG, "    Data: %p", msg.data);
        ESP_LOGI(TAG, "    Data Length: %d", msg.data_len);
        // Handle specific events

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) tcp_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_ERROR_OUTPUT) || ((int)msg.data == AEL_STATUS_ERROR_OPEN)  || ((int)msg.data == AEL_STATUS_ERROR_INPUT) || ((int)msg.data == AEL_STATUS_ERROR_PROCESS) || ((int)msg.data == AEL_STATUS_ERROR_TIMEOUT) || ((int)msg.data == AEL_STATUS_ERROR_CLOSE) || ((int)msg.data == AEL_STATUS_ERROR_UNKNOWN)  )) {
            ESP_LOGE(TAG, "TCP Stop :%d", (int)msg.data);
    esp_event_post_to(event_loop_handle, BOARD_EVENT_BASE, BOARD_EVENT_STOP_RECORD, NULL, 0, portMAX_DELAY);

        }
    }
}
