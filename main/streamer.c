#include "audio_element.h"
#include "audio_error.h"

#include "audio_hal.h"
#include "audio_mem.h"
#include "audio_pipeline.h"
#include "board.h"
#include "board_def.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "fatfs_stream.h"
#include "freertos/idf_additions.h"
#include "i2s_stream.h"
#include "nvs_flash.h"
#include "periph_wifi.h"
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include "http_stream.h"
#include "mp3_decoder.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"

#include "esp_audio.h"
#include "esp_decoder.h"
#include "raw_stream.h"
#include "i2s_stream.h"
#include "http_stream.h"
#include "audio_mem.h"
#include "audio_thread.h"
#include "media_lib_adapter.h"
#include "audio_idf_version.h"
#include "tcp_client_stream.h"

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0))
#include "esp_netif.h"
#else
#include "tcpip_adapter.h"
#endif
#include "esp_mrm_client.h"
#include "esp_netif.h"

#define DEFAULT_PLAY_URL "https://dl.espressif.com/dl/audio/ff-16b-2c-44100hz.mp3"
#define ESP_READ_BUFFER_SIZE    4096

#define CODEC_SAMPLE_RATE 48000
#define CODEC_CHANNEL 2
#define CODEC_BIT_RATE 48000 // 32000
#define OPUS_COMPLEXITY 5    // 5: nghe như MIDI :(, thử để 10 cho tăng chất lượng xem sao! (set >= 8 là treo do CPU 0: el-opus)
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_NUM_4))

extern audio_hal_func_t AUDIO_CODEC_ES8388_DEFAULT_HANDLE;

static const char *TAG = "TCP Streamer";
static bool board_init = false;

static audio_pipeline_handle_t pipeline;
static audio_element_handle_t i2s_stream_reader, tcp_stream_writer;

static void log_init(void) {
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_log_level_set("wifi", ESP_LOG_INFO);
    esp_log_level_set("FFS", ESP_LOG_INFO);
    esp_log_level_set("TCP_STREAM", ESP_LOG_VERBOSE);
    esp_log_level_set("OPUS_ENCODER", ESP_LOG_VERBOSE);
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // esp_event_post_to(event_loop_handle, BOARD_EVENT_BASE, BOARD_EVENT_RECORD, NULL, 0,
        //               portMAX_DELAY)
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGE(TAG, "ESP got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void app_main() {
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    int sample_rate = 0;
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

    log_init();


    ESP_LOGI(TAG, "[1.0] Wifi Connection");
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

    ESP_LOGI(TAG, "[2.0] Create audio pipeline for recording");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline_wav);

    ESP_LOGI(TAG, "[3.0] Create i2s stream to read audio data from codec chip");
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
    //i2s_cfg.i2s_port = CODEC_ADC_I2S_PORT;
    //i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    //i2s_cfg.i2s_config.sample_rate = sample_rate;
#endif // (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))

    //i2s_stream_set_channel_type(&i2s_cfg, channel_format);
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);

    /////////////////
    ESP_LOGI(TAG, "[3.1] Create tcp client stream to read data");
    tcp_stream_cfg_t tcp_cfg = TCP_STREAM_CFG_DEFAULT();
    tcp_cfg.type = AUDIO_STREAM_WRITER;
    tcp_cfg.port = CONFIG_TCP_PORT;
    tcp_cfg.host = CONFIG_TCP_URL;
    tcp_cfg.ext_stack = false;
    tcp_stream_writer = tcp_stream_init(&tcp_cfg);
    AUDIO_NULL_CHECK(TAG, tcp_stream_writer, return);

    audio_element_info_t info = AUDIO_ELEMENT_INFO_DEFAULT();
    audio_element_getinfo(i2s_stream_reader, &info);

    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline, tcp_stream_writer, "tcp");

    ESP_LOGI(TAG, "[3.2] Link it together "
                  "[codec_chip]-->i2s_stream-->tcp_stream_writer");
    // const char *link_wav[3] = {"i2s", "wav", "wav_file"};
    const char *link[2] = {"i2s", "tcp"};

    audio_pipeline_link(pipeline, &link[0], 2);

    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

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


        }
    }
}
