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
#include "tcp_client_stream.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0))
#include "esp_netif.h"
#else
#include "tcpip_adapter.h"
#endif


#define CODEC_SAMPLE_RATE 48000
#define CODEC_CHANNEL 2
#define CODEC_BIT_RATE 48000 // 32000
#define OPUS_COMPLEXITY 5    // 5: nghe như MIDI :(, thử để 10 cho tăng chất lượng xem sao! (set >= 8 là treo do CPU 0: el-opus)
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_NUM_4))

extern audio_hal_func_t AUDIO_CODEC_ES8388_DEFAULT_HANDLE;

static const char *TAG = "TCP MP3 Streamer Writer";
static bool board_init = false;
static audio_board_handle_t m_board_handler = 0;

static audio_pipeline_handle_t pipeline;
static audio_element_handle_t tcp_stream_reader, i2s_stream_writer;


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
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_log_level_set("wifi", ESP_LOG_INFO);
    esp_log_level_set("FFS", ESP_LOG_INFO);
    esp_log_level_set("TCP_STREAM", ESP_LOG_VERBOSE);
    esp_log_level_set("OPUS_ENCODER", ESP_LOG_VERBOSE);
}
///////////////////////////


static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGE(TAG, "DARWINN got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
        vTaskDelay(3000/portTICK_RATE_MS);
        audio_pipeline_run(pipeline);
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

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(GPIO_NUM_4, 1);
    // Init audio hal to communicate with codec
    ESP_LOGI(TAG, "[2.0] Start codec chip");
    audio_board_handle_t board_handle = esp_custom_board_handle_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    audio_hal_set_volume(board_handle->audio_hal, 100);
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[2.1] Create tcp client stream to read data");
    tcp_stream_cfg_t tcp_cfg = TCP_STREAM_CFG_DEFAULT();
    tcp_cfg.type = AUDIO_STREAM_READER;
    tcp_cfg.port = CONFIG_TCP_PORT;
    tcp_cfg.host = CONFIG_TCP_URL;
    tcp_cfg.ext_stack = false;
    tcp_stream_reader = tcp_stream_init(&tcp_cfg);
    AUDIO_NULL_CHECK(TAG, tcp_stream_reader, return);

    ESP_LOGI(TAG, "[2.2] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_cfg.multi_out_num = 1;
    i2s_cfg.task_core = 1;
    sample_rate = 16000;

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    i2s_cfg.chan_cfg.id = CODEC_ADC_I2S_PORT;
    i2s_cfg.std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    i2s_cfg.std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    i2s_cfg.std_cfg.clk_cfg.sample_rate_hz = sample_rate;
#else
    //i2s_cfg.i2s_port = CODEC_ADC_I2S_PORT;
    //i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    //i2s_cfg.i2s_config.sample_rate = sample_rate;
#endif // (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[2.4] Register all elements to audio pipeline111");
    audio_pipeline_register(pipeline, tcp_stream_reader, "tcp");
    audio_pipeline_register(pipeline, i2s_stream_writer,  "i2s");

    ESP_LOGI(TAG, "[2.5] Link it tcp-->i2s_stream-->[codec_chip]");
    const char *link_tag[3] = {"tcp", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);
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

    // Example of using an audio event -- START
    ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
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

        // if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT
        //     && msg.source == (void *) mp3_decoder
        //     && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
        //     audio_element_info_t music_info = {0};
        //     audio_element_getinfo(mp3_decoder, &music_info);

        //     ESP_LOGI(TAG, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
        //              music_info.sample_rates, music_info.bits, music_info.channels);

        //     i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
        //     continue;
        // }

        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            ESP_LOGW(TAG, "[ * ] Stop event received");
        }
    }
        ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");
        audio_pipeline_stop(pipeline);
        audio_pipeline_wait_for_stop(pipeline);
        audio_pipeline_terminate(pipeline);

        /* Terminate the pipeline before removing the listener */
        audio_pipeline_unregister(pipeline, tcp_stream_reader);
        audio_pipeline_unregister(pipeline, i2s_stream_writer);

        audio_pipeline_remove_listener(pipeline);

        /* Stop all peripherals before removing the listener */
        esp_periph_set_stop_all(set);
        audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

        /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
        audio_event_iface_destroy(evt);

        /* Release all resources */
        audio_pipeline_deinit(pipeline);
        audio_element_deinit(tcp_stream_reader);
        audio_element_deinit(i2s_stream_writer);
        esp_periph_set_destroy(set);
}