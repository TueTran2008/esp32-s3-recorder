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

static const char *TAG = "HTTP MP3 Streamer";
static bool board_init = false;
static audio_board_handle_t m_board_handler = 0;

static audio_pipeline_handle_t pipeline;
static audio_element_handle_t player_raw_in_h, i2s_h, http_stream_reader;
static esp_audio_handle_t player;
static esp_mrm_client_handle_t mrm_client;
static bool play_task_run;

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
static void gpio_enable_pa(void) {
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
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "ESP board got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void setup_player(esp_periph_set_handle_t set)
{
    if (player) {
        return ;
    }
    esp_audio_cfg_t cfg = DEFAULT_ESP_AUDIO_CONFIG();
    audio_board_handle_t board_handle = audio_board_init();
    cfg.vol_handle = board_handle->audio_hal;
    cfg.vol_set = (audio_volume_set)audio_hal_set_volume;
    cfg.vol_get = (audio_volume_get)audio_hal_get_volume;
    cfg.prefer_type = ESP_AUDIO_PREFER_MEM;
    cfg.resample_rate = 48000;
    cfg.evt_que = xQueueCreate(3, sizeof(esp_audio_state_t));
    player = esp_audio_create(&cfg);
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
    http_cfg.task_stack = 0;
    http_cfg.out_rb_size = 100 * 1024;
    http_stream_reader = http_stream_init(&http_cfg);

    raw_stream_cfg_t raw_reader = RAW_STREAM_CFG_DEFAULT();
    raw_reader.type = AUDIO_STREAM_READER;
    raw_reader.out_rb_size = 100 * 1024;
    player_raw_in_h = raw_stream_init(&raw_reader);
    esp_audio_input_stream_add(player, player_raw_in_h);

    // Add decoders and encoders to esp_audio
    audio_decoder_t auto_decode[] = {
        DEFAULT_ESP_AMRNB_DECODER_CONFIG(),
        DEFAULT_ESP_AMRWB_DECODER_CONFIG(),
        DEFAULT_ESP_FLAC_DECODER_CONFIG(),
        DEFAULT_ESP_OGG_DECODER_CONFIG(),
        DEFAULT_ESP_OPUS_DECODER_CONFIG(),
        DEFAULT_ESP_MP3_DECODER_CONFIG(),
        DEFAULT_ESP_WAV_DECODER_CONFIG(),
        DEFAULT_ESP_AAC_DECODER_CONFIG(),
        DEFAULT_ESP_M4A_DECODER_CONFIG(),
        DEFAULT_ESP_TS_DECODER_CONFIG(),
    };
    esp_decoder_cfg_t auto_dec_cfg = DEFAULT_ESP_DECODER_CONFIG();
    auto_dec_cfg.out_rb_size = 50 * 1024;
    esp_audio_codec_lib_add(player, AUDIO_CODEC_TYPE_DECODER, esp_decoder_init(&auto_dec_cfg, auto_decode, 10));

    i2s_stream_cfg_t i2s_writer = I2S_STREAM_CFG_DEFAULT();
    i2s_writer.type = AUDIO_STREAM_WRITER;
    i2s_h = i2s_stream_init(&i2s_writer);
    i2s_stream_set_clk(i2s_h, 48000, 16, 2);
    esp_audio_output_stream_add(player, i2s_h);

    // Set default volume
    esp_audio_vol_set(player, 40);
}

static int _player_get_pts()
{
    int time;
    esp_audio_time_get(player, &time);
    return time;
}

static void _multi_room_play_task(void *para)
{
    char *buf = audio_calloc(1, ESP_READ_BUFFER_SIZE);
    AUDIO_MEM_CHECK(TAG, buf, vTaskDelete(NULL); return);

    while (play_task_run) {
        int ret = audio_element_input(http_stream_reader, buf, ESP_READ_BUFFER_SIZE);
        if (AEL_IO_OK == ret) {
            audio_element_set_ringbuf_done(player_raw_in_h);
            audio_element_finish_state(player_raw_in_h);
            break;
        }
        raw_stream_write(player_raw_in_h, buf, ESP_READ_BUFFER_SIZE);
    }

    audio_element_process_deinit(http_stream_reader);
    audio_element_stop(http_stream_reader);
    free(buf);

    esp_mrm_client_master_stop(mrm_client);
    esp_mrm_client_slave_stop(mrm_client);
    ESP_LOGI(TAG, "_multi_room_play_task stop");
    vTaskDelete(NULL);
}

static esp_err_t multi_room_play_start(const char *url)
{
    audio_element_set_uri(http_stream_reader, url);
    audio_element_process_init(http_stream_reader);
    audio_element_run(http_stream_reader);

    play_task_run = true;
    if (audio_thread_create(NULL,
                            "multi_room_play", _multi_room_play_task,
                            NULL,
                            DEFAULT_MRM_TASK_STACK,
                            DEFAULT_MRM_TASK_PRIO,
                            true,
                            0) != ESP_OK) {
        ESP_LOGE(TAG, "Can not start multi_room_play service");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static int _mrm_event_handler(mrm_event_msg_t *event, void *ctx)
{
    int64_t tsf_time = 0;
    int sync = 0;

    switch ((int)event->type) {
        case MRM_EVENT_SET_URL:
            ESP_LOGI(TAG, "slave set url %s", (char *)event->data);
            multi_room_play_start((char *)event->data);
            break;
        case MRM_EVENT_GET_PTS:
            *(int *)event->data = _player_get_pts();
            break;
        case MRM_EVENT_GET_TSF:
            tsf_time = esp_wifi_get_tsf_time(ESP_IF_WIFI_STA);
            *(int64_t *)event->data = tsf_time / 1000;
            break;
        case MRM_EVENT_SET_SYNC:
            sync = *(int *)event->data;
            ESP_LOGD(TAG, "slave got sync %d", sync);
            break;
        case MRM_EVENT_SYNC_FAST:
            sync = *(int *)event->data;
            if (sync < -200) {
                sync = -200;
            }
            i2s_stream_sync_delay(i2s_h, sync);
            break;
        case MRM_EVENT_SYNC_SLOW:
            sync = *(int *)event->data;
            if (sync > 200) {
                sync = 200;
            }
            i2s_stream_sync_delay(i2s_h, sync);
            break;
        case MRM_EVENT_PLAY_STOP:
            play_task_run = false;
            break;
    }

    return ESP_OK;
}

void app_main() {
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

    log_init();

    gpio_enable_pa();

    media_lib_add_default_adapter();

    // esp_audio_play(player, AUDIO_CODEC_TYPE_DECODER, "raw://http/audio", 0);
    // Init audio hal to communicate with codec
    esp_mrm_client_config_t config = {
        .event_handler = _mrm_event_handler,
        .group_addr = DEFAULT_MRM_GROUP_ADDR,
        .sync_sock_port = DEFAULT_MRM_SYNC_SOCK_PORT,
        .ctx = NULL,
    };
    mrm_client = esp_mrm_client_create(&config);

    esp_mrm_client_slave_start(mrm_client);

    esp_mrm_client_master_start(mrm_client, DEFAULT_PLAY_URL);
    multi_room_play_start(DEFAULT_PLAY_URL);
}
