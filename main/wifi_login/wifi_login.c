#include "wifi_login.h"
#include "wifi_login_cfg.h"

#include "cJSON.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_smartconfig.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "string.h"
#include <esp_http_server.h>
#include <string.h>
#include <stdbool.h>

#define TOTAL_WIFI_AUTH_MODE 10
#define MACSTR %02x:%02x:%02x:%02x:%02x:%02x
#define WIFI_CONNECTED 0
#define WIFI_FAIL 1

static int s_retry_num = 0;

wifi_config_web m_wifi_config;
static char *wifi_str;
uint8_t m_is_wifi_connected = 0;
uint8_t m_is_wifi_ap_on = 0;
uint8_t m_is_task_turnoff_wifiap_created = 0;
uint8_t m_is_task_smartcfg_init_creatd = 0;
uint8_t m_is_wifi_store_config = 0;

static const char *TAG = "wifi ";
static const char *TAG_FFS = "FFS";
static esp_vfs_spiffs_conf_t conf = {.base_path = "/wifi", .partition_label = NULL, .max_files = 5, .format_if_mount_failed = true};

static void spiffs_write_wifiinfo();
static void spiffs_read_wifiinfo(wifi_config_t *wifi_config_store);
static void wifi_softap_turnoff_task(void *arg);
esp_timer_handle_t esp_timer_once_handle;

wifi_config_t wifi_config_sta;
// static EventGroupHandle_t s_wifi_event_group;

static uint8_t m_is_wifi_smartcfg_ok = 0;
static esp_timer_handle_t smartcfg_handle;
static void reset_smartcfg_cb(void *arg);
static const esp_timer_create_args_t reset_smartcfg = {.callback = &reset_smartcfg_cb, .arg = NULL, .name = "reset_smartcfg"};
static void reset_smartcfg_cb(void *arg) {
    ESP_LOGI(TAG, "Enter reset smartcfg event");
    if (!m_is_wifi_smartcfg_ok) {
        esp_smartconfig_stop();
        // ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
        smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();

        ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
        ESP_LOGI(TAG, "Restart Smart Config");
    }
    ESP_ERROR_CHECK(esp_timer_stop(smartcfg_handle));
}

/*Event Handler for AP event*/
static void smartconfig_init(void) {
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
}

static void smartconfig_task(void *arg) { /* if device conennectd to an Acess Point => Don't init smartconfig*/
    ESP_LOGI(TAG, "Enter Task SmartConfig for Wifi");
    while (1) {
        vTaskDelay(20000 / portTICK_RATE_MS);
        if (!m_is_wifi_connected) {
            ESP_LOGI(TAG, "Init  SmartConfig ");
            smartconfig_init();
        }
        m_is_task_smartcfg_init_creatd = 0; // change sate to task wasnt created
        vTaskDelete(NULL);                  // Delete this task
    }
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        // if (!m_is_task_smartcfg_init_creatd) {
        //     m_is_task_smartcfg_init_creatd = 1;
        //     xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 2, NULL);
        // }
        // smartconfig_init();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_DEVICE_STA_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            ESP_LOGE(TAG, "Cannot connect to wifi station");
            m_is_wifi_connected = WIFI_FAIL;
            sprintf(wifi_str, "{\"wifi_state\": \"%s\"}", "Disconnected");
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGE(TAG, "DARWINN got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        sprintf(wifi_str, "{\"wifi_state\": \"%s%s.IP Add:%d.%d.%d.%d\"}", "Connected to ", (char *)m_wifi_config.ssid, IP2STR(&event->ip_info.ip));
        // ESP_LOGE(TAG, "DARIWINNNNNNN ------------------%s", wifi_str);
        m_is_wifi_connected = WIFI_CONNECTED;
        if (!m_is_task_turnoff_wifiap_created) {
            xTaskCreate(wifi_softap_turnoff_task, "wifi_softap_turnoff_task", 4097, NULL, 1, NULL);
            ESP_LOGI(TAG, "Wifi SOFTAP turn off task created");
            m_is_task_turnoff_wifiap_created = 1;
        }
        if (m_is_wifi_store_config) // if user wifi info is able to connected => store wifi info
        {
            ESP_LOGI(TAG_FFS, "Save Wifi Info");
            spiffs_write_wifiinfo();
            esp_vfs_spiffs_unregister(conf.partition_label);
        }
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;

        uint8_t ssid[33] = {0};
        uint8_t password[65] = {0};
        uint8_t rvd_data[33] = {0};

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i = 0; i < 33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }
        wifi_config_sta = wifi_config;
        ESP_ERROR_CHECK(esp_timer_create(&reset_smartcfg, &smartcfg_handle));
        ESP_ERROR_CHECK(esp_timer_start_periodic(smartcfg_handle, 10000000));
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        esp_wifi_connect();
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        m_is_wifi_smartcfg_ok = 1;
        ESP_LOGI(TAG, "SmartConfig Done");
    }
}

/*Event Handler for STA event*/
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        // wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        // ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
        esp_smartconfig_stop();
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        // wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        // ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

static void wifi_init_softap(void) {

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg_ap = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg_ap));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    /*wifi config ap*/
    wifi_config_t wifi_config_ap = {
        .ap = {.ssid = CONFIG_DEVICE_AS_SSID,
               .ssid_len = strlen(CONFIG_DEVICE_AS_SSID),
               .channel = CONFIG_DEVICE_AS_WIFI_CHANNEL,
               .password = CONFIG_DEVICE_AS_PASSWORD,
               .max_connection = CONFIG_DEVICE_AP_MAXCONNECTION,
               .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    if (strlen(CONFIG_DEVICE_AS_PASSWORD) == 0) {
        wifi_config_ap.ap.authmode = WIFI_AUTH_OPEN;
    }
    /* wifi config sta*/

    spiffs_read_wifiinfo(&wifi_config_sta);
    wifi_config_sta.sta.pmf_cfg.capable = true;
    wifi_config_sta.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    m_is_wifi_ap_on = 1;
    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", CONFIG_DEVICE_AS_SSID, CONFIG_DEVICE_AS_PASSWORD, CONFIG_DEVICE_AS_WIFI_CHANNEL);
}

static void wifi_configweb_init(wifi_config_web m_wifi_web_cfg) {
    wifi_auth_mode_t wifi_auth_mode;
    ESP_ERROR_CHECK(esp_wifi_stop());
    // http_server_stop_webserver();
    ESP_LOGI(TAG, "wifi_configweb_init");

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // http_server_stop_webserver();

    ESP_LOGI(TAG, "esp_event_handler_instance_register");
    // if ((char *)m_wifi_web_cfg.auth != NULL) {
    wifi_auth_mode = atoi(m_wifi_web_cfg.auth);

    ESP_LOGI(TAG, "%d", wifi_auth_mode);
    wifi_config_sta.sta.threshold.authmode = wifi_auth_mode;

    ESP_LOGI(TAG, "COPY SSID AND PASSWORD");
    memcpy(wifi_config_sta.sta.ssid, m_wifi_web_cfg.ssid, sizeof(wifi_config_sta.sta.ssid));
    memcpy(wifi_config_sta.sta.password, m_wifi_web_cfg.password, sizeof(wifi_config_sta.sta.password));
    /* Config for Acesspoint*/

    wifi_config_t wifi_config_ap = {
        .ap = {.ssid = CONFIG_DEVICE_AS_SSID,
               .ssid_len = strlen(CONFIG_DEVICE_AS_SSID),
               .channel = CONFIG_DEVICE_AS_WIFI_CHANNEL,
               .password = CONFIG_DEVICE_AS_PASSWORD,
               .max_connection = CONFIG_DEVICE_AP_MAXCONNECTION,
               .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    if (strlen(CONFIG_DEVICE_AS_PASSWORD) == 0) {
        wifi_config_ap.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap));
    m_is_wifi_ap_on = 1;
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_staap finished.");
    sprintf(wifi_str, "{\"wifi_state\": \"%s\"}", "Disconnected");
}

static void wificonfig_handledata(char *data, uint16_t u16datalen) {
    cJSON *root = cJSON_Parse((const char *)data);

    if (root) {
        cJSON *SSID = cJSON_GetObjectItem(root, "SSID");
        cJSON *PASS = cJSON_GetObjectItem(root, "PASS");
        cJSON *AUTH = cJSON_GetObjectItem(root, "AUTH");
        if (SSID && cJSON_IsString(SSID)) {
            strcpy((char *)m_wifi_config.ssid, SSID->valuestring);
        }
        if (SSID && cJSON_IsString(PASS)) {
            strcpy((char *)m_wifi_config.password, PASS->valuestring);
        }
        if (SSID && cJSON_IsString(AUTH)) {
            strcpy((char *)m_wifi_config.auth, AUTH->valuestring);
        }
        ESP_LOGI(TAG, "SSID:%s", m_wifi_config.ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", m_wifi_config.password);
        wifi_configweb_init(m_wifi_config);
        m_is_wifi_store_config = 1; // wifi info send by user

        /*assert failed: tlsf_free heap_tlsf.c:872 (!block_is_free(block) && "block already marked as free")*/
        /*cJSON_Delete(root);
        cJSON_Delete(SSID);
        cJSON_Delete(PASS);*/
    }
    m_is_wifi_connected = 0;
}

static void wifi_softap_turnoff_task(void *arg) {
    ESP_LOGI(TAG, "Enter task turnoff softap");
    while (1) {
        vTaskDelay(10000 / portTICK_RATE_MS);
        if (m_is_wifi_ap_on && m_is_wifi_connected) {
            http_server_stop_webserver();
            m_is_wifi_connected = 0;
            m_is_wifi_ap_on = 0;
            esp_wifi_set_mode(WIFI_MODE_STA);
            ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
            start_webserver();
            ESP_LOGI(TAG, "Turn off AcessPoint finished");
            vTaskDelete(NULL);
            m_is_task_turnoff_wifiap_created = 0;
        };
    }
}

void wifi_login_init() {

    wifi_str = http_server_return_wifi_state_str();

    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP\r\n");
    wifi_init_softap();
    start_webserver();
    http_add_callback_function(wificonfig_handledata, NULL);

    // esp_vfs_spiffs_unregister(conf.partition_label);
}

// TODO: change input to wifi_config_t *pointer  to pass pointer point to wifi_config_t
static void spiffs_write_wifiinfo() {
    /******************** SSID Store **************************/
    ESP_LOGI(TAG, "Opening file SSID");
    FILE *file_wifi = fopen("/wifi/ssid.txt", "w");
    if (file_wifi == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(file_wifi, "%s", (char *)wifi_config_sta.sta.ssid);
    fclose(file_wifi);
    ESP_LOGI(TAG, "SSID File written");
    /***************** PASSWORD Store **************************/
    ESP_LOGI(TAG, "Opening file PASSWORD");
    file_wifi = fopen("/wifi/password.txt", "w");
    if (file_wifi == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(file_wifi, "%s", (char *)wifi_config_sta.sta.password);
    fclose(file_wifi);
    ESP_LOGI(TAG, "PASS File written");
    /******************* AUTH MODE Store ************************/

    ESP_LOGI(TAG, "Opening file AUTH");
    file_wifi = fopen("/wifi/authmode.txt", "w");
    if (file_wifi == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(file_wifi, "%d", wifi_config_sta.sta.threshold.authmode);
    fclose(file_wifi);
    ESP_LOGI(TAG, "PASS File written");
}

static void spiffs_read_wifiinfo(wifi_config_t *wifi_config_store) {
    /******************** SSID Read **************************/
    ESP_LOGI(TAG, "Reading file ssid");
    FILE *file_wifi = fopen("/wifi/ssid.txt", "r");
    if (file_wifi == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    fgets((char *)wifi_config_store->sta.ssid, sizeof(wifi_config_store->sta.ssid), file_wifi);
    // strcpy((char*)wifi_config_store->sta.ssid,read_buffer);
    ESP_LOGI(TAG, "Store SSID:%s", wifi_config_store->sta.ssid);
    fclose(file_wifi);
    /***************** PASSWORD Read **************************/
    ESP_LOGI(TAG, "Reading file password");
    file_wifi = fopen("/wifi/password.txt", "r");
    if (file_wifi == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    fgets((char *)wifi_config_store->sta.password, sizeof(wifi_config_store->sta.ssid), file_wifi);
    ESP_LOGI(TAG, "Store SSID:%s", wifi_config_store->sta.password);
    fclose(file_wifi);
    /******************* AUTH MODE Read ************************/
    ESP_LOGI(TAG, "Reading file AUTH");
    file_wifi = fopen("/wifi/authmode.txt", "r");
    if (file_wifi == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char auth[3] = {0};
    fgets(auth, sizeof(wifi_config_store->sta.threshold.authmode), file_wifi);
    wifi_config_store->sta.threshold.authmode = atoi(auth);
    ESP_LOGI(TAG, "Store SSID:%d", wifi_config_store->sta.threshold.authmode);
    fclose(file_wifi);
}

bool wifi_login_connect_status(void) {
    if (m_is_wifi_connected == WIFI_FAIL) {
        return false;
    } else {
        return true;
    }
}
