#include "http_server.h"
#include "esp_netif.h"
#include "esp_tls_crypto.h"
#include "wifi_login_cfg.h"
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <string.h>

#define HTTPD_401 "401 UNAUTHORIZED" /*!< HTTP Response 401 */
char post_handle_buffer[128];
typedef struct {
    char *username;
    char *password;
} basic_auth_info_t;

static httpd_handle_t server = NULL;
basic_auth_info_t wifi_login;
static char wifi_state_str[100] = "";
http_post_callback_function post_callback = NULL;
http_get_callback_function get_callback = NULL;

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
static const char *TAG = "HTTP ";

#if (CONFIG_EXAMPLE_BASIC_AUTH)
static char *http_auth_basic(const char *username, const char *password) {
    int out;
    char *user_info = NULL;
    char *digest = NULL;
    size_t n = 0;
    asprintf(&user_info, "%s:%s", username, password);
    if (!user_info) {
        ESP_LOGE(TAG, "No enough memory for user information");
        return NULL;
    }
    esp_crypto_base64_encode(NULL, 0, &n, (const unsigned char *)user_info, strlen(user_info));

    /* 6: The length of the "Basic " string
     * n: Number of bytes for a base64 encode format
     * 1: Number of bytes for a reserved which be used to fill zero
     */
    digest = calloc(1, 6 + n + 1);
    if (digest) {
        strcpy(digest, "Basic ");
        esp_crypto_base64_encode((unsigned char *)digest + 6, n, (size_t *)&out, (const unsigned char *)user_info, strlen(user_info));
    }
    free(user_info);
    return digest;
}

/* An HTTP GET handler */
static esp_err_t basic_auth_get_handler(httpd_req_t *req) {
    char *buf = NULL;
    size_t buf_len = 0;
    basic_auth_info_t *basic_auth_info = req->user_ctx;

    ESP_LOGI(TAG, "user:%s", basic_auth_info->username);
    ESP_LOGI(TAG, "password:%s", basic_auth_info->password);
    buf_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
    if (buf_len > 1) {
        buf = calloc(1, buf_len);
        if (!buf) {
            ESP_LOGE(TAG, "No enough memory for basic authorization");
            return ESP_ERR_NO_MEM;
        }

        if (httpd_req_get_hdr_value_str(req, "Authorization", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Authorization: %s", buf);
        } else {
            ESP_LOGE(TAG, "No auth value received");
        }

        char *auth_credentials = http_auth_basic(basic_auth_info->username, basic_auth_info->password);
        ESP_LOGI("HTTP", "%s", auth_credentials);
        if (!auth_credentials) {
            ESP_LOGE(TAG, "No enough memory for basic authorization credentials");
            free(buf);
            return ESP_ERR_NO_MEM;
        }

        if (strncmp(auth_credentials, buf, buf_len)) {
            ESP_LOGE(TAG, "Not authenticated");
            httpd_resp_set_status(req, HTTPD_401);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
            httpd_resp_send(req, NULL, 0);
        } else {
            ESP_LOGI(TAG, "Authenticated!");
            char *basic_auth_resp = NULL;
            httpd_resp_set_status(req, HTTPD_200);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            asprintf(&basic_auth_resp, "{\"authenticated\": true,\"user\": \"%s\"}", basic_auth_info->username);
            if (!basic_auth_resp) {
                ESP_LOGE(TAG, "No enough memory for basic authorization response");
                free(auth_credentials);
                free(buf);
                return ESP_ERR_NO_MEM;
            }
            httpd_resp_set_type(req, "text/html");
            httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
            // httpd_resp_send(req, basic_auth_resp, strlen(basic_auth_resp));
            // free(basic_auth_resp);
        }
        free(auth_credentials);
        free(buf);
    } else {
        ESP_LOGE(TAG, "No auth header received");
        httpd_resp_set_status(req, HTTPD_401);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "keep-alive");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
        httpd_resp_send(req, NULL, 0);
    }
    // free(basic_auth_info);
    return ESP_OK;
}

static httpd_uri_t basic_auth = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = basic_auth_get_handler,
};

static void httpd_register_basic_auth(httpd_handle_t server) {
    basic_auth_info_t *basic_auth_info = calloc(1, sizeof(basic_auth_info_t));
    if (basic_auth_info) {
        basic_auth_info->username = CONFIG_DEVICE_BASIC_AUTH_SSID;
        basic_auth_info->password = CONFIG_DEVICE_BASIC_AUTH_PASSWORD;
        basic_auth.user_ctx = basic_auth_info;
        httpd_register_uri_handler(server, &basic_auth);
    }
    // free(basic_auth_info);
}
#endif

/********************************************************************/
static esp_err_t get_handler(httpd_req_t *req) {

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    if (get_callback != NULL) {
    }
    return ESP_OK;
}

static esp_err_t get_update_connection_info(httpd_req_t *req) {
    const char *temp = (const char *)wifi_state_str;
    ESP_LOGI(TAG, "%s", temp);
    httpd_resp_send(req, temp, strlen(temp));
    return ESP_OK;
}

static esp_err_t post_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Enter Http Post Handle\r\n");
    int ret, remaining = req->content_len;
    ESP_LOGI(TAG, "remaining:%d\r\n", remaining);
    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, post_handle_buffer, (remaining > sizeof(post_handle_buffer)) ? sizeof(post_handle_buffer) : remaining)) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, post_handle_buffer);
        ESP_LOGI(TAG, "====================================");
    }
    if (post_callback != NULL)
        post_callback(post_handle_buffer, req->content_len);
    return ESP_OK;
}

/********************************************************************/
httpd_uri_t uri_get = {.uri = "/", .method = HTTP_GET, .handler = get_handler, .user_ctx = NULL};
httpd_uri_t uri_post = {.uri = "/wificonfig", .method = HTTP_POST, .handler = post_handler, .user_ctx = NULL};
httpd_uri_t wifi_state_get = {.uri = "/wifi_state", .method = HTTP_GET, .handler = get_update_connection_info, .user_ctx = NULL};

/********************************************************************/
httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
#if (CONFIG_EXAMPLE_BASIC_AUTH)
        httpd_register_basic_auth(server);
#endif
        // httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_post);
        httpd_register_uri_handler(server, &wifi_state_get);
        return server;
    }
    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

esp_err_t http_server_stop_webserver() {
    esp_err_t ret = ESP_OK;
    if (server) {
        /* Stop the httpd server */
        ret = httpd_stop(server);
    }
    ESP_LOGI(TAG, "Webserver Stopped");
    return ret;
}

void http_add_callback_function(void *cb_post_function, void *cb_get_function) {
    post_callback = cb_post_function;
    get_callback = cb_get_function;
}

char *http_server_return_wifi_state_str() { return wifi_state_str; }
