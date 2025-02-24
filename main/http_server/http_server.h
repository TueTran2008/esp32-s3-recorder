
#include <esp_http_server.h>
#include <esp_event.h>

/* http callback function pointer */
typedef  void(*http_post_callback_function)(char* data,uint16_t u16datalen);
typedef  void(*http_get_callback_function)(void);

esp_err_t http_server_stop_webserver();
extern httpd_handle_t start_webserver(void);
char* http_server_return_wifi_state_str();

//void http_server_esp_parse_password(char* data);


/**
    @brief: Add Get and Post http request function for your application
*/
void http_add_callback_function(void *cb_post_function,void *cb_get_function);


#define CONFIG_EXAMPLE_BASIC_AUTH 1
