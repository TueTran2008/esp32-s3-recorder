#ifndef _WIFI_LOGIN_CFG_H
#define _WIFI_LOGIN_CFG_H
#include "esp_wifi_types.h"
#include "wifi_login.h"

/******************* CONFIG DEVICE AP **************************/
#define CONFIG_DEVICE_AS_SSID           "TueESP32"
#define CONFIG_DEVICE_AS_PASSWORD       "12345678"
#define CONFIG_DEVICE_AS_WIFI_CHANNEL   1
#define CONFIG_DEVICE_AP_MAXCONNECTION  5
/******************* CONFIG DEVICE STA **************************/

#define CONFIG_DEVICE_STA_MAX_RETRY     5
/******************* CONFIG DEVICE AUTH *************************/


#define CONFIG_DEVICE_BASIC_AUTH_SSID        "admin"
#define CONFIG_DEVICE_BASIC_AUTH_PASSWORD    "admin"


#endif
