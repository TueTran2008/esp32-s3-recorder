#ifndef _WIFI_LOGIN_H
#define _WIFI_LOGIN_H

#include <stdbool.h>

typedef struct
{
    char ssid[32];
    char password[64];
    char auth[3];
}wifi_config_web;
/* Initialize Wifi Config Login*/
void wifi_login_init();

bool wifi_login_connect_status(void);
#endif
