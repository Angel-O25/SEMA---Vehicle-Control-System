#ifndef VCS_WEB_H
#define VCS_WEB_H

#include <Arduino.h>
#include "vcs_constants.h"

// [ADDED] C++ Logging Bridge
void vcs_log(String msg);
String getSystemLogs();

#if defined(ESP32_VCS)
    void initWebServer();
    void WebServerTask(void *pvParameters);
#endif

#endif