#ifndef VCS_WEB_H
#define VCS_WEB_H

#include <Arduino.h>
#include "vcs_constants.h"

// C++ Logging Bridge
void vcs_log(String msg);
String getSystemLogs();

void initWebServer();
void WebServerTask(void *pvParameters);

#endif // VCS_WEB_H