#pragma once
#include "mk_mqtt_lib.h"
inline const char* WIFI_SSID="test";
inline const char* WIFI_PASSWORD="test-only";
inline struct OTA { int starts=0,stops=0;void begin(){starts++;}void end(){stops++;} } ArduinoOTA;

