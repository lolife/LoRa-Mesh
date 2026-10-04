#pragma once

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <cstdint>

#define RPC_DOC_SIZE 128

#define TB_PORT   1883
#define TELEMETRY_DOC_SIZE 512

#define TELEMETRY_TOPIC  "v1/devices/me/telemetry"
#define ATTRIBUTES_TOPIC  "v1/devices/me/attributes"
#define RPC_SUBSCRIBE_TOPIC  "v1/devices/me/rpc/request/+"
#define RPC_RESPONSE_TOPIC  "v1/devices/me/rpc/response/"

extern const char* TB_SERVER;
extern WiFiClient wifiClient;
extern PubSubClient mqttClient;

extern unsigned long lastReconnectAttempt;
extern const unsigned long RECONNECT_INTERVAL;
extern int reconnectFailCount;
extern const int MAX_RECONNECT_FAIL;

void reconnectMqtt();
void mqttLoop();
void mqttCallback(char* topic, byte* payload, unsigned int length);

void mqttDisconnect();
bool mqttIsConnected();
