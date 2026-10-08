#pragma once
// Host-only radio/MQTT doubles. Production files are compiled directly by tests.
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <cmath>
#include <utility>
#include <ArduinoJson.h>
#include "network_recovery.h"
using WiFiEvent_t = int;
struct WiFiEventInfo_t { struct { uint8_t reason=0; } wifi_sta_disconnected; };
enum { ARDUINO_EVENT_WIFI_STA_DISCONNECTED, ARDUINO_EVENT_WIFI_STA_LOST_IP, ARDUINO_EVENT_WIFI_STA_GOT_IP };
inline unsigned long nowMs = 100;
inline unsigned long millis() { return nowMs; }
#define WL_CONNECTED 3
inline struct Wifi {
 int state=3, radioChannel=6, attempts=0, disconnects=0;
 bool autoReconnect=true;
 std::function<void(WiFiEvent_t,WiFiEventInfo_t)> eventCallback;
 void onEvent(decltype(eventCallback) callback){eventCallback=callback;}
 bool reconnect(){attempts++;return true;}
 int status(){return state;} int channel(){return radioChannel;}
 std::string macAddress(){return "gateway";}
 struct IP { std::string toString(){return "192.0.2.1";} };
 IP localIP(){return {};}
 void setAutoReconnect(bool value){autoReconnect=value;}
 void disconnect(bool,bool){state=0;disconnects++;}
 void begin(const char*,const char*){state=0;attempts++;}
} WiFi;
#if !defined(TEST_NETWORK_RECOVERY) && !defined(TEST_PRIMARY_MQTT)
inline bool networkAvailable(){return WiFi.status()==WL_CONNECTED;}
#endif
#ifdef TEST_PRIMARY_MQTT
extern const char* TB_SERVER;
#else
inline const char* TB_SERVER="test.invalid";
#endif
#define TB_PORT 1883
#define TELEMETRY_DOC_SIZE 512
#define TELEMETRY_TOPIC "v1/devices/me/telemetry"
#define ATTRIBUTES_TOPIC "v1/devices/me/attributes"
#define RPC_SUBSCRIBE_TOPIC "v1/devices/me/rpc/request/+"
#define RPC_RESPONSE_TOPIC "v1/devices/me/rpc/response/"
inline char TAG[36]="test";
template<typename... Args> void mockLog(Args&&...) {}
#define ESP_LOGI(...) mockLog(__VA_ARGS__)
#define ESP_LOGW(...) mockLog(__VA_ARGS__)
#define ESP_LOGD(...) mockLog(__VA_ARGS__)
#define ESP_LOGE(...) mockLog(__VA_ARGS__)
class WiFiClient { public: int stops=0; void stop(){stops++;} };
class PubSubClient {
public:
 bool online=false, connectOk=true, publishOk=true;
 int connections=0, publishes=0, loops=0, subscriptions=0; std::string username, clientId, payload, topic;
 std::function<void()> onPublish;
 PubSubClient(const char*,int,WiFiClient&){}
 bool connected(){return online;} void loop(){loops++;} void setSocketTimeout(int){} bool setBufferSize(int){return true;}
 bool subscribe(const char*){subscriptions++;return true;}
 bool connect(const char* id,const char* user,const char*){connections++;clientId=id;username=user;return online=connectOk;}
 bool connect(const char* id,const char* user,const char* pass,const char*,int,int,const char*,bool){return connect(id,user,pass);}
 void disconnect(){online=false;}
 int state(){return online ? 0 : -1;}
 bool publish(const char* publishTopic,const char* data,size_t len){topic=publishTopic;publishes++;payload.assign(data,len);if(onPublish)onPublish();return publishOk;}
 bool publish(const char* topic,const char* data){return publish(topic,data,strlen(data));}
};
#ifdef TEST_PRIMARY_MQTT
extern WiFiClient wifiClient;
extern PubSubClient mqttClient;
inline bool allowNetwork=true;
inline bool networkAvailable(){return allowNetwork && WiFi.status()==WL_CONNECTED;}
using byte=uint8_t;
class String : public std::string {
public:
 using std::string::string;
 String(const std::string& s):std::string(s){}
 String(int n):std::string(std::to_string(n)){}
 int lastIndexOf(char c){return static_cast<int>(rfind(c));}
 String substring(int from){return substr(from);}
 friend String operator+(const String& a,const String& b){return std::string(a)+std::string(b);}
 friend String operator+(const char* a,const String& b){return std::string(a)+std::string(b);}
 friend String operator+(const String& a,const char* b){return std::string(a)+std::string(b);}
};
inline struct M5Mock { struct DisplayMock { void setBrightness(int){} } Display; int getBoard(){return 1;} } M5;
inline struct MeshIdentityMock { const char* name="Woods"; } meshIdentityMock;
inline auto* me=&meshIdentityMock;
inline const char* getDeviceToken(){return "primary-token";}
#else
inline WiFiClient ownWifi;
inline PubSubClient mqttClient(TB_SERVER,TB_PORT,ownWifi);
#endif
