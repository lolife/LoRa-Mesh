#include <cassert>
#include "build_info.h"
const char firmwareVersion[] = "FW host-test-build";
#include "../../src/mqtt.cpp"
int main() {
    allowNetwork=false;
    mqttLoop();assert(mqttClient.connections==0);
    allowNetwork=true;
    mqttLoop();assert(mqttClient.connections==1 && mqttClient.subscriptions==2);
    assert(mqttClient.publishes==1 && mqttClient.topic==ATTRIBUTES_TOPIC);
    JsonDocument attributes;
    assert(!deserializeJson(attributes,mqttClient.payload));
    assert(attributes["firmware_version"].as<std::string>() == firmwareVersion);
    assert(attributes["ip_address"]=="192.0.2.1");
    assert(attributes["mac_address"]=="gateway");
    assert(attributes["board"]=="board_M5StackCoreS3");
    assert(attributes["mesh_identity"]=="Woods");
    mqttLoop();assert(mqttClient.loops==1);
    assert(mqttClient.publishes==1);
    allowNetwork=false; mqttLoop();assert(mqttClient.loops==1);
    resetMqttConnection();assert(!mqttClient.connected() && wifiClient.stops==1);
    mqttLoop();assert(mqttClient.connections==1);
    allowNetwork=true;mqttClient.publishOk=false;
    mqttLoop();assert(mqttClient.connections==2 && mqttClient.publishes==2);
    mqttLoop();assert(mqttClient.publishes==2);
    nowMs+=15000;mqttLoop();assert(mqttClient.publishes==3);
    mqttClient.publishOk=true;
    nowMs+=15000;mqttLoop();assert(mqttClient.publishes==4);
    nowMs+=15000;mqttLoop();assert(mqttClient.publishes==4);
    resetMqttConnection();mqttClient.connectOk=false;
    mqttLoop();int attempts=mqttClient.connections;
    mqttLoop();assert(mqttClient.connections==attempts);
    nowMs+=15000;mqttLoop();assert(mqttClient.connections==attempts+1);
    reconnectFailCount=MAX_RECONNECT_FAIL+1;
    nowMs+=15000;mqttLoop();assert(mqttClient.connections==attempts+1);
    nowMs+=45000;mqttLoop();assert(mqttClient.connections==attempts+2);
    assert(reconnectFailCount==MAX_RECONNECT_FAIL+1);
    ++attempts;
    // WiFi recovery clears even a long broker backoff and re-subscribes promptly.
    resetMqttConnection();mqttClient.connectOk=true;
    mqttLoop();assert(mqttClient.connections==attempts+2 && reconnectFailCount==0);
    assert(mqttClient.subscriptions==6);
    assert(mqttClient.publishes==5);
    // A brightness RPC must not switch off the photo-only receiver or report
    // a brightness that differs from the value actually applied.
    const auto brightnessRpc = [](int requested, int expected) {
        char topic[] = "v1/devices/me/rpc/request/7";
        std::string payload = "{\"method\":\"setBrightness\",\"params\":{\"value\":" + std::to_string(requested) + "}}";
        mqttCallback(topic, reinterpret_cast<byte*>(&payload[0]), payload.size());
        assert(M5.Display.brightness == expected);
        JsonDocument response;
        assert(!deserializeJson(response, mqttClient.payload));
        assert(response["brightness"].as<int>() == expected);
    };
    brightnessRpc(0, PHOTO_DISPLAY_BRIGHTNESS);
    brightnessRpc(-1, PHOTO_DISPLAY_BRIGHTNESS);
    brightnessRpc(PHOTO_DISPLAY_BRIGHTNESS - 1, PHOTO_DISPLAY_BRIGHTNESS);
    brightnessRpc(200, 200);
    brightnessRpc(300, 255);
    puts("PASS MQTT recovery: offline pause, immediate resume, subscriptions, transport reset, broker retry/backoff");
    puts("PASS photo receiver brightness: RPC cannot turn off/dim below minimum; applied brightness is reported");
}
