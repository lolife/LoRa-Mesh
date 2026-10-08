#include <cassert>
#include "../../src/mqtt.cpp"
int main() {
    allowNetwork=false;
    mqttLoop();assert(mqttClient.connections==0);
    allowNetwork=true;
    mqttLoop();assert(mqttClient.connections==1 && mqttClient.subscriptions==2);
    assert(mqttClient.publishes==1 && mqttClient.topic==ATTRIBUTES_TOPIC);
    JsonDocument attributes;
    assert(!deserializeJson(attributes,mqttClient.payload));
    assert(attributes["firmware_version"].as<std::string>().find("FW ")==0);
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
    puts("PASS MQTT recovery: offline pause, immediate resume, subscriptions, transport reset, broker retry/backoff");
}
