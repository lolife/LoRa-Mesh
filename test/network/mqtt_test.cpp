#include <cassert>
#include "../../src/mqtt.cpp"
int main() {
    allowNetwork=false;
    mqttLoop();assert(mqttClient.connections==0);
    allowNetwork=true;
    mqttLoop();assert(mqttClient.connections==1 && mqttClient.subscriptions==2);
    mqttLoop();assert(mqttClient.loops==1);
    allowNetwork=false; mqttLoop();assert(mqttClient.loops==1);
    resetMqttConnection();assert(!mqttClient.connected() && wifiClient.stops==1);
    mqttLoop();assert(mqttClient.connections==1);
    allowNetwork=true;mqttLoop();assert(mqttClient.connections==2);
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
    puts("PASS MQTT recovery: offline pause, immediate resume, subscriptions, transport reset, broker retry/backoff");
}
