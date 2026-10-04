#include <cassert>
#include "../../src/network_recovery.cpp"

int primaryResets=0;
void resetMqttConnection(){++primaryResets;}

int expectedResets(int count) {
#ifdef RECEIVER
    return count;
#else
    (void)count;
    return 0;
#endif
}

void event(int kind, uint8_t reason=0) {
    WiFiEventInfo_t info{};info.wifi_sta_disconnected.reason=reason;
    WiFi.eventCallback(kind,info);
}

int main() {
    WiFi.state=0;
    networkRecoveryBegin();
    networkOtaLoop();assert(ArduinoOTA.starts==0);
    assert(!networkAvailable());
    networkRecoveryLoop();assert(WiFi.attempts==0);
    nowMs+=15000;networkRecoveryLoop();assert(WiFi.attempts==1);
    nowMs+=29999;networkRecoveryLoop();assert(WiFi.attempts==1);
    nowMs+=1;networkRecoveryLoop();assert(WiFi.attempts==2);
    nowMs+=60000;networkRecoveryLoop();assert(WiFi.attempts==3);
    nowMs+=60000;networkRecoveryLoop();assert(WiFi.attempts==4);

    WiFi.state=WL_CONNECTED;event(ARDUINO_EVENT_WIFI_STA_GOT_IP);
    assert(!networkAvailable()); // Must close old sockets before any MQTT I/O.
    networkRecoveryLoop();assert(networkAvailable());
    networkOtaLoop();assert(ArduinoOTA.starts==1);
    networkOtaLoop();assert(ArduinoOTA.starts==1);
    assert(primaryResets==expectedResets(1));

    // A transient authentication failure must recover even if status is stale.
    event(ARDUINO_EVENT_WIFI_STA_DISCONNECTED,202);
    assert(!networkAvailable());networkRecoveryLoop();assert(primaryResets==expectedResets(2));
    networkOtaLoop();assert(ArduinoOTA.stops==1);
    nowMs+=15000;networkRecoveryLoop();assert(WiFi.attempts==5);
    WiFi.state=WL_CONNECTED;event(ARDUINO_EVENT_WIFI_STA_GOT_IP);networkRecoveryLoop();
    assert(networkAvailable());
    networkOtaLoop();assert(ArduinoOTA.starts==2);

    // Disconnect and reconnect both occur between service iterations.
    event(ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    event(ARDUINO_EVENT_WIFI_STA_GOT_IP);
    assert(!networkAvailable());networkRecoveryLoop();assert(primaryResets==expectedResets(4));
    assert(networkAvailable());

    // DHCP IP change and lost IP each invalidate every cloud connection.
    event(ARDUINO_EVENT_WIFI_STA_GOT_IP);networkRecoveryLoop();assert(primaryResets==expectedResets(5));
    event(ARDUINO_EVENT_WIFI_STA_LOST_IP);networkRecoveryLoop();assert(!networkAvailable());
    networkOtaLoop();assert(ArduinoOTA.stops==2);
    assert(primaryResets==expectedResets(6));
    networkRecoveryLoop();assert(primaryResets==expectedResets(6));

    WiFi.state=WL_CONNECTED;event(ARDUINO_EVENT_WIFI_STA_GOT_IP);networkRecoveryLoop();
    int resets=primaryResets;
    WiFi.state=0;networkRecoveryLoop();assert(primaryResets==resets+expectedResets(1));
    assert(!networkAvailable());
    networkRecoveryLoop();assert(primaryResets==resets+expectedResets(1));

    // Unsigned time arithmetic still retries across millis() rollover.
    retryAt=0xfffffff0U;retryDelay=15000;wasReady=false;
    nowMs=14983;networkRecoveryLoop();assert(WiFi.attempts==5);
    nowMs=14984;networkRecoveryLoop();assert(WiFi.attempts==6);
#ifdef SENDER
    assert(primaryResets==0); // Sender never invokes MQTT transport handling.
#endif
    puts("PASS network recovery: bounded retries, authentication failure, brief flap, IP change/loss, transport reset, OTA lifecycle, millis rollover");
}
