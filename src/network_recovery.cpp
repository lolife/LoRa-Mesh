#include "network_recovery.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <esp_log.h>
#include <atomic>

extern char TAG[36];

namespace {
constexpr uint32_t READY = 1, RESET_MQTT = 2;
std::atomic<uint32_t> events{0};
std::atomic<uint8_t> disconnectReason{0};
uint32_t retryAt = 0, retryDelay = 15000;
bool wasReady = false;

// Arduino invokes this on its event task. Network I/O belongs to loop().
void networkEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        disconnectReason.store(info.wifi_sta_disconnected.reason);
        events.store(RESET_MQTT);
    } else if (event == ARDUINO_EVENT_WIFI_STA_LOST_IP) {
        events.store(RESET_MQTT);
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
        // Includes DHCP address changes and flaps shorter than a loop iteration.
        events.store(READY | RESET_MQTT);
    }
}
}

void networkRecoveryBegin() {
    retryAt = millis();
    WiFi.onEvent(networkEvent);
}

bool networkAvailable() {
    return events.load() == READY && WiFi.status() == WL_CONNECTED;
}

void networkRecoveryLoop() {
    // Also catch a disconnected status if its event has not arrived yet.
    // Do not overwrite event state here: GOT_IP may arrive concurrently.
    if ((events.fetch_and(~RESET_MQTT) & RESET_MQTT) ||
        (wasReady && WiFi.status() != WL_CONNECTED)) {
#ifdef RECEIVER
        resetMqttConnection();
#endif
        ESP_LOGI(TAG, "Network changed (disconnect reason=%u)",
                 disconnectReason.load());
    }
    const uint32_t now = millis();
    if (networkAvailable()) {
        if (!wasReady) ESP_LOGI(TAG, "WiFi ready at %s", WiFi.localIP().toString().c_str());
        wasReady = true;
        retryAt = now;
        retryDelay = 15000;
        return;
    }
    if (wasReady) {
        ESP_LOGW(TAG, "%s", "WiFi unavailable; local services continue");
        wasReady = false;
        retryAt = now;
        retryDelay = 15000;
    }
    if (now - retryAt < retryDelay) return;
    retryAt = now;
    // Starts association asynchronously; never wait here or turn the radio off.
    const bool started = WiFi.reconnect();
    ESP_LOGI(TAG, "WiFi retry started=%d, status=%d", started, WiFi.status());
    retryDelay = retryDelay < 60000 ? retryDelay * 2 : 60000;
}

// All nodes, including late/offline boots, share the OTA lifecycle.
void networkOtaLoop() {
    static bool otaStarted = false;
    const bool ready = networkAvailable();
    if (ready && !otaStarted) {
        ArduinoOTA.begin();
        otaStarted = true;
        ESP_LOGI(TAG, "OTA ready at %s, channel %d", WiFi.localIP().toString().c_str(), WiFi.channel());
    } else if (!ready && otaStarted) {
        ArduinoOTA.end();
        otaStarted = false;
    }
}
