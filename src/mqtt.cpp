#ifdef RECEIVER
#include "mk_mqtt_lib.h"

#include <M5Unified.h>
#include <esp_log.h>
#include "mk_espmesh_lib.h"
#include "network_recovery.h"

const char* TB_SERVER = "mqtt.thingsboard.cloud";

WiFiClient wifiClient;
PubSubClient mqttClient(TB_SERVER, TB_PORT, wifiClient);

unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 15000; // 15 seconds between retries
int reconnectFailCount = 0;
const int MAX_RECONNECT_FAIL = 5; // After 5 failures, wait longer
static bool mqttAttempted = false;

void resetMqttConnection() {
    // Close TCP first so MQTT disconnect cannot write to a broken transport.
    wifiClient.stop();
    mqttClient.disconnect();
    mqttAttempted = false;
    lastReconnectAttempt = 0;
    reconnectFailCount = 0;
}

void reconnectMqtt() {
    if (!networkAvailable()) {
        return;
    }
    // Slow broker retries after repeated failures.
    unsigned long currentInterval = RECONNECT_INTERVAL;
    if (reconnectFailCount > MAX_RECONNECT_FAIL) {
        currentInterval = RECONNECT_INTERVAL * 4; // Wait 60 seconds after multiple failures
    }

    // Check if enough time has passed since last attempt
    if (mqttAttempted && millis() - lastReconnectAttempt < currentInterval) {
        return;
    }

    lastReconnectAttempt = millis();
    mqttAttempted = true;

    ESP_LOGI( TAG, "%s", "Attempting MQTT connection to ThingsBoard...");

    // Set shorter timeout for connection attempt
    mqttClient.setSocketTimeout(5); // 5 seconds instead of default 15

    // Attempt to connect with Device Token as Username
    // Use clean session to reduce server-side state
    const char* tbToken = getDeviceToken();
    if (!tbToken || tbToken[0] == '\0') {
        ESP_LOGE(TAG, "%s", "No ThingsBoard token configured for this node");
        return;
    }
    if (mqttClient.connect(tbToken, tbToken, NULL, NULL, 0, 0, NULL, true)) {
        ESP_LOGI( TAG, "%s", "connected!" );
        reconnectFailCount = 0; // Reset failure counter

        // --- SUBSCRIPTIONS ---
        // Subscribe to RPC commands
        mqttClient.subscribe(RPC_SUBSCRIBE_TOPIC);
        ESP_LOGI( TAG, "Subscribed to RPC topic: %s", RPC_SUBSCRIBE_TOPIC );

        // Optional: Subscribe to shared attribute updates
        mqttClient.subscribe(ATTRIBUTES_TOPIC);
    } else {
        ESP_LOGE( TAG, "failed, rc=%d, will retry", mqttClient.state());
        if (reconnectFailCount < MAX_RECONNECT_FAIL + 1) ++reconnectFailCount;
        // WiFi recovery is serviced independently, including while MQTT is down.
    }
}

void mqttLoop() {
    if (!networkAvailable()) return;
    if (!mqttClient.connected()) {
        reconnectMqtt();
    } else {
        mqttClient.loop(); // Process incoming messages
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    ESP_LOGI( TAG, "Message received on topic: %s", topic );
    
    // Convert payload bytes to a null-terminated String for JSON parsing
    char payload_str[length + 1];
    memcpy(payload_str, payload, length);
    payload_str[length] = '\0';
    
    // Check if the message is an RPC command topic
    if (strstr(topic, "v1/devices/me/rpc/request/")) {
        // --- 1. Extract the Request ID ---
        String topicStr(topic);
        int lastSlash = topicStr.lastIndexOf('/');
        String requestID = topicStr.substring(lastSlash + 1);

        // --- 2. Parse the Command JSON ---
        JsonDocument doc; 
        
        // Deserialize the payload bytes directly into the document
        DeserializationError error = deserializeJson(doc, payload_str, length);
        
        if (error) {
            ESP_LOGE( TAG, "JSON parse error: %s", error.c_str() );
            return;
        }
        
        const char* method = doc["method"];
        JsonVariant params = doc["params"];

        // --- 3. Execute the Command and Prepare Response ---
        String responseValue = "{}"; // Default empty JSON response

        // Handle the specific 'setValue' command
        if (strcmp(method, "setValue") == 0) {
            responseValue = "{\"error\":\"Invalid parameter type.\"}";
        }
        // Handle 'getStatus' command
        else if (strcmp(method, "getStatus") == 0) {
            responseValue = "{\"status\":\"online\"}";
        }
        // Handle 'setBrightness' command
        else if (strcmp(method, "setBrightness") == 0) {
            if (params.containsKey("value")) {
                int brightness = params["value"];
                M5.Display.setBrightness(brightness);
                responseValue = "{\"result\":\"ok\",\"brightness\":" + String(brightness) + "}";
            }
        }
        // Add more commands as needed
        
        // --- 4. Publish the Response ---
        String responseTopic = RPC_RESPONSE_TOPIC + requestID;
        bool success = mqttClient.publish(responseTopic.c_str(), responseValue.c_str());
        
        if (!success) {
            ESP_LOGE( TAG, "%s", "Failed to publish RPC response" );
        }
    } 
}

// Graceful disconnect function
void mqttDisconnect() {
    if (mqttClient.connected()) {
        mqttClient.disconnect();
        ESP_LOGI( TAG, "%s", "MQTT disconnected gracefully" );
    }
}

// Check connection status without reconnecting
bool mqttIsConnected() {
    return mqttClient.connected();
}
#endif // RECEIVER
