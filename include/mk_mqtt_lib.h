#include <PubSubClient.h>
#include <ArduinoJson.h>


// ThingsBoard Configuration
#define TB_PORT   1883
const char* TB_SERVER = "mqtt.thingsboard.cloud";
#define RPC_DOC_SIZE 128
#define TELEMETRY_DOC_SIZE 256

// --- MQTT Topics ---
#define TELEMETRY_TOPIC  "v1/devices/me/telemetry"
#define ATTRIBUTES_TOPIC  "v1/devices/me/attributes"
#define RPC_SUBSCRIBE_TOPIC  "v1/devices/me/rpc/request/+"
#define RPC_RESPONSE_TOPIC  "v1/devices/me/rpc/response/"

WiFiClient wifiClient;
PubSubClient mqttClient(TB_SERVER, TB_PORT, wifiClient);

// Connection retry management
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 15000; // 15 seconds between retries
int reconnectFailCount = 0;
const int MAX_RECONNECT_FAIL = 5; // After 5 failures, wait longer

extern char TAG[36];

void reconnectMqtt() {
    // Implement exponential backoff
    unsigned long currentInterval = RECONNECT_INTERVAL;
    if (reconnectFailCount > MAX_RECONNECT_FAIL) {
        currentInterval = RECONNECT_INTERVAL * 4; // Wait 60 seconds after multiple failures
    }
    
    // Check if enough time has passed since last attempt
    if (millis() - lastReconnectAttempt < currentInterval) {
        return;
    }
    
    lastReconnectAttempt = millis();
    
    ESP_LOGI( TAG, "%s", "Attempting MQTT connection to ThingsBoard...");
    
    // Set shorter timeout for connection attempt
    mqttClient.setSocketTimeout(5); // 5 seconds instead of default 15
    
    // Attempt to connect with Device Token as Username
    // Use clean session to reduce server-side state
    if (mqttClient.connect(TB_DEVICE_TOKEN, TB_DEVICE_TOKEN, NULL, NULL, 0, 0, NULL, true)) {
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
        reconnectFailCount++;
        
        // If too many failures, consider WiFi issue
        if (reconnectFailCount > MAX_RECONNECT_FAIL * 2) {
            ESP_LOGE( TAG, "%s", "Too many MQTT failures, checking WiFi...");
            if (WiFi.status() != WL_CONNECTED) {
                WiFi.reconnect();
            }
            reconnectFailCount = 0; // Reset counter
        }
    }
}

void mqttLoop() {
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
