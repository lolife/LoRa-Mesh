#include <M5Unified.h>
#include <M5GFX.h>
#include <SPI.h>
#include <ArduinoOTA.h>
#include "esp_log.h"
#include "M5_SX127X.h"
#include "lora_config.h"
#include "display.h"
#include "credentials.h"

// #pragma pack(push, 1) // Force alignment to 1 byte (no padding)
// struct gpsData {
//     float latitude;
//     float longitude;
//     float altitude;
//     float speed;
// };
// #pragma pack(pop) // Restore default alignment

#define TAG "LoRaMesh"

gpsData location = { 0.0, 0.0, 0.0, 0.0 };
char packetBuffer[128];


// Global variables
int counter = 0;
uint16_t screenColor = TFT_NAVY;

#ifdef SENDER
    #include <TinyGPSPlus.h>
    TinyGPSPlus gps;
    static const uint32_t GPSBaud = 115200;
    static const int RXPin = 16, TXPin = 17;
#else
#include "mk_mqtt_lib.h"
#endif

// Function prototypes
void setupLoRa();
void handleSender();
void handleReceiver();
static void smartDelay(unsigned long ms);
void postToThingsBoard(gpsData newData);
bool initializeWiFi();

void setup() {
    Serial.begin( 115200 );
    // Initialize M5Stack with proper configuration
    auto cfg = M5.config();
    cfg.clear_display = true;
    cfg.internal_imu = false;  // Disable IMU to avoid ADC conflict
    cfg.internal_rtc = false;  // Disable RTC to avoid ADC conflict
    cfg.internal_spk = false;  // Disable speaker
    cfg.internal_mic = false;  // Disable microphone
    M5.begin(cfg);
    
    M5.Display.setBrightness(75);
    M5.Display.setRotation(0);
    M5.Display.setFont(&Orbitron_Light_32);

    // Initialize LoRa
    setupLoRa();
    
    // Display initial mode
#ifdef SENDER
    Serial1.begin( GPSBaud, SERIAL_8N1, RXPin, TXPin );
    displayMessage("LoRa Sender", true, screenColor);
#else
    initializeWiFi();
    mqttClient.setCallback(mqttCallback);
    displayMessage("Going Dark", true, screenColor);
    M5.Display.setBrightness(0);
#endif
} 

void loop() {
    M5.update();
    
#ifdef SENDER
    handleSender();
#else
    handleReceiver();
#endif
    
    smartDelay(LOOP_DELAY);
}

void setupLoRa() {
    // Initialize SPI for LoRa module
    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI, -1);
    LoRa.setSPI(&SPI);
    LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);
    
    // Start LoRa
    if (!LoRa.begin(LORA_FREQ)) {
        displayMessage("LoRa init fail.", true, screenColor);
        smartDelay(1000);
    }
    
    // Configure LoRa parameters
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSpreadingFactor(LORA_SF);
}

void handleSender() {
    static long lastPacketTime = millis();
    static long lastDisplayUpdate = millis();
#ifdef SENDER
    if (gps.location.isValid()) {
        location.latitude            = gps.location.lat();
        location.longitude           = gps.location.lng();
        location.altitude            = gps.altitude.meters();
        location.speed               = gps.speed.mph();
        snprintf( packetBuffer, sizeof(packetBuffer), "%.2f %.2f", location.latitude, location.longitude );

        // Send packet at regular intervals
        if (millis() - lastPacketTime > PACKET_INTERVAL) {
            lastPacketTime = millis();

            LoRa.beginPacket();
            LoRa.print( (char *)&location );
            LoRa.endPacket();
            Serial.printf( "Sent packet: %s\n", packetBuffer );
            counter++;
            
            // Update display immediately after sending
            updateDisplay( location, counter, true);
            lastDisplayUpdate = millis();
        }
    }
#endif

    // Periodic display update
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE) {
        updateDisplay(location, counter, true);
        lastDisplayUpdate = millis();
    }
}

#ifdef RECEIVER
void handleReceiver() {
    static long lastPacketTime = millis();
    static long lastDisplayUpdate = millis();

    ArduinoOTA.handle();

    // Check for incoming packets
    int packetSize = LoRa.parsePacket();
    gpsData newLocation  = { 0.0, 0.0, 0.0, 0.0 };
    if (packetSize) {
        char msg[MAX_MSG_SIZE];
        ESP_LOGI( TAG, "Got packet size %d", packetSize );

        lastPacketTime = millis();

        // Limit reading to buffer size
        int bytesToRead = packetSize;
        if (bytesToRead > MAX_MSG_SIZE - 1) {
            bytesToRead = MAX_MSG_SIZE - 1;
        }
        
        // Read packet data
        int i = 0;
        while (LoRa.available() && i < bytesToRead) {
            msg[i++] = (char)LoRa.read();
        }
        // Drain any remaining bytes
        while (LoRa.available()) {
            LoRa.read();
        }
        //msg[i] = '\0';
    
        memcpy( &newLocation, msg, i-1 );
        ESP_LOGI( TAG, "Got msg: %.8f", newLocation.latitude );
        postToThingsBoard( newLocation);
        // Update display with new data
        updateDisplay(newLocation, counter, false);
        lastDisplayUpdate = millis();
    }
    
    // Periodic display update and timeout check
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE) {
        // Check for communication timeout
        // if (millis() - lastPacketTime > NO_CONTACT_TIMEOUT)
        //     snprintf( msg, sizeof(msg), "%s", "No contact" );
        updateDisplay( newLocation, counter, false);
        lastDisplayUpdate = millis();
    }

    // MQTT loop with reduced frequency
    static unsigned long lastMqttLoop = 0;
    if (millis() - lastMqttLoop > 1000) { // Only check MQTT every second
        mqttLoop();
        lastMqttLoop = millis();
    }
}
void postToThingsBoard(gpsData newData) {
    unsigned char payload[TELEMETRY_DOC_SIZE];
    JsonDocument doc;

const double EPSILON = 1e-9;  // or whatever tolerance makes sense for your use case

    if( std::abs(newData.latitude) < EPSILON || 
        newData.latitude > 90.0 || 
        newData.latitude < -90.0 || 
        std::abs(newData.longitude) < EPSILON || 
        newData.longitude > 180.0 || 
        newData.longitude < -180.0 )
        return;

    doc["latitude"] = newData.latitude;
    doc["longitude"] = newData.longitude;
    doc["altitude"] = newData.altitude;
    doc["speed"] = newData.speed;
    doc["ip_address"] = WiFi.localIP().toString().c_str();

    // Serialize the JSON object
    size_t n = serializeJson(doc, payload);

    // Publish the payload
    bool success = mqttClient.publish(TELEMETRY_TOPIC, (const char*)payload, n);
}

bool initializeWiFi() {
    Serial.println("Connecting");
    
    // Configure WiFi for lower power
    WiFi.mode(WIFI_STA);
    
    // Reduce TX power to save energy (adjust based on signal strength needs)
    WiFi.setTxPower(WIFI_POWER_11dBm); // Lower from default 20dBm
    
    // Enable power saving mode
    WiFi.setSleep(WIFI_PS_MIN_MODEM); // Light sleep when idle
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setAutoReconnect(true);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        attempts++;
    }   

    bool connected = (WiFi.status() == WL_CONNECTED);

    if (connected) {
        String ip = WiFi.localIP().toString();
        Serial.println(ip.c_str());
        Serial.println(WiFi.macAddress().c_str());
    } else {
        Serial.println("Network failed!");
        return connected;
    }

    ArduinoOTA.begin();
    return connected;
}
#endif

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {           
  unsigned long start = millis();
  do {         
#ifdef SENDER
    while (Serial1.available()) {
      gps.encode(Serial1.read());
      //Serial.print(".");
    }
#endif
      delay(100);
  } while (millis() - start < ms);
} 