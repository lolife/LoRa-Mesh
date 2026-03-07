#include <M5Unified.h>
#include <M5GFX.h>
#include <SPI.h>
#include <ArduinoOTA.h>
#include "esp_log.h"
#include "M5_SX127X.h"
#include "lora_config.h"
#include "display.h"
#include "credentials.h"
#include "mk_mqtt_lib.h"

gpsData location     = { 0.0, 0.0, 0.0, 0.0 };
gpsData newLocation  = { 0.0, 0.0, 0.0, 0.0 };
static char status[4];
static const char* ack = "ACK";

// Global variables
uint16_t screenColor = TFT_DARKGREEN;
static long lastPacketTime = 0;

#ifdef SENDER
#define TAG "➡️LoRaMeshSender"
    #include <TinyGPSPlus.h>
    TinyGPSPlus gps;
    static const uint32_t GPSBaud = 115200;
    static const int RXPin = 18, TXPin = 17;
#else
    #define TAG "⬅️LoRaMeshReceiver"
#endif

// Function prototypes
void setupLoRa();
void handleSender();
void handleReceiver();
static void smartDelay(unsigned long ms);
void postToThingsBoard(gpsData newData);
bool initializeWiFi();
bool nearlyZero( double valueToCheck );
bool locationInBounds( gpsData newLocation );
bool sendPacket( char *payload, int packetSize );
int receivePacket();

void setup() {
    //Serial.begin( 115200 );
    // Initialize M5Stack with proper configuration
    auto cfg = M5.config();
    cfg.clear_display = true;
    cfg.internal_imu = false;  // Disable IMU to avoid ADC conflict
    cfg.internal_rtc = false;  // Disable RTC to avoid ADC conflict
    cfg.internal_spk = false;  // Disable speaker
    cfg.internal_mic = false;  // Disable microphone
    M5.begin(cfg);
    
    M5.Display.setBrightness(66);
    M5.Display.setRotation(1);
    M5.Display.setFont(&Orbitron_Light_32);

    memset( status, '\0', sizeof(status) );
    
    // Initialize LoRa
    setupLoRa();
    
    // Display initial mode
    initializeWiFi();
    mqttClient.setCallback(mqttCallback);
#ifdef SENDER
    screenColor = TFT_NAVY;
    //M5.Display.setRotation(0);
    Serial1.begin( GPSBaud, SERIAL_8N1, RXPin, TXPin );
    ESP_LOGI( TAG, "%s", "LoRa Sender" );
    displayMessage("LoRa Sender", true, screenColor);
#else
    ESP_LOGI( TAG, "%s", "LoRa Receiverr" );
    //displayMessage("Going Dark", true, screenColor);
    //M5.Display.setBrightness(1);
#endif
} 

void loop() {
    M5.update();
    //ArduinoOTA.handle();

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
        ESP_LOGE( TAG, "%s", "LoRa init fail." );
        displayMessage("LoRa init fail.", true, screenColor);
        smartDelay(1000);
    }
    
    // Configure LoRa parameters
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSpreadingFactor(LORA_SF);
}

bool sendPacket( char *payload, int packetSize ) {
    LoRa.beginPacket();
    for( int i=0; i< packetSize; ++i )
        LoRa.write( (char)payload[i] );
    LoRa.endPacket();
    delay(10);

    //ESP_LOGI( TAG, "Sent packet %s", payload );
    if( LoRa.getWriteError() != 0 )
        return false;
    return true;
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

        // Send packet at regular intervals
        if (millis() - lastPacketTime > PACKET_INTERVAL) {
            lastPacketTime = millis();

            if( ! sendPacket( (char *)&location, sizeof(location) ) )
                ESP_LOGE( TAG, "Error sending location" );
            else {
                int packetType = receivePacket();
                if( packetType == 2 )
                    displayMessage( status, true, TFT_GREEN );
                    smartDelay(2000);
            }
            
            // Update display immediately after sending
            updateDisplay( location, true);
            lastDisplayUpdate = millis();
        }
    }
#endif

    // Periodic display update
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE) {
        updateDisplay(location, true);
        lastDisplayUpdate = millis();
    }
}

int receivePacket() {
    char msg[MAX_MSG_SIZE];
    
    // Check for incoming packets
    int packetSize = LoRa.parsePacket();
    if( packetSize < 1 )
        return 0;

    ESP_LOGI( TAG, "Got packet size %d", packetSize );
    lastPacketTime = millis();

    // Limit reading to buffer size
    int bytesToRead = packetSize;
    if (bytesToRead > MAX_MSG_SIZE) {
        bytesToRead = MAX_MSG_SIZE;
    }
    
    // Read packet data
    int i = 0;
    while (LoRa.available() && i < bytesToRead) {
        msg[i] = (char)LoRa.read();
        //ESP_LOGD( TAG, "Got byte: %x", msg[i] );
        i++;
    }
    // Drain any remaining bytes
    while (LoRa.available()) {
        ESP_LOGV( TAG, "%s", "Throwing away data" );
        LoRa.read();
    }

    if (i == sizeof(location) ) {
        memcpy( &newLocation, msg, i );
        ESP_LOGV( TAG, "Got msg: %.8f", newLocation.speed );
        return 1;
    }
    else if (i == 12 ) {
        loraStatus newStatus = { 0, 0.0, 0 };
        memcpy( &newStatus, msg, i );
        ESP_LOGI( TAG, "Got code: %d", newStatus.code );
        return 2;
   }
   return 0;
}

#ifdef RECEIVER
void handleReceiver() {
    static long lastDisplayUpdate = millis();


    int packetType = receivePacket();
    if( packetType == 1 ) { // got new location
        loraStatus newStatus = { 0, LoRa.packetSnr(), M5.Power.getBatteryLevel() };
        sendPacket( (char*)&newStatus, sizeof(newStatus) );
        updateDisplay( newLocation, false);
        postToThingsBoard( newLocation );
    }

    // Periodic display update and timeout check
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE) {
        // Check for communication timeout
        if (millis() - lastPacketTime > NO_CONTACT_TIMEOUT )
            newLocation = { 0.0, 0.0, 0.0, 0.0 };
        updateDisplay( newLocation, false);
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

    if( nearlyZero(newData.latitude) || newData.latitude > 90.0 || newData.latitude < -90.0 ||
            nearlyZero(newData.longitude) || newData.longitude > 180.0 || newData.longitude < -180.0 )
        return;

    ESP_LOGV( TAG, "Considerintg posting %.1f", newData.speed );
    if( newData.speed < 0.2 )
        return;
    ESP_LOGI( TAG, "%s", "Posting" );

    doc["latitude"] = newData.latitude;
    doc["longitude"] = newData.longitude;
    doc["altitude"] = newData.altitude;
    doc["speed"] = newData.speed;
    //doc["ip_address"] = WiFi.localIP().toString().c_str();
    doc["pkt_rssi"] = LoRa.packetSnr();

    // Serialize the JSON object
    size_t n = serializeJson(doc, payload);

    // Publish the payload
    bool success = mqttClient.publish(TELEMETRY_TOPIC, (const char*)payload, n);
}
#endif
bool initializeWiFi() {
    ESP_LOGI( TAG, "Connecting");
    
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
        ESP_LOGI( TAG, "%s", WiFi.localIP().toString().c_str() );
        ESP_LOGI( TAG, "%s", WiFi.macAddress().c_str()) ;
    } else {
        ESP_LOGE( TAG, "Network failed!" );
        return connected;
    }

    ArduinoOTA.begin();
    return connected;
}

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
    ArduinoOTA.handle();
      delay(100);
  } while (millis() - start < ms);
} 
