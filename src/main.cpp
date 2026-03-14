#include "main.h"

extern char    loraMessage[MAX_MSG_SIZE];

void setup() {
    //Serial.begin( 115200 );
    // Initialize M5Stack with proper configuration
    ESP_LOGI(TAG, "FW %s %s", __DATE__, __TIME__);
    auto cfg = M5.config();
    cfg.clear_display = true;
    cfg.internal_imu = false;  // Disable IMU to avoid ADC conflict
    cfg.internal_rtc = false;  // Disable RTC to avoid ADC conflict
    cfg.internal_spk = false;  // Disable speaker
    cfg.internal_mic = false;  // Disable microphone
    M5.begin(cfg);
    
    M5.Display.setBrightness(80);
    M5.Display.setRotation(1);
    M5.Display.setFont(&Orbitron_Light_32);
    
    // Initialize LoRa
    if( ! setupLoRa() )
        displayMessage("LoRa init failed", true, screenColor );
    
    // Display initial mode
    initializeWiFi();
    mqttClient.setCallback(mqttCallback);
    if (!initESPNow())
        displayMessage("ESP-NOW init failed", true, screenColor );

#ifdef SENDER
    snprintf( TAG, sizeof(TAG), "➡️LoRaMeshSender" ); 
    screenColor = TFT_NAVY;
    //M5.Display.setRotation(0);
    GPS_SERIAL_PORT.begin( GPSBaud, SERIAL_8N1, RXPin, TXPin );
    ESP_LOGI(TAG, "GPS UART pins rx=%d tx=%d baud=%" PRIu32, RXPin, TXPin, GPSBaud);
    if (RXPin == LORA_SCLK || RXPin == LORA_MISO || RXPin == LORA_MOSI ||
        TXPin == LORA_SCLK || TXPin == LORA_MISO || TXPin == LORA_MOSI) {
        ESP_LOGE(TAG, "GPS UART pin conflicts with LoRa SPI pins (SCLK=%d MISO=%d MOSI=%d)",
                 LORA_SCLK, LORA_MISO, LORA_MOSI);
        displayMessage("GPS/LoRa pin conflict", true, TFT_RED);
    }
    //sdLoggingReady = initSdLogging();
    
    ESP_LOGI( TAG, "%s", "LoRa Sender" );
    displayMessage("LoRa Sender", true, screenColor);
#else
    snprintf( TAG, sizeof(TAG), "⬅️LoRaMeshReceive" ); 
    ESP_LOGI( TAG, "%s", "LoRa Receiverr" );
    //displayMessage("Going Dark", true, screenColor);
    //M5.Display.setBrightness(25);
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

void handleSender() {
    static long lastTxPacketTime = millis();
    static long lastDisplayUpdate = millis();
    static unsigned long lastGpsDiag = 0;
#ifdef SENDER
//    if (gps.location.isValid() && gps.location.isUpdated()) {
    if ( gps.location.isValid() ) {
        gpsData rawLocation = {
            (float)gps.location.lat(),
            (float)gps.location.lng(),
            (float)gps.altitude.meters(),
            (float)gps.speed.mph(),
            (int)gps.satellites.value()
        };
        gpsData filteredLocation = rawLocation;
        ESP_LOGV( TAG, "speed is %.2f", rawLocation.speed );

        if (acceptGpsMeasurement(rawLocation, &filteredLocation)) {
            location = filteredLocation;
            locationPkt.payload = location;
        } else {
            ESP_LOGW(TAG, "Rejected implausible GPS point: %.6f, %.6f",
                     rawLocation.latitude, rawLocation.longitude);
        }
    }

    if (millis() - lastGpsDiag > 30000) {
        lastGpsDiag = millis();
        ESP_LOGI(TAG, "GPS diag: chars=%" PRIu32 " sats=%u valid=%d updated=%d",
                 gps.charsProcessed(),
                 gps.satellites.isValid() ? gps.satellites.value() : 0,
                 gps.location.isValid(),
                 gps.location.isUpdated());
        if (gps.charsProcessed() < 10) {
            ESP_LOGW(TAG, "No NMEA stream detected on GPS UART; check wiring/pins/baud");
        }
    }
#endif
    // Send packet at regular intervals
    if (millis() - lastTxPacketTime > PACKET_INTERVAL) {
        lastTxPacketTime = millis();

        if( !sendDataWithAckRetries(ACK_RETRY_COUNT) )
            ESP_LOGE( TAG, "Error sending location" );
        
        // Update display immediately after sending
        updateDisplay( locationPkt, true);
        lastDisplayUpdate = millis();
    }

    // Periodic display update
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE) {
        updateDisplay(locationPkt, true);
        lastDisplayUpdate = millis();
    }
}

int handlePacket() {
    int packetSize = receivePacket();
    if( packetSize < 1 )
        return 0;

    const uint8_t packetType = static_cast<uint8_t>(loraMessage[0]);

    if (packetType == LORA_PKT_LOCATION) {
        if (!decodeLoraGpsPacket(loraMessage, packetSize, &locationPkt)) {
            ESP_LOGW(TAG, "Unexpected size %d for GPS packet", packetSize);
            return 0;
        }
        lastPacketTime = millis();
        lastRxDataSeq = locationPkt.seq;
        newLocation = locationPkt.payload;
        ESP_LOGI( TAG, "Got msg: %.8f", newLocation.speed );
        return 1;
    }
    else if (packetType == LORA_PKT_ENV) {
        loraEnvPacket envPkt;
        if (!decodeLoraEnvPacket(loraMessage, packetSize, &envPkt)) {
            ESP_LOGW(TAG, "Unexpected size %d for ENV packet", packetSize);
            return 0;
        }
        lastPacketTime = millis();
        lastRxDataSeq = envPkt.seq;
        latestEnv = envPkt.payload;
        ESP_LOGI(TAG, "Received ENV payload: temp=%.2fC humidity=%.2f%%",
                 latestEnv.temperature, latestEnv.humidity);
        return 3;
    }
    else if (packetType == LORA_PKT_ACK) {
        if (!decodeLoraStatusPacket(loraMessage, packetSize, &newStatus)) {
            ESP_LOGW(TAG, "Unexpected size %d for ACK packet", packetSize);
            return 0;
        }
        ESP_LOGV( TAG, "SNR = %.1f", newStatus.snr );
        lastRxAckSeq = newStatus.seq;
        return 2;
   }
   ESP_LOGW(TAG, "Ignoring packet with unknown type %u and size %d", packetType, packetSize );
   return 0;
}
bool waitForAck(uint32_t expectedSeq, unsigned long timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        serviceBackgroundTasks();
        const int packetType = handlePacket();
        if (packetType == 2) {
            if (lastRxAckSeq == expectedSeq) {
                ESP_LOGV(TAG, "ACK matched expected seq: %" PRIu32, expectedSeq);
                return true;
            }
            ESP_LOGW(TAG, "ACK seq mismatch: expected=%" PRIu32 " got=%" PRIu32,
                     expectedSeq, lastRxAckSeq);
        }
        delay(5);
    }
    return false;
}

loraTxPayload buildTxPayload() {
#if LORA_TX_PAYLOAD_KIND == LORA_PAYLOAD_KIND_GPS
    return location;
#elif LORA_TX_PAYLOAD_KIND == LORA_PAYLOAD_KIND_ENV
    return latestEnv;
#else
#error Unsupported LORA_TX_PAYLOAD_KIND
#endif
}

bool sendDataWithAckRetries(unsigned int maxAttempts) {
    txPkt = makeLoraDataPacket<loraTxPayload>(
        ++nextTxSeq,
        LoRa.packetSnr(),
        M5.Power.getBatteryLevel(),
        buildTxPayload());
    ESP_LOGI(TAG, "Sending packet type=%u seq=%" PRIu32, txPkt.type, txPkt.seq);
    for (unsigned int attempt = 1; attempt <= maxAttempts; ++attempt) {
        if (!sendPacket((char *)&txPkt, sizeof(txPkt))) {
            ESP_LOGE(TAG, "Send attempt %u failed", attempt);
            continue;
        }
        if (waitForAck(txPkt.seq, ACK_TIMEOUT_MS)) {
            ESP_LOGI(TAG, "ACK received on attempt %u", attempt);
            M5.Display.setColor(TFT_GREEN);
            M5.Display.fillCircle(20,20,10);
            smartDelay(250);
            return true;
        }
        ESP_LOGW(TAG, "ACK timeout on attempt %u", attempt);
        serviceBackgroundTasks();
    }
    return false;
}

void serviceBackgroundTasks() {
    //M5.update();
    ArduinoOTA.handle();
#ifdef SENDER
    while (GPS_SERIAL_PORT.available()) {
        gps.encode( GPS_SERIAL_PORT.read() );

        // auto myByte = GPS_SERIAL_PORT.read();
        // gps.encode( myByte);
        // ESP_LOGD(TAG, "%d", myByte );
    }
#endif
}

#ifdef RECEIVER
void handleReceiver() {
    static long lastDisplayUpdate = millis();

    int packetType = handlePacket();
    if (packetType == 1 || packetType == 3) {
        loraStatus newStatus = { LORA_PKT_ACK, lastRxDataSeq, LoRa.packetSnr(), M5.Power.getBatteryLevel() };
        ESP_LOGI(TAG, "Sending ACK seq: %" PRIu32, newStatus.seq);
        sendPacket( (char*)&newStatus, sizeof(newStatus) );
    }

    if( packetType == 1 ) { // got new location
        updateDisplay( locationPkt, false);
        postToThingsBoard( locationPkt );
        ESP_LOGI( TAG, "%s", "Posting to ESP" );
        msg = { "V", locationPkt.payload.speed };
        sendStatus(msg); // Send our data out to the ESP-NOW network
    }

    // Periodic display update and timeout check
    if (millis() - lastDisplayUpdate > PACKET_INTERVAL) {
        // Check for communication timeout
        if (millis() - lastPacketTime > NO_CONTACT_TIMEOUT ) {
            newLocation = { 0.0, 0.0, 0.0, 0.0 };
            locationPkt.payload = newLocation;
        }
        updateDisplay( locationPkt, false);
        lastDisplayUpdate = millis();
    }

    // MQTT loop with reduced frequency
    static unsigned long lastMqttLoop = 0;
    if (millis() - lastMqttLoop > 1000) { // Only check MQTT every second
        mqttLoop();
        lastMqttLoop = millis();
    }
}

void postToThingsBoard(loraGpsPacket newPkt) {
    unsigned char payload[TELEMETRY_DOC_SIZE];
    JsonDocument doc;

    gpsData newData = newPkt.payload;

    if( nearlyZero(newData.latitude) || newData.latitude > 90.0 || newData.latitude < -90.0 ||
            nearlyZero(newData.longitude) || newData.longitude > 180.0 || newData.longitude < -180.0 )
        return;

//    ESP_LOGV( TAG, "Considerintg posting %.1f", newData.speed );
//    if( newData.speed < 0.1 )
//        return;
    ESP_LOGI( TAG, "%s", "Posting" );

    doc["latitude"] = newData.latitude;
    doc["longitude"] = newData.longitude;
    doc["altitude"] = newData.altitude;
    doc["speed"] = newData.speed;
    //doc["ip_address"] = WiFi.localIP().toString().c_str();
    doc["pkt_rssi"] = LoRa.packetSnr();
    //doc["range"] = newPkt.lastRange;

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
      serviceBackgroundTasks();
      delay(20);
  } while (millis() - start < ms);
} 

bool initESPNow() {
    if (esp_now_init() != ESP_OK) {
        ESP_LOGI( TAG, "ESP-NOW init failed");
        return false;
    }

#if defined(ARDUINO_M5STACK_NANO)
    // Keep radio fully awake on NanoC6 to improve ESP-NOW RX reliability.
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif
    
    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    for(int i = 0; i < NUM_PEERS; ++i) {
        peers[i].lastHeard = millis();
        addESPPeer(peers[i]);
    }
    return true;
}
