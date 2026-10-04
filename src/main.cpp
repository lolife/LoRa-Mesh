#include "main.h"
#include <cmath>
#include <algorithm>
#include <esp_ota_ops.h>
#include <esp_app_desc.h>
#include <esp_system.h>

extern char    loraMessage[MAX_MSG_SIZE];

static void setupOtaDiagnostics() {
    ArduinoOTA.onStart([]() {
        ESP_LOGI(TAG, "OTA start");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned int lastPct = 0;
        const unsigned int pct = (total > 0) ? ((progress * 100U) / total) : 0;
        if (pct >= lastPct + 10U || pct == 100U) {
            lastPct = pct;
            ESP_LOGI(TAG, "OTA progress: %u%%", pct);
        }
    });
    ArduinoOTA.onEnd([]() {
        ESP_LOGI(TAG, "OTA end (device should reboot)");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        ESP_LOGE(TAG, "OTA error: %u", error);
    });
}

void setup() {
    //Serial.begin( 115200 );
    // Initialize M5Stack with proper configuration
    ESP_LOGI(TAG, "FW %s %s", __DATE__, __TIME__);
    auto cfg = M5.config();
    cfg.clear_display = true;
    cfg.internal_imu = true;   // Accelerometer wakes the display on movement.
    cfg.internal_rtc = false;  // Disable RTC to avoid ADC conflict
    cfg.internal_spk = false;  // Disable speaker
    cfg.internal_mic = false;  // Disable microphone
    M5.begin(cfg);
    
    initializeDisplayMotion();
    M5.Display.setRotation(1);
    M5.Display.setFont(&Orbitron_Light_32);
    
    // Initialize LoRa
    if( ! setupLoRa() )
        displayMessage("LoRa init failed", true, screenColor );
    
    // Display initial mode
    initializeWiFi();
    configureMeshIdentity();
    mqttClient.setBufferSize(TELEMETRY_DOC_SIZE + 64);
    mqttClient.setCallback(mqttCallback);

#ifdef SENDER
    snprintf( TAG, sizeof(TAG), "➡️LoRaMeshSender" ); 
    screenColor = TFT_NAVY;
    //M5.Display.setRotation(0);
#ifdef ENV3
    sensorsReady = initializeSensors();
    if (!sensorsReady)
        displayMessage("Sensor(s) failed.", true, screenColor );
#endif
    GPS_SERIAL_PORT.begin( GPSBaud, SERIAL_8N1, RXPin, TXPin );
    ESP_LOGI(TAG, "GPS UART pins rx=%d tx=%d baud=%" PRIu32, RXPin, TXPin, GPSBaud);
    if (RXPin == LORA_SCLK || RXPin == LORA_MISO || RXPin == LORA_MOSI ||
        TXPin == LORA_SCLK || TXPin == LORA_MISO || TXPin == LORA_MOSI) {
        ESP_LOGE(TAG, "GPS UART pin conflicts with LoRa SPI pins (SCLK=%d MISO=%d MOSI=%d)",
                 LORA_SCLK, LORA_MISO, LORA_MOSI);
        displayMessage("GPS/LoRa pin conflict", true, TFT_RED);
    }
    ESP_LOGI( TAG, "%s", "LoRa Sender" );
    displayMessage("LoRa Sender", true, screenColor);
#else
    if (!initESPNow())
        displayMessage("ESP-NOW init failed", true, screenColor );
    snprintf( TAG, sizeof(TAG), "⬅️LoRaMeshReceive" ); 
    ESP_LOGI( TAG, "%s", "LoRa Receiverr" );
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

#ifdef SENDER
#ifdef ENV3
    if (sensorsReady) {
        static unsigned long lastShtRefresh = 0;
        static unsigned long lastPressureRefresh = 0;
        Units.update();
        const unsigned long now = millis();
        if (unitENV3.sht30.updated()) lastShtRefresh = now;
        if (unitENV3.qmp6988.updated()) lastPressureRefresh = now;
        const float temp1 = unitENV3.sht30.temperature();
        const float temp2 = unitENV3.qmp6988.temperature();
        const float humidity = unitENV3.sht30.humidity();
        const float pressure = unitENV3.qmp6988.pressure() / 100.0f;
        if (lastShtRefresh && lastPressureRefresh &&
            now - lastShtRefresh <= NO_CONTACT_TIMEOUT &&
            now - lastPressureRefresh <= NO_CONTACT_TIMEOUT &&
            std::isfinite(temp1) && std::isfinite(temp2) &&
            std::isfinite(humidity) && std::isfinite(pressure)) {
            latestTelemetry.env.temperature = (temp1 + temp2) / 2.0f;
            latestTelemetry.env.humidity = humidity;
            latestTelemetry.env.pressure = pressure;
            // Age follows the older component so neither stale sensor stays valid.
            lastSensorRefresh = now - std::max(now - lastShtRefresh, now - lastPressureRefresh);
            envAvailable = true;
        }
    }
#endif
    static unsigned long lastGpsDiag = 0;
//    if (gps.location.isValid() && gps.location.isUpdated()) {
    if (gps.location.isValid() && gps.location.isUpdated() && gps.location.age() < NO_CONTACT_TIMEOUT) {
        legacyGpsData rawLocation = {
            (float)gps.location.lat(),
            (float)gps.location.lng(),
            (float)gps.altitude.meters(),
            (float)gps.speed.mph(),
            (int)gps.satellites.value()
        };
        legacyGpsData filteredLocation = rawLocation;
        ESP_LOGV( TAG, "speed is %.2f", rawLocation.speed );

        if (acceptGpsMeasurement(rawLocation, &filteredLocation)) {
            location = filteredLocation;
            lastGpsRefresh = millis();
            gpsAvailable = true;
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

        if ((buildTxPayload().available & (TELEMETRY_HAS_GPS | TELEMETRY_HAS_ENV)) &&
            !sendDataWithAckRetries(ACK_RETRY_COUNT))
            ESP_LOGE( TAG, "Error sending payload data" );
        
        // Update display immediately after sending
        updateDisplay(txPkt, true, LoRa.packetSnr(), newStatus.seq == txPkt.seq && txPkt.seq != 0);
        lastDisplayUpdate = millis();
    }

    // Clear expired sensor flags even when there is no packet to send.
    txPkt.payload.available &= buildTxPayload().available;
    // Periodic display update
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE) {
        updateDisplay(txPkt, true, LoRa.packetSnr(), newStatus.seq == txPkt.seq && txPkt.seq != 0);
        lastDisplayUpdate = millis();
    }
}

bool initializeSensors() {
    //Initialize I2C for sensors
    auto pin_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_scl = M5.getPin(m5::pin_name_t::port_a_scl);

    Wire.begin(pin_sda, pin_scl, 400000U);

#ifdef TOF
    if ( !Units.add(tofUnit, Wire) ) {
        displayMessage( "Failed to add ToF Unit", true );
        ESP_LOGE( TAG, "%s", "Failed to add ToF Unit" );
    }
    return(Units.begin());
#endif

#if defined(ENV3)
    if ( !Units.add(unitENV3, Wire) ) {
        displayMessage( "Failed to add ENV Unit", true, screenColor );
        ESP_LOGE( TAG, "%s", "Failed to add ENV Unit" );
        return false;
    }

    if ( !Units.begin() ) {
        displayMessage( "Failed to add start Unit", true, screenColor );
        ESP_LOGE( TAG, "%s", "Failed to start Unit" );
        return false;
    } 
#endif
    return true;
}

int handlePacket() {
    int packetSize = receivePacket();
    if( packetSize < 1 )
        return 0;

    const uint8_t packetType = static_cast<uint8_t>(loraMessage[0]);

    if (packetType == LORA_PKT_TELEMETRY || packetType == 3) {
        loraTelemetryPacket received = {};
        if (!decodeLoraTelemetryPacket(loraMessage, packetSize, &received)) {
            ESP_LOGW(TAG, "Unexpected telemetry packet size %d", packetSize);
            return 0;
        }
        txPkt = received;
        latestTelemetry = received.payload;
        lastPacketTime = millis();
        lastRxDataSeq = received.seq;
        return 1;
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

telemetryData buildTxPayload() {
    telemetryData payload = {};
    if (gpsAvailable && millis() - lastGpsRefresh <= NO_CONTACT_TIMEOUT) {
        payload.available |= TELEMETRY_HAS_GPS;
        payload.gps = {
            static_cast<int64_t>(std::llround(location.latitude * static_cast<double>(GPS_NANODEGREES_PER_DEGREE))),
            static_cast<int64_t>(std::llround(location.longitude * static_cast<double>(GPS_NANODEGREES_PER_DEGREE))),
            location.altitude, location.speed,
            static_cast<uint8_t>(location.sats < 0 ? 0 : location.sats > 255 ? 255 : location.sats), 0
        };
    }
    if (envAvailable && millis() - lastSensorRefresh <= NO_CONTACT_TIMEOUT) {
        payload.available |= TELEMETRY_HAS_ENV;
        payload.env = latestTelemetry.env;
    }
    if (newStatus.type == LORA_PKT_ACK) payload.available |= TELEMETRY_HAS_RETURN_SNR;
    return payload;
}

bool sendDataWithAckRetries(unsigned int maxAttempts) {
    txPkt = {LORA_PKT_TELEMETRY, ++nextTxSeq, LoRa.packetSnr(),
             M5.Power.getBatteryLevel(), buildTxPayload()};
    ESP_LOGI(TAG, "Sending packet type=%u seq=%" PRIu32, txPkt.type, txPkt.seq);
    for (unsigned int attempt = 1; attempt <= maxAttempts; ++attempt) {
        if (!sendPacket((char *)&txPkt, sizeof(txPkt))) {
            ESP_LOGE(TAG, "Send attempt %u failed", attempt);
            continue;
        }
        if (waitForAck(txPkt.seq, ACK_TIMEOUT_MS)) {
            ESP_LOGI(TAG, "ACK received on attempt %u", attempt);
            if (isDisplayAwake()) {
                M5.Display.setColor(TFT_GREEN);
                M5.Display.fillCircle(20,20,10);
            }
            smartDelay(250);
            return true;
        }
        ESP_LOGW(TAG, "ACK timeout on attempt %u", attempt);
        serviceBackgroundTasks();
    }
    return false;
}

void serviceBackgroundTasks() {
    if (serviceDisplayMotion()) {
#ifdef SENDER
        updateDisplay(txPkt, true, LoRa.packetSnr(), newStatus.seq == txPkt.seq && txPkt.seq != 0);
#else
        if (millis() - lastPacketTime > NO_CONTACT_TIMEOUT) txPkt.payload.available = 0;
        updateDisplay(txPkt, false, LoRa.packetSnr(), txPkt.payload.available != 0);
#endif
    }
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
    static unsigned long lastDisplayUpdate = 0;
    static unsigned long lastPost = 0;
    static unsigned long lastMqttLoop = 0;
    const int packetType = handlePacket();
    if (packetType == 1) {
        newStatus = {LORA_PKT_ACK, lastRxDataSeq, LoRa.packetSnr(), M5.Power.getBatteryLevel()};
        sendPacket(reinterpret_cast<char*>(&newStatus), sizeof(newStatus));
        updateDisplay(txPkt, false, LoRa.packetSnr(), true);
        lastDisplayUpdate = millis();
        if (millis() - lastPost > TB_POST_INTERVAL && postToThingsBoard(txPkt)) lastPost = millis();

        // Each available measurement gets its own versioned mesh message.
        if (telemetryHas(txPkt.payload, TELEMETRY_HAS_ENV)) {
            StatusMessage msg = {};
            strncpy(msg.deviceName, me->name, sizeof(msg.deviceName) - 1);
            strcpy(msg.varName, "T");
            msg.varValue = txPkt.payload.env.temperature;
            sendStatus(msg);
        }
        else if (telemetryHas(txPkt.payload, TELEMETRY_HAS_GPS) && locationInBounds(txPkt.payload.gps)) {
            StatusMessage msg = {};
            strncpy(msg.deviceName, me->name, sizeof(msg.deviceName) - 1);
            strcpy(msg.varName, "V");
            msg.varValue = txPkt.payload.gps.speed;
            sendStatus(msg);
        }
    }
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE) {
        if (millis() - lastPacketTime > NO_CONTACT_TIMEOUT) txPkt.payload.available = 0;
        updateDisplay(txPkt, false, LoRa.packetSnr(), txPkt.payload.available != 0);
        lastDisplayUpdate = millis();
    }
    if (millis() - lastMqttLoop > 1000) {
        mqttLoop();
        lastMqttLoop = millis();
    }
}

bool postToThingsBoard(const loraTelemetryPacket &pkt) {
    char payload[TELEMETRY_DOC_SIZE];
    JsonDocument doc;
    if (telemetryHas(pkt.payload, TELEMETRY_HAS_GPS) && locationInBounds(pkt.payload.gps)) {
        doc["latitude"] = gpsLatitudeDegrees(pkt.payload.gps);
        doc["longitude"] = gpsLongitudeDegrees(pkt.payload.gps);
        if (std::isfinite(pkt.payload.gps.altitude)) doc["altitude"] = pkt.payload.gps.altitude;
        if (std::isfinite(pkt.payload.gps.speed)) doc["speed"] = pkt.payload.gps.speed;
        doc["satellites"] = pkt.payload.gps.sats;
    }
    if (telemetryHas(pkt.payload, TELEMETRY_HAS_ENV)) {
        if (std::isfinite(pkt.payload.env.temperature)) doc["temperature"] = pkt.payload.env.temperature;
        if (std::isfinite(pkt.payload.env.humidity)) doc["humidity"] = pkt.payload.env.humidity;
        if (std::isfinite(pkt.payload.env.pressure)) doc["pressure"] = pkt.payload.env.pressure;
    }
    if (doc.size() == 0 || !mqttClient.connected()) return false;
    doc["pkt_rssi"] = LoRa.packetSnr();
    doc["remote_battery"] = pkt.batt;
    doc["battery"] = M5.Power.getBatteryLevel();
    if (measureJson(doc) >= sizeof(payload)) return false;
    const size_t n = serializeJson(doc, payload, sizeof(payload));
    return mqttClient.publish(TELEMETRY_TOPIC, payload, n);
}
#endif
bool initializeWiFi() {
    ESP_LOGI( TAG, "Connecting");
    
    // Configure WiFi for lower power
    WiFi.mode(WIFI_STA);
    
    // Reduce TX power to save energy (adjust based on signal strength needs)
    WiFi.setTxPower(WIFI_POWER_11dBm); // Lower from default 20dBm
    
    // Enable power saving mode
    WiFi.setSleep(false); // Keep the radio awake for ESP-NOW reception.
    
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

    setupOtaDiagnostics();
    ArduinoOTA.begin();
    ESP_LOGI(TAG, "ArduinoOTA ready at %s", WiFi.localIP().toString().c_str());
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
    if (!configureMeshIdentity()) return false;
    validatePeerList();
    if (esp_now_init() != ESP_OK) {
        ESP_LOGI( TAG, "ESP-NOW init failed");
        return false;
    }

    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    for(int i = 0; i < NUM_PEERS; ++i) {
        peers[i].lastHeard = millis();
        addESPPeer(peers[i], 0);
    }
    return initMeshTx();
}
