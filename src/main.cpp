#include "main.h"

extern char    loraMessage[MAX_MSG_SIZE];

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
    //GPS_SERIAL_PORT.begin( GPSBaud );
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
    static long lastPacketTime = millis();
    static long lastDisplayUpdate = millis();
#ifdef SENDER
    if (gps.location.isValid() && gps.location.isUpdated()) {
        gpsData rawLocation = {
            (float)gps.location.lat(),
            (float)gps.location.lng(),
            (float)gps.altitude.meters(),
            (float)gps.speed.mph()
        };
        gpsData filteredLocation = rawLocation;

        if (acceptGpsMeasurement(rawLocation, &filteredLocation)) {
            location = filteredLocation;
        } else {
            ESP_LOGW(TAG, "Rejected implausible GPS point: %.6f, %.6f",
                     rawLocation.latitude, rawLocation.longitude);
        }
    }
#endif
    // Send packet at regular intervals
    if (millis() - lastPacketTime > PACKET_INTERVAL) {
        lastPacketTime = millis();

        if( !sendLocationWithAckRetries(ACK_RETRY_COUNT) )
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

    if (packetSize == sizeof(loraDataPacket) ) {
        //loraDataPacket locationPkt = { 0, 0, 0.0, 0, { 0.0, 0.0, 0.0, 0.0 } };
        memcpy(&locationPkt, &loraMessage, packetSize );
        if (locationPkt.type != LORA_PKT_LOCATION) {
            ESP_LOGW(TAG, "Unexpected type %u for location packet size", locationPkt.type);
            return 0;
        }
        lastRxLocationSeq = locationPkt.seq;
        newLocation = locationPkt.location;
        ESP_LOGV( TAG, "Got msg: %.8f", newLocation.speed );
        return 1;
    }
    else if (packetSize == sizeof(loraStatus) ) {
        //loraStatus newStatus = { 0, 0, 0.0, 0 };
        memcpy( &newStatus, &loraMessage, packetSize );
        ESP_LOGI( TAG, "SNR = %.1f", newStatus.snr );
        if (newStatus.type != LORA_PKT_ACK) {
            ESP_LOGW(TAG, "Unexpected type %u for ACK packet size", newStatus.type);
            return 0;
        }
        lastRxAckSeq = newStatus.seq;
        return 2;
   }
   ESP_LOGW(TAG, "Ignoring packet with unexpected size: %d", packetSize );
   return 0;
}
bool waitForAck(uint32_t expectedSeq, unsigned long timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        serviceBackgroundTasks();
        if (receivePacket() == 2 && lastRxAckSeq == expectedSeq)
            return true;
        delay(5);
    }
    return false;
}

bool sendLocationWithAckRetries(unsigned int maxAttempts) {
    locationPkt = { LORA_PKT_LOCATION,  ++nextTxSeq, LoRa.packetSnr(), M5.Power.getBatteryLevel(), location };
    for (unsigned int attempt = 1; attempt <= maxAttempts; ++attempt) {
        if (!sendPacket((char *)&locationPkt, sizeof(locationPkt))) {
            ESP_LOGE(TAG, "Send attempt %u failed", attempt);
            continue;
        }
        if (waitForAck(locationPkt.seq, ACK_TIMEOUT_MS)) {
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
    if( packetType == 1 ) { // got new location
        loraStatus newStatus = { LORA_PKT_ACK, lastRxLocationSeq, LoRa.packetSnr(), M5.Power.getBatteryLevel() };
        ESP_LOGI(TAG, "Sending ACK seq: %" PRIu32, newStatus.seq);
        sendPacket( (char*)&newStatus, sizeof(newStatus) );
        updateDisplay( locationPkt, false);
        postToThingsBoard( locationPkt );
        ESP_LOGI( TAG, "%s", "Posting to ESP" );
        msg = { "V", locationPkt.location.speed };
        sendStatus(msg); // Send our data out to the ESP-NOW network
    }

    // Periodic display update and timeout check
    if (millis() - lastDisplayUpdate > PACKET_INTERVAL) {
        // Check for communication timeout
        if (millis() - lastPacketTime > NO_CONTACT_TIMEOUT ) {
            newLocation = { 0.0, 0.0, 0.0, 0.0 };
            locationPkt.location = newLocation;
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

void postToThingsBoard(loraDataPacket newPkt) {
    unsigned char payload[TELEMETRY_DOC_SIZE];
    JsonDocument doc;

    gpsData newData = newPkt.location;

const double EPSILON = 1e-9;  // or whatever tolerance makes sense for your use case

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

bool acceptGpsMeasurement(const gpsData &raw, gpsData *filtered) {
    if (!locationInBounds(raw)) {
        return false;
    }

    if (!gGpsFilter.initialized) {
        gGpsFilter.frame.initialized = true;
        gGpsFilter.frame.refLatRad = raw.latitude * KF_DEG_TO_RAD;
        gGpsFilter.frame.refLonRad = raw.longitude * KF_DEG_TO_RAD;
        gGpsFilter.frame.cosRefLat = cosf(gGpsFilter.frame.refLatRad);
        if (fabsf(gGpsFilter.frame.cosRefLat) < 0.01f) {
            gGpsFilter.frame.cosRefLat = 0.01f;
        }

        initAxis(gGpsFilter.east, 0.0f);
        initAxis(gGpsFilter.north, 0.0f);
        gGpsFilter.lastMs = millis();
        gGpsFilter.initialized = true;
        *filtered = raw;
#ifdef SENDER
        if (sdLoggingReady) {
            appendGpsLogRow(raw, raw, raw, newStatus.snr, true, 0.0f);
        }
#endif
        return true;
    }

    const uint32_t nowMs = millis();
    float dt = (nowMs - gGpsFilter.lastMs) / 1000.0f;
    gGpsFilter.lastMs = nowMs;
    if (dt <= 0.0f) {
        dt = 0.05f;
    } else if (dt > FILTER_RESET_DT_S) {
        float eastM = 0.0f;
        float northM = 0.0f;
        latLonToLocal(gGpsFilter.frame, raw.latitude, raw.longitude, eastM, northM);
        initAxis(gGpsFilter.east, eastM);
        initAxis(gGpsFilter.north, northM);
        *filtered = raw;
#ifdef SENDER
        if (sdLoggingReady) {
            appendGpsLogRow(raw, raw, raw, newStatus.snr, true, 0.0f);
        }
#endif
        return true;
    } else if (dt > MAX_FILTER_DT_S) {
        dt = MAX_FILTER_DT_S;
    }

    predictAxis(gGpsFilter.east, dt);
    predictAxis(gGpsFilter.north, dt);

    float measuredEast = 0.0f;
    float measuredNorth = 0.0f;
    latLonToLocal(gGpsFilter.frame, raw.latitude, raw.longitude, measuredEast, measuredNorth);

    const float measurementVar = GPS_MEASUREMENT_SIGMA_M * GPS_MEASUREMENT_SIGMA_M;
    float predictedLat = raw.latitude;
    float predictedLon = raw.longitude;
    localToLatLon(gGpsFilter.frame, gGpsFilter.east.pos, gGpsFilter.north.pos, predictedLat, predictedLon);
    gpsData predictedLocation = { predictedLat, predictedLon, raw.altitude, raw.speed };
    gpsData actualLocation = raw;
    const float nisEast = (measuredEast - gGpsFilter.east.pos) * (measuredEast - gGpsFilter.east.pos) /
                          (gGpsFilter.east.p00 + measurementVar);
    const float nisNorth = (measuredNorth - gGpsFilter.north.pos) * (measuredNorth - gGpsFilter.north.pos) /
                           (gGpsFilter.north.p00 + measurementVar);
    const float nis = nisEast + nisNorth;

    if (nis > OUTLIER_NIS_THRESHOLD) {
        ESP_LOGI( TAG, "%s", "Dropping errant data point" );
#ifdef SENDER
        if (sdLoggingReady) {
            appendGpsLogRow(predictedLocation, actualLocation, predictedLocation, newStatus.snr, false, nis);
        }
#endif
        return false;
    }

    (void)updateAxis(gGpsFilter.east, measuredEast, measurementVar);
    (void)updateAxis(gGpsFilter.north, measuredNorth, measurementVar);

    float latDeg = raw.latitude;
    float lonDeg = raw.longitude;
    localToLatLon(gGpsFilter.frame, gGpsFilter.east.pos, gGpsFilter.north.pos, latDeg, lonDeg);

    filtered->latitude = latDeg;
    filtered->longitude = lonDeg;
    filtered->altitude = raw.altitude;
    filtered->speed = raw.speed;
#ifdef SENDER
    if (sdLoggingReady) {
        appendGpsLogRow(predictedLocation, actualLocation, *filtered, newStatus.snr, true, nis);
    }
#endif
    return true;
}

#ifdef SENDER
bool initSdLogging() {
    if (!SD.begin(SD_SPI_CS)) {
        ESP_LOGE(TAG, "SD init failed; GPS logging disabled");
        return false;
    }

    if (!SD.exists(GPS_LOG_FILE)) {
        File logFile = SD.open(GPS_LOG_FILE, FILE_WRITE);
        if (!logFile) {
            ESP_LOGE(TAG, "Failed to create log file %s", GPS_LOG_FILE);
            return false;
        }
        logFile.println("time,pred_lat,pred_lon,actual_lat,actual_lon,filtered_lat,filtered_lon,speed_mph,snr,accepted,nis");
        logFile.close();
    }

    ESP_LOGI(TAG, "SD logging ready: %s", GPS_LOG_FILE);
    return true;
}

void appendGpsLogRow(const gpsData &predicted,
                     const gpsData &actual,
                     const gpsData &filtered,
                     float snr,
                     bool accepted,
                     float nis) {
    File logFile = SD.open(GPS_LOG_FILE, FILE_APPEND);
    if (!logFile) {
        ESP_LOGW(TAG, "Failed to open %s for append", GPS_LOG_FILE);
        return;
    }

    char timeBuf[32];
    if (gps.date.isValid() && gps.time.isValid()) {
        snprintf(timeBuf, sizeof(timeBuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
        snprintf(timeBuf, sizeof(timeBuf), "millis:%lu", (unsigned long)millis());
    }

    logFile.printf("%s,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.2f,%.2f,%d,%.4f\n",
                   timeBuf,
                   predicted.latitude, predicted.longitude,
                   actual.latitude, actual.longitude,
                   filtered.latitude, filtered.longitude,
                   actual.speed, snr,
                   accepted ? 1 : 0, nis);
    logFile.close();
}
#endif

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

void initAxis(Kalman1D &axis, float initialPos) {
    axis.pos = initialPos;
    axis.vel = 0.0f;
    axis.p00 = 25.0f;   // Position variance (m^2)
    axis.p01 = 0.0f;
    axis.p10 = 0.0f;
    axis.p11 = 100.0f;  // Velocity variance (m^2/s^2)
}

void predictAxis(Kalman1D &axis, float dt) {
    axis.pos += axis.vel * dt;

    const float oldP00 = axis.p00;
    const float oldP01 = axis.p01;
    const float oldP10 = axis.p10;
    const float oldP11 = axis.p11;

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    const float accelVar = MODEL_ACCEL_SIGMA_MPS2 * MODEL_ACCEL_SIGMA_MPS2;

    const float q00 = 0.25f * dt4 * accelVar;
    const float q01 = 0.5f * dt3 * accelVar;
    const float q11 = dt2 * accelVar;

    axis.p00 = oldP00 + dt * (oldP01 + oldP10) + dt2 * oldP11 + q00;
    axis.p01 = oldP01 + dt * oldP11 + q01;
    axis.p10 = oldP10 + dt * oldP11 + q01;
    axis.p11 = oldP11 + q11;
}

float updateAxis(Kalman1D &axis, float measurement, float measurementVar) {
    const float innovation = measurement - axis.pos;
    const float s = axis.p00 + measurementVar;
    const float k0 = axis.p00 / s;
    const float k1 = axis.p10 / s;

    const float oldP00 = axis.p00;
    const float oldP01 = axis.p01;

    axis.pos += k0 * innovation;
    axis.vel += k1 * innovation;

    axis.p00 = axis.p00 - (k0 * oldP00);
    axis.p01 = axis.p01 - (k0 * oldP01);
    axis.p10 = axis.p10 - (k1 * oldP00);
    axis.p11 = axis.p11 - (k1 * oldP01);

    return (innovation * innovation) / s;
}

void latLonToLocal(const LocalFrame &frame, float latDeg, float lonDeg, float &eastM, float &northM) {
    const float latRad = latDeg * KF_DEG_TO_RAD;
    const float lonRad = lonDeg * KF_DEG_TO_RAD;
    eastM = (lonRad - frame.refLonRad) * frame.cosRefLat * EARTH_RADIUS_M;
    northM = (latRad - frame.refLatRad) * EARTH_RADIUS_M;
}

void localToLatLon(const LocalFrame &frame, float eastM, float northM, float &latDeg, float &lonDeg) {
    const float latRad = frame.refLatRad + (northM / EARTH_RADIUS_M);
    const float lonRad = frame.refLonRad + (eastM / (EARTH_RADIUS_M * frame.cosRefLat));
    latDeg = latRad * KF_RAD_TO_DEG;
    lonDeg = lonRad * KF_RAD_TO_DEG;
}
