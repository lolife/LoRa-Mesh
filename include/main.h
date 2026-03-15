#pragma once

#include <M5Unified.h>
#include <M5GFX.h>
#include <SPI.h>
#include <ArduinoOTA.h>
#include <SD.h>
#include <WiFi.h>
#include "esp_log.h"
#include "M5_SX127X.h"
#include <inttypes.h>
#include "lora_config.h"
#include "display.h"
#include "gps_processing.h"
#include "credentials.h"
#include "mk_espmesh_lib.h"
#include "mk_mqtt_lib.h"

gpsData location     = { 0.0, 0.0, 0.0, 0.0 };
gpsData newLocation  = { 0.0, 0.0, 0.0, 0.0 };
envData latestEnv    = { 0.0, 0.0, 0.0, 0.0, 0, 0 };
loraStatus newStatus = { 0, 0, 0.0, 0 };
loraGpsPacket locationPkt = { 0, 0, 0.0, 0, { 0.0, 0.0, 0.0, 0.0 } };
loraEnvPacket envPkt = { 0, 0, 0.0, 0, { 0.0, 0.0, 0.0, 0.0, 0, 0 } };
loraTxPacket txPkt = { 0, 0, 0.0, 0, {} };
unsigned long lastSensorRefresh = 0;

static uint32_t nextTxSeq = 0;
static uint32_t lastRxDataSeq = 0;
static uint32_t lastRxAckSeq = 0;
// Global variables
uint16_t screenColor = TFT_DARKGREEN;
static long lastPacketTime = 0;

gpsData home = {44.89401,-93.47717, 304.42, 0.0 };
static uint16_t lastRange = 0;

char TAG[36];

#ifdef SENDER
    #define GPS_SERIAL_PORT Serial1
    #include <TinyGPSPlus.h>
    TinyGPSPlus gps;
    static const uint32_t GPSBaud = 115200;
    #ifndef GPS_RX_PIN
        #define GPS_RX_PIN 14
    #endif
    #ifndef GPS_TX_PIN
        #define GPS_TX_PIN 13
    #endif
    static const int RXPin = GPS_RX_PIN;
    static const int TXPin = GPS_TX_PIN;
#endif

#if defined(ENV3)
    #include <M5UnitUnified.h>
    #include <M5UnitUnifiedENV.h>
    // M5 Units
    m5::unit::UnitUnified Units;
    m5::unit::UnitENV3 unitENV3;
    bool initializeSensors();
#endif

// Function prototypes
void handleSender();
void handleReceiver();
static void smartDelay(unsigned long ms);
void postToThingsBoard(loraEnvPacket newPkt);
bool initializeWiFi();
bool waitForAck(uint32_t expectedSeq, unsigned long timeoutMs);
bool sendDataWithAckRetries(unsigned int maxAttempts);
loraTxPayload buildTxPayload();
void serviceBackgroundTasks();
bool initESPNow();
int handlePacket();
