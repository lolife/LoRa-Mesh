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
#include "credentials.h"
#include "mk_espmesh_lib.h"
#include "mk_mqtt_lib.h"
#include <math.h>

gpsData location     = { 0.0, 0.0, 0.0, 0.0 };
gpsData newLocation  = { 0.0, 0.0, 0.0, 0.0 };
loraStatus newStatus = { 0, 0, 0.0, 0 };
loraDataPacket locationPkt = { 0, 0, 0.0, 0, { 0.0, 0.0, 0.0, 0.0 } };

static uint32_t nextTxSeq = 0;
static uint32_t lastRxLocationSeq = 0;
static uint32_t lastRxAckSeq = 0;
static StatusMessage msg = { "S", 1.0};
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
    static const int RXPin = 18, TXPin = 17;
#endif

// Function prototypes
void handleSender();
void handleReceiver();
static void smartDelay(unsigned long ms);
void postToThingsBoard(loraDataPacket newPkt);
bool initializeWiFi();
bool nearlyZero( double valueToCheck );
bool locationInBounds( gpsData newLocation );
bool waitForAck(uint32_t expectedSeq, unsigned long timeoutMs);
bool sendLocationWithAckRetries(unsigned int maxAttempts);
void serviceBackgroundTasks();
bool acceptGpsMeasurement(const gpsData &raw, gpsData *filtered);
bool initESPNow();
int handlePacket();
#ifdef SENDER
bool initSdLogging();
void appendGpsLogRow(const gpsData &predicted,
                     const gpsData &actual,
                     const gpsData &filtered,
                     float snr,
                     bool accepted,
                     float nis);
#endif


namespace {
constexpr float KF_DEG_TO_RAD = 0.01745329251994329577f;
constexpr float KF_RAD_TO_DEG = 57.295779513082320876f;
constexpr float EARTH_RADIUS_M = 6371000.0f;
constexpr float GPS_MEASUREMENT_SIGMA_M = 10.0f;    // Typical consumer GPS noise floor.
constexpr float MODEL_ACCEL_SIGMA_MPS2 = 4.0f;      // Motion model uncertainty.
constexpr float OUTLIER_NIS_THRESHOLD = 20.0f;      // Reject only very unlikely jumps.
constexpr float MAX_FILTER_DT_S = 15.0f;            // TX/ACK cycles can introduce multi-second gaps.
constexpr float FILTER_RESET_DT_S = 30.0f;          // Re-sync if update gap is very large.

struct Kalman1D {
    float pos = 0.0f;
    float vel = 0.0f;
    float p00 = 0.0f;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = 0.0f;
};

struct LocalFrame {
    bool initialized = false;
    float refLatRad = 0.0f;
    float refLonRad = 0.0f;
    float cosRefLat = 1.0f;
};

struct GpsKalmanFilter {
    bool initialized = false;
    uint32_t lastMs = 0;
    Kalman1D east;
    Kalman1D north;
    LocalFrame frame;
};
}
GpsKalmanFilter gGpsFilter;

#ifdef SENDER
constexpr int SD_SPI_CS = 4;
constexpr const char *GPS_LOG_FILE = "/gps_kalman_log.csv";
bool sdLoggingReady = false;
#endif

void initAxis(Kalman1D &axis, float initialPos);
void predictAxis(Kalman1D &axis, float dt);
float updateAxis(Kalman1D &axis, float measurement, float measurementVar);
void latLonToLocal(const LocalFrame &frame, float latDeg, float lonDeg, float &eastM, float &northM);
void localToLatLon(const LocalFrame &frame, float eastM, float northM, float &latDeg, float &lonDeg);
