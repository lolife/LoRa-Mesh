#include "gps_processing.h"

#include <M5Unified.h>
#include <cstdio>
#include <math.h>
#include "esp_log.h"

#ifdef SENDER
#include <SD.h>
#include <TinyGPSPlus.h>
extern TinyGPSPlus gps;
#endif

extern char TAG[36];
extern loraStatus newStatus;

namespace {
constexpr double EPSILON = 1e-9;
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

GpsKalmanFilter gGpsFilter;

#ifdef SENDER
constexpr int SD_SPI_CS = 4;
constexpr const char *GPS_LOG_FILE = "/gps_kalman_log.csv";
bool sdLoggingReady = false;
#endif

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
}

bool nearlyZero(double valueToCheck) {
    return fabs(valueToCheck) < EPSILON;
}

bool locationInBounds(gpsData newLocation) {
    if (nearlyZero(newLocation.latitude) || newLocation.latitude > 90.0 || newLocation.latitude < -90.0 ||
        nearlyZero(newLocation.longitude) || newLocation.longitude > 180.0 || newLocation.longitude < -180.0) {
        return false;
    }
    return true;
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
        ESP_LOGI(TAG, "%s", "Dropping errant data point");
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
    sdLoggingReady = true;
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
