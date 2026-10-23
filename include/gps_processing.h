#pragma once

#include "lora_protocol.h"
#include "sensor_config.h"

bool nearlyZero(double valueToCheck);
bool locationInBounds(legacyGpsData newLocation);
bool acceptGpsMeasurement(const legacyGpsData &raw, legacyGpsData *filtered);

#if defined(SENDER) && ENABLE_GPS
bool initSdLogging();
void appendGpsLogRow(const legacyGpsData &predicted,
                     const legacyGpsData &actual,
                     const legacyGpsData &filtered,
                     float snr,
                     bool accepted,
                     float nis);
#endif

inline bool locationInBounds(const gpsData &data) {
    return data.latitudeNanodegrees >= -90 * GPS_NANODEGREES_PER_DEGREE &&
        data.latitudeNanodegrees <= 90 * GPS_NANODEGREES_PER_DEGREE &&
        data.longitudeNanodegrees >= -180 * GPS_NANODEGREES_PER_DEGREE &&
        data.longitudeNanodegrees <= 180 * GPS_NANODEGREES_PER_DEGREE;
}
