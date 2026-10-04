#pragma once

#include "lora_protocol.h"

bool nearlyZero(double valueToCheck);
bool locationInBounds(legacyGpsData newLocation);
bool acceptGpsMeasurement(const legacyGpsData &raw, legacyGpsData *filtered);

#ifdef SENDER
bool initSdLogging();
void appendGpsLogRow(const legacyGpsData &predicted,
                     const legacyGpsData &actual,
                     const legacyGpsData &filtered,
                     float snr,
                     bool accepted,
                     float nis);
#endif

inline bool locationInBounds(const gpsData &data) {
    return locationInBounds(legacyGpsData{static_cast<float>(gpsLatitudeDegrees(data)),
        static_cast<float>(gpsLongitudeDegrees(data)), data.altitude, data.speed, data.sats});
}
