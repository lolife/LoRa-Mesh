#pragma once

#include "lora_protocol.h"

bool nearlyZero(double valueToCheck);
bool locationInBounds(gpsData newLocation);
bool acceptGpsMeasurement(const gpsData &raw, gpsData *filtered);

#ifdef SENDER
bool initSdLogging();
void appendGpsLogRow(const gpsData &predicted,
                     const gpsData &actual,
                     const gpsData &filtered,
                     float snr,
                     bool accepted,
                     float nis);
#endif
