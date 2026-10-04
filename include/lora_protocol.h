#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

static constexpr int64_t GPS_NANODEGREES_PER_DEGREE = 1000000000LL;

#pragma pack(push, 1)
struct gpsData {
    int64_t latitudeNanodegrees;
    int64_t longitudeNanodegrees;
    float altitude;
    float speed;
    uint8_t sats;
    uint8_t quality;
};

struct envData {
    float temperature;
    float humidity;
    float pressure;
    float gas_resistance;
    int32_t iaq;
    int32_t iaq_q;
};

struct telemetryData {
    uint8_t available;
    gpsData gps;
    envData env;
};

struct loraStatus {
    uint8_t type;
    uint32_t seq;
    float snr;
    int32_t batt;
};

struct loraTelemetryPacket {
    uint8_t type;
    uint32_t seq;
    float snr;
    int32_t batt;
    telemetryData payload;
};

// Transitional format used by sender firmware deployed before nanodegree
// coordinates were introduced. The receiver accepts both packet formats.
struct legacyGpsData {
    float latitude;
    float longitude;
    float altitude;
    float speed;
    int32_t sats;
};

struct legacyTelemetryData {
    uint8_t available;
    legacyGpsData gps;
    envData env;
};

struct legacyLoraTelemetryPacket {
    uint8_t type;
    uint32_t seq;
    float snr;
    int32_t batt;
    legacyTelemetryData payload;
};

// Transitional nanodegree format used before the RTK fix quality was added.
struct nanodegreeGpsDataV1 {
    int64_t latitudeNanodegrees;
    int64_t longitudeNanodegrees;
    float altitude;
    float speed;
    uint8_t sats;
};

struct nanodegreeTelemetryDataV1 {
    uint8_t available;
    nanodegreeGpsDataV1 gps;
    envData env;
};

struct nanodegreeLoraTelemetryPacketV1 {
    uint8_t type;
    uint32_t seq;
    float snr;
    int32_t batt;
    nanodegreeTelemetryDataV1 payload;
};
#pragma pack(pop)

static constexpr uint8_t LORA_PKT_TELEMETRY = 0x01;
static constexpr uint8_t LORA_PKT_ACK       = 0x02;

static constexpr uint8_t TELEMETRY_HAS_GPS = 1U << 0;
static constexpr uint8_t TELEMETRY_HAS_ENV = 1U << 1;
static constexpr uint8_t TELEMETRY_HAS_RETURN_SNR = 1U << 2;

inline bool telemetryHas(const telemetryData &data, uint8_t flag) {
    return (data.available & flag) != 0;
}

inline double gpsLatitudeDegrees(const gpsData &data) {
    return static_cast<double>(data.latitudeNanodegrees) /
           static_cast<double>(GPS_NANODEGREES_PER_DEGREE);
}

inline double gpsLongitudeDegrees(const gpsData &data) {
    return static_cast<double>(data.longitudeNanodegrees) /
           static_cast<double>(GPS_NANODEGREES_PER_DEGREE);
}

inline bool decodeLoraTelemetryPacket(const char *buffer, int packetSize,
                                      loraTelemetryPacket *out) {
    if (packetSize == static_cast<int>(sizeof(loraTelemetryPacket))) {
        memcpy(out, buffer, sizeof(loraTelemetryPacket));
        return out->type == LORA_PKT_TELEMETRY;
    }

    if (packetSize == static_cast<int>(sizeof(nanodegreeLoraTelemetryPacketV1))) {
        nanodegreeLoraTelemetryPacketV1 previous = {};
        memcpy(&previous, buffer, sizeof(previous));
        if (previous.type != LORA_PKT_TELEMETRY) {
            return false;
        }

        *out = {};
        out->type = previous.type;
        out->seq = previous.seq;
        out->snr = previous.snr;
        out->batt = previous.batt;
        out->payload.available = previous.payload.available;
        out->payload.gps.latitudeNanodegrees =
            previous.payload.gps.latitudeNanodegrees;
        out->payload.gps.longitudeNanodegrees =
            previous.payload.gps.longitudeNanodegrees;
        out->payload.gps.altitude = previous.payload.gps.altitude;
        out->payload.gps.speed = previous.payload.gps.speed;
        out->payload.gps.sats = previous.payload.gps.sats;
        out->payload.gps.quality = 0;
        memcpy(&out->payload.env, &previous.payload.env, sizeof(envData));
        return true;
    }

    if (packetSize == static_cast<int>(sizeof(legacyLoraTelemetryPacket))) {
        legacyLoraTelemetryPacket legacy = {};
        memcpy(&legacy, buffer, sizeof(legacy));
        if (legacy.type != LORA_PKT_TELEMETRY) {
            return false;
        }

        *out = {};
        out->type = legacy.type;
        out->seq = legacy.seq;
        out->snr = legacy.snr;
        out->batt = legacy.batt;
        out->payload.available = legacy.payload.available;
        if (!std::isfinite(legacy.payload.gps.latitude) || !std::isfinite(legacy.payload.gps.longitude) ||
            std::fabs(legacy.payload.gps.latitude) > 90 || std::fabs(legacy.payload.gps.longitude) > 180) return false;
        out->payload.gps.latitudeNanodegrees =
            static_cast<int64_t>(std::llround(
                static_cast<double>(legacy.payload.gps.latitude) *
                GPS_NANODEGREES_PER_DEGREE));
        out->payload.gps.longitudeNanodegrees =
            static_cast<int64_t>(std::llround(
                static_cast<double>(legacy.payload.gps.longitude) *
                GPS_NANODEGREES_PER_DEGREE));
        out->payload.gps.altitude = legacy.payload.gps.altitude;
        out->payload.gps.speed = legacy.payload.gps.speed;
        out->payload.gps.sats = static_cast<uint8_t>(
            legacy.payload.gps.sats < 0 ? 0 :
            legacy.payload.gps.sats > 255 ? 255 : legacy.payload.gps.sats);
        out->payload.gps.quality = 0;
        memcpy(&out->payload.env, &legacy.payload.env, sizeof(envData));
        return true;
    }

    // Original LoRa-Mesh single-sensor packets (13-byte header).
    if (packetSize == 33 || packetSize == 37) {
        const uint8_t type = static_cast<uint8_t>(buffer[0]);
        if ((packetSize == 33 && type == 1) || (packetSize == 37 && type == 3)) {
            *out = {};
            memcpy(out, buffer, 13);
            out->type = LORA_PKT_TELEMETRY;
            if (type == 3) {
                out->payload.available = TELEMETRY_HAS_ENV;
                memcpy(&out->payload.env, buffer + 13, sizeof(envData));
            } else {
                legacyGpsData gps = {};
                memcpy(&gps, buffer + 13, sizeof(gps));
                if (!std::isfinite(gps.latitude) || !std::isfinite(gps.longitude) ||
                    std::fabs(gps.latitude) > 90 || std::fabs(gps.longitude) > 180) return false;
                out->payload.available = TELEMETRY_HAS_GPS;
                out->payload.gps = {
                    static_cast<int64_t>(std::llround(gps.latitude * static_cast<double>(GPS_NANODEGREES_PER_DEGREE))),
                    static_cast<int64_t>(std::llround(gps.longitude * static_cast<double>(GPS_NANODEGREES_PER_DEGREE))),
                    gps.altitude, gps.speed,
                    static_cast<uint8_t>(gps.sats < 0 ? 0 : gps.sats > 255 ? 255 : gps.sats), 0
                };
            }
            return true;
        }
    }
    return false;
}

inline bool decodeLoraStatusPacket(const char *buffer, int packetSize, loraStatus *out) {
    if (packetSize != static_cast<int>(sizeof(loraStatus))) {
        return false;
    }
    memcpy(out, buffer, sizeof(loraStatus));
    return out->type == LORA_PKT_ACK;
}

static_assert(sizeof(gpsData) == 26, "gpsData size changed");
static_assert(sizeof(envData) == 24, "envData size changed");
static_assert(sizeof(telemetryData) == 51, "telemetryData size changed");
static_assert(sizeof(loraStatus) == 13, "loraStatus size changed");
static_assert(sizeof(loraTelemetryPacket) == 64, "loraTelemetryPacket size changed");
static_assert(sizeof(loraTelemetryPacket) <= 64, "telemetry packet exceeds LoRa buffer");
static_assert(sizeof(nanodegreeGpsDataV1) == 25,
              "previous nanodegree gpsData size changed");
static_assert(sizeof(nanodegreeLoraTelemetryPacketV1) == 63,
              "previous nanodegree telemetry packet size changed");
static_assert(sizeof(legacyGpsData) == 20, "legacyGpsData size changed");
static_assert(sizeof(legacyLoraTelemetryPacket) == 58,
              "legacy telemetry packet size changed");
