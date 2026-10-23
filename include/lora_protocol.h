#pragma once
#include <mesh_protocol/lora.h>

// Local extension: shared MeshProtocol structures and ESP-NOW stay unchanged.
static constexpr uint8_t LORA_PKT_LIGHT_TELEMETRY = 5; // Previous lux-only format.
static constexpr uint8_t LORA_PKT_COMPACT_TELEMETRY = 6;
static constexpr uint8_t TELEMETRY_HAS_ALTITUDE = 1U << 6;
static constexpr uint8_t TELEMETRY_HAS_LIGHT = 1U << 7;

// In-memory adapter only: compact packets omit satellites/quality, so their two
// bytes hold lux. Do not send this expanded 64-byte representation on the radio.
inline void setLightReading(telemetryData &data, uint16_t lux) {
    data.gps.sats = static_cast<uint8_t>(lux);
    data.gps.quality = static_cast<uint8_t>(lux >> 8);
    data.available |= TELEMETRY_HAS_LIGHT;
}
inline uint16_t lightReading(const telemetryData &data) {
    return uint16_t(data.gps.sats) | (uint16_t(data.gps.quality) << 8);
}

#pragma pack(push, 1)
struct CompactTelemetryPacket {
    uint8_t type;
    uint32_t seq;
    float snr;
    int32_t batt;
    uint8_t available;
    int64_t latitudeNanodegrees;
    int64_t longitudeNanodegrees;
    float altitude; // GPS meters; present only with TELEMETRY_HAS_ALTITUDE.
    uint16_t lightLux;
    envData env;
};
#pragma pack(pop)
static_assert(sizeof(CompactTelemetryPacket) == 60, "compact telemetry must be 60 bytes");
static_assert(sizeof(CompactTelemetryPacket) <= 64, "LoRa packet budget exceeded");
inline bool validCompactTelemetry(const CompactTelemetryPacket &p) {
    constexpr uint8_t allowed = TELEMETRY_HAS_GPS | TELEMETRY_HAS_ENV |
        TELEMETRY_HAS_RETURN_SNR | TELEMETRY_HAS_ALTITUDE | TELEMETRY_HAS_LIGHT;
    if (p.type != LORA_PKT_COMPACT_TELEMETRY || (p.available & ~allowed)) return false;
    if (p.available & TELEMETRY_HAS_GPS) {
        if (p.latitudeNanodegrees < -90 * GPS_NANODEGREES_PER_DEGREE ||
            p.latitudeNanodegrees > 90 * GPS_NANODEGREES_PER_DEGREE ||
            p.longitudeNanodegrees < -180 * GPS_NANODEGREES_PER_DEGREE ||
            p.longitudeNanodegrees > 180 * GPS_NANODEGREES_PER_DEGREE) return false;
    }
    if (p.available & TELEMETRY_HAS_ALTITUDE) {
        if (!(p.available & TELEMETRY_HAS_GPS) || !std::isfinite(p.altitude)) return false;
    }
    return !(p.available & TELEMETRY_HAS_LIGHT) || p.lightLux <= 64000;
}

inline bool encodeCompactTelemetryPacket(const loraTelemetryPacket &in,
                                         CompactTelemetryPacket *out) {
    CompactTelemetryPacket p = {};
    p.type = LORA_PKT_COMPACT_TELEMETRY;
    p.seq = in.seq; p.snr = in.snr; p.batt = in.batt;
    p.available = in.payload.available;
    if (telemetryHas(in.payload, TELEMETRY_HAS_GPS)) {
        p.latitudeNanodegrees = in.payload.gps.latitudeNanodegrees;
        p.longitudeNanodegrees = in.payload.gps.longitudeNanodegrees;
    }
    if (telemetryHas(in.payload, TELEMETRY_HAS_ALTITUDE)) p.altitude = in.payload.gps.altitude;
    if (telemetryHas(in.payload, TELEMETRY_HAS_LIGHT)) p.lightLux = lightReading(in.payload);
    if (telemetryHas(in.payload, TELEMETRY_HAS_ENV)) p.env = in.payload.env;
    if (!validCompactTelemetry(p)) return false;
    *out = p;
    return true;
}

inline bool packetHasAltitude(const loraTelemetryPacket &p) {
    return telemetryHas(p.payload, TELEMETRY_HAS_GPS) &&
        (p.type != LORA_PKT_COMPACT_TELEMETRY || telemetryHas(p.payload, TELEMETRY_HAS_ALTITUDE)) &&
        std::isfinite(p.payload.gps.altitude);
}
inline bool packetHasSpeedAndSatellites(const loraTelemetryPacket &p) {
    return p.type != LORA_PKT_COMPACT_TELEMETRY && telemetryHas(p.payload, TELEMETRY_HAS_GPS);
}

inline bool decodeLocalTelemetryPacket(const char *buffer, int size,
                                       loraTelemetryPacket *out) {
    if (size <= 0) return false;
    const uint8_t type = static_cast<uint8_t>(buffer[0]);
    if (type == LORA_PKT_COMPACT_TELEMETRY) {
        if (size != sizeof(CompactTelemetryPacket)) return false;
        CompactTelemetryPacket p;
        memcpy(&p, buffer, sizeof(p));
        if (!validCompactTelemetry(p)) return false;
        loraTelemetryPacket decoded = {p.type, p.seq, p.snr, p.batt, {}};
        decoded.payload.available = p.available;
        if (p.available & TELEMETRY_HAS_GPS) {
            decoded.payload.gps.latitudeNanodegrees = p.latitudeNanodegrees;
            decoded.payload.gps.longitudeNanodegrees = p.longitudeNanodegrees;
        }
        if (p.available & TELEMETRY_HAS_ALTITUDE) decoded.payload.gps.altitude = p.altitude;
        if (p.available & TELEMETRY_HAS_LIGHT) setLightReading(decoded.payload, p.lightLux);
        if (p.available & TELEMETRY_HAS_ENV) decoded.payload.env = p.env;
        *out = decoded;
        return true;
    }
    if (type == LORA_PKT_LIGHT_TELEMETRY) {
        if (size != sizeof(*out)) return false;
        loraTelemetryPacket packet;
        memcpy(&packet, buffer, sizeof(packet));
        constexpr uint8_t allowed = TELEMETRY_HAS_LIGHT | TELEMETRY_HAS_ENV |
                                    TELEMETRY_HAS_RETURN_SNR;
        uint16_t lux = 0;
        memcpy(&lux, &packet.payload.gps, sizeof(lux));
        if (!(packet.payload.available & TELEMETRY_HAS_LIGHT) ||
            (packet.payload.available & ~allowed) || lux > 64000) return false;
        packet.payload.gps = {};
        setLightReading(packet.payload, lux);
        *out = packet;
        return true;
    }
    if (!decodeLoraTelemetryPacket(buffer, size, out)) return false;
    return !(out->payload.available & (TELEMETRY_HAS_LIGHT | TELEMETRY_HAS_ALTITUDE));
}
