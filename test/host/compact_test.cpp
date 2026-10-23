#include <cassert>
#include <cstdio>
#include <limits>
#include "lora_protocol.h"
#define LIGHT_SENSOR 1
#include "sensor_config.h"
#include "gps_processing.h"

int main() {
    static_assert(ENABLE_GPS && LIGHT_SENSOR, "GPS is enabled alongside light by default");
    loraTelemetryPacket in = {LORA_PKT_COMPACT_TELEMETRY, 123, -9.5f, 87, {}};
    in.payload.available = TELEMETRY_HAS_GPS | TELEMETRY_HAS_ENV |
        TELEMETRY_HAS_ALTITUDE | TELEMETRY_HAS_RETURN_SNR;
    in.payload.gps = {44894010000LL, -93477170000LL, -123.25f, 0, 0, 0};
    in.payload.env = {22.25f, 53.5f, 1008.75f, 12, 13, 14};
    setLightReading(in.payload, 64000);
    assert(telemetryHas(in.payload, TELEMETRY_HAS_GPS));
    assert(in.payload.gps.latitudeNanodegrees == 44894010000LL);
    CompactTelemetryPacket wire;
    assert(encodeCompactTelemetryPacket(in, &wire));
    assert(sizeof(wire) == 60 && wire.lightLux == 64000);
    loraTelemetryPacket out;
    const auto decode = [&](int n = sizeof(wire)) {
        return decodeLocalTelemetryPacket(reinterpret_cast<const char*>(&wire), n, &out);
    };
    assert(decode());
    assert(out.type == LORA_PKT_COMPACT_TELEMETRY && out.seq == 123 && out.batt == 87);
    assert(out.snr == -9.5f && out.payload.available == in.payload.available);
    assert(out.payload.gps.latitudeNanodegrees == in.payload.gps.latitudeNanodegrees);
    assert(out.payload.gps.longitudeNanodegrees == in.payload.gps.longitudeNanodegrees);
    assert(packetHasAltitude(out) && out.payload.gps.altitude == -123.25f);
    assert(!packetHasSpeedAndSatellites(out) && lightReading(out.payload) == 64000);
    assert(memcmp(&out.payload.env, &in.payload.env, sizeof(envData)) == 0);
    assert(!decode(59) && !decode(61) && !decode(64) && !decode(0));
    assert(!decodeLoraTelemetryPacket(reinterpret_cast<const char*>(&wire), sizeof(wire), &out));
    wire.latitudeNanodegrees = 90 * GPS_NANODEGREES_PER_DEGREE;
    wire.longitudeNanodegrees = -180 * GPS_NANODEGREES_PER_DEGREE;
    assert(decode());
    ++wire.latitudeNanodegrees; assert(!decode());
    wire.latitudeNanodegrees = 0; --wire.longitudeNanodegrees; assert(!decode());
    wire.longitudeNanodegrees = 0; assert(decode() && locationInBounds(out.payload.gps)); // Equator/prime meridian valid.
    wire.altitude = std::numeric_limits<float>::quiet_NaN(); assert(!decode());
    wire.altitude = std::numeric_limits<float>::infinity(); assert(!decode());
    wire.available &= ~TELEMETRY_HAS_ALTITUDE;
    assert(decode() && !packetHasAltitude(out)); // Altitude unavailable, position usable.
    wire.altitude = 0; wire.available |= TELEMETRY_HAS_ALTITUDE;
    wire.available &= ~TELEMETRY_HAS_GPS; assert(!decode());
    wire.available |= TELEMETRY_HAS_GPS;
    wire.lightLux = 64001; assert(!decode());
    wire.lightLux = 0; assert(decode() && lightReading(out.payload) == 0);
    wire.available |= 1U << 5; assert(!decode());
    // Every sensor combination, including light absent and GPS absent.
    for (uint8_t mask = 0; mask < 8; ++mask) {
        in.payload.available = (mask & 1 ? TELEMETRY_HAS_GPS : 0) |
            (mask & 2 ? TELEMETRY_HAS_ENV : 0) | (mask & 4 ? TELEMETRY_HAS_LIGHT : 0);
        if (mask & 1) in.payload.available |= TELEMETRY_HAS_ALTITUDE;
        in.payload.gps.latitudeNanodegrees = 44894010000LL;
        in.payload.gps.longitudeNanodegrees = -93477170000LL;
        assert(encodeCompactTelemetryPacket(in, &wire) && decode());
        assert(out.payload.available == in.payload.available);
        if (!(mask & 1)) assert(wire.latitudeNanodegrees == 0 && wire.altitude == 0);
        if (!(mask & 4)) assert(wire.lightLux == 0);
    }
    // Legacy full-GPS format retains its metadata semantics.
    in.type = LORA_PKT_TELEMETRY;
    in.payload.available = TELEMETRY_HAS_GPS;
    in.payload.gps.speed = 15;
    in.payload.gps.sats = 9;
    assert(decodeLocalTelemetryPacket(reinterpret_cast<const char*>(&in), sizeof(in), &out));
    assert(packetHasSpeedAndSatellites(out) && packetHasAltitude(out));
    puts("PASS compact: 60-byte combined packets, coordinates/altitude/lux/ENV/battery, flags, missing sensors, invalid packets, legacy GPS");
}
