#include <cassert>
#include <initializer_list>
#include <cmath>
#include <cstdio>
#include <limits>
#include "lora_protocol.h"
#include "espnow_protocol.h"

int main() {
    loraTelemetryPacket combined = {LORA_PKT_TELEMETRY, 42, -7, 90, {}};
    combined.payload.available = TELEMETRY_HAS_GPS | TELEMETRY_HAS_ENV;
    combined.payload.gps = {44894010000LL, -93477170000LL, 304, 12, 9, 0};
    combined.payload.env = {21, 50, 1013, 0, 0, 0};
    loraTelemetryPacket decoded = {};
    assert(decodeLoraTelemetryPacket(reinterpret_cast<const char*>(&combined), sizeof(combined), &decoded));
    assert(decoded.seq == 42 && telemetryHas(decoded.payload, TELEMETRY_HAS_GPS));
    assert(telemetryHas(decoded.payload, TELEMETRY_HAS_ENV));
    assert(decoded.payload.env.pressure == 1013);
    assert(std::fabs(gpsLatitudeDegrees(decoded.payload.gps) - 44.89401) < 1e-8);
    for (uint8_t available : {uint8_t(0), TELEMETRY_HAS_GPS, TELEMETRY_HAS_ENV}) {
        combined.payload.available = available;
        assert(decodeLoraTelemetryPacket(reinterpret_cast<const char*>(&combined), sizeof(combined), &decoded));
        assert(decoded.payload.available == available);
    }
    assert(!decodeLoraTelemetryPacket(reinterpret_cast<const char*>(&combined), 12, &decoded));
    combined.type = 99;
    assert(!decodeLoraTelemetryPacket(reinterpret_cast<const char*>(&combined), sizeof(combined), &decoded));
    legacyLoraTelemetryPacket previous = {1, 13, -5, 60, {3, {44.9f, -93.4f, 300, 4, 8}, {22, 45, 1010, 0, 0, 0}}};
    assert(decodeLoraTelemetryPacket(reinterpret_cast<const char*>(&previous), sizeof(previous), &decoded));
    assert(decoded.seq == 13 && decoded.payload.gps.sats == 8 && decoded.payload.env.temperature == 22);
    previous.payload.gps.latitude = std::numeric_limits<float>::quiet_NaN();
    assert(!decodeLoraTelemetryPacket(reinterpret_cast<const char*>(&previous), sizeof(previous), &decoded));
    // The original standalone packets use the same packed 13-byte header.
    char gpsPacket[33] = {};
    loraStatus header = {1, 77, -6, 30};
    legacyGpsData gps = {44.9f, -93.4f, 300, 4, 8};
    memcpy(gpsPacket, &header, 13); memcpy(gpsPacket + 13, &gps, 20);
    assert(decodeLoraTelemetryPacket(gpsPacket, sizeof(gpsPacket), &decoded));
    assert(decoded.seq == 77 && decoded.payload.available == TELEMETRY_HAS_GPS);
    char envPacket[37] = {};
    header.type = 3;
    envData env = {22, 45, 1010, 0, 0, 0};
    memcpy(envPacket, &header, 13); memcpy(envPacket + 13, &env, 24);
    assert(decodeLoraTelemetryPacket(envPacket, sizeof(envPacket), &decoded));
    assert(decoded.payload.available == TELEMETRY_HAS_ENV && decoded.payload.env.temperature == 22);
    loraStatus ack = {LORA_PKT_ACK, 77, -3, 80}, decodedAck = {};
    assert(decodeLoraStatusPacket(reinterpret_cast<const char*>(&ack), sizeof(ack), &decodedAck));
    assert(decodedAck.seq == decoded.seq);
    assert(!decodeLoraStatusPacket(reinterpret_cast<const char*>(&ack), sizeof(ack) - 1, &decodedAck));
    puts("PASS telemetry: combined/GPS-only/ENV-only/empty, legacy packets, malformed data, ACK sequence");
}
