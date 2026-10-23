#include <cassert>
#include <cstdio>
#include "light_sensor.h"
#include "ltr553.h"
#include <vector>
#include <utility>
#include <cstring>
#include "lora_protocol.h"

struct FakeLightBus {
    uint8_t regs[256] = {};
    bool failRead = false, failWrite = false;
    std::vector<std::pair<uint8_t, uint8_t>> writes;
    bool readRegister(uint8_t address, uint8_t reg, uint8_t *data, unsigned size, uint32_t freq) {
        assert(address == 0x23 && freq == 100000);
        if (failRead) return false;
        std::memcpy(data, regs + reg, size);
        if (reg == 0x88) { assert(size == 4); regs[0x8c] &= ~0x04; }
        return true;
    }
    bool writeRegister8(uint8_t address, uint8_t reg, uint8_t value, uint32_t freq) {
        assert(address == 0x23 && freq == 100000);
        if (failWrite) return false;
        writes.emplace_back(reg, value);
        regs[reg] = value;
        return true;
    }
    void measurement(uint16_t ch0, uint16_t ch1, uint8_t status = 0x04) {
        regs[0x88] = ch1; regs[0x89] = ch1 >> 8;
        regs[0x8a] = ch0; regs[0x8b] = ch0 >> 8;
        regs[0x8c] = status;
    }
};

int main() {
    FakeLightBus bus;
    Ltr553<FakeLightBus> sensor(bus);
    assert(!sensor.begin()); // Wrong device is not treated as a light sensor.
    bus.regs[0x86] = 0x92; bus.regs[0x87] = 0x05;
    bus.failRead = true; assert(!sensor.begin()); bus.failRead = false;
    bus.failWrite = true; assert(!sensor.begin()); bus.failWrite = false;
    assert(sensor.begin());
    const std::vector<std::pair<uint8_t, uint8_t>> expected = {
        {0x80, 0}, {0x81, 0}, {0x85, 2}, {0x80, 1}};
    assert(bus.writes == expected);
    uint16_t lux = 123;
    bus.measurement(0, 0); assert(sensor.sample(lux) && lux == 0);
    assert(!sensor.sample(lux)); // Do not count the same conversion twice.
    bus.measurement(1000, 100); assert(sensor.sample(lux) && lux == 1885);
    bus.measurement(500, 500); assert(sensor.sample(lux) && lux == 1162);
    bus.measurement(300, 700); assert(sensor.sample(lux) && lux == 261);
    bus.measurement(100, 900); assert(sensor.sample(lux) && lux == 0);
    bus.measurement(30000, 0); assert(sensor.sample(lux) && lux == 53229);
    bus.measurement(65535, 0); assert(!sensor.sample(lux));
    bus.measurement(0, 65535); assert(!sensor.sample(lux));
    bus.measurement(40000, 0); assert(!sensor.sample(lux)); // Out of specified range.
    bus.measurement(1000, 100, 0x84); assert(!sensor.sample(lux)); // Invalid.
    assert(!(bus.regs[0x8c] & 4)); // Invalid conversion was consumed.
    bus.measurement(1000, 100, 0x14); assert(!sensor.sample(lux)); // Unexpected gain.
    bus.measurement(1000, 100); bus.failRead = true; assert(!sensor.sample(lux));
    bus.failRead = false; assert(sensor.sample(lux) && lux == 1885);

    LightSamples samples;
    assert(!samples.available(0));
    for (int i = 0; i < 7; ++i) samples.add(0, 100 * i);
    assert(!samples.available(600));
    samples.add(0, 700);
    assert(samples.available(700) && samples.lux() == 0); // Real darkness is valid.
    samples.add(64001, 5000); // Invalid readings must not refresh availability.
    assert(!samples.available(5701));
    samples.add(64000, 5800);
    assert(!samples.available(5800)); // Refill the window after an outage.
    for (int i = 0; i < 7; ++i) samples.add(64000, 5900 + 100 * i);
    assert(samples.available(6500) && samples.lux() == 64000);
    samples.add(0, 6600);
    assert(samples.lux() == 56000);
    LightSamples rollover;
    for (int i = 0; i < 8; ++i) rollover.add(42, 0xffffff00U + i);
    assert(rollover.available(0x100) && rollover.lux() == 42);
    assert(!rollover.available(0x2000));

    // Previous type-5 firmware stored lux at the start of GPS storage.
    loraTelemetryPacket packet = {LORA_PKT_LIGHT_TELEMETRY, 42, -7, 90, {}};
    packet.payload.available = TELEMETRY_HAS_ENV | TELEMETRY_HAS_LIGHT;
    packet.payload.env = {21, 50, 1013, 0, 0, 0};
    uint16_t oldLux = 2048;
    memcpy(&packet.payload.gps, &oldLux, sizeof(oldLux));
    loraTelemetryPacket decoded = {};
    const auto decode = [&](int size = sizeof(packet)) {
        return decodeLocalTelemetryPacket(reinterpret_cast<const char*>(&packet), size, &decoded);
    };
    assert(sizeof(packet) == 64 && decode());
    assert(decoded.seq == 42 && lightReading(decoded.payload) == 2048);
    assert(decoded.payload.env.temperature == 21);
    assert(!decode(63) && !decode(0));
    packet.payload.available |= TELEMETRY_HAS_GPS;
    assert(!decode());
    packet.payload.available &= ~TELEMETRY_HAS_GPS;
    oldLux = 64001;
    memcpy(&packet.payload.gps, &oldLux, sizeof(oldLux));
    assert(!decode());
    oldLux = 0;
    memcpy(&packet.payload.gps, &oldLux, sizeof(oldLux));
    assert(decode() && lightReading(decoded.payload) == 0);
    packet.type = 4;
    assert(!decode()); // ADC counts must never be labelled lux.
    packet.type = LORA_PKT_LIGHT_TELEMETRY;
    packet.payload.available &= ~TELEMETRY_HAS_LIGHT;
    assert(!decode());
    packet.type = LORA_PKT_TELEMETRY;
    assert(decode());
    setLightReading(packet.payload, 12);
    assert(!decode()); // Only local formats can carry light data.
    puts("PASS light: LTR553 registers/lux/status/errors, smoothing, expiry/recovery, rollover, legacy lux packets");
}
