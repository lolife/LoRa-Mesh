#pragma once
#include <cstdint>

// LTR-553ALS-WA: 1x gain, 100 ms integration, 200 ms measurement period.
// Registers: https://docs.m5stack.com/en/core/CoreS3 (linked LTR553 datasheet).
// Two-channel lux formula: https://esphome.io/components/sensor/ltr_als_ps/
// Bus is M5.In_I2C; templating also allows exercising real register logic on host.
template<class Bus> class Ltr553 {
public:
    explicit Ltr553(Bus &bus) : bus(bus) {}
    bool begin() {
        uint8_t id[2] = {};
        return read(0x86, id, 2) && (id[0] & 0xf0) == 0x90 && id[1] == 0x05 &&
               write(0x80, 0x00) && // ALS standby while configuring
               write(0x81, 0x00) && // Proximity emitter stays off
               write(0x85, 0x02) && // 100 ms integration / 200 ms repeat
               write(0x80, 0x01);   // ALS active, gain 1x
    }
    bool sample(uint16_t &lux) {
        uint8_t status = 0, data[4] = {};
        if (!read(0x8c, &status, 1) || !(status & 0x04)) return false;
        // Read CH1 then CH0 in one burst, even for invalid data, to consume it.
        if (!read(0x88, data, 4) || (status & 0xf0)) return false;
        const uint16_t ch1 = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
        const uint16_t ch0 = uint16_t(data[2]) | (uint16_t(data[3]) << 8);
        // Full-scale channels cannot provide a trustworthy lux estimate.
        if (ch0 == 65535 || ch1 == 65535) return false;
        const uint32_t sum = uint32_t(ch0) + ch1;
        if (!sum) { lux = 0; return true; }
        const float ratio = float(ch1) / sum;
        float value = 0;
        if (ratio < 0.45f) value = 1.7743f * ch0 + 1.1059f * ch1;
        else if (ratio < 0.64f) value = 4.2785f * ch0 - 1.9548f * ch1;
        else if (ratio < 0.85f) value = 0.5926f * ch0 + 0.1185f * ch1;
        if (value < 0 || value > 64000) return false;
        lux = static_cast<uint16_t>(value + 0.5f);
        return true;
    }
private:
    Bus &bus;
    bool read(uint8_t reg, uint8_t *data, unsigned length) {
        return bus.readRegister(0x23, reg, data, length, 100000);
    }
    bool write(uint8_t reg, uint8_t value) {
        return bus.writeRegister8(0x23, reg, value, 100000);
    }
};
