#pragma once
#include <cstdint>

// Average eight successful whole-lux samples. Sensor errors never mean dark.
class LightSamples {
public:
    void add(uint16_t lux, uint32_t now) {
        if (lux > 64000) return;
        if (count && uint32_t(now - lastSuccess) > 5000) reset();
        sum -= values[index];
        values[index] = lux;
        sum += lux;
        index = (index + 1) % 8;
        if (count < 8) ++count;
        lastSuccess = now;
    }
    bool available(uint32_t now) const {
        return count == 8 && uint32_t(now - lastSuccess) <= 5000;
    }
    uint16_t lux() const { return static_cast<uint16_t>((sum + 4) / 8); }
private:
    void reset() { *this = LightSamples{}; }
    uint16_t values[8] = {};
    uint32_t sum = 0, lastSuccess = 0;
    uint8_t index = 0, count = 0;
};

bool initializeLightSensor();
void serviceLightSensor();
bool currentLightReading(uint16_t &lux);
