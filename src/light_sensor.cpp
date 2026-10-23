#include "sensor_config.h"
#if defined(SENDER) && LIGHT_SENSOR
#include "light_sensor.h"
#include "ltr553.h"
#include <M5Unified.h>

namespace {
Ltr553<m5::I2C_Class> sensor(m5::In_I2C);
LightSamples samples;
uint32_t lastAttempt = 0, lastInit = 0, initializedAt = 0;
bool ready = false;
}

bool initializeLightSensor() {
    samples = LightSamples{};
    lastInit = millis();
    ready = sensor.begin();
    initializedAt = millis();
    if (ready) log_i("Internal LTR553 ready (1x gain, 100 ms integration, lux)");
    else log_e("Internal LTR553 initialization failed");
    return ready;
}

void serviceLightSensor() {
    const uint32_t now = millis();
    if (!ready) {
        if (uint32_t(now - lastInit) >= 30000) initializeLightSensor();
        return;
    }
    if (uint32_t(now - initializedAt) < 250 || uint32_t(now - lastAttempt) < 100) return;
    lastAttempt = now;
    uint16_t lux = 0;
    if (sensor.sample(lux)) samples.add(lux, now);
    // Failed I2C, invalid/saturated and old data never refresh the expiry timer.
    // Re-probe/reconfigure after a prolonged outage, including sensor resets.
    if (!samples.available(now) && uint32_t(now - lastInit) >= 30000)
        initializeLightSensor();
}

bool currentLightReading(uint16_t &lux) {
    if (!samples.available(millis())) return false;
    lux = samples.lux();
    return true;
}
#endif
