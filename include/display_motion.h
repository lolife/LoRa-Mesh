#pragma once

#include <cmath>
#include <cstdint>

// Acceleration is measured in g, including gravity. Comparing the vector also
// detects a change in orientation without waking on ordinary stationary noise.
constexpr float DISPLAY_MOTION_THRESHOLD_G = 0.12f;
constexpr uint32_t DISPLAY_IDLE_TIMEOUT_MS = 30000;
constexpr uint32_t DISPLAY_MOTION_SAMPLE_MS = 50;

class DisplayMotion {
public:
    bool update(uint32_t now, bool valid, float x, float y, float z) {
        if (valid && std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            if (!hasBaseline_) {
                setBaseline(x, y, z);
                hasBaseline_ = true;
            } else {
                const float dx = x - x_, dy = y - y_, dz = z - z_;
                if (dx * dx + dy * dy + dz * dz >=
                    DISPLAY_MOTION_THRESHOLD_G * DISPLAY_MOTION_THRESHOLD_G) {
                    setBaseline(x, y, z);
                    lastMotion_ = now;
                    awake_ = true;
                }
            }
        }
        if (awake_ && uint32_t(now - lastMotion_) >= DISPLAY_IDLE_TIMEOUT_MS) {
            awake_ = false;
        }
        return awake_;
    }

private:
    void setBaseline(float x, float y, float z) { x_ = x; y_ = y; z_ = z; }
    float x_ = 0, y_ = 0, z_ = 0;
    uint32_t lastMotion_ = 0;
    bool hasBaseline_ = false;
    bool awake_ = false;
};
