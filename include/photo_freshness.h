#pragma once
#include <cstdint>

// Only a newly displayed image resets this deadline, not telemetry, chunks,
// retries, duplicate completion packets, or redraws of the previous image.
class PhotoFreshness {
public:
    explicit PhotoFreshness(uint32_t timeoutMs) : timeoutMs_(timeoutMs) {}
    void displayed(uint32_t now) { lastDisplayed_ = now; offline_ = false; }
    bool offline(uint32_t now) {
        if (uint32_t(now - lastDisplayed_) >= timeoutMs_) offline_ = true;
        return offline_;
    }
private:
    uint32_t timeoutMs_;
    uint32_t lastDisplayed_ = 0;
    bool offline_ = false;
};
