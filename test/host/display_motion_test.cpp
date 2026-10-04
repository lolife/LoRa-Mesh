#include "display_motion.h"
#include <cassert>
#include <limits>

int main() {
    DisplayMotion motion;
    assert(!motion.update(0, true, 0, 0, 1));
    assert(!motion.update(50, true, 0.01f, -0.01f, 1.01f));
    // Small successive tilts accumulate against the resting orientation.
    assert(!motion.update(100, true, 0.06f, 0, 1));
    assert(motion.update(150, true, 0.13f, 0, 1));
    assert(motion.update(30149, true, 0.13f, 0, 1));
    assert(!motion.update(30150, true, 0.13f, 0, 1));
    assert(motion.update(30200, true, 0, 0, 1));
    // Later motion extends the timer, even if the unit subsequently stays put.
    assert(motion.update(50000, true, 0, 0.2f, 1));
    assert(motion.update(79999, false, 0, 0, 0));
    assert(!motion.update(80000, false, 0, 0, 0));
    assert(!motion.update(80050, true, std::numeric_limits<float>::quiet_NaN(), 0, 0));
    assert(!motion.update(80100, false, 1, 1, 1));
    assert(motion.update(80150, true, 0, 0, 1));

    DisplayMotion rollover;
    const uint32_t start = UINT32_MAX - 1000;
    assert(!rollover.update(start, true, 0, 0, 1));
    assert(rollover.update(start + 50, true, 0.2f, 0, 1));
    assert(rollover.update(start + 30049, true, 0.2f, 0, 1));
    assert(!rollover.update(start + 30050, true, 0.2f, 0, 1));
}
