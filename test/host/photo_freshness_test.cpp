#include "photo_freshness.h"
#include <cassert>
#include <cstdint>
#include <cstdio>

int main() {
    PhotoFreshness view(60000);
    // No first image: allow four intervals, then show Offline.
    view.displayed(3000);
    assert(!view.offline(62999));
    assert(view.offline(63000));
    // Incoming traffic and redraws do not call displayed(): stay offline.
    assert(view.offline(90000));
    // A new successfully rendered image immediately recovers the view.
    view.displayed(100000);
    assert(!view.offline(100000));
    assert(!view.offline(159999));
    assert(view.offline(160000));
    // A new image just before expiry starts a full new deadline.
    view.displayed(200000);
    view.displayed(259999);
    assert(!view.offline(260000));
    assert(view.offline(319999));
    // Millis rollover must neither cause premature expiry nor restore a stale view.
    const uint32_t start = UINT32_MAX - 1000;
    view.displayed(start);
    assert(!view.offline(start + 59999));
    assert(view.offline(start + 60000));
    assert(view.offline(start + 1)); // Remain latched even after another full wrap.
    view.displayed(500);
    assert(!view.offline(501));
    puts("PASS photo freshness: initial deadline, stale view, recovery, refresh, rollover");
}
