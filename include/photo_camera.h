#pragma once
#include "photo_protocol.h"

struct CapturedPhoto {
    uint8_t* data = nullptr; // heap_caps allocation; caller frees after transfer
    photo::Info info{};
};
bool capturePhoto(CapturedPhoto& out);
