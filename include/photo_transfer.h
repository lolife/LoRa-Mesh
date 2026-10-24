#pragma once
#include "photo_config.h"
#include "lora_protocol.h"
#include <cstddef>
#include <cstdint>

#if ENABLE_PHOTOS
void initializePhotos();
// Run once per main loop. True requests a display refresh.
bool servicePhotos();
bool handlePhotoPacket(const uint8_t* data, size_t length);
// True means the photo page owns the display (including when asleep).
bool drawPhotoView(bool isSender, const loraTelemetryPacket& pkt);
#else
inline void initializePhotos() {}
inline bool servicePhotos() { return false; }
inline bool handlePhotoPacket(const uint8_t*, size_t) { return false; }
inline bool drawPhotoView(bool, const loraTelemetryPacket&) { return false; }
#endif
