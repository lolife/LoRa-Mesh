#pragma once

// Local, point-to-point photo experiment. Both endpoints must enable photos.
#ifndef ENABLE_PHOTOS
#define ENABLE_PHOTOS 1
#endif
#ifndef PHOTO_INTERVAL_MS
#define PHOTO_INTERVAL_MS (15UL * 1000UL) // Capture every 15 seconds, without overlap.
#endif
#ifndef PHOTO_FIRST_CAPTURE_MS
#define PHOTO_FIRST_CAPTURE_MS 15000UL
#endif
/*    
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_QVGA,     // 320x240
 */
#ifndef PHOTO_FRAME_SIZE
#define PHOTO_FRAME_SIZE FRAMESIZE_QQVGA 
#endif
// Software encoder scale: 1..100, HIGHER is better (unlike camera.jpeg_quality).
#ifndef PHOTO_JPEG_QUALITY
#define PHOTO_JPEG_QUALITY 25
#endif
// GC0308 sensor API scale 0..30; the driver's default is approximately 9.
// Fixed gain prevents AGC from immediately undoing this low-light adjustment.
#ifndef PHOTO_SENSOR_GAIN
#define PHOTO_SENSOR_GAIN 20
#endif
// Automatic-exposure target adjustment: -2..+2, positive is brighter.
#ifndef PHOTO_AE_LEVEL
#define PHOTO_AE_LEVEL 1
#endif
#ifndef PHOTO_SETTLE_MS
#define PHOTO_SETTLE_MS 2000UL
#endif
#ifndef PHOTO_DISPLAY_BRIGHTNESS
#define PHOTO_DISPLAY_BRIGHTNESS 255
#endif
static_assert(PHOTO_DISPLAY_BRIGHTNESS >= 1 && PHOTO_DISPLAY_BRIGHTNESS <= 255,
              "Photo receiver brightness must be 1..255");

constexpr unsigned long PHOTO_ACK_TIMEOUT_MS = 2500;
constexpr unsigned PHOTO_ATTEMPTS = 4;
constexpr unsigned long PHOTO_RX_IDLE_MS = 60000;
constexpr unsigned long PHOTO_FAILURE_RETRY_MS = 5000;

// Continuous capture has no interval; use the normal 15-second cadence for
// its offline deadline instead of treating every image as immediately stale.
constexpr unsigned long PHOTO_OFFLINE_INTERVAL_MS = PHOTO_INTERVAL_MS ? PHOTO_INTERVAL_MS : 15000UL;
static_assert(PHOTO_OFFLINE_INTERVAL_MS <= 0x7fffffffUL / 4,
              "Photo offline deadline must be below half the millis rollover period");
constexpr unsigned long PHOTO_OFFLINE_TIMEOUT_MS = 4UL * PHOTO_OFFLINE_INTERVAL_MS;
