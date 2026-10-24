#include "photo_config.h"
#if ENABLE_PHOTOS && defined(SENDER)
#include "photo_camera.h"
#include <M5Unified.h>
#include <esp_camera.h>
#include <img_converters.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <algorithm>

static_assert(PHOTO_JPEG_QUALITY >= 1 && PHOTO_JPEG_QUALITY <= 100, "JPEG quality must be 1..100");
static_assert(PHOTO_SENSOR_GAIN >= 0 && PHOTO_SENSOR_GAIN <= 30, "Sensor gain must be 0..30");
static_assert(PHOTO_AE_LEVEL >= -2 && PHOTO_AE_LEVEL <= 2, "AE target must be -2..2");

namespace {
struct JpegOutput { uint8_t* bytes; size_t length = 0; bool overflow = false; };
size_t writeJpeg(void* arg, size_t index, const void* data, size_t n) {
    auto& out = *static_cast<JpegOutput*>(arg);
    if (index > photo::MAX_IMAGE_BYTES || n > photo::MAX_IMAGE_BYTES - index) {
        out.overflow = true;
        return 0;
    }
    if (n) memcpy(out.bytes + index, data, n);
    out.length = std::max(out.length, index + n);
    return n;
}

bool configureExposure() {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor) return false;
    // Configure through the GC0308 driver: brightness and gain-ceiling setters
    // are unsupported for this sensor. Keep exposure and white balance automatic,
    // but use an explicit higher gain and a brighter AE luminance target.
    const int agc = sensor->set_gain_ctrl(sensor, 0);
    const int gain = sensor->set_agc_gain(sensor, PHOTO_SENSOR_GAIN);
    const int aec = sensor->set_exposure_ctrl(sensor, 1);
    const int target = sensor->set_ae_level(sensor, PHOTO_AE_LEVEL);
    const int awb = sensor->set_whitebal(sensor, 1);
    if (agc || gain || aec || target || awb) {
        log_e("Camera exposure configuration failed: agc=%d gain=%d aec=%d target=%d awb=%d",
              agc, gain, aec, target, awb);
        return false;
    }
    log_i("Camera: fixed gain %d/30, auto exposure target %+d, settling %lums",
          PHOTO_SENSOR_GAIN, PHOTO_AE_LEVEL, PHOTO_SETTLE_MS);
    return true;
}
}

bool capturePhoto(CapturedPhoto& out) {
    out = {};
    if (M5.getBoard() != m5::board_t::board_M5StackCoreS3 || !psramFound()) {
        log_e("Photos need a CoreS3 with camera and working PSRAM");
        return false;
    }
    uint8_t* jpeg = static_cast<uint8_t*>(heap_caps_malloc(photo::MAX_IMAGE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!jpeg) { log_e("No PSRAM for photo encoder"); return false; }

    camera_config_t cfg = {};
    cfg.pin_pwdn = cfg.pin_reset = cfg.pin_xclk = -1;
    cfg.pin_sccb_sda = 12; cfg.pin_sccb_scl = 11;
    cfg.pin_d0 = 39; cfg.pin_d1 = 40; cfg.pin_d2 = 41; cfg.pin_d3 = 42;
    cfg.pin_d4 = 15; cfg.pin_d5 = 16; cfg.pin_d6 = 48; cfg.pin_d7 = 47;
    cfg.pin_vsync = 46; cfg.pin_href = 38; cfg.pin_pclk = 45;
    cfg.xclk_freq_hz = 20000000;
    cfg.ledc_timer = LEDC_TIMER_0; cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.pixel_format = PIXFORMAT_RGB565;
    cfg.frame_size = PHOTO_FRAME_SIZE;
    cfg.fb_count = 1; cfg.fb_location = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    cfg.sccb_i2c_port = -1;

    // M5GFX and the camera SCCB driver use different I2C implementations.
    // Hand the internal bus over ONLY during capture; restore it before any
    // touch, IMU, power, or LTR553 service (and leave external ENV I2C alone).
    if (!M5.In_I2C.release()) { free(jpeg); return false; }
    const esp_err_t err = esp_camera_init(&cfg);
    bool success = false;
    if (err == ESP_OK) {
        camera_fb_t* frame = nullptr;
        if (configureExposure()) {
            // Consume fresh frames for a full settling period after each camera
            // reinitialization rather than relying on just four fast startup frames.
            const uint32_t start = millis();
            unsigned frames = 0;
            do {
                if (frame) esp_camera_fb_return(frame);
                frame = esp_camera_fb_get();
                if (!frame) break;
                ++frames;
            } while (frames < 4 || uint32_t(millis() - start) < PHOTO_SETTLE_MS);
        }
        if (frame) {
            JpegOutput encoded{jpeg};
            success = frame2jpg_cb(frame, PHOTO_JPEG_QUALITY, writeJpeg, &encoded) &&
                !encoded.overflow && encoded.length >= 4;
            if (success) {
                uint32_t id = esp_random();
                if (!id) id = 1;
                out.info = {id, uint32_t(encoded.length), photo::crc32(jpeg, encoded.length),
                            uint16_t(frame->width), uint16_t(frame->height)};
                success = photo::valid(out.info);
            }
            esp_camera_fb_return(frame);
        }
        esp_camera_deinit();
    } else {
        log_e("Camera init failed: %s", esp_err_to_name(err));
    }
    if (!M5.In_I2C.begin()) { log_e("Internal I2C restore failed"); success = false; }
    if (!success) { free(jpeg); log_e("Photo capture/encoding failed"); return false; }
    out.data = jpeg;
    log_i("Photo %08lx captured: %ux%u, JPEG quality %u, %lu bytes, %u chunks",
          (unsigned long)out.info.id, out.info.width, out.info.height, PHOTO_JPEG_QUALITY,
          (unsigned long)out.info.length, photo::chunks(out.info));
    return true;
}
#endif
