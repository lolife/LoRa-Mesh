#include "photo_transfer.h"
#if ENABLE_PHOTOS
#include "photo_protocol.h"
#include "photo_freshness.h"
#include "display.h"
#include <M5Unified.h>
#include <M5_SX127X.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <time.h>
#ifdef SENDER
#include "photo_camera.h"
#endif

// Transport stays in main.cpp/lora_config.h; do not include main.h here because
// that legacy header defines application globals.
bool sendPacket(char* payload, int length);
int handlePacket();
void serviceBackgroundTasks();
void logRadioRxDiagnostics();

namespace {
bool redraw = false;
bool sendRaw(uint8_t* bytes, size_t n) {
    return sendPacket(reinterpret_cast<char*>(bytes), int(n));
}

#ifdef SENDER
CapturedPhoto sending;
photo::Ack receivedAck{};
bool ackReceived = false;
uint16_t stage = photo::START_INDEX;
uint32_t nextCaptureAt = 0, txStartedAt = 0, retries = 0;
unsigned lastProgress = 0;
photo::Status lastExchangeStatus = photo::Status::invalid;
uint32_t nextStartAttempt = 0;
bool startDelayed = false;
static_assert(PHOTO_INTERVAL_MS < 0x80000000UL, "Photo interval must be under 24.8 days");

bool exchange(uint8_t* packet, size_t n, uint16_t index) {
    lastExchangeStatus = photo::Status::invalid;
    for (unsigned attempt = 0; attempt < PHOTO_ATTEMPTS; ++attempt) {
        ackReceived = false;
        if (attempt) ++retries;
        if (!sendRaw(packet, n)) {
            log_w("Photo %08lx TX failed: stage %u, attempt %u/%u",
                  (unsigned long)sending.info.id, index, attempt + 1, PHOTO_ATTEMPTS);
            continue;
        }
        const uint32_t started = millis();
        while (uint32_t(millis() - started) < PHOTO_ACK_TIMEOUT_MS) {
            // Read before background work so a pending ACK is not left waiting.
            handlePacket();
            if (ackReceived && photo::matches(receivedAck, sending.info.id, index)) {
                lastExchangeStatus = receivedAck.status;
                if (receivedAck.status != photo::Status::ok)
                    log_e("Photo rejected at chunk %u: status %u", index, unsigned(receivedAck.status));
                return receivedAck.status == photo::Status::ok;
            }
            serviceBackgroundTasks();
            delay(5);
        }
        log_w("Photo %08lx ACK missing: stage %u, attempt %u/%u",
              (unsigned long)sending.info.id, index, attempt + 1, PHOTO_ATTEMPTS);
        logRadioRxDiagnostics();
    }
    log_e("Photo ACK timeout at chunk %u", index);
    return false;
}

void stopSending(bool success) {
    if (!success) {
        uint8_t cancel[photo::CONTROL_BYTES];
        photo::encodeControl(cancel, photo::CANCEL, sending.info.id);
        sendRaw(cancel, sizeof(cancel));
    }
    log_i("Photo %08lx %s: %lu bytes, %lu seconds, %lu retries",
          (unsigned long)sending.info.id, success ? "delivered" : "failed",
          (unsigned long)sending.info.length, (unsigned long)((millis() - txStartedAt) / 1000),
          (unsigned long)retries);
    free(sending.data);
    sending = {};
    // Continuous mode has no successful-transfer delay, but failed attempts
    // back off so a missing camera/receiver cannot cause a tight retry loop.
    if (!success && PHOTO_INTERVAL_MS == 0)
        nextCaptureAt = millis() + PHOTO_FAILURE_RETRY_MS;
}
#endif

#ifdef RECEIVER
photo::Assembly assembly;
uint8_t* incoming = nullptr;
uint8_t* displayed = nullptr;
photo::Info displayedInfo{};
uint32_t renderedPhotoId = 0;
uint32_t rxLastActivity = 0, rxStartedAt = 0;
unsigned lastProgress = 0;
bool imageDirty = true;
int renderedProgressWidth = 0;
constexpr int PROGRESS_BAR_HEIGHT = 2;
PhotoFreshness photoFreshness(PHOTO_OFFLINE_TIMEOUT_MS);
struct PhotoOverlay {
    bool offline = false;
    char temperature[24] = "--.- F";
    char humidity[24] = "--.- %";
    char clock[16] = "--:--";
};
PhotoOverlay displayedOverlay;

PhotoOverlay formatOverlay(const loraTelemetryPacket& pkt) {
    PhotoOverlay result;
    result.offline = photoFreshness.offline(millis());
    if (telemetryHas(pkt.payload, TELEMETRY_HAS_ENV)) {
        if (std::isfinite(pkt.payload.env.temperature)) {
            snprintf(result.temperature, sizeof(result.temperature), "%.1f F",
                     pkt.payload.env.temperature * 1.8f + 32.0f);
        }
        if (std::isfinite(pkt.payload.env.humidity)) {
            snprintf(result.humidity, sizeof(result.humidity), "%.1f %%", pkt.payload.env.humidity);
        }
    }
    const time_t now = time(nullptr);
    struct tm local{};
    // Do not block radio handling waiting for NTP, or show an unset clock.
    if (now >= 1704067200 && localtime_r(&now, &local)) {
        strftime(result.clock, sizeof(result.clock), "%I:%M %p", &local);
    }
    return result;
}

bool sameOverlay(const PhotoOverlay& a, const PhotoOverlay& b) {
    return a.offline == b.offline && strcmp(a.clock, b.clock) == 0 &&
           (a.offline || (strcmp(a.temperature, b.temperature) == 0 &&
                          strcmp(a.humidity, b.humidity) == 0));
}

void drawOverlay(const PhotoOverlay& overlay) {
    int textColor = TFT_LIGHTSKYBLUE;
    // Match DTunnel's local detail screen: large temperature, humidity, clock.
    M5.Display.setFont(&fonts::FreeSans18pt7b);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(lgfx::textdatum_t::top_left);
    constexpr int margin = 32;
    const int width = M5.Display.width() - 2 * margin;
    const int height = M5.Display.height() - 2 * margin;
    const int baseHeight = M5.Display.fontHeight();
    const int primaryHeight = height * 4 / 9;
    const int secondaryHeight = height / 3;
    const int clockHeight = height - primaryHeight - secondaryHeight;
    const int labelWidth = width / 3;
    const auto text = [&](const char* content, int x, int top, int rowHeight, int maxWidth) {
        M5.Display.setTextSize(1);
        float scale = float(rowHeight - 4) / baseHeight;
        const int naturalWidth = M5.Display.textWidth(content);
        if (naturalWidth > 0) scale = std::min(scale, float(maxWidth) / naturalWidth);
        M5.Display.setTextSize(scale);
        const int y = top + (rowHeight - M5.Display.fontHeight()) / 2;
        // Transparent yellow glyphs with a two-pixel black outline.
        M5.Display.setTextColor(TFT_BLACK);
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                if (dx || dy) M5.Display.drawString(content, x + dx, y + dy);
            }
        }
        M5.Display.setTextColor(textColor);
        M5.Display.drawString(content, x, y);
    };
    const char* labels[] = {"Temp", "Humidity", "Time"};
    const char* values[] = {overlay.temperature, overlay.humidity, overlay.clock};
    const int rowHeights[] = {primaryHeight, secondaryHeight, clockHeight};
    int y = margin;
    if (overlay.offline) {
        text("Offline", margin, margin, primaryHeight + secondaryHeight, width);
        y += primaryHeight + secondaryHeight;
    }
    for (int i = overlay.offline ? 2 : 0; i < 3; ++i) {
        const int labelHeight = i < 2 ? rowHeights[i] / 2 : rowHeights[i];
        text(labels[i], margin, y + (rowHeights[i] - labelHeight) / 2,
             labelHeight, labelWidth - 4);
        text(values[i], margin + labelWidth, y, rowHeights[i], width - labelWidth - 4);
        y += rowHeights[i];
    }
    M5.Display.setTextSize(1);
}

void discardIncoming() {
    free(incoming); incoming = nullptr; assembly = photo::Assembly{};
    redraw = true; // Remove the progress bar, restoring the previous image.
}
void reply(uint32_t id, uint16_t index, photo::Status status) {
    uint8_t bytes[photo::ACK_BYTES];
    photo::encodeAck(bytes, {id, index, status});
    const bool sent = sendRaw(bytes, sizeof(bytes));
    if (index == photo::START_INDEX || status != photo::Status::ok || !sent) {
        log_i("Photo %08lx ACK: stage %u, status %u, TX %s",
              (unsigned long)id, index, unsigned(status), sent ? "ok" : "failed");
    }
}
float displayScale(const photo::Info& i) {
    return std::min(float(M5.Display.width()) / i.width, float(M5.Display.height()) / i.height);
}
bool decodes(const uint8_t* jpeg, const photo::Info& i) {
    // Decode off-screen first so a bad image cannot replace the previous view.
    M5Canvas candidate(&M5.Display);
    candidate.setPsram(true);
    candidate.setColorDepth(16);
    if (!candidate.createSprite(M5.Display.width(), M5.Display.height())) return false;
    return candidate.drawJpg(jpeg, i.length, 0, 0, candidate.width(), candidate.height(), 0, 0, displayScale(i));
}

void receiveStart(const uint8_t* p, size_t n) {
    photo::Info info;
    if (!photo::decodeInfo(p, n, info)) return;
    if (incoming) {
        const bool duplicate = photo::same(assembly.info(), info);
        if (duplicate) rxLastActivity = millis();
        log_i("Photo %08lx start repeated: %s; received %u/%u chunks",
              (unsigned long)info.id, duplicate ? "same image" : "receiver busy",
              assembly.next(), photo::chunks(assembly.info()));
        reply(info.id, photo::START_INDEX, duplicate ? photo::Status::ok : photo::Status::busy);
        return;
    }
    incoming = static_cast<uint8_t*>(heap_caps_malloc(info.length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!incoming) { reply(info.id, photo::START_INDEX, photo::Status::noMemory); return; }
    assembly.begin(info, incoming, info.length);
    rxStartedAt = rxLastActivity = millis();
    lastProgress = 0;
    log_i("Receiving photo %08lx: %ux%u, %lu bytes, RSSI %d dBm, SNR %.1f dB",
          (unsigned long)info.id, info.width, info.height, (unsigned long)info.length,
          LoRa.packetRssi(), LoRa.packetSnr());
    reply(info.id, photo::START_INDEX, photo::Status::ok);
    redraw = true;
}
void receiveData(const uint8_t* p, size_t n) {
    if (n <= photo::DATA_HEADER_BYTES || n > photo::MAX_PACKET_BYTES) return;
    const uint32_t id = photo::get32(p + 2);
    const uint16_t index = photo::get16(p + 6);
    const auto status = assembly.accept(p, n);
    if (status == photo::Status::ok) {
        rxLastActivity = millis();
        redraw = true;
        const unsigned percent = uint32_t(assembly.next()) * 100 / photo::chunks(assembly.info());
        if (percent >= lastProgress + 10 || percent == 100) {
            lastProgress = percent;
            log_i("Receiving %lu KB: %u%%  %lus",
                     (unsigned long)((assembly.info().length + 1023) / 1024), percent,
                     (unsigned long)((millis() - rxStartedAt) / 1000));
        }
    }
    reply(id, index, status);
}
void receiveEnd(uint32_t id) {
    // A lost final ACK must not turn a completed transfer into a failure.
    if (displayed && id == displayedInfo.id) { reply(id, photo::END_INDEX, photo::Status::ok); return; }
    if (!incoming || id != assembly.info().id) { reply(id, photo::END_INDEX, photo::Status::invalid); return; }
    auto status = assembly.finish();
    if (status == photo::Status::ok && !decodes(incoming, assembly.info())) status = photo::Status::corrupt;
    if (status == photo::Status::ok) {
        free(displayed);
        displayed = incoming; displayedInfo = assembly.info(); incoming = nullptr;
        assembly = photo::Assembly{};
        log_i("Photo %08lx complete: %ux%u  %lu KB  transfer %lus", (unsigned long)id,
                 displayedInfo.width, displayedInfo.height,
                 (unsigned long)((displayedInfo.length + 1023) / 1024),
                 (unsigned long)((millis() - rxStartedAt) / 1000));
        imageDirty = true;
        redraw = true;
    } else {
        discardIncoming();
        log_e("Photo failed integrity/decode (%u)", unsigned(status));
    }
    reply(id, photo::END_INDEX, status);
    if (status == photo::Status::ok) wakeDisplay();
}
#endif
}

void initializePhotos() {
#ifdef SENDER
    nextCaptureAt = millis() + PHOTO_FIRST_CAPTURE_MS;
    if (PHOTO_INTERVAL_MS == 0) {
        log_i("Photos: JPEG quality %u; first in %lus, continuous back-to-back transfers",
              PHOTO_JPEG_QUALITY, PHOTO_FIRST_CAPTURE_MS / 1000UL);
    } else {
        log_i("Photos: JPEG quality %u; first in %lus, capture interval %lus (no overlapping transfers); tap sender to capture",
              PHOTO_JPEG_QUALITY, PHOTO_FIRST_CAPTURE_MS / 1000UL, PHOTO_INTERVAL_MS / 1000UL);
    }
#else
    photoFreshness.displayed(millis()); // Start the initial wait for the first photo.
    configTzTime("CST6CDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");
    log_i("Photo receiver ready; yellow outlined ENV and Central time overlay enabled");
#endif
}

bool handlePhotoPacket(const uint8_t* p, size_t n) {
    if (!p || !n || p[0] < photo::START || p[0] > photo::CANCEL) return false;
    if (n < 2 || p[1] != photo::VERSION) {
        log_w("Photo packet header rejected: type 0x%02x, %u bytes, version %u",
              p[0], unsigned(n), n >= 2 ? p[1] : 0);
        return true;
    }
#ifdef SENDER
    if (p[0] == photo::ACK) {
        photo::Ack ack{};
        if (photo::decodeAck(p, n, ack)) {
            receivedAck = ack;
            ackReceived = true;
            if (ack.index == photo::START_INDEX || ack.status != photo::Status::ok ||
                !photo::matches(ack, sending.info.id, stage)) {
                log_i("Photo ACK received: id %08lx stage %u status %u; expecting %08lx/%u",
                      (unsigned long)ack.id, ack.index, unsigned(ack.status),
                      (unsigned long)sending.info.id, stage);
            }
        } else {
            log_w("Photo ACK malformed: %u bytes", unsigned(n));
        }
    }
#endif
#ifdef RECEIVER
    switch (p[0]) {
    case photo::START: receiveStart(p, n); break;
    case photo::DATA: receiveData(p, n); break;
    case photo::END: if (n == photo::CONTROL_BYTES) receiveEnd(photo::get32(p + 2)); break;
    case photo::CANCEL:
        if (n == photo::CONTROL_BYTES && incoming && photo::get32(p + 2) == assembly.info().id) {
            discardIncoming(); log_w("Sender cancelled photo");
        }
        break;
    default: break;
    }
#endif
    return true;
}

bool servicePhotos() {
#ifdef SENDER
    if (!sending.data) {
        if (int32_t(millis() - nextCaptureAt) >= 0 || M5.Touch.getDetail().wasPressed()) {
            // Schedule from the capture attempt, so transmission time is part
            // of the interval. An overdue timer starts only one next photo.
            nextCaptureAt = millis() + PHOTO_INTERVAL_MS;
            if (capturePhoto(sending)) {
                stage = photo::START_INDEX; txStartedAt = millis(); retries = 0; lastProgress = 0;
                startDelayed = false;
            } else if (PHOTO_INTERVAL_MS == 0) {
                nextCaptureAt = millis() + PHOTO_FAILURE_RETRY_MS;
            }
        }
    } else {
        if (stage == photo::START_INDEX && startDelayed && int32_t(millis() - nextStartAttempt) < 0)
            return false;
        // One stop-and-wait exchange per loop, allowing existing telemetry
        // between chunks. ACK waits continue servicing GPS, light, OTA, motion.
        uint8_t packet[photo::MAX_PACKET_BYTES];
        size_t n;
        if (stage == photo::START_INDEX) { photo::encodeInfo(packet, sending.info); n = photo::START_BYTES; }
        else if (stage == photo::END_INDEX) { photo::encodeControl(packet, photo::END, sending.info.id); n = photo::CONTROL_BYTES; }
        else n = photo::encodeData(packet, sending.info, stage, sending.data);
        if (!exchange(packet, n, stage)) {
            // A rebooted sender may find its previous transfer still occupying
            // the receiver. Retain this capture while that session expires,
            // rather than throwing the new image away for another capture interval.
            if (stage == photo::START_INDEX && lastExchangeStatus == photo::Status::busy &&
                uint32_t(millis() - txStartedAt) < PHOTO_RX_IDLE_MS + 30000UL) {
                startDelayed = true;
                nextStartAttempt = millis() + 10000;
                log_w("Receiver busy; retrying photo start in 10 seconds");
            } else stopSending(false);
        }
        else if (stage == photo::END_INDEX) stopSending(true);
        else {
            stage = stage == photo::START_INDEX ? 0 : stage + 1;
            const unsigned percent = uint32_t(stage) * 100 / photo::chunks(sending.info);
            if (percent >= lastProgress + 10) {
                lastProgress = percent;
                log_i("Photo TX: %u%%, %lus, %lu retries", percent,
                      (unsigned long)((millis() - txStartedAt) / 1000), (unsigned long)retries);
            }
            if (stage == photo::chunks(sending.info)) stage = photo::END_INDEX;
        }
    }
#endif
#ifdef RECEIVER
    static uint32_t lastClockCheck = 0;
    if (uint32_t(millis() - lastClockCheck) >= 1000) {
        lastClockCheck = millis();
        redraw = true;
    }
    if (incoming && uint32_t(millis() - rxLastActivity) >= PHOTO_RX_IDLE_MS) {
        log_w("Photo %08lx stalled: received %u/%u chunks",
              (unsigned long)assembly.info().id, assembly.next(), photo::chunks(assembly.info()));
        discardIncoming();
        log_w("Photo timed out; keeping last image");
    }
#endif
    const bool result = redraw; redraw = false;
    return result;
}

bool drawPhotoView(bool isSender, const loraTelemetryPacket& pkt) {
#ifdef RECEIVER
    if (isSender) return false;
    if (!isDisplayAwake()) return true;
    PhotoOverlay overlay = formatOverlay(pkt);
    const uint32_t receivedBytes = incoming
        ? std::min(uint32_t(assembly.next()) * uint32_t(photo::CHUNK_BYTES), assembly.info().length)
        : 0;
    const int progressWidth = incoming
        ? int(uint64_t(receivedBytes) * M5.Display.width() / assembly.info().length)
        : 0;
    const bool repaintView = imageDirty || !sameOverlay(overlay, displayedOverlay) ||
                             progressWidth < renderedProgressWidth;
    if (repaintView) {
        M5.Display.fillScreen(TFT_BLACK);
        if (displayed) {
            const float scale = displayScale(displayedInfo);
            const int x = (M5.Display.width() - int(displayedInfo.width * scale)) / 2;
            const int y = (M5.Display.height() - int(displayedInfo.height * scale)) / 2;
            if (!M5.Display.drawJpg(displayed, displayedInfo.length, x, y, M5.Display.width(), M5.Display.height(), 0, 0, scale)) {
                imageDirty = true;
                return true;
            }
            if (renderedPhotoId != displayedInfo.id) {
                renderedPhotoId = displayedInfo.id;
                photoFreshness.displayed(millis());
                overlay = formatOverlay(pkt);
            }
        }
        if (displayed || overlay.offline) drawOverlay(overlay);
        displayedOverlay = overlay;
        imageDirty = false;
        renderedProgressWidth = 0;
    }
    // Paint only newly received progress after ACK handling. Do not decode the
    // entire JPEG on each chunk, or count this as a newly displayed photo.
    if (progressWidth > renderedProgressWidth) {
        M5.Display.fillRect(renderedProgressWidth, M5.Display.height() - PROGRESS_BAR_HEIGHT,
                            progressWidth - renderedProgressWidth, PROGRESS_BAR_HEIGHT, TFT_RED);
        renderedProgressWidth = progressWidth;
    }
    return true;
#else
    (void)isSender;
    (void)pkt;
    return false;
#endif
}
#endif
