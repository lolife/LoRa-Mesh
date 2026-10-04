#include "display.h"
#include "display_motion.h"
#include <cstdio>

extern uint16_t screenColor;
extern loraStatus newStatus;

namespace {
DisplayMotion motion;
bool displayAwake = false;
uint32_t lastMotionSample = 0;
constexpr uint8_t DISPLAY_AWAKE_BRIGHTNESS = 80;
}

bool isDisplayAwake() { return displayAwake; }

void initializeDisplayMotion() {
    M5.Display.setBrightness(0);
    M5.Display.sleep();
    motion = DisplayMotion{};
    displayAwake = false;
    lastMotionSample = millis() - DISPLAY_MOTION_SAMPLE_MS;
    serviceDisplayMotion(); // Establish the initial resting orientation.
    if (!M5.Imu.isEnabled()) {
        log_w("Display motion wake unavailable: internal IMU not detected");
    }
}

bool serviceDisplayMotion() {
    const uint32_t now = millis();
    if (uint32_t(now - lastMotionSample) < DISPLAY_MOTION_SAMPLE_MS) return false;
    lastMotionSample = now;
    float x = 0, y = 0, z = 0;
    const bool valid = M5.Imu.isEnabled() && M5.Imu.getAccel(&x, &y, &z);
    const bool awake = motion.update(now, valid, x, y, z);
    if (awake == displayAwake) return false;
    displayAwake = awake;
    if (awake) {
        M5.Display.wakeup();
        M5.Display.setBrightness(DISPLAY_AWAKE_BRIGHTNESS);
    } else {
        M5.Display.setBrightness(0);
        M5.Display.sleep();
    }
    return awake;
}

void displayMessage(const char* msg, bool clearScreen, uint16_t bgColor) {
    if (!isDisplayAwake()) return;
    if (clearScreen) {
        M5.Display.clear(bgColor);
    }
    M5.Display.setTextColor(TFT_WHITE, bgColor);
    centerCursor(&fonts::FreeSansBold18pt7b, 1, msg);
    M5.Display.print(msg);
}

void centerCursor(const lgfx::GFXfont* font, int size, const char* text) {
    M5.Display.setFont(font);
    M5.Display.setTextSize(size);
    int textWidth = M5.Display.textWidth(text);
    int textHeight = M5.Display.fontHeight();
    M5.Display.setCursor((M5.Display.width() - textWidth) / 2, 
                         (M5.Display.height() - textHeight) / 2);
}

void updateDisplay(const loraTelemetryPacket &pkt, bool isSender,
                   float localSnr, bool localSnrValid) {
    if (!isDisplayAwake()) return;
    static char msg[64];

    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextColor(TFT_WHITE, screenColor);
    M5.Display.clear(screenColor);
    int y = M5.Display.fontHeight();
    const auto printLine = [&](const char *text) {
        M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(text)) / 2, y);
        M5.Display.print(text);
        y += M5.Display.fontHeight();
    };

    if (telemetryHas(pkt.payload, TELEMETRY_HAS_GPS)) {
        snprintf(msg, sizeof(msg), "Lat %.9f",
                 gpsLatitudeDegrees(pkt.payload.gps));
        printLine(msg);
        snprintf(msg, sizeof(msg), "Lon %.9f",
                 gpsLongitudeDegrees(pkt.payload.gps));
        printLine(msg);
        snprintf(msg, sizeof(msg), "Alt %.0f m  %.1f mph",
                 pkt.payload.gps.altitude, pkt.payload.gps.speed);
        printLine(msg);
        snprintf(msg, sizeof(msg), "%u sats", static_cast<unsigned>(pkt.payload.gps.sats));
        printLine(msg);
    }

    if (telemetryHas(pkt.payload, TELEMETRY_HAS_ENV)) {
        snprintf(msg, sizeof(msg), "%.1f C  %.1f%% RH",
                 pkt.payload.env.temperature, pkt.payload.env.humidity);
        printLine(msg);
        snprintf(msg, sizeof(msg), "%.1f hPa", pkt.payload.env.pressure);
        printLine(msg);
    }

    if (!telemetryHas(pkt.payload, TELEMETRY_HAS_GPS) &&
        !telemetryHas(pkt.payload, TELEMETRY_HAS_ENV)) {
        printLine("No telemetry");
    }

    float uplinkSnr = 0.0f;
    float downlinkSnr = 0.0f;
    bool uplinkValid = false;
    bool downlinkValid = false;
    if (isSender) {
        const bool currentAck = pkt.seq != 0 && newStatus.seq == pkt.seq;
        uplinkSnr = newStatus.snr;
        uplinkValid = currentAck;
        downlinkSnr = localSnr;
        downlinkValid = currentAck && localSnrValid;
    } else {
        uplinkSnr = localSnr;
        uplinkValid = localSnrValid;
        downlinkSnr = pkt.snr;
        downlinkValid = telemetryHas(pkt.payload, TELEMETRY_HAS_RETURN_SNR);
    }

    char uplinkText[12] = "--";
    char downlinkText[12] = "--";
    if (uplinkValid) {
        snprintf(uplinkText, sizeof(uplinkText), "%+.0f", uplinkSnr);
    }
    if (downlinkValid) {
        snprintf(downlinkText, sizeof(downlinkText), "%+.0f", downlinkSnr);
    }
    snprintf(msg, sizeof(msg), "SNR U/D %s / %s", uplinkText, downlinkText);
    printLine(msg);

    const int remoteBattery = isSender ? newStatus.batt : pkt.batt;
    snprintf(msg, sizeof(msg), "Batt %d%% / %d%%",
             remoteBattery, M5.Power.getBatteryLevel());
    printLine(msg);
}
