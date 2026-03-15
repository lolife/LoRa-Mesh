#include "display.h"
#include "M5_SX127X.h"

extern uint16_t screenColor;
extern loraStatus newStatus;

void displayMessage(const char* msg, bool clearScreen, uint16_t bgColor) {
    if (clearScreen) {
        M5.Display.clear(bgColor);
    }
    M5.Display.setTextColor(TFT_WHITE, bgColor);
    centerCursor(&FreeSansBold18pt7b, 1, msg);
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

void updateDisplay(loraEnvPacket newPkt, bool isSender) {
    static char msg[32];
    envData env = newPkt.payload;

    M5.Display.setFont(&FreeSansBold18pt7b);
    M5.Display.setTextColor(TFT_WHITE, screenColor);
    M5.Display.setCursor(0, 10);

    M5.Display.clear(screenColor);
    snprintf(msg, sizeof(msg), "%.1f C", env.temperature);
    M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(msg)) / 2, (M5.Display.height() - M5.Display.fontHeight()) / 4);
    M5.Display.print( msg );

    if (isSender) {
        newPkt.snr = newStatus.snr;
        newPkt.batt = newStatus.batt;
    }

    snprintf( msg, sizeof( msg ), "SNR: %.0f / %.0f", newPkt.snr, LoRa.packetSnr() );
    M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(msg)) / 2, M5.Display.getCursorY()+M5.Display.fontHeight()+10 );
    M5.Display.print( msg );

    snprintf( msg, sizeof( msg ), "Batt: %d%% / %d%%", newPkt.batt, M5.Power.getBatteryLevel() );
    M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(msg)) / 2, M5.Display.getCursorY()+M5.Display.fontHeight()  );
    M5.Display.print( msg );

    snprintf(msg, sizeof(msg), "Hum: %.1f%%", env.humidity);
    M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(msg)) / 2, M5.Display.getCursorY()+M5.Display.fontHeight());
    M5.Display.print(msg);

    snprintf(msg, sizeof(msg), "Pres: %.1f hPa", env.pressure);
    M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(msg)) / 2, M5.Display.getCursorY()+M5.Display.fontHeight()  );
    M5.Display.print( msg );
}
