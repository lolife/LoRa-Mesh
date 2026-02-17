#include "display.h"
#include "M5_SX127X.h"

void displayMessage(const char* msg, bool clearScreen, uint16_t bgColor) {
    if (clearScreen) {
        M5.Display.clear(bgColor);
    }
    M5.Display.setTextColor(TFT_SILVER, bgColor);
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

void updateDisplay( gpsData location, int counter, bool isSender) {
    M5.Display.clear();
    M5.Display.setFont(&Roboto_Thin_24);
    M5.Display.setTextColor(TFT_SILVER, TFT_NAVY);
    M5.Display.setCursor(0, 10);

    M5.Display.printf( " Lat: %.6f\n Lon: %.6f\n Alt: %.0f\n Spd: %.2f", location.latitude, location.longitude, location.altitude, location.speed ) ;
    if (! isSender) {
        M5.Display.printf("\n SNR: %.1f\n f Err: %d\n RSSI: %d\n Batt: %d%%",
                              LoRa.packetSnr(), 
                              LoRa.packetFrequencyError(), 
                              LoRa.rssi(), 
                              M5.Power.getBatteryLevel() );
    }
}
