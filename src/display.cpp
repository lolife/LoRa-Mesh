#include "display.h"
#include "M5_SX127X.h"

void displayMessage(const char* msg, bool clearScreen, uint16_t bgColor) {
    if (clearScreen) {
        M5.Display.clear(bgColor);
    }
    M5.Display.setTextColor(TFT_SILVER, bgColor);
    centerCursor(&FreeSansBold18pt7b, 1, msg);
    M5.Display.print(msg);
    delay(1000);
}

void centerCursor(const lgfx::GFXfont* font, int size, const char* text) {
    M5.Display.setFont(font);
    M5.Display.setTextSize(size);
    int textWidth = M5.Display.textWidth(text);
    int textHeight = M5.Display.fontHeight();
    M5.Display.setCursor((M5.Display.width() - textWidth) / 2, 
                         (M5.Display.height() - textHeight) / 2);
}

void rgb565ToRGB(uint16_t color, int& r, int& g, int& b) {
    r = (color >> 11) & 0x1F;  // 5 bits
    g = (color >> 5) & 0x3F;   // 6 bits
    b = color & 0x1F;          // 5 bits
    
    // Scale back to 0-255 range
    r = (r * 255) / 31;
    g = (g * 255) / 63;
    b = (b * 255) / 31;
}

int calculateLuminance(int r, int g, int b) {
    return (int)(0.299 * r + 0.587 * g + 0.114 * b);
}

void updateDisplay(uint16_t screenColor, int counter, bool isSender) {
    int r, g, b;
    rgb565ToRGB(screenColor, r, g, b);
    
    // Calculate perceived luminance for text contrast
    int lum = calculateLuminance(r, g, b);
    uint16_t textColor = (lum > 128) ? TFT_BLACK : TFT_WHITE;

    M5.Display.clear(screenColor);
    M5.Display.setTextColor(textColor, screenColor);
    M5.Display.setCursor(0, 10);
    
    if (isSender) {
        // Sender display - show color values and packet count
        //M5.Display.printf(" R: %d\n G: %d\n B: %d\n Lum: %d\n Pkts: %d", r, g, b, lum, counter) ;
        ;
    } else {
        // Receiver display - show radio stats
        // Try to get battery level safely
        int battLevel = M5.Power.getBatteryLevel();
        
        if (battLevel >= 0) {
            M5.Display.printf(" SNR: %.1f\n f Err: %d\n RSSI: %d\n Batt: %d%%",
                              LoRa.packetSnr(), 
                              LoRa.packetFrequencyError(), 
                              LoRa.rssi(), 
                              battLevel);
        } else {
            M5.Display.printf(" SNR: %.1f\n f Err: %d\n RSSI: %d\n Batt: N/A",
                              LoRa.packetSnr(), 
                              LoRa.packetFrequencyError(), 
                              LoRa.rssi());
        }
    }
}
