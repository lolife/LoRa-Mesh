#include "display.h"
#include "M5_SX127X.h"

extern uint16_t screenColor;

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

void updateDisplay( gpsData location, bool isSender) {
    static char msg[32];

    M5.Display.clear(screenColor);
    M5.Display.setFont(&FreeSansBold18pt7b);
    M5.Display.setTextColor(TFT_WHITE, screenColor);
    M5.Display.setCursor(0, 10);

    if( ! locationInBounds( location ) ) {
        M5.Display.setTextColor(TFT_RED, screenColor);
        M5.Display.printf( "No Position" );
        M5.Display.setTextColor(TFT_WHITE, screenColor);
    }
    else {
        if (isSender) {
            snprintf( msg, sizeof( msg ), "%.2f mph", location.speed );
            M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(msg)) / 2, (M5.Display.height() - M5.Display.fontHeight()) / 2);
            M5.Display.print( msg );

            snprintf( msg, sizeof( msg ), "Batt: %d%%", M5.Power.getBatteryLevel() );
            M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(msg)) / 2, M5.Display.height()*3/4 );
            M5.Display.print( msg );
        }
        else {
            snprintf( msg, sizeof( msg ), "%.2f mph", location.speed );
            M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(msg)) / 2, M5.Display.height()/4 );
            M5.Display.print( msg );

            snprintf( msg, sizeof( msg ), "SNR: %.0f Batt: %d%%", LoRa.packetSnr(), M5.Power.getBatteryLevel() );
            M5.Display.setCursor((M5.Display.width() - M5.Display.textWidth(msg)) / 2, M5.Display.height()*3/4 );
            M5.Display.print( msg );
        }
    }
}

bool nearlyZero( double valueToCheck ) {
    const double EPSILON = 1e-9;  // or whatever tolerance makes sense for your use case

    if( std::abs(valueToCheck) < EPSILON )
        return true;
    return false;
}

bool locationInBounds( gpsData newLocation ) {
    if( nearlyZero(newLocation.latitude) || newLocation.latitude > 90.0 || newLocation.latitude < -90.0 ||
        nearlyZero(newLocation.longitude) || newLocation.longitude > 180.0 || newLocation.longitude < -180.0 ) {
            return false;
    }
    return true;
}