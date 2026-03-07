#pragma once
#include <M5Unified.h>

#pragma pack(push, 1) // Force alignment to 1 byte (no padding)
struct gpsData {
    double latitude;
    double longitude;
    float altitude;
    float speed;
};

struct loraStatus {
    int code;
    float snr;
    int batt;
};

#pragma pack(pop) // Restore default alignment

/**
 * Display a centered message on the screen
 * @param msg Message to display
 * @param clearScreen Whether to clear the screen first
 * @param bgColor Background color for the display
 */
void displayMessage(const char* msg, bool clearScreen, uint16_t bgColor);

/**
 * Center the cursor for text drawing
 * @param font Font to use
 * @param size Text size
 * @param text Text to be drawn (used for width calculation)
 */
void centerCursor(const lgfx::GFXfont* font, int size, const char* text);

/**
 * Update display with current screen color and stats
 * @param screenColor Current screen color in RGB565 format
 * @param isSender True if sender, false if receiver
 */
void updateDisplay( gpsData location, bool isSender);

bool nearlyZero( double valueToCheck );
bool locationInBounds( gpsData newLocation );