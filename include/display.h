#pragma once
#include <M5Unified.h>
#include <cstdint>

#pragma pack(push, 1) // Force alignment to 1 byte (no padding)
struct gpsData {
    float latitude;
    float longitude;
    float altitude;
    float speed;
};

struct loraStatus {
    uint8_t type;
    uint32_t seq;
    float snr;
    int32_t batt;
};

struct loraLocationPacket {
    uint8_t type;
    uint32_t seq;
    float snr;
    int32_t batt;
    gpsData location;
};

#pragma pack(pop) // Restore default alignment

static constexpr uint8_t LORA_PKT_LOCATION = 0x01;
static constexpr uint8_t LORA_PKT_ACK      = 0x02;

static_assert(sizeof(gpsData) == 16, "gpsData size changed");
static_assert(sizeof(loraStatus) == 13, "loraStatus size changed");
static_assert(sizeof(loraLocationPacket) == 29, "loraLocationPacket size changed");

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
void updateDisplay( loraLocationPacket newPkt, bool isSender);

bool nearlyZero( double valueToCheck );
bool locationInBounds( gpsData newLocation );
