#pragma once
#include <M5Unified.h>
#include "lora_protocol.h"

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
void updateDisplay(const loraGpsPacket &newPkt, bool isSender);
void updateDisplay(const loraEnvPacket &newPkt, bool isSender);
