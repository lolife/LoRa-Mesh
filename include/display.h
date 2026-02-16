#pragma once
#include <M5Unified.h>

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
 * @param counter Packet counter (for sender)
 * @param isSender True if sender, false if receiver
 */
void updateDisplay(uint16_t screenColor, int counter, bool isSender);

/**
 * Convert RGB565 color to individual RGB components
 * @param color RGB565 color value
 * @param r Output red component (0-255)
 * @param g Output green component (0-255)
 * @param b Output blue component (0-255)
 */
void rgb565ToRGB(uint16_t color, int& r, int& g, int& b);

/**
 * Calculate perceived luminance from RGB values
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return Luminance value (0-255)
 */
int calculateLuminance(int r, int g, int b);
