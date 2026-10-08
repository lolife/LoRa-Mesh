#pragma once

#include <lgfx/boards.hpp>

// Canonical enum names from M5GFX's lgfx::boards::board_t. Aliases share
// an ID and resolve to the canonical name. Add cases when M5GFX adds boards.
inline const char* boardName(lgfx::boards::board_t board)
{
    switch (board) {
#define BOARD_NAME_CASE(name) case lgfx::boards::name: return #name
        BOARD_NAME_CASE(board_unknown);
        BOARD_NAME_CASE(board_M5Stack);
        BOARD_NAME_CASE(board_M5StackCore2);
        BOARD_NAME_CASE(board_M5StickC);
        BOARD_NAME_CASE(board_M5StickCPlus);
        BOARD_NAME_CASE(board_M5StickCPlus2);
        BOARD_NAME_CASE(board_M5StackCoreInk);
        BOARD_NAME_CASE(board_M5Paper);
        BOARD_NAME_CASE(board_M5Tough);
        BOARD_NAME_CASE(board_M5Station);
        BOARD_NAME_CASE(board_M5StackCoreS3);
        BOARD_NAME_CASE(board_M5AtomS3);
        BOARD_NAME_CASE(board_M5Dial);
        BOARD_NAME_CASE(board_M5DinMeter);
        BOARD_NAME_CASE(board_M5Cardputer);
        BOARD_NAME_CASE(board_M5AirQ);
        BOARD_NAME_CASE(board_M5VAMeter);
        BOARD_NAME_CASE(board_M5StackCoreS3SE);
        BOARD_NAME_CASE(board_M5AtomS3R);
        BOARD_NAME_CASE(board_M5PaperS3);
        BOARD_NAME_CASE(board_M5CoreMP135);
        BOARD_NAME_CASE(board_M5StampPLC);
        BOARD_NAME_CASE(board_M5Tab5);
        BOARD_NAME_CASE(board_ArduinoNessoN1);
        BOARD_NAME_CASE(board_M5CardputerADV);
        BOARD_NAME_CASE(board_M5UnitC6L);
        BOARD_NAME_CASE(board_M5StickS3);
        BOARD_NAME_CASE(board_M5StackChan);
        BOARD_NAME_CASE(board_M5PaperColor);
        BOARD_NAME_CASE(board_M5PaperMono);
        BOARD_NAME_CASE(board_M5StopWatch);
        BOARD_NAME_CASE(board_M5CoreP4X);
        BOARD_NAME_CASE(board_M5ChainCaptain);
        BOARD_NAME_CASE(board_M5ToughC5);
        BOARD_NAME_CASE(board_M5PaperDIY);
        BOARD_NAME_CASE(board_M5Tab5X);
        BOARD_NAME_CASE(board_M5AtomLite);
        BOARD_NAME_CASE(board_M5AtomPsram);
        BOARD_NAME_CASE(board_M5AtomU);
        BOARD_NAME_CASE(board_M5Camera);
        BOARD_NAME_CASE(board_M5TimerCam);
        BOARD_NAME_CASE(board_M5StampPico);
        BOARD_NAME_CASE(board_M5StampC3);
        BOARD_NAME_CASE(board_M5StampC3U);
        BOARD_NAME_CASE(board_M5StampS3);
        BOARD_NAME_CASE(board_M5AtomS3Lite);
        BOARD_NAME_CASE(board_M5AtomS3U);
        BOARD_NAME_CASE(board_M5Capsule);
        BOARD_NAME_CASE(board_M5NanoC6);
        BOARD_NAME_CASE(board_M5AtomMatrix);
        BOARD_NAME_CASE(board_M5AtomVoice);
        BOARD_NAME_CASE(board_M5AtomS3RExt);
        BOARD_NAME_CASE(board_M5AtomS3RCam);
        BOARD_NAME_CASE(board_M5AtomVoiceS3R);
        BOARD_NAME_CASE(board_M5PowerHub);
        BOARD_NAME_CASE(board_M5DualKey);
        BOARD_NAME_CASE(board_M5UnitPoEP4);
        BOARD_NAME_CASE(board_M5StampS3Bat);
        BOARD_NAME_CASE(board_M5StampP4);
        BOARD_NAME_CASE(board_M5NanoH2);
        BOARD_NAME_CASE(board_M5CoreMatrix);
        BOARD_NAME_CASE(board_M5StampC5);
        BOARD_NAME_CASE(board_M5StampC6);
        BOARD_NAME_CASE(board_M5StampS3Mini);
        BOARD_NAME_CASE(board_M5StampP4X);
        BOARD_NAME_CASE(board_M5AtomDisplay);
        BOARD_NAME_CASE(board_M5UnitLCD);
        BOARD_NAME_CASE(board_M5UnitOLED);
        BOARD_NAME_CASE(board_M5UnitMiniOLED);
        BOARD_NAME_CASE(board_M5UnitGLASS);
        BOARD_NAME_CASE(board_M5UnitGLASS2);
        BOARD_NAME_CASE(board_M5UnitRCA);
        BOARD_NAME_CASE(board_M5ModuleDisplay);
        BOARD_NAME_CASE(board_M5ModuleRCA);
        BOARD_NAME_CASE(board_M5UnitPoEP4HDMI);
        BOARD_NAME_CASE(board_FrameBuffer);
#undef BOARD_NAME_CASE
        default: return "board_unknown";
    }
}
