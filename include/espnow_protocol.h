#pragma once

#include <cstdint>

// ESP-NOW wire contract shared by DTunnel, LoRa-Mesh and LoRa-Mesh-RAK. Keep this file
// byte-for-byte identical in all repositories.
struct StatusMessage {
  char deviceName[16];
  uint8_t deviceAddr[6];
  char varName[10];
  float varValue;
  uint16_t messageId;
  uint8_t relayCount;
  uint8_t version;
};

// Receive-only compatibility for devices that have not yet been upgraded.
struct LegacyStatusMessage {
  char deviceName[16];
  char varName[10];
  float varValue;
};

static constexpr uint8_t ESPNOW_STATUS_VERSION = 1;

static_assert(sizeof(StatusMessage) == 40, "StatusMessage wire size changed");
static_assert(sizeof(LegacyStatusMessage) == 32,
              "LegacyStatusMessage wire size changed");
