#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "espnow_protocol.h"
extern char TAG[36];

#include <esp_mac.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_wifi.h>

struct Peer {
  char name[16];
  uint8_t address[6];
  unsigned long lastHeard;
  StatusMessage currentData;
};

enum MeshNodeId : uint8_t {
  NODE_CORE1 = 0,
  NODE_CORE2,
  NODE_CORE3,
  NODE_STICK,
  NODE_NANO1,
  NODE_NANO2,
  NODE_NANO3,
  NODE_NANO4,
  NODE_M5GO,
  NODE_PAPER,
  NODE_PAPER2,
  NODE_BASIC,
  NODE_DINM,
  NODE_COUNT
};

struct MeshNodeConfig {
  const char* name;
  uint8_t address[6];
  const char* tbDeviceToken;
};

static constexpr size_t MAX_MESH_PEERS = NODE_COUNT - 1;
static_assert(MAX_MESH_PEERS <= ESP_NOW_MAX_TOTAL_PEER_NUM,
              "Mesh registry exceeds the ESP-NOW peer limit");

extern Peer meStorage;
extern Peer* me;
extern Peer peers[MAX_MESH_PEERS];
extern size_t NUM_PEERS;
extern const char* meshDeviceToken;
extern bool meshConfigured;

String stringMacAddress(const uint8_t* mac);
int getPeer(const uint8_t* mac);
int findPeer(const uint8_t* mac);
bool isZeroMac(const uint8_t* mac);
bool configureMeshIdentity();
const char* getDeviceToken();
const MeshNodeConfig* getMeshNodeConfig(MeshNodeId node);
void validatePeerList();
bool rememberStatusMessage(const StatusMessage& msg);
void onDataSent(const esp_now_send_info_t* tx_info, esp_now_send_status_t status);
void onDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len);
void addESPPeer(Peer p, int channel);
void sendStatus(StatusMessage msg, const uint8_t* excludeAddr = nullptr);
bool initMeshTx();
void meshTxComplete();
bool queueMeshPacket(const uint8_t* destination, const uint8_t* data, size_t length, bool priority = true);
