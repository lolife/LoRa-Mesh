#include "mk_espmesh_lib.h"
#include <freertos/FreeRTOS.h>

/*
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
 */
static const MeshNodeConfig kMeshNodes[NODE_COUNT] = {
  { "Living Room",  {0x30, 0xED, 0xA0, 0xD4, 0xAF, 0x2C}, "k74m15ci6vmdkccye2np" },
  { "Core2",        {0xF4, 0x12, 0xFA, 0xBA, 0x1A, 0x10}, "0ounzp3k0q205tosgb8u" },
  { "Core3",        {0x30, 0xED, 0xA0, 0xD4, 0xBC, 0x08}, "343qS0FTegOCYGz4ubqM" },
  { "G-Door",       {0x00, 0x4B, 0x12, 0xC4, 0x6F, 0xCC}, "2t88z3nig0q7y0eem8qk" },
  { "Alert",        {0x54, 0x32, 0x04, 0x3E, 0xFE, 0xF4}, "oghxcyqc2b2eqdegi8cj" },
  { "Nano2",        {0x9C, 0x13, 0x9E, 0xCC, 0x0C, 0x80}, "r1t00l9ejy9jc8hyw3zu" },
  { "Woods",        {0x54, 0x32, 0x04, 0x3F, 0x02, 0xCC}, "iz7gag56h83tg3zupkfj" },
  { "Outside",      {0x9C, 0x13, 0x9E, 0xCC, 0x23, 0xB4}, "dhcazminng3fad7n83wc" },
  { "Studio",       {0x2C, 0xBC, 0xBB, 0x94, 0x32, 0x3C}, "x3tX07tqwZTJlSu99rtw" },
  { "Upstairs",     {0x5c, 0x01, 0x3b, 0x0d, 0xba, 0x20}, "bflwj0wzaiv574unjuc6" },
  { "Paper2",       {0x5C, 0x01, 0x3B, 0x0D, 0xB8, 0xB4}, "8mr1ooxbf4iqd0l2r3m0" },
  { "Pumphouse",    {0x5C, 0x01, 0x3B, 0x13, 0x94, 0x74}, "faqm7ko7zmo9x0eackem" },
  { "Furnace",      {0xD8, 0x85, 0xAC, 0xA3, 0x60, 0x74}, "ouvt3yocz6g7gk3pbur7" },
};


Peer meStorage = {};
Peer* me = &meStorage;
Peer peers[MAX_MESH_PEERS] = {};
size_t NUM_PEERS = 0;
const char* meshDeviceToken = "";
bool meshConfigured = false;

static void copyPeerFromConfig(Peer& dst, const MeshNodeConfig& src) {
  memset(&dst, 0, sizeof(dst));
  strncpy(dst.name, src.name, sizeof(dst.name) - 1);
  memcpy(dst.address, src.address, sizeof(dst.address));
}

static int defaultNodeFromBuild() {
#if defined(MESH_NODE_PAPER) || defined(ARDUINO_M5STACK_PAPER)
  return NODE_PAPER2;
#elif defined(DINM)
  return NODE_DINM;
#elif defined(ARDUINO_M5STACK_STICKC_PLUS2)
  return NODE_STICK;
#elif defined(CORE1)
  return NODE_CORE1;
#elif defined(CORE2)
  return NODE_CORE2;
#elif defined(CORE3)
  return NODE_CORE3;
#elif defined(ARDUINO_M5STACK_NANO) && defined(NANO1)
  return NODE_NANO1;
#elif defined(ARDUINO_M5STACK_NANO) && defined(NANO2)
  return NODE_NANO2;
#elif defined(ARDUINO_M5STACK_NANO) && defined(NANO3)
  return NODE_NANO3;
#elif defined(ARDUINO_M5STACK_NANO) && defined(NANO4)
  return NODE_NANO4;
#elif defined(ARDUINO_M5STACK_BASIC)
  return NODE_BASIC;
#elif defined(ARDUINO_M5Stack_Core_ESP32)
  return NODE_M5GO;
#else
  return NODE_CORE2;
#endif
}

bool configureMeshIdentity() {
  if (meshConfigured) {
    return true;
  }

  uint8_t localMac[ESP_NOW_ETH_ALEN] = {};
  if (esp_wifi_get_mac(WIFI_IF_STA, localMac) != ESP_OK) {
    if (esp_efuse_mac_get_default(localMac) != ESP_OK) {
      ESP_LOGE(TAG, "%s", "Failed to read local MAC");
      return false;
    }
  }

  int selected = -1;
  for (int i = 0; i < static_cast<int>(NODE_COUNT); ++i) {
    if (memcmp(localMac, kMeshNodes[i].address, ESP_NOW_ETH_ALEN) == 0) {
      selected = i;
      break;
    }
  }

  if (selected < 0) {
    selected = defaultNodeFromBuild();
    if (selected < 0) {
      ESP_LOGE(TAG, "Unknown local MAC %s and no build fallback", stringMacAddress(localMac).c_str());
      return false;
    }
    ESP_LOGW(TAG, "Unknown local MAC %s, using build fallback node %s",
             stringMacAddress(localMac).c_str(), kMeshNodes[selected].name);
  }

  const MeshNodeConfig& meCfg = kMeshNodes[selected];
  copyPeerFromConfig(meStorage, meCfg);
  meshDeviceToken = meCfg.tbDeviceToken;
  // Every registered node is a peer of every other registered node.
  NUM_PEERS = 0;
  for (int i = 0; i < static_cast<int>(NODE_COUNT); ++i) {
    if (i == selected) {
      continue;
    }
    copyPeerFromConfig(peers[NUM_PEERS++], kMeshNodes[i]);
  }

  meshConfigured = true;
  ESP_LOGI(TAG, "Mesh identity: %s (%s), peers=%d",
           me->name, stringMacAddress(me->address).c_str(), static_cast<int>(NUM_PEERS));
  return true;
}

const char* getDeviceToken() {
  if (!meshConfigured) {
    configureMeshIdentity();
  }
  return meshDeviceToken;
}

const MeshNodeConfig* getMeshNodeConfig(MeshNodeId node) {
  return node < NODE_COUNT ? &kMeshNodes[node] : nullptr;
}

struct RebroadcastCacheEntry {
  uint8_t deviceAddr[6];
  uint16_t messageId;
  unsigned long seenAt;
  bool used;
};

static constexpr size_t REBROADCAST_CACHE_SIZE = 64;
static constexpr unsigned long REBROADCAST_CACHE_TTL_MS = 180000;
RebroadcastCacheEntry rebroadcastCache[REBROADCAST_CACHE_SIZE] = {};
size_t rebroadcastCacheNext = 0;

String stringMacAddress(const uint8_t* mac) {
  char buf[18 + 1]; // "AA:BB:CC:DD:EE:FF" + '\0'
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

int getPeer(const uint8_t* mac) {
  for(size_t i = 0; i < NUM_PEERS; ++i) {
      //Peer p = peers[i];
      //ESP_LOGD( TAG, "Checking peer %d: %s", i, peers[i].name );
      // Direct memory comparison is faster than string comparison
      if(memcmp(mac, peers[i].address, ESP_NOW_ETH_ALEN) == 0) {
        return i;
      }
  }
  ESP_LOGE( TAG, "Peer %s not found", stringMacAddress(mac).c_str());
  return -1;
}

int findPeer(const uint8_t* mac) {
  for (size_t i = 0; i < NUM_PEERS; ++i) {
    if (memcmp(mac, peers[i].address, ESP_NOW_ETH_ALEN) == 0) {
      return i;
    }
  }
  return -1;
}

bool isZeroMac(const uint8_t* mac) {
  static const uint8_t zeroMac[6] = {0, 0, 0, 0, 0, 0};
  return memcmp(mac, zeroMac, 6) == 0;
}

void validatePeerList() {
  for (size_t i = 0; i < NUM_PEERS; ++i) {
    for (size_t j = i + 1; j < NUM_PEERS; ++j) {
      if (memcmp(peers[i].address, peers[j].address, ESP_NOW_ETH_ALEN) == 0) {
        ESP_LOGE(TAG, "Duplicate peer MAC: %s and %s both use %s",
                 peers[i].name, peers[j].name, stringMacAddress(peers[i].address).c_str());
      }
      if (strncmp(peers[i].name, peers[j].name, sizeof(peers[i].name)) == 0) {
        ESP_LOGW(TAG, "Duplicate peer name: %s (%s, %s)",
                 peers[i].name,
                 stringMacAddress(peers[i].address).c_str(),
                 stringMacAddress(peers[j].address).c_str());
      }
    }
  }
}

static portMUX_TYPE cacheMux = portMUX_INITIALIZER_UNLOCKED;

bool rememberStatusMessage(const StatusMessage& msg) {
  if (msg.messageId == 0 || isZeroMac(msg.deviceAddr)) {
    return false;
  }

  const unsigned long now = millis();
  portENTER_CRITICAL(&cacheMux);
  int freeSlot = -1;

  for (size_t i = 0; i < REBROADCAST_CACHE_SIZE; ++i) {
    const bool isUsed = rebroadcastCache[i].used;
    if (!isUsed) {
      if (freeSlot < 0) {
        freeSlot = static_cast<int>(i);
      }
      continue;
    }

    if (now - rebroadcastCache[i].seenAt > REBROADCAST_CACHE_TTL_MS) {
      rebroadcastCache[i].used = false;
      if (freeSlot < 0) {
        freeSlot = static_cast<int>(i);
      }
      continue;
    }

    if (rebroadcastCache[i].messageId == msg.messageId &&
        memcmp(rebroadcastCache[i].deviceAddr, msg.deviceAddr, 6) == 0) {
      portEXIT_CRITICAL(&cacheMux);
      return true;
    }
  }

  if (freeSlot < 0) {
    freeSlot = static_cast<int>(rebroadcastCacheNext);
    rebroadcastCacheNext = (rebroadcastCacheNext + 1) % REBROADCAST_CACHE_SIZE;
  }

  memcpy(rebroadcastCache[freeSlot].deviceAddr, msg.deviceAddr, 6);
  rebroadcastCache[freeSlot].messageId = msg.messageId;
  rebroadcastCache[freeSlot].seenAt = now;
  rebroadcastCache[freeSlot].used = true;
  portEXIT_CRITICAL(&cacheMux);
  return false;
}

void onDataSent(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
  meshTxComplete();
  int p = findPeer(tx_info->des_addr);
  const char* peerName = (p >= 0) ? peers[p].name : "Unknown";
  if (status == ESP_NOW_SEND_SUCCESS) {
    ESP_LOGV(TAG, "Send ACK from %s (%s)", peerName, stringMacAddress(tx_info->des_addr).c_str());
  } else {
    ESP_LOGV(TAG, "Send NACK from %s (%s), status=%d", peerName, stringMacAddress(tx_info->des_addr).c_str(), status);
  }
}

void onDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) {
  // DTunnel also broadcasts 126-byte Sky Spy frames. Its receive dispatcher
  // consumes the SKY1 protocol before status parsing, including malformed
  // frames. This bridge does not process Sky Spy events, so skip them here.
  if (data_len >= 4 && memcmp(data, "SKY1", 4) == 0) return;
  StatusMessage msg = {};
  bool hasRelayMetadata = false;
  const bool srcIsMe = (memcmp(esp_now_info->src_addr, me->address, ESP_NOW_ETH_ALEN) == 0);
  int srcPeer = findPeer(esp_now_info->src_addr);
  const char* srcName = (srcPeer != -1) ? peers[srcPeer].name : (srcIsMe ? me->name : "Unknown");

  if (data_len == sizeof(StatusMessage)) {
    memcpy(&msg, data, sizeof(StatusMessage));
    hasRelayMetadata = (msg.version == ESPNOW_STATUS_VERSION && msg.messageId != 0 && !isZeroMac(msg.deviceAddr));
  } else if (data_len > static_cast<int>(sizeof(StatusMessage))) {
    memcpy(&msg, data, sizeof(StatusMessage));
    hasRelayMetadata = (msg.version == ESPNOW_STATUS_VERSION && msg.messageId != 0 && !isZeroMac(msg.deviceAddr));
    if (!hasRelayMetadata) {
      ESP_LOGE(TAG, "Bad length = %d from %s (%s), expected %d or %d",
               data_len, srcName, stringMacAddress(esp_now_info->src_addr).c_str(),
               sizeof(StatusMessage), sizeof(LegacyStatusMessage));
      return;
    }
    ESP_LOGW(TAG, "Accepted %d-byte status from %s (%s) using %d-byte prefix",
             data_len, srcName, stringMacAddress(esp_now_info->src_addr).c_str(),
             sizeof(StatusMessage));
  } else if (data_len == sizeof(LegacyStatusMessage)) {
    LegacyStatusMessage legacy = {};
    memcpy(&legacy, data, sizeof(LegacyStatusMessage));
    memcpy(msg.deviceName, legacy.deviceName, sizeof(msg.deviceName));
    memcpy(msg.varName, legacy.varName, sizeof(msg.varName));
    msg.varValue = legacy.varValue;
    memcpy(msg.deviceAddr, esp_now_info->src_addr, sizeof(msg.deviceAddr));
    msg.messageId = 0;
    msg.relayCount = 0;
    msg.version = 0;
  } else {
    ESP_LOGE(TAG, "Bad length = %d from %s (%s), expected %d or %d",
             data_len, srcName, stringMacAddress(esp_now_info->src_addr).c_str(),
             sizeof(StatusMessage), sizeof(LegacyStatusMessage));
    return;
  }

  if (data_len == sizeof(StatusMessage) && msg.version != ESPNOW_STATUS_VERSION) {
    ESP_LOGW(TAG, "Ignoring status with unsupported version=%u from %s",
             msg.version, stringMacAddress(esp_now_info->src_addr).c_str());
    return;
  }

  msg.deviceName[sizeof(msg.deviceName) - 1] = 0;
  msg.varName[sizeof(msg.varName) - 1] = 0;
  const unsigned long now = millis();
  if (srcPeer != -1) {
    // Source liveness is tied to source MAC, not message origin name.
    peers[srcPeer].lastHeard = now;
  } else if (!srcIsMe) {
    ESP_LOGE( TAG, "Peer %s not found?", stringMacAddress(esp_now_info->src_addr).c_str());
  }

  int originPeer = findPeer(msg.deviceAddr);
  const bool originIsMe = (memcmp(msg.deviceAddr, me->address, ESP_NOW_ETH_ALEN) == 0);
  if (originPeer != -1) {
    peers[originPeer].lastHeard = now;
    peers[originPeer].currentData = msg;
    ESP_LOGV( TAG, "From %s via %s: %s = %0.1f",
              peers[originPeer].name,
              (srcPeer != -1) ? peers[srcPeer].name : "Unknown",
              msg.varName, msg.varValue);
  } else if (originIsMe) {
    ESP_LOGV(TAG, "From self via %s: %s = %0.1f",
             (srcPeer != -1) ? peers[srcPeer].name : "Unknown",
             msg.varName, msg.varValue);
  } else {
    ESP_LOGW(TAG, "Unknown origin %s (%s)",
             msg.deviceName, stringMacAddress(msg.deviceAddr).c_str());
  }

  bool fromMe = (memcmp(msg.deviceAddr, me->address, 6) == 0);
  bool seenBefore = rememberStatusMessage(msg);
  if (hasRelayMetadata && !fromMe && !seenBefore) {
    StatusMessage relayMsg = msg;
    relayMsg.relayCount = (relayMsg.relayCount < 255) ? relayMsg.relayCount + 1 : relayMsg.relayCount;
    sendStatus(relayMsg, esp_now_info->src_addr);
  }
}

void addESPPeer(Peer p, int channel) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, p.address, ESP_NOW_ETH_ALEN);
  peerInfo.channel = static_cast<uint8_t>(channel);
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_is_peer_exist(p.address)) {
    ESP_LOGI(TAG, "Peer already exists %s", p.name);
    return;
  }

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    ESP_LOGE( TAG, "Failed to add peer %s", p.name );
    return;
  }
  ESP_LOGI( TAG, "Added peer %s", p.name);
}

void sendStatus(StatusMessage msg, const uint8_t* excludeAddr) {
  if (msg.version != ESPNOW_STATUS_VERSION || msg.messageId == 0) {
    static uint16_t nextMessageId = static_cast<uint16_t>(esp_random()) | 1;
    msg.messageId = nextMessageId++;
    if (nextMessageId == 0) nextMessageId = 1;
    memcpy(msg.deviceAddr, me->address, sizeof(msg.deviceAddr));
    msg.version = ESPNOW_STATUS_VERSION;
    msg.relayCount = 0;
    rememberStatusMessage(msg);
  }
  // Called by both the main loop and WiFi RX task. Enqueue only: never sleep
  // or wait for TX completion inside a WiFi callback.
  const unsigned long now = millis();
  bool hasActivePeer = false;
  for (size_t i = 0; i < NUM_PEERS; ++i) {
    if (excludeAddr && memcmp(excludeAddr, peers[i].address, ESP_NOW_ETH_ALEN) == 0) continue;
    if (now - peers[i].lastHeard < 300000) hasActivePeer = true;
  }
  for (size_t i = 0; i < NUM_PEERS; ++i) {
    if (excludeAddr && memcmp(excludeAddr, peers[i].address, ESP_NOW_ETH_ALEN) == 0) continue;
    if (hasActivePeer && now - peers[i].lastHeard >= 300000) continue;
    queueMeshPacket(peers[i].address, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg), false);
  }
}
