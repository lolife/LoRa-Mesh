//#include "main.h"
#include <esp_now.h>
#include <esp_log.h>

extern char TAG[36];

typedef struct {
  char deviceName[16];
  char varName[10];
  float varValue;
} StatusMessage;

struct Peer {
  char name[16];
  uint8_t address[6];
  unsigned long lastHeard;
  StatusMessage currentData;
};

// Define all peers as constants
//const Peer core1 = { "Core1",    {0x24, 0xEC, 0x4A, 0x36, 0xF7, 0x94} };
const Peer core2 = { "In Mid",  {0xF4, 0x12, 0xFA, 0xBA, 0x1A, 0x10} };
const Peer core3 = { "GPS",    {0x30, 0xED, 0xA0, 0xD4, 0xBC, 0x08} };
const Peer stick = { "Stick", {0x00, 0x4B, 0x12, 0xC4, 0x6F, 0xCC} };
const Peer nano  = { "Nano",     {0x54, 0x32, 0x04, 0x3E, 0xFE, 0xF4} };
const Peer nano2 = { "Out",    {0x54, 0x32, 0x04, 0x3F, 0x02, 0xCC } };
const Peer nano3 = { "Nano3",    {0x54, 0x32, 0x04, 0x3E, 0xF7, 0x9C } };
const Peer m5go  = { "In Up",   {0x2C, 0xBC, 0xBB, 0x94, 0x32, 0x3C} };
const Peer atom  = { "Atom",      {0x98, 0x88, 0xE0, 0x0E, 0x65, 0x4C} };
const Peer m5basic = { "Studio",  {0x84, 0x1F, 0xE8, 0x83, 0x45, 0x54} };

// Configure based on hardware
#if defined(ARDUINO_M5Stack_ATOMS3)
	#ifdef ATOMS3R
  Peer *me = const_cast<Peer*>(&atom);
  Peer peers[] = { core1, core2, core3, nano, nano2, stick, m5basic };
  #endif
  
#elif defined(ARDUINO_M5STACK_STICKC_PLUS2)
  Peer *me = const_cast<Peer*>(&stick);
  Peer peers[] = { atom, nano, nano2, m5basic, m5go };
  
#elif defined(ARDUINO_M5STACK_CORES3)
  #if defined(CORE1)
    Peer *me = const_cast<Peer*>(&core1);
    Peer peers[] = { atom, core2, core3, stick, nano, m5go, m5basic };
    const char* TB_DEVICE_TOKEN = "d408tvoz3tzwxrg7jewx"; // M5-CoreS3-01 24:EC:4A:36:F7:94
  #elif defined(CORE2)
    Peer *me = const_cast<Peer*>(&core2);
    Peer peers[] = { core3 };
    const char* TB_DEVICE_TOKEN = "0ounzp3k0q205tosgb8u"; // M5-CoreS3-02 F4:12:FA:BA:1A:10
  #elif defined(CORE3)
    Peer *me = const_cast<Peer*>(&core3);
    Peer peers[] = { m5basic };
    const char* TB_DEVICE_TOKEN = "343qS0FTegOCYGz4ubqM"; // M5-CoreS3-03 30:ED:A0:D4:BC:08
  #endif

#elif defined(ARDUINO_M5STACK_NANO)
  #ifdef PIR
  Peer *me = const_cast<Peer*>(&nano);
  Peer peers[] = { atom, stick, m5go, m5basic, nano2, nano3 };
  #else
    Peer *me = const_cast<Peer*>(&nano2);
    Peer peers[] = { atom, stick, m5go, m5basic, nano, nano3 };
    //Peer *me = const_cast<Peer*>(&nano3);
    //Peer peers[] = { atom, stick, m5go, m5basic, nano, nano2 };
  #endif

#elifdef ARDUINO_M5STACK_BASIC
  Peer *me = const_cast<Peer*>(&m5basic);
  Peer peers[] = { atom, stick, nano, m5go, nano2, nano3 };

#elif defined(ARDUINO_M5Stack_Core_ESP32)
  Peer *me = const_cast<Peer*>(&m5go);
  Peer peers[] = { core2, core3, atom, stick, nano, nano2, m5basic };

#else
  #error "Unknown hardware platform"
#endif

// Automatically derive NUM_PEERS from array size
#define NUM_PEERS (sizeof(peers) / sizeof(peers[0]))

String stringMacAddress(const uint8_t *mac) {
  char buf[18 + 1]; // "AA:BB:CC:DD:EE:FF" + '\0'
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

int getPeer(const uint8_t *mac) {
  for(int i = 0; i < NUM_PEERS; ++i) {
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

int findPeer(const uint8_t *mac) {
  for (int i = 0; i < NUM_PEERS; ++i) {
    if (memcmp(mac, peers[i].address, ESP_NOW_ETH_ALEN) == 0) {
      return i;
    }
  }
  return -1;
}

// Callback when data is sent
void onDataSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
  int p = findPeer(tx_info->des_addr);
  const char* peerName = (p >= 0) ? peers[p].name : "Unknown";
  if (status == ESP_NOW_SEND_SUCCESS) {
    ESP_LOGV(TAG, "Send ACK from %s (%s)", peerName, stringMacAddress(tx_info->des_addr).c_str());
  } else {
    ESP_LOGW(TAG, "Send NACK from %s (%s), status=%d", peerName, stringMacAddress(tx_info->des_addr).c_str(), status);
  }
} 

// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
  if (data_len == sizeof(StatusMessage)) {
    StatusMessage msg;
    memcpy(&msg, data, sizeof(StatusMessage));
    int p = getPeer(esp_now_info->src_addr);
    if(p != -1) {
      memcpy( peers[p].name, msg.deviceName, sizeof(peers[p].name ) );
      peers[p].lastHeard = millis();
      peers[p].currentData = msg;
      //ESP_LOGV( TAG, "From %s: %s = %0.1f", peers[p].name, msg.varName, msg.varValue);
    }
    else
      ESP_LOGE( TAG, "Peer %s not found?", stringMacAddress(esp_now_info->src_addr).c_str());
  }
  else
    ESP_LOGE( TAG, "Bad length = %d, should be %d", data_len, sizeof(StatusMessage));
}

void addESPPeer(Peer p) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, p.address, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    ESP_LOGE( TAG, "%s", "Failed to add peer" );
    return;
  }
  ESP_LOGI( TAG, "Added peer %s", p.name);
}

void sendStatus(StatusMessage msg) {
  esp_err_t status = ESP_OK;
  int sentCount = 0;
  static unsigned long lastNoPeerLog = 0;
  
  // Only send to peers we've heard from recently (active peers)
  // This reduces power consumption by not sending to inactive peers
  for(int i = 0; i < NUM_PEERS; ++i) {
    if(millis() - peers[i].lastHeard < 300000) { // Only if heard in last 5 minutes
      status = esp_now_send(peers[i].address, (uint8_t *)&msg, sizeof(msg));
      
      if (status != ESP_OK) {
        ESP_LOGE( TAG, "Failed to send to %s: %d", peers[i].name, status);
      } else {
        ESP_LOGV( TAG, "Sent to %s: %d", peers[i].name, status);
        sentCount++;
      }
      
      // Small delay between sends to prevent congestion
      // Reduced from 10ms to 5ms for better responsiveness
      delay(5);
    }
  }
  
  if (sentCount == 0) {
    // No active peers is expected when no node has reported in recently.
    // Keep this diagnostic but rate-limit and lower severity.
    if (millis() - lastNoPeerLog > 60000) {
      ESP_LOGW( TAG, "%s", "No active peers to send to");
      lastNoPeerLog = millis();
    }

    // Fallback: if our local active list is stale, still push one round to all
    // configured peers so bidirectional traffic can recover.
    for (int i = 0; i < NUM_PEERS; ++i) {
      status = esp_now_send(peers[i].address, (uint8_t *)&msg, sizeof(msg));
      if (status != ESP_OK) {
        ESP_LOGE( TAG, "Fallback send failed to %s: %d", peers[i].name, status);
      } else {
        sentCount++;
      }
      delay(5);
    }
  }
  
  // Removed unnecessary 100ms delay at the end
}
