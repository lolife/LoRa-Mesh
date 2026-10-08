#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <functional>
#include "espnow_protocol.h"
inline char TAG[36] = {};
inline unsigned long mockNow = 10000;
inline unsigned long millis() { return mockNow; }
template<class... Args> void mockLog(Args&&...) {}
#define ESP_LOGW(...) mockLog(__VA_ARGS__)
#include <vector>
#define ESP_NOW_ETH_ALEN 6
#define ESP_OK 0
inline std::vector<uint8_t> sentPacket;
inline uint8_t sentTo[6];
inline std::function<int()> onSend;
inline int esp_now_send(const uint8_t* dest,const uint8_t* data,size_t n){memcpy(sentTo,dest,6);sentPacket.assign(data,data+n);return onSend ? onSend() : 0;}

using esp_err_t = int;
#define ESP_ERR_ESPNOW_NO_MEM 12391
inline const char* esp_err_to_name(int){return "mock error";}
#ifdef TEST_MESH
using String = std::string;
String stringMacAddress(const uint8_t*);
#else
inline std::string stringMacAddress(const uint8_t*){return "test-mac";}
#endif
#ifndef TEST_TX_WORKER
inline bool queueMeshPacket(const uint8_t* dest,const uint8_t* data,size_t n,bool){return esp_now_send(dest,data,n)==ESP_OK;}
#endif

#ifdef TEST_MESH
#define ESP_NOW_MAX_TOTAL_PEER_NUM 20
#define WIFI_IF_STA 0
#define ESP_NOW_SEND_SUCCESS 0
#define ESP_LOGE(...) mockLog(__VA_ARGS__)
#define ESP_LOGI(...) mockLog(__VA_ARGS__)
#define ESP_LOGV(...) mockLog(__VA_ARGS__)
struct esp_now_send_info_t { uint8_t des_addr[6]; };
using esp_now_send_status_t = int;
struct esp_now_recv_info_t { uint8_t src_addr[6]; };
struct esp_now_peer_info_t { uint8_t peer_addr[6]; uint8_t channel; int ifidx; bool encrypt; };
inline uint8_t mockMac[6] = {0xF4,0x12,0xFA,0xBA,0x1A,0x10};
inline int esp_wifi_get_mac(int, uint8_t* mac) { memcpy(mac, mockMac, 6); return ESP_OK; }
inline int esp_efuse_mac_get_default(uint8_t* mac) { memcpy(mac, mockMac, 6); return ESP_OK; }
inline bool esp_now_is_peer_exist(const uint8_t*) { return false; }
inline int esp_now_add_peer(const esp_now_peer_info_t*) { return ESP_OK; }
inline uint32_t esp_random() { return 1234; }
#ifndef TEST_TX_WORKER
inline void meshTxComplete() {}
#endif

#endif
