#include <cassert>
#include "../../src/mesh.cpp"
int main() {
    assert(configureMeshIdentity());
    assert(NUM_PEERS == NODE_COUNT - 1);
    assert(memcmp(me->address, mockMac, 6) == 0);
    int sends = 0;
    onSend = [&]() { ++sends; return ESP_OK; };
    for (size_t i = 0; i < NUM_PEERS; ++i) peers[i].lastHeard = mockNow;
    StatusMessage local = {};
    strcpy(local.deviceName, me->name); strcpy(local.varName, "T"); local.varValue = 22;
    sendStatus(local);
    assert(sends == static_cast<int>(NUM_PEERS));
    StatusMessage encoded = {}; memcpy(&encoded, sentPacket.data(), sizeof(encoded));
    assert(encoded.version == ESPNOW_STATUS_VERSION && encoded.messageId != 0);
    assert(memcmp(encoded.deviceAddr, me->address, 6) == 0);
    assert(rememberStatusMessage(encoded));

    StatusMessage remote = {};
    memcpy(remote.deviceAddr, peers[1].address, 6);
    strcpy(remote.deviceName, "origin"); strcpy(remote.varName, "V");
    remote.messageId = 7; remote.version = ESPNOW_STATUS_VERSION; remote.varValue = 4;
    esp_now_recv_info_t source = {}; memcpy(source.src_addr, peers[0].address, 6);
    sends = 0;
    onSend = [&]() { ++sends; assert(memcmp(sentTo, source.src_addr, 6) != 0); return ESP_OK; };
    onDataRecv(&source, reinterpret_cast<const uint8_t*>(&remote), sizeof(remote));
    assert(sends == static_cast<int>(NUM_PEERS) - 1);
    memcpy(&encoded, sentPacket.data(), sizeof(encoded));
    assert(encoded.relayCount == 1 && encoded.messageId == 7);
    assert(memcmp(encoded.deviceAddr, remote.deviceAddr, 6) == 0);
    assert(peers[1].currentData.varValue == 4);
    sends = 0;
    onDataRecv(&source, reinterpret_cast<const uint8_t*>(&remote), sizeof(remote));
    assert(sends == 0);
    mockNow += REBROADCAST_CACHE_TTL_MS + 1;
    onDataRecv(&source, reinterpret_cast<const uint8_t*>(&remote), sizeof(remote));
    assert(sends > 0);
    sends = 0;
    remote.version = 99;
    onDataRecv(&source, reinterpret_cast<const uint8_t*>(&remote), sizeof(remote));
    assert(sends == 0);
    LegacyStatusMessage legacy = {}; strcpy(legacy.varName, "T"); legacy.varValue = 25;
    onDataRecv(&source, reinterpret_cast<const uint8_t*>(&legacy), sizeof(legacy));
    assert(sends == 0 && peers[0].currentData.varValue == 25);
    assert(memcmp(peers[0].currentData.deviceAddr, source.src_addr, 6) == 0);
    onDataRecv(&source, reinterpret_cast<const uint8_t*>(&remote), 2);
    assert(sends == 0);
    // Sky Spy broadcasts must never reach status parsing, even if their
    // bytes happen to resemble a valid extended status-message prefix.
    uint8_t skyspy[126] = {};
    remote.version = ESPNOW_STATUS_VERSION;
    remote.messageId = 8;
    memcpy(skyspy, &remote, sizeof(remote));
    memcpy(skyspy, "SKY1", 4);
    const StatusMessage before = peers[1].currentData;
    onDataRecv(&source, skyspy, sizeof(skyspy));
    onDataRecv(&source, skyspy, 4); // malformed/truncated Sky Spy frame
    assert(sends == 0);
    assert(memcmp(&before, &peers[1].currentData, sizeof(before)) == 0);
    assert(!rememberStatusMessage(remote)); // no status dedup cache pollution
    // An ordinary status still works after the unrelated traffic.
    remote.messageId = 9;
    onDataRecv(&source, reinterpret_cast<const uint8_t*>(&remote), sizeof(remote));
    assert(sends > 0);
    sends = 0;
    onSend = [&]() { ++sends; return ESP_OK; };
    mockNow += 300001;
    sendStatus(local);
    assert(sends == static_cast<int>(NUM_PEERS));
    puts("PASS mesh: MAC identity, full peer registry, origin metadata, relay exclusion, dedup expiry, legacy RX, Sky Spy dispatch, inactive fallback");
}
