#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#define LOW 0
#define HIGH 1
#define INPUT 1
#define MSBFIRST 1
#define SPI_MODE0 0
#define LORA_DEFAULT_SPI_FREQUENCY 8E6
struct SPISettings { SPISettings(double, int, int) {} };
void digitalWrite(int, int) {}
int inputPin = -1;
void pinMode(int pin, int mode) { assert(mode == INPUT); inputPin = pin; }

#define ARDUINO_M5STACK_CORES3
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGV(...) ((void)0)

struct FakeSpi {
    bool addressPending = true;
    uint8_t address = 0;
    void begin(int, int, int, int) {}
    void beginTransaction(const SPISettings&) { addressPending = true; }
    uint8_t transfer(uint8_t value);
    void endTransaction() {}
} SPI;

struct FakeRadio {
    enum Mode { standby, single, continuous, transmitting } mode = standby;
    std::vector<uint8_t> received;
    size_t cursor = 0;
    bool scheduleAck = false;
    bool inFlight = false;
    bool crcError = false;
    int receiveCalls = 0;
    int polls = 0;
    int writes = 0;
    void setSPI(FakeSpi*) {}
    void setPins(int, int, int) {}
    bool begin(double) { return true; }
    void setTxPower(int) {}
    void setSignalBandwidth(double) {}
    void setSpreadingFactor(int) {}
    void setCodingRate4(int) {}
    void setSyncWord(int) {}
    void enableCrc() {}
    void receive() { ++receiveCalls; mode = continuous; }
    bool beginPacket() {
        if (mode == transmitting) return false;
        mode = standby;
        return true;
    }
    void write(char) { ++writes; }
    void endPacket() { mode = standby; }
    int getWriteError() { return 0; }
    int parsePacket() {
        ++polls;
        if (!received.empty()) {
            mode = standby;
            cursor = 0;
            return int(received.size());
        }
        // Model the actual driver: empty parse switches continuous to single
        // RX, interrupting an in-progress packet when RX is then restored.
        inFlight = false;
        crcError = false;
        mode = single;
        return 0;
    }
    int available() { return int(received.size() - cursor); }
    int read() {
        const int value = received[cursor++];
        if (cursor == received.size()) { received.clear(); cursor = 0; }
        return value;
    }
} LoRa;

uint8_t FakeSpi::transfer(uint8_t value) {
    if (addressPending) {
        assert((value & 0x80) == 0); // status polling must never write a register.
        address = value;
        addressPending = false;
        return 0;
    }
    assert(value == 0);
    if (address == 0x12) {
        return (!LoRa.received.empty() || LoRa.crcError ? 0x40 : 0) |
               (LoRa.crcError ? 0x20 : 0);
    }
    return 0;
}
bool forceDioLow = false, forceDioHigh = false;
int digitalRead(int pin) {
    assert(pin == inputPin);
    if (forceDioLow) return LOW;
    return (!LoRa.received.empty() || LoRa.crcError || forceDioHigh) ? HIGH : LOW;
}

void delay(unsigned ms) {
    // The peer's short ACK arrives during the legacy 10 ms post-TX delay.
    // A radio left in standby loses it, just as it does on the hardware.
    if (LoRa.scheduleAck && ms >= 10) {
        if (LoRa.mode == FakeRadio::continuous) LoRa.received = {0x73, 1, 42};
        LoRa.scheduleAck = false;
    }
}

char TAG[36] = "radio-test";
#include "lora_config.h"

int main() {
    assert(setupLoRa());
    assert(LoRa.mode == FakeRadio::continuous);
    char payload[] = {0x70, 1};
    LoRa.scheduleAck = true;
    assert(sendPacket(payload, sizeof(payload)));
    assert(LoRa.mode == FakeRadio::continuous);
    assert(receivePacket() == 3);
    assert(static_cast<uint8_t>(loraMessage[0]) == 0x73);
    assert(LoRa.mode == FakeRadio::continuous);
    // Poll throughout a long packet. RxDone is clear until completion. Neither
    // parsePacket nor receive may be invoked while the modem is receiving it.
    LoRa.inFlight = true;
    const int polls = LoRa.polls;
    const int receiveCalls = LoRa.receiveCalls;
    for (int i = 0; i < 40; ++i) assert(receivePacket() == 0);
    assert(LoRa.inFlight && LoRa.polls == polls && LoRa.receiveCalls == receiveCalls);
    assert(LoRa.mode == FakeRadio::continuous);
    LoRa.inFlight = false;
    LoRa.received.assign(248, 0x71);
    assert(receivePacket() == 248);
    assert(LoRa.mode == FakeRadio::continuous);
    // A completed packet with a CRC error is discarded and RX is restored.
    LoRa.crcError = true;
    assert(receivePacket() == 0);
    assert(LoRa.mode == FakeRadio::continuous && !LoRa.crcError);
    // Missing/wrong DIO0 wiring must not hide a packet the radio received.
    forceDioLow = true;
    LoRa.received = {0x73, 1, 42};
    assert(receivePacket() == 3);
    assert(radioRxDiagnostics.dioMisses == 1);
    assert(radioRxDiagnostics.crcErrors == 1);
    assert(radioRxDiagnostics.lastType == 0x73);
    forceDioLow = false;
    forceDioHigh = true;
    const int previousPolls = LoRa.polls;
    assert(receivePacket() == 0 && LoRa.polls == previousPolls);
    forceDioHigh = false;
    const int writes = LoRa.writes;
    assert(!sendPacket(nullptr, 1));
    assert(!sendPacket(payload, 0));
    assert(!sendPacket(payload, MAX_MSG_SIZE + 1));
    assert(LoRa.writes == writes);
    // A busy transmitter must not be switched into receive mode mid-packet.
    LoRa.mode = FakeRadio::transmitting;
    assert(!sendPacket(payload, sizeof(payload)));
    assert(LoRa.mode == FakeRadio::transmitting);
    puts("PASS radio: fast ACK reception, immediate RX after packet drain, undisturbed in-flight RX, full data packets, CRC recovery, missing/spurious DIO0, invalid/busy TX");
}
