#pragma once
#include "radio_profile.h"

// SX127x packet limit. Existing telemetry wire formats remain <=64 bytes.
#define MAX_MSG_SIZE 255

// Module Connection Pins

#ifdef ARDUINO_M5STACK_BASIC
    #define CS_PIN  5
    #define RST_PIN 25
    #define IRQ_PIN 34
#elif defined(ARDUINO_M5STACK_CORES3)
    #define CS_PIN  6
    #define RST_PIN 5
    #define IRQ_PIN 10
#endif

// #else
// #define CS_PIN  5
// #define RST_PIN 25
// #define IRQ_PIN 34
// #endif

#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCLK 18

// LoRa Parameters
#define LORA_FREQ     433E6
#define LORA_TX_POWER 14
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0x2D

// Timing constants
#define PACKET_INTERVAL (30UL * 1000UL) // Sender telemetry every 30 seconds.
#define DISPLAY_UPDATE 5000      // Update display every n seconds
#define NO_CONTACT_TIMEOUT 120000 // Show "No contact" after n seconds
#define LOOP_DELAY 100           // Main loop delay
#define ACK_TIMEOUT_MS 2500
#define ACK_RETRY_COUNT 4

char    loraMessage[MAX_MSG_SIZE];
extern char TAG[36];

// Read-only status polling avoids parsePacket()'s mode changes and does not
// require the module's DIO0 wire to be connected to the configured GPIO.
uint8_t readLoRaRegister(uint8_t address) {
    SPI.beginTransaction(SPISettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(address & 0x7f);
    const uint8_t value = SPI.transfer(0);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    return value;
}

struct RadioRxDiagnostics {
    uint32_t packets = 0, crcErrors = 0, dioMisses = 0;
    int lastLength = 0;
    uint8_t lastType = 0;
} radioRxDiagnostics;

void logRadioRxDiagnostics() {
    ESP_LOGI(TAG, "Radio RX: packets=%lu CRC_errors=%lu DIO_misses=%lu last=%d bytes/type 0x%02x; mode=0x%02x IRQ=0x%02x config=0x%02x/0x%02x PA=0x%02x sync=0x%02x",
             (unsigned long)radioRxDiagnostics.packets, (unsigned long)radioRxDiagnostics.crcErrors,
             (unsigned long)radioRxDiagnostics.dioMisses, radioRxDiagnostics.lastLength,
             radioRxDiagnostics.lastType, readLoRaRegister(0x01), readLoRaRegister(0x12),
             readLoRaRegister(0x1d), readLoRaRegister(0x1e),
             readLoRaRegister(0x09), readLoRaRegister(0x39));
}

bool setupLoRa() {
    // Initialize SPI for LoRa module
    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI, -1);
    LoRa.setSPI(&SPI);
    LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);
    pinMode(IRQ_PIN, INPUT);
    
    // Start LoRa
    if (!LoRa.begin(LORA_FREQ)) {
        ESP_LOGE( TAG, "%s", "LoRa init fail." );
        return false;
    }
    
    // Configure LoRa parameters
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();
    LoRa.receive();

    ESP_LOGI(TAG, "LoRa %.3f MHz, SF%d, BW %lu kHz, CR 4/%d, TX %d dBm",
             LORA_FREQ / 1E6, LORA_SF, static_cast<unsigned long>(LORA_BW / 1000),
             LORA_CODING_RATE, LORA_TX_POWER);
    logRadioRxDiagnostics();

    return true;
}

bool sendPacket( char *payload, int packetSize ) {
    if (!payload || packetSize <= 0 || packetSize > MAX_MSG_SIZE || !LoRa.beginPacket())
        return false;
    for( int i=0; i< packetSize; ++i )
        LoRa.write( (char)payload[i] );
    LoRa.endPacket();
    // TX completion leaves the SX127x in standby. Listen BEFORE the delay:
    // a fast SF7/250 kHz ACK has only about 6 ms of preamble.
    LoRa.receive();
    delay(10);

    //ESP_LOGI( TAG, "Sent packet %s", payload );
    if( LoRa.getWriteError() != 0 )
        return false;
    return true;
}

int receivePacket() {
    // Read RegIrqFlags without touching mode/FIFO registers. Only parse after
    // RxDone (bit 6); keep the modem undisturbed while a packet is in flight.
    const uint8_t irq = readLoRaRegister(0x12);
    if (!(irq & 0x40)) return 0;
    if (digitalRead(IRQ_PIN) == LOW) ++radioRxDiagnostics.dioMisses;
    if (irq & 0x20) ++radioRxDiagnostics.crcErrors;
    int packetSize = LoRa.parsePacket();
    if( packetSize < 1 ) {
        // parsePacket() selects single RX, which can time out between polls.
        LoRa.receive();
        return 0;
    }

    ESP_LOGV( TAG, "Got packet size %d", packetSize );

    // Limit reading to buffer size
    int bytesToRead = packetSize;
    if (bytesToRead > MAX_MSG_SIZE) {
        bytesToRead = MAX_MSG_SIZE;
    }
    
    memset( loraMessage, '\0', MAX_MSG_SIZE );

    // Read packet data
    int i = 0;
    while (LoRa.available() && i < bytesToRead) {
        loraMessage[i] = (char)LoRa.read();
        //ESP_LOGD( TAG, "Got byte: %x", loraMessage[i] );
        i++;
    }
    // Drain any remaining bytes
    while (LoRa.available()) {
        ESP_LOGV( TAG, "%s", "Throwing away data" );
        LoRa.read();
    }
    // parsePacket() returns to standby after a packet. Resume listening before
    // application processing (MQTT, display, ACK handling) can delay the loop.
    LoRa.receive();
    ++radioRxDiagnostics.packets;
    radioRxDiagnostics.lastLength = packetSize;
    radioRxDiagnostics.lastType = static_cast<uint8_t>(loraMessage[0]);
    return packetSize;
}
