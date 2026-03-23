#pragma once

// Message size limit
#define MAX_MSG_SIZE 64

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
#define LORA_SF       9
#define LORA_BW       125E3
#define LORA_TX_POWER 14
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0x2D

// Timing constants
#define PACKET_INTERVAL 5000     // Send packet every n seconds (sender)
#define DISPLAY_UPDATE 1500      // Update display every n seconds
#define NO_CONTACT_TIMEOUT 60000 // Show "No contact" after n seconds
#define LOOP_DELAY 100           // Main loop delay
#define ACK_TIMEOUT_MS 2500
#define ACK_RETRY_COUNT 4

char    loraMessage[MAX_MSG_SIZE];
extern char TAG[36];

bool setupLoRa() {
    // Initialize SPI for LoRa module
    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI, -1);
    LoRa.setSPI(&SPI);
    LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);
    
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

    return true;
}

bool sendPacket( char *payload, int packetSize ) {
    LoRa.beginPacket();
    for( int i=0; i< packetSize; ++i )
        LoRa.write( (char)payload[i] );
    LoRa.endPacket();
    delay(10);

    //ESP_LOGI( TAG, "Sent packet %s", payload );
    if( LoRa.getWriteError() != 0 )
        return false;
    return true;
}

int receivePacket() {
    // Check for incoming packets
    int packetSize = LoRa.parsePacket();
    if( packetSize < 1 )
        return 0;

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
    return packetSize;
}
