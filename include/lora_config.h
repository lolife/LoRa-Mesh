#pragma once

// Message size limit
#define MAX_MSG_SIZE 64

// Module Connection Pins
//#ifdef SENDER
#define CS_PIN  6
#define RST_PIN 5
#define IRQ_PIN 10
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
#define PACKET_INTERVAL 3000     // Send packet every n seconds (sender)
#define DISPLAY_UPDATE 3000      // Update display every n seconds
#define NO_CONTACT_TIMEOUT 60000 // Show "No contact" after n seconds
#define LOOP_DELAY 100           // Main loop delay
#define ACK_TIMEOUT_MS 2500
#define ACK_RETRY_COUNT 4
