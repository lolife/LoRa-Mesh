#pragma once

// Message size limit
#define MAX_MSG_SIZE 64

// Module Connection Pins
#define CS_PIN  5
#define RST_PIN 25
#define IRQ_PIN 34
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCLK 18

// LoRa Parameters
#define LORA_FREQ     433E6
#define LORA_SF       12
#define LORA_BW       125E3
#define LORA_TX_POWER 9

// Timing constants
#define PACKET_INTERVAL 10000   // Send packet every n seconds (sender)
#define DISPLAY_UPDATE 10001     // Update display every n seconds
#define NO_CONTACT_TIMEOUT 60000 // Show "No contact" after n seconds
#define LOOP_DELAY 500          // Main loop delay
