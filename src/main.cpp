#include <M5Unified.h>
#include <SPI.h>
#include "M5_SX127X.h"
#include "lora_config.h"
#include "display.h"

// Global variables
int counter = 0;
uint16_t screenColor = TFT_NAVY;

// Function prototypes
void setupLoRa();
void handleSender();
void handleReceiver();

void setup() {
    Serial.begin( 115200 );
    // Initialize M5Stack with proper configuration
    auto cfg = M5.config();
    cfg.clear_display = true;
    cfg.internal_imu = false;  // Disable IMU to avoid ADC conflict
    cfg.internal_rtc = false;  // Disable RTC to avoid ADC conflict
    cfg.internal_spk = false;  // Disable speaker
    cfg.internal_mic = false;  // Disable microphone
    M5.begin(cfg);
    
    M5.Display.setBrightness(75);
    M5.Display.setRotation(0);
    M5.Display.setFont(&FreeSansBold18pt7b);

    // Initialize LoRa
    setupLoRa();
    
    // Display initial mode
#ifdef SENDER
    displayMessage("LoRa Sender", true, screenColor);
#else
    displayMessage("LoRa Receiver", true, screenColor);
#endif
}

void loop() {
    M5.update();
    
#ifdef SENDER
    handleSender();
#else
    handleReceiver();
#endif
    
    delay(LOOP_DELAY);
}

void setupLoRa() {
    // Initialize SPI for LoRa module
    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI, -1);
    LoRa.setSPI(&SPI);
    LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);
    
    // Start LoRa
    if (!LoRa.begin(LORA_FREQ)) {
        displayMessage("LoRa init fail.", true, screenColor);
        while (1) {
            delay(1000);
        }
    }
    
    // Configure LoRa parameters
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSpreadingFactor(LORA_SF);
}

void handleSender() {
    static long lastPacketTime = millis();
    static long lastDisplayUpdate = millis();
    
    // Send packet at regular intervals
    if (millis() - lastPacketTime > PACKET_INTERVAL) {
        lastPacketTime = millis();
        
        // Generate random color
        int r_val = random(256);   
        int g_val = random(256);     
        int b_val = random(256); 
        screenColor = M5.Display.color565(r_val, g_val, b_val);
        
        // Send color value via LoRa
        LoRa.beginPacket();
        LoRa.print(screenColor);
        LoRa.endPacket();
        Serial.printf( "Sent a packet: %d\n", screenColor );

        
        counter++;
        
        // Update display immediately after sending
        updateDisplay(screenColor, counter, true);
        lastDisplayUpdate = millis();
    }
    
    // Periodic display update
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE) {
        updateDisplay(screenColor, counter, true);
        lastDisplayUpdate = millis();
    }
}

void handleReceiver() {
    static long lastPacketTime = millis();
    static long lastDisplayUpdate = millis();
    
    // Check for incoming packets
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        Serial.printf( "Got a packet of size %d\n", packetSize );
        lastPacketTime = millis();
        char msg[MAX_MSG_SIZE];
        
        // Limit reading to buffer size
        int bytesToRead = packetSize;
        if (bytesToRead > MAX_MSG_SIZE - 1) {
            bytesToRead = MAX_MSG_SIZE - 1;
        }
        
        // Read packet data
        int i = 0;
        while (LoRa.available() && i < bytesToRead) {
            msg[i++] = (char)LoRa.read();
        }
        
        // Drain any remaining bytes
        while (LoRa.available()) {
            LoRa.read();
        }
        
        msg[i] = '\0';
        
        // Parse and update color
        int newColor = atoi(msg);
        if (newColor >= 0) {
            screenColor = newColor;
        }
        
        // Update display with new data
        updateDisplay(screenColor, counter, false);
        lastDisplayUpdate = millis();
    }
    
    // Periodic display update and timeout check
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE) {
        // Check for communication timeout
        if (millis() - lastPacketTime > NO_CONTACT_TIMEOUT) {
            displayMessage("No contact", true, screenColor);
            delay(2000);
            lastDisplayUpdate = millis();
        } else {
            updateDisplay(screenColor, counter, false);
            lastDisplayUpdate = millis();
        }
    }
}
