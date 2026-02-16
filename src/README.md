# M5Stick LoRa Sender/Receiver Project

This project implements a dual-mode LoRa communication system for M5Stick devices using PlatformIO's multiple environment feature.

## Project Structure

```
project/
├── platformio.ini          # PlatformIO configuration with sender/receiver environments
├── src/
│   ├── main.cpp           # Main application code with conditional compilation
│   ├── display.h          # Display function declarations
│   ├── display.cpp        # Display function implementations
│   └── lora_config.h      # Shared LoRa and timing configuration
```

## Setup Instructions

1. **Create a new PlatformIO project** or use existing one
2. **Copy files to your project:**
   - `platformio.ini` → project root
   - `main.cpp`, `display.h`, `display.cpp`, `lora_config.h` → `src/` folder

3. **Update upload ports** in `platformio.ini`:
   - Change `/dev/ttyUSB0` and `/dev/ttyUSB1` to match your system
   - Windows: `COM3`, `COM4`, etc.
   - macOS: `/dev/cu.usbserial-*`
   - Linux: `/dev/ttyUSB*` or `/dev/ttyACM*`

## Building and Flashing

### Using VS Code / PlatformIO Extension:

1. **Switch environments:**
   - Click on the environment name in the bottom toolbar
   - Select either `sender` or `receiver`

2. **Build:**
   - Click the checkmark (✓) icon, or
   - Press `Ctrl+Alt+B` (Windows/Linux) or `Cmd+Alt+B` (macOS)

3. **Upload:**
   - Click the arrow (→) icon, or
   - Press `Ctrl+Alt+U` (Windows/Linux) or `Cmd+Alt+U` (macOS)

### Using Command Line:

```bash
# Build sender
pio run -e sender

# Upload sender
pio run -e sender -t upload

# Build receiver
pio run -e receiver

# Upload receiver
pio run -e receiver -t upload

# Monitor serial output
pio device monitor -e sender
pio device monitor -e receiver
```

## Workflow for Two Devices

### Option 1: Sequential Flashing (One USB Port)
```bash
# Flash device 1 as sender
pio run -e sender -t upload

# Disconnect device 1, connect device 2
# Flash device 2 as receiver
pio run -e receiver -t upload
```

### Option 2: Simultaneous Flashing (Two USB Ports)
1. Update `platformio.ini` with correct ports for both devices
2. Open two terminals:
   ```bash
   # Terminal 1
   pio run -e sender -t upload
   
   # Terminal 2
   pio run -e receiver -t upload
   ```

## Configuration

### LoRa Parameters (lora_config.h)
- **Frequency:** 433 MHz (adjust for your region)
- **Spreading Factor:** 12 (maximum range, slowest)
- **Bandwidth:** 125 kHz
- **TX Power:** 9 dBm

### Timing Parameters (lora_config.h)
- **PACKET_INTERVAL:** 10 seconds (sender broadcast interval)
- **DISPLAY_UPDATE:** 5 seconds (display refresh rate)
- **NO_CONTACT_TIMEOUT:** 60 seconds (receiver shows "No contact")
- **LOOP_DELAY:** 500 ms (main loop delay)

## Features

### Sender Mode
- Generates random RGB colors every 10 seconds
- Broadcasts color as RGB565 value via LoRa
- Displays current color with RGB values and packet count
- Automatic text contrast (black/white based on background luminance)

### Receiver Mode
- Receives color data from sender
- Updates display background to match sender
- Shows LoRa statistics:
  - SNR (Signal-to-Noise Ratio)
  - Frequency Error
  - RSSI (Received Signal Strength)
  - Battery Level
- Displays "No contact" if no packets received for 60 seconds

## Customization

### Change Packet Interval
Edit `lora_config.h`:
```cpp
#define PACKET_INTERVAL 5000  // Send every 5 seconds
```

### Modify Display Behavior
Edit `display.cpp` to customize what's shown in sender or receiver mode.

### Add Serial Debugging
Uncomment in `main.cpp`:
```cpp
void setup() {
    Serial.begin(115200);  // Uncomment this line
    M5.begin();
    // ...
}
```

Then add debug statements:
```cpp
Serial.printf("Sent packet %d, color: %d\n", counter, screenColor);
```

## Troubleshooting

### LoRa Not Initializing
- Check wiring connections (CS, RST, IRQ pins)
- Verify SPI pins match your hardware
- Ensure antenna is connected

### No Communication Between Devices
- Verify both devices use same frequency (check local regulations)
- Check spreading factor and bandwidth match
- Ensure antennas are properly connected
- Increase TX power if needed (max 20 dBm, check local limits)

### Upload Fails
- Verify correct COM port in `platformio.ini`
- Check USB cable (use data cable, not charge-only)
- Press reset button on M5Stick before upload
- Try reducing upload speed: add `upload_speed = 115200` to platformio.ini

## Next Steps

1. **Add More Data:** Send sensor readings, button states, etc.
2. **Implement ACK Protocol:** Add acknowledgment for reliable delivery
3. **Multi-Device Network:** Support multiple senders/receivers
4. **Power Management:** Add sleep modes for battery operation
5. **Encryption:** Add message encryption for security

## License

Customize as needed for your project.
