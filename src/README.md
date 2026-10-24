# LoRa GPS and ENV telemetry bridge

## CoreS3 camera / local LoRa photo experiment

### Enabling or disabling photos

Photos are currently disabled for both devices in `platformio.ini`:

```ini
[photos]
enabled = 0
```

Set `enabled = 1` to re-enable the experiment, then rebuild and flash both
devices. With `0`, photo capture, transfer, the photo overlay, progress bar,
and photo Offline indicator are compiled out. The receiver returns to its
telemetry display and motion-based sleep behavior, and brightness commands no
longer have a photo-specific minimum. Sensor telemetry, networking, and OTA
remain enabled. The photo settings below apply when this flag is enabled.

### Switching radio settings

Change the single `profile` line under `[radio]` in `platformio.ini`:

```ini
[radio]
profile = LORA_PROFILE_SF9_125
```

| Profile | Spreading factor | Bandwidth |
| --- | --- | --- |
| `LORA_PROFILE_SF9_125` (default) | SF9 | 125 kHz |
| `LORA_PROFILE_SF7_125` | SF7 | 125 kHz |
| `LORA_PROFILE_SF7_250` | SF7 | 250 kHz |

The selection applies to `sender` and `receiver`. Rebuild with
`pio run -e sender -e receiver`, then flash both endpoints with matching builds
when ready. Building does not flash anything. The startup log prints the actual
SF and bandwidth. Unknown profile names fail compilation. Other settings,
including the 15-second photo capture interval, stay independent of the profile.

### Photo behavior

When enabled, the `sender` and `receiver` builds include point-to-point photos
over the **433 MHz, SF9, 125 kHz, coding rate 4/5** radio setup. There
is no photo upload, Wi-Fi photo transport, SD storage, or LoRa routing in this
experiment. Existing sensor telemetry, MQTT, ESP-NOW and OTA remain enabled;
photo bytes are never forwarded to those services. The radio resumes continuous receive
immediately after every transmission and after draining a received packet. This
keeps short ACK preambles from being missed during the post-TX delay
or between main-loop polls. Polling reads the radio RxDone register over SPI before calling the
driver parser, so empty polls never switch radio modes during an in-flight packet.
Reception does not depend on a connected DIO0 wire. Sender ACK timeouts log
received-packet, CRC-error, and DIO-mismatch counts plus radio mode/configuration;
valid or malformed START ACKs are logged separately. This firmware uses the
shared RF configuration on both endpoints and does not establish its regulatory compliance.

Update **receiver first, then sender**:

```sh
pio run -e receiver -e sender
# Flash to the existing OTA addresses configured in platformio.ini:
pio run -e receiver -t upload
pio run -e sender -t upload
```

The serial monitor currently uses PlatformIO's default DTR/RTS behavior, with
no post-OTA delay, to test Upload and Monitor without those overrides. A receiver
that returned `OK` for OTA was previously observed with its new slot marked
`ESP_OTA_IMG_ABORTED` and the previous slot still valid; repeating Upload with an
already-open monitor booted the new build successfully. A five-second delay was
tried after that observation and has now been removed for this test.

USB port paths can be reassigned when a board is unplugged or moved. Before
any USB flash, verify the hardware MAC with `esptool read-mac`; the current
sender is **F4:12:FA:BA:1A:10**, and the receiver is **30:ED:A0:D4:BC:08**.
Do not identify a board by a previously recorded `/dev/cu.usbmodem...` path
alone. Flashing sender firmware onto the receiver enables the sender's
30-second display timeout instead of the receiver's always-on photo view.

The sender captures its first image **15 seconds after setup finishes**, or
when its screen is tapped while idle. The current test defaults are
**240x176** resolution and software JPEG **quality 30** on the encoder's 1..100
scale (previously 320x240 and 160x120 at quality 30, 320x240 at quality 90,
and initially 640x480 at quality 100). Capture uses RGB565 in PSRAM and
then encodes to JPEG; no automatic quality reduction takes place. The camera
uses fixed gain 20 on the GC0308 driver's 0..30 scale, automatic exposure with
a brighter target (+2), automatic white balance, and at least two seconds of
fresh frames to settle after initialization. Gain and AE target use the
[supported GC0308 controls](https://github.com/espressif/esp32-camera/blob/master/sensors/gc0308.c);
the driver's generic brightness and gain-ceiling controls are unsupported.
Higher gain can increase noise; adjust the settings below after viewing a new
image in the same lighting. A previous 320x240 quality-90 local capture produced a
10,652-byte JPEG (45 chunks), versus 95,640 bytes in the initial VGA test;
sizes still depend on the scene. The
receiver keeps the complete JPEG in PSRAM and scales a view to fit its
entire 320x240 screen, preserving the aspect ratio. The receiver display stays
on continuously and uses the last completed sky image as the background for a
DTunnel-style detail screen. Three large rows show the latest sender temperature
(Fahrenheit), humidity (% RH), and receiver local time, with labels on the left
and values on the right. Temperature gets the largest row, followed by humidity
and time; text scales to fit each row. Yellow text with a two-pixel black outline
provides contrast over light and dark scenes. The
clock uses WiFi NTP and US Central time with automatic daylight saving changes;
it shows `--:--` until synchronized. Missing or expired ENV readings show dashes.
The overlay refreshes when readings or the displayed minute change, restoring
the underlying JPEG first so old text does not linger. If no new image has been
successfully drawn for four capture intervals (currently 60 seconds), a large
`Offline` replaces both temperature and humidity; the time and previous image
remain visible. Only a newly drawn photo resets this deadline. Telemetry, photo
chunks, failed transfers, duplicate END packets, and old-image redraws do not.
The next newly drawn photo restores the readings immediately. Before the first
photo, the screen stays black until the same deadline, then shows `Offline` and
the clock on black. With continuous capture (`PHOTO_INTERVAL_MS=0`), the offline
deadline uses a 15-second baseline, giving a 60-second timeout.
The receiver enforces a minimum backlight brightness of 80/255, including
MQTT brightness commands; a background check restores that minimum if needed.
During reception, a two-pixel red bar grows left to right along the bottom edge,
proportional to the bytes accepted for the current photo. Retransmitted chunks
never advance it twice. The previous photo and overlay stay visible behind it;
only newly filled bar pixels are drawn on ordinary progress updates. The bar is
removed when a photo completes or the transfer is cancelled, rejected at final
validation, or times out. Progress does not reset the Offline deadline.
Transfer progress and errors remain available in serial logs. The sender
retains its existing telemetry display and motion-based sleep behavior.
The 15-second startup delay starts **capture**, not display: the receiver
stays black while awaiting a complete, validated JPEG, or shows `Offline` once
the initial four-interval deadline expires.
In a local two-device serial test, one 640x480 quality-100 photo was 95,640
bytes (399 chunks) and took **727 seconds (12 minutes 7 seconds)** to deliver,
including 10 recovered retries. The receiver passed CRC/JPEG validation and
off-screen decoding, and the received photo was confirmed visible on the
physical receiver. Other scenes and radio conditions can change this time.
A subsequent 320x240 quality-90 test with gain 20 and AE target +2 delivered
a 22,034-byte image in **182 seconds (3 minutes 2 seconds)** with five
recovered retries; receiver CRC/JPEG validation and decoding succeeded.
The continuous 160x120 quality-30 test delivered seven consecutive images of
**1,811–2,182 bytes**, with **13–19-second** transfers. Including the next
capture's exposure settling and other processing, receiver refresh intervals
were **15.8–21.7 seconds** (about 18.5 seconds on average). The displayed
thumbnails were confirmed useful for testing. Scene content and retries can
change these timings.
At the previous SF9/125 kHz settings, the 320x240 quality-30 setting was reported to take about 45
seconds per transfer. Interpolating by pixel count between that result and
the smaller thumbnail test suggests roughly 29–30 seconds for the
240x176 setting at SF9/125 kHz. After unsuccessful trials of SF7/250 kHz
and SF7/125 kHz at the deployment location, SF9/125 kHz has been restored; scene content
and retries affect the actual transfer time.

Automatic captures now run **every 15 seconds, measured from the start of
the previous capture attempt**. Telemetry is also sent every 30 seconds.
There is only one photo transfer at a time and no backlog: a transfer that
exceeds 15 seconds finishes before the next capture starts. Camera
initialization, its two-second exposure settling period, JPEG encoding,
radio airtime and ACKs are included in that interval. Set `PHOTO_INTERVAL_MS`
to `0` to return to back-to-back photos; in that mode failed attempts back
off five seconds. Taps while
sending are ignored. Capture/encoding and synchronous radio transmission
briefly occupy the main loop; GPS, light, network recovery, OTA and motion
continue during ACK waits, and normal telemetry runs between photo chunks.
The camera temporarily owns internal I2C during capture and releases it before
touch/IMU/light/power servicing resumes. External ENV I2C is separate.
The previous 30-second capture mode at SF9/125 kHz was checked on the devices: consecutive capture intervals
were 30.035 and 29.978 seconds, and telemetry intervals were 30.028 and 30.006
seconds, with successful photo deliveries.

At the previous SF9/125 kHz settings, high-quality VGA could take **many minutes per
photo**. Serial logs report dimensions, JPEG byte count, number of chunks,
10% progress, elapsed seconds, retry count and final delivery/failure. These
logs, image quality and observed packet loss should guide interval/quality
tuning; no measured delivery time is promised by a successful build.

Settings are in `include/photo_config.h` and can also be overridden with build
flags. Encoder quality increases with the number, unlike the hardware-camera
`jpeg_quality` convention:

| Setting | Default | Adjustment |
| --- | --- | --- |
| `PHOTO_JPEG_QUALITY` | `30` | Higher is better quality/larger; current setting prioritizes small files |
| `PHOTO_FRAME_SIZE` | `FRAMESIZE_HQVGA` | 240x176; `FRAMESIZE_QVGA` restores 320x240, `FRAMESIZE_QQVGA` restores 160x120 |
| `PHOTO_SENSOR_GAIN` | `20` | Fixed GC0308 driver gain, 0..30; previous default was approximately 9 |
| `PHOTO_AE_LEVEL` | `2` | Automatic-exposure target, -2..+2; positive is brighter |
| `PHOTO_SETTLE_MS` | `2000` | Fresh-frame settling time after camera initialization |
| `PHOTO_DISPLAY_BRIGHTNESS` | `80` | Minimum receiver backlight, 1..255; brightness RPCs cannot turn it off |
| `PHOTO_INTERVAL_MS` | `15000` | 15-second capture interval; `0` enables back-to-back photos |
| `PHOTO_FIRST_CAPTURE_MS` | `15000` | Delay after setup |
| `ENABLE_PHOTOS` | `0` via `platformio.ini` | Controlled by the shared `[photos] enabled` flag |

The protocol uses versioned types 0x70..0x74: start metadata, binary data,
completion, ACK and cancellation. Each data packet carries a photo ID, a
chunk index, and up to 96 JPEG bytes (104 bytes including its header). Photo
protocol version 3 identifies this chunk layout; both endpoints must be updated.
Version-1 peers using 240-byte chunks and version-2 peers using 64-byte chunks
are rejected. The 96-byte trial reduces ACK overhead compared with 64-byte
chunks; transfer timing and retries still need to be measured at the deployment
location. The
radio receive buffer is now 255 bytes; telemetry wire formats remain 60/64
bytes or their existing legacy sizes. Each exchange retries up to four times
with a 2.5-second ACK timeout after transmission. Duplicate chunks are
idempotent, and ACKs must match both photo ID and chunk index. The final ACK
is repeatable if lost. Receivers check length, resolution, full-image CRC32,
JPEG markers, and off-screen JPEG decoding before replacing the previous
image. Incomplete transfers expire after 60 seconds without valid activity.
The previous completed image survives failed transfers, but all photos are
lost on reboot. Image allocation is bounded to 1 MiB per JPEG; larger images
fail explicitly rather than silently lowering quality. This version supports
the existing single-sender relationship.
If a receiver is busy with a stale transfer after a sender restart, the sender
retains the new image and retries its start every 10 seconds for up to 90
seconds while the receiver's old session expires.

Host photo checks cover packet bounds, byte order, exact/partial chunks,
wrong IDs and versions, out-of-order packets, lost-ACK duplicate handling,
full-size 1 MiB reassembly, checksum corruption and JPEG markers. The local
serial test above also verified camera capture, complete transfer, image
validation, JPEG decoding and visible receiver display. Color/orientation/
exposure, I2C restoration, receiver reset, deliberately lost completion ACKs,
and coexistence with OTA still need targeted physical testing. For a local
test, observe both serial logs, check delivery
with Wi-Fi disconnected, and interrupt a transfer: the receiver should retain
its previous image and the sender should log failure after bounded retries.

Camera wiring/capture reference:
[M5Stack CoreS3 camera](https://docs.m5stack.com/en/arduino/m5cores3/camera).
Software JPEG API:
[Espressif esp32-camera](https://github.com/espressif/esp32-camera).

The default `sender` build collects GPS position/altitude, internal LTR-553ALS-WA
light and ENV3 measurements together. GPS uses TinyGPSPlus on Serial1 at 115200
baud, RX GPIO8 / TX GPIO9 (Port B); ENV3 uses Port A. The light sensor needs no
external wiring. GPS pins can be overridden with `GPS_RX_PIN` / `GPS_TX_PIN`.
`LIGHT_SENSOR` and `ENABLE_GPS` are independent switches; GPS defaults to enabled.
`pio run -e sender-gps` builds the same sender with the light sensor disabled.
Both environments target the same sender; flash only one.

### Internal ambient-light sensor

The LTR553 uses M5Unified's internal I2C bus (SDA12/SCL11), address 0x23,
separate from ENV3's Port A bus. The driver verifies the sensor identity,
keeps proximity sensing/its IR emitter off, and enables ambient-light sensing
at 1x gain with 100 ms integration and a 200 ms measurement period.
It reads both channels together, converts them to estimated whole lux using
the two-channel ratio formula, and averages eight fresh successful readings
(roughly 1.6 seconds). It polls every 100 ms without blocking background work.
I2C errors, invalid status, old data and saturated/out-of-range samples do not
become darkness or refresh the reading. Measurements expire after five seconds
without a successful sample; recovery refills the averaging window.
Initialization and prolonged failures retry every 30 seconds.

The sender display shows `Light N lux`; the receiver publishes `light_lux` to
ThingsBoard instead of `light_raw`. Fixed 1x gain prioritizes the approximately
1–64,000 lux range over sub-lux sensitivity. These are estimates, not calibrated
lux-meter measurements. Keep the sensor window below the screen exposed in the
intended mounting position. Check covered/exposed readings, bright-light
saturation, display on/off effects, WiFi/OTA, ENV readings and ACK reception
on the physical units after flashing.

### Combined telemetry and upgrade order

Update the receiver first, then the sender. The sender emits local packet type 6,
exactly **60 bytes**, with four bytes left in the 64-byte radio budget:

| Field | Bytes |
| --- | ---: |
| Type, sequence, return SNR, sender battery | 13 |
| Availability flags | 1 |
| Latitude/longitude (signed nanodegree integers) | 16 |
| GPS altitude (float, meters) | 4 |
| Light (unsigned whole lux) | 2 |
| Existing ENV fields | 24 |
| Total | 60 |

Speed, satellites and fix quality are omitted. Availability bit 6 marks valid
altitude and bit 7 marks light; GPS and ENV retain their existing bits. Altitude
is optional independently of coordinates and expires with its own GPS timestamp.
All available measurements travel in one packet every 30 seconds. GPS and ENV
expire after 120 seconds without refresh; light expires after five seconds.
Empty telemetry is not sent. Missing GPS does not stop light or ENV telemetry.
ACK retries reuse the same serialized packet and sequence number.

The receiver publishes `latitude`, `longitude`, optional `altitude` (GPS meters),
`light_lux`, ENV values and both battery values to ThingsBoard at most once every
60 seconds. Compact packets do not publish speed, satellites or fix quality.
Pressure is `pressure` in hPa. Displays show coordinates, altitude, light and ENV
when available. The GPS Kalman filter is retained; nanodegree wire coordinates
do not increase the precision of its float input.

The receiver also accepts previous 64-byte type-5 lux packets and current/legacy
GPS/ENV formats (64/63/58/37/33 bytes). Type 4 analog packets remain rejected.
Old receivers reject type 6. The shared MeshProtocol library and ESP-NOW format
are unchanged. The 64-byte shared structure is only an in-memory adapter for
compact packets: its unused satellites/quality bytes hold lux. Radio transmission
always uses the separate packed 60-byte structure. No omitted GPS metadata is
invented or forwarded over ESP-NOW; legacy GPS packets retain their old behavior.
Light is published directly by the receiver and is not forwarded over ESP-NOW.

## Motion-controlled displays

With photos enabled, the receiver stays on and shows only the last photo.
The following motion behavior applies to the sender, and to the receiver when
photos are disabled. These units start with the display and backlight off. Their internal accelerometers
are sampled every 50 ms, including during ACK waits and background delays. Moving
or tilting a unit by an acceleration-vector change of at least 0.12 g wakes its
display at brightness 80 and immediately redraws current telemetry. Further motion
extends the wake period; 30 seconds without detected motion turns the display off.
Radio, GPS, sensors, mesh, MQTT, and OTA continue running while the display sleeps.
If the IMU is missing or reads fail, the display stays off (or times out normally).
Threshold and timing constants live in `include/display_motion.h`.

## ESP-NOW mesh

LoRa-Mesh, DTunnel and WindSens use the standalone sibling
[MeshProtocol](../../MeshProtocol/README.md) library for their role registry, wire
contracts and common mesh runtime. Oui Spy is excluded. Physical MAC addresses
map to roles; role names and ThingsBoard tokens remain stable when devices are
reassigned. Keep the application and library checkouts beside each other, and
provide the library's ignored `config/role_tokens.local.h` on a fresh installation.

The receiver uses DTunnel's current mesh node registry and 40-byte versioned
`StatusMessage` contract. Identity is selected by local MAC, with the same build
fallback rules as DTunnel. Every other registered node becomes a peer. WiFi
power saving is disabled and peers follow the station's current channel.

GPS speed (`V`) and ENV temperature (`T`) are both forwarded when available.
Messages carry origin MAC and unique nonzero IDs. Relays preserve the origin,
exclude the sending hop, and use a 64-entry deduplication cache with a three-minute
expiry. Legacy 32-byte status messages are accepted without relaying. Peer
liveness follows the source and origin MACs. If no eligible peers are active,
transmission falls back to all configured peers.

One worker performs ESP-NOW sends from a bounded queue, waiting for completion
before submitting the next packet. Receive callbacks only enqueue relays. The
mesh does not include DTunnel's separate Sky Spy gateway features. Sky Spy
`SKY1` broadcasts are recognized and skipped before status-message parsing.

### Previous SF7 / 250 kHz USB bench check (2026-10-24)

The radio profile has since been restored to SF9/125 kHz after deployment
tests with SF7/250 kHz and SF7/125 kHz proved unreliable.
At the previous SF7/250 kHz settings, with both endpoints using read-only SPI
RxDone polling, two consecutive
240x176 quality-30 photos of 2,324 and 2,291 bytes completed in four logged
seconds each, with zero retries. Capture timestamps were 15.483 seconds apart.
The receiver measured approximately -58 dBm RSSI and 9.2–10.2 dB SNR. The
previous failing location measured approximately -109 dBm RSSI and 6 dB SNR;
this bench result does not establish reliability back at that location or
isolate the software change from the signal-strength change. Retest there
before treating the faster profile as reliable for deployment.

## Firmware build stamp

Each PlatformIO build generates a fresh `FW` timestamp in US Central time,
including milliseconds and the CST/CDT suffix. The startup log and ThingsBoard
`firmware_version` attribute share the same string. The generated header lives
under each environment's build directory and is used only by `build_info.cpp`,
so incremental builds refresh the stamp without recompiling `main.cpp`, MQTT,
or the libraries. A clean removes the generated header; the next build recreates
it. This does not add upload delays or change serial monitor control signals.

## WiFi and MQTT recovery

Both sender and receiver use DTunnel's event-driven WiFi and OTA recovery. On the
receiver, WiFi disconnects, lost IP addresses, brief reconnects, and DHCP address
changes invalidate the MQTT transport. Event callbacks only record state; the
firmware loop closes TCP before resetting MQTT and retries WiFi asynchronously
after 15, 30, then 60 seconds.
Retries keep station mode and the radio enabled for ESP-NOW.

MQTT runs only on the receiver, pauses while WiFi is unavailable, and reconnects promptly
after recovery with fresh RPC and attribute subscriptions. Each connection publishes
`firmware_version`, `ip_address`, `mac_address`, `board` (the M5GFX enum name),
and `mesh_identity`, matching DTunnel. Failed attribute posts retry every 15 seconds
until successful; a reconnect publishes the current values again. Broker failures retry
every 15 seconds, slowing to 60 seconds after repeated failures. MQTT connection
attempts use a five-second socket timeout and run outside LoRa ACK waits. OTA
starts once WiFi obtains an IP, including after an offline boot, and stops while
WiFi is unavailable. The receiver remains responsible for publishing telemetry.
Both units request the highest WiFi transmit power setting (`WIFI_POWER_21dBm`),
subject to the driver's hardware/country limits, and allow 10 seconds between
incoming OTA data to tolerate temporary transfer stalls.

## Build and check

Credentials stay in the ignored `include/credentials.h`. USB/OTA upload settings
remain in `platformio.ini`.

```sh
pio run -e sender -e receiver -e sender-gps
python3 test/host/run.py
python3 test/network/run.py
```

Host checks cover telemetry availability flags, current and legacy wire formats,
malformed packets, ACK sequence preservation, queue limits, low-memory retries,
completion pacing, mesh relay deduplication, origin identity, inactive-peer fallback,
and display motion detection, timeout extension, failed reads, and timer rollover.
Network checks cover WiFi backoff, authentication failure, brief reconnects,
IP changes/loss, MQTT transport resets and retry pacing, subscriptions, OTA
lifecycle, and retry timing across timer rollover.
Device reception, sensor wiring, display wake sensitivity, and mesh propagation
still require a hardware check after flashing.
