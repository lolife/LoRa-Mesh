# LoRa GPS and ENV telemetry bridge

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

Both displays show `Light N lux`; the receiver publishes `light_lux` to
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
All available measurements travel in one packet every ten seconds. GPS and ENV
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

Both units start with the display and backlight off. Their internal accelerometers
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
