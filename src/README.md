# LoRa GPS and ENV telemetry bridge

The sender reads TinyGPSPlus on Serial1 and probes an ENV3 unit on Port A I2C.
GPS and ENV operate independently: failure or absence of either sensor does not
prevent reporting the other. `ENV3` is enabled in the sender build; remove that
flag for a GPS-only build. GPS RX/TX default to GPIO 8/9 and can be overridden with
`GPS_RX_PIN` and `GPS_TX_PIN`.

Every five seconds, the sender puts all available measurements into one 64-byte
LoRa telemetry packet, using the current LoRa-Mesh-RAK wire format. Each sensor
expires after 60 seconds without a valid refresh. Empty telemetry is not sent.
The existing GPS Kalman filter is retained; nanodegree wire coordinates do not
increase the precision of that filter's float input. ACK retries reuse one packet
and sequence number, and only a matching sequence completes the send.

The receiver displays both sensors and publishes them together to ThingsBoard
at most once every 30 seconds. Pressure is published as `pressure` in hPa;
`altitude` contains GPS altitude in meters. It accepts the current RAK telemetry
format, previous 58/63-byte combined formats, and this project's original
33-byte GPS and 37-byte ENV packets. Upgrade the receiver before the sender:
old receiver firmware cannot decode the new combined packet.

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

## Build and check

Credentials stay in the ignored `include/credentials.h`. USB/OTA upload settings
remain in `platformio.ini`.

```sh
pio run -e sender -e receiver -e sender-M5Basic
python3 test/host/run.py
```

Host checks cover telemetry availability flags, current and legacy wire formats,
malformed packets, ACK sequence preservation, queue limits, low-memory retries,
completion pacing, mesh relay deduplication, origin identity, inactive-peer fallback,
and display motion detection, timeout extension, failed reads, and timer rollover.
Device reception, sensor wiring, display wake sensitivity, and mesh propagation
still require a hardware check after flashing.
