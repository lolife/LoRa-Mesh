#pragma once

#include <cstdint>
#include <cstring>

#pragma pack(push, 1)
struct gpsData {
    float latitude;
    float longitude;
    float altitude;
    float speed;
    int   sats;
};

struct envData {
    float temperature;
    float humidity;
    float pressure;
    float gas_resistance;
    int32_t iaq;
    int32_t iaq_q;
};

struct loraStatus {
    uint8_t type;
    uint32_t seq;
    float snr;
    int32_t batt;
};

template <typename PayloadT>
struct loraDataPacket {
    uint8_t type;
    uint32_t seq;
    float snr;
    int32_t batt;
    PayloadT payload;
};
#pragma pack(pop)

static constexpr uint8_t LORA_PKT_LOCATION = 0x01;
static constexpr uint8_t LORA_PKT_ACK      = 0x02;
static constexpr uint8_t LORA_PKT_ENV      = 0x03;

template <typename PayloadT>
struct LoraPayloadTraits;

template <>
struct LoraPayloadTraits<gpsData> {
    static constexpr uint8_t packetType = LORA_PKT_LOCATION;
    static constexpr const char *name = "gpsData";
};

template <>
struct LoraPayloadTraits<envData> {
    static constexpr uint8_t packetType = LORA_PKT_ENV;
    static constexpr const char *name = "envData";
};

static constexpr int LORA_PAYLOAD_KIND_GPS = 1;
static constexpr int LORA_PAYLOAD_KIND_ENV = 2;

#ifndef LORA_TX_PAYLOAD_KIND
#define LORA_TX_PAYLOAD_KIND LORA_PAYLOAD_KIND_GPS
#endif

template <int PayloadKind>
struct LoraPayloadByKind;

template <>
struct LoraPayloadByKind<LORA_PAYLOAD_KIND_GPS> {
    using type = gpsData;
};

template <>
struct LoraPayloadByKind<LORA_PAYLOAD_KIND_ENV> {
    using type = envData;
};

static_assert(
    LORA_TX_PAYLOAD_KIND == LORA_PAYLOAD_KIND_GPS ||
    LORA_TX_PAYLOAD_KIND == LORA_PAYLOAD_KIND_ENV,
    "Unsupported LORA_TX_PAYLOAD_KIND");

using loraTxPayload = typename LoraPayloadByKind<LORA_TX_PAYLOAD_KIND>::type;
using loraTxPacket = loraDataPacket<loraTxPayload>;
using loraGpsPacket = loraDataPacket<gpsData>;
using loraEnvPacket = loraDataPacket<envData>;

template <typename PayloadT>
constexpr uint8_t loraPayloadType() {
    return LoraPayloadTraits<PayloadT>::packetType;
}

template <typename PayloadT>
constexpr loraDataPacket<PayloadT> makeLoraDataPacket(uint32_t seq,
                                                      float snr,
                                                      int32_t batt,
                                                      const PayloadT &payload) {
    return { loraPayloadType<PayloadT>(), seq, snr, batt, payload };
}

inline bool decodeLoraGpsPacket(const char *buffer, int packetSize, loraGpsPacket *out) {
    if (packetSize != static_cast<int>(sizeof(loraGpsPacket))) {
        return false;
    }
    memcpy(out, buffer, sizeof(loraGpsPacket));
    return out->type == LORA_PKT_LOCATION;
}

inline bool decodeLoraEnvPacket(const char *buffer, int packetSize, loraEnvPacket *out) {
    if (packetSize != static_cast<int>(sizeof(loraEnvPacket))) {
        return false;
    }
    memcpy(out, buffer, sizeof(loraEnvPacket));
    return out->type == LORA_PKT_ENV;
}

inline bool decodeLoraStatusPacket(const char *buffer, int packetSize, loraStatus *out) {
    if (packetSize != static_cast<int>(sizeof(loraStatus))) {
        return false;
    }
    memcpy(out, buffer, sizeof(loraStatus));
    return out->type == LORA_PKT_ACK;
}

// To add a new payload type:
// 1) Define the payload struct above.
// 2) Add a LoraPayloadTraits<YourType> specialization with a packetType.
// 3) Add a LORA_PAYLOAD_KIND_* constant and LoraPayloadByKind specialization.
static_assert(sizeof(gpsData) == 20, "gpsData size changed");
static_assert(sizeof(loraStatus) == 13, "loraStatus size changed");
static_assert(sizeof(loraGpsPacket) == 33, "loraGpsPacket size changed");
