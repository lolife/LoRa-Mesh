#include "photo_protocol.h"
#include <array>
#include <cassert>
#include <vector>
#include <cstdio>

using namespace photo;

static std::vector<uint8_t> image(size_t n) {
    std::vector<uint8_t> bytes(n);
    for (size_t j = 0; j < n; ++j) bytes[j] = uint8_t(j * 71);
    bytes[0] = bytes[n - 2] = 0xff; bytes[1] = 0xd8; bytes[n - 1] = 0xd9;
    return bytes;
}
static Info infoFor(const std::vector<uint8_t>& bytes) {
    return {0x78563412, uint32_t(bytes.size()), crc32(bytes.data(), bytes.size()), 640, 480};
}

int main() {
    const uint8_t known[] = "123456789";
    assert(crc32(known, 9) == 0xcbf43926);
    auto original = image(2 * CHUNK_BYTES + 19);
    Info info = infoFor(original), parsed;
    std::array<uint8_t, MAX_PACKET_BYTES> packet{};
    encodeInfo(packet.data(), info);
    assert(packet[2] == 0x12 && packet[5] == 0x78); // Explicit little-endian wire contract.
    assert(decodeInfo(packet.data(), START_BYTES, parsed) && same(info, parsed));
    assert(!decodeInfo(packet.data(), START_BYTES - 1, parsed));
    assert(!decodeInfo(packet.data(), START_BYTES + 1, parsed));
    packet[1]++;
    assert(!decodeInfo(packet.data(), START_BYTES, parsed));
    packet[1] = 1; // Legacy peer implicitly uses 240-byte chunks.
    assert(!decodeInfo(packet.data(), START_BYTES, parsed));
    packet[1] = 2; // Previous trial uses 64-byte chunks.
    assert(!decodeInfo(packet.data(), START_BYTES, parsed));
    Info invalid = info; invalid.length = MAX_IMAGE_BYTES + 1; assert(!valid(invalid));
    invalid = info; invalid.id = 0; assert(!valid(invalid));
    invalid = info; invalid.width = 641; assert(!valid(invalid));
    invalid = info; invalid.height = 481; assert(!valid(invalid));
    invalid = info; invalid.length = 3; assert(!valid(invalid));

    // Guard bytes detect writes outside the receiver's exact allocation.
    std::vector<uint8_t> storage(original.size() + 2, 0xa5);
    Assembly rx;
    assert(!rx.begin(info, nullptr, info.length));
    assert(!rx.begin(info, storage.data() + 1, info.length - 1));
    assert(rx.begin(info, storage.data() + 1, info.length));
    assert(rx.finish() == Status::outOfOrder);

    size_t n = encodeData(packet.data(), info, 1, original.data());
    assert(rx.accept(packet.data(), n) == Status::outOfOrder && rx.next() == 0);
    n = encodeData(packet.data(), info, 0, original.data());
    assert(n == MAX_PACKET_BYTES);
    assert(rx.accept(packet.data(), n - 1) == Status::invalid);
    assert(rx.accept(packet.data(), n) == Status::ok && rx.next() == 1);
    // Lost ACK: retransmitting a chunk is harmless, and must not advance twice.
    for (int attempt = 0; attempt < 4; ++attempt)
        assert(rx.accept(packet.data(), n) == Status::ok && rx.next() == 1);
    packet[DATA_HEADER_BYTES] ^= 1;
    assert(rx.accept(packet.data(), n) == Status::corrupt && rx.next() == 1);
    packet[DATA_HEADER_BYTES] ^= 1;
    packet[2] ^= 1;
    assert(rx.accept(packet.data(), n) == Status::invalid); // Different photo / stale packet.
    packet[2] ^= 1;
    put16(packet.data() + 6, 0xffff);
    assert(rx.accept(packet.data(), n) == Status::invalid);
    assert(encodeData(packet.data(), info, chunks(info), original.data()) == 0);

    n = encodeData(packet.data(), info, 1, original.data());
    assert(rx.accept(packet.data(), n) == Status::ok);
    n = encodeData(packet.data(), info, 2, original.data());
    assert(n == DATA_HEADER_BYTES + 19);
    assert(rx.accept(packet.data(), n + 1) == Status::invalid);
    assert(rx.accept(packet.data(), n) == Status::ok);
    assert(rx.finish() == Status::ok);
    assert(rx.finish() == Status::ok); // Lost completion ACK: idempotent validation.
    assert(storage.front() == 0xa5 && storage.back() == 0xa5);
    assert(memcmp(storage.data() + 1, original.data(), original.size()) == 0);
    storage[40] ^= 1;
    assert(rx.finish() == Status::corrupt);

    // Full-sized images, exact multiples, minimum image and a short final chunk.
    for (size_t length : {size_t(4), CHUNK_BYTES, CHUNK_BYTES + 1, size_t(MAX_IMAGE_BYTES)}) {
        auto bytes = image(length);
        const Info large = infoFor(bytes);
        std::vector<uint8_t> dest(length);
        Assembly receiver;
        assert(receiver.begin(large, dest.data(), dest.size()));
        for (uint16_t index = 0; index < chunks(large); ++index) {
            n = encodeData(packet.data(), large, index, bytes.data());
            assert(receiver.accept(packet.data(), n) == Status::ok);
        }
        assert(receiver.finish() == Status::ok && bytes == dest);
    }
    // A matching CRC does not make a non-JPEG acceptable.
    auto badJpeg = original; badJpeg[1] = 0;
    const Info badInfo = infoFor(badJpeg);
    Assembly bad;
    assert(bad.begin(badInfo, storage.data() + 1, badInfo.length));
    for (uint16_t index = 0; index < chunks(badInfo); ++index) {
        n = encodeData(packet.data(), badInfo, index, badJpeg.data());
        assert(bad.accept(packet.data(), n) == Status::ok);
    }
    assert(bad.finish() == Status::corrupt);

    Ack ack{info.id, END_INDEX, Status::ok}, decoded{};
    encodeAck(packet.data(), ack);
    assert(decodeAck(packet.data(), ACK_BYTES, decoded));
    assert(matches(decoded, info.id, END_INDEX));
    assert(!matches(decoded, info.id + 1, END_INDEX));
    assert(!matches(decoded, info.id, START_INDEX));
    assert(!decodeAck(packet.data(), ACK_BYTES - 1, decoded));
    assert(!decodeAck(packet.data(), ACK_BYTES + 1, decoded));
    packet[8] = 255;
    assert(!decodeAck(packet.data(), ACK_BYTES, decoded));
    assert(!header(nullptr, 10, START));
    assert(rx.accept(nullptr, 0) == Status::invalid);
    puts("photo protocol / reassembly checks passed");
}
