#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace photo {
constexpr uint8_t VERSION = 1; // 96-byte chunks; reject peers using other chunk layouts.
constexpr uint8_t START = 0x70, DATA = 0x71, END = 0x72, ACK = 0x73, CANCEL = 0x74;
constexpr size_t CHUNK_BYTES = 240, DATA_HEADER_BYTES = 8;
constexpr size_t START_BYTES = 18, CONTROL_BYTES = 6, ACK_BYTES = 9;
constexpr size_t MAX_PACKET_BYTES = DATA_HEADER_BYTES + CHUNK_BYTES;
constexpr uint32_t MAX_IMAGE_BYTES = 1024 * 1024;
constexpr uint16_t START_INDEX = 0xffff, END_INDEX = 0xfffe;
static_assert(MAX_PACKET_BYTES <= 255, "SX127x payload limit");
enum class Status : uint8_t { ok, invalid, noMemory, outOfOrder, corrupt, busy };

struct Info {
    uint32_t id = 0, length = 0, crc = 0;
    uint16_t width = 0, height = 0;
};
struct Ack { uint32_t id; uint16_t index; Status status; };
inline bool matches(const Ack& a, uint32_t id, uint16_t index) {
    return a.id == id && a.index == index;
}

inline uint16_t get16(const uint8_t* p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
inline uint32_t get32(const uint8_t* p) { return uint32_t(get16(p)) | uint32_t(get16(p + 2)) << 16; }
inline void put16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
inline void put32(uint8_t* p, uint32_t v) { put16(p, uint16_t(v)); put16(p + 2, uint16_t(v >> 16)); }
inline bool header(const uint8_t* p, size_t n, uint8_t type) {
    return p && n >= 2 && p[0] == type && p[1] == VERSION;
}
inline bool valid(const Info& i) {
    return i.id && i.length >= 4 && i.length <= MAX_IMAGE_BYTES &&
        i.width && i.width <= 640 && i.height && i.height <= 480;
}
inline bool same(const Info& a, const Info& b) {
    return a.id == b.id && a.length == b.length && a.crc == b.crc &&
        a.width == b.width && a.height == b.height;
}
inline uint16_t chunks(const Info& i) { return uint16_t((i.length + CHUNK_BYTES - 1) / CHUNK_BYTES); }
inline size_t chunkLength(const Info& i, uint16_t index) {
    const uint32_t offset = uint32_t(index) * CHUNK_BYTES;
    return offset >= i.length ? 0 : (i.length - offset < CHUNK_BYTES ? i.length - offset : CHUNK_BYTES);
}
inline uint32_t crc32(const uint8_t* data, size_t n) {
    uint32_t crc = 0xffffffff;
    while (n--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}
inline void encodeControl(uint8_t* out, uint8_t type, uint32_t id) {
    out[0] = type; out[1] = VERSION; put32(out + 2, id);
}
inline void encodeInfo(uint8_t* out, const Info& i) {
    encodeControl(out, START, i.id); put32(out + 6, i.length);
    put32(out + 10, i.crc); put16(out + 14, i.width); put16(out + 16, i.height);
}
inline bool decodeInfo(const uint8_t* p, size_t n, Info& i) {
    if (n != START_BYTES || !header(p, n, START)) return false;
    i = {get32(p + 2), get32(p + 6), get32(p + 10), get16(p + 14), get16(p + 16)};
    return valid(i);
}
inline size_t encodeData(uint8_t* out, const Info& i, uint16_t index, const uint8_t* image) {
    const size_t n = chunkLength(i, index);
    if (!n) return 0;
    encodeControl(out, DATA, i.id); put16(out + 6, index);
    memcpy(out + DATA_HEADER_BYTES, image + uint32_t(index) * CHUNK_BYTES, n);
    return DATA_HEADER_BYTES + n;
}
inline void encodeAck(uint8_t* out, const Ack& a) {
    encodeControl(out, ACK, a.id); put16(out + 6, a.index); out[8] = uint8_t(a.status);
}
inline bool decodeAck(const uint8_t* p, size_t n, Ack& a) {
    if (n != ACK_BYTES || !header(p, n, ACK) || p[8] > uint8_t(Status::busy)) return false;
    a = {get32(p + 2), get16(p + 6), Status(p[8])};
    return a.id != 0;
}

// Storage belongs to the caller. Only sequential, exact-length chunks advance
// the transfer; identical duplicates are ACKed again after a lost ACK.
class Assembly {
public:
    bool begin(const Info& info, uint8_t* storage, size_t capacity) {
        if (!valid(info) || !storage || capacity < info.length) return false;
        info_ = info; data_ = storage; next_ = 0;
        return true;
    }
    Status accept(const uint8_t* p, size_t n) {
        if (!data_ || n <= DATA_HEADER_BYTES || n > MAX_PACKET_BYTES || !header(p, n, DATA) ||
            get32(p + 2) != info_.id) return Status::invalid;
        const uint16_t index = get16(p + 6);
        const size_t length = chunkLength(info_, index);
        if (!length || n != DATA_HEADER_BYTES + length) return Status::invalid;
        if (index > next_) return Status::outOfOrder;
        uint8_t* dest = data_ + uint32_t(index) * CHUNK_BYTES;
        if (index < next_) return memcmp(dest, p + DATA_HEADER_BYTES, length) ? Status::corrupt : Status::ok;
        memcpy(dest, p + DATA_HEADER_BYTES, length);
        ++next_;
        return Status::ok;
    }
    Status finish() const {
        if (!data_ || next_ != chunks(info_)) return Status::outOfOrder;
        if (crc32(data_, info_.length) != info_.crc || data_[0] != 0xff || data_[1] != 0xd8 ||
            data_[info_.length - 2] != 0xff || data_[info_.length - 1] != 0xd9) return Status::corrupt;
        return Status::ok;
    }
    const Info& info() const { return info_; }
    uint16_t next() const { return next_; }
private:
    Info info_{};
    uint8_t* data_ = nullptr;
    uint16_t next_ = 0;
};
} // namespace photo
