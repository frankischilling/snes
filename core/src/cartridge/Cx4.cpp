#include "snes/core/Cx4.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <numbers>

namespace snes::core {
namespace {
int16_t Signed16(uint32_t value) { return std::bit_cast<int16_t>(uint16_t(value)); }
int32_t Signed32(uint32_t value) { return std::bit_cast<int32_t>(value); }
int16_t Float16(double value) {
    if (!std::isfinite(value)) return 0;
    return Signed16(uint32_t(int32_t(std::fmod(std::trunc(value), 65536.0))));
}

// The command ROM's trigonometric coefficients have fifteen fractional bits.
int Sine(unsigned angle) {
    static const auto table = [] {
        std::array<int, 512> values{};
        for (unsigned i = 0; i < 256; ++i) {
            values[i] = std::min(32767, int(std::sin(i * std::numbers::pi / 256.0) * 32768.0));
            values[i + 256] = -values[i];
        }
        return values;
    }();
    return table[angle & 511];
}
int Cosine(unsigned angle) { return Sine(angle + 128); }

struct LineStep { int16_t x, y; uint16_t count; };
LineStep Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    const int dx = Signed16(uint16_t(x1) - uint16_t(x0));
    const int dy = Signed16(uint16_t(y1) - uint16_t(y0));
    const int length = std::max(std::abs(dx), std::abs(dy));
    if (!length) return {0, 0, 0};
    return {int16_t(dx * 256 / length), int16_t(dy * 256 / length), uint16_t(length + 1)};
}
}

uint32_t Cx4::Value(uint32_t offset, unsigned bytes) const {
    uint32_t value = 0;
    for (unsigned i = 0; i < bytes; ++i) value |= uint32_t(Byte(offset + i)) << (i * 8);
    return value;
}

void Cx4::Store(uint32_t offset, uint64_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) ram_[(offset + i) & 0x1fff] = uint8_t(value >> (i * 8));
}

uint8_t Cx4::RomByte(uint32_t offset) const {
    if (rom_.empty()) return 0xff;
    size_t address = offset, size = rom_.size(), base = 0;
    while (address >= size) {
        const size_t bit = std::bit_floor(address);
        address -= bit;
        if (size > bit) { size -= bit; base += bit; }
    }
    return rom_[base + address];
}

uint8_t Cx4::Read(uint32_t address) const {
    return (address & 0xffff) == 0x7f5e ? 0 : Byte(address);
}

void Cx4::Write(uint32_t address, uint8_t value) {
    ram_[address & 0x1fff] = value;
    if ((address & 0xffff) == 0x7f4f) Execute(value);
    if ((address & 0xffff) == 0x7f47) {
        const uint32_t source = RomOffset(Value(0x1f40, 3));
        const unsigned destination = Value(0x1f45), count = Value(0x1f43);
        for (unsigned i = 0; i < count; ++i) ram_[(destination + i) & 0x1fff] = RomByte(source + i);
    }
}

std::array<int16_t, 2> Cx4::Rotate(int16_t x, int16_t y, int16_t z,
        unsigned rx, unsigned ry, unsigned rz, int16_t scale, bool perspective) const {
    // Rotation commands use a 128-step circle and a fixed camera distance.
    constexpr double angleUnit = -3.14159265 / 64.0;
    const double ax = rx * angleUnit, ay = ry * angleUnit, az = rz * angleUnit;
    const double depth = double(z) - (perspective ? 149.0 : 0.0);
    const double by = y * std::cos(ax) - depth * std::sin(ax);
    const double bz = y * std::sin(ax) + depth * std::cos(ax);
    const double cx = x * std::cos(ay) + bz * std::sin(ay);
    const double cz = -x * std::sin(ay) + bz * std::cos(ay);
    const double dx = cx * std::cos(az) - by * std::sin(az);
    const double dy = cx * std::sin(az) + by * std::cos(az);
    if (perspective) return {Float16(dx * scale / (144.0 * (cz + 149.0)) * 149.0),
                             Float16(dy * scale / (144.0 * (cz + 149.0)) * 149.0)};
    return {Float16(dx * scale / 256.0), Float16(dy * scale / 256.0)};
}

void Cx4::Execute(uint8_t command) {
    if (Byte(0x1f4d) == 0x0e && command < 0x40 && !(command & 3)) {
        Store(0x1f80, command / 4, 1);
        return;
    }
    const int x = Signed16(Value(0x1f80)), y = Signed16(Value(0x1f83));
    switch (command) {
    case 0x00:
        switch (Byte(0x1f4d)) {
        case 0: BuildSprites(); break;
        case 3: Affine(0); break;
        case 5: TransformVertices(); break;
        case 7: Affine(64); break;
        case 8: Wireframe(false); break;
        case 11: Dissolve(); break;
        case 12: Wave(); break;
        default: break;
        }
        break;
    case 0x01: Wireframe(true); break;
    case 0x05: {
        const auto denominator = Value(0x1f83);
        Store(0x1f80, denominator ? ((65536u / denominator) * Value(0x1f81)) >> 8 : 65536u);
        break;
    }
    case 0x0d: {
        const double length = std::sqrt(double(x) * x + double(y) * y);
        const double factor = length ? Signed16(Value(0x1f86)) / length : 0;
        Store(0x1f89, uint16_t(Float16(x * factor * 0.98)));
        Store(0x1f8c, uint16_t(Float16(y * factor * 0.99)));
        break;
    }
    case 0x10: case 0x13: {
        const int radius = command == 0x10 ? y : int(Value(0x1f83));
        const unsigned shift = command == 0x10 ? 16 : 8;
        // The product wraps in the command engine's 32-bit accumulator.
        const int dx = Signed32(uint32_t(radius * Cosine(unsigned(x))) * 2) >> shift;
        const int dy = Signed32(uint32_t(radius * Sine(unsigned(x))) * 2) >> shift;
        Store(0x1f86, uint32_t(dx), 3);
        Store(0x1f89, uint32_t(command == 0x10 ? dy - (dy >> 6) : dy), 3);
        break;
    }
    case 0x15: Store(0x1f80, uint32_t(std::sqrt(double(x) * x + double(y) * y))); break;
    case 0x1f: {
        int angle = x ? int(std::atan(double(y) / x) * 512.0 / (2 * 3.14159265)) + (x < 0 ? 256 : 0) :
                        (y > 0 ? 128 : 384);
        Store(0x1f86, unsigned(angle) & 511);
        break;
    }
    case 0x22: Trapezoid(); break;
    case 0x25: Store(0x1f80, uint64_t(Value(0x1f80, 3)) * Value(0x1f83, 3), 3); break;
    case 0x2d: {
        const auto result = Rotate(Signed16(Value(0x1f81)), Signed16(Value(0x1f84)), Signed16(Value(0x1f87)),
            Byte(0x1f89), Byte(0x1f8a), Byte(0x1f8b), Signed16(Value(0x1f90)), false);
        Store(0x1f80, uint16_t(result[0])); Store(0x1f83, uint16_t(result[1]));
        break;
    }
    case 0x40: {
        unsigned sum = 0;
        for (unsigned i = 0; i < 2048; ++i) sum += Byte(i);
        Store(0x1f80, sum);
        break;
    }
    case 0x54: {
        const uint32_t bits = Value(0x1f80, 3);
        const int64_t value = int64_t(bits) - ((bits & 0x800000) ? 0x1000000 : 0);
        Store(0x1f83, uint64_t(value * value), 6);
        break;
    }
    case 0x5c: {
        constexpr uint8_t pattern[]{0,0,0,255, 255,255,0,255, 0,0,0,255, 255,255,0,0,
            255,255,0,0, 128,255,255,127, 0,128,0,255, 127,0,255,127,
            255,127,255,255, 0,0,1,255, 255,254,0,1, 0,255,254,0};
        std::copy(std::begin(pattern), std::end(pattern), ram_.begin());
        break;
    }
    case 0x89: Store(0x1f80, 0x054336, 3); break;
    default: break;
    }
}

void Cx4::BuildSprites() {
    unsigned slot = Byte(0x626);
    for (unsigned i = std::min(slot, 128u); i < 128; ++i) ram_[i * 4 + 1] = 0xe0;
    const unsigned total = Byte(0x620), scrollX = Value(0x621), scrollY = Value(0x623);
    auto emit = [&](int sx, int sy, uint8_t tile, uint8_t attributes, bool large) {
        Store(slot * 4, uint8_t(sx), 1); Store(slot * 4 + 1, uint8_t(sy), 1);
        Store(slot * 4 + 2, tile, 1); Store(slot * 4 + 3, attributes, 1);
        const unsigned shift = (slot & 3) * 2, high = 0x200 + (slot >> 2);
        ram_[high] = uint8_t((ram_[high] & ~(3u << shift)) | ((((unsigned(sx) >> 8) & 1) | (large ? 2 : 0)) << shift));
        ++slot;
    };
    for (unsigned sprite = 0; sprite < total && slot < 128; ++sprite) {
        const unsigned base = 0x220 + sprite * 16;
        const int sx = Signed16(Value(base) - scrollX), sy = Signed16(Value(base + 2) - scrollY);
        const uint8_t tile = Byte(base + 5), attributes = Byte(base + 4) | Byte(base + 6);
        uint32_t source = RomOffset(Value(base + 7, 3));
        const unsigned parts = RomByte(source++);
        if (!parts) { emit(sx, sy, tile, attributes, true); continue; }
        for (unsigned part = 0; part < parts && slot < 128; ++part, source += 4) {
            const uint8_t flags = RomByte(source);
            const bool large = (flags & 0x20) != 0;
            int dx = std::bit_cast<int8_t>(RomByte(source + 1));
            int dy = std::bit_cast<int8_t>(RomByte(source + 2));
            if (attributes & 0x40) dx = -dx - (large ? 16 : 8);
            if (attributes & 0x80) dy = -dy - (large ? 16 : 8);
            const int px = Signed16(unsigned(sx + dx)), py = Signed16(unsigned(sy + dy));
            if (px >= -16 && px <= 272 && py >= -16 && py <= 224)
                emit(px, py, uint8_t(tile + RomByte(source + 3)), attributes ^ (flags & 0xc0), large);
        }
    }
}

void Cx4::Plot4(uint32_t index, unsigned bit, uint8_t color) {
    for (unsigned plane = 0; plane < 4; ++plane) {
        if (color & (1u << plane)) ram_[(index + (plane & 1) + (plane >> 1) * 16) & 0x1fff] |= uint8_t(bit);
    }
}

void Cx4::Affine(unsigned padding) {
    const unsigned angle = Value(0x1f80), width = Byte(0x1f89) & ~7u, height = Byte(0x1f8c) & ~7u;
    const int sx = std::min(Value(0x1f8f), 32767u), sy = std::min(Value(0x1f92), 32767u);
    int a, b, c, d;
    if (angle == 0) { a = sx; b = c = 0; d = sy; }
    else if (angle == 128) { a = d = 0; b = -sy; c = sx; }
    else if (angle == 256) { a = -sx; b = c = 0; d = -sy; }
    else if (angle == 384) { a = d = 0; b = sy; c = -sx; }
    else {
        a = Signed16(unsigned((Cosine(angle) * sx) >> 15));
        b = Signed16(unsigned(-((Sine(angle) * sy) >> 15)));
        c = Signed16(unsigned((Sine(angle) * sx) >> 15));
        d = Signed16(unsigned((Cosine(angle) * sy) >> 15));
    }
    std::fill_n(ram_.begin(), std::min<size_t>((width + padding / 4) * height / 2, ram_.size()), 0);
    const int cx = Signed16(Value(0x1f83)), cy = Signed16(Value(0x1f86));
    uint32_t lineX = uint32_t(int64_t(cx) * (4096 - a - b)), lineY = uint32_t(int64_t(cy) * (4096 - c - d));
    unsigned destination = 0;
    for (unsigned y = 0; y < height; ++y) {
        uint32_t px = lineX, py = lineY;
        for (unsigned x = 0; x < width; ++x, px += unsigned(a), py += unsigned(c)) {
            if ((px >> 12) < width && (py >> 12) < height) {
                const unsigned pixel = (py >> 12) * width + (px >> 12);
                const auto color = uint8_t(Byte(0x600 + pixel / 2) >> ((pixel & 1) * 4));
                Plot4(destination + (x / 8) * 32, 0x80 >> (x & 7), color);
            }
        }
        destination += width * 4 + 2 + padding;
        if (destination & 16) destination &= ~16u;
        else destination -= width * 4 + padding;
        lineX += unsigned(b); lineY += unsigned(d);
    }
}

void Cx4::Dissolve() {
    const unsigned width = Byte(0x1f89), height = Byte(0x1f8c);
    const int cx = Signed16(Value(0x1f80)), cy = Signed16(Value(0x1f83));
    const int dx = Signed16(Value(0x1f86)), dy = Signed16(Value(0x1f8f));
    const uint32_t startX = uint32_t(cx * (256 - dx)), startY = uint32_t(cy * (256 - dy));
    std::fill_n(ram_.begin(), std::min<size_t>(width * height / 2, ram_.size()), 0);
    unsigned source = 0x600;
    for (unsigned row = 0; row < height; ++row) {
        const uint32_t py = startY + row * unsigned(dy);
        for (unsigned col = 0; col < width; ++col) {
            const uint32_t px = startX + col * unsigned(dx);
            if ((px >> 8) < width && (py >> 8) < height && (py >> 8) * width + (px >> 8) < 8192) {
                const unsigned dest = (py >> 11) * width * 4 + (px >> 11) * 32 + ((py >> 8) & 7) * 2;
                Plot4(dest, 0x80 >> ((px >> 8) & 7), Byte(source) >> ((col & 1) * 4));
            }
            if (col & 1) ++source;
        }
    }
}

void Cx4::TransformVertices() {
    const unsigned count = Value(0x1f80);
    const unsigned rx = Byte(0x1f83), ry = Byte(0x1f86), rz = Byte(0x1f89), scale = Byte(0x1f8c);
    for (unsigned i = 0; i < std::min(count, 512u); ++i) {
        const unsigned v = i * 16;
        const auto point = Rotate(Signed16(Value(v + 1)), Signed16(Value(v + 5)), Signed16(Value(v + 9)), rx, ry, rz, int16_t(scale), true);
        Store(v + 1, uint16_t(point[0] + 128)); Store(v + 5, uint16_t(point[1] + 80));
    }
    for (unsigned slot : {0x600u, 0x608u}) { Store(slot, 23); Store(slot + 2, 96); Store(slot + 5, 64); }
    const unsigned edges = std::min(Value(0xb00), 1024u);
    for (unsigned i = 0; i < edges; ++i) {
        const unsigned from = Byte(0xb02 + i * 2) * 16, to = Byte(0xb03 + i * 2) * 16;
        const auto step = Line(Signed16(Value(from + 1)), Signed16(Value(from + 5)),
                               Signed16(Value(to + 1)), Signed16(Value(to + 5)));
        Store(0x600 + i * 8, step.count ? step.count : 1);
        Store(0x602 + i * 8, uint16_t(step.x)); Store(0x605 + i * 8, uint16_t(step.y));
    }
}

void Cx4::Wireframe(bool clear) {
    if (clear) std::fill_n(ram_.begin() + 0x300, 2304, 0);
    const auto source = Value(0x1f80, 3);
    const uint32_t list = RomOffset(source), bank = source & 0xff0000;
    const unsigned count = Byte(0x295), scale = Byte(0x1f90);
    uint16_t previous = 0;
    auto point = [&](uint16_t address) {
        const uint32_t offset = RomOffset(bank | address);
        auto word = [&](unsigned at) { return Signed16(unsigned(RomByte(offset + at)) * 256 + RomByte(offset + at + 1)); };
        return Rotate(word(0), word(2), word(4), Byte(0x1f86), Byte(0x1f87), Byte(0x1f88), int16_t(scale), false);
    };
    for (unsigned i = 0; i < count; ++i) {
        const uint32_t edge = list + i * 5;
        const uint16_t first = uint16_t(unsigned(RomByte(edge)) * 256 + RomByte(edge + 1));
        const uint16_t second = uint16_t(unsigned(RomByte(edge + 2)) * 256 + RomByte(edge + 3));
        const auto from = point(first == 0xffff ? previous : first), to = point(second);
        if (second != 0xffff) previous = second;
        const auto step = Line(Signed16(from[0] + 48), Signed16(from[1] + 48), Signed16(to[0] + 48), Signed16(to[1] + 48));
        int px = (int(from[0]) + 48) * 256, py = (int(from[1]) + 48) * 256;
        const uint8_t color = RomByte(edge + 4);
        for (unsigned k = 0; k < std::max(unsigned(step.count), 1u); ++k, px += step.x, py += step.y) {
            if (px < 256 || px >= 24576 || py < 256 || py >= 24576) continue;
            const unsigned x = unsigned(px) >> 8, y = unsigned(py) >> 8;
            const unsigned dest = 0x300 + (y / 8) * 192 + (x / 8) * 16 + (y & 7) * 2;
            const unsigned bit = 0x80 >> (x & 7);
            for (unsigned plane = 0; plane < 2; ++plane)
                ram_[dest + plane] = uint8_t((ram_[dest + plane] & ~bit) | ((color & (1 << plane)) ? bit : 0));
        }
    }
}

void Cx4::Wave() {
    unsigned wave = Byte(0x1f83);
    for (unsigned column = 0; column < 32; ++column) {
        for (unsigned pair = 0; pair < 4; ++pair) {
            int height = -std::bit_cast<int8_t>(Byte(0xb00 + wave)) - 16;
            const unsigned mask = 0xc0c0 >> (pair * 2);
            for (unsigned row = 0; row < 40; ++row, ++height) {
                const unsigned destination = column * 16 + (row / 8) * 512 + (row & 7) * 2;
                const unsigned pattern = height < 0 ? 0 : height < 8 ? Value(0xa00 + (column & 1) * 16 + height * 2) : 0xff00;
                Store(destination, (Value(destination) & ~mask) | (pattern & mask));
            }
            wave = (wave + 1) & 127;
        }
    }
}

void Cx4::Trapezoid() {
    const unsigned angles[]{Value(0x1f8c), Value(0x1f8f)};
    int32_t tangent[2];
    for (unsigned side = 0; side < 2; ++side) tangent[side] = Cosine(angles[side]) ?
        int32_t(int64_t(Sine(angles[side])) * 65536 / Cosine(angles[side])) : INT32_MIN;
    int y = Signed16(Value(0x1f83) - Value(0x1f89));
    const int offset = int(Value(0x1f86)) - int(Value(0x1f80)), width = int(Value(0x1f93));
    for (unsigned row = 0; row < 225; ++row, y = Signed16(unsigned(y + 1))) {
        int left = 1, right = 0;
        if (y >= 0) {
            left = Signed16(unsigned((Signed32(uint32_t(tangent[0]) * unsigned(y)) >> 16) + offset));
            right = Signed16(unsigned((Signed32(uint32_t(tangent[1]) * unsigned(y)) >> 16) + offset + width));
            if (left < 0 && right < 0) { left = 1; right = 0; }
            else { left = std::max(left, 0); right = std::max(right, 0); }
            if (left > 255 && right > 255) { left = 255; right = 254; }
            else { left = std::min(left, 255); right = std::min(right, 255); }
        }
        Store(0x800 + row, unsigned(left), 1); Store(0x900 + row, unsigned(right), 1);
    }
}

} // namespace snes::core
