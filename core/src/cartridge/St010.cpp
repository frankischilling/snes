#include "snes/core/St010.hpp"
#include "St010Tables.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdlib>

namespace snes::core {
namespace {
int16_t Signed(uint16_t value) { return std::bit_cast<int16_t>(value); }
int16_t Wrap(int value) { return Signed(static_cast<uint16_t>(value)); }
uint16_t Word(std::span<const uint8_t> ram, unsigned offset) {
    return uint16_t(ram[offset] | (uint16_t(ram[offset + 1]) << 8));
}
int16_t SWord(std::span<const uint8_t> ram, unsigned offset) { return Signed(Word(ram, offset)); }
int32_t Dword(std::span<const uint8_t> ram, unsigned offset) {
    return std::bit_cast<int32_t>(uint32_t(Word(ram, offset)) | (uint32_t(Word(ram, offset + 2)) << 16));
}
void Put(std::span<uint8_t> ram, unsigned offset, uint32_t value, unsigned bytes = 2) {
    for (unsigned i = 0; i < bytes; ++i) ram[offset + i] = uint8_t(value >> (i * 8));
}
int Sin(uint16_t angle) { return st010::Sine[angle >> 8]; }
int Cos(uint16_t angle) { return Sin(uint16_t(angle + 0x4000)); }

struct Heading { int16_t x, y; uint16_t quadrant, angle; };
Heading Compass(int16_t x, int16_t y) {
    Heading result{x, y, 0, 0};
    if (x <= 0 && y < 0) result = {Wrap(-x), Wrap(-y), 0x8000, 0};
    else if (x < 0) result = {y, Wrap(-x), 0xc000, 0};
    else if (y < 0) result = {Wrap(-y), x, 0x4000, 0};
    while (result.x > 31 || result.y > 31) {
        if (result.x > 1) result.x >>= 1;
        if (result.y > 1) result.y >>= 1;
    }
    const unsigned index = (unsigned(result.x) & 31) * 32 + (unsigned(result.y) & 31);
    result.angle = uint16_t(((st010::Arc[index] << 8) | result.quadrant) ^ 0x8000);
    if (x == 0 && y < 0) result.quadrant = 0x4000;
    return result;
}

void Navigate(std::span<uint8_t> ram) {
    const int16_t dx = Wrap(SWord(ram, 0xc0) - (Dword(ram, 0xc4) >> 16));
    const int16_t dy = Wrap(SWord(ram, 0xc2) - (Dword(ram, 0xc8) >> 16));
    const int16_t turn = Wrap(Compass(dx, dy).angle - Word(ram, 0xcc));
    uint16_t angle = Word(ram, 0xcc);
    if (uint16_t(turn) & 0xff00) angle = uint16_t(angle + (turn < 0 ? -640 : 640));
    // The signed 16-bit absolute value wraps at $8000 before shifting.
    const uint16_t penalty = uint16_t((turn < 0 ? Wrap(-turn) : turn) >> 4);
    unsigned speed = Word(ram, 0xd4);
    if (penalty < 256) speed = std::min(speed + Word(ram, 0xd6), unsigned(Word(ram, 0xd8)));
    else speed = unsigned(std::max(0, int(speed) - int(penalty)));
    const uint32_t x = uint32_t(Dword(ram, 0xc4)) - uint32_t((Sin(angle) >> 5) * int(speed >> 8) * 2);
    const uint32_t y = uint32_t(Dword(ram, 0xc8)) - uint32_t((Cos(angle) >> 5) * int(speed >> 8) * 2);
    Put(ram, 0xc4, x & 0x1fffffff, 4);
    Put(ram, 0xc8, y & 0x1fffffff, 4);
    Put(ram, 0xcc, angle); Put(ram, 0xce, uint16_t(turn));
    Put(ram, 0xd0, uint16_t(dx)); Put(ram, 0xd2, uint16_t(dy)); Put(ram, 0xd4, speed);
    const bool vertical = (Word(ram, 0xda) & 0x8000) != 0;
    if (std::abs(int(dx)) < (vertical ? 8 : 128) && std::abs(int(dy)) < (vertical ? 128 : 8)) {
        Put(ram, 0xc0, Word(ram, 0xde));
        Put(ram, 0xc2, Word(ram, 0xe0) & 0xfff);
        Put(ram, 0xda, Word(ram, 0xe0) & 0x8000 ? 0xffff : 0);
        Put(ram, 0xdc, Word(ram, 0xdc) | 8);
    }
}
}

bool St010::Selects(uint32_t address) noexcept {
    const unsigned bank = (address >> 16) & 0xff, offset = address & 0xffff;
    return (bank >= 0x60 && bank <= 0x67 && offset < 0x4000) ||
           (bank >= 0x68 && bank <= 0x6f && offset < 0x8000);
}

uint8_t St010::Read(uint32_t address, std::span<const uint8_t> ram) const {
    if (!(address & 0x80000)) return 0x80;
    const unsigned offset = address & 0xfff;
    if (offset == 0x20) return command_;
    if (offset == 0x21) return execute_;
    return ram[offset];
}

void St010::Write(uint32_t address, uint8_t value, std::span<uint8_t> ram) {
    if (!(address & 0x80000)) { enabled_ = true; return; }
    const unsigned offset = address & 0xfff;
    if (enabled_ && offset == 0x20) command_ = value;
    if (enabled_ && offset == 0x21) execute_ = value;
    else ram[offset] = value;
    if (execute_ & 0x80) {
        Execute(ram);
        command_ = execute_ = 0;
    }
}

void St010::Execute(std::span<uint8_t> ram) {
    switch (command_) {
    case 1: {
        const auto heading = Compass(SWord(ram, 0), SWord(ram, 2));
        Put(ram, 6, Word(ram, 2));
        Put(ram, 0, uint16_t(heading.x)); Put(ram, 2, uint16_t(heading.y));
        Put(ram, 4, heading.quadrant); Put(ram, 0x10, heading.angle);
        break;
    }
    case 2: {
        struct Driver { uint16_t place, id; };
        std::array<Driver, 32> drivers{};
        const unsigned count = std::min(unsigned(Word(ram, 0x24)), 32u);
        for (unsigned i = 0; i < count; ++i) drivers[i] = {Word(ram, 0x40 + 2 * i), Word(ram, 0x80 + 2 * i)};
        std::stable_sort(drivers.begin(), drivers.begin() + count,
                         [](const auto& a, const auto& b) { return a.place > b.place; });
        for (unsigned i = 0; i < count; ++i) {
            Put(ram, 0x40 + 2 * i, drivers[i].place); Put(ram, 0x80 + 2 * i, drivers[i].id);
        }
        break;
    }
    case 3:
        Put(ram, 0x10, uint32_t(int64_t(SWord(ram, 0)) * SWord(ram, 4) * 2), 4);
        Put(ram, 0x14, uint32_t(int64_t(SWord(ram, 2)) * SWord(ram, 4) * 2), 4);
        break;
    case 4: {
        const int16_t x = Wrap(std::abs(int(SWord(ram, 0))));
        const int16_t y = Wrap(std::abs(int(SWord(ram, 2))));
        const bool xMajor = uint16_t(x) >= uint16_t(y);
        const int64_t sum = int64_t(xMajor ? x : y) * 0x3d78 + int64_t(xMajor ? y : x) * 0x1976;
        Put(ram, 0x10, (uint32_t(sum * 4) + 0x8000u) >> 16);
        break;
    }
    case 5: Navigate(ram); break;
    case 6:
        Put(ram, 0x10, uint32_t(int64_t(SWord(ram, 0)) * SWord(ram, 2) * 2), 4);
        break;
    case 7: {
        const auto angle = Word(ram, 0);
        for (unsigned row = 0; row < 176; ++row) {
            const unsigned divisor = 44 + row * 5;
            const int scale = int((39424 + divisor / 2) / divisor);
            const int a = (scale * Cos(angle)) >> 15;
            const int b = (scale * Sin(angle)) >> 15;
            Put(ram, 0xf0 + row * 2, uint16_t(a)); Put(ram, 0x510 + row * 2, uint16_t(a));
            Put(ram, 0x250 + row * 2, uint16_t(b)); Put(ram, 0x3b0 + row * 2, uint16_t(b ? ~b : 0));
        }
        Put(ram, 0, angle >> 8);
        break;
    }
    case 8: {
        const int x = SWord(ram, 0), y = SWord(ram, 2);
        const auto angle = Word(ram, 4);
        Put(ram, 0x10, uint16_t(((y * Sin(angle)) >> 15) + ((x * Cos(angle)) >> 15)));
        Put(ram, 0x12, uint16_t(((y * Cos(angle)) >> 15) - ((x * Sin(angle)) >> 15)));
        break;
    }
    default: break;
    }
}
} // namespace snes::core
