// Cx4 cartridge command processor and graphics work RAM.
#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace snes::core {

class Cx4 {
public:
    explicit Cx4(std::span<const uint8_t> rom) : rom_(rom) {}
    static bool Selects(uint32_t address) noexcept { return (address & 0x40e000) == 0x006000; }
    void Reset() { ram_.fill(0); }
    uint8_t Read(uint32_t address) const;
    void Write(uint32_t address, uint8_t value);

private:
    std::span<const uint8_t> rom_;
    std::array<uint8_t, 8192> ram_{};

    uint8_t Byte(uint32_t offset) const { return ram_[offset & 0x1fff]; }
    uint32_t Value(uint32_t offset, unsigned bytes = 2) const;
    void Store(uint32_t offset, uint64_t value, unsigned bytes = 2);
    uint8_t RomByte(uint32_t offset) const;
    static uint32_t RomOffset(uint32_t address) { return ((address & 0xff0000) >> 1) | (address & 0x7fff); }
    void Execute(uint8_t command);
    void BuildSprites();
    void Affine(unsigned padding);
    void TransformVertices();
    void Wireframe(bool clear);
    void Dissolve();
    void Wave();
    void Trapezoid();
    void Plot4(uint32_t index, unsigned bit, uint8_t color);
    std::array<int16_t, 2> Rotate(int16_t x, int16_t y, int16_t z,
                                unsigned rx, unsigned ry, unsigned rz, int16_t scale, bool perspective) const;
};

} // namespace snes::core
