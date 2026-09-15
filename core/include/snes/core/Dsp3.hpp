#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace snes::core {

// Command-level model of the cartridge's hex-map and bitstream processor.
class Dsp3 {
public:
    Dsp3() = default;
    void Reset();
    uint8_t Read(uint16_t address);
    void Write(uint16_t address, uint8_t value);
    static bool Selects(uint32_t address) noexcept;

private:
    enum class Phase : uint8_t {
        Command, Finish, Window, Index, Direction, Move, MoveIndex,
        Coordinate, Zero, Absorb, DoubleZero, RomStart, RomRead,
        PlanarCount, PlanarData, PlanarRead, DecodeCount, DecodeLength,
        Decode, Origin, SearchRange, SearchCell, Terrain, Cost,
        Solve, ResultRange, ResultCell, ResultCost
    };
    enum class DecodePhase : uint8_t {
        SymbolPrefix, SymbolValue, TreeSize, TreeLength, Prefix, Suffix,
        CopySize, CopyOffset
    };
    struct Cell {
        uint8_t terrain = 0;
        uint8_t cost = 0;
        int16_t weight = 0;
    };
    struct Ring {
        int16_t x = 0, y = 0;
        unsigned low = 0, high = 0, radius = 0, steps = 0, side = 0;
    };

    void Transfer();
    void Idle();
    uint16_t Linear(unsigned x, unsigned y) const;
    void Move(int direction, int16_t& x, int16_t& y, bool wrap);
    void StartRing(bool results);
    bool NextRing();
    void EmitCell(bool results);
    void SolvePaths();
    void Decode();
    bool Bits(unsigned count, uint16_t& value);
    static uint16_t DataRom(unsigned index);

    Phase phase_ = Phase::Command;
    uint16_t data_ = 0x80;
    uint8_t status_ = 0x84;
    uint16_t index_ = 0, count_ = 0;
    uint8_t width_ = 0, height_ = 0;
    int16_t moveX_ = 0, moveY_ = 0;
    uint16_t x_ = 0, y_ = 0;
    std::array<uint8_t, 8> pixels_{};
    std::array<uint8_t, 8> planes_{};

    DecodePhase decodePhase_ = DecodePhase::SymbolPrefix;
    std::array<uint16_t, 512> symbols_{};
    std::array<uint16_t, 8> offsets_{};
    std::array<uint8_t, 8> lengths_{};
    uint16_t symbolsLeft_ = 0, outputsLeft_ = 0, symbol_ = 0;
    uint16_t bitWord_ = 0, partialBits_ = 0;
    unsigned bitsAvailable_ = 0, bitsNeeded_ = 0;
    unsigned symbolPrefix_ = 0, treeEntries_ = 0, prefixBits_ = 0, prefix_ = 0;
    unsigned copyBits_ = 0;

    std::array<Cell, 8192> cells_{};
    Ring ring_{};
    int16_t originX_ = 0, originY_ = 0;
    unsigned searchedRadius_ = 0, returnedRadius_ = 0;
    uint16_t cell_ = 0;
};

} // namespace snes::core
