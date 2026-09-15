// snes emulator
// SA-1 normal DMA and packed/unpacked character conversion.
#include "snes/core/Sa1.hpp"

#include <algorithm>

namespace snes::core {

void Sa1::StartDma() {
    const uint8_t control = registers_[0x30];
    const uint8_t source = control & 3;
    const bool bwram = (control & 4) != 0;
    // Same-device transfers and the fourth source selection are not defined.
    if (source == 3 || (source == 1 && bwram) || (source == 2 && !bwram)) return;
    dma_.source = RegisterAddress(0x32);
    dma_.destination = RegisterAddress(0x35);
    dma_.remaining = RegisterWord(0x38);
    // A zero-length request completes without touching either memory port.
    if (!dma_.remaining) {
        dma_.active = false;
        sa1Flags_ |= 0x20;
        return;
    }
    dma_.sourceDevice = source;
    dma_.destinationBwram = bwram;
    dma_.clocksRemaining = (source == 0 && !bwram) ? 2 : 4;
    dma_.active = true;
}

void Sa1::StepDma() {
    const uint32_t budget = static_cast<uint32_t>(requestedClocks_ - executedClocks_);
    const uint8_t clocks = static_cast<uint8_t>(std::min<uint32_t>(budget, dma_.clocksRemaining));
    Clock(clocks);
    dma_.clocksRemaining -= clocks;
    if (dma_.clocksRemaining) return;

    uint8_t value = 0;
    if (dma_.sourceDevice == 0) {
        uint32_t offset;
        if (RomOffset(dma_.source, offset)) value = ReadRom(offset, regs().mdr);
        else value = regs().mdr;
        dma_.source = (dma_.source + 1) & 0xffffff;
    } else if (dma_.sourceDevice == 1) {
        value = ReadBwram(dma_.source);
        dma_.source = (dma_.source + 1) & 0x3ffff;
    } else {
        value = iram_[dma_.source & 0x7ff];
        dma_.source = (dma_.source + 1) & 0x7ff;
    }
    // DMA owns these memory ports; CPU write-enable masks gate CPU writes.
    if (dma_.destinationBwram) {
        WriteBwram(dma_.destination, value, Side::Sa1, true);
        dma_.destination = (dma_.destination + 1) & 0x3ffff;
    } else {
        iram_[dma_.destination & 0x7ff] = value;
        dma_.destination = (dma_.destination + 1) & 0x7ff;
    }
    --dma_.remaining;
    StoreAddress(0x32, dma_.source);
    StoreAddress(0x35, dma_.destination);
    registers_[0x38] = static_cast<uint8_t>(dma_.remaining);
    registers_[0x39] = static_cast<uint8_t>(dma_.remaining >> 8);
    if (!dma_.remaining) {
        dma_.active = false;
        sa1Flags_ |= 0x20;
    } else {
        dma_.clocksRemaining = (dma_.sourceDevice == 0 && !dma_.destinationBwram) ? 2 : 4;
    }
}

uint8_t Sa1::ConversionDepth() const noexcept {
    switch (registers_[0x31] & 3) {
    case 0: return 8;
    case 1: return 4;
    default: return 2;
    }
}

void Sa1::WritePlanarTile(const std::array<uint8_t, 64>& pixels, uint16_t destination) {
    const unsigned depth = ConversionDepth();
    for (unsigned plane = 0; plane < depth; ++plane) {
        for (unsigned y = 0; y < 8; ++y) {
            uint8_t value = 0;
            for (unsigned x = 0; x < 8; ++x)
                value |= ((pixels[y * 8 + x] >> plane) & 1) << (7 - x);
            const unsigned offset = (plane / 2) * 16 + y * 2 + (plane & 1);
            iram_[(destination + offset) & 0x7ff] = value;
        }
    }
}

void Sa1::StartConversion() {
    const unsigned rowBytes = (8 * ConversionDepth()) << ((registers_[0x31] >> 2) & 7);
    conversionSource_ = (RegisterAddress(0x32) & 0x3ffff) & ~(rowBytes - 1);
    const unsigned bufferSize = 16 * ConversionDepth();
    conversionDestination_ = static_cast<uint16_t>((RegisterWord(0x35) & 0x7ff) & ~(bufferSize - 1));
    conversionActive_ = true;
    conversionTile_ = UINT32_MAX;
    ConvertBitmapTile(0);
    cpuFlags_ |= 0x20;
}

void Sa1::ConvertBitmapTile(uint32_t tile) {
    const unsigned depth = ConversionDepth();
    const unsigned width = 1u << ((registers_[0x31] >> 2) & 7);
    const uint32_t tileRow = tile / width;
    const uint32_t tileColumn = tile % width;
    std::array<uint8_t, 64> pixels{};
    for (unsigned y = 0; y < 8; ++y) {
        const uint32_t sourceRow = conversionSource_ + (tileRow * 8 + y) * width * depth + tileColumn * depth;
        for (unsigned x = 0; x < 8; ++x) {
            const unsigned bit = x * depth;
            pixels[y * 8 + x] = static_cast<uint8_t>((ReadBwram(sourceRow + bit / 8) >> (bit & 7)) & ((1u << depth) - 1));
        }
    }
    const uint16_t destination = static_cast<uint16_t>(conversionDestination_ + (tile & 1) * depth * 8);
    WritePlanarTile(pixels, destination);
    conversionTile_ = tile;
}

uint8_t Sa1::ReadConverted(uint32_t offset) {
    const unsigned tileBytes = 8 * ConversionDepth();
    const uint32_t position = (offset - conversionSource_) & 0x3ffff;
    const uint32_t tile = position / tileBytes;
    if (tile != conversionTile_) ConvertBitmapTile(tile);
    return iram_[(conversionDestination_ + (tile & 1) * tileBytes + position % tileBytes) & 0x7ff];
}

void Sa1::PushConversionRows() {
    // Each completed eight-pixel register row is visible immediately. The two
    // register halves alternate while the output walks eight planar rows.
    const unsigned registerBase = 0x40 + (conversionRows_ & 1) * 8;
    const unsigned tileBytes = 8 * ConversionDepth();
    const unsigned base = (RegisterWord(0x35) & 0x7ff) & ~(tileBytes * 2 - 1);
    for (unsigned plane = 0; plane < ConversionDepth(); ++plane) {
        uint8_t value = 0;
        for (unsigned pixel = 0; pixel < 8; ++pixel)
            value |= ((registers_[registerBase + pixel] >> plane) & 1) << (7 - pixel);
        const unsigned offset = conversionBuffer_ * tileBytes + (plane / 2) * 16 +
                                conversionRows_ * 2 + (plane & 1);
        iram_[(base + offset) & 0x7ff] = value;
    }
    if (++conversionRows_ == 8) {
        conversionRows_ = 0;
        conversionBuffer_ ^= 1;
    }
}

} // namespace snes::core
