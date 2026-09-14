#include "snes/core/Emulator.hpp"
#include "snes/core/Sdd1.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace snes::core;
namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

std::vector<uint8_t> Image(size_t size = 0x600000) {
    std::vector<uint8_t> rom(size);
    for (size_t p = 0; p < size; ++p) rom[p] = uint8_t(p >> 16);
    std::fill_n(rom.begin() + 0x7fc0, 64, 0);
    std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
    rom[0x7fc0] = 'T'; rom[0x7fd5] = 0x32; rom[0x7fd6] = 0x45;
    rom[0x7fd7] = 13; rom[0x7fd8] = 3; rom[0x7fd9] = 1;
    rom[0x7fdc] = 0xff; rom[0x7fdd] = 0xff; rom[0x7ffd] = 0x80;
    rom[0] = 0x80; rom[1] = 0xfe; // BRA to self.
    return rom;
}

std::unique_ptr<Emulator> Load(const std::vector<uint8_t>& rom) {
    auto emu = std::make_unique<Emulator>();
    std::string error;
    if (!emu->LoadCartridge(rom, &error)) throw std::runtime_error(error);
    return emu;
}

void DecoderVectors() {
    // Digests of complete outputs, including odd lengths and all context modes.
    constexpr uint32_t expected[] = {
        0x7bd0036e, 0x72d5042a, 0x46e81486, 0x5f2cd4bf,
        0xf5ebf5a6, 0x2d4bf27e, 0xc46b0020, 0x1a2e5d2d,
        0x991ea7c5, 0xf2e5a48d, 0x28741861, 0x7498a2c4,
        0x319c80e3, 0x1af8e407, 0xf55461e7, 0xb601a7b4
    };
    std::vector<uint8_t> input(524300);
    uint32_t rng = 1;
    for (auto& byte : input) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        byte = uint8_t(rng);
    }
    for (unsigned mode = 0; mode < 16; ++mode) {
        input[0] = uint8_t((input[0] & 15) | (mode << 4));
        uint32_t digest = 2166136261u;
        for (unsigned length : {1u, 2u, 17u, 64u, 257u, 65536u}) {
            const auto output = DecompressSdd1([&](uint32_t p) { return input.at(p); }, length);
            Check(output.size() == length, "Decoder output length");
            for (auto byte : output) { digest ^= byte; digest *= 16777619u; }
        }
        Check(digest == expected[mode], "Decoder output digest");
        const auto zeros = DecompressSdd1([mode](uint32_t p) { return uint8_t(p ? 0 : mode << 4); }, 65536);
        Check(std::all_of(zeros.begin(), zeros.end(), [](uint8_t b) { return b == 0; }), "Zero runs");
    }
}

void MappingAndSaves() {
    auto rom = Image();
    auto emu = Load(rom);
    auto& bus = emu->GetBus();
    Check(emu->LoadedCartridge()->Header().mapping == MappingType::Sdd1, "S-DD1 mapper detection");
    for (unsigned window = 0; window < 4; ++window) {
        const unsigned address = 0xc01234 + (window << 20);
        Check(bus.Read(address) == window * 16, "Default MMC bank");
        for (unsigned bank = 0; bank < 8; ++bank) {
            bus.Write(0x804804 + window, uint8_t(0xf8 | bank));
            Check(bus.Read(0x004804 + window) == (0xf8 | bank), "MMC register mirrors and readback");
            Check(bus.Read(address) == (bank < 6 ? bank : bank - 2) * 16, "MMC ROM line mirrors");
        }
    }
    Check(bus.Read(0x038123) == rom[0x18123], "Lower LoROM window");
    Check(bus.Read(0x838123) == rom[0x18123], "Upper LoROM window");
    Check(bus.Read(0x601234) == rom[0x1234], "Fixed full bank window");
    Check(bus.Read(0x718123) == rom[0x118123], "ROM above SRAM window");
    for (unsigned address : {0x401234u, 0x408123u, 0x206123u}) {
        bus.SetOpenBus(0xa9);
        Check(bus.Read(address) == 0xa9, "Unmapped board addresses");
    }
    bus.Write(0x700123, 0x57);
    Check(bus.Read(0xa06123) == 0x57, "SRAM alias");
    bus.Write(0xbf6124, 0x83);
    const auto saved = emu->LoadedCartridge()->SramData();
    std::vector<uint8_t> bytes(saved.begin(), saved.end());
    Check(emu->LoadCartridge(rom), "Cartridge reload");
    emu->LoadSram(bytes);
    Check(bus.Read(0x700123) == 0x57 && bus.Read(0x700124) == 0x83, "Battery save round trip");
    Check(bus.Read(0x4804) == 0 && bus.Read(0x4807) == 3 && bus.Read(0x4801) == 0, "Reload resets registers");
    bus.Write(0x7e1234, 0xb6);
    Check(bus.Read(0x7e1234) == 0xb6, "WRAM priority");
    rom[0x7fd8] = 0;
    Check(emu->LoadCartridge(rom), "SRAM-less image loads");
    bus.SetOpenBus(0x64);
    Check(bus.Read(0x700123) == 0x64, "Unpopulated SRAM is open bus");
}

void DmaDecompression() {
    auto rom = Image();
    // Header $C0 followed by zero codewords: packed output is entirely zero.
    std::fill(rom.begin() + 0x100000, rom.begin() + 0x180000, 0);
    rom[0x100000] = 0xc0;
    auto emu = Load(rom);
    auto& bus = emu->GetBus();
    auto& dma = emu->GetDma();
    dma.SetClockCallback({}); // Isolate data-path checks from video timing.
    std::vector<std::pair<uint32_t, uint8_t>> writes;
    bus.Map(0, 0, 0x2100, 0x21ff, [](uint32_t, uint8_t) { return uint8_t(0x69); },
        [&](uint32_t p, uint8_t b) { writes.emplace_back(p, b); });
    constexpr unsigned pattern[8][4] = {
        {0,0,0,0}, {0,1,0,1}, {0,0,0,0}, {0,0,1,1},
        {0,1,2,3}, {0,1,0,1}, {0,0,0,0}, {0,0,1,1}
    };
    for (unsigned channel = 0; channel < 8; ++channel) {
        for (unsigned mode = 0; mode < 8; ++mode) {
            auto& ch = dma.Channel(channel);
            ch.writeControl(uint8_t(mode | 8)); ch.targetAddress = 0x20;
            ch.sourceBank = 0xd0; ch.sourceAddress = 0; ch.transferSize = 17;
            bus.Write(0x4800, uint8_t(1u << channel));
            bus.Write(0x4801, 0xff);
            Check(bus.Read(0xd00000) == 0xc0, "Ordinary CPU reads stay compressed");
            writes.clear(); dma.EnableDma(uint8_t(1u << channel));
            Check(dma.RunDma() == 16 + 17 * 8, "Decompression DMA clocks");
            Check(writes.size() == 17, "Decompression DMA byte count");
            for (unsigned i = 0; i < writes.size(); ++i)
                Check(writes[i].second == 0 && writes[i].first == 0x2120 + pattern[mode][i % 4], "DMA transfer pattern");
            Check(ch.sourceAddress == 0 && ch.transferSize == 0, "Fixed source and completed count");
            Check(bus.Read(0x4801) == (0xff ^ (1u << channel)), "Only completed channel disarms");
        }
    }
    auto& ch = dma.Channel(0);
    for (unsigned scenario = 0; scenario < 3; ++scenario) {
        ch.writeControl(scenario == 2 ? 0 : 8); ch.sourceAddress = 0; ch.transferSize = 1;
        bus.Write(0x4800, scenario == 0 ? 2 : 1); bus.Write(0x4801, scenario == 1 ? 2 : 1);
        writes.clear(); dma.EnableDma(1); dma.RunDma();
        Check(writes.size() == 1 && writes[0].second == 0xc0, "Both enables and fixed addressing are required");
    }
    ch.writeControl(8); ch.sourceAddress = 0; ch.transferSize = 0;
    bus.Write(0x4800, 1); bus.Write(0x4801, 1);
    writes.clear(); dma.EnableDma(1);
    Check(dma.RunDma() == 16 + 65536 * 8 && writes.size() == 65536, "Zero count transfers 64 KiB");
    Check(std::all_of(writes.begin(), writes.end(), [](auto w) { return w.second == 0; }), "Complete 64 KiB output");

    // Reverse DMA writes SRAM without consuming a compressed stream.
    ch.writeControl(0x88); ch.sourceBank = 0x70; ch.sourceAddress = 0; ch.transferSize = 1;
    bus.Write(0x4801, 1); dma.EnableDma(1); dma.RunDma();
    Check(bus.Read(0x700000) == 0x69 && bus.Read(0x4801) == 1, "Reverse DMA bypasses decoder");
}

void ExpandedImages() {
    for (size_t size : {0x800000u, 0xc00000u}) {
        auto rom = Image(size);
        auto emu = Load(rom);
        auto& bus = emu->GetBus();
        Check(emu->LoadedCartridge()->Header().mapping == MappingType::DecompressedSdd1, "Expanded image detection");
        const size_t half = size / 2;
        Check(bus.Read(0x008123) == rom[0x123], "Expanded reset window");
        Check(bus.Read(0x408123) == rom[0x200123], "Expanded upper half window");
        Check(bus.Read(0x401123) == rom[half + 0x201123], "Expanded lower half window");
        if (size > 0x800000) {
            Check(bus.Read(0xc08123) == rom[half + 0x123], "Expanded high bank upper window");
            Check(bus.Read(0xc01123) == rom[half + 0x401123], "Expanded high bank lower window");
        } else {
            bus.SetOpenBus(0x57);
            Check(bus.Read(0xc08123) == 0x57 && bus.Read(0xc01123) == 0x57, "Absent expanded bank is open bus");
        }
        Check(emu->LoadedCartridge()->SramData().empty(), "Expanded layout has no SRAM decode");
        bus.Write(0x7e1123, 0x52);
        Check(bus.Read(0x7e1123) == 0x52, "Expanded layout preserves WRAM");
    }
}

void CompressedBankBoundary() {
    auto rom = Image();
    std::fill(rom.begin() + 0x200000, rom.begin() + 0x201000, 0);
    std::fill(rom.begin() + 0x500000, rom.begin() + 0x501000, 0xff);
    rom[0x4ffffe] = 0xc0; rom[0x4fffff] = 0;
    auto cart = Cartridge::FromRomImage(rom, nullptr, nullptr, nullptr);
    Check(cart.has_value(), "Boundary fixture loads");
    cart->Write(0x4804, 4); cart->Write(0x4805, 2);
    cart->Write(0x4800, 1); cart->Write(0x4801, 1);
    const auto output = cart->BeginDma(0, 0xcffffe, 257, true, false);
    Check(output.size() == 257 && std::all_of(output.begin(), output.end(), [](uint8_t b) { return b == 0; }),
          "Compressed input crosses the MMC boundary into the selected ROM bank");
}
}

int main() {
    try { DecoderVectors(); MappingAndSaves(); DmaDecompression(); ExpandedImages(); CompressedBankBoundary(); }
    catch (const std::exception& e) { std::fprintf(stderr, "%s\n", e.what()); return 1; }
    std::puts("S-DD1 decoder, mapper, DMA, and save checks passed");
}
