// snes emulator
// tests/test_hardware_regressions.cpp
// Regression coverage for cartridge, PPU, DMA, and timing hardware.

#include "snes/core/Emulator.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

using namespace snes::core;
namespace {
int failures = 0;
int checks = 0;

void Check(const char* label, unsigned actual, unsigned expected) {
    ++checks;
    if (actual != expected) {
        std::fprintf(stderr, "FAIL %s: actual=$%X expected=$%X\n", label, actual, expected);
        ++failures;
    }
}

std::vector<uint8_t> Rom(size_t size, uint8_t mode, uint8_t type = 0) {
    std::vector<uint8_t> data(size, 0xea);
    const size_t header = (mode & 15) == 5 ? 0x40ffc0 :
                          (mode & 15) == 1 ? 0xffc0 : 0x7fc0;
    std::fill_n(data.begin() + header, 0x40, 0);
    std::fill_n(data.begin() + header, 21, ' ');
    data[header] = 'T';
    data[header + 0x15] = mode;
    data[header + 0x16] = type;
    data[header + 0x17] = 13;
    data[header + 0x19] = 1;
    data[header + 0x1c] = 0x34;
    data[header + 0x1d] = 0x12;
    data[header + 0x1e] = 0xcb;
    data[header + 0x1f] = 0xed;
    data[header + 0x3d] = 0x80;
    return data;
}

std::unique_ptr<Emulator> Load(const std::vector<uint8_t>& rom) {
    auto emu = std::make_unique<Emulator>();
    std::string error;
    if (!emu->LoadCartridge(rom, &error)) {
        std::fprintf(stderr, "Fixture failed to load: %s\n", error.c_str());
        std::exit(2);
    }
    return emu;
}

unsigned HCounter(MemoryBus& bus) {
    bus.Read(0x213f);
    const unsigned low = bus.Read(0x213c);
    return low | ((bus.Read(0x213c) & 1) << 8);
}

void CartridgeMapping() {
    // A prototype can have an intact reset vector and no mapper metadata.
    auto prototype = Rom(0x80000, 0x20);
    std::fill(prototype.begin() + 0x7fd5, prototype.begin() + 0x7fe0, 0xff);
    prototype[0] = 0x18; // CLC at the reset vector.
    auto prototypeEmu = Load(prototype);
    Check("Prototype inferred LoROM", unsigned(prototypeEmu->LoadedCartridge()->Header().mapping),
          unsigned(MappingType::LoRom));
    Check("Prototype reset vector is mapped", prototypeEmu->GetBus().Read(0xfffd), 0x80);
    Check("Prototype reset opcode is mapped", prototypeEmu->GetBus().Read(0x8000), 0x18);

    auto hiPrototype = Rom(0x80000, 0x21);
    std::fill(hiPrototype.begin() + 0xffd5, hiPrototype.begin() + 0xffe0, 0xff);
    hiPrototype[0x8000] = 0x78;
    hiPrototype.insert(hiPrototype.begin(), 512, 0); // Copier header.
    prototypeEmu = Load(hiPrototype);
    Check("Prototype inferred HiROM", unsigned(prototypeEmu->LoadedCartridge()->Header().mapping),
          unsigned(MappingType::HiRom));
    Check("Headered prototype reset opcode", prototypeEmu->GetBus().Read(0x8000), 0x78);

    auto rom = Rom(0x800000, 0x25);
    rom[0x008000] = 0x11;
    rom[0x408000] = 0x22;
    auto emu = Load(rom);
    Check("ExHiROM fixture mapping", unsigned(emu->LoadedCartridge()->Header().mapping),
          unsigned(MappingType::ExHiRom));
    Check("ExHiROM $00:8000", emu->GetBus().Read(0x008000), 0x22);
    Check("ExHiROM $80:8000", emu->GetBus().Read(0x808000), 0x11);

    rom = Rom(0x300000, 0x20);
    rom[0] = 0x11;
    rom[0x200000] = 0x33;
    emu = Load(rom);
    Check("3 MiB LoROM $40:8000", emu->GetBus().Read(0x408000), 0x33);
    Check("3 MiB LoROM $60:8000 mirror", emu->GetBus().Read(0x608000), 0x33);

    rom = Rom(0x700000, 0x25);
    rom[0x008000] = 0x11;
    rom[0x408000] = 0x44;
    rom[0x608000] = 0x66;
    emu = Load(rom);
    for (unsigned bank : {0x30u, 0x70u})
        Check("7 MiB ExHiROM tail mirror", emu->GetBus().Read((bank << 16) | 0x8000), 0x66);
    for (unsigned bank : {0x80u, 0xc0u})
        Check("7 MiB ExHiROM first half", emu->GetBus().Read((bank << 16) | 0x8000), 0x11);

    // A power-of-two image retains its ordinary whole-image mirrors.
    rom = Rom(0x100000, 0x20);
    rom[0x012345] = 0x57;
    emu = Load(rom);
    for (unsigned address : {0x02a345u, 0x22a345u, 0xa2a345u})
        Check("1 MiB LoROM mirror", emu->GetBus().Read(address), 0x57);
}

void DspMapping() {
    auto emu = Load(Rom(0x80000, 0x20, 3));
    Check("Small LoROM DSP data $20:8000", emu->GetBus().Read(0x208000), 0x80);
    emu = Load(Rom(0x200000, 0x20, 3));
    Check("Large LoROM DSP data $60:0000", emu->GetBus().Read(0x600000), 0x80);
    emu = Load(Rom(0x100000, 0x21, 3));
    // All addresses below $7000 select data, including odd addresses.
    Check("HiROM DSP data $00:6001", emu->GetBus().Read(0x006001), 0x80);
    emu = Load(Rom(0x100000, 0x21, 3));
    // A status read must not feed a byte into the command state machine.
    emu->GetBus().Write(0x007000, 0x00);
    Check("HiROM DSP status write ignored", emu->GetBus().Read(0x006000), 0x80);

    struct Board { size_t size; uint8_t mode; unsigned data; unsigned status; };
    for (const auto board : {Board{0x100000, 0x20, 0x208000, 0x20c000},
                             Board{0x200000, 0x20, 0x600000, 0x604000},
                             Board{0x100000, 0x21, 0x006000, 0x007000}}) {
        emu = Load(Rom(board.size, board.mode, 3));
        auto& bus = emu->GetBus();
        // Status is always the ready flag here; polling must not consume data.
        for (unsigned offset : {0u, 1u, 0x800000u, 0x800001u}) {
            Check("DSP status polling", bus.Read(board.status + offset) & 0x80, 0x80);
            bus.Write(board.status + offset, 0x00);
        }
        Check("DSP data after status writes", bus.Read(board.data), 0x80);
        // Command $00: $4000 * $4000 / $8000 = $2000.
        // Alternate odd/even data addresses and the bank mirror.
        bus.Write(board.data + 1, 0x00);
        bus.Write(board.data + 0x800000, 0x00);
        bus.Write(board.data + 0x800001, 0x40);
        bus.Write(board.data, 0x00);
        bus.Write(board.data + 1, 0x40);
        Check("DSP result ready", bus.Read(board.status) & 0x80, 0x80);
        const unsigned lo = bus.Read(board.data + 0x800001);
        Check("DSP multiplication result", lo | (bus.Read(board.data) << 8), 0x2000);
        Check("DSP command completion", bus.Read(board.data), 0x80);
    }
}

void AdditionalCartridgeLayouts() {
    for (size_t size : {size_t(0x500000), size_t(0x700000), size_t(0x800000)}) {
        auto rom = Rom(size, 0x20);
        std::copy_n(rom.begin() + 0x7fc0, 64, rom.begin() + 0x407fc0);
        rom[0x000000] = 0x11;
        rom[0x200000] = 0x22;
        rom[0x400000] = 0x44;
        if (size > 0x600000) rom[0x600000] = 0x66;
        auto emu = Load(rom);
        auto& bus = emu->GetBus();
        Check("Extended LoROM header selection", unsigned(emu->LoadedCartridge()->Header().mapping),
              unsigned(MappingType::ExLoRom));
        Check("Extended LoROM lower reset bank", bus.Read(0x008000), 0x44);
        Check("Extended LoROM upper reset bank", bus.Read(0x808000), 0x11);
        Check("Extended LoROM full upper bank", bus.Read(0xc00000), 0x22);
        Check("Extended LoROM full lower bank", bus.Read(0x400000), size > 0x600000 ? 0x66 : 0x44);
        Check("Extended LoROM tail mirror", bus.Read(0x600000), size == 0x700000 ? 0x66 :
              size == 0x500000 ? 0x44 : 0xea);
        bus.Write(0x7e8000, 0x57);
        Check("Extended LoROM WRAM priority", bus.Read(0x7e8000), 0x57);
    }

    auto namedRom = [](size_t size, const char* title, uint8_t sramShift) {
        auto rom = Rom(size, 0x20);
        std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
        std::copy_n(title, std::char_traits<char>::length(title), rom.begin() + 0x7fc0);
        rom[0x7fd8] = sramShift;
        return rom;
    };
    auto emu = Load(namedRom(0x100000, "WANDERERS FROM YS", 3));
    Check("NoMAD-1 detection", unsigned(emu->LoadedCartridge()->Header().mapping),
          unsigned(MappingType::LoRomNoMad1));
    emu->GetBus().Write(0x708123, 0x57);
    Check("NoMAD-1 SRAM upper half", emu->GetBus().Read(0x700123), 0x57);
    Check("NoMAD-1 SRAM upper-bank mirror", emu->GetBus().Read(0xf08123), 0x57);
    emu->GetBus().Write(0x7e8123, 0x63);
    Check("NoMAD-1 WRAM priority", emu->GetBus().Read(0x7e8123), 0x63);

    for (const char* title : {"SOUND NOVEL-TCOOL", "DERBY STALLION 96"}) {
        auto rom = namedRom(0x300000, title, 3);
        rom[0] = 0x11; rom[0x100000] = 0x22; rom[0x200000] = 0x33;
        emu = Load(rom);
        Check("24 Mbit board detection", unsigned(emu->LoadedCartridge()->Header().mapping),
              unsigned(MappingType::LoRom24Mbit));
        for (unsigned bank : {0x00u, 0x20u, 0x80u, 0xa0u})
            Check("24 Mbit ROM chip selection", emu->GetBus().Read((bank << 16) | 0x8000),
                  bank & 0x20 ? 0x22 : bank & 0x80 ? 0x33 : 0x11);
        emu->GetBus().Write(0x700123, 0x63);
        Check("24 Mbit SRAM mirror", emu->GetBus().Read(0xf00123), 0x63);
    }
    for (const char* title : {"THOROUGHBRED BREEDER3", "RPG-TCOOL 2"}) {
        emu = Load(namedRom(0x300000, title, 7));
        Check("Large SRAM board detection", unsigned(emu->LoadedCartridge()->Header().mapping),
              unsigned(MappingType::LoRomLargeSram));
        for (unsigned bank = 0x70; bank <= 0x73; ++bank)
            emu->GetBus().Write((bank << 16) | 0x123, static_cast<uint8_t>(bank));
        for (unsigned bank = 0x70; bank <= 0x73; ++bank) {
            Check("Large SRAM independent bank", emu->GetBus().Read((bank << 16) | 0x123), bank);
            Check("Large SRAM overlapping upper half", emu->GetBus().Read((bank << 16) | 0x8123),
                  bank == 0x73 ? 0x70 : bank + 1);
        }
    }
}

void CounterLatching() {
    auto emu = Load(Rom(0x8000, 0x20));
    auto& bus = emu->GetBus();
    emu->GetTiming().Tick(400);
    bus.Read(0x2137);
    Check("H latch at 400 master clocks", HCounter(bus), 100);

    bus.Write(0x4201, 0x00);
    const unsigned latched = HCounter(bus);
    emu->GetTiming().Tick(100);
    bus.Read(0x2137);
    Check("SLHV blocked with WRIO bit 7 low", HCounter(bus), latched);
    Check("STAT78 latch flag held with PIO low", bus.Read(0x213f) & 0x40, 0x40);
    Check("STAT78 latch flag still held", bus.Read(0x213f) & 0x40, 0x40);
    bus.Write(0x4201, 0x80);
    bus.Read(0x213f);
    Check("STAT78 latch flag clears with PIO high", bus.Read(0x213f) & 0x40, 0);
    bus.Read(0x2137);
    Check("SLHV enabled again", HCounter(bus), 125);

    emu = Load(Rom(0x8000, 0x20));
    auto& timing = emu->GetTiming();
    struct Position { unsigned clocks; unsigned dots; };
    for (const auto pos : {Position{1288, 322}, Position{1292, 322},
                           Position{1294, 323}, Position{1306, 326},
                           Position{1308, 326}, Position{1310, 326},
                           Position{1312, 327}, Position{1362, 339}}) {
        timing.Tick(pos.clocks - timing.HCounter());
        emu->GetBus().Read(0x2137);
        Check("Long-dot H counter", HCounter(emu->GetBus()), pos.dots);
    }
    // First odd field, scanline 240: every dot takes four clocks.
    timing.Tick(262 * 1364 - timing.MasterClocksElapsed() + 240 * 1364);
    Check("Short scanline fixture", timing.HPeriod(), 1360);
    timing.Tick(1292);
    emu->GetBus().Read(0x2137);
    Check("Short-line H counter", HCounter(emu->GetBus()), 323);
    timing.Tick(20);
    emu->GetBus().Write(0x4201, 0);
    Check("WRIO uses dot conversion too", HCounter(emu->GetBus()), 328);
}

void PaletteReadWriteLatches() {
    auto ppu = std::make_unique<Ppu>();
    ppu->WriteIO(0x2100, 0x80);
    ppu->CgramData()[0] = 0x1234;
    ppu->WriteIO(0x2121, 0);
    ppu->WriteIO(0x2122, 0x56);
    Check("CGRAM low read after low write", ppu->ReadIO(0x213b, 0), 0x34);
    ppu->WriteIO(0x2122, 0x07);
    Check("CGRAM write pair survives intervening read", ppu->CgramData()[0], 0x0756);

    ppu->WriteIO(0x2121, 0);
    Check("CGRAM first read", ppu->ReadIO(0x213b, 0), 0x56);
    ppu->WriteIO(0x2122, 0xaa);
    Check("CGRAM high read after low write", ppu->ReadIO(0x213b, 0) & 0x7f, 0x07);
    // CGADD resets both phases even with a partially written color.
    ppu->WriteIO(0x2121, 255);
    ppu->WriteIO(0x2122, 0xbc);
    ppu->WriteIO(0x2122, 0x9a);
    Check("CGRAM masks high color bit", ppu->CgramData()[255], 0x1abc);
    ppu->WriteIO(0x2122, 0xde);
    ppu->WriteIO(0x2122, 0x12);
    Check("CGRAM write address wraps", ppu->CgramData()[0], 0x12de);
    ppu->WriteIO(0x2121, 255);
    Check("CGRAM read low", ppu->ReadIO(0x213b, 0), 0xbc);
    Check("CGRAM read high retains open bus bit", ppu->ReadIO(0x213b, 0), 0x9a);
    Check("CGRAM read address wraps", ppu->ReadIO(0x213b, 0), 0xde);
}

void OamMirror() {
    auto ppu = std::make_unique<Ppu>();
    ppu->WriteIO(0x2100, 0x80);
    ppu->OamData()[0x200] = 0x5a;
    ppu->WriteIO(0x2102, 0x10);
    ppu->WriteIO(0x2103, 0x01);
    Check("OAM byte $220 mirrors $200", ppu->ReadIO(0x2138, 0), 0x5a);
    for (unsigned base = 0x200; base < 0x400; base += 0x20) {
        ppu->WriteIO(0x2102, static_cast<uint8_t>(base >> 1));
        ppu->WriteIO(0x2103, 1);
        for (unsigned offset = 0; offset < 32; ++offset)
            ppu->WriteIO(0x2104, static_cast<uint8_t>(base / 32 + offset));
        for (unsigned offset = 0; offset < 32; ++offset)
            Check("OAM mirror write", ppu->OamData()[0x200 + offset], base / 32 + offset);
        ppu->WriteIO(0x2102, static_cast<uint8_t>(base >> 1));
        ppu->WriteIO(0x2103, 1);
        for (unsigned offset = 0; offset < 32; ++offset)
            Check("OAM mirror read", ppu->ReadIO(0x2138, 0), base / 32 + offset);
    }
    // $3FF wraps to the low table, whose writes still commit in pairs.
    ppu->WriteIO(0x2104, 0x34);
    Check("OAM low write waits for high byte", ppu->OamData()[0], 0);
    ppu->WriteIO(0x2104, 0x12);
    Check("OAM wrapped low byte", ppu->OamData()[0], 0x34);
    Check("OAM wrapped high byte", ppu->OamData()[1], 0x12);
}

void OverscanTiming() {
    auto emu = Load(Rom(0x8000, 0x20));
    emu->GetBus().Write(0x2133, 0x04);
    // Cross a full frame so a frame-latched change has time to take effect.
    emu->GetTiming().Tick(262 * 1364);
    Check("Overscan PPU vblank line", emu->GetPpu().VDisp(), 240);
    Check("Overscan timing vblank line", emu->GetTiming().VDisp(), 240);
    emu->GetTiming().Tick(225 * 1364);
    Check("Overscan HVBJOY at line 225", emu->GetBus().Read(0x4212) & 0x80, 0);
    emu->GetTiming().Tick(15 * 1364);
    Check("Overscan HVBJOY at line 240", emu->GetBus().Read(0x4212) & 0x80, 0x80);
    emu->GetBus().Write(0x2133, 0);
    Check("Normal display timing restored", emu->GetTiming().VDisp(), 225);

    emu = Load(Rom(0x8000, 0x20));
    emu->GetBus().Write(0x2133, 4);
    emu->GetBus().Write(0x2100, 15);
    emu->GetPpu().CgramData()[0] = 0x001f;
    // Real callbacks must cache and render the bottom overscan row.
    emu->GetTiming().Tick(240 * 1364);
    Check("Overscan bottom row rendered",
          emu->GetPpu().OutputData()[238 * Ppu::OutputWidth], 0xff0000ff);

    unsigned renders = 0, transfers = 0, vblankLine = 0, nmiLine = 0;
    auto& timing = emu->GetTiming();
    timing.onRenderCycle = [&](uint16_t) { ++renders; };
    timing.onHdmaTransfer = [&](uint16_t) { ++transfers; };
    timing.onVBlankBegin = [&]() { vblankLine = timing.VCounter(); };
    timing.onNmiPoint = [&]() { nmiLine = timing.VCounter(); };
    for (bool overscan : {false, true}) {
        timing.Tick(timing.HPeriod() - timing.HCounter());
        while (timing.VCounter() != 0) timing.Tick(timing.HPeriod());
        emu->GetBus().Write(0x2133, overscan ? 4 : 0);
        renders = transfers = vblankLine = nmiLine = 0;
        const unsigned firstBlank = overscan ? 240 : 225;
        timing.Tick(firstBlank * 1364 + 2);
        Check("Visible scanline callback count", renders, firstBlank - 1);
        Check("HDMA callback count includes line zero", transfers, firstBlank);
        Check("VBlank callback line", vblankLine, firstBlank);
        Check("NMI callback line", nmiLine, firstBlank);
    }
}

void IndirectHdmaTermination() {
    // Test both setup-time termination and termination after a data transfer.
    for (bool afterTransfer : {false, true}) {
        for (bool laterChannel : {false, true}) {
            auto emu = Load(Rom(0x80000, 0x20));
            auto& bus = emu->GetBus();
            auto& dma = emu->GetDma();
            dma.Reset();
            auto& ch = dma.Channel(0);
            ch.writeControl(0x40);
            ch.sourceBank = ch.indirectBank = 0x7e;
            ch.sourceAddress = afterTransfer ? 0x1000 : 0x1003;
            ch.targetAddress = 0x00;
            bus.Write(0x7e1000, 1);
            bus.Write(0x7e1001, 0x00);
            bus.Write(0x7e1002, 0x20);
            bus.Write(0x7e1003, 0); // Terminator.
            bus.Write(0x7e1004, 0x56);
            bus.Write(0x7e1005, 0x34);
            bus.Write(0x7e2000, 0x8f);
            auto& next = dma.Channel(1);
            next.writeControl(0);
            next.sourceBank = 0x7e;
            next.sourceAddress = 0x3000;
            next.targetAddress = 0x00;
            bus.Write(0x7e3000, 2);
            bus.Write(0x7e3001, 0x8f);
            dma.EnableHdma(laterChannel ? 3 : 1);
            dma.HdmaReset();
            unsigned cycles = dma.HdmaSetup();
            if (afterTransfer) {
                Check("HDMA initial indirect address", ch.indirectAddress(), 0x2000);
                cycles = dma.HdmaRun();
            }
            Check("HDMA terminator completes channel", ch.hdmaCompleted, true);
            Check("HDMA terminator indirect register", ch.indirectAddress(),
                  laterChannel ? 0x3456 : 0x5600);
            Check("HDMA terminator table address", ch.hdmaAddress,
                  laterChannel ? 0x1006 : 0x1005);
            Check("HDMA terminator cycle count", cycles,
                  afterTransfer ? (laterChannel ? 56 : 32) : (laterChannel ? 40 : 24));
            Check("HDMA terminator disables transfers", ch.hdmaDoTransfer, false);
        }
    }
}

} // namespace

int main() {
    CartridgeMapping();
    AdditionalCartridgeLayouts();
    DspMapping();
    CounterLatching();
    PaletteReadWriteLatches();
    OamMirror();
    OverscanTiming();
    IndirectHdmaTermination();
    std::printf("%d hardware checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
