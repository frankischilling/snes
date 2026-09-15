#include "snes/core/Emulator.hpp"
#include "snes/core/SnesCpu.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace snes::core;
namespace {
void Check(bool value, const char* message) {
    if (!value) { std::fprintf(stderr, "%s\n", message); throw std::runtime_error(message); }
}

std::vector<uint8_t> Rom(uint8_t mode, uint8_t type, uint8_t maker = 0) {
    std::vector<uint8_t> rom(0x100000, 0xea);
    std::fill_n(rom.begin() + 0x7fc0, 64, 0);
    std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
    rom[0x7fc0] = 'T'; rom[0x7fd5] = mode; rom[0x7fd6] = type;
    rom[0x7fd7] = 10; rom[0x7fd9] = 1; rom[0x7fda] = maker;
    rom[0x7fdc] = rom[0x7fdd] = 0xff;
    rom[0x7ffd] = 0x80; rom[0x7fff] = 0xa0;
    return rom;
}

struct Program {
    std::vector<uint8_t> bytes;
    void Write(uint32_t address, uint8_t value) {
        bytes.insert(bytes.end(), {0xa9, value, 0x8f, uint8_t(address), uint8_t(address >> 8), uint8_t(address >> 16)});
    }
    void PortWord(uint32_t address, uint16_t word) { Write(address, uint8_t(word)); Write(address, uint8_t(word >> 8)); }
    void Read(uint32_t address, uint8_t destination) {
        bytes.insert(bytes.end(), {0xaf, uint8_t(address), uint8_t(address >> 8), uint8_t(address >> 16), 0x85, destination});
    }
    void Run(Emulator& machine, std::vector<uint8_t> rom) {
        bytes.push_back(0xdb);
        std::copy(bytes.begin(), bytes.end(), rom.begin());
        Check(machine.LoadCartridge(rom), "Command test cartridge loads");
        for (unsigned i = 0; i < 1000 && !machine.GetCpu()->regs().stp; ++i) machine.GetCpu()->Step();
        Check(machine.GetCpu()->regs().stp, "Native 65816 command program reaches its stop instruction");
    }
};

void CommandBuses() {
    auto machine = std::make_unique<Emulator>();
    Program hex;
    hex.Read(0x20c000, 0);
    hex.Write(0x208000, 6); hex.PortWord(0xa08000, 0x1010);
    hex.Write(0xbf8000, 3); hex.PortWord(0x208000, 0x0304);
    hex.Read(0xa08000, 1); hex.Read(0x3f8000, 2); hex.Read(0xbfc000, 3);
    hex.Run(*machine, Rom(0x30, 5, 0xb2));
    auto& bus = machine->GetBus();
    Check(bus.Read(0) == 0x84 && bus.Read(1) == 52 && bus.Read(2) == 0 && bus.Read(3) == 0x84,
          "DSP-3 consumes mirrored command bytes and returns the hex-grid index through the console bus");
    Check(bus.Read(0x409000) == 0xea && bus.Read(0x208000) == 0x80,
          "DSP-3 port mapping preserves adjacent cartridge ROM banks");

    Program road;
    road.Read(0x30c000, 0); road.PortWord(0x308000, 0);
    road.PortWord(0xb08000, uint16_t(-3)); road.PortWord(0x3f8000, 5);
    for (uint8_t i = 0; i < 4; ++i) road.Read(i & 1 ? 0xbf8000 : 0x308000, uint8_t(i + 1));
    road.Read(0xbf8000, 5);
    road.Run(*machine, Rom(0x30, 3));
    Check(bus.Read(0) == 0x80 && bus.Read(1) == 0xf1 && bus.Read(2) == 0xff &&
          bus.Read(3) == 0xff && bus.Read(4) == 0xff && bus.Read(5) == 0xff,
          "DSP-4 signed multiplication and output exhaustion are visible through all port mirrors");
    Check(bus.Read(0x209000) == 0xea, "DSP-4 retains ROM in banks outside its narrower port range");

    Program graphics;
    graphics.Write(0x807f80, 0x34); graphics.Write(0x007f81, 0x12); graphics.Write(0x3f7f82, 0);
    graphics.Write(0xbf7f83, 3); graphics.Write(0x007f84, 0); graphics.Write(0x007f85, 0);
    graphics.Write(0x807f4f, 0x25);
    graphics.Read(0x007f80, 0); graphics.Read(0x807f81, 1); graphics.Read(0xbf7f82, 2);
    graphics.Run(*machine, Rom(0x20, 0xf3));
    Check(bus.Read(0) == 0x9c && bus.Read(1) == 0x36 && bus.Read(2) == 0,
          "Cx4 commands execute through the real CPU and mirrored cartridge work RAM");
    bus.Write(0x7e1234, 0x65);
    Check(bus.Read(0x001234) == 0x65 && bus.Read(0x409000) == 0xea,
          "Cx4's command windows preserve WRAM and ordinary cartridge ROM");
}

void BroadcastBus() {
    auto bios = Rom(0x20, 0);
    std::copy_n("Satellaview BS-X     ", 21, bios.begin() + 0x7fc0);
    auto machine = std::make_unique<Emulator>();
    Check(machine->LoadCartridge(bios) && machine->LoadedCartridge()->Header().mapping == MappingType::Bsx,
          "An ordinary BIOS load selects the BS-X memory controller");
    Check(machine->LoadedCartridge()->SramData().size() == 0x8000 &&
          machine->LoadedCartridge()->MemoryPackData().size() == 0x100000,
          "BIOS-only loading supplies battery RAM and a blank one-MiB flash pack");
    auto& bus = machine->GetBus();
    Check(bus.Read(0x802196) == 0x10 && bus.Read(0x002197) == 0x80, "Broadcast registers are installed in CPU system banks");
    bus.Write(0x2181, 0x34); bus.Write(0x2182, 0x12); bus.Write(0x2183, 0); bus.Write(0x2180, 0x72);
    Check(bus.Read(0x7e1234) == 0x72, "Radio ports do not replace adjacent console WRAM ports");
    for (unsigned bank = 0x10; bank < 0x18; ++bank) bus.Write((bank << 16) | 0x5123, uint8_t(bank));
    Check(machine->LoadedCartridge()->SramData()[0x7123] == 0x17, "The memory controller writes actual persistent cartridge RAM");
    auto mmc = [&](unsigned reg, uint8_t value) { bus.Write((reg << 16) | 0x5000, value); };
    auto bare = [&]() {
        for (unsigned reg = 1; reg <= 13; ++reg) mmc(reg, reg == 2 || reg == 12 ? 0x80 : 0);
        mmc(14, 0x80);
    };
    bare();
    mmc(1, 0x80);
    bus.Write(0xc05555, 0x40); bus.Write(0xc11234, 0xa6);
    Check(machine->LoadedCartridge()->MemoryPackData()[0x11234] == 0xa6 && machine->LoadedCartridge()->CpuIrqPending(),
          "Flash programming updates the persistent pack and asserts the console IRQ");
    // Run a console NOP from WRAM while the memory controller hides the BIOS.
    bus.Write(0x7e0000, 0xea); bus.Write(0x7e0001, 0xea);
    auto& cpu = *machine->GetCpu(); cpu.regs().pc = 0; cpu.regs().p &= ~Processor65816::FlagI;
    cpu.Step();
    const auto stack = cpu.regs().s;
    cpu.Step();
    Check(cpu.regs().s == uint16_t(stack - 3), "Flash-ready IRQ enters the CPU through the normal interrupt sampler");
    mmc(0, 0x80);
    Check(!machine->LoadedCartridge()->CpuIrqPending(), "MMC ready acknowledgement clears the cartridge interrupt request");

    Check(machine->SetBroadcastTimeSource([] { return BroadcastTime{2026,9,14,2,12,34,56}; }),
          "Host can supply a deterministic broadcast clock");
    bus.Write(0x2188, 0); bus.Write(0x2189, 0); bus.Write(0x218b, 1); bus.Write(0x218c, 1);
    Check(bus.Read(0x218a) == 1 && bus.Read(0x218b) == 0x90, "Clock channel offers a complete packet through the bus");
    std::array<uint8_t, 23> clock{};
    for (auto& byte : clock) byte = bus.Read(0x80218c);
    Check(clock[10] == 56 && clock[11] == 34 && clock[12] == 12 && clock[14] == 14,
          "CPU reads the stable clock record provided by the host");
    const std::array<uint8_t, 3> payload{0x32,0x87,0x64};
    Check(machine->LoadBroadcastStream(0x123, 0, payload), "Host can attach a broadcast payload to the loaded BIOS");
    bus.Write(0x218e, 0x23); bus.Write(0x218f, 1); bus.Write(0x2191, 1); bus.Write(0x2192, 1);
    Check(bus.Read(0x2190) == 1 && bus.Read(0x2191) == 0x90, "Second receiver offers the attached stream");
    for (auto byte : payload) Check(bus.Read(0x802192) == byte, "Second receiver returns payload bytes in order");
    Check(bus.Read(0x2192) == 0xff, "Final stream packet pads absent bytes without reading outside its payload");

    std::vector<uint8_t> pack(0x200000, 0xff); pack[0x112345] = 0x5a;
    Check(machine->LoadBroadcastCartridge(bios, pack), "BIOS loader accepts a two-MiB memory pack");
    bare();
    Check(bus.Read(0xd12345) == 0x5a && machine->LoadedCartridge()->MemoryPackData().size() == pack.size(),
          "Upper flash banks and persistent save length follow the attached pack capacity");
    pack[0x112345] = 0x49;
    Check(machine->LoadMemoryPack(pack), "A flash save loads without invalidating device spans");
    bare(); Check(bus.Read(0xd12345) == 0x49, "Flash reload reaches the same controller allocation");
    std::vector<uint8_t> saved(0x8000, 0x47);
    machine->LoadSram(saved);
    Check(bus.Read(0x105123) == 0x47 && bus.Read(0x175123) == 0x47, "Battery save loading preserves controller RAM mappings");
    pack[0xff00] = 'M'; pack[0xff02] = 'P'; pack[0xff06] = 0x70;
    Check(machine->LoadBroadcastCartridge(bios, pack), "BIOS accepts a read-only mask-ROM pack");
    bare(); bus.Write(0xc05555, 0x40); bus.Write(0xd12345, 0);
    Check(bus.Read(0xd12345) == 0x49 && machine->LoadedCartridge()->MemoryPackData().empty(),
          "Mask-ROM packs remain read-only and are omitted from flash saves");
    Check(!machine->LoadBroadcastCartridge(bios, std::span<const uint8_t>(pack).first(123)),
          "Invalid pack capacity is rejected without replacing the running cartridge");
    Check(bus.Read(0xd12345) == 0x49, "A rejected reload preserves the installed media");
    Check(machine->LoadCartridge(Rom(0x20, 0)) && !machine->LoadBroadcastStream(1, 0, payload),
          "Reloading an ordinary cartridge removes broadcast device APIs");
    bus.SetOpenBus(0x35);
    Check(bus.Read(0x2188) == 0x35, "Reloading removes the old radio bus handler");
}
}

int main() {
    try { CommandBuses(); BroadcastBus(); }
    catch (const std::exception&) { return 1; }
    std::puts("Native cartridge command, broadcast mapping, flash, IRQ, clock, stream and persistence checks passed");
}
