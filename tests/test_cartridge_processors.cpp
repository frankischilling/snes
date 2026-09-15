#include "snes/core/Emulator.hpp"
#include "snes/core/SnesCpu.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace snes::core;
namespace {
void Check(bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        throw std::runtime_error(message);
    }
}

std::vector<uint8_t> Rom(uint8_t mode, uint8_t type, size_t size = 0x100000) {
    std::vector<uint8_t> bytes(size, 0xea);
    std::fill_n(bytes.begin() + 0x7fc0, 64, 0);
    std::fill_n(bytes.begin() + 0x7fc0, 21, ' ');
    bytes[0x7fc0] = 'T'; bytes[0x7fd5] = mode; bytes[0x7fd6] = type;
    bytes[0x7fd7] = 10; bytes[0x7fd8] = 7; bytes[0x7fd9] = 1;
    bytes[0x7fdc] = bytes[0x7fdd] = 0xff;
    bytes[0x7ffd] = 0x80; bytes[0x7fff] = 0xa0;
    return bytes;
}

std::unique_ptr<Emulator> Machine(const std::vector<uint8_t>& rom) {
    auto machine = std::make_unique<Emulator>();
    std::string error;
    if (!machine->LoadCartridge(rom, &error)) throw std::runtime_error(error);
    return machine;
}

void StepToIrq(Emulator& machine) {
    auto& cpu = *machine.GetCpu();
    cpu.regs().p &= ~Processor65816::FlagI;
    for (unsigned i = 0; i < 300 && cpu.regs().pc != 0xa000; ++i) cpu.Step();
    Check(cpu.regs().pc == 0xa000, "Cartridge IRQ reaches the console CPU through the scheduler");
}

void Sa1Integration() {
    auto rom = Rom(0x23, 0x35);
    const uint8_t program[]{
        0xa9, 0x80, 0x8d, 0x27, 0x22, // Enable shared BW-RAM writes.
        0xa9, 0x5a, 0x8f, 0x34, 0x12, 0x40, // Store through the SA-1 linear bank.
        0xa9, 0x80, 0x8d, 0x09, 0x22, // Raise the console IRQ.
        0xdb
    };
    std::copy(std::begin(program), std::end(program), rom.begin() + 0x1000);
    auto machine = Machine(rom);
    auto& bus = machine->GetBus();
    Check(machine->LoadedCartridge()->Header().mapping == MappingType::Sa1, "SA-1 selects its cartridge map");
    Check(bus.Read(0x8030ff) == 0 && bus.Read(0x80230e) == 0x23, "SA-1 IRAM and MMIO mirrors are mapped");
    bus.Write(0x7e0010, 0x6c);
    bus.Write(0x2229, 1); bus.Write(0x3001, 0x41);
    Check(bus.Read(0x803001) == 0x41 && bus.Read(0x000010) == 0x6c, "IRAM windows preserve the console WRAM mirrors");
    bus.Write(0x2201, 0x80);
    bus.Write(0x2203, 0); bus.Write(0x2204, 0x90); bus.Write(0x2200, 0);
    StepToIrq(*machine);
    Check(bus.Read(0x401234) == 0x5a && machine->LoadedCartridge()->SramData()[0x1234] == 0x5a,
          "SA-1 executes from ROM and writes the actual cartridge save storage");
    Check((bus.Read(0x2300) & 0x80) && machine->LoadedCartridge()->CpuIrqPending(), "SA-1 IRQ status remains asserted until acknowledged");
    bus.Write(0x802202, 0x80);
    Check(!machine->LoadedCartridge()->CpuIrqPending(), "Mirrored CPU acknowledge clears the SA-1 IRQ");
    std::vector<uint8_t> save(0x20000, 0x39);
    machine->LoadSram(save);
    Check(bus.Read(0x4f1234) == 0x39, "Loading SRAM preserves the processor's shared storage span");
}

void Sa1Expansion() {
    auto rom = Rom(0x23, 0x35, 0x300000);
    std::copy_n("ZX3J", 4, rom.begin() + 0x7fb2);
    rom[0x7fda] = 0x33;
    std::vector<uint8_t> expansion(0x80000, 0xa5);
    expansion[0x1234] = 0x62;
    auto cart = Cartridge::FromBroadcastCartridge(rom, expansion);
    Check(cart.has_value() && cart->Header().mapping == MappingType::BroadcastSa1,
          "SA-1 slot board accepts a separate read-only expansion");
    Check(cart->MemoryPackData().empty() && !cart->LoadMemoryPack(expansion), "Expansion ROM is never treated as a flash save");
    cart->Write(0x2220, 0x84);
    Check(cart->Read(0xc01234) == 0x62 && cart->Read(0xc81234) == 0x62 && cart->Read(0x009234) == 0x62,
          "Linear and LoROM expansion windows mirror the physical ROM address lines");
    cart->Write(0x2220, 0);
    Check(cart->Read(0xc01234) == rom[0x1234] && cart->Read(0x009234) == rom[0x1234],
          "MMC switches back to the base cartridge without replacing its ROM storage");
    cart->Write(0x2226, 0x80); cart->Write(0x701234, 0x37);
    Check(cart->Read(0x401234) == 0x37 && cart->Read(0x601234) == 0x37 && cart->Read(0x7c1234) == 0x37,
          "Slot-board shared RAM aliases across the expanded console windows");
    auto empty = Cartridge::FromBroadcastCartridge(rom, {});
    empty->Write(0x2220, 4);
    Check(empty->Read(0xc01234, 0x71) == 0x71 && empty->Read(0x009234, 0x53) == 0x53,
          "An absent expansion leaves its selected ROM bus undriven");
    Check(!Cartridge::FromBroadcastCartridge(rom, std::span<const uint8_t>(expansion).first(4096)),
          "Invalid expansion sizes are rejected");
    auto machine = std::make_unique<Emulator>();
    Check(machine->LoadBroadcastCartridge(rom, expansion), "Slot-board loader connects SA-1 and its expansion");
    auto& bus = machine->GetBus();
    bus.Write(0x2226, 0x80); bus.Write(0x721234, 0x6e);
    Check(bus.Read(0x401234) == 0x6e, "Expanded shared RAM is installed in the console bus");
    bus.Write(0x7e1234, 0x42);
    Check(bus.Read(0x7e1234) == 0x42 && bus.Read(0x401234) == 0x6e, "Expanded cartridge banks preserve console WRAM");
    bus.Write(0x2220, 0x84);
    Check(bus.Read(0xc01234) == 0x62, "Expansion MMC works after the cartridge moves into the emulator");

    std::vector<uint8_t> ram(0x20000);
    Sa1 processor(rom, ram);
    processor.SetExpansionRom(expansion);
    processor.WriteCpu(0x2220, 0x84);
    Check(processor.ReadSa1(0xc01234) == 0x62, "Coprocessor and console share expansion bank selection");
    processor.WriteSa1(0x600000, 7);
    Check((processor.ReadCpu(0x400000) & 15) == 7, "Coprocessor bitmap access retains priority over console RAM aliases");
    processor.Reset(); processor.WriteCpu(0x2220, 4);
    Check(processor.ReadSa1(0xc01234) == 0x62, "Processor reset preserves the attached expansion");
}

void SuperFxIntegration() {
    auto rom = Rom(0x20, 0x15);
    const uint8_t program[]{0xf0, 0xef, 0xbe, 0xf1, 0x00, 0x01, 0x31, 0x00, 0x01};
    std::copy(std::begin(program), std::end(program), rom.begin() + 0x1000);
    auto machine = Machine(rom);
    auto& bus = machine->GetBus();
    Check(machine->LoadedCartridge()->Header().mapping == MappingType::SuperFx, "GSU selects its cartridge map");
    Check(machine->LoadedCartridge()->SramData().size() == 0x8000, "Older GSU header supplies 32 KiB of work RAM");
    Check(bus.Read(0x80303b) == 4 && bus.Read(0x409000) == 0xea, "GSU MMIO and linear ROM banks are mapped");
    // The console must execute from WRAM while the graphics processor owns ROM.
    for (uint32_t address = 0; address < 0x400; ++address) bus.Write(0x7e0000 + address, 0xea);
    machine->GetCpu()->regs().pc = 0;
    bus.Write(0x303a, 0x18);
    bus.Write(0x301e, 0); bus.Write(0x301f, 0x90);
    StepToIrq(*machine);
    Check(bus.Read(0x700100) == 0xef && bus.Read(0xf00101) == 0xbe,
          "Clocked GSU program writes shared RAM through both console aliases");
    Check(machine->LoadedCartridge()->CpuIrqPending(), "GSU STOP requests a console IRQ");
    Check((bus.Read(0x803031) & 0x80) && !machine->LoadedCartridge()->CpuIrqPending(), "Reading the high GSU status byte acknowledges IRQ");
    std::vector<uint8_t> save(0x8000, 0x29);
    machine->LoadSram(save);
    Check(bus.Read(0x006100) == 0x29, "GSU and save loading retain the same RAM storage after cartridge moves");

    auto large = Rom(0x20, 0x1a, 0x400000);
    large[0x7fda] = 0x33; large[0x7fbd] = 7;
    large[0x1234] = 0x12; large[0x201234] = 0x56; large[0x301234] = 0x9a;
    Check(machine->LoadCartridge(large), "Reload a four-MiB GSU2 cartridge");
    Check(machine->LoadedCartridge()->SramData().size() == 0x20000, "GSU2 work RAM follows the extended header");
    Check(bus.Read(0xc01234) == 0x12 && bus.Read(0xe01234) == 0x56 && bus.Read(0xf01234) == 0x9a,
          "GSU2 console banks expose the second half of a four-MiB ROM");
    bus.Write(0xf01234, 0);
    Check(bus.Read(0xf01234) == 0x9a, "GSU2 upper ROM banks do not alias writable RAM");
    bus.Write(0x711234, 0x87);
    Check(machine->LoadedCartridge()->SramData()[0x11234] == 0x87, "GSU2 RAM remains at its separate banks");
}

void SuperFxExtendedBoards() {
    auto rom = Rom(0x20, 0x1a, 0x400000);
    rom[0x7fda] = 0x33;
    rom[0x7fbd] = 8;
    auto machine = Machine(rom);
    auto& bus = machine->GetBus();
    bus.Write(0x721234, 0x42);
    bus.Write(0x731234, 0x63);
    Check(machine->LoadedCartridge()->SramData()[0x21234] == 0x42 &&
          machine->LoadedCartridge()->SramData()[0x31234] == 0x63,
          "Console bus exposes all four connected GSU RAM banks");
    Check(bus.Read(0x721234) == 0x42 && bus.Read(0x731234) == 0x63,
          "Extended GSU RAM reads reach the same saved storage");

    rom[0x7fbd] = 10;
    Check(machine->LoadCartridge(rom), "Load a GSU board with one MiB of CPU RAM");
    for (unsigned bank = 0x70; bank <= 0x7d; ++bank)
        bus.Write((bank << 16) | 0x1234, uint8_t(bank));
    for (unsigned bank = 0x70; bank <= 0x7d; ++bank) {
        Check(bus.Read((bank << 16) | 0x1234) == bank &&
              machine->LoadedCartridge()->SramData()[((bank - 0x70) << 16) | 0x1234] == bank,
              "CPU expansion RAM banks retain independent address lines");
    }
    bus.Write(0x7e1234, 0x91);
    bus.Write(0x7f1234, 0xa2);
    Check(bus.Read(0x7e1234) == 0x91 && bus.Read(0x7f1234) == 0xa2 &&
          bus.Read(0x761234) == 0x76,
          "Console WRAM has priority over expansion RAM at banks 7E and 7F");

    auto expanded = Rom(0x20, 0x1a, 0xb00000);
    for (size_t bank = 0; bank < expanded.size() / 0x10000; ++bank) {
        expanded[bank * 0x10000 + 0x1234] = uint8_t(bank);
        expanded[bank * 0x10000 + 0x9234] = uint8_t(bank);
    }
    expanded[0x7fd7] = 14;
    expanded[0x7fda] = 0x33;
    expanded[0x7fbd] = 8;
    Check(machine->LoadCartridge(expanded), "Load an eleven-MiB GSU image with the extended layout marker");
    for (unsigned bank = 0; bank < 0x40; ++bank) {
        Check(bus.Read((bank << 16) | 0x9234) == bank / 2 &&
              bus.Read(((bank + 0x80) << 16) | 0x9234) == 0x20 + bank / 2,
              "Extended GSU system windows select separate two-MiB ROM regions");
        Check(bus.Read(((bank + 0xc0) << 16) | 0x1234) == 0x40 + bank,
              "Extended GSU upper linear window selects ROM bytes four through eight MiB");
    }
    for (unsigned bank = 0x40; bank <= 0x6f; ++bank)
        Check(bus.Read((bank << 16) | 0x1234) == bank + 0x40,
              "Extended GSU lower linear window exposes the final three MiB");
    bus.Write(0x601234, 0);
    Check(bus.Read(0x601234) == 0xa0, "Extended GSU ROM windows ignore writes");
    bus.Write(0x731234, 0xd7);
    Check(machine->LoadedCartridge()->SramData()[0x31234] == 0xd7,
          "Extended ROM retains the separate four-bank RAM window");
}
}

int main() {
    try { Sa1Integration(); Sa1Expansion(); SuperFxIntegration(); SuperFxExtendedBoards(); }
    catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
    std::puts("Cartridge processor bus, scheduling, IRQ and save integration checks passed");
}
