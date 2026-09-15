#include "snes/core/SetaProtocol.hpp"
#include "snes/core/Emulator.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace snes::core;
namespace {
void Check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }

void BoardProtocol() {
    std::vector<uint8_t> ram(4096, 0x55);
    St011 chip(ram);
    Check(chip.Read(1) == 0xff && chip.Read(0x101) == 0x55, "ST011 status does not replace ordinary RAM");
    chip.Write(0, 1);
    for (unsigned i = 0; i < 127; ++i) chip.Write(0, uint8_t(i));
    Check(chip.Board()[80] == 0, "Board packet remains incomplete through byte 127");
    chip.Write(0, 127);
    for (unsigned row = 0; row < 9; ++row) for (unsigned col = 0; col < 9; ++col)
        Check(chip.Board()[row * 9 + col] == row * 10 + col, "Board rows discard each wire-format padding byte");
    chip.Write(0x12c, 0x92); chip.Write(0x12d, 0x93); chip.Write(0x12e, 0x94);
    chip.Write(0, 4);
    Check(chip.Read(0x12c) == 0 && chip.Read(0x12d) == 0x93 && chip.Read(0x12e) == 0,
          "Observed command 4 clears only its two result bytes");
    chip.Write(0, 2);
    for (uint8_t byte : {4,5,6,14}) chip.Write(0, byte);
    Check(chip.Read(0x12d) == 0x93, "Command 2 consumes four parameters without interpreting them as commands");
    chip.Write(0, 14);
    Check(chip.Read(0x12d) == 0, "A completed packet accepts the next command");
    chip.Write(0x1002, 0x87);
    Check(chip.Read(0x1002) == 0x87 && ram[2] == 0x55, "Extra command work RAM does not alias saved RAM");
    chip.Write(0, 1); chip.Write(0, 99); chip.Reset(); chip.Write(0, 5);
    Check(chip.Board()[0] == 0 && chip.Read(0x1002) == 0 && ram[2] == 0x55, "Reset cancels partial packets and preserves battery storage");
}

void StartupProtocol() {
    std::vector<uint8_t> ram(4096, 0x67);
    St018 chip(ram);
    for (uint8_t middle : {uint8_t(1), uint8_t(255)}) {
        chip.Write(0x3804, 0); chip.Write(0x3804, middle);
        Check(chip.PendingOutput() == 0, "ST018 waits for all three command bytes");
        chip.Write(0x3804, 0);
        Check(chip.PendingOutput() == 2, "Recognized startup command produces two response bytes");
        Check(chip.Read(0x3804) == 0x81 && chip.Read(0x3804) == 0x81 && chip.PendingOutput() == 0,
              "Initial handshake response is drained in order");
        for (unsigned pass = 0; pass < 2; ++pass) {
            chip.Write(0x3802, uint8_t(pass));
            Check(chip.PendingOutput() == 3 && chip.Read(0x3800) == 0, "Each startup parameter produces three replies and ready status");
            for (unsigned byte = 0; byte < 3; ++byte) Check(chip.Read(0x3804) == 0x81, "Startup response byte");
        }
        Check(chip.PendingOutput() == 0 && chip.Read(0x3804) == 0x81, "Empty output port returns its observed idle value");
    }
    Check(std::all_of(ram.begin(), ram.end(), [](uint8_t byte) { return byte == 0x67; }), "Startup I/O cannot overwrite battery RAM through address wrapping");
    chip.Write(0x3804, 0xff); chip.Reset();
    chip.Write(0x3804, 0); chip.Write(0x3804, 1); chip.Write(0x3804, 0);
    Check(chip.PendingOutput() == 2, "Reset discards an incomplete startup command");
}

std::vector<uint8_t> Image(uint8_t type) {
    std::vector<uint8_t> rom(0x80000, 0xea);
    std::fill_n(rom.begin() + 0x7fc0, 64, 0);
    std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
    rom[0x7fc0] = 'T'; rom[0x7fd5] = 0x30; rom[0x7fd6] = type; rom[0x7fd7] = 9;
    rom[0x7fd9] = 1; rom[0x7fdc] = rom[0x7fdd] = 0xff; rom[0x7ffd] = 0x80;
    return rom;
}

void CartridgeMaps() {
    auto machine = std::make_unique<Emulator>();
    Check(machine->LoadCartridge(Image(0xf6)), "ST011 image loads its supported command interface");
    auto& bus = machine->GetBus();
    Check(machine->LoadedCartridge()->Header().chip == EnhancementChip::St011 &&
          machine->LoadedCartridge()->Header().mapping == MappingType::St011, "ST011 keeps a distinct board type");
    bus.Write(0x68012c, 0x21); bus.Write(0x600000, 4);
    Check(bus.Read(0x680001) == 0xff && bus.Read(0x70012c) == 0, "ST011 command and RAM windows share cartridge storage");
    bus.Write(0x700002, 0x64); bus.Write(0x601002, 0x31);
    Check(bus.Read(0x700002) == 0x64 && bus.Read(0x681002) == 0x31, "ST011 extra work RAM is separate from ordinary SRAM mirrors");
    Check(machine->LoadedCartridge()->SramData().size() == 4096, "Only the board's 4 KiB battery storage is persisted");
    Check(machine->LoadCartridge(Image(0xf5)), "ST018 image loads its supported startup interface");
    bus.Write(0x700802, 0x76);
    bus.Write(0x803804, 0); bus.Write(0x003804, 1); bus.Write(0xbf3804, 0);
    Check(bus.Read(0x003804) == 0x81 && bus.Read(0xbf3800) == 0, "ST018 status and data ports mirror through system banks");
    bus.Write(0x003802, 0);
    Check(bus.Read(0x700802) == 0x76 && bus.Read(0x008000) == 0xea, "ST018 port writes preserve saved data and LoROM mapping");
}
}
int main() {
    try { BoardProtocol(); StartupProtocol(); CartridgeMaps(); }
    catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
    std::puts("ST011 packet and ST018 startup protocol checks passed; internal processors are not covered");
}
