#include "snes/core/Emulator.hpp"
#include "snes/core/St010.hpp"
#include "st010_cases.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>

using namespace snes::core;
namespace {
void Check(bool result, const char* message) { if (!result) throw std::runtime_error(message); }
using Ram = std::array<uint8_t, 4096>;
void Word(Ram& ram, unsigned offset, uint16_t value) { ram[offset] = uint8_t(value); ram[offset + 1] = uint8_t(value >> 8); }
uint16_t Word(const Ram& ram, unsigned offset) { return uint16_t(ram[offset] | (uint16_t(ram[offset + 1]) << 8)); }
uint32_t Long(const Ram& ram, unsigned offset) { return Word(ram, offset) | (uint32_t(Word(ram, offset + 2)) << 16); }
void Run(St010& device, Ram& ram, uint8_t command) {
    device.Write(0x600000, 0, ram);
    device.Write(0x680020, command, ram);
    device.Write(0x680021, 0x80, ram);
    Check(device.Read(0x680020, ram) == 0 && device.Read(0x680021, ram) == 0,
          "Command and execution registers clear on completion");
}

void RegistersAndMath() {
    St010 device;
    Ram ram{};
    Word(ram, 0, 3); Word(ram, 2, 7);
    device.Write(0x680020, 6, ram); device.Write(0x680021, 0x80, ram);
    Check(device.Read(0x680020, ram) == 0 && Long(ram, 0x10) == 0,
          "Register writes before control enable only update backing RAM");
    Check(ram[0x20] == 6 && ram[0x21] == 0x80, "Disabled registers retain RAM writes");
    Check(device.Read(0x600000, ram) == 0x80, "Control window identification byte");
    Run(device, ram, 6);
    Check(Long(ram, 0x10) == 42, "Signed multiply produces twice the product");
    Word(ram, 0, 0x8000); Word(ram, 2, 0x8000); Run(device, ram, 6);
    Check(Long(ram, 0x10) == 0x80000000, "Multiply wraps the positive 32-bit overflow");
    Word(ram, 0, 0xfffd); Word(ram, 2, 4); Word(ram, 4, 5); Run(device, ram, 3);
    Check(Long(ram, 0x10) == uint32_t(-30) && Long(ram, 0x14) == 40, "Signed coordinate scaling");
    Word(ram, 0, 3); Word(ram, 2, 4); Run(device, ram, 4);
    Check(Word(ram, 0x10) == 5, "Vector distance");
    Word(ram, 0, 100); Word(ram, 2, 200); Word(ram, 4, 0x4000); Run(device, ram, 8);
    Check(Word(ram, 0x10) == 199 && Word(ram, 0x12) == uint16_t(-99), "Quarter-turn rotation keeps integer rounding");
    Word(ram, 0, 0); Word(ram, 2, 0xffff); Run(device, ram, 1);
    Check(Word(ram, 4) == 0x4000 && Word(ram, 6) == 0xffff && Word(ram, 0x10) == 0,
          "Compass preserves original Y and negative-axis quadrant behavior");
    Word(ram, 0, 0); Run(device, ram, 7);
    Check(Word(ram, 0xf0) == 895 && Word(ram, 0x510) == 895 && Word(ram, 0x3b0) == 0,
          "Raster output duplicates A/D and leaves zero B/C terms zero");
    Check(Word(ram, 0x24e) == 42 && Word(ram, 0x66e) == 42, "Raster includes final row");
    const auto before = ram;
    Run(device, ram, 0xff);
    auto expected = before; expected[0x20] = 0xff;
    Check(ram == expected, "Unknown commands acknowledge without modifying results");
}

void DriverSorting() {
    St010 device; Ram ram{};
    Word(ram, 0x24, 4);
    for (unsigned i = 0; i < 4; ++i) {
        Word(ram, 0x40 + 2 * i, i == 2 ? 20 : 10);
        Word(ram, 0x80 + 2 * i, uint16_t(100 + i));
    }
    Run(device, ram, 2);
    Check(Word(ram, 0x40) == 20 && Word(ram, 0x80) == 102 && Word(ram, 0x82) == 100 &&
          Word(ram, 0x84) == 101 && Word(ram, 0x86) == 103, "Sort is descending and stable across ties");
    Word(ram, 0x24, 0xffff);
    std::fill(ram.begin() + 0xc0, ram.end(), 0xa5);
    Run(device, ram, 2);
    Check(std::all_of(ram.begin() + 0xc0, ram.end(), [](uint8_t b) { return b == 0xa5; }),
          "Oversized driver counts stay within the 32-entry workspace");
}

void CommandVectors() {
    constexpr std::array<uint64_t, 8> expected{
        0x2c0c89e7475f3fbc, 0xb42ad000bc0cba40, 0x2a54f766417d0cb5, 0xa593785c17d2fbb9,
        0x00fc2c6268befc45, 0x972db69269cd6f12, 0xd6ad9e558119e811, 0x5f0466019cd1ad38};
    for (unsigned command = 1; command <= 8; ++command) {
        uint64_t digest = 14695981039346656037ull;
        for (unsigned index = 0; index < st010_test::Cases; ++index) {
            auto ram = st010_test::Input(command, index);
            St010 device;
            Run(device, ram, uint8_t(command));
            digest = st010_test::Digest(digest, ram);
        }
        if (digest != expected[command - 1]) {
            std::fprintf(stderr, "Command %u digest %016llx\n", command, static_cast<unsigned long long>(digest));
            throw std::runtime_error("Command output vector mismatch");
        }
    }
}

std::vector<uint8_t> Rom(uint8_t size = 10) {
    std::vector<uint8_t> rom(0x100000);
    for (unsigned bank = 0; bank < 32; ++bank)
        std::fill_n(rom.begin() + bank * 0x8000, 0x8000, uint8_t(bank));
    std::fill_n(rom.begin() + 0x7fc0, 64, 0);
    std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
    rom[0] = 0xdb; rom[0x7fc0] = 'T'; rom[0x7fd5] = 0x30; rom[0x7fd6] = 0xf6;
    rom[0x7fd7] = size; rom[0x7fdc] = rom[0x7fdd] = 0xff; rom[0x7ffd] = 0x80;
    return rom;
}

void CartridgeMapping() {
    auto image = Rom();
    auto cart = Cartridge::FromRomImage(image, nullptr, nullptr, nullptr);
    Check(cart && cart->Header().mapping == MappingType::St010 && cart->SramData().size() == 4096,
          "ST010 detection forces the board layout and RAM capacity");
    CartridgeDatabase database;
    DatabaseOverride override;
    override.forceSramSize = 0;
    database.AddOverride(cart->RomCrc32(), override);
    const auto overridden = Cartridge::FromRomImage(image, nullptr, &database, nullptr);
    Check(overridden && overridden->SramData().size() == 4096, "Header overrides cannot remove required device RAM");
    auto emu = std::make_unique<Emulator>();
    Check(emu->LoadCartridge(image), "ST010 is accepted by emulator");
    auto& bus = emu->GetBus();
    for (unsigned bank = 0; bank < 256; ++bank) {
        if (bank == 0x7e || bank == 0x7f) continue;
        const bool sram = (bank >= 0x70 && bank <= 0x7d) || bank >= 0xf0;
        Check(bus.Read((bank << 16) | 0x8100) == (sram ? 0 : (bank & 31)), "Upper-bank ROM pages and SRAM precedence");
        if (bank >= 0x60 && bank <= 0x67)
            Check(bus.Read((bank << 16) | 0x1234) == 0x80, "Every control bank decodes");
    }
    for (unsigned bank : {0x40, 0x5f, 0x60, 0x67, 0xc0, 0xe8, 0xef}) {
        const unsigned address = (bank << 16) | 0x4567;
        Check(cart->Read(address, 0x5a) == 0x5a, "Lower unused cartridge windows stay open");
        bus.Write(0x7e0000, 0x5a);
        Check(bus.Read(address) == 0x5a, "Bus leaves lower unused windows unmapped");
    }
    bus.Write(0x680123, 0x69);
    for (unsigned bank = 0x68; bank <= 0x6f; ++bank)
        for (unsigned page = 0; page < 8; ++page)
            Check(bus.Read((bank << 16) | (page << 12) | 0x123) == 0x69, "RAM aliases every 4 KiB in device banks");
    Check(bus.Read(0x700123) == 0x69 && bus.Read(0xf0f123) == 0x69, "Device and battery RAM share storage");
    bus.Write(0x600100, 0); bus.Write(0x6f7000, 3); bus.Write(0x680002, 7);
    bus.Write(0x6fa020, 6); // Outside the device window: ROM write, no command.
    bus.Write(0x6f7020, 6); bus.Write(0x687021, 0x80);
    Check(bus.Read(0x700010) == 42 && bus.Read(0x6f7020) == 0, "Mirrored command registers execute through the bus");
    const auto saved = emu->LoadedCartridge()->SramData();
    cart->LoadSram(saved);
    Check(cart->Read(0x680123) == 0x69 && cart->Read(0x700010) == 42, "Save restoration shares device RAM");
    Check(cart->Read(0x680020) == 0, "Save loading does not enable command registers");
    const auto sibling = Rom(9);
    std::string error;
    Check(!emu->LoadCartridge(std::span<const uint8_t>(image).first(4096), &error),
          "Truncated cartridge is rejected");
    Check(bus.Read(0x680123) == 0x69, "Rejected cartridge preserves the loaded machine");
    Check(emu->LoadCartridge(sibling) && emu->LoadedCartridge()->Header().chip == EnhancementChip::St011 &&
          bus.Read(0x600001) == 0xff, "Sibling board selects its distinct ST011 packet interface");
    Check(emu->LoadCartridge(image), "Reload ST010");
    bus.Write(0x680020, 6); bus.Write(0x680021, 0x80);
    Check(bus.Read(0x680020) == 0 && bus.Read(0x680010) == 0, "Reload clears enable, command and RAM state");
}

void CpuCommandProgram() {
    auto rom = Rom();
    const std::array<uint8_t, 47> program{
        0xa9, 0, 0x8f, 0, 0, 0x60,       // Enable control registers.
        0xa9, 12, 0x8f, 0, 0, 0x68,
        0xa9, 13, 0x8f, 2, 0, 0x68,
        0xa9, 6, 0x8f, 0x20, 0, 0x68,
        0xa9, 0x80, 0x8f, 0x21, 0, 0x68,
        0xaf, 0x10, 0, 0x68, 0x8f, 0, 0, 0x7e,
        0xaf, 0x11, 0, 0x68, 0x8f, 1, 0, 0x7e,
        0xdb};
    std::copy(program.begin(), program.end(), rom.begin());
    auto emu = std::make_unique<Emulator>();
    Check(emu->LoadCartridge(rom), "CPU fixture loads");
    emu->StepFrame();
    Check(emu->GetBus().Read(0x7e0000) == 0x38 && emu->GetBus().Read(0x7e0001) == 1,
          "65816 program executes a cartridge math command and reads the shared result");
}
}

int main() {
    try { RegistersAndMath(); DriverSorting(); CommandVectors(); CartridgeMapping(); CpuCommandProgram(); }
    catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
    std::puts("ST010 command, mapping, and shared RAM checks passed");
}
