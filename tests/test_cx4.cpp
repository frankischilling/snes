#include "snes/core/Cx4.hpp"

#include <array>
#include <cstdio>
#include <stdexcept>
#include <vector>

using snes::core::Cx4;
namespace {
void Check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void Put(Cx4& chip, unsigned address, uint64_t value, unsigned bytes = 2) {
    for (unsigned i = 0; i < bytes; ++i) chip.Write(0x6000 + address + i, uint8_t(value >> (i * 8)));
}
uint64_t Get(const Cx4& chip, unsigned address, unsigned bytes = 2) {
    uint64_t value = 0;
    for (unsigned i = 0; i < bytes; ++i) value |= uint64_t(chip.Read(0x6000 + address + i)) << (i * 8);
    return value;
}
void Command(Cx4& chip, uint8_t command) { chip.Write(0x7f4f, command); }

void MathAndRegisters() {
    Cx4 chip({});
    Check(Cx4::Selects(0x006000) && Cx4::Selects(0xbf7fff) && !Cx4::Selects(0x406000), "Cx4 register window decoder");
    chip.Write(0x806123, 0x59);
    Check(chip.Read(0x006123) == 0x59, "Work RAM shares the CPU bank mirrors");
    chip.Write(0x7f5e, 0xff); Check(chip.Read(0x7f5e) == 0, "Command completion port reads ready");
    for (uint32_t a : {0u, 1u, 0x7fffu, 0x8000u, 0x7fffffu, 0x800000u, 0xffffffu}) {
        for (uint32_t b : {0u, 1u, 0x1001u, 0x7fffffu, 0xffffffu}) {
            Put(chip, 0x1f80, a, 3); Put(chip, 0x1f83, b, 3); Command(chip, 0x25);
            Check(Get(chip, 0x1f80, 3) == (uint64_t(a) * b & 0xffffff), "Multiply returns the low 24 bits");
        }
        Put(chip, 0x1f80, a, 3); Command(chip, 0x54);
        const int64_t signedA = int64_t(a) - ((a & 0x800000) ? 0x1000000 : 0);
        Check(Get(chip, 0x1f83, 6) == uint64_t(signedA * signedA), "Square sign-extends 24-bit input and writes 48 bits");
    }
    Put(chip, 0x1f80, 300); Put(chip, 0x1f83, 400); Command(chip, 0x15);
    Check(Get(chip, 0x1f80) == 500, "Vector length follows a 3-4-5 triangle");
    for (unsigned i = 0; i < 4; ++i) {
        const std::array<int,4> xs{100,0,-100,0}, ys{0,100,0,-100};
        Put(chip, 0x1f80, uint16_t(xs[i])); Put(chip, 0x1f83, uint16_t(ys[i])); Command(chip, 0x1f);
        Check(Get(chip, 0x1f86) == i * 128, "Cardinal angles use a 512-step circle");
    }
    Put(chip, 0x1f80, 3); Put(chip, 0x1f83, 4); Put(chip, 0x1f86, 100); Command(chip, 0x0d);
    Check(Get(chip, 0x1f89) == 58 && Get(chip, 0x1f8c) == 79, "Vector normalization preserves command rounding");
    Put(chip, 0x1f80, 0); Put(chip, 0x1f83, 0); Command(chip, 0x0d);
    Check(Get(chip, 0x1f89) == 0 && Get(chip, 0x1f8c) == 0, "Zero-length normalization is bounded");
    Command(chip, 0x89); Check(Get(chip, 0x1f80, 3) == 0x054336, "ROM identification command");
    Put(chip, 0x1f4d, 0x0e, 1); Command(chip, 0x3c);
    Check(Get(chip, 0x1f80, 1) == 15, "Test-mode command returns its immediate index");
    chip.Reset();
    unsigned sum = 0;
    for (unsigned i = 0; i < 2048; ++i) { chip.Write(0x6000 + i, uint8_t(i)); sum += uint8_t(i); }
    Command(chip, 0x40); Check(Get(chip, 0x1f80) == (sum & 65535), "RAM checksum wraps at sixteen bits");
}

void Graphics() {
    std::vector<uint8_t> rom(0x8000);
    Cx4 chip(rom);
    // One identity-transformed packed tile contains the same eight colors on every row.
    for (unsigned row = 0; row < 8; ++row) for (unsigned col = 0; col < 4; ++col)
        chip.Write(0x6600 + row * 4 + col, uint8_t((col * 2) | ((col * 2 + 1) << 4)));
    Put(chip, 0x1f4d, 3, 1); Put(chip, 0x1f89, 8, 1); Put(chip, 0x1f8c, 8, 1);
    Put(chip, 0x1f83, 4); Put(chip, 0x1f86, 4); Put(chip, 0x1f8f, 4096); Put(chip, 0x1f92, 4096);
    Command(chip, 0);
    for (unsigned row = 0; row < 8; ++row) {
        Check(Get(chip, row * 2) == 0x3355 && Get(chip, 16 + row * 2) == 0x000f,
              "Identity affine conversion emits four SNES bitplanes");
    }
    chip.Reset();
    Put(chip, 0x620, 1, 1); Put(chip, 0x626, 3, 1);
    Put(chip, 0x220, uint16_t(-4)); Put(chip, 0x222, 20);
    Put(chip, 0x224, 0x12, 1); Put(chip, 0x225, 7, 1); Put(chip, 0x227, 0x008000, 3);
    Command(chip, 0);
    Check(Get(chip, 12, 4) == 0x120714fc && (Get(chip, 0x200, 1) & 0xc0) == 0xc0,
          "OAM output packs signed X and large-sprite flags into the correct high table slot");
    Check(Get(chip, 17, 1) == 0xe0 && Get(chip, 509, 1) == 0xe0, "Unused OAM slots are hidden");
    chip.Reset();
    Put(chip, 0x1f81, 20); Put(chip, 0x1f84, uint16_t(-10)); Put(chip, 0x1f87, 0); Put(chip, 0x1f90, 256);
    Command(chip, 0x2d);
    Check(Get(chip, 0x1f80) == 20 && Get(chip, 0x1f83) == uint16_t(-10), "Zero-angle 3D transform preserves coordinates");
    chip.Reset();
    Put(chip, 0x1f80, 10); Put(chip, 0x1f83, 0); Put(chip, 0x1f86, 30); Put(chip, 0x1f89, 0);
    Put(chip, 0x1f93, 40); Command(chip, 0x22);
    Check(Get(chip, 0x800, 1) == 20 && Get(chip, 0x900, 1) == 60 && Get(chip, 0x8e0, 1) == 20,
          "Zero-slope trapezoid produces stable window edges");
}

void DmaAndBounds() {
    std::vector<uint8_t> rom(0x18000);
    for (unsigned i = 0; i < rom.size(); ++i) rom[i] = uint8_t(i + i / 256);
    Cx4 chip(rom);
    Put(chip, 0x1f40, 0x00fff0, 3); Put(chip, 0x1f43, 32); Put(chip, 0x1f45, 0x6080);
    chip.Write(0x7f47, 0);
    for (unsigned i = 0; i < 32; ++i) Check(chip.Read(0x6080 + i) == rom[0x7ff0 + i], "ROM transfer crosses physical ROM offsets continuously");
    Put(chip, 0x1f40, 0x03fff0, 3); Put(chip, 0x1f43, 64); Put(chip, 0x1f45, 0x7ff0);
    chip.Write(0x7f47, 0);
    Check(chip.Read(0x6000) == rom[0], "DMA safely wraps physical work RAM and ROM address lines");
    // Malformed dimensions cannot index beyond the chip's physical RAM.
    for (uint8_t mode : {uint8_t(3),uint8_t(5),uint8_t(7),uint8_t(11),uint8_t(12)}) {
        chip.Reset();
        for (unsigned offset = 0x1f80; offset < 0x1fa0; ++offset) Put(chip, offset, 255, 1);
        Put(chip, 0x1f4d, mode, 1); Command(chip, 0);
    }
}
}

int main() {
    try { MathAndRegisters(); Graphics(); DmaAndBounds(); }
    catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
    std::puts("Cx4 command, graphics, ROM transfer and bounds checks passed");
}
