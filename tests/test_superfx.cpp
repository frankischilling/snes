#include "snes/core/SuperFx.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <initializer_list>
#include <stdexcept>
#include <utility>
#include <vector>

using snes::core::SuperFx;

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

struct Fixture {
    std::vector<uint8_t> rom;
    std::vector<uint8_t> ram;
    SuperFx chip;

    explicit Fixture(size_t romSize = 0x8000, size_t ramSize = 0x10000)
        : rom(romSize), ram(ramSize), chip(rom, ram) {
        chip.WriteCpu(0x303a, 0x18);
    }
    void Reg(unsigned n, uint16_t value) {
        chip.WriteCpu(0x3000 + n * 2, uint8_t(value));
        chip.WriteCpu(0x3001 + n * 2, uint8_t(value >> 8));
    }
    uint16_t Reg(unsigned n) {
        return uint16_t(chip.ReadCpu(0x3000 + n * 2) | (uint16_t(chip.ReadCpu(0x3001 + n * 2)) << 8));
    }
    void Program(std::initializer_list<uint8_t> bytes, size_t offset = 0) {
        Check(offset + bytes.size() <= rom.size(), "Program fits ROM fixture");
        std::copy(bytes.begin(), bytes.end(), rom.begin() + offset);
    }
    void Start(uint16_t address = 0x8000) { Reg(15, address); }
    bool Running() { return (chip.ReadCpu(0x3030) & 0x20) != 0; }
    void Run(uint16_t address = 0x8000) {
        Start(address);
        chip.Advance(10000);
        Check(!Running(), "Native GSU program reaches STOP");
    }
    void Cache(std::initializer_list<uint8_t> program) {
        std::array<uint8_t, 16> line{};
        Check(program.size() <= line.size(), "Program fits one cache line");
        std::copy(program.begin(), program.end(), line.begin());
        for (unsigned i = 0; i < line.size(); ++i) chip.WriteCpu(0x3100 + i, line[i]);
    }
};

constexpr uint8_t Prefix(unsigned mode) { return mode ? uint8_t(0x3c + mode) : uint8_t(1); }
int Signed8(uint16_t value) { return (value & 255) < 128 ? (value & 255) : int(value & 255) - 256; }
int Signed16(uint16_t value) { return value < 32768 ? value : int(value) - 65536; }

void RegistersAndMapping() {
    Fixture f(0x200000, 0x40000);
    for (size_t i = 0; i < f.rom.size(); ++i) f.rom[i] = uint8_t(i / 0x8000);
    for (unsigned bank = 0; bank < 64; ++bank) {
        Check(f.chip.ReadCpu((bank << 16) | 0x9234) == bank, "LoROM cartridge bank mapping");
        Check(f.chip.ReadCpu(((bank + 0x80) << 16) | 0x9234) == bank, "LoROM upper-bank mirror");
    }
    for (unsigned bank = 0x40; bank < 0x60; ++bank) {
        Check(f.chip.ReadCpu((bank << 16) | 0x1234) == (bank - 0x40) * 2, "Linear ROM lower half");
        Check(f.chip.ReadCpu(((bank + 0x80) << 16) | 0x9234) == (bank - 0x40) * 2 + 1,
              "Linear ROM upper half and high bank mirror");
    }
    for (unsigned bank : {0x60, 0x6f, 0x74, 0x7d, 0xe0, 0xef})
        Check(f.chip.ReadCpu(bank << 16, 0xa5) == 0xa5, "Unconnected cartridge bank is open bus");
    f.chip.WriteCpu(0x006123, 0x97);
    Check(f.chip.ReadCpu(0x3f6123) == 0x97 && f.chip.ReadCpu(0xbf6123) == 0x97 &&
          f.chip.ReadCpu(0x700123) == 0x97, "CPU small RAM windows share the physical first page");
    for (unsigned bank = 0; bank < 4; ++bank) {
        f.chip.WriteCpu(((bank + 0x70) << 16) | 0x2222, uint8_t(0x40 + bank));
        Check(f.ram[(bank << 16) | 0x2222] == 0x40 + bank, "Full RAM banks keep independent storage");
        if (bank < 2)
            Check(f.chip.ReadCpu(((bank + 0xf0) << 16) | 0x2222) == 0x40 + bank, "First two RAM banks have upper aliases");
        else
            Check(f.chip.ReadCpu(((bank + 0xf0) << 16) | 0x2222, 0xa5) == 0xa5, "Additional RAM banks have no upper alias");
    }
    for (unsigned n = 0; n < 15; ++n) {
        f.Reg(n, uint16_t(0x1234 + n));
        Check(f.Reg(n) == 0x1234 + n, "CPU register low and high byte access");
    }
    f.chip.WriteCpu(0x3034, 0xff);
    Check(f.chip.ReadCpu(0x803034) == 0x7f, "PBR is seven bits and MMIO mirrors");
    f.chip.WriteCpu(0x3036, 0x42);
    f.chip.WriteCpu(0x303c, 3);
    f.chip.WriteCpu(0x303e, 0xf0);
    f.chip.WriteCpu(0x303f, 0xff);
    f.chip.WriteCpu(0x303b, 0);
    Check(f.chip.ReadCpu(0x3036) == 0 && f.chip.ReadCpu(0x303c) == 0 &&
          f.chip.ReadCpu(0x303e) == 0 && f.chip.ReadCpu(0x303f) == 0 && f.chip.ReadCpu(0x303b) == 4,
          "CPU cannot write ROMBR, RAMBR, CBR or version code");
    f.chip.WriteCpu(0x3037, 0xff);
    Check(f.chip.ReadCpu(0x3037) == 0 && f.chip.ReadCpu(0x3020) == 0, "Write-only and unused registers read zero");
    f.chip.Reset();
    Check(f.Reg(0) == 0 && f.Reg(14) == 0 && !f.Running() && !f.chip.CpuIrqPending(), "Reset clears register and IRQ state");
    Check(f.ram[0x123] == 0x97, "Reset preserves cartridge RAM");

    Fixture small(0x18000, 0x8000);
    std::fill(small.rom.begin(), small.rom.begin() + 0x10000, 0x11);
    std::fill(small.rom.begin() + 0x10000, small.rom.end(), 0x22);
    Check(small.chip.ReadCpu(0x018100) == 0x11 && small.chip.ReadCpu(0x028100) == 0x22 &&
          small.chip.ReadCpu(0x038100) == 0x22, "Non-power-of-two ROM folds only absent address lines");
    small.chip.WriteCpu(0x710123, 0x73);
    Check(small.chip.ReadCpu(0x708123) == 0x73, "Small RAM mirrors disconnected address lines");

    Fixture large(0x400000, 0x40000);
    for (size_t i = 0; i < large.rom.size(); ++i) large.rom[i] = uint8_t(i / 0x8000);
    for (unsigned bank = 0xc0; bank <= 0xff; ++bank) {
        Check(large.chip.ReadCpu((bank << 16) | 0x1234) == (bank - 0xc0) * 2 &&
              large.chip.ReadCpu((bank << 16) | 0x9234) == (bank - 0xc0) * 2 + 1,
              "Four-megabyte ROM occupies the full CPU C0-FF linear window");
    }
    for (unsigned bank = 0; bank < 0x40; ++bank) {
        Check(large.chip.ReadCpu((bank << 16) | 0x9234) == bank &&
              large.chip.ReadCpu(((bank + 0x80) << 16) | 0x9234) == bank,
              "Four-megabyte boards retain first-two-megabyte system ROM windows");
    }
    for (unsigned bank = 0x40; bank < 0x60; ++bank)
        Check(large.chip.ReadCpu((bank << 16) | 0x1234) == (bank - 0x40) * 2,
              "Lower linear ROM window remains restricted to the first two megabytes");
    for (unsigned bank = 0; bank < 4; ++bank) {
        large.chip.WriteCpu(((bank + 0x70) << 16) | 0x1234, uint8_t(0x90 + bank));
        large.chip.WriteCpu(((bank + 0xf0) << 16) | 0x1234, 0);
        Check(large.ram[(bank << 16) | 0x1234] == 0x90 + bank &&
              large.chip.ReadCpu(((bank + 0xf0) << 16) | 0x1234) == 0x60 + bank * 2,
              "Upper CPU ROM wins over RAM aliases and ignores CPU writes on large boards");
    }
    large.Program({0x3f, 0xdf, 0xde, 0xef, 0, 1});
    large.Reg(0, 0x40); large.Reg(14, 0x200);
    large.Run();
    Check(large.Reg(0) == 0 && large.chip.ReadCpu(0xe00201) == 0x40,
          "CPU extension does not expand the GSU's two-megabyte ROM address bus");

    Check(SuperFx::Selects(0x803100) && SuperFx::Selects(0x406000) && SuperFx::Selects(0x706000) &&
          SuperFx::Selects(0xff1234) && !SuperFx::Selects(0x002100) && !SuperFx::Selects(0x7e1000),
          "Integration window decoder includes the optional upper ROM extension");
}

void ArithmeticMatrix() {
    constexpr std::array<uint16_t, 10> values{0, 1, 0x7f, 0x80, 0xff, 0x7fff, 0x8000, 0xffff, 0x1234, 0xfedc};
    unsigned cases = 0;
    for (unsigned family : {0x50, 0x60, 0x70, 0x80, 0xc0}) {
        for (unsigned n = (family == 0x70 || family == 0xc0) ? 1 : 0; n < 16; ++n) {
            for (unsigned mode = 0; mode < 4; ++mode) {
                for (unsigned k = 0; k < values.size(); ++k) {
                    for (unsigned carryIn = 0; carryIn < 2; ++carryIn) {
                        Fixture f;
                        const uint16_t left = values[k], seed = values[(k + 3) % values.size()];
                        for (unsigned reg = 1; reg < 15; ++reg) f.Reg(reg, seed);
                        f.Reg(0, left);
                        f.chip.WriteCpu(0x3030, uint8_t(0x1a | (carryIn << 2)));
                        f.Program({Prefix(mode), 0x13, uint8_t(family + n), 0, 1});
                        const bool sub = family == 0x60;
                        const bool isImmediate = (mode & 2) && !(sub && mode == 3);
                        const uint16_t right = isImmediate ? uint16_t(n) : n == 0 ? left : n == 15 ? 0x8003 : seed;
                        uint16_t result = 0;
                        uint8_t flags = uint8_t(0x10 | (carryIn << 2));
                        if (family == 0x50 || sub) {
                            const int adjustment = sub ? (mode == 1 ? int(carryIn) - 1 : 0) :
                                                        ((mode & 1) ? int(carryIn) : 0);
                            const int wide = int(left) + (sub ? -int(right) : int(right)) + adjustment;
                            const int signedWide = Signed16(left) + (sub ? -Signed16(right) : Signed16(right)) + adjustment;
                            result = uint16_t(wide);
                            flags = uint8_t((sub ? wide >= 0 : wide >= 65536) ? 4 : 0);
                            if (signedWide < -32768 || signedWide > 32767) flags |= 0x10;
                        } else if (family == 0x70) result = uint16_t(left & ((mode & 1) ? uint16_t(~right) : right));
                        else if (family == 0xc0) result = uint16_t((mode & 1) ? left ^ right : left | right);
                        else result = uint16_t((mode & 1) ? int(left & 255) * int(right & 255) : Signed8(left) * Signed8(right));
                        if (result == 0) flags |= 2;
                        if (result & 0x8000) flags |= 8;
                        const uint16_t expectedRegister = sub && mode == 3 ? seed : result;
                        f.Run();
                        const uint8_t actualFlags = f.chip.ReadCpu(0x3030) & 0x1e;
                        if (f.Reg(3) != expectedRegister || actualFlags != flags) {
                            std::fprintf(stderr, "ALU op=%02x alt=%u left=%04x right=%04x carry=%u result=%04x/%04x flags=%02x/%02x\n",
                                family + n, mode, left, right, carryIn, f.Reg(3), expectedRegister, actualFlags, flags);
                            throw std::runtime_error("ALU register or arithmetic flag mismatch");
                        }
                        ++cases;
                    }
                }
            }
        }
    }
    Check(cases == 6240, "All ALU register/immediate encodings were exercised");
}

void ExpandedCpuWindows() {
    Fixture f(0xb00000, 0x100000);
    f.rom[0x7fd7] = 14;
    for (unsigned bank = 0; bank < 0xb0; ++bank) f.rom[bank * 0x10000 + 0x234] = uint8_t(bank);
    Check(f.chip.ReadCpu(0x800234, 0x51) == 0x51 && f.chip.ReadCpu(0x808234) == 0x20 &&
          f.chip.ReadCpu(0xc00234) == 0x40 && f.chip.ReadCpu(0x600234) == 0xa0,
          "CPU expansion changes ROM windows without claiming low system memory");
    f.chip.WriteCpu(0x700234, 0x19);
    f.chip.WriteCpu(0x740234, 0x29);
    Check(f.chip.ReadCpu(0x700234) == 0x19 && f.chip.ReadCpu(0x740234) == 0x29,
          "CPU RAM extension keeps address bit eighteen");
    f.Reg(0, 7); f.Reg(1, 0x0234);
    f.Program({0x3e, 0xdf, 0x41, 0, 1});
    f.ram[0x10234] = 0x67;
    f.ram[0x30234] = 0x87;
    f.ram[0x70234] = 0x98;
    f.Run();
    Check(f.Reg(0) == 0x67 && f.chip.ReadCpu(0x303c) == 1,
          "GSU RAMB retains its one-bit register when CPU-only expansion RAM is present");
    f.Reg(0, 0x40); f.Reg(14, 0x234);
    f.Program({0x3f, 0xdf, 0xde, 0xee, 0xef, 0, 1});
    f.Run();
    Check(f.Reg(0) == 0 && f.chip.ReadCpu(0x400234) == 0x80,
          "GSU ROM reads still see the first two MiB through its own bank decoder");

    Fixture partial(0x900000);
    partial.rom[0x7fd7] = 14;
    partial.rom[0x800123] = 0x76;
    Check(partial.chip.ReadCpu(0x400123) == 0x76 && partial.chip.ReadCpu(0x600123) == 0x76,
          "A partly populated final ROM region mirrors its connected address lines");
    Fixture shortImage(0x200000);
    shortImage.rom[0x7fd7] = 14;
    shortImage.chip.WriteCpu(0xf00123, 0x75);
    Check(shortImage.chip.ReadCpu(0xf00123, 0x5a) == 0x5a && shortImage.ram[0x123] == 0,
          "Unpopulated expanded ROM reads open bus and cannot alias small-board RAM");
}

void TransfersAndSpecialOperations() {
    for (unsigned mode = 0; mode < 4; ++mode) {
        for (unsigned n = 0; n < 16; ++n) {
            Fixture f;
            f.Reg(0, 0x3456);
            f.Program({Prefix(mode), 0x20, uint8_t(0x10 + n), 0, 1});
            f.Run();
            Check(f.Reg(n) == (n == 15 ? 0x3457 : 0x3456), "WITH/TO MOVE encodings, including R15 destination");
            Fixture g;
            if (n != 15) g.Reg(n, 0x8080);
            g.chip.WriteCpu(0x3030, 4);
            g.Program({Prefix(mode), 0x23, uint8_t(0xb0 + n), 0, 1});
            g.Run();
            const uint16_t expected = n == 15 ? 0x8003 : 0x8080;
            Check(g.Reg(3) == expected, "WITH/FROM MOVES encodings");
            Check((g.chip.ReadCpu(0x3030) & 0x1e) == (n == 15 ? 0x0c : 0x1c), "MOVES derives overflow from bit seven");
        }
    }
    struct Unary { uint8_t opcode, mode; uint16_t input, result; uint8_t flags; };
    constexpr Unary cases[]{
        {0x03, 0, 1, 0, 6}, {0x03, 3, 0xffff, 0x7fff, 4},
        {0x04, 0, 0x8000, 1, 4}, {0x04, 3, 0x7fff, 0xffff, 8},
        {0x4d, 0, 0x0180, 0x8001, 12}, {0x4f, 0, 0xffff, 0, 6},
        {0x95, 0, 0x1280, 0xff80, 12}, {0x95, 2, 0xff7f, 0x007f, 4},
        {0x96, 0, 0xffff, 0xffff, 12}, {0x96, 1, 0xffff, 0, 6},
        {0x96, 3, 0xfffd, 0xfffe, 12}, {0x97, 0, 2, 0x8001, 8},
        {0x9e, 0, 0x1280, 0x0080, 12}, {0xc0, 3, 0x8001, 0x0080, 12}
    };
    for (const auto& test : cases) {
        Fixture f;
        f.Reg(0, test.input);
        f.chip.WriteCpu(0x3030, 4);
        f.Program({Prefix(test.mode), test.opcode, 0, 1});
        f.Run();
        Check(f.Reg(0) == test.result && (f.chip.ReadCpu(0x3030) & 0x1e) == test.flags,
              "Unary operation result and carry/sign/zero edge behavior");
    }
    for (unsigned mode = 0; mode < 4; ++mode) {
        Fixture f;
        f.Reg(7, 0x1234); f.Reg(8, 0x80ff);
        f.Program({Prefix(mode), 0x70, 0, 1}); f.Run();
        Check(f.Reg(0) == 0x1280 && (f.chip.ReadCpu(0x3030) & 0x1e) == 0x1e, "MERGE texture flag masks");
        Fixture g;
        g.Reg(0, 0x8000); g.Reg(6, 3);
        g.Program({Prefix(mode), 0x9f, 0, 1}); g.Run();
        Check(g.Reg(0) == 0xfffe && g.Reg(4) == ((mode & 1) ? 0x8000 : 0) &&
              (g.chip.ReadCpu(0x3030) & 0x1e) == 12, "FMULT/LMULT signed full product and rounding carry");
        for (unsigned n = 0; n < 15; ++n) {
            Fixture h;
            h.Reg(n, 0xffff);
            h.Program({Prefix(mode), uint8_t(0xd0 + n), 0, 1}); h.Run();
            Check(h.Reg(n) == 0 && (h.chip.ReadCpu(0x3030) & 0x0a) == 2, "INC wraps and updates flags for each register");
            h.Program({Prefix(mode), uint8_t(0xe0 + n), 0, 1}); h.Run();
            Check(h.Reg(n) == 0xffff && (h.chip.ReadCpu(0x3030) & 0x0a) == 8, "DEC wraps and updates flags for each register");
        }
    }
}

void BranchesAndPipeline() {
    for (unsigned mode = 0; mode < 4; ++mode) {
        for (unsigned flags = 0; flags < 16; ++flags) {
            const bool z = flags & 1, c = flags & 2, s = flags & 4, v = flags & 8;
            const std::array<bool, 11> take{true, s == v, s != v, !z, z, !s, s, !c, c, !v, v};
            for (unsigned n = 0; n < take.size(); ++n) {
                Fixture f;
                f.chip.WriteCpu(0x3030, uint8_t(flags << 1));
                f.Program({Prefix(mode), uint8_t(5 + n), 4, 0xd1, 0xe1, 0xe1, 0xe1, 0xd1, 0, 1});
                f.Run();
                Check(f.Reg(1) == (take[n] ? 2 : 0xffff), "Conditional branch truth table and executed delay slot");
            }
        }
    }
    Fixture f;
    f.Reg(0, 1); f.Reg(1, 50);
    f.Program({0x3e, 0x05, 4, 0x13, 0, 0, 0, 0x51, 0, 1}); f.Run();
    Check(f.Reg(3) == 2, "Branches retain ALT prefixes and a TO in the delay slot selects the destination");

    for (unsigned mode = 0; mode < 4; ++mode) {
        for (unsigned n = 8; n <= 13; ++n) {
            Fixture g(0x200000);
            g.Reg(n, uint16_t((mode & 1) ? 0x41 : 0x8120));
            g.Reg(0, 0x8120);
            g.Program({Prefix(mode), uint8_t(0x90 + n), 0xd1, 0, 1});
            g.Program({0xd2, 0, 1}, (mode & 1) ? 0x18120 : 0x120);
            g.Run();
            Check(g.Reg(1) == 1 && g.Reg(2) == 1, "JMP/LJMP execute the old prefetched instruction then the destination");
            Check(g.chip.ReadCpu(0x3034) == ((mode & 1) ? 0x41 : 0), "LJMP takes program bank from the opcode register");
        }
        Fixture g;
        g.Reg(12, 3); g.Reg(13, 0x8001);
        g.Program({Prefix(mode), 0xd0, 0x3c, 0xd1, 0, 1}); g.Run();
        Check(g.Reg(0) == 3 && g.Reg(1) == 3 && g.Reg(12) == 0, "LOOP decrement and final delay-slot execution");
        for (unsigned n = 1; n <= 4; ++n) {
            Fixture h;
            h.Program({Prefix(mode), uint8_t(0x90 + n), 0, 1}); h.Run();
            Check(h.Reg(11) == 0x8002 + n, "LINK offsets use prefetched PC");
        }
    }
    Fixture h;
    h.Reg(0, 0x8100);
    h.Program({0x20, 0x1f, 0xd1, 0, 1}); h.Program({0xd2, 0, 1}, 0x100); h.Run();
    Check(h.Reg(1) == 1 && h.Reg(2) == 1, "MOVE into R15 has one delay slot and no implicit increment");

    Fixture slots;
    slots.Reg(8, 0x8100);
    slots.Program({0x98, 0xa0, 0x66, 0, 1});
    slots.Program({0x77, 0, 1}, 0x100);
    slots.Run();
    Check(slots.Reg(0) == 0x77, "A multi-byte delay-slot instruction consumes immediate data from the jump target");
}

void RamInstructions() {
    for (unsigned mode = 0; mode < 4; ++mode) {
        for (unsigned n = 0; n < 16; ++n) {
            for (bool word : {false, true}) {
                Fixture f;
                f.Reg(n == 15 ? 0 : n, 0x5678);
                const uint8_t opcode = uint8_t((word ? 0xf0 : 0xa0) + n);
                const uint16_t address = word ? 0x0123 : 0x0106;
                f.ram[address] = 0xcd; f.ram[address ^ 1] = 0xab;
                if (word) f.Program({Prefix(mode), opcode, 0x23, 0x01, 0, 1});
                else f.Program({Prefix(mode), opcode, 0x83, 0, 1});
                f.Run();
                if (mode == 2) {
                    const uint16_t expected = n == 15 ? 0x8002 : 0x5678;
                    Check(f.ram[address] == uint8_t(expected) && f.ram[address ^ 1] == uint8_t(expected >> 8),
                          "SM/SMS sample source register and write byte-paired RAM address");
                } else {
                    const uint16_t expected = !mode ? (word ? 0x0123 : 0xff83) : 0xabcd;
                    Check(f.Reg(n) == uint16_t(expected + (n == 15 ? 1 : 0)), "IBT/IWT/LM/LMS all register encodings and ALT aliases");
                }
            }
        }
        for (unsigned n = 0; n < 12; ++n) {
            Fixture f;
            f.Reg(0, 0x12ab); f.Reg(n, 0xffff);
            const uint16_t stored = n == 0 ? 0xffff : 0x12ab;
            f.Program({Prefix(mode), uint8_t(0x30 + n), 0, 1}); f.Run();
            Check(f.ram[0xffff] == uint8_t(stored) && f.ram[0xfffe] == ((mode & 1) ? 0 : uint8_t(stored >> 8)),
                  "STB/STW use odd address and XOR adjacent high byte");
            Fixture g;
            g.Reg(n, 0xffff); g.ram[0xffff] = 0x9a; g.ram[0xfffe] = 0xbc;
            g.chip.WriteCpu(0x3030, 0x1e);
            g.Program({Prefix(mode), 0x13, uint8_t(0x40 + n), 0, 1}); g.Run();
            Check(g.Reg(3) == ((mode & 1) ? 0x009a : 0xbc9a), "LDB/LDW all address registers");
            Check((g.chip.ReadCpu(0x3030) & 0x1e) == 0x1e, "RAM loads preserve arithmetic flags");
        }
    }
    // Nintendo Book II 2-4-6: RAMBR contains only A16. Extra CPU RAM
    // address lines cannot turn the reserved bits into GSU bank selects.
    for (unsigned selected = 0; selected < 256; ++selected) {
        Fixture f(0x8000, 0x40000);
        f.Reg(0, uint16_t(selected));
        f.Program({0x3e, 0xdf, 0xa1, 0x21, 0x41, 0xa0, 0x5a, 0x90, 0, 1});
        const unsigned base = (selected & 1) << 16;
        f.ram[base + 0x21] = 0x34; f.ram[base + 0x20] = 0x12;
        f.ram[0x30021] = 0x87;
        f.Run();
        Check(f.chip.ReadCpu(0x303c) == (selected & 1) &&
              f.ram[base + 0x21] == 0x5a && f.ram[base + 0x20] == 0,
              "RAMB selects bank 70/71 and SBK uses the last load address");
        Check(f.ram[(base ^ 0x10000) + 0x21] == 0 && f.ram[0x30021] == 0x87,
              "GSU banked access leaves other CPU RAM banks untouched");
    }
}

void RomBufferInstructions() {
    for (unsigned mode = 0; mode < 4; ++mode) {
        Fixture f;
        f.rom[0x200] = 0x85;
        f.Reg(0, 0x1234); f.Reg(14, 0x8200);
        f.chip.WriteCpu(0x3030, 0x1e);
        f.Program({Prefix(mode), 0xef, 0, 1}); f.Run();
        constexpr uint16_t expected[]{0x0085, 0x8534, 0x1285, 0xff85};
        Check(f.Reg(0) == expected[mode] && (f.chip.ReadCpu(0x3030) & 0x1e) == 0x1e,
              "GETB/GETBH/GETBL/GETBS combine with source bytes without changing flags");
    }
    Fixture f(0x200000);
    f.rom[0x200] = 0x11; f.rom[0x10200] = 0x22;
    f.Reg(14, 0x8200); f.Reg(0, 2);
    f.Program({0x3f, 0xdf, 0x13, 0xef, 0xee, 0xde, 0x14, 0xef, 0, 1}); f.Run();
    Check(f.Reg(3) == 0x11 && f.Reg(4) == 0x22 && f.chip.ReadCpu(0x3036) == 2,
          "ROMB retains buffered data until an R14 write requests a new bank read");
    Fixture g;
    g.rom[0x234] = 0x6b;
    g.chip.WriteCpu(0x303a, 0);
    g.Reg(14, 0x8234);
    g.chip.Advance(100);
    Check(g.chip.ReadCpu(0x3030) & 0x40, "ROM read remains pending while CPU owns ROM");
    g.chip.WriteCpu(0x303a, 0x10);
    g.chip.Advance(5);
    Check(g.chip.ReadCpu(0x3030) & 0x40, "ROM buffer busy flag lasts the full memory latency");
    g.chip.Advance(1);
    Check(!(g.chip.ReadCpu(0x3030) & 0x40), "ROM buffer completion advances even while GSU is stopped");
}

unsigned TileOffset(unsigned x, unsigned y, unsigned bpp, unsigned heightMode, unsigned base) {
    const unsigned tx = x / 8, ty = y / 8;
    unsigned tile;
    if (heightMode < 3) tile = tx * (128 + 32 * heightMode) / 8 + ty;
    else tile = (x >= 128 ? 256 : 0) + (y >= 128 ? 512 : 0) + (ty % 16) * 16 + tx % 16;
    return base * 1024 + tile * bpp * 8 + 2 * (y % 8);
}

void PixelsAndScreenModes() {
    for (unsigned depthMode = 0; depthMode < 4; ++depthMode) {
        for (unsigned height = 0; height < 4; ++height) {
            for (bool obj : {false, true}) {
                Fixture f(0x8000, 0x40000);
                const unsigned bpp = depthMode == 0 ? 2 : depthMode == 3 ? 8 : 4;
                const unsigned x = 137, y = 145, color = 0xad;
                const auto scmr = uint8_t(0x18 | depthMode | ((height & 1) << 2) | ((height & 2) << 4));
                f.chip.WriteCpu(0x303a, scmr); f.chip.WriteCpu(0x3038, 2);
                f.Reg(0, obj ? 16 : 0); f.Reg(1, x); f.Reg(2, y); f.Reg(3, color);
                f.Program({0x3d, 0x4e, 0xb3, 0x4e, 0x4c, 0xe1, 0x3d, 0x4c, 0, 1});
                f.Run();
                const unsigned address = TileOffset(x, y, bpp, obj ? 3 : height, 2);
                for (unsigned plane = 0; plane < bpp; ++plane)
                    Check(f.ram[address + (plane / 2) * 16 + plane % 2] == ((color & (1u << plane)) ? 0x40 : 0),
                          "Screen height, screen base, color depth and OBJ tile arrangement");
                Check(f.Reg(0) == (color & ((1u << bpp) - 1)), "RPIX flushes both pixel rows and reads planar color");
            }
        }
    }
    Fixture f;
    f.Reg(0, 3);
    f.Program({0x4e, 0x4c, 0, 1}); f.Run();
    Check(f.ram[0] == 0 && f.ram[1] == 0 && f.Reg(1) == 1, "STOP does not implicitly flush a partial pixel row");
    f.Program({0xe1, 0x3d, 0x4c, 0, 1}); f.Run();
    Check(f.ram[0] == 0x80 && f.ram[1] == 0x80 && f.Reg(0) == 3, "Pixel rows survive STOP and RPIX writes them back");

    Fixture g;
    g.Reg(0, 0); g.Reg(1, 2);
    std::fill(g.ram.begin(), g.ram.end(), 0xff);
    g.Program({0x4e, 0x4c, 0xe1, 0x3d, 0x4c, 0, 1}); g.Run();
    Check(g.ram[0] == 0xff && g.Reg(0) == 3, "Transparent color zero leaves existing pixels intact");
    g.Reg(0, 1); g.Reg(3, 0);
    g.Program({0x3d, 0x4e, 0xb3, 0x4e, 0x4c, 0xe1, 0x3d, 0x4c, 0, 1}); g.Run();
    Check(g.ram[0] == 0xdf && g.ram[1] == 0xdf && g.Reg(0) == 0,
          "POR transparency disable plots zero while preserving the other seven pixels");

    Fixture d;
    d.chip.WriteCpu(0x303a, 0x19); d.Reg(0, 2); d.Reg(3, 0xa5);
    d.Program({0x3d, 0x4e, 0xb3, 0x4e, 0x4c, 0x4c, 0xe1, 0x3d, 0x4c, 0, 1}); d.Run();
    Check(d.ram[0] == 0x80 && d.ram[1] == 0x40 && d.ram[16] == 0x80 && d.ram[17] == 0x40 && d.Reg(0) == 10,
          "Dither alternates the color nibbles across neighboring pixels");

    Fixture c;
    c.chip.WriteCpu(0x303a, 0x1b); c.Reg(0, 0xa0); c.Reg(3, 4); c.Reg(4, 0x53);
    c.Program({0x4e, 0xb3, 0x3d, 0x4e, 0xb4, 0x4e, 0x4c, 0xe1, 0x3d, 0x4c, 0, 1}); c.Run();
    Check(c.Reg(0) == 0xa5, "High-nibble COLOR mode preserves the previous high color nibble");

    Fixture cache;
    cache.Reg(0, 3);
    cache.Program({0x4e, 0x4c, 0xa1, 8, 0x4c, 0xa1, 16, 0x4c, 0, 1}); cache.Run();
    Check(cache.ram[0] == 0x80 && cache.ram[256] == 0 && cache.ram[512] == 0,
          "PLOT retains two pending rows and evicts the oldest when a third row begins");
}

void CacheAndSynchronization() {
    for (unsigned fast = 0; fast < 2; ++fast) {
        Fixture f;
        f.chip.WriteCpu(0x3039, uint8_t(fast)); f.Program({0, 1}); f.Start();
        f.chip.Advance(0);
        Check(f.Running() && !f.chip.CpuIrqPending(), "Zero clocks execute no instructions");
        const unsigned stopTime = fast ? 10 : 12;
        f.chip.Advance(stopTime - 1);
        Check(f.Running() && !f.chip.CpuIrqPending(), "Uncached STOP cannot publish IRQ before both fetches finish");
        f.chip.Advance(1);
        Check(!f.Running() && f.chip.CpuIrqPending(), "STOP and IRQ become visible at the exact completion boundary");
        Check(f.chip.ReadCpu(0x3030) == 0 && f.chip.CpuIrqPending(), "Reading low SFR does not acknowledge IRQ");
        Check((f.chip.ReadCpu(0x3031) & 0x80) && !f.chip.CpuIrqPending(), "Reading high SFR returns and clears IRQ");

        Fixture g;
        g.chip.WriteCpu(0x3039, uint8_t(fast)); g.chip.WriteCpu(0x303a, 0);
        g.Cache({0xa0, 7, 0, 1}); g.Start(0);
        const unsigned cachedStop = fast ? 4 : 8;
        for (unsigned tick = 1; tick < cachedStop; ++tick) {
            g.chip.Advance(1);
            Check(g.Running(), "Cached program remains active until its final clock");
        }
        g.chip.Advance(1);
        Check(!g.Running() && g.Reg(0) == 7, "CPU-uploaded cache executes with no external memory ownership");
    }
    Fixture f;
    f.chip.WriteCpu(0x303a, 0); f.Program({0xa0, 0x73, 0, 1}); f.Start();
    f.chip.Advance(1000);
    Check(f.Running() && f.Reg(0) == 0, "Uncached code waits for ROM ownership without executing a stub");
    f.chip.WriteCpu(0x303a, 0x10); f.chip.Advance(100);
    Check(!f.Running() && f.Reg(0) == 0x73, "Granting ROM resumes the same suspended fetch");

    Fixture g;
    g.chip.WriteCpu(0x3034, 0x70);
    const std::array<uint8_t, 5> code{0x02, 0xa0, 7, 0, 1};
    std::copy(code.begin(), code.end(), g.ram.begin() + 0x8000);
    g.Run();
    Check(g.Reg(0) == 7 && g.chip.ReadCpu(0x303e) == 0 && g.chip.ReadCpu(0x303f) == 0x80,
          "CACHE aligns to the next instruction's address");
    g.ram[0x8002] = 9;
    g.Run();
    Check(g.Reg(0) == 7, "Valid instruction cache retains bytes after backing RAM changes");
    g.chip.WriteCpu(0x3030, 0x20); g.chip.WriteCpu(0x3030, 0);
    g.Run();
    Check(g.Reg(0) == 9, "CPU clearing GO invalidates cache and resets CBR");

    Fixture buffered;
    buffered.chip.WriteCpu(0x3039, 1); buffered.chip.WriteCpu(0x303a, 8);
    buffered.Reg(0, 0xaa); buffered.Reg(1, 0x1000);
    buffered.Cache({0x3d, 0x31, 0, 1}); buffered.Start(0); buffered.chip.Advance(4);
    Check(!buffered.Running() && buffered.ram[0x1000] == 0, "Buffered RAM write can outlive STOP");
    buffered.chip.Advance(3);
    Check(buffered.ram[0x1000] == 0, "Buffered RAM write is not visible one clock early");
    buffered.chip.Advance(1);
    Check(buffered.ram[0x1000] == 0xaa, "Pending RAM write completes at its physical bus latency");

    Fixture masked;
    masked.chip.WriteCpu(0x3037, 0x80); masked.Program({0, 1}); masked.Run();
    Check(!masked.chip.CpuIrqPending() && !(masked.chip.ReadCpu(0x3031) & 0x80), "CFGR masks STOP interrupt generation");
    Fixture bus;
    bus.Program({0x05, 0xfe, 1}); bus.Start();
    Check(bus.chip.ReadCpu(0x008004) == 4 && bus.chip.ReadCpu(0xc0000a) == 8,
          "CPU reads observe the GSU driven ROM vector pattern while ROM is owned");
    bus.ram[0x123] = 0x54;
    Check(bus.chip.ReadCpu(0x700123, 0x29) == 0x29, "CPU RAM read is open bus while GSU owns RAM");
    bus.chip.WriteCpu(0x700123, 0x99);
    Check(bus.ram[0x123] == 0x54, "CPU writes cannot corrupt RAM assigned to the GSU");
    bus.chip.WriteCpu(0x303a, 0x10);
    bus.chip.WriteCpu(0x700123, 0x68);
    Check(bus.chip.ReadCpu(0x700123) == 0x68, "RAM ownership can be handed back while GSU continues");

    Fixture one, pieces;
    const auto initialize = [](Fixture& chip) {
        chip.Program({0x02, 0xac, 40, 0xfd, 6, 0x80, 0xd0, 0x3c, 0xd1, 0x3e, 0xf0, 0x22, 0x11, 0, 1});
        chip.Start();
    };
    initialize(one); initialize(pieces);
    one.chip.Advance(4000);
    for (unsigned i = 0; i < 4000; ++i) pieces.chip.Advance(1);
    for (unsigned n = 0; n < 16; ++n) Check(one.Reg(n) == pieces.Reg(n), "Advance chunk size does not change GSU register results");
    Check(one.ram == pieces.ram && one.chip.CpuIrqPending() == pieces.chip.CpuIrqPending(),
          "Advance chunk size preserves RAM writes and interrupt publication");
    Check(one.Reg(0) == 40 && one.Reg(1) == 40 && one.ram[0x1122] == 40, "Cache loop executes forty iterations and stores its result");
}

void CacheBoundaryAndClockModes() {
    Fixture partial;
    partial.chip.WriteCpu(0x3100, 0xff);
    partial.Start(0);
    partial.chip.Advance(97);
    Check(partial.Running(), "An incomplete CPU cache line still requires a full sixteen-byte memory fill");
    partial.chip.Advance(1);
    Check(!partial.Running() && partial.chip.ReadCpu(0x3100) == 0,
          "Only writing the final cache-line byte validates a CPU-uploaded line");

    Fixture wrapped;
    wrapped.Program({0x02, 0xf0, 0x34, 0x12}, 0x7ffc);
    wrapped.Program({0, 1});
    wrapped.Run(0xfffc);
    Check(wrapped.Reg(0) == 0x1234 && wrapped.Reg(15) == 2,
          "Instruction operands and cache lines wrap across the sixteen-bit program counter boundary");
    Check(wrapped.chip.ReadCpu(0x303e) == 0xf0 && wrapped.chip.ReadCpu(0x303f) == 0xff &&
          wrapped.chip.ReadCpu(0x311c) == 0x02,
          "CPU cache addressing rotates by CBR even when the program cache straddles bank end");

    Fixture ramCode;
    ramCode.chip.WriteCpu(0x3034, 0x70);
    ramCode.chip.WriteCpu(0x303a, 8);
    ramCode.ram[0x8000] = 0xa0; ramCode.ram[0x8001] = 0x75;
    ramCode.Run();
    Check(ramCode.Reg(0) == 0x75, "GSU executes uncached RAM code without ROM ownership");

    Fixture blockedLoad;
    blockedLoad.chip.WriteCpu(0x303a, 0x10);
    blockedLoad.Reg(1, 0x2000);
    blockedLoad.ram[0x2000] = 0x34; blockedLoad.ram[0x2001] = 0x12;
    blockedLoad.Program({0x41, 0, 1}); blockedLoad.Start(); blockedLoad.chip.Advance(100);
    Check(blockedLoad.Running() && blockedLoad.Reg(0) == 0, "RAM load stalls until the CPU hands over RAM");
    blockedLoad.chip.WriteCpu(0x303a, 0x18); blockedLoad.chip.Advance(100);
    Check(!blockedLoad.Running() && blockedLoad.Reg(0) == 0x1234, "RAM load resumes with its original address and operands");

    for (unsigned fast = 0; fast < 2; ++fast) {
        for (unsigned multiplierFast = 0; multiplierFast < 2; ++multiplierFast) {
            for (bool wide : {false, true}) {
                Fixture f;
                f.chip.WriteCpu(0x3039, uint8_t(fast));
                f.chip.WriteCpu(0x3037, uint8_t(multiplierFast * 0x20));
                f.chip.WriteCpu(0x303a, 0);
                f.Reg(0, 5); f.Reg(1, 7); f.Reg(6, 7);
                f.Cache({uint8_t(wide ? 0x9f : 0x81), 0, 1}); f.Start(0);
                const unsigned extra = wide ? (multiplierFast ? 3 : 7) : (multiplierFast ? 0 : 1);
                const unsigned completion = (3 + extra) * (fast ? 1 : 2);
                f.chip.Advance(completion - 1);
                Check(f.Running(), "Multiplier latency respects CFGR MS0 and CLSR through the last clock");
                f.chip.Advance(1);
                Check(!f.Running() && f.Reg(0) == (wide ? 0 : 35), "Both multiplier sizes complete at their selected clock rates");
            }
        }
    }
}
}

int main() {
    try {
        RegistersAndMapping(); ExpandedCpuWindows(); ArithmeticMatrix(); TransfersAndSpecialOperations();
        BranchesAndPipeline(); RamInstructions(); RomBufferInstructions();
        PixelsAndScreenModes(); CacheAndSynchronization(); CacheBoundaryAndClockModes();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "Super FX: %s\n", error.what());
        return 1;
    }
    std::puts("Super FX instruction, pipeline, pixel, cache, mapping, IRQ and clock checks passed");
}
