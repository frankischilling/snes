#include "snes/core/Sa1.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <stdexcept>
#include <vector>

using snes::core::Sa1;

namespace {

void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void Set(Sa1& chip, uint32_t address, uint32_t value, unsigned bytes = 2) {
    for (unsigned i = 0; i < bytes; ++i) chip.WriteSa1(address + i, static_cast<uint8_t>(value >> (i * 8)));
}

void SetCpu(Sa1& chip, uint32_t address, uint32_t value, unsigned bytes = 2) {
    for (unsigned i = 0; i < bytes; ++i) chip.WriteCpu(address + i, static_cast<uint8_t>(value >> (i * 8)));
}

uint64_t Get(Sa1& chip, uint32_t address, unsigned bytes = 2) {
    uint64_t value = 0;
    for (unsigned i = 0; i < bytes; ++i) value |= uint64_t(chip.ReadSa1(address + i)) << (i * 8);
    return value;
}

struct Fixture {
    std::vector<uint8_t> rom;
    std::vector<uint8_t> ram;
    Sa1 chip;

    explicit Fixture(size_t romSize = 0x400000, size_t ramSize = 0x40000)
        : rom(romSize, 0xea), ram(ramSize), chip(rom, ram) {}

    void Code(uint16_t address, std::initializer_list<uint8_t> bytes) {
        std::copy(bytes.begin(), bytes.end(), rom.begin() + address - 0x8000);
    }

    void Start(uint16_t address = 0x8000) {
        SetCpu(chip, 0x2203, address);
        chip.WriteCpu(0x2200, 0);
    }
};

void RegistersAndProtection() {
    Fixture f;
    auto& c = f.chip;
    Check(c.ReadCpu(0x230e) == 0x23, "Version code is available to the console CPU");
    Check(c.ReadSa1(0x230e, 0x65) == 0x65, "SA-1 cannot read the console-only version port");
    Check(c.ReadCpu(0x2200, 0x6d) == 0x6d && c.ReadSa1(0x2220, 0x5b) == 0x5b, "Write-only registers return open bus");
    c.WriteCpu(0x2209, 0x80);
    Check(c.ReadCpu(0x2300) == 0, "Console writes cannot assert their own cartridge IRQ");
    c.WriteSa1(0x2200, 0x80);
    Check(c.ReadSa1(0x2301) == 0, "SA-1 cannot write console control registers");
    c.WriteCpu(0x2200, 0x2b);
    c.WriteSa1(0x2209, 0x5a);
    Check(c.ReadSa1(0x2301) == 0x0b && c.ReadCpu(0x2300) == 0x5a, "Both four-bit mailboxes and vector control bits are visible");
    c.WriteSa1(0x2209, 0x80);
    Check(!c.CpuIrqPending() && (c.ReadCpu(0x2300) & 0x80), "A disabled console IRQ still latches its status flag");
    c.WriteCpu(0x2201, 0x80);
    Check(c.CpuIrqPending(), "Enabling an already latched console IRQ asserts the line");
    c.WriteCpu(0x2201, 0);
    Check(!c.CpuIrqPending() && (c.ReadCpu(0x2300) & 0x80), "Masking an IRQ preserves its status flag");
    c.WriteCpu(0x2202, 0x80); c.WriteCpu(0x2201, 0x80);
    Check(!c.CpuIrqPending(), "Acknowledging a masked IRQ prevents it from reappearing on enable");

    c.WriteCpu(0x3000, 0x11);
    c.WriteSa1(0, 0x22);
    Check(c.ReadCpu(0x3000) == 0, "IRAM powers up protected on both ports");
    c.WriteCpu(0x2229, 1);
    c.WriteCpu(0x8030ff, 0x31);
    c.WriteCpu(0x3100, 0x42);
    Check(c.ReadSa1(0x00ff) == 0x31 && c.ReadSa1(0x0100) == 0, "Console IRAM mask gates each 256-byte page");
    c.WriteSa1(0x222a, 2);
    c.WriteSa1(0x80100, 0x53);
    c.WriteSa1(0xff, 0x64);
    Check(c.ReadCpu(0x803100) == 0x53 && c.ReadCpu(0x30ff) == 0x31, "SA-1 IRAM mask is independent of the console mask");
    Check(c.ReadCpu(0, 0x9c) == 0x9c && c.ReadSa1(0x0800, 0x8d) == 0x8d &&
          c.ReadSa1(0x7e0000, 0x7e) == 0x7e, "CPU WRAM and IRAM overrun addresses remain unmapped on the SA-1 bus");

    c.WriteCpu(0x400000, 0x55);
    c.WriteSa1(0x400000, 0x66);
    Check(f.ram[0] == 0, "BW-RAM powers up protected");
    c.WriteCpu(0x2228, 0);
    c.WriteCpu(0x4000ff, 0x12);
    c.WriteCpu(0x400100, 0x34);
    Check(f.ram[0xff] == 0 && f.ram[0x100] == 0x34, "BW-RAM protected prefix has an exact 256-byte minimum");
    c.WriteCpu(0x2226, 0x80);
    c.WriteCpu(0x400000, 0x56);
    c.WriteSa1(0x400000, 0x78);
    Check(f.ram[0] == 0x78, "Either BW-RAM enable opens the shared write gate for both processors");
    c.WriteCpu(0x2226, 0);
    c.WriteSa1(0x2227, 0x80);
    c.WriteCpu(0x400000, 0x78);
    Check(f.ram[0] == 0x78 && c.ReadCpu(0x4c0000) == 0x78, "BW-RAM write enable and full-bank mirrors share cartridge save storage");
    c.Reset();
    Check(f.ram[0] == 0x78 && c.ReadCpu(0x3000) == 0, "Device reset preserves battery RAM");
}

void Mapping() {
    Fixture f(0x800000);
    auto& c = f.chip;
    for (size_t i = 0; i < f.rom.size(); ++i) f.rom[i] = static_cast<uint8_t>((i >> 20) * 17 + ((i >> 15) & 31));
    const std::array<uint8_t, 4> lowBanks{0x00, 0x20, 0x80, 0xa0};
    for (unsigned block = 0; block < 4; ++block) {
        const uint32_t low = (uint32_t(lowBanks[block]) << 16) | 0x8123;
        const uint32_t high = 0xc00123 + block * 0x100000;
        Check(c.ReadCpu(low) == block * 17 && c.ReadSa1(high) == block * 17, "Four default LoROM and HiROM windows");
        for (unsigned page = 0; page < 8; ++page) {
            c.WriteCpu(0x2220 + block, static_cast<uint8_t>(page));
            Check(c.ReadCpu(low) == block * 17 && c.ReadSa1(high) == page * 17, "HiROM-only bank change preserves the fixed LoROM window");
            c.WriteCpu(0x2220 + block, static_cast<uint8_t>(0x80 | page));
            Check(c.ReadCpu(low) == page * 17 && c.ReadSa1(low) == page * 17, "LoROM bank override is shared by both processors");
            Check(c.ReadCpu(high + 0xffff) == f.rom[page * 0x100000 + 0x10122], "Mapped HiROM offset survives a bank boundary");
        }
    }
    c.WriteSa1(0x2220, 0);
    Check(c.ReadCpu(0xc00000) == 119, "SA-1 cannot change console-owned MMC registers");
    c.WriteCpu(0x2226, 0x80);
    c.WriteSa1(0x2227, 0x80);
    c.WriteCpu(0x2224, 3);
    c.WriteSa1(0x2225, 5);
    c.WriteCpu(0x006123, 0x39);
    c.WriteSa1(0x006123, 0x57);
    Check(f.ram[0x6123] == 0x39 && f.ram[0xa123] == 0x57, "Independent raw BW-RAM windows");

    for (uint8_t depth : {2, 4}) {
        c.WriteSa1(0x223f, depth == 2 ? 0x80 : 0);
        const unsigned perByte = 8 / depth;
        const unsigned mask = (1u << depth) - 1;
        std::fill(f.ram.begin(), f.ram.end(), 0xa5);
        c.WriteSa1(0x600001, 0xff);
        const uint8_t expected = static_cast<uint8_t>((0xa5 & ~(mask << depth)) | (mask << depth));
        Check(f.ram[0] == expected, "Bitmap writes mask one pixel without disturbing neighbors");
        Check(c.ReadSa1(0x600001) == mask && c.ReadSa1(0x600000) == (0xa5 & mask), "Bitmap reads zero-extend individual pixels");
        c.WriteSa1(0x2225, 0x83);
        c.WriteSa1(0x006123, 9);
        const unsigned byte = 3 * 0x800 + 0x123 / perByte;
        Check(((f.ram[byte] >> ((0x123 % perByte) * depth)) & mask) == (9 & mask), "Bitmap window selects fixed two-kilobyte physical pages");
        const uint32_t alias = 0x600000 + 3 * 0x800 * perByte + 0x123;
        Check(c.ReadSa1(alias) == (9 & mask), "Bitmap window aliases the matching full pixel address");
        c.WriteSa1(0x2227, 0);
        c.WriteCpu(0x2226, 0);
        c.WriteCpu(0x2228, 15);
        const auto old = f.ram[0];
        c.WriteSa1(0x400000, 0);
        Check(f.ram[0] == old, "Linear BW-RAM writes are protected when both enables are clear");
        c.WriteSa1(0x600000, 0);
        Check(f.ram[0] == (old & ~mask), "Bitmap pixel writes use their own port and bypass linear protection");
        c.WriteSa1(0x2227, 0x80);
    }

    Fixture odd(0x300000, 0x2000);
    std::fill_n(odd.rom.begin(), 0x100000, 0x11);
    std::fill_n(odd.rom.begin() + 0x100000, 0x100000, 0x22);
    std::fill_n(odd.rom.begin() + 0x200000, 0x100000, 0x33);
    Check(odd.chip.ReadCpu(0xf00000) == 0x33, "Three-megabyte ROM mirrors its last address-line branch");
    odd.chip.WriteCpu(0x2226, 0x80);
    odd.chip.WriteCpu(0x43ffff, 0x8a);
    Check(odd.ram[0x1fff] == 0x8a, "Small physical BW-RAM mirrors safely");
    Sa1 empty({}, {});
    Check(empty.ReadCpu(0xc00000, 0x91) == 0x91 && empty.ReadSa1(0x600000, 0x82) == 0x82, "Absent memory stays open bus");
}

void Arithmetic() {
    Fixture f;
    auto& c = f.chip;
    constexpr uint64_t mask = (uint64_t{1} << 40) - 1;
    const std::array<uint16_t, 8> values{0, 1, 2, 0x100, 0x7fff, 0x8000, 0xfffe, 0xffff};
    for (unsigned mode : {0u, 1u}) {
        c.WriteSa1(0x2250, static_cast<uint8_t>(mode));
        for (uint16_t a : values) for (uint16_t b : values) {
            Set(c, 0x2251, a);
            Set(c, 0x2253, b);
            c.Advance(10);
            const int64_t signedA = a < 0x8000 ? a : int64_t(a) - 65536;
            const int64_t signedB = b < 0x8000 ? b : int64_t(b) - 65536;
            uint64_t expected = 0;
            if (!mode) expected = static_cast<uint64_t>(signedA * signedB) & mask;
            else if (b) {
                const int64_t quotient = signedA / b;
                int64_t remainder = signedA % b;
                if (remainder < 0) remainder = -remainder;
                expected = uint16_t(quotient) | (uint64_t(remainder) << 16);
            } else {
                expected = (signedA < 0 ? 1u : 0xffffu) |
                           (uint64_t(signedA < 0 ? -signedA : signedA) << 16);
            }
            Check(Get(c, 0x2306, mode ? 4 : 5) == expected, "Arithmetic signed edge grid, including hardware division and divide by zero");
        }
    }
    struct DivisionCase { uint16_t dividend, divisor, quotient, remainder; };
    for (const auto test : {
        DivisionCase{0x87f8, 0xfffb, 0x0000, 0x7808},
        DivisionCase{0x8000, 0x492a, 0xffff, 0x36d6},
        DivisionCase{0x87f8, 0x0000, 0x0001, 0x7808},
        DivisionCase{0x8000, 0x0000, 0x0001, 0x8000}}) {
        c.WriteSa1(0x2250, 1);
        Set(c, 0x2251, test.dividend); Set(c, 0x2253, test.divisor); c.Advance(10);
        Check(Get(c, 0x2306, 4) == (uint64_t(test.remainder) << 16 | test.quotient),
              "Division matches measured signed quotient and unsigned remainder cases");
    }
    c.WriteSa1(0x2250, 0);
    Set(c, 0x2251, 3); Set(c, 0x2253, 7); c.Advance(10);
    Set(c, 0x2253, 11); c.Advance(9);
    Check(Get(c, 0x2306, 5) == 21, "Arithmetic result remains latched during its five-cycle execution");
    c.Advance(1);
    Check(Get(c, 0x2306, 5) == 33, "Multiplicand survives multiplication and result appears on cycle five");
    c.WriteSa1(0x2254, 0); c.Advance(10);
    Check(Get(c, 0x2306, 5) == 0, "Multiplier is consumed by the preceding operation");
    c.WriteSa1(0x2250, 2);
    Set(c, 0x2251, 0x8000);
    for (unsigned i = 0; i < 512; ++i) {
        Set(c, 0x2253, 0x8000); c.Advance(12);
        Check(c.ReadSa1(0x230b) == (i == 511 ? 0x80 : 0), "Accumulator overflow occurs at the signed 40-bit boundary");
    }
    Check(Get(c, 0x2306, 5) == (uint64_t{1} << 39), "Accumulator retains all forty bits after wrapping");
    c.WriteSa1(0x2250, 2);
    Set(c, 0x2251, 0xffff); Set(c, 0x2253, 1); c.Advance(11);
    Check(Get(c, 0x2306, 5) == 0, "Accumulate has a six-cycle execution latency");
    c.Advance(1);
    Check(Get(c, 0x2306, 5) == mask && c.ReadSa1(0x230b) == 0, "A negative sum sign-extends without a spurious unsigned-underflow flag");
}

void VariableBits() {
    Fixture f;
    auto& c = f.chip;
    constexpr unsigned source = 0xfffc;
    for (unsigned i = 0; i < 160; ++i) f.rom[source + i] = static_cast<uint8_t>(i * 59 + 0x93);
    const auto expected = [&](unsigned bit) {
        uint16_t result = 0;
        for (unsigned i = 0; i < 16; ++i) {
            const unsigned at = bit + i;
            result |= ((f.rom[source + at / 8] >> (at % 8)) & 1) << i;
        }
        return result;
    };
    c.WriteSa1(0x2258, 0x85);
    Set(c, 0x2259, 0xc0fffd, 3);
    Check(Get(c, 0x230c) == expected(0) && Get(c, 0x230c) == expected(0),
          "Bit reader ignores the address low bit and address writes clear automatic mode");
    for (unsigned length = 1; length <= 16; ++length) {
        Set(c, 0x2259, 0xc0fffd, 3);
        for (unsigned bit = 0; bit < 160; bit += length) {
            Check(Get(c, 0x230c) == expected(bit), "Manual variable-bit reads cross both word and ROM-bank boundaries");
            Check(Get(c, 0x230c) == expected(bit), "Manual port reads do not consume bits");
            c.WriteSa1(0x2258, static_cast<uint8_t>(length & 15));
        }
        Set(c, 0x2259, 0xc0fffd, 3);
        c.WriteSa1(0x2258, static_cast<uint8_t>(0x80 | (length & 15)));
        for (unsigned bit = length; bit < 160; bit += length) {
            Check(c.ReadSa1(0x230c) == uint8_t(expected(bit)) && c.ReadSa1(0x230c) == uint8_t(expected(bit)), "Low data reads do not advance the automatic stream");
            Check(Get(c, 0x230c) == expected(bit), "Automatic stream advances after returning the high byte");
        }
    }

    Fixture mapped(0x800000);
    mapped.rom[0x0122] = 0x34; mapped.rom[0x0123] = 0x12;
    mapped.rom[0x7122] = 0xef; mapped.rom[0x7123] = 0xbe;
    mapped.rom[0x8122] = 0x78; mapped.rom[0x8123] = 0x56;
    mapped.rom[0x400122] = 0xbc; mapped.rom[0x400123] = 0x9a;
    Set(mapped.chip, 0x2259, 0x000123, 3);
    Check(Get(mapped.chip, 0x230c) == 0x1234, "Bit reader mirrors low system windows to their LoROM half");
    Set(mapped.chip, 0x2259, 0x010123, 3);
    Check(Get(mapped.chip, 0x230c) == 0x1234, "Bit reader mirrors non-ROM blocks in other LoROM banks to bank zero");
    Set(mapped.chip, 0x2259, 0x3f7123, 3);
    Check(Get(mapped.chip, 0x230c) == 0xbeef, "Bit reader maps every non-ROM 32K block to bank zero while preserving its block offset");
    Set(mapped.chip, 0x2259, 0x418123, 3);
    Check(Get(mapped.chip, 0x230c) == 0x1234, "Bit reader mirrors banks 40-7F to bank-zero LoROM");
    mapped.chip.WriteCpu(0x2220, 0x84);
    Set(mapped.chip, 0x2259, 0x000123, 3);
    Check(Get(mapped.chip, 0x230c) == 0x9abc, "Bit reader follows Super MMC projection for LoROM windows");
    Set(mapped.chip, 0x2259, 0xc00123, 3);
    Check(Get(mapped.chip, 0x230c) == 0x9abc, "Bit reader follows Super MMC projection for HiROM banks");
}

void Timers() {
    Fixture f;
    auto& c = f.chip;
    Set(c, 0x2212, 3);
    c.WriteSa1(0x2210, 1);
    c.Advance(11);
    Check(!(c.ReadSa1(0x2301) & 0x40), "H timer does not fire before its compare dot");
    c.Advance(1);
    Check((c.ReadSa1(0x2301) & 0x40) != 0, "H timer fires at four master clocks per dot even while CPU reset is held");
    c.WriteSa1(0x220b, 0x40);
    c.Advance(1);
    Check(!(c.ReadSa1(0x2301) & 0x40), "Acknowledgment within a compare dot does not immediately reassert");
    c.Advance(1364);
    Check((c.ReadSa1(0x2301) & 0x40) != 0, "H compare repeats on the next scanline");
    c.WriteSa1(0x2211, 0);
    c.WriteSa1(0x220b, 0x40);
    Set(c, 0x2214, 1);
    c.WriteSa1(0x2210, 2);
    c.Advance(1363);
    Check(!(c.ReadSa1(0x2301) & 0x40), "V-only compare waits for the selected scanline");
    c.Advance(1);
    Check((c.ReadSa1(0x2301) & 0x40) != 0, "V-only compare fires at the start of the scanline");
    c.WriteSa1(0x220b, 0x40);
    c.Advance(100);
    Check(!(c.ReadSa1(0x2301) & 0x40), "V-only IRQ stays acknowledged for the rest of the matching line");
    c.WriteSa1(0x2211, 0); c.WriteSa1(0x2210, 3); Set(c, 0x2212, 2);
    c.Advance(1371);
    Check(!(c.ReadSa1(0x2301) & 0x40), "Combined H/V compare requires both coordinates");
    c.Advance(1);
    Check((c.ReadSa1(0x2301) & 0x40) != 0, "Combined H/V compare fires at the selected dot");
    c.WriteSa1(0x2210, 0x80); c.WriteSa1(0x2211, 0);
    c.Advance(511 * 4);
    Check(c.ReadSa1(0x2302) == 0xff && c.ReadSa1(0x2303) == 1 && Get(c, 0x2304) == 0, "Linear counter exposes all nine horizontal bits");
    c.Advance(4);
    Check(Get(c, 0x2304) == 0 && c.ReadSa1(0x2303) == 1, "Counter high bytes remain latched across a rollover");
    Check(c.ReadSa1(0x2302) == 0 && Get(c, 0x2304) == 1, "Reading H low atomically refreshes the H/V latch");
    c.Advance(511 * 2048);
    Check(c.ReadSa1(0x2302) == 0 && Get(c, 0x2304) == 0, "Linear counter wraps its nine-bit vertical coordinate");
    c.SetPal(true); c.WriteSa1(0x2210, 0); c.WriteSa1(0x2211, 0);
    c.Advance(311 * 1364);
    c.ReadSa1(0x2302);
    Check(Get(c, 0x2304) == 311, "PAL timer uses 312 lines");
    c.Advance(1364); c.ReadSa1(0x2302);
    Check(Get(c, 0x2304) == 0, "PAL timer frame wraps");
    Set(c, 0x2212, 511); c.WriteSa1(0x2210, 1); c.WriteSa1(0x220b, 0x40);
    c.Advance(312 * 1364);
    Check(!(c.ReadSa1(0x2301) & 0x40), "An out-of-range HV compare never fires");
}

void ConfigureDma(Sa1& c, uint8_t control, uint32_t source, uint32_t destination, uint16_t length) {
    c.WriteSa1(0x2230, control);
    Set(c, 0x2232, source, 3);
    Set(c, 0x2238, length);
    Set(c, 0x2235, destination, (control & 4) ? 3 : 2);
}

void Dma() {
    Fixture f;
    auto& c = f.chip;
    for (unsigned i = 0; i < 6; ++i) f.rom[0x100 + i] = static_cast<uint8_t>(0x51 + i);
    ConfigureDma(c, 0x80, 0xc00100, 0x7fd, 6);
    c.Advance(1);
    Check(c.DmaActive() && c.ReadSa1(0x7fd) == 0, "DMA waits for a complete byte cycle");
    c.Advance(1);
    Check(c.ReadSa1(0x7fd) == 0x51 && !(c.ReadSa1(0x2301) & 0x20), "DMA progressively transfers bytes before its completion IRQ");
    c.Advance(9);
    Check(c.DmaActive(), "Six ROM-to-IRAM bytes require twelve master clocks");
    c.Advance(1);
    for (unsigned i = 0; i < 6; ++i) Check(c.ReadSa1((0x7fd + i) & 0x7ff) == 0x51 + i, "DMA uses all 2KB of IRAM and wraps its destination");
    Check(!c.DmaActive() && (c.ReadSa1(0x2301) & 0x20) && !c.CpuIrqPending(), "Normal DMA raises the SA-1 completion flag only");

    c.WriteSa1(0x220b, 0x20);
    f.rom[0xffffe] = 0x71; f.rom[0xfffff] = 0x82;
    f.rom[0x300000] = 0x93; f.rom[0x300001] = 0xa4;
    c.WriteCpu(0x2221, 3);
    ConfigureDma(c, 0x80, 0xcffffe, 0x500, 4); c.Advance(8);
    Check(Get(c, 0x500, 4) == 0xa4938271, "DMA redecodes ROM mapping across a one-megabyte bank boundary");

    c.WriteSa1(0x222a, 0xff);
    Set(c, 0x7fe, 0x1234);
    Set(c, 0, 0x5678);
    ConfigureDma(c, 0x86, 0x7fe, 0x3ffff, 4); c.Advance(15);
    Check(c.DmaActive(), "BW-RAM transfers take four clocks per byte");
    c.Advance(1);
    Check(f.ram.back() == 0x34 && f.ram[0] == 0x12 && f.ram[1] == 0x78 && f.ram[2] == 0x56, "IRAM-to-BW-RAM DMA wraps both independent physical ports");
    ConfigureDma(c, 0x81, 0x3ffff, 0x700, 4); c.Advance(16);
    Check(Get(c, 0x700, 4) == 0x56781234, "BW-RAM-to-IRAM DMA reads shared save storage");

    c.WriteSa1(0x220b, 0x20);
    ConfigureDma(c, 0x80, 0xc00100, 0x600, 4); c.Advance(1);
    c.WriteSa1(0x2230, 0); c.Advance(20);
    Check(!c.DmaActive() && c.ReadSa1(0x600) == 0 && !(c.ReadSa1(0x2301) & 0x20), "Disabling DMA cancels an unfinished byte without a completion interrupt");
    ConfigureDma(c, 0x82, 0, 0, 4);
    Check(!c.DmaActive(), "Unsupported same-device transfer does not start");
    ConfigureDma(c, 0x80, 0xc00100, 0x600, 0);
    Check(!c.DmaActive() && c.ReadSa1(0x600) == 0 && (c.ReadSa1(0x2301) & 0x20), "Zero-length DMA completes without copying or underflowing the byte count");
}

std::vector<uint8_t> Planar(const std::array<uint8_t, 64>& pixels, unsigned depth) {
    std::vector<uint8_t> output(depth * 8);
    // Independent pixel-scatter oracle: each source pixel contributes one bit
    // to each output plane. The device itself gathers complete plane rows.
    for (unsigned pixel = 0; pixel < 64; ++pixel) for (unsigned plane = 0; plane < depth; ++plane) {
        if (pixels[pixel] & (1u << plane))
            output[(plane & ~1u) * 8 + (pixel / 8) * 2 + (plane & 1)] |= static_cast<uint8_t>(0x80 >> (pixel & 7));
    }
    return output;
}

void CharacterConversion() {
    for (unsigned mode = 0; mode < 3; ++mode) {
        const unsigned depth = 8 >> mode;
        Fixture f;
        auto& c = f.chip;
        c.WriteSa1(0x2230, 0xa0);
        c.WriteSa1(0x2231, static_cast<uint8_t>(mode));
        Set(c, 0x2235, 0x780);
        for (unsigned tile = 0; tile < 3; ++tile) {
            std::array<uint8_t, 64> pixels{};
            for (unsigned i = 0; i < pixels.size(); ++i) {
                pixels[i] = static_cast<uint8_t>(i * 29 + tile * 47 + 13);
                c.WriteSa1(0x2240 + (i & 15), pixels[i]);
                if ((i & 7) == 7) {
                    const auto partial = Planar(pixels, depth);
                    const unsigned row = i / 8;
                    for (unsigned plane = 0; plane < depth; ++plane) {
                        const unsigned at = (plane / 2) * 16 + row * 2 + (plane & 1);
                        Check(c.ReadCpu(0x3780 + (tile & 1) * depth * 8 + at) == partial[at], "Type-2 conversion exposes each completed row before the tile is finished");
                    }
                }
            }
            const auto expected = Planar(pixels, depth);
            for (unsigned i = 0; i < expected.size(); ++i)
                Check(c.ReadCpu(0x3000 + ((0x780 + (tile & 1) * expected.size() + i) & 0x7ff)) == expected[i], "Type-2 conversion matches independent planar pixels and alternates IRAM buffers");
        }
        c.WriteSa1(0x2230, 0);
        const auto previous = c.ReadCpu(0x3780);
        for (unsigned i = 0; i < 64; ++i) c.WriteSa1(0x2240 + (i & 15), 0);
        Check(c.ReadCpu(0x3780) == previous, "Disabled type-2 conversion leaves the output buffer intact");

        for (unsigned width : {1u, 2u, 8u}) {
            unsigned widthCode = 0;
            while ((1u << widthCode) != width) ++widthCode;
            c.WriteSa1(0x2230, 0xb0);
            c.WriteCpu(0x2231, static_cast<uint8_t>(mode | (widthCode << 2)));
            const unsigned source = 0x1000;
            std::vector<std::array<uint8_t, 64>> tiles(width * 2);
            for (unsigned tile = 0; tile < tiles.size(); ++tile) {
                for (unsigned i = 0; i < 64; ++i) {
                    const uint8_t pixel = static_cast<uint8_t>((tile * 23 + i * 37 + 5) & ((1u << depth) - 1));
                    tiles[tile][i] = pixel;
                    const unsigned x = (tile % width) * 8 + (i & 7);
                    const unsigned y = (tile / width) * 8 + i / 8;
                    const unsigned bit = (y * width * 8 + x) * depth;
                    const unsigned at = source + bit / 8;
                    const unsigned mask = ((1u << depth) - 1) << (bit & 7);
                    f.ram[at] = static_cast<uint8_t>((f.ram[at] & ~mask) | (pixel << (bit & 7)));
                }
            }
            c.WriteCpu(0x2201, 0x20);
            SetCpu(c, 0x2232, source, 3);
            SetCpu(c, 0x2235, 0x780);
            Check(c.CpuIrqPending() && (c.ReadCpu(0x2300) & 0x20), "Type-1 first-character notification reaches the console IRQ");
            c.WriteCpu(0x2202, 0x20);
            for (unsigned tile = 0; tile < tiles.size(); ++tile) {
                const auto expected = Planar(tiles[tile], depth);
                for (unsigned i = 0; i < expected.size(); ++i) {
                    const unsigned address = source + tile * expected.size() + i;
                    Check(c.ReadCpu(0x400000 + address) == expected[i], "Type-1 packed bitmap conversion respects depth, tile-row stride, and source base");
                    Check(c.ReadSa1(0x400000 + address) == f.ram[address], "SA-1 retains raw BW-RAM access while the console sees converted tiles");
                }
            }
            c.WriteCpu(0x2224, 0);
            Check(c.ReadCpu(0x7000) == Planar(tiles[0], depth)[0], "Type-1 conversion is also visible through the console's 8KB BW-RAM window");
            c.WriteCpu(0x2231, 0x80);
            Check(c.ReadCpu(0x401000) == f.ram[0x1000], "Ending type-1 conversion restores raw console BW-RAM reads");
        }
    }
}

void ExecutableProgram() {
    Fixture f;
    auto& c = f.chip;
    f.Code(0x8000, {
        0xa9,0xff, 0x8d,0x2a,0x22, // Enable SA-1 IRAM writes, including stack.
        0xa9,0x80, 0x8d,0x27,0x22, // Enable shared BW-RAM writes.
        0x18,0xfb, 0xc2,0x30,      // Enter native mode, 16-bit A/X/Y.
        0xa2,0x04,0x00, 0xa9,0x00,0x00,
        0x18, 0x69,0x07,0x00, 0xca, 0xd0,0xfa,
        0x8d,0x10,0x00, 0x8d,0x00,0x60,
        0x20,0x40,0x80,            // JSR verifies stack ownership and 16-bit PHA/PLA.
        0xe2,0x20, 0xa9,0x8d, 0x8d,0x09,0x22, 0xdb
    });
    f.Code(0x8040, {0x48,0xa9,0xef,0xbe,0x8d,0x02,0x60,0x68,0x60});
    c.WriteCpu(0x2201, 0x80);
    c.Advance(1000);
    Check(!c.CpuRegisters().stp && c.CpuRegisters().pc == 0 && f.ram[0] == 0, "Power-on reset holds off instruction execution");
    f.Start(); c.Advance(4096);
    Check(c.CpuRegisters().stp && !c.CpuRegisters().e && c.CpuRegisters().s == 0x1ff, "Native machine-code program runs to STP and balances its stack");
    Check(Get(c, 0x10) == 28 && f.ram[0] == 28 && f.ram[1] == 0 && f.ram[2] == 0xef && f.ram[3] == 0xbe,
          "Machine-code loop, subroutine, 16-bit arithmetic, IRAM, and BW-RAM produce expected results");
    Check(c.CpuIrqPending() && (c.ReadCpu(0x2300) & 15) == 13, "SA-1 program sends its completion IRQ and four-bit message");
    c.WriteCpu(0x2202, 0x80);
    Check(!c.CpuIrqPending(), "Console acknowledges program completion");
    const auto stoppedPc = c.CpuRegisters().pc;
    c.WriteCpu(0x2200, 0x90); c.Advance(200);
    Check(c.CpuRegisters().stp && c.CpuRegisters().pc == stoppedPc, "Neither IRQ nor NMI can release STP");

    c.WriteCpu(0x2229, 0xff);
    const std::array<uint8_t, 7> iramProgram{0xa9,0x6c,0x8f,0x00,0x01,0x40,0xdb};
    for (unsigned i = 0; i < iramProgram.size(); ++i) c.WriteCpu(0x3300 + i, iramProgram[i]);
    c.WriteCpu(0x2200, 0x20); SetCpu(c, 0x2203, 0x300); c.WriteCpu(0x2200, 0);
    c.Advance(100);
    Check(c.CpuRegisters().stp && c.CpuRegisters().e && f.ram[0x100] == 0x6c, "Reset releases STP, resets CPU mode, and executes a program from low IRAM");
}

void Interrupts() {
    Fixture f;
    auto& c = f.chip;
    f.Code(0x8000, {0xa9,0xff,0x8d,0x2a,0x22,0xa9,0xf0,0x8d,0x0a,0x22,0x58,0xcb,0xe6,0x10,0xdb});
    f.Code(0x8100, {0xe6,0x20,0xa9,0xe0,0x8d,0x0b,0x22,0x40});
    f.Code(0x8200, {0xe6,0x21,0xa9,0x10,0x8d,0x0b,0x22,0x40});
    SetCpu(c, 0x2205, 0x8200); SetCpu(c, 0x2207, 0x8100);
    f.Start(); c.Advance(100);
    Check(c.CpuRegisters().wai && c.ReadSa1(0x10) == 0, "WAI suspends the instruction following it");
    c.WriteCpu(0x2200, 0x90); c.Advance(200);
    Check(c.ReadSa1(0x20) == 1 && c.ReadSa1(0x21) == 1 && c.ReadSa1(0x10) == 1 && c.CpuRegisters().stp,
          "NMI and IRQ use independent programmable vectors and return through the protected IRAM stack");

    Fixture masked;
    masked.Code(0x8000, {0xcb,0xa9,0x73,0xdb});
    masked.chip.WriteSa1(0x220a, 0x80); masked.Start(); masked.chip.Advance(20);
    masked.chip.WriteCpu(0x2200, 0x80); masked.chip.Advance(20);
    Check(masked.chip.CpuRegisters().stp && masked.chip.CpuRegisters().a == 0x73,
          "Enabled IRQ wakes WAI with I set without taking the IRQ vector");

    Fixture latency;
    auto& l = latency.chip;
    latency.Code(0x8000, {0x58,0x78,0xea,0xea});
    latency.Code(0x8100, {0xdb});
    l.WriteSa1(0x222a, 0xff); l.WriteSa1(0x220a, 0x80);
    SetCpu(l, 0x2207, 0x8100); latency.Start();
    l.Advance(4); // CLI samples the previous set-I state.
    Check(l.CpuRegisters().pc == 0x8001, "CLI completes before an IRQ can be taken");
    l.WriteCpu(0x2200, 0x80); l.Advance(4); // SEI samples clear I before setting it.
    Check(l.CpuRegisters().pc == 0x8002 && (l.CpuRegisters().p & 4), "SEI samples IRQ before its final-cycle I update");
    l.Advance(1);
    Check(l.CpuRegisters().pc == 0x8100 && !l.CpuRegisters().stp, "A latched IRQ is taken even after SEI sets I");

    Fixture cli;
    auto& q = cli.chip;
    cli.Code(0x8000, {0x58,0xea,0xdb});
    q.WriteSa1(0x222a, 0xff); q.WriteSa1(0x220a, 0x80); SetCpu(q, 0x2207, 0x8100);
    cli.Start(); q.WriteCpu(0x2200, 0x80); q.Advance(4);
    Check(q.CpuRegisters().pc == 0x8001, "Already asserted IRQ is delayed through CLI");
    q.Advance(4);
    Check(q.CpuRegisters().pc == 0x8002, "One instruction after CLI executes before the pending IRQ");
    q.Advance(1);
    Check(q.CpuRegisters().pc == 0x8100, "CLI-delayed IRQ then enters its programmed vector");

    Fixture plp;
    auto& p = plp.chip;
    plp.Code(0x8000, {0x58,0xa9,0x34,0x48,0x28,0xea});
    p.WriteSa1(0x222a, 0xff); p.WriteSa1(0x220a, 0x80); SetCpu(p, 0x2207, 0x8100);
    plp.Start(); p.Advance(14); // CLI + LDA + PHA.
    p.WriteCpu(0x2200, 0x80); p.Advance(8); // PLP samples I before pulling the set-I status.
    Check(p.CpuRegisters().pc == 0x8005 && (p.CpuRegisters().p & 4), "PLP restores I after its final-cycle IRQ sample");
    p.Advance(1);
    Check(p.CpuRegisters().pc == 0x8100, "An IRQ sampled by PLP survives the pulled I flag");

    Set(p, 0x220c, 0xabcd); Set(p, 0x220e, 0x9876); p.WriteSa1(0x2209, 0x50);
    for (uint32_t vector : {0xffeau, 0xfffau})
        Check(p.ReadCpu(vector) == 0xcd && p.ReadCpu(vector + 1) == 0xab, "Console NMI vectors are substituted in both CPU modes");
    for (uint32_t vector : {0xffeeu, 0xfffeu})
        Check(p.ReadCpu(vector) == 0x76 && p.ReadCpu(vector + 1) == 0x98, "Console IRQ vectors are substituted in both CPU modes");
    Check(p.ReadCpu(0xfffc) == 0xea && p.ReadCpu(0xffe4) == 0xea, "Vector control leaves reset and COP vectors intact");
}

void Synchronization() {
    Fixture whole, pieces;
    for (auto* f : {&whole, &pieces}) {
        f->Code(0x8000, {0xa9,0xff,0x8d,0x2a,0x22,0xe6,0x20,0x80,0xfc});
        f->Start();
    }
    whole.chip.Advance(10003);
    unsigned remaining = 10003;
    unsigned step = 1;
    while (remaining) {
        const unsigned clocks = std::min(remaining, step);
        pieces.chip.Advance(clocks);
        remaining -= clocks;
        step = (step * 7) % 29 + 1;
    }
    Check(whole.chip.ReadSa1(0x20) == pieces.chip.ReadSa1(0x20) &&
          whole.chip.CpuRegisters().pc == pieces.chip.CpuRegisters().pc &&
          whole.chip.CpuRegisters().p == pieces.chip.CpuRegisters().p,
          "Instruction clock debt makes fragmented and batched scheduling equivalent");
    Check(whole.chip.MasterClocks() == 10003 && pieces.chip.MasterClocks() == 10003,
          "Master clock accounting retains the exact scheduler budget");
    const uint16_t pc = pieces.chip.CpuRegisters().pc;
    pieces.chip.WriteCpu(0x2200, 0x40); pieces.chip.Advance(1000);
    Check(pieces.chip.CpuRegisters().pc == pc, "Console WAIT freezes instruction execution");
    pieces.chip.WriteCpu(0x2200, 0); pieces.chip.Advance(100);
    Check(pieces.chip.ReadSa1(0x20) != whole.chip.ReadSa1(0x20), "WAIT release resumes execution without a reset");
}

void ExecutableDmaAndTimer() {
    Fixture f;
    auto& c = f.chip;
    f.Code(0x8000, {
        0xa9,0xff,0x8d,0x2a,0x22, // Stack/IRAM write enable.
        0xa9,0x20,0x8d,0x0a,0x22,0x58, // DMA IRQ enable and CLI.
        0xa9,0x80,0x8d,0x30,0x22, // ROM -> IRAM DMA.
        0xa9,0x00,0x8d,0x32,0x22,
        0xa9,0x90,0x8d,0x33,0x22,
        0xa9,0x00,0x8d,0x34,0x22, // Source $00:9000.
        0xa9,0x04,0x8d,0x38,0x22,0x9c,0x39,0x22,
        0xa9,0x00,0x8d,0x35,0x22,
        0xa9,0x06,0x8d,0x36,0x22, // Destination $0600; launches DMA.
        0xcb,0xe6,0x10,0xdb
    });
    f.Code(0x8100, {0xe6,0x20,0xa9,0x20,0x8d,0x0b,0x22,0x40});
    f.Code(0x9000, {0x12,0x34,0x56,0x78});
    SetCpu(c, 0x2207, 0x8100);
    f.Start(); c.Advance(1000);
    Check(c.CpuRegisters().stp && Get(c, 0x600, 4) == 0x78563412 &&
          c.ReadSa1(0x20) == 1 && c.ReadSa1(0x10) == 1 && !(c.ReadSa1(0x2301) & 0x20),
          "A running SA-1 program starts DMA, receives its completion IRQ, acknowledges it, and resumes past WAI");

    Fixture timer;
    auto& t = timer.chip;
    timer.Code(0x8000, {0xa9,0xff,0x8d,0x2a,0x22,0x18,0xfb,
                       0xa9,0x40,0x8d,0x0a,0x22,0x58,0xcb,0xe6,0x10,0xdb});
    timer.Code(0x8100, {0xe6,0x20,0xa9,0x40,0x8d,0x0b,0x22,0x9c,0x10,0x22,0x40});
    SetCpu(t, 0x2207, 0x8100); Set(t, 0x2212, 40); t.WriteSa1(0x2210, 1);
    timer.Start(); t.Advance(1000);
    Check(t.CpuRegisters().stp && !t.CpuRegisters().e && t.CpuRegisters().s == 0x1ff &&
          t.ReadSa1(0x20) == 1 && t.ReadSa1(0x10) == 1 && !(t.ReadSa1(0x2301) & 0x40),
          "Native-mode WAI wakes on the timer IRQ and RTI restores its four-byte interrupt stack frame");
}

} // namespace

int main() {
    try {
        RegistersAndProtection(); Mapping(); Arithmetic(); VariableBits();
        Timers(); Dma(); CharacterConversion(); ExecutableProgram(); Interrupts(); Synchronization(); ExecutableDmaAndTimer();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "SA-1 regression failed: %s\n", error.what());
        return 1;
    }
    std::puts("SA-1 processor, mapping, protection, interrupts, timers, arithmetic, DMA, conversion, and synchronization checks passed");
}
