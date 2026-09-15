#include "snes/core/Emulator.hpp"
#include "snes/core/Spc7110.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>

using namespace snes::core;
namespace {
void Check(bool result, const char* message) { if (!result) throw std::runtime_error(message); }
void Set(Spc7110& chip, unsigned address, uint32_t value, unsigned bytes = 2) {
    for (unsigned i = 0; i < bytes; ++i) chip.mmio_write(address + i, uint8_t(value >> (i * 8)));
}
uint32_t Get(Spc7110& chip, unsigned address, unsigned bytes = 2) {
    uint32_t value = 0;
    for (unsigned i = 0; i < bytes; ++i) value |= uint32_t(chip.mmio_read(address + i)) << (i * 8);
    return value;
}
std::vector<uint8_t> Rom(bool rtc = false, size_t size = 0x300000) {
    std::vector<uint8_t> rom(size);
    for (size_t i = 0; i < size; ++i) rom[i] = uint8_t((i >> 20) * 31 + i);
    std::fill_n(rom.begin() + 0xffc0, 64, 0);
    std::fill_n(rom.begin() + 0xffc0, 21, ' ');
    rom[0x8000] = 0xdb; rom[0xffc0] = 'T'; rom[0xffd5] = 0x3a;
    rom[0xffd6] = rtc ? 0xf9 : 0xf5; rom[0xffd7] = size > 0x500000 ? 13 : 12;
    rom[0xffd8] = 3; rom[0xffdc] = rom[0xffdd] = 0xff; rom[0xfffd] = 0x80;
    return rom;
}

void Arithmetic() {
    const auto rom = Rom(); Spc7110 chip(rom, false);
    for (bool sign : {false, true}) {
        chip.mmio_write(0x482e, sign);
        for (uint16_t a : {0, 1, 0x7fff, 0x8000, 0xffff})
            for (uint16_t b : {0, 1, 0x7fff, 0x8000, 0xffff}) {
                Set(chip, 0x4820, a); Set(chip, 0x4824, b);
                const uint32_t expected = sign ? uint32_t(int64_t(int16_t(a)) * int16_t(b)) : uint32_t(a) * b;
                Check(Get(chip, 0x4828, 4) == expected, "Signed and unsigned multiplication");
                Check(chip.mmio_read(0x482f) == 0x80 && chip.mmio_read(0x482f) == 0, "Math status reads clear bit seven");
            }
        for (uint32_t a : {0u, 1u, 0x7fffffffu, 0x80000000u, 0xffffffffu})
            for (uint16_t b : {0, 1, 2, 0x7fff, 0x8000, 0xffff}) {
                Set(chip, 0x4820, a, 4); Set(chip, 0x4826, b);
                const int64_t dividend = sign ? int64_t(int32_t(a)) : int64_t(a);
                const int64_t divisor = sign ? int64_t(int16_t(b)) : int64_t(b);
                Check(Get(chip, 0x4828, 4) == (divisor ? uint32_t(dividend / divisor) : 0), "Division includes overflow and zero divisor");
                Check(Get(chip, 0x482c) == (divisor ? uint16_t(dividend % divisor) : uint16_t(a)), "Division remainder");
            }
    }
    chip.mmio_write(0x482e, 0);
    for (unsigned address = 0x4820; address <= 0x482d; ++address)
        Check(chip.mmio_read(address) == 0, "Math control resets operands and results");
    Check(chip.mmio_read(0x4840, 0x56) == 0x56 && chip.SaveRtc().empty(), "Non-clock board leaves RTC open");
}

void DataPorts() {
    const auto rom = Rom(); Spc7110 chip(rom, false);
    chip.mmio_write(0x4818, 0xff);
    Check(chip.mmio_read(0x4818) == 0 && chip.mmio_read(0x4810) == 0, "Data port waits for all pointer bytes");
    Set(chip, 0x4811, 100, 3);
    Check(chip.mmio_read(0x4810) == rom[0x100064] && Get(chip, 0x4811, 3) == 101, "Default data read increments pointer");
    Set(chip, 0x4816, 0xfffe); chip.mmio_write(0x4818, 5);
    chip.mmio_read(0x4810);
    Check(Get(chip, 0x4811, 3) == 99, "Signed negative increment");
    Set(chip, 0x4814, 0xffff); chip.mmio_write(0x4818, 0x0a);
    Check(chip.mmio_read(0x4810) == rom[0x100062] && Get(chip, 0x4814) == 0 &&
          Get(chip, 0x4811, 3) == 99, "Adjusted reads advance adjustment instead of pointer");
    chip.mmio_write(0x4818, 0x22); Set(chip, 0x4814, 3);
    Check(Get(chip, 0x4811, 3) == 102, "Adjust-write mode advances only after both bytes arrive");
    chip.mmio_write(0x4818, 0x68); Set(chip, 0x4814, 0xfffe);
    Check(chip.mmio_read(0x481a) == rom[0x100064] && Get(chip, 0x4811, 3) == 100, "Secondary data port applies signed adjustment");
    chip.mmio_write(0x4818, 0x70); Set(chip, 0x4814, 3);
    chip.mmio_read(0x481a);
    Check(Get(chip, 0x4814) == 6 && Get(chip, 0x4811, 3) == 100, "Secondary port can double adjustment");
    chip.mmio_write(0x4818, 0); Set(chip, 0x4811, 0x1fffff, 3);
    Check(chip.mmio_read(0x4810) == rom.back() && chip.mmio_read(0x4810) == rom[0x100000], "Data ROM wraps at its physical size");
}

void DecoderIsolation() {
    auto rom = Rom();
    uint32_t state = 0x3157a913;
    for (size_t i = 0x100000; i < rom.size(); ++i) { state ^= state << 13; state ^= state >> 17; state ^= state << 5; rom[i] = uint8_t(state); }
    for (unsigned mode = 0; mode < 3; ++mode) {
        Spc7110Decoder first, second, expected;
        first.SetRom(rom); second.SetRom(rom); expected.SetRom(rom);
        first.init(mode, 0x1234, 0); expected.init(mode, 0x1234, 0);
        second.init((mode + 1) % 3, 0x1fffff, 111);
        std::vector<uint8_t> bytes(4096);
        for (auto& byte : bytes) byte = expected.read();
        for (auto byte : bytes) { Check(first.read() == byte, "Interleaved decoder instances retain independent state"); second.read(); }
        first.init(mode, 0x1234, 123);
        for (unsigned i = 123; i < bytes.size(); ++i) Check(first.read() == bytes[i], "Decoder seek preserves stream position");
    }
    Spc7110 chip(rom, false);
    for (unsigned mode : {3, 32, 255}) {
        rom[0x100000] = uint8_t(mode);
        Set(chip, 0x4805, 0xffff);
        Check(chip.mmio_read(0x480c) == 0x80 && chip.mmio_read(0x480c) == 0, "Decompression status is read-clear");
        Check(chip.mmio_read(0x4800) == 0, "Invalid mode produces zero without an invalid shift or long seek");
    }
}

void DecoderVectors() {
    std::vector<uint8_t> rom(0x300000);
    for (unsigned i = 0; i < rom.size(); ++i) rom[i] = uint8_t((i * 37) ^ (i >> 4) ^ 0xa5);
    constexpr uint8_t prefixes[3][32] = {
        {0x03,0x03,0x02,0x60,0x1e,0x6c,0x02,0x00,0x11,0x0c,0x02,0x10,0x0e,0x03,0x1d,0x0e,
         0x01,0x03,0x12,0x3f,0x1e,0x33,0x02,0x01,0x11,0x01,0x72,0x0c,0x6e,0x00,0x72,0x1c},
        {0x00,0x00,0x0c,0x18,0x10,0xcc,0x97,0x28,0x00,0xef,0xa1,0xee,0xf5,0xe0,0xfb,0x6c,
         0x77,0x2e,0x72,0x05,0x71,0x88,0x51,0xac,0x77,0x88,0x73,0x94,0x69,0x9e,0x74,0x9a},
        {0x3b,0x39,0xff,0x89,0xfa,0x05,0xaa,0x05,0x32,0xc0,0x0d,0xdd,0xe1,0xde,0x45,0xa9,
         0x33,0x20,0x40,0x3f,0xa4,0xbf,0xff,0xfa,0xff,0x20,0xcc,0x29,0x1e,0x51,0x5e,0xf1}
    };
    constexpr uint64_t hashes[]{0x47da7bb6bb025fceULL, 0xb41a6b3287517f28ULL, 0xd7be3ec8ef36ab90ULL};
    for (unsigned mode = 0; mode < 3; ++mode) {
        Spc7110Decoder decoder;
        decoder.SetRom(rom);
        decoder.init(mode, 0x1234, 0);
        uint64_t hash = 14695981039346656037ULL;
        for (unsigned i = 0; i < 4096; ++i) {
            const auto byte = decoder.read();
            if (i < 32) Check(byte == prefixes[mode][i], "Recorded decoder prefix covers each compressed graphics mode");
            hash = (hash ^ byte) * 1099511628211ULL;
        }
        Check(hash == hashes[mode], "Recorded 4096-byte stream covers context evolution and buffer refills");
    }
}

void MappingAndDma() {
    auto rom = Rom();
    auto emu = std::make_unique<Emulator>();
    Check(emu->LoadCartridge(rom), "SPC7110 loads");
    Check(emu->LoadedCartridge()->Header().mapping == MappingType::Spc7110, "Chip selects its board mapper");
    auto& bus = emu->GetBus();
    Check(bus.Read(0x008000) == 0xdb && bus.Read(0xc08000) == 0xdb, "Program ROM mapping");
    for (unsigned page = 0; page < 8; ++page) {
        bus.Write(0x004831, uint8_t(page));
        Check(bus.Read(0xd01234) == rom[0x100000 + ((page & 1) << 20) + 0x1234], "Data bank selector wraps ROM pages");
    }
    bus.Write(0x7e0000, 0x5a);
    Check(bus.Read(0x108000) == 0x5a, "Unpopulated program window stays open");
    bus.Write(0x006000, 0x43);
    Check(bus.Read(0x006000) == 0, "SRAM powers up write protected");
    bus.Write(0x804830, 0x80); bus.Write(0x306123, 0x43);
    Check(bus.Read(0x006123) == 0x43 && emu->LoadedCartridge()->SramData()[0x123] == 0x43, "Shared SRAM aliases and save storage");
    bus.Write(0x004830, 0); bus.Write(0x006123, 0x10);
    Check(bus.Read(0x306123) == 0x43, "SRAM protection can be restored");
    // A table entry in data ROM selects a mode-1 stream at offset $1000.
    rom[0x100000] = 1; rom[0x100001] = 0; rom[0x100002] = 0x10; rom[0x100003] = 0;
    Check(emu->LoadCartridge(rom), "Reload stream fixture");
    bus.Write(0x004806, 0);
    Spc7110Decoder decoder; decoder.SetRom(rom); decoder.init(1, 0x1000, 0);
    std::vector<uint8_t> actual;
    bus.Map(0, 0, 0x2110, 0x2110, [](uint32_t, uint8_t v) { return v; },
            [&](uint32_t, uint8_t value) { actual.push_back(value); });
    auto& dma = emu->GetDma(); auto& channel = dma.Channel(0);
    channel.writeControl(0); channel.sourceBank = 0x50; channel.sourceAddress = 0xfff0;
    channel.targetAddress = 0x10; channel.transferSize = 100;
    dma.EnableDma(1); dma.RunDma();
    Check(actual.size() == 100, "DMA reads decompression stream across source-bank wrap");
    for (auto byte : actual) Check(byte == decoder.read(), "DMA and manual decoder output agree");
    Check(bus.Read(0x004809) == uint8_t(-100) && bus.Read(0x00480a) == 0xff, "Decompression length wraps on every read");
    auto broken = rom; broken.resize(0x100000);
    Check(!emu->LoadCartridge(broken), "Truncated SPC7110 image is rejected");
    Check(bus.Read(0x008000) == 0xdb, "Failed load preserves running cartridge");
}

void Clock() {
    using namespace std::chrono;
    const auto rom = Rom(true);
    int64_t now = duration_cast<seconds>(sys_days(2024y/February/28).time_since_epoch()).count();
    Spc7110 chip(rom, true, [&] { return now; });
    const auto select = [&](unsigned index) { chip.mmio_write(0x4840, 1); chip.mmio_write(0x4841, 3); chip.mmio_write(0x4841, uint8_t(index)); };
    select(0);
    const std::array<uint8_t, 16> digits{9,5,9,5,3,2,8,2,2,0,4,2,3,0,0,0};
    for (auto d : digits) chip.mmio_write(0x4841, d);
    chip.mmio_write(0x4840, 0);
    ++now;
    auto save = chip.SaveRtc();
    Check(save.size() == 24 && save[0] == 0 && save[4] == 0 && save[6] == 9 && save[7] == 2 && save[12] == 4, "RTC crosses midnight into leap day");
    select(13); chip.mmio_write(0x4841, 2);
    Check(chip.SaveRtc()[0] == 1, "RTC second increment command advances time");
    select(15); chip.mmio_write(0x4841, 2);
    now += 3600;
    Check(chip.SaveRtc()[0] == 1 && chip.SaveRtc()[4] == 0, "Stopped RTC does not advance");
    select(15); chip.mmio_write(0x4841, 0); now += 1;
    Check(chip.SaveRtc()[0] == 2, "RTC resumes without counting the stopped interval");
    save = chip.SaveRtc();
    Spc7110 restored(rom, true, [&] { return now; });
    Check(restored.LoadRtc(save) && restored.SaveRtc() == save, "Clock save round trip");
    save[0] = 0xff;
    Check(!restored.LoadRtc(save), "Malformed clock save is rejected");
    chip.mmio_write(0x4840, 1); chip.mmio_write(0x4841, 0x0c); chip.mmio_write(0x4841, 6);
    Check(chip.mmio_read(0x4841) == 9 && chip.mmio_read(0x4841) == 2, "Indexed clock reads advance nibble position");
    Check(chip.mmio_read(0x4842) == 0x80 && chip.mmio_read(0x4842) == 0, "Clock status clears on read");
    auto emu = std::make_unique<Emulator>();
    Check(emu->LoadCartridge(rom) && emu->SaveRtc().size() == 24, "RTC board exposes frontend save data");
    Check(emu->LoadRtc(emu->SaveRtc()), "Frontend clock load is connected");
}
}
int main() {
    try { Arithmetic(); DataPorts(); DecoderIsolation(); DecoderVectors(); MappingAndDma(); Clock(); }
    catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
    std::puts("SPC7110 decoder, registers, mapping, DMA, and clock checks passed");
}
