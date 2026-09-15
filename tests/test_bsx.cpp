#include "snes/core/Bsx.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <stdexcept>
#include <vector>

using snes::core::Bsx;
using snes::core::BroadcastTime;

namespace {
void Check(bool condition, const char* description) {
    if (!condition) throw std::runtime_error(description);
}
struct Fixture {
    std::vector<uint8_t> bios = std::vector<uint8_t>(0x100000);
    std::vector<uint8_t> pack;
    std::vector<uint8_t> sram = std::vector<uint8_t>(0x8000);
    Bsx chip;
    explicit Fixture(size_t packSize = 0x100000) : pack(packSize, 0xff), chip(bios, pack, sram) {
        for (size_t i = 0; i < bios.size(); ++i) bios[i] = uint8_t(i / 0x8000);
        for (size_t i = 0; i < chip.Psram().size(); ++i) chip.Psram()[i] = uint8_t(0x40 + i / 0x8000);
    }
    void Mmc(unsigned index, uint8_t value) { chip.WriteCpu((index << 16) | 0x5000, value); }
    void Commit(uint8_t value = 0x80) { Mmc(14, value); }
    void Bare(bool high = true) {
        for (unsigned n = 1; n <= 13; ++n) Mmc(n, 0);
        Mmc(2, high ? 0x80 : 0); Mmc(12, 0x80); Commit();
    }
    void Receiver(unsigned stream, uint16_t channel) {
        const unsigned base = 0x2188 + stream * 6;
        chip.WriteCpu(base, uint8_t(channel)); chip.WriteCpu(base + 1, uint8_t(channel >> 8));
        chip.WriteCpu(base + 3, 1); chip.WriteCpu(base + 4, 1);
    }
};

void MapperLatchesAndStorage() {
    Fixture f;
    for (unsigned reg = 0; reg < 16; ++reg) {
        const uint8_t expected = (0x0bec & (1u << reg)) ? 0x80 : 0;
        Check(f.chip.ReadCpu((reg << 16) | 0x5abc, 0x25) == (expected | 0x25),
              "MMC reset state and disconnected low data-bus bits");
    }
    f.Mmc(2, 0);
    Check(f.chip.ReadCpu(0x025000, 0) == 0x80, "Reading MMC returns the applied bit before commit");
    f.Commit(0);
    Check(f.chip.ReadCpu(0x025000, 0) == 0, "Any write to commit applies the pending mapping");
    f.Mmc(2, 0x7f); f.Commit();
    Check(f.chip.ReadCpu(0x025000, 0) == 0, "MMC writes sample only bit seven");
    f.Mmc(2, 0x80); f.Commit();
    for (unsigned bank = 0; bank < 64; ++bank) {
        Check(f.chip.ReadCpu((bank << 16) | 0x8123) == (bank & 31) &&
              f.chip.ReadCpu(((bank + 0x80) << 16) | 0x8123) == (bank & 31),
              "BIOS occupies both 64-bank upper-half windows with its physical one-megabyte mirror");
    }
    for (unsigned bank = 0x10; bank < 0x18; ++bank) {
        f.chip.WriteCpu((bank << 16) | 0x5abc, uint8_t(bank));
        Check(f.sram[(bank - 0x10) * 4096 + 0xabc] == bank, "SRAM is eight independent four-kilobyte banks");
        Check(f.chip.ReadCpu(((bank + 0x80) << 16) | 0x5abc, 0x73) == 0x73, "SRAM has no high-bank alias");
    }
    const uint8_t original = f.chip.Psram()[0x123];
    f.chip.WriteCpu(0x008123, 0xee);
    Check(f.bios[0x123] == 0 && f.chip.Psram()[0x123] == original, "BIOS writes cannot leak into underlying PSRAM");
    Check(f.chip.ReadCpu(0x600123, 0x37) == 0x37, "The default expansion-memory hole is open bus");
    f.Mmc(15, 0x80);
    for (unsigned bit = 0; bit < 8; ++bit)
        Check(f.chip.ReadCpu((bit << 16) | 0x5000, 0) == (bit < 6 ? 0x80 : 0), "Hidden MMC page power-up bits");
    f.Mmc(15, 0x80);
    Check(f.chip.ReadCpu(0x075000, 0) == 0x80, "Hidden bit seven can be written after hidden access is enabled");
    f.Mmc(15, 0);
    Check(f.chip.ReadCpu(0x0f5000, 0) == 0, "Leaving the hidden page restores the reserved-register read value");
    f.pack[3] = 0x26; f.chip.Psram()[5] = 0x51;
    f.chip.Reset();
    Check(f.pack[3] == 0x26 && f.chip.Psram()[5] == 0x51 && f.sram[0xabc] == 0x10,
          "Controller reset preserves flash, PSRAM and battery RAM");
    Check(Bsx::Selects(0x005fff) && Bsx::Selects(0x0f5000) && Bsx::Selects(0xa06abc) &&
          Bsx::Selects(0x802188) && !Bsx::Selects(0x002180) && !Bsx::Selects(0x7e1234),
          "Device windows include all mapper ports and exclude CPU WRAM accesses");
}

void PsramAndPackMapping() {
    Fixture f(0x200000);
    for (size_t i = 0; i < f.pack.size(); ++i) f.pack[i] = uint8_t(0x80 + i / 0x8000);
    for (unsigned high = 0; high < 2; ++high) {
        for (unsigned position = 0; position < 4; ++position) {
            f.Bare(high != 0);
            f.Mmc(3, 0x80); f.Mmc(4, 0x80);
            f.Mmc(5, (position & 1) ? 0x80 : 0); f.Mmc(6, (position & 2) ? 0x80 : 0); f.Commit();
            for (unsigned upper : {0u, 0x80u}) {
                const unsigned start = upper + position * (high ? 16 : 32);
                for (unsigned bank = 0; bank < (high ? 8u : 16u); ++bank) {
                    const unsigned logical = start + bank;
                    const size_t physical = bank * (high ? 65536 : 32768) + (high ? 0x8123 : 0x123);
                    Check(f.chip.ReadCpu((logical << 16) | 0x8123) == f.chip.Psram()[physical],
                          "Each selectable PSRAM window resolves the correct physical bank and half");
                    if (high)
                        Check(f.chip.ReadCpu(((logical + 0x40) << 16) | 0x0123) == f.chip.Psram()[bank * 65536 + 0x123],
                              "HiROM PSRAM also exposes the complete bank through the full-bank window");
                    else if (logical & 0x40)
                        Check(f.chip.ReadCpu((logical << 16) | 0x0123) == f.chip.ReadCpu((logical << 16) | 0x8123),
                              "LoROM PSRAM full banks mirror their lower and upper halves");
                }
                if (high) {
                    for (unsigned bank = 0x20; bank < 0x40; ++bank)
                        Check(f.chip.ReadCpu(((upper + bank) << 16) | 0x6123) == f.chip.Psram()[(bank & 7) * 65536 + 0x6123],
                              "HiROM PSRAM eight-kilobyte snippets retain their physical 64K bank stride");
                } else {
                    const unsigned end = upper ? 0x80 : 0x7e;
                    for (unsigned bank = 0x70; bank < end; ++bank) {
                        Check(f.chip.ReadCpu(((upper + bank) << 16) | 0x0123) == f.chip.Psram()[(bank & 15) * 32768 + 0x123],
                              "LoROM permanent PSRAM window covers the lower halves of high banks");
                        Check(f.chip.ReadCpu(((upper + bank) << 16) | 0x8123) == f.pack[(bank & 0x3f) * 32768 + 0x123],
                              "The upper halves of the permanent LoROM PSRAM window remain memory pack");
                    }
                }
            }
        }
    }
    for (bool high : {false, true}) {
        f.Bare(high);
        for (unsigned bank = 0; bank < 256; ++bank) {
            if (bank == 0x7e || bank == 0x7f) continue;
            const unsigned offset = bank & 0x40 ? 0x1234 : 0x9234;
            const size_t physical = high ? ((bank & 31) * 65536 + offset) : ((bank & 63) * 32768 + (offset & 0x7fff));
            Check(f.chip.ReadCpu((bank << 16) | offset) == f.pack[physical], "Memory pack uses committed HiROM/LoROM address lines");
        }
    }
}

void MappingPriority() {
    Fixture f;
    for (bool high : {false, true}) {
        for (unsigned position = 0; position < 2; ++position) {
            f.Bare(high); f.Mmc(9, 0x80); f.Mmc(10, 0x80); f.Mmc(11, position ? 0x80 : 0); f.Commit();
            for (unsigned upper : {0u, 0x80u}) {
                const unsigned bank = upper + position * (high ? 32 : 64);
                Check(f.chip.ReadCpu((bank << 16) | 0x9234, 0x6c) == 0x6c, "Enabled expansion memory shadows the pack with open bus");
                if (high) Check(f.chip.ReadCpu(((bank + 0x40) << 16) | 0x1234, 0x6c) == 0x6c,
                                "HiROM expansion hole also covers its corresponding full-bank region");
            }
        }
    }
    f.Bare(false); f.Mmc(9, 0x80); f.Mmc(3, 0x80); f.Commit();
    Check(f.chip.ReadCpu(0x008123, 0x21) == 0x40, "PSRAM overrides an expansion hole");
    f.Mmc(7, 0x80); f.Commit();
    Check(f.chip.ReadCpu(0x008123, 0x21) == 0, "BIOS overrides PSRAM and an expansion hole");
    f.Mmc(7, 0); f.Mmc(3, 0); f.Commit();
    f.chip.WriteCpu(0x008123, 0x40);
    f.Mmc(9, 0); f.Commit();
    f.chip.WriteCpu(0x008123, 0x26);
    Check(f.pack[0x123] == 0xff, "Writes to an unmapped expansion hole cannot arm a flash operation");
}

void FlashCommandsAndIrq() {
    Fixture f(0x200000);
    f.Bare();
    f.chip.WriteCpu(0xc05555, 0x75);
    Check(f.chip.ReadCpu(0xc0ff00) == 'M' && f.chip.ReadCpu(0xc0ff02) == 'P' &&
          f.chip.ReadCpu(0xc0ff06) == 0x1a && f.chip.ReadCpu(0xc0ff01) == 0xff,
          "Vendor reads identify a two-megabyte pack only at connected even offsets");
    f.chip.WriteCpu(0xc05555, 0xff);
    f.chip.WriteCpu(0xc05555, 0x70);
    Check(f.chip.ReadCpu(0xc01234) == 0x80 && f.chip.ReadCpu(0xc01234) == 0xff,
          "Compatible status has a one-read latch followed by array data");
    f.chip.WriteCpu(0xc05555, 0x71);
    Check(f.chip.ReadCpu(0xc00002) == 0xc0 && f.chip.ReadCpu(0xc08002) == 0xc0 &&
          f.chip.ReadCpu(0xc00004) == 0x82 && f.chip.ReadCpu(0xc08004) == 0x82,
          "Extended status exposes page and global status in both half-bank aliases");
    f.chip.WriteCpu(0xc05555, 0);
    f.Mmc(1, 0x80);
    f.chip.WriteCpu(0xc05555, 0x40); f.chip.WriteCpu(0xc1002b, 0xa5);
    Check(f.pack[0x1002b] == 0xa5 && f.chip.CpuIrqPending(), "Byte programming completes and raises an enabled ready interrupt");
    Check(f.chip.ReadCpu(0xc1002b) == 0x80 && f.chip.ReadCpu(0xc1002b) == 0xa5, "Program status and array data remain distinct");
    f.Mmc(1, 0);
    Check(!f.chip.CpuIrqPending() && f.chip.ReadCpu(0x005000, 0) == 0x80, "Masking ready IRQ preserves its latched flag");
    f.Mmc(0, 0x80);
    Check(f.chip.ReadCpu(0x005000, 0) == 0, "Writing mapper register zero acknowledges ready IRQ");
    f.chip.WriteCpu(0xc05555, 0x10); f.chip.WriteCpu(0xc1002b, 0x5a);
    Check(f.pack[0x1002b] == 0, "Flash programming can clear bits but cannot set them");
    f.chip.WriteCpu(0xc05555, 0x40); f.chip.WriteCpu(0xc1002b, 0xff);
    Check(f.pack[0x1002b] == 0, "A programmed FF data byte does not become a reset command");

    std::fill(f.pack.begin() + 0x10000, f.pack.begin() + 0x30000, 0);
    f.chip.WriteCpu(0xc15555, 0x20); f.chip.WriteCpu(0xc17aaa, 0xd0);
    Check(std::all_of(f.pack.begin() + 0x10000, f.pack.begin() + 0x20000, [](uint8_t v) { return v == 0xff; }) &&
          f.pack[0x20000] == 0, "Block erase affects exactly the selected physical 64K block");
    f.chip.WriteCpu(0xc05555, 0xa7); f.chip.WriteCpu(0xc05555, 0xd0);
    Check(std::all_of(f.pack.begin(), f.pack.end(), [](uint8_t v) { return v == 0xff; }), "Type-one chip erase restores the entire pack");
    f.Mmc(12, 0); f.Commit();
    f.chip.WriteCpu(0xc05555, 0x40); f.chip.WriteCpu(0xc00021, 0);
    Check(f.pack[0x21] == 0xff, "Mapper write gating protects the flash command port");

    Fixture small;
    small.Bare(); small.pack[0] = 0x23;
    small.chip.WriteCpu(0xc05555, 0x75);
    Check(small.chip.ReadCpu(0xc0ff06) == 0x2a, "A one-megabyte pack reports its own type and capacity");
    small.chip.WriteCpu(0xc05555, 0xa7); small.chip.WriteCpu(0xc05555, 0xd0);
    Check(small.pack[0] == 0x23, "Type-two packs do not implement the type-one chip-erase command");
    small.Bare(false);
    small.chip.WriteCpu(0xc05555, 0x40); small.chip.WriteCpu(0xc382ab, 0x59);
    Check(small.pack[0x182ab] == 0x59, "LoROM flash writes use the same bank mapping as reads");

    std::vector<uint8_t> mask(0x100000, 0xff);
    mask[0xff00] = 'M'; mask[0xff02] = 'P'; mask[0xff06] = 0x70;
    Bsx romPack(small.bios, mask, small.sram);
    for (unsigned n = 1; n <= 13; ++n) romPack.WriteCpu((n << 16) | 0x5000, n == 2 || n == 12 ? 0x80 : 0);
    romPack.WriteCpu(0x0e5000, 0);
    romPack.WriteCpu(0xc05555, 0x40); romPack.WriteCpu(0xc00004, 0);
    Check(mask[4] == 0xff && !romPack.CpuIrqPending(), "A mask-ROM pack ignores flash write commands");

    Fixture swapped;
    swapped.pack[0xff00] = 'M'; swapped.pack[0xff02] = 'P'; swapped.pack[0xff06] = 0x70;
    swapped.chip.Reset(); swapped.Bare();
    swapped.chip.WriteCpu(0xc05555, 0x40); swapped.chip.WriteCpu(0xc00123, 0);
    Check(swapped.pack[0x123] == 0xff && !swapped.chip.CpuIrqPending(),
          "Reset refreshes mask-ROM identity when the host replaces media bytes in a stable span");
    swapped.pack[0xff06] = 0x2a;
    swapped.chip.Reset(); swapped.Bare();
    swapped.chip.WriteCpu(0xc05555, 0x40); swapped.chip.WriteCpu(0xc00123, 0x4b);
    Check(swapped.pack[0x123] == 0x4b,
          "Replacing a mask-ROM image with writable flash and resetting restores programming");
}

void RadioAndClock() {
    Fixture f;
    Check(f.chip.ReadCpu(0x2196) == 0x10 && f.chip.ReadCpu(0x2197) == 0x80,
          "Radio interface resets to its device-present and sound settings values");
    f.chip.WriteCpu(0x2194, 0xff); f.chip.WriteCpu(0x2197, 0x53);
    Check(f.chip.ReadCpu(0x802194) == 15 && f.chip.ReadCpu(0x3f2197) == 0x53,
          "Radio registers mirror in system banks and mask the four-bit control register");
    f.Receiver(0, 0x123);
    for (unsigned i = 0; i < 300; ++i)
        Check(f.chip.ReadCpu(0x218a) == 0 && f.chip.ReadCpu(0x218b) == 0 && f.chip.ReadCpu(0x218c) == 0,
              "An absent broadcast remains empty without synthesized packet data or count underflow");
    f.chip.WriteCpu(0x2189, 0xff);
    Check(f.chip.ReadCpu(0x2189) == 0x3f, "Logical channel high byte has six connected bits");

    unsigned calls = 0;
    BroadcastTime now{2026, 9, 15, 3, 12, 34, 56};
    f.chip.SetTimeSource([&] { ++calls; return now; });
    f.Receiver(0, 0); f.Receiver(1, 0);
    Check(f.chip.ReadCpu(0x218a) == 1 && f.chip.ReadCpu(0x2190) == 1 && f.chip.ReadCpu(0x218b) == 0x90,
          "The clock channel offers one complete prefix packet to either receiver");
    Check(f.chip.ReadCpu(0x218d) == 0x90 && f.chip.ReadCpu(0x218d) == 0, "Prefix OR status clears when read");
    std::array<uint8_t, 23> record{};
    for (unsigned i = 0; i < record.size(); ++i) {
        record[i] = f.chip.ReadCpu(i < 10 ? 0x218c : 0x2192);
        now.second = 1;
    }
    Check(calls == 1 && record[4] == 0x10 && record[5] == 1 && record[6] == 1 &&
          record[10] == 56 && record[11] == 34 && record[12] == 12 && record[13] == 3 &&
          record[14] == 15 && record[15] == 9 && record[16] == 0xea && record[17] == 7,
          "Both clock data ports share one stable little-endian calendar record");
    f.chip.ReadCpu(0x218c);
    Check(calls == 2, "The next clock record samples the time provider again");
    f.chip.WriteCpu(0x218c, 0);
    Check(f.chip.ReadCpu(0x218c) == 0 && f.chip.ReadCpu(0x218a) == 0, "Disabling data also gates prefix availability");
}

void PacketStreams() {
    Fixture f;
    std::vector<uint8_t> first(45), second(22, 0xa5), large(22 * 130, 0x5a);
    for (unsigned i = 0; i < first.size(); ++i) first[i] = uint8_t(i);
    Check(!f.chip.LoadStream(0, 0, first) && !f.chip.LoadStream(0x4000, 0, first), "Host stream loader rejects reserved or unconnected channels");
    Check(f.chip.LoadStream(0x123, 0, first) && f.chip.LoadStream(0x123, 1, second) &&
          f.chip.LoadStream(0x456, 0, large), "Host can install independent channel sequences");
    f.Receiver(0, 0x123); f.Receiver(1, 0x456);
    Check(f.chip.ReadCpu(0x218a) == 3 && f.chip.ReadCpu(0x218a) == 3 && f.chip.ReadCpu(0x2190) == 127,
          "Packet count rounds up partial packets and saturates without consuming them");
    for (unsigned packet = 0; packet < 3; ++packet) {
        Check(f.chip.ReadCpu(0x218b) == (packet == 0 ? 0x10 : packet == 2 ? 0x80 : 0), "Packet prefixes distinguish first, middle and last");
        for (unsigned byte = 0; byte < 22; ++byte) {
            const unsigned position = packet * 22 + byte;
            Check(f.chip.ReadCpu(0x218c) == (position < first.size() ? first[position] : 0xff),
                  "Stream data remains sequential and pads the exhausted final packet with FF");
        }
    }
    Check(f.chip.ReadCpu(0x218b) == 0, "An extra prefix read cannot underflow an exhausted queue");
    Check(f.chip.ReadCpu(0x218d) == 0x90 && f.chip.ReadCpu(0x218d) == 0, "Prefix flags accumulate until OR status is read");
    Check(f.chip.ReadCpu(0x218a) == 1 && f.chip.ReadCpu(0x218b) == 0x90, "Next available sequence loads after the previous packet queue ends");
    for (unsigned i = 0; i < second.size(); ++i) Check(f.chip.ReadCpu(0x218c) == 0xa5, "Second stream asset has independent bytes");
    Check(f.chip.ReadCpu(0x218a) == 3, "Missing subsequent sequence wraps to the first installed asset");
    for (unsigned i = 0; i < 4; ++i) f.chip.ReadCpu(0x2191);
    Check(f.chip.ReadCpu(0x2190) == 126 && f.chip.ReadCpu(0x2192) == 0x5a,
          "The second receiver maintains its own prefix count and data cursor");

    f.Receiver(0, 0x123); f.Receiver(1, 0x123);
    f.chip.ReadCpu(0x218a); f.chip.ReadCpu(0x2190);
    Check(f.chip.ReadCpu(0x218c) == 0 && f.chip.ReadCpu(0x218c) == 1 && f.chip.ReadCpu(0x2192) == 0,
          "Two receivers can consume one asset independently");
    f.chip.ClearStreams();
    Check(f.chip.ReadCpu(0x218a) == 0 && f.chip.ReadCpu(0x2190) == 0 && f.chip.ReadCpu(0x218c) == 0,
          "Removing broadcast assets invalidates both receivers without leaving stale data");
    f.chip.LoadStream(0x123, 0, first);
    f.chip.Reset(); f.Receiver(0, 0x123);
    Check(f.chip.ReadCpu(0x218a) == 3, "Controller reset retains host-installed broadcasts but rewinds receiver state");
    f.chip.LoadStream(0x123, 0, {});
    Check(f.chip.ReadCpu(0x218a) == 0, "An empty host payload removes the selected asset");
}
}

int main() {
    try {
        MapperLatchesAndStorage(); PsramAndPackMapping(); MappingPriority();
        FlashCommandsAndIrq(); RadioAndClock(); PacketStreams();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "BS-X regression: %s\n", error.what());
        return 1;
    }
    std::puts("BS-X mapper, flash, IRQ, PSRAM, radio, clock and packet-stream checks passed");
}
