// snes emulator
// tests/test_cartridge_devices.cpp
// Regression coverage for cartridge mappers and enhancement devices.

#include "snes/core/Emulator.hpp"
#include "snes/core/SnesCpu.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

using namespace snes::core;
namespace {
unsigned checks = 0;
void Check(bool value, const char* label) {
    ++checks;
    if (!value) throw std::runtime_error(label);
}

std::vector<uint8_t> Rom(size_t size = 0x100000, uint8_t mode = 0x20, uint8_t type = 0) {
    std::vector<uint8_t> bytes(size, 0xea);
    const size_t header = mode == 0x35 || mode == 0x25 ? 0x40ffc0 :
                          (mode & 1) && (mode & 15) == 1 ? 0xffc0 : 0x7fc0;
    std::fill_n(bytes.begin() + header, 64, 0);
    std::fill_n(bytes.begin() + header, 21, ' ');
    bytes[header] = 'T';
    bytes[header + 0x15] = mode;
    bytes[header + 0x16] = type;
    bytes[header + 0x17] = 10;
    bytes[header + 0x19] = 1;
    bytes[header + 0x1c] = 0x34; bytes[header + 0x1d] = 0x12;
    bytes[header + 0x1e] = 0xcb; bytes[header + 0x1f] = 0xed;
    bytes[header + 0x3d] = 0x80;
    return bytes;
}

std::unique_ptr<Emulator> Load(const std::vector<uint8_t>& bytes) {
    auto emu = std::make_unique<Emulator>();
    std::string error;
    if (!emu->LoadCartridge(bytes, &error)) throw std::runtime_error(error);
    return emu;
}

void DetectionAndReload() {
    struct Fixture { uint8_t mode, type, maker; EnhancementChip chip; bool supported; };
    for (const auto f : {
        Fixture{0x20, 3, 0, EnhancementChip::Dsp1, true},
        Fixture{0x21, 5, 0, EnhancementChip::Dsp1, true},
        Fixture{0x30, 5, 0, EnhancementChip::Dsp1, true},
        Fixture{0x20, 5, 0, EnhancementChip::Dsp2, true},
        Fixture{0x30, 5, 0xb2, EnhancementChip::Dsp3, false},
        Fixture{0x30, 3, 0, EnhancementChip::Dsp4, false},
        Fixture{0x30, 0x25, 0, EnhancementChip::Obc1, true},
        Fixture{0x35, 0x55, 0, EnhancementChip::Srtc, true},
        Fixture{0x23, 0x35, 0, EnhancementChip::Sa1, false},
        Fixture{0x20, 0x15, 0, EnhancementChip::SuperFx, false},
        Fixture{0x32, 0x43, 0, EnhancementChip::Sdd1, true},
        Fixture{0x3a, 0xf5, 0, EnhancementChip::Spc7110, false},
        Fixture{0x3a, 0xf9, 0, EnhancementChip::Spc7110Rtc, false},
        Fixture{0x20, 0xf3, 0, EnhancementChip::Cx4, false},
        Fixture{0x30, 0xf5, 0, EnhancementChip::St018, false},
        Fixture{0x30, 0xf6, 0, EnhancementChip::St010, true}}) {
        auto bytes = Rom(f.mode == 0x35 ? 0x500000 : 0x100000, f.mode, f.type);
        const size_t header = f.mode == 0x35 ? 0x40ffc0 : f.mode == 0x21 ? 0xffc0 : 0x7fc0;
        bytes[header + 0x1a] = f.maker;
        auto cart = Cartridge::FromRomImage(bytes, nullptr, nullptr, nullptr);
        Check(cart && cart->Header().chip == f.chip, "Chip detection uses mode, type and maker");
        auto emu = Load(Rom());
        emu->GetBus().Write(0x7e0123, 0x5a);
        std::string error;
        Check(emu->LoadCartridge(bytes, &error) == f.supported, "Load support matches the chip implementation");
        if (!f.supported) {
            Check(error.find(ChipName(f.chip)) != std::string::npos, "Unsupported chip error identifies the device");
            Check(emu->Initialized() && emu->GetBus().Read(0x7e0123) == 0x5a,
                  "Failed load preserves the running machine");
            Check(emu->GetBus().Read(0x008000) == 0xea, "Failed load leaves valid ROM bus handlers");
            emu->StepFrame();
        }
    }
    auto st011 = Rom(0x100000, 0x30, 0xf6);
    st011[0x7fd7] = 9;
    Check(Cartridge::FromRomImage(st011, nullptr, nullptr, nullptr)->Header().chip == EnhancementChip::St011,
          "ST011 detection includes ROM size");
    auto emu = Load(Rom());
    emu->StepFrame();
    Check(emu->LoadCartridge(Rom()), "Reload succeeds");
    Check(emu->CurrentFrame() == 0 && emu->CurrentMasterCycles() == 0, "Reload resets frame counters");
}

void LoaderAndRam() {
    std::vector<uint8_t> blank(0x10000, 0);
    Check(!Cartridge::FromRomImage(blank, nullptr, nullptr, nullptr), "Reject a blank image without a reset vector");
    std::fill(blank.begin(), blank.end(), 0xff);
    Check(!Cartridge::FromRomImage(blank, nullptr, nullptr, nullptr), "Reject an erased image");
    auto bytes = Rom();
    bytes[0x7fd8] = 3;
    auto cart = Cartridge::FromRomImage(bytes, nullptr, nullptr, nullptr);
    CartridgeDatabase database;
    DatabaseOverride override;
    override.forceSramSize = 0;
    database.AddOverride(cart->RomCrc32(), override);
    cart = Cartridge::FromRomImage(bytes, nullptr, &database, nullptr);
    Check(cart->SramData().empty(), "A zero-byte SRAM override disables header SRAM");
    auto emu = Load(bytes);
    auto& bus = emu->GetBus();
    bus.Write(0x708123, 0x57);
    Check(bus.Read(0x700123) == 0x57 && bus.Read(0xf08123) == 0x57, "Small LoROM SRAM uses both bank halves");
    bytes[0x7fd8] = 6;
    emu = Load(bytes);
    emu->GetBus().Write(0x708123, 0x57);
    Check(emu->GetBus().Read(0x708123) == 0xea, "Large SRAM boards retain ROM in the upper half");
    emu->GetBus().Write(0x700123, 0x57);
    emu->GetBus().Write(0x710123, 0x63);
    Check(emu->GetBus().Read(0x700123) == 0x57 && emu->GetBus().Read(0x710123) == 0x63,
          "Large SRAM banks remain independent");
    emu = Load(Rom());
    emu->GetBus().Write(0x7e0000, 0x63);
    Check(emu->GetBus().Read(0x700000) == 0x63, "Absent LoROM SRAM reads open bus");
    Check(emu->GetBus().Read(0x708000) == 0x63, "Absent small-board upper SRAM reads open bus");
    Check(emu->GetBus().Read(0xf08000) == 0xea, "Upper ROM mirror survives when SRAM is absent");
    emu = Load(Rom(0x100000, 0x21));
    emu->GetBus().Write(0x7e0000, 0x57);
    Check(emu->GetBus().Read(0x206000) == 0x57, "Absent HiROM SRAM reads open bus");
    emu->GetBus().Write(0x420d, 1);
    Check(emu->GetBus().Speed(0xc08000) == 6 && emu->LoadedCartridge()->AccessCycles(0xc08000) == 6,
          "MEMSEL timing does not depend on ROM header speed");

    // A 96 KiB image cannot be split into equal pairs of 32 KiB blocks.
    bytes = Rom(0x18000);
    bytes[0x7fd5] = 0x21;
    bytes.back() = 0x57;
    RomNormalizationInfo info;
    cart = Cartridge::FromRomImage(bytes, &info, nullptr, nullptr);
    Check(cart && !info.hadInterleave && cart->RomData().back() == 0x57, "Odd block count cannot drop the last bank");

    // Random low-bank bytes must not reorder a valid HiROM prototype.
    bytes = Rom(0x20000, 0x21);
    bytes[0x7fd5] = 0x21;
    bytes[0xffd5] = 0xff;
    bytes[0x8000] = 0x78;
    cart = Cartridge::FromRomImage(bytes, &info, nullptr, nullptr);
    Check(cart && !info.hadInterleave && cart->Read(0x008000) == 0x78, "Best header controls deinterleave detection");

    bytes = Rom(0x20000, 0x21);
    std::vector<uint8_t> interleaved(bytes.size());
    for (size_t pair = 0; pair < 2; ++pair) {
        std::copy_n(bytes.begin() + (pair * 2 + 1) * 0x8000, 0x8000, interleaved.begin() + pair * 0x8000);
        std::copy_n(bytes.begin() + pair * 0x10000, 0x8000, interleaved.begin() + (pair + 2) * 0x8000);
    }
    cart = Cartridge::FromRomImage(interleaved, &info, nullptr, nullptr);
    Check(cart && info.hadInterleave && std::equal(bytes.begin(), bytes.end(), cart->RomData().begin()),
          "Even interleaved HiROM restores every byte");
    bytes = Rom(0x500000, 0x35);
    bytes[0x40ffd5] = 0x21;
    bytes[0x408000] = 0x78;
    cart = Cartridge::FromRomImage(bytes, &info, nullptr, nullptr);
    Check(cart && cart->Header().mapping == MappingType::ExHiRom && cart->Read(0x008000) == 0x78,
          "Extended HiROM header location selects the upper ROM chip");
}

void BitmapProcessor() {
    auto emu = Load(Rom(0x100000, 0x20, 5));
    auto& bus = emu->GetBus();
    const std::array<unsigned, 4> ports{0x206000, 0x3f8001, 0xa06fff, 0xbfbfff};
    for (unsigned port : ports) Check(bus.Read(port) == 0xff, "DSP-2 empty read");
    Check(bus.Read(0x20c000) == 0xea, "DSP-2 does not replace the upper ROM window");
    bus.Write(0x206000, 9);
    for (unsigned i = 0; i < 4; ++i) bus.Write(ports[i], 0xff);
    const std::array<uint8_t, 4> product{1, 0, 0xfe, 0xff};
    for (unsigned i = 0; i < 4; ++i) Check(bus.Read(ports[i]) == product[i], "DSP-2 unsigned multiplication");
    Check(bus.Read(ports[0]) == 0xff, "DSP-2 output consumption");
    bus.Write(ports[0], 1);
    for (unsigned row = 0; row < 8; ++row)
        for (uint8_t value : {0x01, 0x23, 0x45, 0x67}) bus.Write(ports[row % 4], value);
    for (unsigned i = 0; i < 32; ++i)
        Check(bus.Read(ports[i % 4]) == (i < 16 ? (i % 2 ? 0x33 : 0x55) : (i % 2 ? 0 : 0x0f)),
              "DSP-2 packed pixels become four bitplanes");
    for (uint8_t byte : {3, 0x1f, 5, 2, 0x12, 0x34, 0xf5, 0x6f}) bus.Write(ports[0], byte);
    Check(bus.Read(ports[1]) == 0x15 && bus.Read(ports[2]) == 0x64, "DSP-2 transparent overlay");
    for (uint8_t byte : {6, 3, 0x12, 0x34, 0x56}) bus.Write(ports[0], byte);
    for (uint8_t byte : {0x65, 0x43, 0x21}) Check(bus.Read(ports[3]) == byte, "DSP-2 reverse pixels");
    // Fixed-point shrinking samples source pixels at 0, 3, 6, 9.
    for (uint8_t byte : {0x0d, 8, 2, 0x12, 0x34, 0x56, 0x78}) bus.Write(ports[0], byte);
    Check(bus.Read(ports[0]) == 0x14 && bus.Read(ports[0]) == 0x71, "DSP-2 fixed-point scale and retained parameter RAM");
    for (uint8_t op : {5, 6, 0x0d}) {
        bus.Write(ports[0], op); bus.Write(ports[0], 0);
        if (op == 0x0d) bus.Write(ports[0], 0);
        for (uint8_t byte : {9, 2, 0, 3, 0}) bus.Write(ports[0], byte);
        Check(bus.Read(ports[0]) == 6, "Empty DSP-2 requests do not swallow the next command");
        for (unsigned i = 0; i < 3; ++i) Check(bus.Read(ports[0]) == 0, "DSP-2 result after an empty request");
    }
    bus.Write(ports[0], 5); bus.Write(ports[0], 255);
    for (unsigned i = 0; i < 255; ++i) bus.Write(ports[0], 0xab);
    for (unsigned i = 0; i < 255; ++i) bus.Write(ports[0], 0xff);
    for (unsigned i = 0; i < 255; ++i) Check(bus.Read(ports[0]) == 0xab, "DSP-2 maximum overlay length");
    Check(emu->LoadCartridge(Rom()), "Replacing a DSP-2 cartridge succeeds");
    Check(emu->GetBus().Read(0x208000) == 0xea, "Reload removes DSP-2 handlers");
}

void ObjectController() {
    auto emu = Load(Rom(0x100000, 0x30, 0x25));
    auto& bus = emu->GetBus();
    Check(bus.Read(0x006000) == 0xff && bus.Read(0xbf7ff6) == 0xff, "OBC1 power-on RAM");
    for (unsigned select = 0; select < 2; ++select) {
        bus.Write(0x007ff5, static_cast<uint8_t>(select));
        const unsigned base = select ? 0x7800 : 0x7c00;
        for (unsigned object : {0u, 1u, 126u, 127u}) {
            bus.Write(0x807ff6, static_cast<uint8_t>(object | 0x80));
            for (unsigned byte = 0; byte < 4; ++byte) {
                bus.Write(0x3f7ff0 + byte, static_cast<uint8_t>(object + byte));
                Check(bus.Read(base + object * 4 + byte) == uint8_t(object + byte), "OBC1 object port writes backing RAM");
                Check(bus.Read(0xbf7ff0 + byte) == uint8_t(object + byte), "OBC1 object ports share bank mirrors");
                Check(bus.Read(0x7e7ff0 + byte) != uint8_t(object + byte), "OBC1 leaves WRAM mapped");
            }
        }
        bus.Write(base + 0x200, 0);
        for (unsigned object = 0; object < 4; ++object) {
            bus.Write(0x007ff6, static_cast<uint8_t>(object));
            bus.Write(0x007ff4, static_cast<uint8_t>(object | 0xfc));
        }
        Check(bus.Read(0x007ff4) == 0xe4 && bus.Read(base + 0x200) == 0xe4, "OBC1 two-bit attributes preserve neighbors");
    }
    Check(bus.Read(0x007ff5) == 1 && bus.Read(0x807ff6) == 3, "OBC1 selector readback");
    Check(emu->LoadCartridge(Rom(0x100000, 0x30, 0x25)), "OBC1 reload");
    Check(emu->GetBus().Read(0x007ff6) == 0xff && emu->GetBus().Read(0x007800) == 0xff, "OBC1 resets on reload");
}

void Signature(std::vector<uint8_t>& bytes, size_t offset, std::string_view text) {
    std::copy(text.begin(), text.end(), bytes.begin() + offset);
}

void Sufami() {
    auto bios = Rom(0x40000);
    Signature(bios, 0, "BANDAI SFC-ADX");
    Signature(bios, 0x10, "SFC-ADX BACKUP");
    std::vector<uint8_t> a(0x80000, 0x11), b(0x100000, 0x22);
    Signature(a, 0, "BANDAI SFC-ADX"); Signature(b, 0, "BANDAI SFC-ADX");
    auto emu = std::make_unique<Emulator>();
    Check(emu->LoadSufamiTurbo(bios, a, b), "Load two Sufami Turbo slots");
    Check(emu->LoadedCartridge()->Header().mapping == MappingType::SufamiTurbo, "Sufami Turbo mapper selection");
    auto& bus = emu->GetBus();
    for (unsigned bank : {0u, 7u, 8u, 0x1fu, 0x80u, 0x9fu})
        Check(bus.Read((bank << 16) | 0x8123) == bios[((bank & 7) << 15) | 0x123], "Sufami BIOS bank mirroring");
    for (unsigned bank : {0x20u, 0x2fu, 0x30u, 0x3fu, 0xa0u, 0xbfu})
        Check(bus.Read((bank << 16) | 0x8123) == 0x11, "Sufami slot A mirrors its own ROM");
    for (unsigned bank : {0x40u, 0x5fu, 0xc0u, 0xdfu})
        Check(bus.Read((bank << 16) | 0x8123) == 0x22, "Sufami slot B bank selection");
    bus.Write(0x608123, 0x57); bus.Write(0x708123, 0x63);
    Check(bus.Read(0xe3c123) == 0x57 && bus.Read(0xf3c123) == 0x63, "Sufami slot SRAM is separate and mirrored");
    Check(emu->LoadedCartridge()->SlotSramData(0)[0x123] == 0x57 &&
          emu->LoadedCartridge()->SlotSramData(1)[0x123] == 0x63, "Sufami save API exposes both slots");
    const std::vector<uint8_t> saved(emu->LoadedCartridge()->SlotSramData(0).begin(),
                                     emu->LoadedCartridge()->SlotSramData(0).end());
    bus.Write(0x7e8123, 0xa5);
    Check(bus.Read(0x7e8123) == 0xa5, "Sufami preserves WRAM");
    bus.Write(0x400123, 0x76);
    Check(bus.Read(0x400123) == 0x76, "Sufami lower cartridge windows read open bus");
    Check(!emu->LoadSufamiTurbo(bios, bios, b), "Reject a BIOS passed as a game");
    Check(bus.Read(0x608123) == 0x57, "Failed Sufami load preserves the existing slots");
    bios.insert(bios.begin(), 512, 0);
    Check(emu->LoadSufamiTurbo(bios, {}, b), "Sufami supports an empty first slot and copier headers");
    bus.Write(0x7e0000, 0x42);
    Check(bus.Read(0x208123) == 0x42 && bus.Read(0x608123) == 0x42, "Absent Sufami slot reads open bus");
    Check(emu->LoadedCartridge()->SlotSramData(0).empty(), "Absent Sufami slot has no save");
    emu->LoadSlotSram(1, saved);
    Check(bus.Read(0xf08123) == 0x57, "A Sufami save can follow its game to the other slot");
    Check(emu->LoadSufamiTurbo(bios, {}, {}), "Sufami BIOS-only operation");
}

std::array<uint8_t, 13> ReadClock(Srtc& clock) {
    clock.Write(13);
    Check(clock.Read() == 15, "S-RTC leading marker");
    std::array<uint8_t, 13> digits;
    for (auto& digit : digits) digit = clock.Read();
    Check(clock.Read() == 15, "S-RTC trailing marker");
    return digits;
}

void Clock() {
    int64_t now = 1700000000;
    Srtc clock([&] { return now; });
    const auto program = [&](std::array<uint8_t, 12> digits) {
        clock.Write(14); clock.Write(0);
        for (uint8_t digit : digits) clock.Write(digit);
    };
    program({9, 5, 9, 5, 3, 2, 8, 2, 2, 4, 2, 10}); // 2024-02-28 23:59:59
    auto digits = ReadClock(clock);
    Check(digits[12] == 3, "S-RTC computes weekday when programmed");
    clock.Write(13);
    Check(clock.Read() == 15 && clock.Read() == 9, "S-RTC starts a coherent read snapshot");
    now += 2;
    clock.Save();
    Check(clock.Read() == 5, "Saving during a clock read does not tear the timestamp");
    digits = ReadClock(clock);
    Check(digits[0] == 1 && digits[1] == 0 && digits[4] == 0 && digits[6] == 9 && digits[7] == 2 && digits[12] == 4,
          "S-RTC leap-day rollover");
    const auto saved = clock.Save();
    now += 86400;
    Srtc restored([&] { return now; });
    Check(restored.Load(saved), "S-RTC save round trip");
    digits = ReadClock(restored);
    Check(digits[6] == 1 && digits[7] == 0 && digits[8] == 3 && digits[12] == 5,
          "S-RTC advances while the emulator is closed");
    now -= 3600;
    Check(ReadClock(restored) == digits, "S-RTC ignores host clock rollback");
    auto corrupt = saved; corrupt[13] = 0;
    Check(!restored.Load(corrupt), "S-RTC rejects an invalid save signature");
    Check(!restored.Load(std::span(saved).first(20)), "S-RTC rejects truncated saves");
    program({9, 5, 9, 5, 3, 2, 8, 2, 2, 0, 0, 11}); // 2100 is not a leap year.
    ++now;
    digits = ReadClock(clock);
    Check(digits[6] == 1 && digits[7] == 0 && digits[8] == 3, "S-RTC Gregorian century rule");
    clock.Write(14); clock.Write(4);
    Check(clock.Read() == 0, "S-RTC clear leaves read mode");
    digits = ReadClock(clock);
    Check(std::all_of(digits.begin(), digits.end(), [](uint8_t d) { return d == 0; }), "S-RTC clear command");
    auto emu = Load(Rom(0x500000, 0x35, 0x55));
    auto& bus = emu->GetBus();
    bus.Write(0xbf2801, 0x1e); bus.Write(0x002801, 4); bus.Write(0x802801, 13);
    Check(bus.Read(0x3f2800) == 15 && bus.Read(0x802800) == 0, "S-RTC ports share mirrors and mask command nibbles");
    bus.Write(0x7e0000, 0x57);
    Check(bus.Read(0x002801) == 0x57, "S-RTC write port reads open bus");
    Check(emu->SaveRtc().size() == Srtc::SaveSize && emu->LoadRtc(saved), "Emulator exposes cartridge clock persistence");
}

void Regions() {
    for (unsigned country : {0u, 1u, 2u, 12u, 13u, 18u, 0xffu}) {
        auto bytes = Rom();
        bytes[0] = 0x80; bytes[1] = 0xfe; // BRA to itself.
        bytes[0x7fd9] = static_cast<uint8_t>(country);
        auto emu = Load(bytes);
        const bool pal = (country >= 2 && country <= 12) || country == 18;
        Check(emu->GetTiming().GetRegion() == (pal ? Region::PAL : Region::NTSC), "Header selects the television region");
        Check((emu->GetBus().Read(0x213f) & 0x10) == (pal ? 0x10 : 0), "PPU reports television region");
        Check(emu->GetTiming().VPeriod() == (pal ? 312 : 262), "Region selects the scanline count");
        const auto frame = emu->StepFrame();
        const uint64_t target = pal ? 425568 : 357368;
        Check(frame.masterCycles >= target && frame.masterCycles < target + 100, "Frame execution uses the selected region");
        const uint64_t apuTarget = frame.masterCycles * Smp::kClockFrequency / (pal ? 21281370 : 21477272);
        Check(emu->GetSmp().CycleCount() >= apuTarget && emu->GetSmp().CycleCount() < apuTarget + 20,
              "APU synchronization uses the region clock");
    }
}

void CombinedSufami() {
    auto bios = Rom(0x40000);
    const std::string_view signature = "BANDAI SFC-ADX";
    const std::string_view backup = "SFC-ADX BACKUP";
    std::copy(signature.begin(), signature.end(), bios.begin());
    std::copy(backup.begin(), backup.end(), bios.begin() + 0x10);
    std::vector<uint8_t> combined(0x300000, 0xff);
    std::copy(bios.begin(), bios.end(), combined.begin());
    for (const auto offset : {0x100000u, 0x200000u}) {
        std::fill_n(combined.begin() + offset, 0x100000, offset == 0x100000 ? 0x31 : 0x72);
        std::copy(signature.begin(), signature.end(), combined.begin() + offset);
    }
    auto emu = Load(combined);
    Check(emu->LoadedCartridge()->Header().mapping == MappingType::SufamiTurbo, "Combined image selects Sufami mapping");
    auto& bus = emu->GetBus();
    Check(bus.Read(0x208100) == 0x31 && bus.Read(0x408100) == 0x72, "Combined image maps both game slots");
    bus.Write(0x608010, 0x19); bus.Write(0x708010, 0x83);
    const auto save = emu->LoadedCartridge()->SramData();
    const std::vector<uint8_t> saved(save.begin(), save.end());
    Check(saved.size() == 0x8000 && saved[0x10] == 0x19 && saved[0x4010] == 0x83,
          "Combined saves contain two independent SRAM regions");
    combined.insert(combined.begin(), 512, 0);
    RomNormalizationInfo info;
    auto cart = Cartridge::FromRomImage(combined, &info, nullptr, nullptr);
    Check(cart && info.hadCopierHeader && !info.hadInterleave, "Combined copier header normalization");
    Check(emu->LoadCartridge(combined), "Reload combined cartridge");
    emu->LoadSram(saved);
    Check(bus.Read(0xe08010) == 0x19 && bus.Read(0xf08010) == 0x83, "Combined save round trip");
    combined.erase(combined.begin(), combined.begin() + 512);
    std::fill(combined.begin() + 0x100000, combined.begin() + 0x200000, 0xff);
    Check(emu->LoadCartridge(combined), "Combined image permits an erased slot");
    bus.Write(0x7e0000, 0x53);
    Check(bus.Read(0x208100) == 0x53 && bus.Read(0x608100) == 0x53, "Empty combined slot is open bus");
    combined.resize(0x280000);
    Check(!emu->LoadCartridge(combined), "Reject truncated combined slot");
    Check(bus.Read(0x408100) == 0x72, "Failed combined load preserves cartridge");
    Check(emu->LoadCartridge(bios), "Standalone base image loads");
    bus.Write(0x7e0000, 0x91);
    Check(bus.Read(0x208000) == 0x91, "Standalone base image has no game ROM in slot A");
}

void BroadcastBoards() {
    for (const bool hi : {false, true}) {
        auto base = Rom(0x200000, hi ? 0x21 : 0x20);
        const unsigned header = hi ? 0xffc0 : 0x7fc0;
        base[header - 14] = 'Z'; base[header - 11] = 'J'; base[header + 0x1a] = 0x33;
        base[header + 0x18] = 3;
        base[0x8123] = 0x36;
        std::vector<uint8_t> pack(0x100000, 0xff);
        pack[0x8123] = 0x53;
        pack[0x18123] = 0x72;
        auto emu = std::make_unique<Emulator>();
        std::string error;
        Check(emu->LoadBroadcastCartridge(base, pack, &error), "BS base and memory pack load");
        auto& bus = emu->GetBus();
        Check(emu->LoadedCartridge()->Header().mapping == (hi ? MappingType::BroadcastHiRom : MappingType::BroadcastLoRom),
              "BS board selects the correct mapper");
        Check(bus.Read(hi ? 0x008123 : 0x018123) == 0x36, "BS base ROM read");
        const unsigned port = hi ? 0xe08123 : 0xc18123;
        Check(bus.Read(port) == 0x53, "BS memory pack ROM read");
        Check(bus.Read(hi ? 0xf08123 : 0xe18123) == 0x53, "BS memory pack bank mirror");
        const unsigned ram = hi ? 0x206123 : 0x700123;
        bus.Write(ram, 0xa9);
        Check(bus.Read(ram) == 0xa9, "BS base SRAM remains writable");
        bus.Write(0x7e8123, 0x94);
        Check(bus.Read(0x7e8123) == 0x94, "BS mapping preserves WRAM");
        if (hi) {
            for (unsigned address : {0x208123u, 0x608123u, 0xa08123u})
                Check(bus.Read(address) == 0x53, "HiROM pack has ROM-only aliases");
            bus.Write(0x208123, 0x40); bus.Write(0x208123, 0);
            Check(bus.Read(port) == 0x53, "HiROM ROM-only aliases ignore flash commands");
        } else {
            bus.Write(0x7e0000, 0x62);
            Check(bus.Read(0x408000) == 0x62, "BS LoROM full-bank upper half is open bus");
        }
        bus.Write(port, 0x40); bus.Write(port, 0x0f);
        Check(bus.Read(port) == 0x80 && bus.Read(port) == 3, "Flash byte programming and one-shot status");
        bus.Write(port, 0x10); bus.Write(port, 0xff);
        bus.Read(port);
        Check(bus.Read(port) == 3, "Flash programming cannot set cleared bits");
        bus.Write(port, 0x71);
        const unsigned regBank = hi ? 0xe00000 : 0xc00000;
        Check(bus.Read(regBank + 2) == 0xc0 && bus.Read(regBank + 0x8004) == 0x82, "Flash page and global status");
        bus.Write(port, 0x75);
        Check(bus.Read(regBank + 0xff00) == 'M' && bus.Read(regBank + 0xff02) == 'P' &&
              bus.Read(regBank + 0xff06) == 0x2a, "Flash vendor identity reports one MiB");
        bus.Write(port, 0xff);
        bus.Write(port, 0x20); bus.Write(port, 0xd0);
        Check(bus.Read(port) == 0xff, "Flash block erase restores ones");
        Check(bus.Read(hi ? 0xe18123 : 0xc38123) == 0x72, "Block erase preserves adjacent block");
        bus.Write(port, 0x40); bus.Write(port, 0x64); bus.Read(port);
        const auto data = emu->LoadedCartridge()->MemoryPackData();
        const std::vector<uint8_t> saved(data.begin(), data.end());
        Check(!emu->LoadMemoryPack(std::span(saved).first(50)), "Reject short memory-pack save");
        Check(bus.Read(port) == 0x64, "Invalid save leaves memory pack intact");
        Check(emu->LoadBroadcastCartridge(base, {}, &error), "Blank memory pack loads");
        Check(emu->LoadMemoryPack(saved) && bus.Read(port) == 0x64, "Memory pack save round trip");
        base[header + 0x1a] = 0;
        Check(!emu->LoadBroadcastCartridge(base, pack, &error), "Reject a base without slot identification");
        Check(bus.Read(port) == 0x64, "Rejected base leaves the loaded pack intact");
        base[header + 0x1a] = 0x33;
        Check(!emu->LoadBroadcastCartridge(base, std::span(pack).first(0x80000), &error), "Reject unsupported pack size");
        pack[0xff00] = 'M'; pack[0xff02] = 'P'; pack[0xff06] = 0x70;
        Check(emu->LoadBroadcastCartridge(base, pack, &error), "Read-only mask ROM pack loads");
        bus.Write(port, 0x40); bus.Write(port, 0);
        Check(bus.Read(port) == 0x53 && emu->LoadedCartridge()->MemoryPackData().empty(),
              "Mask ROM pack ignores writes and has no flash save");
    }
    auto base = Rom(0x300000);
    const std::string_view title = "SOUND NOVEL-TCOOL";
    std::fill_n(base.begin() + 0x7fc0, 21, ' ');
    std::copy(title.begin(), title.end(), base.begin() + 0x7fc0);
    base[0x7fb2] = 'Z'; base[0x7fb5] = 'J'; base[0x7fda] = 0x33;
    base[0x123] = 0x11; base[0x100123] = 0x22; base[0x200123] = 0x33;
    auto cart = Cartridge::FromBroadcastCartridge(base, {});
    Check(cart && cart->Read(0x008123) == 0x11 && cart->Read(0x208123) == 0x22 &&
          cart->Read(0x808123) == 0x33 && cart->Read(0xa08123) == 0x22,
          "BS 24-Mbit board selects each base ROM chip");
}
}

int main() {
    try {
        DetectionAndReload(); LoaderAndRam(); BitmapProcessor(); ObjectController(); Sufami(); Clock(); Regions();
        CombinedSufami(); BroadcastBoards();
        std::printf("%u cartridge device checks passed\n", checks);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL after %u checks: %s\n", checks, error.what());
        return 1;
    }
}
