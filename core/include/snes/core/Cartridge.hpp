// snes emulator
// core/include/snes/core/Cartridge.hpp
// Cartridge metadata, address mapping, enhancement devices, and saves.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace snes::core {

enum class MappingType {
    Unknown,
    LoRom,
    HiRom,
    ExHiRom,
    ExLoRom,
    LoRomNoMad1,
    LoRom24Mbit,
    LoRomLargeSram,
    SufamiTurbo
};

const char* MappingName(MappingType mapping) noexcept;

enum class EnhancementChip {
    None, Dsp1, Dsp2, Dsp3, Dsp4, Obc1, Srtc, SuperFx, Sa1,
    Sdd1, Spc7110, Spc7110Rtc, Cx4, St010, St011, St018
};

const char* ChipName(EnhancementChip chip) noexcept;

enum class RomSpeed {
    Slow,
    Fast
};

struct RomHeader {
    size_t headerOffset = 0;
    std::string title;
    MappingType mapping = MappingType::Unknown;
    RomSpeed speed = RomSpeed::Slow;
    uint8_t mapMode = 0;
    uint8_t maker = 0;
    EnhancementChip chip = EnhancementChip::None;
    uint8_t cartridgeType = 0;
    uint8_t romSizeShift = 0;
    uint8_t sramSizeShift = 0;
    uint8_t country = 0;
    uint8_t version = 0;
    uint16_t resetVector = 0;
    uint16_t checksum = 0;
    uint16_t checksumComplement = 0;
    uint32_t score = 0;
};

struct RomNormalizationInfo {
    bool hadCopierHeader = false;
    bool hadInterleave = false;
};

struct DatabaseOverride {
    std::optional<MappingType> forceMapping;
    std::optional<size_t> forceSramSize;
    std::optional<RomSpeed> forceSpeed;
    std::optional<std::string> forceTitle;
};

class CartridgeDatabase {
public:
    CartridgeDatabase();

    void AddOverride(uint32_t crc32, DatabaseOverride entry);
    std::optional<DatabaseOverride> Find(uint32_t crc32) const;

private:
    std::unordered_map<uint32_t, DatabaseOverride> overrides_;
};

class Cartridge {
public:
    static std::optional<Cartridge> FromRomImage(std::span<const uint8_t> romImage,
                                                 RomNormalizationInfo* normalization,
                                                 const CartridgeDatabase* database,
                                                 std::string* error);

    static std::optional<Cartridge> FromSufamiTurbo(
        std::span<const uint8_t> bios, std::span<const uint8_t> slotA,
        std::span<const uint8_t> slotB, std::string* error = nullptr);

    const RomHeader& Header() const noexcept;
    uint32_t RomCrc32() const noexcept;

    void SetMemselFast(bool enabled) noexcept;
    bool MemselFast() const noexcept;

    uint8_t Read(uint32_t cpuAddress, uint8_t openBus = 0xff) const;
    void Write(uint32_t cpuAddress, uint8_t value);

    uint32_t AccessCycles(uint32_t cpuAddress) const noexcept;

    std::span<const uint8_t> RomData() const noexcept;
    std::span<const uint8_t> SramData() const noexcept;
    void LoadSram(std::span<const uint8_t> data);
    std::span<const uint8_t> SlotSramData(unsigned slot) const noexcept;
    void LoadSlotSram(unsigned slot, std::span<const uint8_t> data);

private:
    static std::optional<RomHeader> ParseHeader(std::span<const uint8_t> rom);

    static std::vector<uint8_t> RemoveCopierHeader(std::span<const uint8_t> rom,
                                                   bool* removed);
    static bool LooksLikeInterleavedHiRom(std::span<const uint8_t> rom);
    static std::vector<uint8_t> DeinterleaveHiRom(std::span<const uint8_t> rom);

    std::optional<size_t> ResolveRomOffset(uint32_t cpuAddress) const;
    std::optional<size_t> ResolveSramOffset(uint32_t cpuAddress) const;
    bool SelectsLoRomSram(uint32_t cpuAddress) const;

    static uint32_t ComputeCrc32(std::span<const uint8_t> data);

    std::vector<uint8_t> rom_;
    std::vector<uint8_t> sram_;
    RomHeader header_{};
    uint32_t romCrc32_ = 0;
    bool memselFast_ = false;
    size_t slotSize_[2]{};
    size_t slotOffset_[2]{};
};

} // namespace snes::core
