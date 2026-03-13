#include "snes/core/Cartridge.hpp"

#include "snes/core/Logging.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <limits>

namespace snes::core {

namespace {

constexpr std::array<size_t, 3> kHeaderOffsets{0x7FC0, 0xFFC0, 0x40FFC0};

size_t HeaderRomSize(uint8_t shift) {
    if (shift < 8 || shift > 30) {
        return 0;
    }

    return static_cast<size_t>(1024) << shift;
}

size_t HeaderSramSize(uint8_t shift) {
    if (shift == 0) {
        return 0;
    }

    if (shift > 14) {
        return 0;
    }

    return static_cast<size_t>(1024) << shift;
}

MappingType MappingFromModeByte(uint8_t mapMode) {
    switch (mapMode & 0x0F) {
    case 0x00:
        return MappingType::LoRom;
    case 0x01:
        return MappingType::HiRom;
    case 0x05:
        return MappingType::ExHiRom;
    default:
        return MappingType::Unknown;
    }
}

RomSpeed SpeedFromModeByte(uint8_t mapMode) {
    return (mapMode & 0x10) != 0 ? RomSpeed::Fast : RomSpeed::Slow;
}

bool IsMostlyPrintable(std::span<const uint8_t> bytes) {
    size_t printable = 0;

    for (auto value : bytes) {
        if (value == 0 || value == ' ') {
            ++printable;
            continue;
        }

        if (std::isprint(static_cast<unsigned char>(value)) != 0) {
            ++printable;
        }
    }

    return printable >= (bytes.size() * 3) / 4;
}

std::string TrimTitle(std::span<const uint8_t> bytes) {
    std::string out;
    out.reserve(bytes.size());

    for (auto value : bytes) {
        if (value == 0) {
            break;
        }

        out.push_back(static_cast<char>(value));
    }

    while (!out.empty() && (out.back() == ' ' || out.back() == '\0')) {
        out.pop_back();
    }

    return out;
}

uint16_t Read16(std::span<const uint8_t> data, size_t offset) {
    return static_cast<uint16_t>(data[offset]) |
           (static_cast<uint16_t>(data[offset + 1]) << 8);
}

size_t MirrorOffset(size_t offset, size_t size) {
    if (size == 0) {
        return 0;
    }

    return offset % size;
}

bool IsFastRegion(uint32_t cpuAddress) {
    const auto bank = static_cast<uint8_t>(cpuAddress >> 16);
    const auto addr = static_cast<uint16_t>(cpuAddress & 0xFFFF);

    if (bank >= 0xC0) {
        return true;
    }

    if (bank >= 0x80 && bank <= 0xBF && addr >= 0x8000) {
        return true;
    }

    return false;
}

} // namespace

CartridgeDatabase::CartridgeDatabase() {
    AddOverride(0x00000000, DatabaseOverride{});
}

void CartridgeDatabase::AddOverride(uint32_t crc32, DatabaseOverride entry) {
    overrides_[crc32] = std::move(entry);
}

std::optional<DatabaseOverride> CartridgeDatabase::Find(uint32_t crc32) const {
    if (const auto it = overrides_.find(crc32); it != overrides_.end()) {
        return it->second;
    }

    return std::nullopt;
}

std::optional<Cartridge> Cartridge::FromRomImage(std::span<const uint8_t> romImage,
                                                 RomNormalizationInfo* normalization,
                                                 const CartridgeDatabase* database,
                                                 std::string* error) {
    bool removedHeader = false;
    auto normalized = RemoveCopierHeader(romImage, &removedHeader);

    bool deinterleaved = false;
    if (LooksLikeInterleavedHiRom(normalized)) {
        normalized = DeinterleaveHiRom(normalized);
        deinterleaved = true;
    }

    auto header = ParseHeader(normalized);
    if (!header.has_value()) {
        if (error != nullptr) {
            *error = "Unable to parse SNES header from ROM image";
        }
        return std::nullopt;
    }

    Cartridge cart;
    cart.rom_ = std::move(normalized);
    cart.header_ = std::move(*header);
    cart.romCrc32_ = ComputeCrc32(cart.rom_);
    cart.memselFast_ = false;

    if (database != nullptr) {
        if (const auto overrideEntry = database->Find(cart.romCrc32_); overrideEntry.has_value()) {
            if (overrideEntry->forceMapping.has_value()) {
                cart.header_.mapping = *overrideEntry->forceMapping;
            }
            if (overrideEntry->forceSpeed.has_value()) {
                cart.header_.speed = *overrideEntry->forceSpeed;
            }
            if (overrideEntry->forceTitle.has_value()) {
                cart.header_.title = *overrideEntry->forceTitle;
            }
            if (overrideEntry->forceSramSize.has_value()) {
                cart.sram_.assign(*overrideEntry->forceSramSize, 0x00);
            }
        }
    }

    if (cart.sram_.empty()) {
        const auto sramSize = HeaderSramSize(cart.header_.sramSizeShift);
        cart.sram_.assign(sramSize, 0x00);
    }

    if (normalization != nullptr) {
        normalization->hadCopierHeader = removedHeader;
        normalization->hadInterleave = deinterleaved;
    }

    return cart;
}

const RomHeader& Cartridge::Header() const noexcept {
    return header_;
}

uint32_t Cartridge::RomCrc32() const noexcept {
    return romCrc32_;
}

void Cartridge::SetMemselFast(bool enabled) noexcept {
    memselFast_ = enabled;
}

bool Cartridge::MemselFast() const noexcept {
    return memselFast_;
}

uint8_t Cartridge::Read(uint32_t cpuAddress) const {
    if (const auto sramOffset = ResolveSramOffset(cpuAddress); sramOffset.has_value() && !sram_.empty()) {
        return sram_[*sramOffset];
    }

    if (const auto romOffset = ResolveRomOffset(cpuAddress); romOffset.has_value() && !rom_.empty()) {
        return rom_[*romOffset];
    }

    return 0xFF;
}

void Cartridge::Write(uint32_t cpuAddress, uint8_t value) {
    if (const auto sramOffset = ResolveSramOffset(cpuAddress); sramOffset.has_value() && !sram_.empty()) {
        sram_[*sramOffset] = value;
    }
}

uint32_t Cartridge::AccessCycles(uint32_t cpuAddress) const noexcept {
    const bool fastAllowed = memselFast_ && header_.speed == RomSpeed::Fast;
    return (fastAllowed && IsFastRegion(cpuAddress)) ? 6u : 8u;
}

std::span<const uint8_t> Cartridge::RomData() const noexcept {
    return rom_;
}

std::span<const uint8_t> Cartridge::SramData() const noexcept {
    return sram_;
}

void Cartridge::LoadSram(std::span<const uint8_t> data) {
    if (sram_.empty()) {
        return;
    }

    const auto bytes = std::min(sram_.size(), data.size());
    std::copy_n(data.begin(), bytes, sram_.begin());
}

std::optional<RomHeader> Cartridge::ParseHeader(std::span<const uint8_t> rom) {
    std::optional<RomHeader> best;

    for (const auto base : kHeaderOffsets) {
        if (base + 0x40 > rom.size()) {
            continue;
        }

        RomHeader candidate;
        candidate.headerOffset = base;
        candidate.title = TrimTitle(rom.subspan(base, 21));

        const auto mapMode = rom[base + 0x15];
        candidate.mapping = MappingFromModeByte(mapMode);
        candidate.speed = SpeedFromModeByte(mapMode);
        candidate.cartridgeType = rom[base + 0x16];
        candidate.romSizeShift = rom[base + 0x17];
        candidate.sramSizeShift = rom[base + 0x18];
        candidate.country = rom[base + 0x19];
        candidate.version = rom[base + 0x1B];
        candidate.checksumComplement = Read16(rom, base + 0x1C);
        candidate.checksum = Read16(rom, base + 0x1E);
        candidate.resetVector = Read16(rom, base + 0x3C);

        uint32_t score = 0;
        if (candidate.mapping != MappingType::Unknown) {
            score += 4;
        }
        if (candidate.resetVector >= 0x8000) {
            score += 4;
        }
        if ((candidate.checksum ^ candidate.checksumComplement) == 0xFFFF) {
            score += 6;
        }
        if (IsMostlyPrintable(rom.subspan(base, 21))) {
            score += 2;
        }
        if (HeaderRomSize(candidate.romSizeShift) > 0) {
            score += 1;
        }

        candidate.score = score;

        if (!best.has_value() || candidate.score > best->score) {
            best = candidate;
        }
    }

    if (best.has_value() && best->score >= 4) {
        return best;
    }

    return std::nullopt;
}

std::vector<uint8_t> Cartridge::RemoveCopierHeader(std::span<const uint8_t> rom, bool* removed) {
    if (removed != nullptr) {
        *removed = false;
    }

    if (rom.size() > 512 && (rom.size() % 1024) == 512) {
        if (removed != nullptr) {
            *removed = true;
        }

        return std::vector<uint8_t>(rom.begin() + 512, rom.end());
    }

    return std::vector<uint8_t>(rom.begin(), rom.end());
}

bool Cartridge::LooksLikeInterleavedHiRom(std::span<const uint8_t> rom) {
    if (rom.size() < 0x10000 || (rom.size() % 0x8000) != 0) {
        return false;
    }

    if (rom.size() < 0xFFC0 + 0x40 || rom.size() < 0x7FC0 + 0x40) {
        return false;
    }

    const auto at7f = ParseHeader(rom.subspan(0));
    if (!at7f.has_value()) {
        return false;
    }

    const auto mapAt7f = MappingFromModeByte(rom[0x7FC0 + 0x15]);
    const auto mapAtff = MappingFromModeByte(rom[0xFFC0 + 0x15]);

    return mapAt7f == MappingType::HiRom && mapAtff != MappingType::HiRom;
}

std::vector<uint8_t> Cartridge::DeinterleaveHiRom(std::span<const uint8_t> rom) {
    const size_t half = rom.size() / 2;
    std::vector<uint8_t> out(rom.size(), 0);

    if ((rom.size() % 0x8000) != 0 || half == 0) {
        std::copy(rom.begin(), rom.end(), out.begin());
        return out;
    }

    const size_t bankCount = rom.size() / 0x8000;
    const size_t halfBanks = bankCount / 2;

    for (size_t i = 0; i < halfBanks; ++i) {
        const size_t dstA = (i * 2) * 0x8000;
        const size_t dstB = (i * 2 + 1) * 0x8000;
        const size_t srcA = (halfBanks + i) * 0x8000;
        const size_t srcB = i * 0x8000;

        std::copy_n(rom.begin() + srcA, 0x8000, out.begin() + dstA);
        std::copy_n(rom.begin() + srcB, 0x8000, out.begin() + dstB);
    }

    return out;
}

std::optional<size_t> Cartridge::ResolveRomOffset(uint32_t cpuAddress) const {
    if (rom_.empty()) {
        return std::nullopt;
    }

    const auto bank = static_cast<uint8_t>(cpuAddress >> 16);
    const auto addr = static_cast<uint16_t>(cpuAddress & 0xFFFF);

    switch (header_.mapping) {
    case MappingType::LoRom: {
        // bsnes LOROM/LOROM-RAM defaults only decode ROM in upper halves.
        // Address decoding still uses 32KB pages: (bank << 15) | (addr & 0x7FFF).
        if (addr >= 0x8000) {
            const auto linear =
                (static_cast<size_t>(bank & 0x7F) << 15) |
                static_cast<size_t>(addr & 0x7FFF);
            return MirrorOffset(linear, rom_.size());
        }

        return std::nullopt;
    }

    case MappingType::HiRom: {
        if ((bank >= 0x40 && bank <= 0x7D) || bank >= 0xC0) {
            const auto linear = (static_cast<size_t>(bank & 0x3F) * 0x10000) + addr;
            return MirrorOffset(linear, rom_.size());
        }

        if ((bank <= 0x3F || (bank >= 0x80 && bank <= 0xBF)) && addr >= 0x8000) {
            const auto linear = (static_cast<size_t>(bank & 0x3F) * 0x10000) + addr;
            return MirrorOffset(linear, rom_.size());
        }

        return std::nullopt;
    }

    case MappingType::ExHiRom: {
        size_t linear = std::numeric_limits<size_t>::max();

        if (bank >= 0xC0) {
            linear = (static_cast<size_t>(bank - 0xC0) * 0x10000) + addr;
        } else if (bank >= 0x40 && bank <= 0x7D) {
            linear = 0x400000 + (static_cast<size_t>(bank - 0x40) * 0x10000) + addr;
        } else if ((bank <= 0x3F || (bank >= 0x80 && bank <= 0xBF)) && addr >= 0x8000) {
            linear = 0x400000 + (static_cast<size_t>(bank & 0x3F) * 0x10000) + addr;
        }

        if (linear == std::numeric_limits<size_t>::max()) {
            return std::nullopt;
        }

        return MirrorOffset(linear, rom_.size());
    }

    case MappingType::Unknown:
    default:
        return std::nullopt;
    }
}

std::optional<size_t> Cartridge::ResolveSramOffset(uint32_t cpuAddress) const {
    if (sram_.empty()) {
        return std::nullopt;
    }

    const auto bank = static_cast<uint8_t>(cpuAddress >> 16);
    const auto addr = static_cast<uint16_t>(cpuAddress & 0xFFFF);

    switch (header_.mapping) {
    case MappingType::LoRom: {
        if (((bank >= 0x70 && bank <= 0x7D) || bank >= 0xF0) && addr <= 0x7FFF) {
            const auto linear = (static_cast<size_t>(bank & 0x0F) * 0x8000) + addr;
            return MirrorOffset(linear, sram_.size());
        }
        return std::nullopt;
    }

    case MappingType::HiRom:
    case MappingType::ExHiRom: {
        if (((bank >= 0x20 && bank <= 0x3F) || (bank >= 0xA0 && bank <= 0xBF)) &&
            addr >= 0x6000 && addr <= 0x7FFF) {
            const auto linear = (static_cast<size_t>(bank & 0x1F) * 0x2000) + (addr - 0x6000);
            return MirrorOffset(linear, sram_.size());
        }
        return std::nullopt;
    }

    case MappingType::Unknown:
    default:
        return std::nullopt;
    }
}

uint32_t Cartridge::ComputeCrc32(std::span<const uint8_t> data) {
    constexpr uint32_t polynomial = 0xEDB88320;

    uint32_t crc = 0xFFFFFFFF;
    for (auto byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            crc = (crc >> 1) ^ ((crc & 1) ? polynomial : 0);
        }
    }

    return ~crc;
}

} // namespace snes::core
