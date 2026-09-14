#include "snes/core/Cartridge.hpp"

#include "snes/core/Logging.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <limits>
#include <string_view>

namespace snes::core {

namespace {

constexpr std::array<size_t, 4> kHeaderOffsets{0x7FC0, 0xFFC0, 0x407FC0, 0x40FFC0};

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

    // Fold unconnected address lines while retaining each populated block.
    // For example, a 3 MiB image repeats its last MiB at offsets 3..4 MiB.
    size_t base = 0;
    while (offset >= size) {
        const size_t block = std::bit_floor(offset);
        offset -= block;
        if (size > block) {
            base += block;
            size -= block;
        }
    }
    return base + offset;
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

EnhancementChip DetectChip(const RomHeader& header) {
    const auto mode = header.mapMode;
    const auto type = header.cartridgeType;
    if (type == 3) return mode == 0x30 ? EnhancementChip::Dsp4 : EnhancementChip::Dsp1;
    if (type == 5) {
        if (mode == 0x20) return EnhancementChip::Dsp2;
        if (mode == 0x30 && header.maker == 0xb2) return EnhancementChip::Dsp3;
        return EnhancementChip::Dsp1;
    }
    switch ((unsigned(type) << 8) | mode) {
    case 0x2530: return EnhancementChip::Obc1;
    case 0x5535: return EnhancementChip::Srtc;
    case 0x1320: case 0x1420: case 0x1520: case 0x1a20:
    case 0x1330: case 0x1430: case 0x1530: case 0x1a30: return EnhancementChip::SuperFx;
    case 0x3423: case 0x3523: return EnhancementChip::Sa1;
    case 0x4332: case 0x4532: return EnhancementChip::Sdd1;
    case 0xf53a: return EnhancementChip::Spc7110;
    case 0xf93a: return EnhancementChip::Spc7110Rtc;
    case 0xf320: return EnhancementChip::Cx4;
    case 0xf530: return EnhancementChip::St018;
    case 0xf630: return header.romSizeShift == 9 ? EnhancementChip::St011 : EnhancementChip::St010;
    default: return EnhancementChip::None;
    }
}

bool HasSignature(std::span<const uint8_t> bytes, size_t offset, std::string_view signature) {
    return offset <= bytes.size() && signature.size() <= bytes.size() - offset &&
        std::equal(signature.begin(), signature.end(), bytes.begin() + offset);
}

} // namespace

const char* MappingName(MappingType mapping) noexcept {
    switch (mapping) {
    case MappingType::LoRom: return "LoROM";
    case MappingType::HiRom: return "HiROM";
    case MappingType::ExLoRom: return "ExLoROM";
    case MappingType::ExHiRom: return "ExHiROM";
    case MappingType::LoRomNoMad1: return "LoROM (NoMAD-1)";
    case MappingType::LoRom24Mbit: return "LoROM (24 Mbit board)";
    case MappingType::LoRomLargeSram: return "LoROM (large SRAM board)";
    case MappingType::SufamiTurbo: return "Sufami Turbo";
    default: return "Unknown";
    }
}

const char* ChipName(EnhancementChip chip) noexcept {
    switch (chip) {
    case EnhancementChip::None: return "None";
    case EnhancementChip::Dsp1: return "DSP-1";
    case EnhancementChip::Dsp2: return "DSP-2";
    case EnhancementChip::Dsp3: return "DSP-3";
    case EnhancementChip::Dsp4: return "DSP-4";
    case EnhancementChip::Obc1: return "OBC1";
    case EnhancementChip::Srtc: return "S-RTC";
    case EnhancementChip::SuperFx: return "Super FX";
    case EnhancementChip::Sa1: return "SA-1";
    case EnhancementChip::Sdd1: return "S-DD1";
    case EnhancementChip::Spc7110: return "SPC7110";
    case EnhancementChip::Spc7110Rtc: return "SPC7110 + RTC";
    case EnhancementChip::Cx4: return "Cx4";
    case EnhancementChip::St010: return "ST010";
    case EnhancementChip::St011: return "ST011";
    case EnhancementChip::St018: return "ST018";
    }
    return "Unknown";
}

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
    if (error) error->clear();
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

    if (cart.header_.mapping == MappingType::LoRom) {
        const auto& title = cart.header_.title;
        if (title == "WANDERERS FROM YS") cart.header_.mapping = MappingType::LoRomNoMad1;
        else if (title == "SOUND NOVEL-TCOOL" || title == "DERBY STALLION 96")
            cart.header_.mapping = MappingType::LoRom24Mbit;
        else if (title == "THOROUGHBRED BREEDER3" || title == "RPG-TCOOL 2")
            cart.header_.mapping = MappingType::LoRomLargeSram;
    }

    std::optional<size_t> forcedSramSize;
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
                forcedSramSize = *overrideEntry->forceSramSize;
            }
        }
    }

    cart.sram_.assign(forcedSramSize.value_or(HeaderSramSize(cart.header_.sramSizeShift)), 0x00);

    if (normalization != nullptr) {
        normalization->hadCopierHeader = removedHeader;
        normalization->hadInterleave = deinterleaved;
    }

    return cart;
}

std::optional<Cartridge> Cartridge::FromSufamiTurbo(
    std::span<const uint8_t> bios, std::span<const uint8_t> slotA,
    std::span<const uint8_t> slotB, std::string* error) {
    if (error) error->clear();
    const auto normalizedBios = RemoveCopierHeader(bios, nullptr);
    const std::array slots{RemoveCopierHeader(slotA, nullptr), RemoveCopierHeader(slotB, nullptr)};
    if (normalizedBios.size() != 0x40000 || !HasSignature(normalizedBios, 0, "BANDAI SFC-ADX") ||
        !HasSignature(normalizedBios, 0x10, "SFC-ADX BACKUP")) {
        if (error) *error = "Invalid Sufami Turbo BIOS (expected a 256 KiB base cartridge)";
        return std::nullopt;
    }
    for (const auto& slot : slots) {
        if (!slot.empty() && (slot.size() < 0x80000 || slot.size() > 0x100000 ||
            !HasSignature(slot, 0, "BANDAI SFC-ADX") || HasSignature(slot, 0x10, "SFC-ADX BACKUP"))) {
            if (error) *error = "Invalid Sufami Turbo slot image (expected a 512 KiB to 1 MiB game)";
            return std::nullopt;
        }
    }
    auto cart = FromRomImage(normalizedBios, nullptr, nullptr, error);
    if (!cart) return std::nullopt;
    cart->header_.mapping = MappingType::SufamiTurbo;
    cart->header_.chip = EnhancementChip::None;
    // A stable 16 KiB region per slot keeps saves independent of slot occupancy.
    cart->sram_.assign(0x8000, 0);
    for (unsigned i = 0; i < 2; ++i) {
        cart->slotOffset_[i] = cart->rom_.size();
        cart->slotSize_[i] = slots[i].size();
        cart->rom_.insert(cart->rom_.end(), slots[i].begin(), slots[i].end());
    }
    cart->romCrc32_ = ComputeCrc32(cart->rom_);
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

uint8_t Cartridge::Read(uint32_t cpuAddress, uint8_t openBus) const {
    if (const auto sramOffset = ResolveSramOffset(cpuAddress); sramOffset.has_value() && !sram_.empty()) {
        return sram_[*sramOffset];
    }

    if (const auto romOffset = ResolveRomOffset(cpuAddress); romOffset.has_value() && !rom_.empty()) {
        return rom_[*romOffset];
    }

    return openBus;
}

void Cartridge::Write(uint32_t cpuAddress, uint8_t value) {
    if (const auto sramOffset = ResolveSramOffset(cpuAddress); sramOffset.has_value() && !sram_.empty()) {
        sram_[*sramOffset] = value;
    }
}

uint32_t Cartridge::AccessCycles(uint32_t cpuAddress) const noexcept {
    return (memselFast_ && IsFastRegion(cpuAddress)) ? 6u : 8u;
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

std::span<const uint8_t> Cartridge::SlotSramData(unsigned slot) const noexcept {
    if (header_.mapping != MappingType::SufamiTurbo || slot >= 2 || slotSize_[slot] == 0) return {};
    return std::span<const uint8_t>(sram_).subspan(slot * 0x4000, 0x4000);
}

void Cartridge::LoadSlotSram(unsigned slot, std::span<const uint8_t> data) {
    const auto size = SlotSramData(slot).size();
    if (size) std::copy_n(data.begin(), std::min(data.size(), size), sram_.begin() + slot * 0x4000);
}

std::optional<RomHeader> Cartridge::ParseHeader(std::span<const uint8_t> rom) {
    std::optional<RomHeader> best;

    for (const auto base : kHeaderOffsets) {
        if (base + 0x40 > rom.size()) {
            continue;
        }
        const auto bytes = rom.subspan(base, 64);
        if (std::all_of(bytes.begin(), bytes.end(), [](uint8_t byte) { return byte == 0xff; })) continue;

        RomHeader candidate;
        candidate.headerOffset = base;
        candidate.title = TrimTitle(rom.subspan(base, 21));

        const auto mapMode = rom[base + 0x15];
        candidate.mapMode = mapMode;
        candidate.maker = rom[base + 0x1a];
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
        candidate.chip = DetectChip(candidate);

        // A bootable cartridge must supply a reset vector in cartridge space.
        if (candidate.resetVector < 0x8000) continue;

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

        // Prototype headers may omit the mode byte. The header's physical
        // position still identifies the layout used by the reset vector.
        if (candidate.mapping == MappingType::Unknown) {
            candidate.mapping = base == 0x7fc0 ? MappingType::LoRom :
                                base == 0xffc0 ? MappingType::HiRom :
                                base == 0x407fc0 ? MappingType::ExLoRom : MappingType::ExHiRom;
            candidate.speed = RomSpeed::Slow;
        }
        if (base == 0x407fc0 && candidate.mapping == MappingType::LoRom)
            candidate.mapping = MappingType::ExLoRom;
        if (base == 0x40ffc0 && candidate.mapping == MappingType::HiRom)
            candidate.mapping = MappingType::ExHiRom;

        if (!best.has_value() || candidate.score > best->score ||
            (base >= 0x400000 && candidate.score == best->score)) {
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
    if (rom.size() < 0x10000 || (rom.size() % 0x10000) != 0) {
        return false;
    }

    if (rom.size() < 0xFFC0 + 0x40 || rom.size() < 0x7FC0 + 0x40) {
        return false;
    }

    const auto at7f = ParseHeader(rom.subspan(0));
    if (!at7f.has_value() || at7f->headerOffset != 0x7fc0) {
        return false;
    }

    const auto mapAt7f = MappingFromModeByte(rom[0x7FC0 + 0x15]);
    const auto mapAtff = MappingFromModeByte(rom[0xFFC0 + 0x15]);

    return mapAt7f == MappingType::HiRom && mapAtff != MappingType::HiRom;
}

std::vector<uint8_t> Cartridge::DeinterleaveHiRom(std::span<const uint8_t> rom) {
    const size_t half = rom.size() / 2;
    std::vector<uint8_t> out(rom.size(), 0);

    if ((rom.size() % 0x10000) != 0 || half == 0) {
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

    if (SelectsLoRomSram(cpuAddress)) return std::nullopt;

    switch (header_.mapping) {
    case MappingType::SufamiTurbo: {
        if (addr < 0x8000 || (bank & 0x7f) >= 0x60) return std::nullopt;
        const unsigned window = (bank & 0x7f) / 0x20;
        const size_t linear = (size_t(bank & 0x1f) << 15) | (addr & 0x7fff);
        if (window == 0) return MirrorOffset(linear, std::min(rom_.size(), size_t(0x40000)));
        const unsigned slot = window - 1;
        if (slotSize_[slot] == 0) return std::nullopt;
        return slotOffset_[slot] + MirrorOffset(linear, slotSize_[slot]);
    }
    case MappingType::ExLoRom: {
        if (bank == 0x7e || bank == 0x7f || (!(bank & 0x40) && addr < 0x8000))
            return std::nullopt;
        const size_t linear = (static_cast<size_t>(bank & 0x7f) << 15) | (addr & 0x7fff);
        if (bank & 0x80) return MirrorOffset(linear, std::min(rom_.size(), size_t(0x400000)));
        if (rom_.size() <= 0x400000) return MirrorOffset(linear, rom_.size());
        return 0x400000 + MirrorOffset(linear, rom_.size() - 0x400000);
    }

    case MappingType::LoRom24Mbit: {
        if ((bank & 0x40) || addr < 0x8000) return std::nullopt;
        const size_t chip = (bank & 0x20) ? 0x100000 : (bank & 0x80) ? 0x200000 : 0;
        const size_t linear = chip + (static_cast<size_t>(bank & 0x1f) << 15) + (addr & 0x7fff);
        return MirrorOffset(linear, rom_.size());
    }

    case MappingType::LoRomNoMad1:
    case MappingType::LoRomLargeSram:
    case MappingType::LoRom: {
        // Mirror each 32 KiB page into both halves of the full-ROM banks.
        const bool fullRomBank = (bank >= 0x40 && bank <= 0x7D) || bank >= 0xC0;
        if (fullRomBank || (addr >= 0x8000 && bank != 0x7E && bank != 0x7F)) {
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
            const size_t half = (bank & 0x80) ? 0 : 0x400000;
            linear = half + (static_cast<size_t>(bank & 0x3F) * 0x10000) + addr;
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

bool Cartridge::SelectsLoRomSram(uint32_t cpuAddress) const {
    switch (header_.mapping) {
    case MappingType::LoRom: case MappingType::ExLoRom:
    case MappingType::LoRomNoMad1: case MappingType::LoRom24Mbit: break;
    default: return false;
    }
    const unsigned bank = (cpuAddress >> 16) & 0xff;
    if (!((bank >= 0x70 && bank <= 0x7d) || (bank >= 0xf0 && !sram_.empty()))) return false;
    return (cpuAddress & 0xffff) < 0x8000 || header_.mapping == MappingType::LoRomNoMad1 ||
        (header_.romSizeShift <= 11 && sram_.size() <= 0x8000);
}

std::optional<size_t> Cartridge::ResolveSramOffset(uint32_t cpuAddress) const {
    if (sram_.empty()) {
        return std::nullopt;
    }

    const auto bank = static_cast<uint8_t>(cpuAddress >> 16);
    const auto addr = static_cast<uint16_t>(cpuAddress & 0xFFFF);

    switch (header_.mapping) {
    case MappingType::SufamiTurbo: {
        const unsigned mirroredBank = bank & 0x7f;
        if (addr < 0x8000 || !((mirroredBank >= 0x60 && mirroredBank <= 0x63) ||
            (mirroredBank >= 0x70 && mirroredBank <= 0x73))) return std::nullopt;
        const unsigned slot = mirroredBank >= 0x70 ? 1 : 0;
        if (slotSize_[slot] == 0) return std::nullopt;
        return slot * 0x4000 + (addr & 0x3fff);
    }
    case MappingType::LoRomLargeSram:
        if (bank >= 0x70 && bank <= 0x73) {
            const size_t linear = static_cast<size_t>(bank - 0x70) * 0x8000 + addr;
            return MirrorOffset(linear, sram_.size());
        }
        return std::nullopt;
    case MappingType::ExLoRom:
    case MappingType::LoRomNoMad1:
    case MappingType::LoRom24Mbit:
    case MappingType::LoRom: {
        if (SelectsLoRomSram(cpuAddress)) {
            const auto linear = (static_cast<size_t>(bank & 0x0F) * 0x8000) + (addr & 0x7fff);
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
