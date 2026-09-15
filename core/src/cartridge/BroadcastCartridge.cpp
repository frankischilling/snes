#include "snes/core/Cartridge.hpp"

#include <algorithm>
#include <ctime>

namespace snes::core {

void Cartridge::InitializeBroadcast(std::span<const uint8_t> pack) {
    bsx_.reset();
    header_.sramSizeShift = 5;
    sram_.assign(0x8000, 0);
    if (pack.empty()) memoryPack_.assign(0x100000, 0xff);
    else memoryPack_.assign(pack.begin(), pack.end());
    memoryPackReadOnly_ = memoryPack_[0xff00] == 'M' && memoryPack_[0xff02] == 'P' &&
                         (memoryPack_[0xff06] & 0xf0) == 0x70;
    bsx_ = std::make_unique<Bsx>(rom_, memoryPack_, sram_);
    bsx_->SetTimeSource([]() {
        BroadcastTime time;
        const std::time_t now = std::time(nullptr);
        std::tm local{};
#ifdef _WIN32
        if (localtime_s(&local, &now) != 0) return time;
#else
        if (!localtime_r(&now, &local)) return time;
#endif
        time.year = uint16_t(local.tm_year + 1900);
        time.month = uint8_t(local.tm_mon + 1); time.day = uint8_t(local.tm_mday);
        time.weekday = uint8_t(local.tm_wday + 1);
        time.hour = uint8_t(local.tm_hour); time.minute = uint8_t(local.tm_min); time.second = uint8_t(local.tm_sec);
        return time;
    });
}

bool Cartridge::LoadBroadcastStream(uint16_t channel, uint8_t sequence, std::span<const uint8_t> data) {
    return bsx_ && bsx_->LoadStream(channel, sequence, data);
}

bool Cartridge::SetBroadcastTimeSource(std::function<BroadcastTime()> source) {
    if (!bsx_) return false;
    bsx_->SetTimeSource(std::move(source));
    return true;
}

std::optional<Cartridge> Cartridge::FromBroadcastCartridge(
    std::span<const uint8_t> base, std::span<const uint8_t> pack, std::string* error) {
    auto cart = FromRomImage(base, nullptr, nullptr, error);
    if (!cart) return std::nullopt;
    const auto& header = cart->header_;
    if (header.mapping == MappingType::Bsx) {
        auto normalizedPack = RemoveCopierHeader(pack, nullptr);
        if (!normalizedPack.empty() && normalizedPack.size() != 0x100000 && normalizedPack.size() != 0x200000) {
            if (error) *error = "BS-X memory pack must be 1 or 2 MiB";
            return std::nullopt;
        }
        cart->InitializeBroadcast(normalizedPack);
        return cart;
    }
    if (header.mapping == MappingType::BroadcastSa1) {
        auto expansion = RemoveCopierHeader(pack, nullptr);
        if (!expansion.empty() && expansion.size() != 0x80000) {
            if (error) *error = "SA-1 slot expansion must be 512 KiB";
            return std::nullopt;
        }
        cart->memoryPack_ = std::move(expansion);
        cart->memoryPackReadOnly_ = true;
        cart->sa1_->SetExpansionRom(cart->memoryPack_);
        return cart;
    }
    const bool lo = header.mapping == MappingType::LoRom || header.mapping == MappingType::LoRom24Mbit;
    if ((!lo && header.mapping != MappingType::HiRom) || header.maker != 0x33 ||
        cart->rom_[header.headerOffset - 14] != 'Z' || cart->rom_[header.headerOffset - 11] == ' ' ||
        header.chip != EnhancementChip::None) {
        if (error) *error = "Expected a BS slot base cartridge without an enhancement processor";
        return std::nullopt;
    }
    auto normalizedPack = RemoveCopierHeader(pack, nullptr);
    if (!normalizedPack.empty() && normalizedPack.size() != 0x100000) {
        if (error) *error = "BS memory pack must be 1 MiB";
        return std::nullopt;
    }
    cart->broadcast24Mbit_ = header.mapping == MappingType::LoRom24Mbit;
    cart->header_.mapping = lo ? MappingType::BroadcastLoRom : MappingType::BroadcastHiRom;
    cart->memoryPack_ = std::move(normalizedPack);
    if (cart->memoryPack_.empty()) cart->memoryPack_.assign(0x100000, 0xff);
    const auto& bytes = cart->memoryPack_;
    cart->memoryPackReadOnly_ = bytes[0xff00] == 'M' && bytes[0xff02] == 'P' && (bytes[0xff06] & 0xf0) == 0x70;
    return cart;
}

std::span<const uint8_t> Cartridge::MemoryPackData() const noexcept {
    return memoryPackReadOnly_ ? std::span<const uint8_t>{} : memoryPack_;
}

bool Cartridge::LoadMemoryPack(std::span<const uint8_t> data) {
    if (memoryPackReadOnly_ || memoryPack_.empty() || data.size() != memoryPack_.size()) return false;
    std::copy(data.begin(), data.end(), memoryPack_.begin());
    flashCommand_ = 0;
    flashProgram_ = flashVendor_ = flashExtendedStatus_ = flashStatus_ = false;
    if (bsx_) bsx_->Reset();
    return true;
}

std::optional<size_t> Cartridge::ResolveMemoryPackOffset(uint32_t address) const {
    const unsigned bank = (address >> 16) & 0xff;
    const unsigned offset = address & 0xffff;
    if (memoryPack_.empty()) return std::nullopt;
    if (header_.mapping == MappingType::BroadcastLoRom) {
        if (bank < 0xc0 || bank > 0xef) return std::nullopt;
        return ((size_t(bank & 0x1f) << 15) | (offset & 0x7fff));
    }
    if (header_.mapping == MappingType::BroadcastHiRom) {
        if (bank == 0x7e || bank == 0x7f || !(bank & 0x20) || (!(bank & 0x40) && offset < 0x8000))
            return std::nullopt;
        return address & 0xfffff;
    }
    return std::nullopt;
}

bool Cartridge::SelectsFlashIo(uint32_t address) const {
    if (memoryPackReadOnly_) return false;
    const auto bank = (address >> 16) & 0xff;
    return header_.mapping == MappingType::BroadcastLoRom ? bank >= 0xc0 && bank <= 0xef : bank >= 0xe0;
}

uint8_t Cartridge::ReadMemoryPack(uint32_t address, size_t offset) const {
    if (!SelectsFlashIo(address)) return memoryPack_[offset];
    if (flashStatus_) {
        flashStatus_ = false;
        return 0x80;
    }
    const unsigned reg = address & 0xffff;
    if (flashVendor_ && reg >= 0xff00 && reg <= 0xff12 && !(reg & 1)) {
        if (reg == 0xff00) return 'M';
        if (reg == 0xff02) return 'P';
        if (reg == 0xff06) return 0x2a; // One MiB memory pack.
        return 0;
    }
    if (flashExtendedStatus_) {
        if ((reg & 0x7fff) == 2) return 0xc0;
        if ((reg & 0x7fff) == 4) return 0x82;
    }
    return memoryPack_[offset];
}

void Cartridge::WriteMemoryPack(uint32_t /*address*/, size_t offset, uint8_t value) {
    if (flashProgram_) {
        memoryPack_[offset] &= value;
        flashProgram_ = false;
        return;
    }
    const auto previous = flashCommand_;
    flashCommand_ = value;
    if (value == 0xd0 && previous == 0x20) {
        const size_t block = offset & ~size_t(0xffff);
        std::fill_n(memoryPack_.begin() + block, 0x10000, 0xff);
        return;
    }
    if (value == 0 || value == 0xff || value == 0x10 || value == 0x40 || value == 0x70 || value == 0x71) {
        flashProgram_ = value == 0x10 || value == 0x40;
        flashStatus_ = flashProgram_ || value == 0x70;
        flashExtendedStatus_ = value == 0x71;
        flashVendor_ = false;
    } else if (value == 0x50) {
        flashStatus_ = flashExtendedStatus_ = false;
    } else if (value == 0x75) {
        flashVendor_ = true;
        flashStatus_ = false;
    }
}

} // namespace snes::core
