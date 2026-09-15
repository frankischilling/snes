#include "snes/core/Bsx.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <map>
#include <utility>
#include <vector>

namespace snes::core {
namespace {
size_t Fold(size_t address, size_t size) {
    if (!size || address < size) return size ? address : 0;
    const size_t top = std::bit_floor(address);
    return size > top ? top + Fold(address - top, size - top) : Fold(address - top, size);
}
}

struct Bsx::Impl {
    enum class Region { Open, Mapper, Radio, Bios, Pack, Psram, Sram };
    struct Location { Region kind = Region::Open; size_t offset = 0; };
    using StreamKey = std::pair<uint16_t, uint8_t>;
    struct Receiver {
        bool prefixEnabled = false, dataEnabled = false, loaded = false, first = false;
        uint8_t nextSequence = 0;
        StreamKey asset{};
        size_t cursor = 0, remaining = 0;
    };

    std::span<const uint8_t> bios;
    std::span<uint8_t> pack, sram;
    std::vector<uint8_t> psram = std::vector<uint8_t>(0x80000);
    std::array<uint8_t, 16> pending{}, active{};
    uint8_t hidden = 0x3f;
    bool ready = false;
    std::array<uint8_t, 18> radio{};
    std::array<Receiver, 2> receivers{};
    std::map<StreamKey, std::vector<uint8_t>> assets;
    std::function<BroadcastTime()> timeSource;
    std::array<uint8_t, 23> clockRecord{};
    unsigned clockIndex = 0;
    uint8_t lastCommand = 0;
    bool programNext = false, compatibleStatus = false, extendedStatus = false, vendor = false;
    bool readOnly = false;

    Impl(std::span<const uint8_t> image, std::span<uint8_t> cartridge, std::span<uint8_t> save)
        : bios(image), pack(cartridge), sram(save) {
        Reset();
    }
    void Reset() {
        readOnly = pack.size() > 0xff06 && pack[0xff00] == 'M' && pack[0xff02] == 'P' && (pack[0xff06] & 0xf0) == 0x70;
        pending = {};
        for (unsigned bit : {2u, 3u, 5u, 6u, 7u, 8u, 9u, 11u}) pending[bit] = 0x80;
        active = pending;
        ready = false;
        radio = {}; radio[0x0e] = 0x10; radio[0x0f] = 0x80;
        receivers = {};
        clockIndex = 0;
        clockRecord = {};
        lastCommand = 0;
        programNext = compatibleStatus = extendedStatus = vendor = false;
    }

    Location Locate(uint32_t address) const {
        const unsigned bank = address >> 16;
        const unsigned halfBank = bank & 0x7f;
        const unsigned offset = address & 0xffff;
        if (bank == 0x7e || bank == 0x7f) return {};
        if (bank < 16 && (offset & 0xf000) == 0x5000) return {Region::Mapper, bank};
        if (bank >= 0x10 && bank <= 0x17 && (offset & 0xf000) == 0x5000)
            return {Region::Sram, ((bank & 7) << 12) | (offset & 0xfff)};
        if (halfBank < 0x40 && offset >= 0x2188 && offset <= 0x2199) return {Region::Radio, offset - 0x2188};

        const bool high = active[2] != 0;
        const bool ramEnabled = active[bank < 0x80 ? 3 : 4] != 0;
        if (halfBank < 0x40 && offset < 0x8000) {
            if (high && ramEnabled && halfBank >= 0x20 && offset >= 0x6000)
                return {Region::Psram, ((bank & 7) << 16) | offset};
            return {};
        }
        if (halfBank < 0x40 && active[bank < 0x80 ? 7 : 8])
            return {Region::Bios, ((bank & 0x1f) << 15) | (offset & 0x7fff)};

        if (ramEnabled) {
            const unsigned position = unsigned(active[5] != 0) + 2 * unsigned(active[6] != 0);
            if (high) {
                if ((bank & 0x3f) >= position * 16 && (bank & 0x3f) < position * 16 + 8)
                    return {Region::Psram, ((bank & 7) << 16) | offset};
            } else if ((halfBank >= position * 32 && halfBank < position * 32 + 16) ||
                       (halfBank >= 0x70 && offset < 0x8000)) {
                return {Region::Psram, ((bank & 15) << 15) | (offset & 0x7fff)};
            }
        }
        if (active[bank < 0x80 ? 9 : 10]) {
            const unsigned position = unsigned(active[11] != 0);
            const bool hole = high ? (bank & 0x3f) >= position * 32 && (bank & 0x3f) < position * 32 + 16
                                   : halfBank >= position * 64 && halfBank < position * 64 + 32;
            if (hole) return {};
        }
        const size_t packOffset = high ? address & 0x3fffff : ((bank & 0x3f) << 15) | (offset & 0x7fff);
        return {Region::Pack, packOffset};
    }

    uint8_t ReadMapper(unsigned index, uint8_t openBus) const {
        bool bit = false;
        if (pending[15]) bit = (hidden & (1u << (index & 7))) != 0;
        else if (index == 0) bit = ready;
        else if (index == 1) bit = pending[1] != 0;
        else if (index < 14) bit = active[index] != 0;
        return uint8_t((openBus & 0x7f) | (bit ? 0x80 : 0));
    }
    void WriteMapper(unsigned index, uint8_t value) {
        const uint8_t bit = value & 0x80;
        if (pending[15]) hidden = uint8_t((hidden & ~(1u << (index & 7))) | ((bit ? 1u : 0u) << (index & 7)));
        pending[index] = bit;
        if (index == 0) ready = false;
        if (index == 14) for (unsigned reg = 2; reg <= 13; ++reg) active[reg] = pending[reg];
    }

    uint8_t ReadPack(uint32_t address, size_t offset, uint8_t openBus) {
        if (pack.empty()) return openBus;
        const auto value = pack[Fold(offset, pack.size())];
        if (readOnly) return value;
        if (compatibleStatus) { compatibleStatus = false; return 0x80; }
        const unsigned low = address & 0xffff;
        if (extendedStatus) {
            if ((low & 0x7fff) == 2) return 0xc0;
            if ((low & 0x7fff) == 4) return 0x82;
        }
        if (vendor && low >= 0xff00 && low <= 0xff12 && !(low & 1)) {
            if (low == 0xff00) return 'M';
            if (low == 0xff02) return 'P';
            if (low == 0xff06) return pack.size() > 0x100000 ? 0x1a : 0x2a;
            return 0;
        }
        return value;
    }
    void WritePack(size_t offset, uint8_t value) {
        if (pack.empty() || readOnly || !active[12]) return;
        offset = Fold(offset, pack.size());
        if (programNext) {
            pack[offset] &= value;
            programNext = false;
            ready = true;
            return;
        }
        const uint8_t previous = std::exchange(lastCommand, value);
        if (value == 0 || value == 0xff || value == 0x10 || value == 0x40 || value == 0x70 || value == 0x71) {
            programNext = value == 0x10 || value == 0x40;
            compatibleStatus = programNext || value == 0x70;
            extendedStatus = value == 0x71;
            vendor = false;
        } else if (value == 0x50) compatibleStatus = extendedStatus = false;
        else if (value == 0x75) { vendor = true; compatibleStatus = false; }
        else if (value == 0xd0 && previous == 0x20) {
            const size_t start = offset & ~size_t(0xffff);
            std::fill(pack.begin() + start, pack.begin() + std::min(start + 0x10000, pack.size()), 0xff);
            ready = true;
        } else if (value == 0xd0 && previous == 0xa7 && pack.size() > 0x100000) {
            std::fill(pack.begin(), pack.end(), 0xff);
            ready = true;
        }
    }

    uint16_t Channel(unsigned receiver) const {
        const unsigned base = receiver * 6;
        return uint16_t(radio[base] | (uint16_t(radio[base + 1]) << 8));
    }
    void SelectAsset(unsigned receiver) {
        auto& state = receivers[receiver];
        auto key = StreamKey{Channel(receiver), state.nextSequence++};
        auto asset = assets.find(key);
        if (asset == assets.end() && key.second != 0) {
            key.second = 0;
            state.nextSequence = 1;
            asset = assets.find(key);
        }
        state.loaded = asset != assets.end();
        state.remaining = 0;
        state.cursor = 0;
        state.first = false;
        radio[receiver * 6 + 3] = 0;
        if (!state.loaded) return;
        state.asset = key;
        state.remaining = (asset->second.size() + 21) / 22;
        state.first = true;
        radio[receiver * 6 + 5] = 0;
    }
    uint8_t ClockByte() {
        if (clockIndex == 0) {
            const BroadcastTime time = timeSource ? timeSource() : BroadcastTime{};
            clockRecord = {};
            clockRecord[4] = 0x10;
            clockRecord[5] = clockRecord[6] = 1;
            clockRecord[10] = time.second; clockRecord[11] = time.minute; clockRecord[12] = time.hour;
            clockRecord[13] = time.weekday; clockRecord[14] = time.day; clockRecord[15] = time.month;
            clockRecord[16] = uint8_t(time.year); clockRecord[17] = uint8_t(time.year >> 8);
        }
        const uint8_t value = clockRecord[clockIndex];
        clockIndex = (clockIndex + 1) % unsigned(clockRecord.size());
        return value;
    }
    uint8_t ReadRadio(unsigned reg) {
        if (reg >= 12) return radio[reg];
        const unsigned receiver = reg / 6, field = reg % 6, base = receiver * 6;
        auto& state = receivers[receiver];
        const bool clock = Channel(receiver) == 0;
        switch (field) {
        case 0: case 1: return radio[reg];
        case 2:
            if (!state.prefixEnabled || !state.dataEnabled) return 0;
            if (clock) return 1;
            if (!state.remaining) SelectAsset(receiver);
            return uint8_t(std::min(state.remaining, size_t(127)));
        case 3:
            if (!state.prefixEnabled) return 0;
            if (clock) radio[reg] = 0x90;
            else {
                if (!state.loaded || !state.remaining) return 0;
                radio[reg] = uint8_t((state.first ? 0x10 : 0) | (state.remaining == 1 ? 0x80 : 0));
                state.first = false;
                --state.remaining;
            }
            radio[base + 5] |= radio[reg];
            return radio[reg];
        case 4: {
            if (!state.dataEnabled) return 0;
            if (clock) return ClockByte();
            if (!state.loaded) return 0;
            const auto asset = assets.find(state.asset);
            if (asset == assets.end() || state.cursor >= asset->second.size()) return 0xff;
            return asset->second[state.cursor++];
        }
        case 5: return std::exchange(radio[reg], 0);
        }
        return 0;
    }
    void WriteRadio(unsigned reg, uint8_t value) {
        if (reg >= 12) {
            if (reg == 12) radio[reg] = value & 15;
            if (reg == 15) radio[reg] = value;
            return;
        }
        const unsigned receiver = reg / 6, field = reg % 6;
        auto& state = receivers[receiver];
        switch (field) {
        case 0: case 1:
            radio[reg] = field == 1 ? value & 0x3f : value;
            state.loaded = false;
            state.cursor = state.remaining = state.nextSequence = 0;
            radio[receiver * 6 + 3] = radio[receiver * 6 + 5] = 0;
            break;
        case 3: state.prefixEnabled = value != 0; break;
        case 4:
            state.dataEnabled = value != 0;
            if (Channel(receiver) == 0) clockIndex = 0;
            break;
        default: break;
        }
    }
};

Bsx::Bsx(std::span<const uint8_t> bios, std::span<uint8_t> pack, std::span<uint8_t> sram)
    : impl_(std::make_unique<Impl>(bios, pack, sram)) {}
Bsx::~Bsx() = default;
Bsx::Bsx(Bsx&&) noexcept = default;
Bsx& Bsx::operator=(Bsx&&) noexcept = default;
void Bsx::Reset() { impl_->Reset(); }
bool Bsx::CpuIrqPending() const noexcept { return impl_->ready && impl_->pending[1]; }
std::span<uint8_t> Bsx::Psram() noexcept { return impl_->psram; }

bool Bsx::Selects(uint32_t address) noexcept {
    address &= 0xffffff;
    const unsigned bank = address >> 16, halfBank = bank & 0x7f, offset = address & 0xffff;
    if (bank == 0x7e || bank == 0x7f) return false;
    if (halfBank >= 0x40) return true;
    if (offset >= 0x8000 || (offset >= 0x2188 && offset <= 0x2199)) return true;
    if (bank < 0x18 && (offset & 0xf000) == 0x5000) return true;
    return halfBank >= 0x20 && offset >= 0x6000;
}

uint8_t Bsx::ReadCpu(uint32_t address, uint8_t openBus) {
    address &= 0xffffff;
    const auto target = impl_->Locate(address);
    switch (target.kind) {
    case Impl::Region::Mapper: return impl_->ReadMapper(unsigned(target.offset), openBus);
    case Impl::Region::Radio: return impl_->ReadRadio(unsigned(target.offset));
    case Impl::Region::Bios: return impl_->bios.empty() ? openBus : impl_->bios[Fold(target.offset, impl_->bios.size())];
    case Impl::Region::Psram: return impl_->psram[target.offset];
    case Impl::Region::Sram: return impl_->sram.empty() ? openBus : impl_->sram[Fold(target.offset, impl_->sram.size())];
    case Impl::Region::Pack: return impl_->ReadPack(address, target.offset, openBus);
    default: return openBus;
    }
}

void Bsx::WriteCpu(uint32_t address, uint8_t value) {
    address &= 0xffffff;
    const auto target = impl_->Locate(address);
    switch (target.kind) {
    case Impl::Region::Mapper: impl_->WriteMapper(unsigned(target.offset), value); break;
    case Impl::Region::Radio: impl_->WriteRadio(unsigned(target.offset), value); break;
    case Impl::Region::Psram: impl_->psram[target.offset] = value; break;
    case Impl::Region::Sram:
        if (!impl_->sram.empty()) impl_->sram[Fold(target.offset, impl_->sram.size())] = value;
        break;
    case Impl::Region::Pack: impl_->WritePack(target.offset, value); break;
    default: break;
    }
}

bool Bsx::LoadStream(uint16_t channel, uint8_t sequence, std::span<const uint8_t> payload) {
    if (!channel || channel > 0x3fff || payload.size() > size_t(22) * 65535) return false;
    const auto key = Impl::StreamKey{channel, sequence};
    if (payload.empty()) impl_->assets.erase(key);
    else impl_->assets[key] = std::vector<uint8_t>(payload.begin(), payload.end());
    for (auto& receiver : impl_->receivers) {
        if (receiver.loaded && receiver.asset == key) {
            receiver.loaded = false;
            receiver.remaining = receiver.cursor = 0;
            receiver.nextSequence = sequence;
        }
    }
    return true;
}

void Bsx::ClearStreams() {
    impl_->assets.clear();
    for (auto& receiver : impl_->receivers) {
        receiver.loaded = false;
        receiver.cursor = receiver.remaining = receiver.nextSequence = 0;
    }
    impl_->radio[3] = impl_->radio[5] = impl_->radio[9] = impl_->radio[11] = 0;
}

void Bsx::SetTimeSource(std::function<BroadcastTime()> source) { impl_->timeSource = std::move(source); }

} // namespace snes::core
