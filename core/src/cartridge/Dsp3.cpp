#include "snes/core/Dsp3.hpp"
#include "Dsp3Tables.hpp"

#include <bit>

namespace snes::core {

bool Dsp3::Selects(uint32_t address) noexcept {
    const unsigned bank = (address >> 16) & 0x7f;
    return bank >= 0x20 && bank <= 0x3f && (address & 0x8000);
}

void Dsp3::Reset() { *this = Dsp3{}; }

void Dsp3::Idle() {
    phase_ = Phase::Command;
    data_ = 0x80;
    status_ = 0x84;
}

uint8_t Dsp3::Read(uint16_t address) {
    if (address >= 0xc000) return status_;
    if (status_ & 4) {
        const auto result = uint8_t(data_);
        Transfer();
        return result;
    }
    const bool high = (status_ & 0x10) != 0;
    status_ ^= 0x10;
    const auto result = uint8_t(data_ >> (high ? 8 : 0));
    if (high) Transfer();
    return result;
}

void Dsp3::Write(uint16_t address, uint8_t value) {
    if (address >= 0xc000) return;
    const bool byteMode = (status_ & 4) != 0;
    const bool high = !byteMode && (status_ & 0x10);
    data_ = high ? uint16_t((data_ & 255) | (unsigned(value) << 8)) :
                   uint16_t((data_ & 0xff00) | value);
    if (!byteMode) status_ ^= 0x10;
    if (byteMode || high) Transfer();
}

uint16_t Dsp3::DataRom(unsigned index) { return detail::Dsp3Rom[index & 1023]; }

uint16_t Dsp3::Linear(unsigned x, unsigned y) const {
    const auto doubled = uint16_t((unsigned(width_) * y + x) * 2);
    return uint16_t(std::bit_cast<int16_t>(doubled) >> 1);
}

void Dsp3::Transfer() {
    switch (phase_) {
    case Phase::Command:
        switch (data_) {
        case 0x02: phase_ = Phase::Coordinate; break;
        case 0x03: phase_ = Phase::Index; break;
        case 0x06: phase_ = Phase::Window; break;
        case 0x07: phase_ = Phase::Direction; return;
        case 0x0c: case 0x0f: phase_ = Phase::Zero; break;
        case 0x10: phase_ = Phase::Absorb; break;
        case 0x18: phase_ = Phase::PlanarCount; break;
        case 0x1c: phase_ = Phase::DoubleZero; break;
        case 0x1e: phase_ = Phase::SearchRange; break;
        case 0x1f: phase_ = Phase::RomStart; break;
        case 0x38: phase_ = Phase::DecodeCount; break;
        case 0x3e: phase_ = Phase::Origin; break;
        default: return;
        }
        status_ = 0x80;
        index_ = 0;
        return;
    case Phase::Finish: Idle(); return;
    case Phase::Window:
        width_ = uint8_t(data_);
        height_ = uint8_t(data_ >> 8);
        Idle();
        return;
    case Phase::Index:
        data_ = Linear(uint8_t(data_), uint8_t(data_ >> 8));
        phase_ = Phase::Finish;
        return;
    case Phase::Direction:
        moveY_ = std::bit_cast<int16_t>(DataRom(0x3b2 + data_ * 2));
        moveX_ = std::bit_cast<int16_t>(DataRom(0x3b3 + data_ * 2));
        phase_ = Phase::Move;
        status_ = 0x80;
        return;
    case Phase::Move: {
        const int x = uint8_t(data_);
        moveY_ = int16_t(moveY_ + uint8_t(data_ >> 8) + ((x & 1) ? (moveX_ & 1) : 0));
        moveX_ = int16_t(moveX_ + x);
        if (moveX_ < 0) moveX_ += width_;
        else if (moveX_ >= width_) moveX_ -= width_;
        if (moveY_ < 0) moveY_ += height_;
        else if (moveY_ >= height_) moveY_ -= height_;
        data_ = uint16_t(uint16_t(moveX_) | (unsigned(uint16_t(moveY_)) << 8) | ((moveY_ >> 8) & 255));
        phase_ = Phase::MoveIndex;
        return;
    }
    case Phase::MoveIndex:
        data_ = Linear(uint16_t(moveX_), uint16_t(moveY_));
        phase_ = Phase::Finish;
        return;
    case Phase::Coordinate:
        ++index_;
        if (index_ == 3 && data_ == 0xffff) Idle();
        if (index_ == 4) x_ = data_;
        if (index_ == 5) { y_ = data_; data_ = 1; }
        if (index_ == 6) data_ = x_;
        if (index_ == 7) { data_ = y_; index_ = 0; }
        return;
    case Phase::Zero:
        data_ = 0;
        phase_ = Phase::Finish;
        return;
    case Phase::Absorb:
        if (data_ == 0xffff) Idle();
        return;
    case Phase::DoubleZero:
        if (++index_ >= 3) data_ = 0;
        if (index_ == 4) phase_ = Phase::Finish;
        return;
    case Phase::RomStart:
        index_ = 0;
        phase_ = Phase::RomRead;
        [[fallthrough]];
    case Phase::RomRead:
        data_ = DataRom(index_++);
        if (index_ == 1024) phase_ = Phase::Finish;
        return;
    case Phase::PlanarCount:
        count_ = data_;
        index_ = 0;
        phase_ = Phase::PlanarData;
        return;
    case Phase::PlanarData:
        pixels_[index_++] = uint8_t(data_);
        pixels_[index_++] = uint8_t(data_ >> 8);
        if (index_ != 8) return;
        planes_.fill(0);
        for (unsigned x = 0; x < 8; ++x)
            for (unsigned bit = 0; bit < 8; ++bit)
                planes_[bit] |= uint8_t(((pixels_[x] >> bit) & 1) << (7 - x));
        --count_;
        index_ = 0;
        phase_ = Phase::PlanarRead;
        [[fallthrough]];
    case Phase::PlanarRead:
        if (index_ < 8) {
            data_ = uint16_t(planes_[index_] | (unsigned(planes_[index_ + 1]) << 8));
            index_ += 2;
        } else {
            index_ = 0;
            if (count_) phase_ = Phase::PlanarData;
            else Idle();
        }
        return;
    case Phase::DecodeCount:
        symbolsLeft_ = data_;
        phase_ = Phase::DecodeLength;
        return;
    case Phase::DecodeLength:
        outputsLeft_ = data_;
        index_ = symbol_ = 0;
        bitsAvailable_ = bitsNeeded_ = 0;
        decodePhase_ = DecodePhase::SymbolPrefix;
        phase_ = Phase::Decode;
        status_ = 0xc0;
        if (!symbolsLeft_ || symbolsLeft_ > symbols_.size() || !outputsLeft_) Idle();
        return;
    case Phase::Decode: Decode(); return;
    case Phase::Origin:
        originX_ = uint8_t(data_);
        originY_ = uint8_t(data_ >> 8);
        data_ = Linear(originX_, originY_);
        cells_[data_ & 8191] = {0, 255, 0};
        searchedRadius_ = returnedRadius_ = 0;
        phase_ = Phase::Finish;
        return;
    case Phase::SearchRange: StartRing(false); return;
    case Phase::SearchCell:
        status_ = 0x84;
        phase_ = Phase::Terrain;
        return;
    case Phase::Terrain:
        cells_[cell_ & 8191].terrain = uint8_t(data_);
        phase_ = Phase::Cost;
        return;
    case Phase::Cost: {
        auto& cell = cells_[cell_ & 8191];
        cell.cost = uint8_t(data_);
        cell.weight = ring_.radius == 1 && !(cell.terrain & 1) ? cell.cost : 255;
        Move(int(ring_.side + 2), ring_.x, ring_.y, true);
        --ring_.steps;
        EmitCell(false);
        return;
    }
    case Phase::Solve:
        SolvePaths();
        phase_ = Phase::ResultRange;
        return;
    case Phase::ResultRange: StartRing(true); return;
    case Phase::ResultCell:
        data_ = uint16_t(cells_[cell_ & 8191].weight);
        Move(int(ring_.side + 2), ring_.x, ring_.y, true);
        --ring_.steps;
        status_ = 0x84;
        phase_ = Phase::ResultCost;
        return;
    case Phase::ResultCost: EmitCell(true); return;
    }
}

} // namespace snes::core
