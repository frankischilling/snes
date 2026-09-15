#include "snes/core/Dsp4.hpp"

#include <algorithm>
#include <bit>

namespace snes::core {

bool Dsp4::Selects(uint32_t address) noexcept {
    const unsigned bank = (address >> 16) & 0x7f;
    return bank >= 0x30 && bank <= 0x3f && (address & 0x8000);
}

void Dsp4::Reset() { *this = Dsp4{}; }

uint8_t Dsp4::Read(uint16_t address) {
    if (address >= 0xc000) return 0x80;
    if (outputCursor_ >= output_.size()) return 0xff;
    return output_[outputCursor_++];
}

void Dsp4::Write(uint16_t address, uint8_t value) {
    if (address >= 0xc000) return;
    // A host write acknowledges an unread byte without using its value.
    if (outputCursor_ < output_.size()) { ++outputCursor_; return; }
    if (commandReady_) {
        if (!commandBytes_) {
            command_ = value;
            commandBytes_ = 1;
            return;
        }
        command_ |= uint16_t(unsigned(value) << 8);
        commandBytes_ = 0;
        ClearOutput();
        size_t length;
        switch (command_) {
        case 0x00: length = 4; break;
        case 0x01: length = 44; break;
        case 0x03: case 0x05: case 0x06: case 0x0e: length = 0; break;
        case 0x07: length = 34; break;
        case 0x08: length = 90; break;
        case 0x09: length = 14; break;
        case 0x0a: case 0x0b: length = 6; break;
        case 0x0d: length = 42; break;
        case 0x0f: length = 46; break;
        case 0x10: length = 36; break;
        case 0x11: length = 8; break;
        default: return;
        }
        Need(length, Stage::Start);
    } else {
        input_[received_++] = value;
    }
    if (received_ != needed_) return;
    cursor_ = 0;
    commandReady_ = true;
    Execute();
}

void Dsp4::Need(size_t bytes, Stage stage) {
    needed_ = bytes;
    received_ = 0;
    stage_ = stage;
    commandReady_ = false;
}

void Dsp4::Finish() {
    commandReady_ = true;
    commandBytes_ = 0;
    received_ = needed_ = 0;
}

int16_t Dsp4::Wrap16(int64_t value) { return std::bit_cast<int16_t>(uint16_t(value)); }
int32_t Dsp4::Wrap32(int64_t value) { return std::bit_cast<int32_t>(uint32_t(value)); }
int32_t Dsp4::Fixed(int value) { return int32_t(Wrap16(value)) * 65536; }
int16_t Dsp4::Project(int value, int16_t distance) { return Wrap16((int64_t(value) * distance) >> 15); }

int32_t Dsp4::Slope(int delta, int16_t lines) {
    const int divisor = std::clamp<int>(lines, 0, 63);
    const int reciprocal = divisor ? Wrap16(32768 / divisor) : 0;
    return Wrap32(int64_t(delta) * reciprocal * 2);
}

int16_t Dsp4::Word() {
    const uint16_t value = uint16_t(input_[cursor_] | (unsigned(input_[cursor_ + 1]) << 8));
    cursor_ += 2;
    return std::bit_cast<int16_t>(value);
}

int32_t Dsp4::Long() {
    const uint32_t low = uint16_t(Word());
    const uint32_t high = uint16_t(Word());
    return std::bit_cast<int32_t>(low | (high << 16));
}

void Dsp4::ByteOut(uint8_t value) { output_.push_back(value); }
void Dsp4::WordOut(int value) {
    ByteOut(uint8_t(value));
    ByteOut(uint8_t(unsigned(value) >> 8));
}
void Dsp4::ClearOutput() { output_.clear(); outputCursor_ = 0; }

void Dsp4::Execute() {
    switch (command_) {
    case 0x00: {
        const int32_t a = Word(), b = Word();
        // The multiplier's result is a signed 31-bit quantity.
        const int32_t product = Wrap32(int64_t(a) * b * 2) >> 1;
        ClearOutput();
        WordOut(product);
        WordOut(product >> 16);
        return;
    }
    case 0x01: case 0x07: case 0x0d: case 0x0f: case 0x10:
        RoadCommand();
        return;
    case 0x03: case 0x0e:
        rowLimit_ = command_ == 3 ? 33 : 16;
        rows_.fill(0);
        return;
    case 0x05:
        objectCount_ = 0;
        attributes_.fill(0);
        return;
    case 0x06:
        ClearOutput();
        for (const auto word : attributes_) WordOut(word);
        return;
    case 0x08: WindowCommand(); return;
    case 0x09: ObjectCommand(); return;
    case 0x0a: {
        Word();
        const auto packed = uint16_t(Word());
        ClearOutput();
        for (unsigned nibble : {2u, 3u, 0u, 1u}) {
            const int value = (packed >> (nibble * 4)) & 15;
            WordOut((value >= 8 ? value - 16 : value) * 48);
        }
        return;
    }
    case 0x0b: {
        const int16_t x = Word(), y = Word(), attr = Word();
        ClearOutput();
        bool draw = true;
        EmitObject(draw, x, y, attr, false, true);
        return;
    }
    case 0x11: {
        unsigned packed = 0;
        for (unsigned nibble = 0; nibble < 4; ++nibble)
            packed |= unsigned((int32_t(Word()) * 341 >> 14) & 15) << (nibble * 4);
        ClearOutput();
        WordOut(int(packed));
        return;
    }
    default: Finish(); return;
    }
}

} // namespace snes::core
