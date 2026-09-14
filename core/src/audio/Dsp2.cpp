// snes emulator
// core/src/audio/Dsp2.cpp
// DSP-2 bitmap and arithmetic command processing.

#include "snes/core/Dsp2.hpp"

namespace snes::core {

uint8_t Dsp2::Read() {
    return read_ < available_ ? output_[read_++] : 0xff;
}

void Dsp2::Write(uint8_t value) {
    if (needed_ == 0) {
        command_ = value;
        received_ = 0;
        payload_ = false;
        switch (command_) {
        case 0x01: needed_ = 32; break;
        case 0x03: case 0x05: case 0x06: needed_ = 1; break;
        case 0x09: needed_ = 4; break;
        case 0x0d: needed_ = 2; break;
        default: break;
        }
        return;
    }

    input_[received_++] = value;
    if (received_ != needed_) return;
    needed_ = 0;

    if (!payload_ && (command_ == 0x05 || command_ == 0x06 || command_ == 0x0d)) {
        length_ = input_[0];
        scaledLength_ = command_ == 0x0d ? input_[1] : length_;
        needed_ = command_ == 0x05 ? length_ * 2 :
                  command_ == 0x06 ? length_ : (length_ + 1) / 2;
        received_ = 0;
        payload_ = true;
        // Empty requests complete without leaving a pending payload behind.
        if (length_ == 0 || scaledLength_ == 0) {
            needed_ = 0;
            available_ = read_ = 0;
        }
        return;
    }

    Execute();
}

void Dsp2::Execute() {
    read_ = available_ = 0;
    switch (command_) {
    case 0x01:
        // Eight packed pixel rows become four SNES bitplanes.
        output_.fill(0);
        for (size_t row = 0; row < 8; ++row) {
            for (size_t x = 0; x < 8; ++x) {
                const unsigned pixel = (input_[row * 4 + x / 2] >> ((x & 1) ? 0 : 4)) & 15;
                for (size_t plane = 0; plane < 4; ++plane) {
                    const size_t offset = (plane / 2) * 16 + row * 2 + plane % 2;
                    output_[offset] |= static_cast<uint8_t>(((pixel >> plane) & 1) << (7 - x));
                }
            }
        }
        available_ = 32;
        break;
    case 0x03:
        transparent_ = input_[0] & 15;
        break;
    case 0x05:
        for (size_t i = 0; i < length_; ++i) {
            unsigned result = 0;
            for (unsigned shift : {0u, 4u}) {
                const unsigned overlay = (input_[length_ + i] >> shift) & 15;
                const unsigned pixel = overlay == transparent_ ? (input_[i] >> shift) & 15 : overlay;
                result |= pixel << shift;
            }
            output_[i] = static_cast<uint8_t>(result);
        }
        available_ = length_;
        break;
    case 0x06:
        for (size_t i = 0; i < length_; ++i) {
            const uint8_t pixels = input_[length_ - 1 - i];
            output_[i] = static_cast<uint8_t>((pixels >> 4) | (pixels << 4));
        }
        available_ = length_;
        break;
    case 0x09: {
        const uint32_t a = input_[0] | (uint32_t(input_[1]) << 8);
        const uint32_t b = input_[2] | (uint32_t(input_[3]) << 8);
        for (unsigned i = 0; i < 4; ++i) output_[i] = static_cast<uint8_t>((a * b) >> (i * 8));
        available_ = 4;
        break;
    }
    case 0x0d: {
        const uint32_t step = length_ <= scaledLength_ ? 0x10000 :
            static_cast<uint32_t>((length_ << 17) / (scaledLength_ * 2 + 1));
        for (size_t i = 0; i < scaledLength_; ++i) {
            unsigned packed = 0;
            for (unsigned half = 0; half < 2; ++half) {
                const size_t pixel = ((i * 2 + half) * step) >> 16;
                packed = (packed << 4) | ((input_[pixel / 2] >> ((pixel & 1) ? 0 : 4)) & 15);
            }
            output_[i] = static_cast<uint8_t>(packed);
        }
        available_ = scaledLength_;
        break;
    }
    default: break;
    }
}

}
