// snes emulator
// SA-1 timer, arithmetic unit, and variable-width bit stream.
#include "snes/core/Sa1.hpp"

namespace snes::core {

void Sa1::AdvanceTimer(uint32_t masterClocks) {
    const uint8_t control = registers_[0x10];
    const uint32_t width = (control & 0x80) ? 512 : 341;
    const uint32_t height = (control & 0x80) ? 512 : videoLines_;
    const uint32_t hTarget = RegisterWord(0x12) & 0x1ff;
    const uint32_t vTarget = RegisterWord(0x14) & 0x1ff;
    const uint32_t oldDot = hClocks_ / 4;
    const uint32_t position = vCounter_ * width + oldDot;
    const uint64_t fractional = uint64_t(hClocks_ & 3) + masterClocks;
    const uint64_t dots = fractional / 4;
    const auto matches = [&](uint32_t h, uint32_t v) {
        return (control & 3) && (!(control & 1) || h == hTarget) &&
               (!(control & 2) || v == vTarget);
    };

    // Find an entry into the comparator's matching interval, even when one
    // scheduler call crosses several lines or frames. Acknowledging a V-only
    // interrupt in the matching line must not immediately reassert it.
    if (masterClocks && !timerMatch_ && matches(oldDot, vCounter_)) sa1Flags_ |= 0x40;
    if ((control & 3) && (!(control & 1) || hTarget < width) &&
        (!(control & 2) || vTarget < height)) {
        const uint32_t period = (control & 2) ? width * height : width;
        const uint32_t target = ((control & 2) ? vTarget * width : 0) +
                                ((control & 1) ? hTarget : 0);
        uint32_t distance = (target + period - (position % period)) % period;
        if (!distance) distance = period;
        if (dots >= distance) sa1Flags_ |= 0x40;
    }
    const uint32_t next = static_cast<uint32_t>((position + dots) % (width * height));
    vCounter_ = static_cast<uint16_t>(next / width);
    hClocks_ = static_cast<uint16_t>((next % width) * 4 + (fractional & 3));
    timerMatch_ = matches(hClocks_ / 4, vCounter_);
}

void Sa1::StartArithmetic() {
    const uint16_t bitsA = RegisterWord(0x51);
    const uint16_t bitsB = RegisterWord(0x53);
    const int64_t a = bitsA < 0x8000 ? int64_t(bitsA) : int64_t(bitsA) - 0x10000;
    const int64_t b = bitsB < 0x8000 ? int64_t(bitsB) : int64_t(bitsB) - 0x10000;
    const uint8_t mode = registers_[0x50] & 3;
    arithmeticNextOverflow_ = false;
    arithmeticClocks_ = (mode & 2) ? 12 : 10;

    if (mode & 2) {
        const int64_t accumulator = (arithmeticResult_ & (uint64_t{1} << 39))
            ? int64_t(arithmeticResult_) - (int64_t{1} << 40) : int64_t(arithmeticResult_);
        const int64_t result = accumulator + a * b;
        arithmeticNextOverflow_ = result < -(int64_t{1} << 39) || result >= (int64_t{1} << 39);
        arithmeticNext_ = static_cast<uint64_t>(result) & AccumulatorMask;
    } else if (mode == 1) {
        arithmeticNext_ = 0;
        if (bitsB) {
            // The dividend is signed, but the remainder is nonnegative. This
            // is floor division, including negative, nonintegral quotients.
            int64_t quotient = a / bitsB;
            int64_t remainder = a % bitsB;
            if (remainder < 0) { --quotient; remainder += bitsB; }
            arithmeticNext_ = uint16_t(quotient) | (uint64_t(remainder) << 16);
            // Division fills the same five-byte signed result latch as a
            // multiply. Preserve bit 31 in its upper byte as well.
            if (arithmeticNext_ & 0x80000000) arithmeticNext_ |= uint64_t{0xff} << 32;
        }
        registers_[0x51] = registers_[0x52] = 0;
    } else {
        arithmeticNext_ = static_cast<uint64_t>(a * b) & AccumulatorMask;
    }
    registers_[0x53] = registers_[0x54] = 0;
}

void Sa1::RefreshBitResult() {
    uint32_t bits = 0;
    for (unsigned byte = 0; byte < 4; ++byte) {
        const uint32_t address = (bitAddress_ + byte) & 0xffffff;
        // A bit stream cannot recursively invoke its own register read port.
        bits |= uint32_t(ReadMemory(address, regs().mdr, Side::Sa1, false)) << (byte * 8);
    }
    bitResult_ = static_cast<uint16_t>(bits >> bitOffset_);
}

void Sa1::AdvanceBits() {
    const unsigned length = (registers_[0x58] & 15) ? (registers_[0x58] & 15) : 16;
    const unsigned position = bitOffset_ + length;
    bitAddress_ = (bitAddress_ + (position / 16) * 2) & 0xffffff;
    bitOffset_ = static_cast<uint8_t>(position & 15);
    StoreAddress(0x59, bitAddress_);
    RefreshBitResult();
}

} // namespace snes::core
