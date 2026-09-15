#include "snes/core/Dsp3.hpp"

namespace snes::core {

bool Dsp3::Bits(unsigned count, uint16_t& value) {
    if (!bitsNeeded_) {
        bitsNeeded_ = count;
        partialBits_ = 0;
    }
    while (bitsNeeded_) {
        if (!bitsAvailable_) {
            status_ = 0xc0;
            return false;
        }
        partialBits_ = uint16_t((partialBits_ << 1) | (bitWord_ >> 15));
        bitWord_ = uint16_t(bitWord_ << 1);
        --bitsAvailable_;
        --bitsNeeded_;
    }
    value = partialBits_;
    return true;
}

void Dsp3::Decode() {
    if (!bitsAvailable_) {
        if (status_ & 0x40) {
            bitWord_ = data_;
            bitsAvailable_ = 16;
        } else {
            status_ = 0xc0;
            return;
        }
    }
    uint16_t bits = 0;
    for (;;) {
        switch (decodePhase_) {
        case DecodePhase::SymbolPrefix:
            if (!Bits(2, bits)) return;
            symbolPrefix_ = bits;
            decodePhase_ = DecodePhase::SymbolValue;
            [[fallthrough]];
        case DecodePhase::SymbolValue:
            if (symbolPrefix_ != 1) {
                const unsigned size = symbolPrefix_ == 0 ? 9 : symbolPrefix_ == 2 ? 1 : 4;
                if (!Bits(size, bits)) return;
            }
            if (symbolPrefix_ == 0) symbol_ = bits;
            else symbol_ = uint16_t(symbol_ + (symbolPrefix_ == 1 ? 1 : (symbolPrefix_ == 2 ? 2 : 4) + bits));
            symbols_[index_++] = symbol_;
            if (--symbolsLeft_) { decodePhase_ = DecodePhase::SymbolPrefix; break; }
            index_ = symbol_ = 0;
            decodePhase_ = DecodePhase::TreeSize;
            break;
        case DecodePhase::TreeSize:
            if (!Bits(1, bits)) return;
            prefixBits_ = bits ? 3 : 2;
            treeEntries_ = 1u << prefixBits_;
            decodePhase_ = DecodePhase::TreeLength;
            break;
        case DecodePhase::TreeLength:
            if (!Bits(3, bits)) return;
            lengths_[index_] = uint8_t(bits + 1);
            offsets_[index_++] = symbol_;
            symbol_ = uint16_t(symbol_ + (1u << (bits + 1)));
            if (--treeEntries_) break;
            decodePhase_ = DecodePhase::Prefix;
            break;
        case DecodePhase::Prefix:
            if (!Bits(prefixBits_, bits)) return;
            prefix_ = bits;
            decodePhase_ = DecodePhase::Suffix;
            break;
        case DecodePhase::Suffix:
            if (!Bits(lengths_[prefix_], bits)) return;
            // Physical symbol storage has nine address bits. Malformed tables
            // cannot address outside the device's private working RAM.
            data_ = symbols_[(offsets_[prefix_] + bits) & 511];
            if (data_ & 0xff00) {
                data_ = uint16_t(data_ + 0x7f02);
                decodePhase_ = DecodePhase::CopySize;
            } else {
                decodePhase_ = DecodePhase::Prefix;
                if (--outputsLeft_ == 0) phase_ = Phase::Finish;
            }
            status_ = 0x80;
            return;
        case DecodePhase::CopySize:
            if (!Bits(1, bits)) return;
            copyBits_ = bits ? 12 : 8;
            decodePhase_ = DecodePhase::CopyOffset;
            break;
        case DecodePhase::CopyOffset:
            if (!Bits(copyBits_, bits)) return;
            data_ = bits;
            decodePhase_ = DecodePhase::Prefix;
            if (--outputsLeft_ == 0) phase_ = Phase::Finish;
            status_ = 0x80;
            return;
        }
    }
}

} // namespace snes::core
