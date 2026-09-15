#include "snes/core/SetaProtocol.hpp"

#include <algorithm>

namespace snes::core {

void St011::Reset() {
    packet_.fill(0); board_.fill(0); work_.fill(0);
    command_ = 0; received_ = required_ = 0;
}

uint8_t St011::Read(uint32_t address) const {
    const unsigned offset = address & 0xffff;
    if (offset == 1) return 0xff;
    return offset < ram_.size() ? ram_[offset] : work_[offset];
}

void St011::Store(unsigned offset, uint8_t value) {
    if (offset < ram_.size()) ram_[offset] = value;
    else work_[offset & 0xffff] = value;
}

void St011::Write(uint32_t address, uint8_t value) {
    Store(address & 0xffff, value);
    if ((address & 0xffff) != 0) return;
    if (required_) {
        packet_[received_++] = value;
        if (received_ == required_) Complete();
        return;
    }
    command_ = value;
    received_ = 0;
    if (command_ == 1) required_ = 128;
    else if (command_ == 2) required_ = 4;
    else Complete();
}

void St011::Complete() {
    if (command_ == 1) {
        for (unsigned row = 0; row < 9; ++row) std::copy_n(packet_.begin() + row * 10, 9, board_.begin() + row * 9);
    } else if (command_ == 4 || command_ == 5) {
        Store(0x12c, 0); Store(0x12e, 0);
    } else if (command_ == 14) {
        Store(0x12c, 0); Store(0x12d, 0);
    }
    // Commands 2, 6 and 7 have no established result calculation here.
    required_ = received_ = 0;
}

void St018::Reset() {
    command_ = commandBytes_ = pass_ = outputCount_ = 0;
    awaitingParameter_ = false;
}

uint8_t St018::Read(uint32_t address) {
    if ((address & 0xffff) == 0x3804) {
        if (outputCount_) --outputCount_;
        return 0x81;
    }
    return 0;
}

void St018::Write(uint32_t address, uint8_t value) {
    const unsigned offset = address & 0xffff;
    // The startup ports are outside the board's persistent 4 KiB RAM. They
    // must not wrap into it and corrupt a saved game at $0802/$0804.
    if (offset < ram_.size()) ram_[offset] = value;
    if (offset == 0x3802 && awaitingParameter_) {
        outputCount_ = 3;
        awaitingParameter_ = ++pass_ < 3;
    } else if (offset == 0x3804 && !awaitingParameter_) {
        command_ = ((command_ << 8) | value) & 0xffffff;
        if (++commandBytes_ == 3) {
            commandBytes_ = 0;
            pass_ = 0;
            outputCount_ = 0;
            if (command_ == 0x000100 || command_ == 0x00ff00) {
                pass_ = 1;
                awaitingParameter_ = true;
                outputCount_ = 2;
            }
        }
    }
}

} // namespace snes::core
