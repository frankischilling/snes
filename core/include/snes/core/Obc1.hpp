// snes emulator
// core/include/snes/core/Obc1.hpp
// OBC1 object RAM and register interface.

#pragma once

#include <array>
#include <cstdint>

namespace snes::core {

class Obc1 {
public:
    Obc1() { ram_.fill(0xff); }
    uint8_t Read(uint16_t address) const {
        const unsigned port = address & 0x1fff;
        if (port >= 0x1ff0 && port <= 0x1ff3)
            return ram_[base_ + object_ * 4 + port - 0x1ff0];
        if (port == 0x1ff4) return ram_[base_ + 0x200 + object_ / 4];
        return ram_[port];
    }
    void Write(uint16_t address, uint8_t value) {
        const unsigned port = address & 0x1fff;
        if (port >= 0x1ff0 && port <= 0x1ff3) {
            ram_[base_ + object_ * 4 + port - 0x1ff0] = value;
        } else if (port == 0x1ff4) {
            auto& attributes = ram_[base_ + 0x200 + object_ / 4];
            const unsigned shift = (object_ % 4) * 2;
            attributes = static_cast<uint8_t>((attributes & ~(3u << shift)) | ((value & 3u) << shift));
        } else if (port == 0x1ff5) {
            base_ = (value & 1) ? 0x1800 : 0x1c00;
        } else if (port == 0x1ff6) {
            object_ = value & 0x7f;
        }
        ram_[port] = value;
    }

private:
    std::array<uint8_t, 8192> ram_{};
    unsigned base_ = 0x1800;
    unsigned object_ = 0x7f;
};

}
