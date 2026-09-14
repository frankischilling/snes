#include "snes/core/ControllerPorts.hpp"
#include "snes/core/AutoJoypad.hpp"

#include <algorithm>
#include <stdexcept>

namespace snes::core {

void ControllerPorts::Reset() {
    latch_ = false;
    pio_ = 0xff;
    for (auto& port : ports_) {
        const auto device = port.device;
        const auto players = port.players;
        port = {};
        port.device = device;
        port.players = players;
    }
}

void ControllerPorts::Configure(unsigned port, ControllerDevice device) {
    if (port >= ports_.size()) throw std::out_of_range("Controller port must be 0 or 1");
    const int first = static_cast<int>(port);
    Configure(port, device, {first, first + 1, first + 2, first + 3});
}

void ControllerPorts::Configure(unsigned port, ControllerDevice device, std::array<int, 4> players) {
    if (port >= ports_.size()) throw std::out_of_range("Controller port must be 0 or 1");
    for (const auto player : players)
        if (player < -1 || player > 7) throw std::out_of_range("Controller player must be -1 through 7");
    ports_[port] = {};
    ports_[port].device = device;
    ports_[port].players = players;
}

void ControllerPorts::SetLatch(bool high) {
    if (high && !latch_) {
        for (unsigned index = 0; index < ports_.size(); ++index) {
            auto& port = ports_[index];
            port.positions = {};
            if (port.device == ControllerDevice::Gamepad || port.device == ControllerDevice::Multitap) {
                for (unsigned pad = 0; pad < (port.device == ControllerDevice::Multitap ? 4u : 1u); ++pad) {
                    port.pads[pad] = port.players[pad] >= 0 && padInput_ ?
                        AutoJoypad::InputStateToSnesFormat(padInput_(port.players[pad])) : 0;
                }
            } else if (port.device == ControllerDevice::Mouse) {
                const auto state = mouseInput_ ? mouseInput_(index) : MouseState{};
                port.pendingX += state.dx;
                port.pendingY += state.dy;
                const auto movement = [](int64_t& pending) {
                    const int delta = static_cast<int>(std::clamp(pending, int64_t(-127), int64_t(127)));
                    pending -= delta;
                    return static_cast<uint8_t>(delta < 0 ? 0x80 | -delta : delta);
                };
                const auto x = movement(port.pendingX);
                const auto y = movement(port.pendingY);
                const uint8_t buttons = 1 | (port.sensitivity << 4) |
                    (state.left ? 0x40 : 0) | (state.right ? 0x80 : 0);
                port.mousePacket = (uint32_t(buttons) << 16) | (uint32_t(y) << 8) | x;
            }
        }
    }
    latch_ = high;
}

uint8_t ControllerPorts::Read(unsigned index) {
    if (index >= ports_.size()) return 0;
    auto& port = ports_[index];
    if (port.device == ControllerDevice::None) return 0;
    if (latch_) {
        if (port.device == ControllerDevice::Multitap) return 2;
        if (port.device == ControllerDevice::Mouse) {
            port.sensitivity = (port.sensitivity + 1) % 3;
            port.mousePacket = (port.mousePacket & ~uint32_t(0x00300000)) |
                               (uint32_t(port.sensitivity) << 20);
            return 0;
        }
        return (port.pads[0] >> 15) & 1;
    }

    const unsigned pair = port.device == ControllerDevice::Multitap && !(pio_ & (0x40 << index)) ? 1 : 0;
    auto& position = port.positions[pair];
    uint8_t data = 0;
    if (port.device == ControllerDevice::Mouse) {
        data = position < 32 ? (port.mousePacket >> (31 - position)) & 1 : 1;
    } else {
        for (unsigned wire = 0; wire < (port.device == ControllerDevice::Multitap ? 2u : 1u); ++wire) {
            const unsigned pad = pair * 2 + wire;
            if (port.players[pad] >= 0)
                data |= (position < 16 ? (port.pads[pad] >> (15 - position)) & 1 : 1) << wire;
        }
    }
    // Saturation prevents long reads from wrapping back to the packet start.
    if (position < 32) ++position;
    return data;
}

} // namespace snes::core
