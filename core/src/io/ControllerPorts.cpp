#include "snes/core/ControllerPorts.hpp"
#include "snes/core/AutoJoypad.hpp"

#include <algorithm>
#include <stdexcept>

namespace snes::core {

void ControllerPorts::Reset() {
    latch_ = false;
    pio_ = 0xff;
    FrameBegin();
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
    if (port == 1) FrameBegin();
}

void ControllerPorts::FrameBegin() {
    beamFired_ = false;
    beamPulse_ = false;
    for (auto& port : ports_) port.scopeBeamArmed = false;
}

void ControllerPorts::PollLightGuns() {
    for (unsigned index = 0; index < ports_.size(); ++index) {
        auto& port = ports_[index];
        if (!IsLightGun(port.device)) continue;
        for (unsigned gun = 0; gun < (port.device == ControllerDevice::Justifiers ? 2u : 1u); ++gun) {
            const auto previous = port.guns[gun];
            const auto state = gunInput_ ? gunInput_(index, gun) : LightGunState{};
            port.guns[gun] = state;
            if (port.device != ControllerDevice::SuperScope) continue;
            if (!state.trigger || (previous.turbo && !state.turbo)) port.scopePending &= ~0x80;
            if (!state.cursor || (previous.turbo && !state.turbo)) port.scopePending &= ~0x40;
            if (!state.pause) port.scopePending &= ~0x10;
            if (state.trigger && (!previous.trigger || state.turbo)) port.scopePending |= 0x80;
            if (state.cursor && (!previous.cursor || state.turbo)) port.scopePending |= 0x40;
            if (state.pause && !previous.pause) port.scopePending |= 0x10;
            if (port.scopePending & 0xc0) port.scopeBeamArmed = true;
        }
    }
}

void ControllerPorts::BeamPosition(uint16_t h, uint16_t v, uint16_t visibleLines) {
    beamPulse_ = false;
    const auto& port = ports_[1];
    if (!IsLightGun(port.device) || beamFired_) return;
    unsigned gun = 0;
    if (port.device == ControllerDevice::SuperScope && !port.scopeBeamArmed) return;
    if (port.device == ControllerDevice::Justifier && port.selectSecond) return;
    if (port.device == ControllerDevice::Justifiers) gun = unsigned(port.selectSecond);
    const auto& state = port.guns[gun];
    if (state.offscreen || state.x < 0 || state.x >= 256 || state.y < 0 || state.y >= visibleLines) return;
    const bool rifle = port.device == ControllerDevice::MacsRifle;
    const auto targetH = uint16_t(state.x + (rifle ? 76 : 40));
    const auto targetV = uint16_t(state.y + (rifle ? 42 : 1));
    if (v != targetV || h != targetH) return;
    beamFired_ = true;
    beamPulse_ = true;
    if ((pio_ & 0x80) && gunLatch_) gunLatch_(targetH, targetV);
}

void ControllerPorts::SetLatch(bool high) {
    if (high && !latch_) {
        PollLightGuns();
        for (unsigned index = 0; index < ports_.size(); ++index) {
            auto& port = ports_[index];
            port.positions = {};
            if (port.device == ControllerDevice::Gamepad || port.device == ControllerDevice::Multitap) {
                for (unsigned pad = 0; pad < (port.device == ControllerDevice::Multitap ? 4u : 1u); ++pad) {
                    port.pads[pad] = port.players[pad] >= 0 && padInput_ ?
                        AutoJoypad::InputStateToSnesFormat(padInput_(port.players[pad])) : 0;
                }
            } else if (port.device == ControllerDevice::SuperScope) {
                const auto& gun = port.guns[0];
                if (port.scopePending & 0x80)
                    port.scopePending = uint8_t((port.scopePending & ~0x20) | (gun.turbo ? 0x20 : 0));
                if (port.scopePending & 0xc0)
                    port.scopePending = uint8_t((port.scopePending & ~2) | (gun.offscreen ? 2 : 0));
                port.scopePacket = port.scopePending;
                port.scopePending &= uint8_t(~(gun.turbo ? 0x10 : 0xd0));
            } else if (port.device == ControllerDevice::Justifier || port.device == ControllerDevice::Justifiers) {
                port.selectSecond = !port.selectSecond;
                const auto& first = port.guns[0];
                const auto& second = port.guns[1];
                port.justifierPacket = uint8_t((first.trigger ? 0x80 : 0) | (first.start ? 0x20 : 0) |
                    (port.selectSecond ? 8 : 0));
                if (port.device == ControllerDevice::Justifiers)
                    port.justifierPacket |= uint8_t((second.trigger ? 0x40 : 0) | (second.start ? 0x10 : 0));
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
    if (port.device == ControllerDevice::MacsRifle) {
        // The rifle drives a live trigger level rather than a shift register.
        if (gunInput_) port.guns[0] = gunInput_(index, 0);
        return port.guns[0].trigger ? 1 : 0;
    }
    if (latch_) {
        if (port.device == ControllerDevice::SuperScope) return port.scopePacket >> 7;
        if (port.device == ControllerDevice::Justifier || port.device == ControllerDevice::Justifiers) return 0;
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
    if (port.device == ControllerDevice::SuperScope) {
        data = position < 8 ? (port.scopePacket >> (7 - position)) & 1 : 1;
    } else if (port.device == ControllerDevice::Justifier || port.device == ControllerDevice::Justifiers) {
        const uint32_t packet = 0x000e5500 | port.justifierPacket;
        data = position < 32 ? (packet >> (31 - position)) & 1 : 1;
    } else if (port.device == ControllerDevice::Mouse) {
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
