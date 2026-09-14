#pragma once

#include "snes/core/Platform.hpp"

#include <array>
#include <cstdint>
#include <functional>

namespace snes::core {

enum class ControllerDevice { None, Gamepad, Mouse, Multitap };

// The manual and automatic readers clock the same two physical ports.
class ControllerPorts {
public:
    using PadCallback = std::function<InputState(int player)>;
    using MouseCallback = std::function<MouseState(int port)>;

    void Reset();
    void Configure(unsigned port, ControllerDevice device);
    void Configure(unsigned port, ControllerDevice device, std::array<int, 4> players);
    void SetPadCallback(PadCallback callback) { padInput_ = std::move(callback); }
    void SetMouseCallback(MouseCallback callback) { mouseInput_ = std::move(callback); }
    void SetLatch(bool high);
    void SetPio(uint8_t value) { pio_ = value; }
    uint8_t Read(unsigned port);

private:
    struct Port {
        ControllerDevice device = ControllerDevice::Gamepad;
        std::array<int, 4> players{};
        std::array<uint16_t, 4> pads{};
        std::array<unsigned, 2> positions{};
        uint32_t mousePacket = 0;
        uint8_t sensitivity = 0;
        int64_t pendingX = 0, pendingY = 0;
    };
    std::array<Port, 2> ports_{{Port{ControllerDevice::Gamepad, {0, -1, -1, -1}},
                               Port{ControllerDevice::Gamepad, {1, -1, -1, -1}}}};
    PadCallback padInput_;
    MouseCallback mouseInput_;
    bool latch_ = false;
    uint8_t pio_ = 0xff;
};

} // namespace snes::core
