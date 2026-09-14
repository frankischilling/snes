#pragma once

#include "snes/core/Platform.hpp"

#include <array>
#include <cstdint>
#include <functional>

namespace snes::core {

enum class ControllerDevice { None, Gamepad, Mouse, Multitap, SuperScope, Justifier, Justifiers, MacsRifle };

constexpr bool IsLightGun(ControllerDevice device) {
    return device == ControllerDevice::SuperScope || device == ControllerDevice::Justifier ||
           device == ControllerDevice::Justifiers || device == ControllerDevice::MacsRifle;
}

// The manual and automatic readers clock the same two physical ports.
class ControllerPorts {
public:
    using PadCallback = std::function<InputState(int player)>;
    using MouseCallback = std::function<MouseState(int port)>;
    using GunCallback = std::function<LightGunState(int port, int gun)>;

    void Reset();
    void Configure(unsigned port, ControllerDevice device);
    void Configure(unsigned port, ControllerDevice device, std::array<int, 4> players);
    void SetPadCallback(PadCallback callback) { padInput_ = std::move(callback); }
    void SetMouseCallback(MouseCallback callback) { mouseInput_ = std::move(callback); }
    void SetGunCallback(GunCallback callback) { gunInput_ = std::move(callback); }
    void SetGunLatchCallback(std::function<void(uint16_t, uint16_t)> callback) { gunLatch_ = std::move(callback); }
    void PollLightGuns();
    void FrameBegin();
    void BeamPosition(uint16_t h, uint16_t v, uint16_t visibleLines);
    uint8_t IoBits() const { return beamPulse_ ? 0x7f : 0xff; }
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
        std::array<LightGunState, 2> guns{};
        uint8_t scopePending = 0, scopePacket = 0;
        uint8_t justifierPacket = 0;
        bool selectSecond = false;
        bool scopeBeamArmed = false;
    };
    std::array<Port, 2> ports_{{Port{ControllerDevice::Gamepad, {0, -1, -1, -1}},
                               Port{ControllerDevice::Gamepad, {1, -1, -1, -1}}}};
    PadCallback padInput_;
    MouseCallback mouseInput_;
    GunCallback gunInput_;
    std::function<void(uint16_t, uint16_t)> gunLatch_;
    bool beamFired_ = false, beamPulse_ = false;
    bool latch_ = false;
    uint8_t pio_ = 0xff;
};

} // namespace snes::core
