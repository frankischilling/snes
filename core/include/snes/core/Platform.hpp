// snes emulator
// core/include/snes/core/Platform.hpp
// Platform-facing video, audio, and input contracts.

#pragma once

#include <cstdint>
#include <span>
#include <vector>

namespace snes::core {

struct VideoFrame {
    uint32_t width = 256;
    uint32_t height = 224;
    std::vector<uint32_t> pixels;
};

struct AudioBuffer {
    uint32_t sampleRate = 32000;
    std::vector<float> interleavedStereo;
};

struct InputState {
    bool up = false;
    bool down = false;
    bool left = false;
    bool right = false;
    bool a = false;
    bool b = false;
    bool x = false;
    bool y = false;
    bool l = false;
    bool r = false;
    bool start = false;
    bool select = false;
};

struct MouseState {
    int32_t dx = 0;
    int32_t dy = 0;
    bool left = false;
    bool right = false;
};

struct LightGunState {
    // Coordinates use 256 horizontal dots and visible (non-interlaced) rows.
    int32_t x = 128, y = 112;
    bool trigger = false;
    bool cursor = false;
    bool turbo = false;
    bool pause = false;
    bool start = false;
    bool offscreen = true;
};

class IVideoOutput {
public:
    virtual ~IVideoOutput() = default;
    virtual void Present(const VideoFrame& frame) = 0;
};

class IAudioOutput {
public:
    virtual ~IAudioOutput() = default;
    virtual void Submit(const AudioBuffer& buffer) = 0;
};

class IInputProvider {
public:
    virtual ~IInputProvider() = default;
    virtual InputState Poll(uint64_t frameIndex) = 0;
    virtual InputState PollController(int player, uint64_t frameIndex) {
        return player == 0 ? Poll(frameIndex) : InputState{};
    }
    // Relative movement is consumed once when the console latches the mouse.
    virtual MouseState PollMouse(int /*port*/, uint64_t /*frameIndex*/) { return {}; }
    virtual LightGunState PollLightGun(int /*port*/, int /*gun*/, uint64_t /*frameIndex*/) { return {}; }
};

class ITimingSource {
public:
    virtual ~ITimingSource() = default;
    virtual uint64_t NowNanoseconds() const = 0;
};

} // namespace snes::core
