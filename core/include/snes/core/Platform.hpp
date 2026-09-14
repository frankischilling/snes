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
};

class ITimingSource {
public:
    virtual ~ITimingSource() = default;
    virtual uint64_t NowNanoseconds() const = 0;
};

} // namespace snes::core
