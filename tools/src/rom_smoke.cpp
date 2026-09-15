// snes emulator
// tools/src/rom_smoke.cpp
// Headless ROM boot, frame, and output smoke test.

#include "snes/core/Emulator.hpp"
#include "snes/core/SnesCpu.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <stdexcept>

using namespace snes::core;
namespace {
unsigned Number(const char* text) {
    const std::string_view value(text);
    unsigned result = 0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
        throw std::runtime_error("Frame counts must be nonnegative integers");
    return result;
}

struct Output : IVideoOutput, IAudioOutput, IInputProvider {
    VideoFrame last;
    unsigned visibleFrames = 0;
    unsigned audibleSamples = 0;
    unsigned startFrame = 240;
    void Present(const VideoFrame& frame) override {
        last = frame;
        if (std::any_of(frame.pixels.begin(), frame.pixels.end(),
                        [](uint32_t pixel) { return (pixel & 0xffffff) != 0; })) ++visibleFrames;
    }
    void Submit(const AudioBuffer& buffer) override {
        for (float sample : buffer.interleavedStereo)
            if (std::abs(sample) > 0.001f) ++audibleSamples;
    }
    InputState Poll(uint64_t frame) override {
        InputState input;
        input.start = frame >= startFrame && frame < uint64_t(startFrame) + 20;
        return input;
    }
    bool Save(const char* path) {
        std::ofstream file(path, std::ios::binary);
        auto word = [&](uint32_t value, unsigned bytes) {
            for (unsigned i = 0; i < bytes; ++i) file.put(static_cast<char>(value >> (8 * i)));
        };
        word(0x4d42, 2); word(54 + last.width * last.height * 4, 4);
        word(0, 4); word(54, 4); word(40, 4);
        word(last.width, 4); word(last.height, 4); word(1, 2); word(32, 2);
        for (int i = 0; i < 6; ++i) word(0, 4);
        for (unsigned y = last.height; y-- > 0;) {
            for (unsigned x = 0; x < last.width; ++x) {
                // The core stores ABGR words; BMP stores blue first in memory.
                const uint32_t pixel = last.pixels[y * last.width + x];
                word((pixel & 0xff00ff00u) | ((pixel & 0xffu) << 16) | ((pixel >> 16) & 0xffu), 4);
            }
        }
        return bool(file);
    }
};
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "Usage: snes_rom_smoke ROM frames [output.bmp] [start-frame]\n");
        return 2;
    }
    try {
        const unsigned frames = Number(argv[2]);
        if (frames == 0) return 2;
        auto emu = std::make_unique<Emulator>();
        Output output;
        if (argc > 4) output.startFrame = Number(argv[4]);
        emu->AttachVideoOutput(&output);
        emu->AttachAudioOutput(&output);
        emu->AttachInputProvider(&output);
        std::string error;
        if (!emu->LoadCartridgeFromFile(argv[1], &error)) {
            std::fprintf(stderr, "%s\n", error.c_str());
            return 2;
        }
        for (unsigned i = 0; i < frames; ++i) emu->StepFrame();
        const auto& r = emu->GetCpu()->regs();
        std::printf("frames=%u visible=%u audible_samples=%u PC=%02X:%04X display=%s brightness=%u\n",
                    frames, output.visibleFrames, output.audibleSamples, r.pb, r.pc,
                    emu->GetPpu().DisplayDisable() ? "off" : "on", emu->GetPpu().Brightness());
        if (argc > 3 && !output.Save(argv[3])) return 2;
        return output.visibleFrames ? 0 : 1;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 2;
    }
}
