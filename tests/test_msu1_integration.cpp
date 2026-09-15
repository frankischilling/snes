#include "snes/core/Emulator.hpp"
#include "snes/core/SnesCpu.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

using namespace snes::core;

namespace {
using PcmFrame = std::array<int16_t, 2>;

void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

class Fixture {
public:
    Fixture() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        for (unsigned attempt = 0; attempt < 64; ++attempt) {
            auto candidate = std::filesystem::current_path() /
                ("msu1-console-" + std::to_string(stamp) + "-" + std::to_string(attempt));
            if (std::filesystem::create_directory(candidate)) {
                directory_ = std::move(candidate);
                return;
            }
        }
        throw std::runtime_error("Cannot create MSU-1 console fixture");
    }

    ~Fixture() {
        std::error_code error;
        for (const auto& path : files_) std::filesystem::remove(path, error);
        std::filesystem::remove(directory_, error);
    }

    std::filesystem::path Write(const char* name, std::span<const uint8_t> bytes) {
        const auto path = directory_ / name;
        files_.push_back(path);
        std::ofstream output(path, std::ios::binary);
        output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        output.close();
        Check(bool(output), "Write console fixture");
        return path;
    }

    void Pack(unsigned frameCount = 6000) {
        const std::vector<PcmFrame> frames(frameCount, PcmFrame{10000, -7000});
        Pack(frames);
    }

    void Pack(std::span<const PcmFrame> frames, uint32_t loopFrame = 0) {
        std::array<uint8_t, 256> data{};
        for (unsigned i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i ^ 0x59);
        Write("game.msu", data);
        std::vector<uint8_t> pcm{'M', 'S', 'U', '1', 0, 0, 0, 0};
        for (unsigned byte = 0; byte < 4; ++byte)
            pcm[4 + byte] = static_cast<uint8_t>(loopFrame >> (8 * byte));
        for (const auto& frame : frames) {
            for (const int16_t value : frame) {
                const auto bits = static_cast<uint16_t>(value);
                pcm.push_back(static_cast<uint8_t>(bits));
                pcm.push_back(static_cast<uint8_t>(bits >> 8));
            }
        }
        Write("game-1.pcm", pcm);
    }

private:
    std::filesystem::path directory_;
    std::vector<std::filesystem::path> files_;
};

std::vector<uint8_t> Rom(std::initializer_list<uint8_t> code = {0x80, 0xfe}, bool pal = false) {
    std::vector<uint8_t> rom(0x8000, 0xea);
    std::copy(code.begin(), code.end(), rom.begin());
    std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
    rom[0x7fc0] = 'T';
    rom[0x7fd5] = 0x20; rom[0x7fd6] = 0; rom[0x7fd7] = 5;
    rom[0x7fd8] = 0; rom[0x7fd9] = pal ? 2 : 1;
    rom[0x7fdc] = 0xff; rom[0x7fdd] = 0xff;
    rom[0x7fde] = 0; rom[0x7fdf] = 0;
    rom[0x7ffc] = 0; rom[0x7ffd] = 0x80;
    return rom;
}

std::unique_ptr<Emulator> Load(const std::filesystem::path& path) {
    auto machine = std::make_unique<Emulator>();
    std::string error;
    if (!machine->LoadCartridgeFromFile(path.string(), &error)) throw std::runtime_error(error);
    return machine;
}

void Play(Emulator& machine, bool repeat = false) {
    auto& bus = machine.GetBus();
    bus.Write(0x2004, 1);
    bus.Write(0x2005, 0);
    bus.Write(0x2006, 255);
    bus.Write(0x2007, repeat ? 3 : 1);
}

struct Audio final : IAudioOutput {
    std::vector<float> samples;
    void Submit(const AudioBuffer& buffer) override {
        Check(buffer.sampleRate == 32000, "Mixed PCM follows the core output rate");
        samples.insert(samples.end(), buffer.interleavedStereo.begin(), buffer.interleavedStereo.end());
    }
};

std::vector<PcmFrame> Waveform(size_t count) {
    std::vector<PcmFrame> frames(count);
    for (size_t i = 0; i < count; ++i) {
        frames[i] = {
            static_cast<int16_t>(static_cast<int>((i * 137 + 17) % 60001) - 30000),
            static_cast<int16_t>(static_cast<int>((i * 911 + 77) % 56003) - 28001)};
    }
    return frames;
}

// Evaluate the waveform's cumulative integral at arbitrary times. This has no
// playback cursor or resampler state and never calls the device under test.
class PcmOracle {
public:
    explicit PcmOracle(std::span<const PcmFrame> frames, bool repeat = false, size_t loopFrame = 0)
        : frames_(frames), prefix_(frames.size() + 1), repeat_(repeat), loopFrame_(loopFrame) {
        Check(!frames.empty() && loopFrame < frames.size(), "Valid PCM oracle fixture");
        for (size_t i = 0; i < frames.size(); ++i)
            for (unsigned channel = 0; channel < 2; ++channel)
                prefix_[i + 1][channel] = prefix_[i][channel] + frames[i][channel];
    }

    PcmFrame Frame(uint64_t index) const {
        PcmFrame result{};
        for (unsigned channel = 0; channel < 2; ++channel) {
            const int64_t area = Integral((index + 1) * 44100, channel) -
                                 Integral(index * 44100, channel);
            result[channel] = static_cast<int16_t>(area / 44100);
        }
        return result;
    }

private:
    int64_t Integral(uint64_t ticks, unsigned channel) const {
        // A source frame lasts 32,000 ticks; an output interval lasts 44,100.
        const uint64_t wholeFrames = ticks / 32000;
        size_t frame;
        int64_t sum;
        if (wholeFrames < frames_.size()) {
            frame = static_cast<size_t>(wholeFrames);
            sum = prefix_[frame][channel];
        } else if (repeat_) {
            const uint64_t extra = wholeFrames - frames_.size();
            const size_t period = frames_.size() - loopFrame_;
            frame = loopFrame_ + static_cast<size_t>(extra % period);
            const int64_t loopSum = prefix_.back()[channel] - prefix_[loopFrame_][channel];
            sum = prefix_.back()[channel] + static_cast<int64_t>(extra / period) * loopSum +
                  prefix_[frame][channel] - prefix_[loopFrame_][channel];
        } else {
            return prefix_.back()[channel] * 32000;
        }
        return sum * 32000 + int64_t{frames_[frame][channel]} * static_cast<int64_t>(ticks % 32000);
    }

    std::span<const PcmFrame> frames_;
    std::vector<std::array<int64_t, 2>> prefix_;
    bool repeat_;
    size_t loopFrame_;
};

uint64_t DspFrameCount(Emulator& machine) {
    const uint64_t clocks = machine.GetTiming().MasterClocksElapsed() * Smp::kClockFrequency /
                            machine.GetTiming().MasterClockHz();
    Check(machine.GetSmp().CycleCount() == clocks, "SMP reaches the exact master-clock deadline");
    Check(machine.GetDsp().Phase() == clocks % 32, "DSP phase remains continuous across output buffers");
    // Phase 27 publishes output: the first frame follows 28 SMP clocks, then
    // one follows every 32. SamplesWritten() is deliberately not the oracle.
    return (clocks + 4) / 32;
}

void CheckWaveform(std::span<const float> samples, uint64_t firstFrame, const PcmOracle& oracle) {
    Check(samples.size() % 2 == 0, "PCM output contains complete stereo frames");
    for (size_t i = 0; i < samples.size() / 2; ++i) {
        const auto expected = oracle.Frame(firstFrame + i);
        for (unsigned channel = 0; channel < 2; ++channel) {
            if (samples[i * 2 + channel] != static_cast<float>(expected[channel]) / 32768.0f) {
                throw std::runtime_error("PCM mismatch at output frame " + std::to_string(firstFrame + i) +
                    ", channel " + std::to_string(channel) + ": expected " +
                    std::to_string(expected[channel]) + ", got " +
                    std::to_string(samples[i * 2 + channel] * 32768.0f));
            }
        }
    }
}

void MappingAndLifecycle() {
    Fixture fixture;
    fixture.Pack();
    const auto rom = Rom();
    const auto path = fixture.Write("game.sfc", rom);
    auto machine = Load(path);
    auto& bus = machine->GetBus();
    for (uint32_t bank = 0; bank <= 0xbf; ++bank) {
        if (bank & 0x40) continue;
        Check(bus.Read((bank << 16) | 0x2000) == 2, "MSU-1 status maps across both system-bank ranges");
        Check(bus.Read((bank << 16) | 0x2002) == 'S' &&
              bus.Read((bank << 16) | 0x2007) == '1', "MSU-1 signature has mirrored bus decoding");
    }
    bus.Write(0x7e2002, 0xa5);
    Check(bus.Read(0x7e2002) == 0xa5, "MSU-1 leaves full WRAM banks intact");
    Check(bus.Read(0x2008) == 0xa5, "MSU-1 leaves adjacent addresses as open bus");

    // A fixed A-bus source still advances the data stream on every DMA read.
    bus.Write(0x2181, 0); bus.Write(0x2182, 0); bus.Write(0x2183, 0);
    bus.Write(0x4300, 8); bus.Write(0x4301, 0x80);
    bus.Write(0x4302, 1); bus.Write(0x4303, 0x20); bus.Write(0x4304, 0x95);
    bus.Write(0x4305, 16); bus.Write(0x4306, 0);
    machine->GetDma().EnableDma(1);
    machine->GetDma().RunDma();
    for (unsigned i = 0; i < 16; ++i)
        Check(bus.Read(0x7e0000 + i) == (i ^ 0x59), "DMA reads advance the MSU-1 stream through a mirror");

    const std::array<uint8_t, 1> invalid{0};
    Check(!machine->LoadCartridge(invalid), "Invalid ROM is rejected");
    Check(bus.Read(0x2002) == 'S', "Failed ROM loads preserve the active MSU-1 mapping");
    Check(machine->LoadCartridge(rom), "Plain in-memory ROM loads after an MSU-1 game");
    bus.Write(0x7e0000, 0xa5);
    Check(bus.Read(0x2002) == 0xa5, "A new plain cartridge removes old MSU-1 handlers");

    Check(machine->LoadCartridgeFromFile(path.string()), "MSU-1 game can be reloaded");
    Check(bus.Read(0x2000) == 2 && bus.Read(0x2001) == 0x59,
          "Reloading resets playback registers and the data position");
    const auto plain = fixture.Write("plain.sfc", rom);
    Check(machine->LoadCartridgeFromFile(plain.string()), "Missing optional sidecar permits ordinary ROM loading");
    bus.Write(0x7e0000, 0xa5);
    Check(bus.Read(0x2002) == 0xa5, "A ROM without a sidecar keeps normal expansion-bus behavior");
}

void TimedAudioControls() {
    for (const bool pal : {false, true}) {
        Fixture fixture;
        fixture.Pack();
        const auto rom = Rom({
            0xa9, 1, 0x8d, 4, 0x20, 0x9c, 5, 0x20, // Select track 1.
            0xa9, 255, 0x8d, 6, 0x20,
            0xa9, 1, 0x8d, 7, 0x20,                // Play.
            0xa2, 0, 0xe8, 0xd0, 0xfd,             // Wait 256 loop iterations.
            0x9c, 6, 0x20,                         // Mute during the same frame.
            0xa2, 0, 0xe8, 0xd0, 0xfd,
            0xa9, 255, 0x8d, 6, 0x20,              // Restore volume.
            0xa2, 0, 0xe8, 0xd0, 0xfd,
            0x9c, 7, 0x20,                         // Stop before the frame ends.
            0x80, 0xfe,
        }, pal);
        auto machine = Load(fixture.Write("game.sfc", rom));
        Audio audio;
        machine->AttachAudioOutput(&audio);
        machine->StepFrame();
        Check(audio.samples.size() >= 1000, "A complete region-timed audio frame is delivered");
        std::vector<bool> transitions;
        for (size_t i = 0; i < audio.samples.size(); i += 2) {
            const bool playing = audio.samples[i] != 0;
            Check(audio.samples[i] == (playing ? 10000 / 32768.0f : 0) &&
                  audio.samples[i + 1] == (playing ? -7000 / 32768.0f : 0),
                  "PCM preserves signed stereo and the volume active at each DSP sample");
            if (transitions.empty() && !playing) continue;
            if (transitions.empty() || transitions.back() != playing) transitions.push_back(playing);
        }
        Check(transitions == std::vector<bool>({true, false, true, false}),
              "Mid-frame mute, unmute and stop retain audio already produced");
        Check((machine->GetBus().Read(0x2000) & 0x10) == 0, "CPU stop command clears playing status");
    }
}

void PlaybackWithoutOutput() {
    Fixture fixture;
    fixture.Pack();
    auto machine = Load(fixture.Write("game.sfc", Rom()));
    Play(*machine);
    const auto target = machine->GetTiming().MasterClockHz() / 5;
    while (machine->GetTiming().MasterClocksElapsed() < target) machine->GetCpu()->idle();
    Check((machine->GetBus().Read(0x2000) & 0x10) == 0,
          "PCM reaches EOF without a sink, including after the DSP buffer fills");
    Audio audio;
    machine->AttachAudioOutput(&audio);
    machine->StepFrame();
    Check(!audio.samples.empty() && std::all_of(audio.samples.begin(), audio.samples.end(),
          [](float value) { return value == 0; }), "Attaching a sink does not replay old headless audio");
}

void ActivePlaybackWithoutOutput() {
    for (const bool pal : {false, true}) {
        Fixture fixture;
        const auto pcm = Waveform(10007);
        fixture.Pack(pcm);
        const PcmOracle oracle(pcm);
        auto machine = Load(fixture.Write("game.sfc", Rom({0x80, 0xfe}, pal)));
        Play(*machine);
        const uint64_t firstFrame = DspFrameCount(*machine);

        // Pass a complete headless buffer, then attach between DSP publications
        // while the PCM cursor also lies partway through a source frame.
        constexpr uint64_t hiddenFrames = 2309;
        const uint64_t target = hiddenFrames * 32 + 7;
        while (machine->GetSmp().CycleCount() < target) machine->GetCpu()->idle();
        const uint64_t skipped = DspFrameCount(*machine) - firstFrame;
        Check(skipped == hiddenFrames && skipped > 2048, "Headless playback crosses a full DSP buffer");
        Check(machine->GetDsp().Phase() != 0 && (skipped * 44100) % 32000 != 0,
              "Audio attachment exercises fractional DSP and PCM positions");
        Check((machine->GetBus().Read(0x2000) & 0x18) == 0x10, "PCM is still playing at audio attachment");

        Audio audio;
        machine->AttachAudioOutput(&audio);
        for (unsigned frame = 0; frame < 2; ++frame) {
            const size_t delivered = audio.samples.size();
            machine->StepFrame();
            const uint64_t audible = DspFrameCount(*machine) - firstFrame - skipped;
            Check(audio.samples.size() == audible * 2 && audio.samples.size() > delivered,
                  "Attached audio contains exactly the newly published DSP frames");
            CheckWaveform(std::span<const float>{audio.samples}.subspan(delivered),
                          skipped + delivered / 2, oracle);
        }
        Check((machine->GetBus().Read(0x2000) & 0x18) == 0x10,
              "Headless-to-audio continuity is checked before PCM reaches EOF");
    }
}

void DmaAudioAcrossFrames() {
    Fixture fixture;
    const auto pcm = Waveform(4099);
    constexpr uint32_t loopFrame = 137;
    fixture.Pack(pcm, loopFrame);
    const PcmOracle oracle(pcm, true, loopFrame);
    auto machine = Load(fixture.Write("game.sfc", Rom()));
    Audio audio;
    machine->AttachAudioOutput(&audio);
    Play(*machine, true);
    const uint64_t firstFrame = DspFrameCount(*machine);
    auto& bus = machine->GetBus();
    for (unsigned channel = 0; channel < 8; ++channel) {
        const uint32_t reg = 0x4300 + channel * 16;
        bus.Write(reg, 8); bus.Write(reg + 1, 0x80);
        bus.Write(reg + 2, 1); bus.Write(reg + 3, 0x20); bus.Write(reg + 4, 0);
        bus.Write(reg + 5, 0); bus.Write(reg + 6, 0); // 65,536 bytes per channel.
    }
    bus.Write(0x420b, 0xff);
    machine->StepFrame();
    const uint64_t dmaFrames = DspFrameCount(*machine) - firstFrame;
    Check(machine->GetTiming().FrameCount() > 1 && dmaFrames >= 3 * 2048,
          "Long DMA spans hardware frames and flushes at least three audio buffers");
    Check(audio.samples.size() == dmaFrames * 2, "DMA delivers every DSP frame exactly once");
    CheckWaveform(audio.samples, 0, oracle);

    const size_t delivered = audio.samples.size();
    machine->StepFrame();
    const uint64_t totalFrames = DspFrameCount(*machine) - firstFrame;
    Check(totalFrames > dmaFrames && audio.samples.size() == totalFrames * 2,
          "The frame after long DMA preserves the exact cumulative audio count");
    CheckWaveform(std::span<const float>{audio.samples}.subspan(delivered), dmaFrames, oracle);
    Check((bus.Read(0x2000) & 0x38) == 0x30, "Looping PCM remains active after DMA and the following frame");
}

void NativeAudioMixing() {
    Fixture fixture;
    const std::array<PcmFrame, 1> pcm{PcmFrame{30000, -30000}};
    fixture.Pack(pcm);
    const auto rom = Rom();
    auto native = Load(fixture.Write("plain.sfc", rom));
    auto mixed = Load(fixture.Write("game.sfc", rom));

    for (auto* machine : {native.get(), mixed.get()}) {
        // Stop the sound program so it cannot alter the synthetic voice; its
        // idle clocks still drive the entire DSP pipeline through the emulator.
        auto& smp = machine->GetSmp();
        smp.r.stop = true;
        auto* ram = smp.Ram();
        ram[0x100] = 0; ram[0x101] = 2;
        ram[0x102] = 0; ram[0x103] = 2;
        std::fill_n(ram + 0x200, 9, uint8_t{0});
        ram[0x200] = 3; // A silent looping BRR block keeps the noise voice keyed on.

        auto& dsp = machine->GetDsp();
        for (unsigned reg = 0; reg < Dsp::RegisterCount; ++reg)
            dsp.Write(static_cast<uint8_t>(reg), 0);
        dsp.Write(Dsp::kDir, 1);
        dsp.Write(Dsp::kPitchH, 0x10);
        dsp.Write(Dsp::kVolL, 0x7f);
        dsp.Write(Dsp::kVolR, 0x7f);
        dsp.Write(Dsp::kMVolL, 0x7f);
        dsp.Write(Dsp::kMVolR, 0x7f);
        dsp.Write(Dsp::kGain, 0x7f);
        dsp.Write(Dsp::kNon, 1);
        dsp.Write(Dsp::kFlg, 0x3f); // Fast noise, echo writes disabled, output unmuted.
        dsp.Write(Dsp::kKon, 1);
    }

    Audio nativeAudio, mixedAudio;
    native->AttachAudioOutput(&nativeAudio);
    mixed->AttachAudioOutput(&mixedAudio);
    Play(*mixed, true);
    for (unsigned frame = 0; frame < 2; ++frame) {
        native->StepFrame();
        mixed->StepFrame();
    }
    const uint64_t frames = DspFrameCount(*native);
    Check(frames != 0 && DspFrameCount(*mixed) == frames &&
          nativeAudio.samples.size() == frames * 2 && mixedAudio.samples.size() == frames * 2,
          "Plain and MSU machines publish the same complete native DSP timeline");

    bool nonzeroNative = false, positiveClip = false, negativeClip = false, unclippedMix = false;
    for (size_t i = 0; i < nativeAudio.samples.size(); ++i) {
        const int value = static_cast<int>(nativeAudio.samples[i] * 32768.0f);
        const int sum = value + pcm[0][i % 2];
        const int expected = std::clamp(sum, -32768, 32767);
        Check(mixedAudio.samples[i] == static_cast<float>(expected) / 32768.0f,
              "MSU-1 adds PCM to native DSP audio with signed saturation");
        nonzeroNative |= value != 0;
        positiveClip |= sum > 32767;
        negativeClip |= sum < -32768;
        unclippedMix |= value != 0 && sum > -32768 && sum < 32767;
    }
    Check(nonzeroNative && unclippedMix, "Mixer comparison includes nonzero native audio and unclipped addition");
    Check(positiveClip && negativeClip, "Mixer comparison exercises both positive and negative saturation");
}
} // namespace

int main() {
    try {
        MappingAndLifecycle();
        TimedAudioControls();
        PlaybackWithoutOutput();
        ActivePlaybackWithoutOutput();
        DmaAudioAcrossFrames();
        NativeAudioMixing();
        std::puts("MSU-1 console integration passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
