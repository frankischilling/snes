#include "snes/core/Emulator.hpp"
#include "snes/core/SnesCpu.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace snes::core;
namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
std::unique_ptr<Emulator> Machine() {
    std::vector<uint8_t> rom(0x8000, 0);
    std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
    rom[0x7fc0] = 'T'; rom[0x7fd5] = 0x20; rom[0x7fd7] = 5;
    rom[0x7fdc] = 0xff; rom[0x7fdd] = 0xff; rom[0x7ffd] = 0x80;
    rom[0] = 0xdb; // STP, so only hardware events advance in frame tests.
    auto emu = std::make_unique<Emulator>();
    std::string error;
    if (!emu->LoadCartridge(rom, &error)) throw std::runtime_error(error);
    return emu;
}

void ClockedTransfers() {
    auto emu = Machine();
    auto& dma = emu->GetDma();
    auto& bus = emu->GetBus();
    std::vector<uint64_t> clocks;
    bus.Map(0, 0, 0x2110, 0x2110, [](uint32_t, uint8_t b) { return b; },
        [&](uint32_t, uint8_t) { clocks.push_back(emu->GetTiming().MasterClocksElapsed()); });
    auto& ch = dma.Channel(0);
    ch.writeControl(0); ch.sourceBank = 0x7e; ch.sourceAddress = 0xff00;
    ch.targetAddress = 0x10; ch.transferSize = 512;
    const auto cpuPc = emu->GetCpu()->regs().pc;
    dma.EnableDma(1);
    const uint32_t ownClocks = dma.RunDma();
    Check(ownClocks == 8 + 16 + 512 * 8 + 2 && clocks.size() == 512, "DMA alignment, transfer and resume cycle accounting");
    Check(clocks.front() == 32 && clocks.back() > 4000, "Writes follow alignment, startup and the first eight-clock read");
    Check(std::is_sorted(clocks.begin(), clocks.end()), "DMA time is monotonic");
    unsigned refreshes = 0;
    for (size_t i = 1; i < clocks.size(); ++i)
        if (clocks[i] - clocks[i - 1] == 48) ++refreshes;
    Check(refreshes == 3, "Refresh pauses DMA by 40 clocks on each crossed scanline");
    Check(emu->GetTiming().MasterClocksElapsed() == ownClocks + 120, "Refresh is counted once");
    Check(ch.sourceBank == 0x7e && ch.sourceAddress == 0x100, "DMA wraps within the source bank");
    Check(emu->GetCpu()->regs().pc == cpuPc, "DMA stalls CPU execution");
    Check(emu->GetSmp().CycleCount() > 0, "APU advances during DMA");
}

void PaletteClockUnits() {
    for (unsigned start : {200u, 1200u}) {
        auto emu = Machine();
        auto& bus = emu->GetBus();
        auto& dma = emu->GetDma();
        bus.Write(0x2100, 15);
        bus.Write(0x2121, 5);
        bus.Write(0x7e0000, 0x1f);
        bus.Write(0x7e0001, 0);
        emu->GetTiming().Tick(1364 + start);
        auto& ch = dma.Channel(0);
        ch.writeControl(0); ch.sourceBank = 0x7e; ch.sourceAddress = 0;
        ch.targetAddress = 0x22; ch.transferSize = 2;
        dma.EnableDma(1); dma.RunDma();
        Check(emu->GetPpu().CgramData()[5] == (start == 1200 ? 0x1f : 0),
              "CGRAM gating uses master clocks: active writes redirect, HBlank writes use CGADD");
        Check(emu->GetPpu().CgramData()[0] == (start == 200 ? 0x1f : 0),
              "Active CGRAM write uses the rendering latch");
    }
}

void HdmaInterruptsDma() {
    for (bool sameChannel : {false, true}) {
        auto emu = Machine();
        auto& dma = emu->GetDma();
        auto& bus = emu->GetBus();
        // Let the scanline-zero setup point pass before manually initializing
        // HDMA; automatic setup is covered separately below.
        while (emu->GetTiming().HCounter() < 18) emu->GetCpu()->idle();
        std::vector<uint8_t> output;
        bus.Map(0, 0, 0x2110, 0x2110, [](uint32_t, uint8_t b) { return b; },
            [&](uint32_t, uint8_t b) { output.push_back(b); });
        auto& hdma = dma.Channel(sameChannel ? 0 : 1);
        hdma.writeControl(0); hdma.sourceBank = 0x7e; hdma.sourceAddress = 0x1000;
        hdma.targetAddress = 0x10;
        bus.Write(0x7e1000, 0x81); bus.Write(0x7e1001, 0xb9); bus.Write(0x7e1002, 0);
        dma.EnableHdma(sameChannel ? 1 : 2);
        dma.HdmaReset(); dma.HdmaSetup();
        auto& ch = dma.Channel(0);
        ch.writeControl(0); ch.sourceBank = 0x7e; ch.sourceAddress = 0x2000;
        ch.targetAddress = 0x10; ch.transferSize = 256;
        std::fill_n(bus.WramData() + 0x2000, 256, 0x37);
        dma.EnableDma(1); dma.RunDma();
        auto marker = std::find(output.begin(), output.end(), 0xb9);
        Check(marker != output.end() && marker != output.begin(), "HDMA runs after some DMA bytes");
        if (sameChannel) {
            Check(marker + 1 == output.end() && ch.transferSize > 0 && ch.transferSize < 256,
                  "HDMA cancels DMA on its own channel with remaining count intact");
        } else {
            Check(marker + 1 != output.end() && output.size() == 257 && ch.transferSize == 0,
                  "Another channel resumes DMA after HDMA");
        }
    }
}

void LongDmaAudio() {
    struct Output final : IAudioOutput {
        AudioBuffer last;
        unsigned calls = 0;
        void Submit(const AudioBuffer& audio) override { last = audio; ++calls; }
    } output;
    auto emu = Machine();
    emu->AttachAudioOutput(&output);
    auto& dsp = emu->GetDsp();
    dsp.Write(Dsp::kFlg, 0x3f);
    dsp.Write(Dsp::kMVolL, 0x7f); dsp.Write(Dsp::kMVolR, 0x7f);
    dsp.Write(Dsp::kVolL, 0x7f); dsp.Write(Dsp::kVolR, 0x7f);
    dsp.Write(Dsp::kAdsr0, 0); dsp.Write(Dsp::kGain, 0x7f);
    dsp.Write(Dsp::kDir, 0x20); dsp.Write(Dsp::kNon, 1); dsp.Write(Dsp::kKon, 1);
    auto& dma = emu->GetDma();
    for (unsigned i = 0; i < 8; ++i) {
        auto& channel = dma.Channel(i);
        channel.writeControl(8); // Fixed source, A-to-B.
        channel.sourceBank = 0x7e; channel.sourceAddress = 0;
        channel.targetAddress = 0x10; channel.transferSize = 0; // 65536 bytes.
    }
    dma.EnableDma(0xff);
    emu->StepFrame();
    const auto before = emu->GetSmp().CycleCount();
    // Output occurs at phase 27, followed by one output every 32 SMP clocks.
    const auto generated = (before + 4) / 32;
    Check(emu->GetTiming().FrameCount() > 1 && generated > 2048, "A maximal DMA crosses fields and exceeds one audio buffer");
    Check(output.calls == 1 && output.last.sampleRate == 32000 && output.last.interleavedStereo.size() == generated * 2,
          "Every DSP sample survives a DMA operation longer than the frame buffer");
    Check(std::any_of(output.last.interleavedStereo.begin() + 4096, output.last.interleavedStereo.end(),
                     [](float sample) { return sample != 0.0f; }), "Overflow blocks retain generated audio rather than padding silence");
    emu->StepFrame();
    const auto next = (emu->GetSmp().CycleCount() + 4) / 32 - generated;
    Check(output.calls == 2 && output.last.interleavedStereo.size() == next * 2 && next < 700,
          "The next frame contains only newly generated samples, with no stale overflow");
}

void DividerAlignment() {
    const std::array<uint32_t, 4> expectedDuration{36, 36, 30, 30};
    for (unsigned phase = 0; phase < 8; phase += 2) {
        auto bus = std::make_unique<MemoryBus>();
        DmaController dma;
        dma.SetBus(bus.get());
        uint64_t elapsed = phase;
        uint64_t readAt = 0, writeAt = 0;
        dma.SetClockCallback([&](uint32_t clocks) { elapsed += clocks; });
        dma.SetClockQuery([&]() { return elapsed; });
        bus->Map(0x40, 0x40, 0, 0,
            [&](uint32_t, uint8_t) { readAt = elapsed; return uint8_t(0x6b); },
            [](uint32_t, uint8_t) {});
        bus->Map(0, 0, 0x2110, 0x2110,
            [](uint32_t, uint8_t value) { return value; },
            [&](uint32_t, uint8_t value) { Check(value == 0x6b, "DMA carries the sampled byte"); writeAt = elapsed; });
        auto& channel = dma.Channel(0);
        channel.writeControl(0); channel.sourceBank = 0x40; channel.sourceAddress = 0;
        channel.targetAddress = 0x10; channel.transferSize = 1;
        dma.EnableDma(1);
        Check(dma.RunDma() == expectedDuration[phase / 2], "DMA returns at the next CPU cycle boundary");
        Check(elapsed == phase + expectedDuration[phase / 2], "Reported DMA duration matches hardware time");
        Check(readAt == 28 && writeAt == 32, "Divider alignment puts the read and write on distinct bus phases");
    }
}

void TimingBatchInvariance() {
    std::vector<uint64_t> expected;
    Timing batched, split;
    batched.onDramRefresh = [&]() { expected.push_back(batched.MasterClocksElapsed()); };
    std::vector<uint64_t> actual;
    split.onDramRefresh = [&]() { actual.push_back(split.MasterClocksElapsed()); };
    unsigned setups = 0;
    split.onHdmaSetup = [&]() { Check(split.HCounter() == 12, "Initial HDMA setup follows the divider phase"); ++setups; };
    batched.Tick(4092);
    for (unsigned i = 0; i < 4092; ++i) split.Tick(1);
    Check(expected == std::vector<uint64_t>{538, 1898, 3266}, "DRAM refresh alternates 538/534 on normal scanlines");
    Check(actual == expected && setups == 1, "Odd clock batches preserve event positions and counts");
    Check(split.HCounter() == batched.HCounter() && split.VCounter() == batched.VCounter() &&
          split.MasterClocksElapsed() == batched.MasterClocksElapsed(), "Odd clock batches preserve counters");
    Timing shortLine;
    shortLine.Tick(357368 + 240 * 1364);
    Check(shortLine.VCounter() == 240 && shortLine.HPeriod() == 1360, "Test reaches the odd-field short line");
    const auto refresh = shortLine.DramRefreshPosition();
    shortLine.Tick(1360);
    Check(shortLine.DramRefreshPosition() == refresh, "A short line preserves the refresh divider phase");
}

struct Video : IVideoOutput {
    VideoFrame frame;
    void Present(const VideoFrame& value) override { frame = value; }
};

void FieldTimingAndPresentation() {
    for (bool pal : {false, true}) {
        auto emu = Machine();
        auto& timing = emu->GetTiming();
        timing.SetRegion(pal ? Region::PAL : Region::NTSC);
        emu->GetBus().Write(0x2133, 0x09); // Interlace and pseudo-hires.
        Video video;
        emu->AttachVideoOutput(&video);
        for (unsigned frame = 1; frame <= 4; ++frame) {
            emu->StepFrame();
            Check(timing.FrameCount() == frame, "StepFrame stops at the next field boundary");
            Check(timing.VCounter() == 0 && timing.HCounter() < 8, "No partial scanline accumulates across frames");
            Check(video.frame.width == 512 && video.frame.height == 448 && video.frame.pixels.size() == 512 * 448,
                  "Presentation retains hires pixels and interlaced fields");
            Check(timing.Interlace(), "SETINI reaches video timing");
            const uint64_t pairs = frame / 2;
            const uint64_t base = pal ? 312 : 262;
            const uint64_t expected = pairs * ((base * 2 + 1) * 1364 + (pal ? 4 : 0)) +
                (frame % 2 ? (base + 1) * 1364 : 0);
            Check(timing.MasterClocksElapsed() >= expected && timing.MasterClocksElapsed() - expected < 8,
                  "Interlaced long and short fields have correct duration");
        }
    }
}
}
int main() {
    try { ClockedTransfers(); PaletteClockUnits(); HdmaInterruptsDma(); LongDmaAudio(); DividerAlignment(); TimingBatchInvariance(); FieldTimingAndPresentation(); }
    catch (const std::exception& e) { std::fprintf(stderr, "%s\n", e.what()); return 1; }
    std::puts("DMA timing, HDMA interruption, and field presentation checks passed");
}
