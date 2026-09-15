#include "FramePacer.hpp"
#include "snes/core/Timing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

using snes::frontend::FramePacer;
using snes::core::Region;
using snes::core::Timing;

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void ConsoleClockDeadlines() {
    for (Region region : {Region::NTSC, Region::PAL}) {
        for (bool interlace : {false, true}) {
            Timing timing(region);
            timing.SetInterlace(interlace);
            FramePacer pacer;
            const uint64_t epoch = 5'000'000'000;
            pacer.Schedule(epoch, 0, timing.MasterClockHz(), 40);
            uint64_t clocks = 0, deadline = epoch;
            for (unsigned frame = 0; frame < 600; ++frame) {
                const auto before = timing.MasterClocksElapsed();
                const auto target = timing.FrameCount() + 1;
                while (timing.FrameCount() < target) timing.Tick(2);
                const auto elapsed = timing.MasterClocksElapsed() - before;
                clocks += elapsed;
                deadline = pacer.Schedule(deadline + 3'000'000, elapsed,
                                          timing.MasterClockHz(), 40);
                const double expected = double(clocks) * 1'000'000'000 / timing.MasterClockHz();
                Check(std::abs(double(deadline - epoch) - expected) < 1.1,
                      "Presentation preserves region, field lengths and fractional clock time");
            }
        }
    }
}

void StallsAndMultipleFields() {
    FramePacer pacer;
    const uint64_t epoch = 1'000'000'000;
    const auto clocks = Timing::kMasterClocksPerFrameNTSC;
    pacer.Schedule(epoch, clocks, Timing::kMasterClockHz, 40);
    auto due = pacer.Schedule(epoch + 100'000'000, clocks, Timing::kMasterClockHz, 40);
    Check(due == epoch + 100'000'000, "A stalled frame can be shown immediately");
    const auto next = pacer.Schedule(due + 2'000'000, clocks, Timing::kMasterClockHz, 40);
    Check(next - due > 16'000'000, "Recovery has no burst of overdue frames");
    due = pacer.Schedule(next + 2'000'000, uint64_t(clocks) * 4, Timing::kMasterClockHz, 40);
    Check(due - next > 66'000'000 && due - next < 67'000'000,
          "A transfer spanning multiple fields retains all elapsed console time");
    pacer.Reset();
    Check(pacer.Schedule(due + 500'000'000, clocks, Timing::kMasterClockHz, 40) == due + 500'000'000,
          "Refilling an audio stream starts a fresh presentation epoch");
}

void DelayedPresentation() {
    FramePacer pacer;
    const auto clocks = Timing::kMasterClocksPerFrameNTSC;
    const double period = double(clocks) * 1'000'000'000 / Timing::kMasterClockHz;
    uint64_t due = 0;
    for (uint64_t delay : {8'000'000, 100'000'000}) {
        pacer.Reset();
        due = pacer.Schedule(1'000'000'000, clocks, Timing::kMasterClockHz, 40);
        pacer.Presented(due);
        due = pacer.Schedule(due + 2'000'000, clocks, Timing::kMasterClockHz, 40);
        const auto late = due + delay;
        pacer.Presented(late);
        const auto next = pacer.Schedule(late + 2'000'000, clocks, Timing::kMasterClockHz, 40);
        Check(std::abs(double(next - late) - period) < 1.1,
              "A stall after scheduling cannot bunch the following picture");
    }

    // Timer wake-up noise must not accumulate into a slower emulation clock.
    pacer.Reset();
    const uint64_t epoch = 2'000'000'000;
    due = pacer.Schedule(epoch, clocks, Timing::kMasterClockHz, 40);
    for (unsigned i = 0; i < 600; ++i) {
        due = pacer.Schedule(due + 500'000 + 2'000'000, clocks, Timing::kMasterClockHz, 40);
        pacer.Presented(due + 500'000);
    }
    const double expected = double(clocks) * 600 * 1'000'000'000 / Timing::kMasterClockHz;
    Check(std::abs(double(due - epoch) - expected) < 1.1,
          "Small presentation jitter does not accumulate as clock drift");
}

// Simulate audio callbacks that drain a 21.3 ms block at once, with a device
// clock slightly faster or slower than the console. This used to release
// multiple frames at one callback followed by a long wait at the next one.
void BatchedAudioDemand() {
    const double frameNs = double(Timing::kMasterClocksPerFrameNTSC) * 1'000'000'000 / Timing::kMasterClockHz;
    for (double deviceSpeed : {0.996, 1.0, 1.004}) {
        FramePacer pacer;
        double queuedMs = 50;
        uint64_t shown = 1'000'000'000;
        double nextCallback = double(shown) + 21'333'333 / deviceSpeed;
        const double callbackPeriod = 21'333'333 / deviceSpeed;
        pacer.Schedule(shown, 0, Timing::kMasterClockHz, 40);
        auto drain = [&](uint64_t until) {
            while (nextCallback <= double(until)) {
                queuedMs -= 21.333333;
                Check(queuedMs > 0, "Batched device consumption must not underrun during steady playback");
                nextCallback += callbackPeriod;
            }
        };
        for (unsigned frame = 0; frame < 12'000; ++frame) {
            const auto ready = shown + (frame % 3 + 1) * 2'000'000;
            drain(ready);
            queuedMs += frameNs / 1'000'000;
            const auto due = pacer.Schedule(ready, Timing::kMasterClocksPerFrameNTSC,
                                            Timing::kMasterClockHz, queuedMs);
            Check(double(due - shown) >= frameNs * 0.9949 && double(due - shown) <= frameNs * 1.0051,
                  "Audio callbacks and variable computation cannot bunch normal presentations");
            drain(due);
            Check(queuedMs < 80, "Clock drift must not grow an unbounded audio backlog");
            pacer.Presented(due);
            shown = due;
        }
    }
}
}

int main() {
    try {
        ConsoleClockDeadlines();
        StallsAndMultipleFields();
        DelayedPresentation();
        BatchedAudioDemand();
        std::puts("Console clock, audio callback, drift and stall pacing checks passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
