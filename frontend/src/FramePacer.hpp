#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace snes::frontend {

// Schedule completed pictures from elapsed console clocks. Audio queue feedback
// follows device clock drift gradually instead of following individual callbacks.
class FramePacer {
public:
    void Reset() noexcept {
        active_ = false;
        queueError_ = 0;
        fractionalNanoseconds_ = 0;
        periodNanoseconds_ = 0;
    }

    uint64_t Schedule(uint64_t now, uint64_t masterClocks, uint32_t clockHz,
                      double queuedMilliseconds) {
        if (clockHz == 0) throw std::invalid_argument("Frame clock frequency must be positive");
        if (!active_) {
            active_ = true;
            deadline_ = now;
            return deadline_;
        }

        // The input queue drains in device-sized blocks. Average those steps
        // before correcting playback speed, with at most a 0.5% adjustment.
        queueError_ += (queuedMilliseconds - 40.0 - queueError_) / 64.0;
        const double adjustment = 1.0 + std::clamp(queueError_ / 2000.0, -0.005, 0.005);
        const double period = double(masterClocks) * 1'000'000'000.0 / clockHz * adjustment
                            + fractionalNanoseconds_;
        const auto whole = static_cast<uint64_t>(period);
        fractionalNanoseconds_ = period - double(whole);
        periodNanoseconds_ = whole;
        deadline_ += whole;

        // A slow frame or suspended window starts a fresh cadence. Replaying a
        // backlog of deadlines would display several pictures almost at once.
        if (deadline_ < now) deadline_ = now;
        return deadline_;
    }

    void Presented(uint64_t now) noexcept {
        // Event handling, save writes or the renderer can stall after Schedule.
        // Small wake-up jitter keeps the absolute clock; a material delay starts
        // the next interval at the picture that was actually displayed.
        if (active_ && now > deadline_ && now - deadline_ > periodNanoseconds_ / 4)
            deadline_ = now;
    }

private:
    uint64_t deadline_ = 0;
    uint64_t periodNanoseconds_ = 0;
    double queueError_ = 0;
    double fractionalNanoseconds_ = 0;
    bool active_ = false;
};

} // namespace snes::frontend
