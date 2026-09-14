// ApuScheduler.hpp — CPU ↔ APU synchronization (single-threaded)
//
// Converts CPU master clock cycles to SPC700 (SMP) cycles using a
// Bresenham-style accumulator.  After each CPU instruction (or batch),
// call Run() to step the SMP/DSP proportionally.
//
// Clock rates:
//   CPU master clock: 21,477,272 Hz
//   SPC700 clock:      1,024,000 Hz
//   Ratio:  SMP_cycles = master_cycles × 1024000 / 21477272
//
// DSP sample generation is handled internally by the SMP — every 32 SMP
// bus cycles, SMP::tickDsp() fires Dsp::RunSample().
//
// Usage:
//   ApuScheduler sched;
//   sched.Reset(smp);
//   ...
//   uint32_t cpuClocks = cpu.Step();
//   sched.Run(smp, cpuClocks);
//
// Reference: bsnes sfc/smp/smp.cpp (thread synchronization)

#pragma once

#include "snes/core/Smp.hpp"
#include <cstdint>

namespace snes::core {

struct ApuScheduler {
    // Clock frequencies (Hz)
    static constexpr uint64_t kMasterClockRate = 21477272;  // ~21.477 MHz
    static constexpr uint64_t kSmpClockRate    = 1024000;   // ~1.024 MHz

    // State
    uint64_t accumulator    = 0;  // Fractional SMP-cycle remainder
    uint64_t smpTargetCycle = 0;  // Running total of SMP cycles owed

    // Reset — call when power-cycling or loading a new game.
    // Also resets the SMP itself.
    void Reset(Smp& smp) {
        accumulator    = 0;
        smpTargetCycle = 0;
        smp.Power();
    }

    // Run — call after the CPU consumes `masterClocks` master clocks.
    //
    // Converts master clocks → SMP clocks with exact fractional tracking,
    // then runs the SMP until it catches up.
    //
    //   smpClocks = masterClocks × kSmpClockRate / kMasterClockRate
    //
    // The remainder is carried in `accumulator` so no cycles are lost.
    void Run(Smp& smp, uint32_t masterClocks) {
        accumulator += static_cast<uint64_t>(masterClocks) * kSmpClockRate;
        uint64_t smpClocks = accumulator / kMasterClockRate;
        accumulator %= kMasterClockRate;
        smpTargetCycle += smpClocks;

        smp.RunUntil(smpTargetCycle);
    }

    // Convenience — compute the SMP target without running.
    // Useful for inspection / testing.
    uint64_t Advance(uint32_t masterClocks) {
        accumulator += static_cast<uint64_t>(masterClocks) * kSmpClockRate;
        uint64_t smpClocks = accumulator / kMasterClockRate;
        accumulator %= kMasterClockRate;
        smpTargetCycle += smpClocks;
        return smpTargetCycle;
    }
};

} // namespace snes::core
