// snes emulator
// core/include/snes/core/Smp.hpp
// SMP audio processor and timer interface.

// Smp.hpp — SNES Sound Module Processor (Sony S-SMP / CXP1100Q-1)
//
// The SMP is the SNES APU wrapper that inherits the SPC700 processor core
// and provides:
//   - 64 KB APU RAM
//   - 64-byte IPL boot ROM (overlay at $FFC0-$FFFF)
//   - I/O registers at $00F0-$00FF
//   - 4 bidirectional CPU ↔ SMP communication ports
//
// Port communication model (dual-latch architecture):
//   CPU writes $2140-$2143 → sets apuInput_[0-3] → SMP reads $F4-$F7
//   SMP writes $F4-$F7     → sets cpuOutput_[0-3] → CPU reads $2140-$2143
//   Each direction has its own set of latches — no contention.
//
// Reference: bsnes sfc/smp/smp.hpp, sfc/smp/io.cpp, sfc/smp/memory.cpp

#pragma once

#include "snes/core/Spc700.hpp"
#include <array>
#include <cstdint>

namespace snes::core {

class Dsp;  // Forward declaration

class Smp : public Spc700 {
public:
    // Timer — 4-stage pipeline (following bsnes's SMP::Timer)
    //
    // Template parameter Frequency: number of SMP clocks per stage0 overflow.
    //   T0, T1: Frequency=128 → ~8 kHz
    //   T2:     Frequency=16  → ~64 kHz
    //
    // Stage 0: clock accumulator — counts up to Frequency
    // Stage 1: pulse toggle — XOR'd with 1 each overflow → square wave
    // Line:    edge-detection latch — falling-edge triggers stage 2
    // Stage 2: 8-bit divider counter — counts to target, then resets
    // Stage 3: 4-bit output counter — read-and-clear via $FD-$FF
    template <unsigned Frequency>
    struct Timer {
        uint8_t  stage0 = 0;    // Clock accumulator
        uint8_t  stage1 = 0;    // Pulse toggle (0 or 1)
        uint8_t  stage2 = 0;    // 8-bit divider counter
        uint8_t  stage3 = 0;    // 4-bit output counter (masked to 0x0F)
        bool     line   = false; // Edge-detection latch
        bool     enable = false; // Timer enabled (CONTROL bits 0-2)
        uint8_t  target = 0;    // 8-bit divisor ($FA/$FB/$FC)

        // Advance the timer by 'clocks' SMP timer-clock ticks.
        void Step(unsigned clocks, bool timersEnable, bool timersDisable);

        // Re-evaluate the gated stage1 signal and check for falling edge.
        // Called when TEST register changes timersEnable/timersDisable.
        void SynchronizeStage1(bool timersEnable, bool timersDisable);
    };

    Smp();

    // Bus interface (Spc700 pure virtuals)
    void    Idle() override;
    uint8_t Read(uint16_t address) override;
    void    Write(uint16_t address, uint8_t data) override;

    // CPU-side port access — called by MemoryBus for $2140-$2143
    //
    // portRead:  CPU reads $2140+n → returns what SMP wrote to $F4+n
    // portWrite: CPU writes $2140+n → sets data for SMP to read from $F4+n
    uint8_t PortRead(uint8_t port) const;
    void    PortWrite(uint8_t port, uint8_t data);

    // Timer stepping — call after each SMP bus cycle to advance timers
    // 'clocks' is the number of ideal timer ticks for this bus operation.
    // Default wait states: 2 ticks per half-cycle.
    void StepTimers(unsigned clocks);

    // DSP connection
    void SetDsp(Dsp& dsp) { dsp_ = &dsp; }

    // Clock constants
    static constexpr uint64_t kClockFrequency    = 1024000; // ~1.024 MHz
    static constexpr int      kDspSampleInterval = 32;      // SMP clocks per DSP sample

    // Batch execution — run SMP until CycleCount() >= targetCycles.
    // Each Step() executes one SPC700 instruction (consuming 2-8+ bus cycles).
    // DSP samples and timers are ticked automatically during bus operations.
    void RunUntil(uint64_t targetCycles);

    // Power-on reset
    void Power();

    // Direct access (for testing / debugging)
    uint8_t*       Ram()       noexcept { return ram_.data(); }
    const uint8_t* Ram() const noexcept { return ram_.data(); }
    static constexpr size_t RamSize = 0x10000; // 64 KB

    static constexpr size_t IplRomSize = 64;
    const uint8_t* IplRom() const noexcept { return iplRom_.data(); }

    // I/O state (for testing / inspection)
    struct IO {
        // $00F0 — TEST register (write-only, requires P=0)
        bool    timersDisable      = false;
        bool    ramWritable        = true;
        bool    ramDisable         = false;
        bool    timersEnable       = true;
        uint8_t externalWaitStates = 0;
        uint8_t internalWaitStates = 0;

        // $00F1 — CONTROL
        bool    iplRomEnable       = true;

        // $00F2 — DSPADDR
        uint8_t dspAddr            = 0;

        // $00F8-$00F9 — auxiliary I/O (RAM-backed, no special behavior)
        uint8_t aux4               = 0;
        uint8_t aux5               = 0;
    };

    IO& IoState() noexcept { return io_; }
    const IO& IoState() const noexcept { return io_; }

    // Port latch access for testing
    uint8_t ApuInput(uint8_t port) const { return apuInput_[port & 3]; }
    uint8_t CpuOutput(uint8_t port) const { return cpuOutput_[port & 3]; }

    // Timer access for testing/inspection
    Timer<128>&       GetTimer0()       noexcept { return timer0_; }
    const Timer<128>& GetTimer0() const noexcept { return timer0_; }
    Timer<128>&       GetTimer1()       noexcept { return timer1_; }
    const Timer<128>& GetTimer1() const noexcept { return timer1_; }
    Timer<16>&        GetTimer2()       noexcept { return timer2_; }
    const Timer<16>&  GetTimer2() const noexcept { return timer2_; }

private:
    // Internal I/O dispatch
    uint8_t readIO(uint16_t address);
    void    writeIO(uint16_t address, uint8_t data);

    // Internal RAM access (handles IPL ROM overlay)
    uint8_t readRam(uint16_t address) const;
    void    writeRam(uint16_t address, uint8_t data);

    // State

    // 64 KB APU RAM
    std::array<uint8_t, RamSize> ram_{};

    // 64-byte IPL boot ROM
    std::array<uint8_t, IplRomSize> iplRom_{};

    // I/O register state
    IO io_{};

    // Port latches — dual-latch architecture
    // apuInput_[0-3]:  CPU→SMP direction (CPU writes, SMP reads $F4-$F7)
    // cpuOutput_[0-3]: SMP→CPU direction (SMP writes $F4-$F7, CPU reads)
    std::array<uint8_t, 4> apuInput_{};   // written by CPU portWrite()
    std::array<uint8_t, 4> cpuOutput_{};  // written by SMP writeIO($F4-$F7)

    // Each SMP bus/idle cycle advances one DSP pipeline phase.
    void tickDsp();

    // DSP pointer (set via SetDsp)
    Dsp* dsp_ = nullptr;

    // Three SPC700 timers
    Timer<128> timer0_{};  // T0: ~8 kHz
    Timer<128> timer1_{};  // T1: ~8 kHz
    Timer<16>  timer2_{};  // T2: ~64 kHz
};

} // namespace snes::core
