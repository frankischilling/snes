// snes emulator
// core/include/snes/core/AutoJoypad.hpp
// Automatic controller polling interface.

#pragma once
// AutoJoypad.hpp — SNES auto-joypad polling state machine
//
// Implements the automatic controller polling that occurs at VBlank start
// when enabled via NMITIMEN ($4200) bit 0.
//
// At V = vdisp (VBlank start), if auto-joypad polling is enabled:
//   1. Controllers are latched (button state captured)
//   2. 16 bits are serially shifted into joy1-4 registers ($4218-$421F)
//   3. The entire sequence takes ~4224 master clocks (33 × 128)
//   4. HVBJOY ($4212) bit 0 reports "busy" during this time
//
// The state machine runs every 128 master clocks, matching bsnes's
// joypadEdge() in timing.cpp which fires when (counter.cpu & 127) == 0.
//
// Button bit layout in joy registers (MSB → LSB):
//   B  Y  Sel Sta  Up Dn Le Ri  A  X  L  R  0 0 0 0
//   15 14 13  12   11 10  9  8  7  6  5  4  3 2 1 0
//
// Usage:
//   1. Timing::onJoypadPoll  → AutoJoypad::Tick128(h, v, vdisp)
//   2. Timing::onFrameBegin  → AutoJoypad::FrameBegin()
//   3. CpuIoRegisters $4200  → AutoJoypad::SetAutoJoypadPoll()
//   4. After polling, sync joy1-4 → CpuIoRegisters setJoy1-4
//
// Reference: bsnes sfc/cpu/timing.cpp  (joypadEdge)
//            bsnes sfc/cpu/io.cpp      ($4200, $4212, $4218-$421F)
//            bsnes sfc/controller/gamepad/gamepad.cpp (serial bit order)

#include <cstdint>
#include <functional>

#include "snes/core/Platform.hpp"

namespace snes::core {

class AutoJoypad {
public:
    AutoJoypad();
    ~AutoJoypad() = default;

    /// Reset all state to power-on defaults.
    void Reset();

    // Timing interface

    /// Called every 128 master clocks by the timing system.
    /// Implements the 34-step state machine (counter 0..33).
    /// @param h     Current H counter (master clocks, 0..hperiod-1)
    /// @param v     Current V counter (scanline, 0..vperiod-1)
    /// @param vdisp First VBlank scanline (225 or 240)
    void Tick128(uint16_t h, uint16_t v, uint16_t vdisp);

    /// Called at V=0 (start of frame) to reset polling state.
    /// Matches bsnes: status.autoJoypadCounter = 33 at vcounter()==0.
    void FrameBegin();

    // $4200 NMITIMEN interaction

    /// Set the auto-joypad poll enable flag (NMITIMEN bit 0).
    /// If disabled during active polling (counter >= 2), aborts polling.
    void SetAutoJoypadPoll(bool enabled);

    /// Returns the current auto-joypad poll enable state.
    bool AutoJoypadPollEnabled() const noexcept { return autoJoypadPoll_; }

    // Input source

    /// Callback to snapshot input state for a controller port (0 or 1).
    /// Called once per port at the start of each polling sequence.
    using InputCallback = std::function<InputState(int port)>;
    void SetInputCallback(InputCallback cb) { onInput_ = std::move(cb); }

    // Output registers ($4218-$421F)

    uint16_t Joy1() const noexcept { return joy1_; }
    uint16_t Joy2() const noexcept { return joy2_; }
    uint16_t Joy3() const noexcept { return joy3_; }
    uint16_t Joy4() const noexcept { return joy4_; }

    // Status

    /// Current state machine counter (0-33; 33 = inactive).
    uint8_t Counter() const noexcept { return counter_; }

    /// Returns true if auto-joypad polling is enabled AND actively in progress.
    /// Used for HVBJOY ($4212) bit 0.
    bool IsPolling() const noexcept { return autoJoypadPoll_ && counter_ < 33; }

    // Utility

    /// Convert an InputState to the 16-bit SNES joypad register format.
    /// D-pad opposition is enforced (Up+Down → neither, Left+Right → neither).
    static uint16_t InputStateToSnesFormat(const InputState& input);

private:
    bool autoJoypadPoll_ = false;   ///< NMITIMEN bit 0
    uint8_t counter_     = 33;      ///< State machine (0..33; 33 = inactive)
    bool div256_         = false;   ///< 256-clock alignment divider

    // Latched 16-bit controller data (captured at counter=0)
    uint16_t port1Pad_   = 0;
    uint16_t port2Pad_   = 0;

    // Serial data temps (2 bits per port, matching bsnes autoJoypadPort1/2)
    uint8_t port1Data_   = 0;
    uint8_t port2Data_   = 0;

    // Shift position counter (0..15 during read/shift phase)
    uint8_t shiftPos_    = 0;

    // Output registers
    uint16_t joy1_       = 0;
    uint16_t joy2_       = 0;
    uint16_t joy3_       = 0;
    uint16_t joy4_       = 0;

    InputCallback onInput_;
};

} // namespace snes::core
