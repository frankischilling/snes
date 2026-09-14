// AutoJoypad.cpp — SNES auto-joypad polling implementation
//
// Implements the 34-step state machine matching bsnes's joypadEdge().
// Called every 128 master clocks by the timing system.
//
// State machine:
//   Counter 0     : Latch controllers (capture button state)
//   Counter 1     : Release latch
//   Counter 2-33  : Read/shift 16 bits (one bit per 256 clocks)
//     Even counter: read serial data from controller shift register
//     Odd counter : shift data into joy1-4 registers
//   Counter 33    : Inactive (polling complete)
//
// The start condition requires V = vdisp, H ∈ [130, 384], and 256-clock
// alignment (every other 128-clock edge).  This matches bsnes's condition:
//   vcounter()==ppu.vdisp() && (counter.cpu&255)==0 &&
//   hcounter()>=130 && hcounter()<=384
//
// Reference: bsnes sfc/cpu/timing.cpp (joypadEdge)

#include "snes/core/AutoJoypad.hpp"

namespace snes::core {

// Construction & Reset

AutoJoypad::AutoJoypad() {
    Reset();
}

void AutoJoypad::Reset() {
    autoJoypadPoll_ = false;
    counter_    = 33;
    div256_     = false;
    port1Pad_   = 0;
    port2Pad_   = 0;
    port1Data_  = 0;
    port2Data_  = 0;
    shiftPos_   = 0;
    joy1_ = joy2_ = joy3_ = joy4_ = 0;
}

// FrameBegin — reset for new frame

void AutoJoypad::FrameBegin() {
    // bsnes timing.cpp scanline(): at vcounter()==0, autoJoypadCounter = 33
    counter_ = 33;
}

// SetAutoJoypadPoll — $4200 NMITIMEN bit 0

void AutoJoypad::SetAutoJoypadPoll(bool enabled) {
    autoJoypadPoll_ = enabled;

    // bsnes io.cpp $4200: if disabled during active shifting, abort
    if (!enabled && counter_ >= 2 && counter_ < 33) {
        counter_ = 33;
    }
}

// InputStateToSnesFormat — button → 16-bit register conversion

uint16_t AutoJoypad::InputStateToSnesFormat(const InputState& input) {
    uint16_t v = 0;
    if (input.b)      v |= 0x8000;  // bit 15
    if (input.y)      v |= 0x4000;  // bit 14
    if (input.select) v |= 0x2000;  // bit 13
    if (input.start)  v |= 0x1000;  // bit 12
    // D-pad: prevent opposing directions simultaneously
    if (input.up   && !input.down)  v |= 0x0800;  // bit 11
    if (input.down && !input.up)    v |= 0x0400;  // bit 10
    if (input.left && !input.right) v |= 0x0200;  // bit 9
    if (input.right && !input.left) v |= 0x0100;  // bit 8
    if (input.a)      v |= 0x0080;  // bit 7
    if (input.x)      v |= 0x0040;  // bit 6
    if (input.l)      v |= 0x0020;  // bit 5
    if (input.r)      v |= 0x0010;  // bit 4
    // bits 3-0: signature (always 0 for standard gamepad)
    return v;
}

// Tick128 — 128-clock state machine step

void AutoJoypad::Tick128(uint16_t h, uint16_t v, uint16_t vdisp) {
    // 256-clock alignment: alternates every 128 clocks (true = aligned)
    div256_ = !div256_;

    // Start condition: V = vdisp, 256-clock aligned, H ∈ [130, 384]
    if (v == vdisp && div256_ && h >= 130 && h <= 384) {
        counter_ = 0;
    } else {
        if (counter_ >= 33) return;
        counter_++;
    }

    // Counter 0: Latch controllers
    // bsnes: latch signal is always asserted (even if disabled), but
    // joy registers are only cleared when auto-joypad is enabled.
    if (counter_ == 0) {
        if (autoJoypadPoll_) {
            // Capture input and convert to SNES serial format
            if (onInput_) {
                port1Pad_ = InputStateToSnesFormat(onInput_(0));
                port2Pad_ = InputStateToSnesFormat(onInput_(1));
            } else {
                port1Pad_ = 0;
                port2Pad_ = 0;
            }
            // Clear shift registers at start of polling
            joy1_ = joy2_ = joy3_ = joy4_ = 0;
            shiftPos_ = 0;
        }
    }

    // Counter 1: Release latch (no-op in simplified serial model)

    // Abort if disabled and not at counter 1
    // bsnes: counter 1 always runs (to properly release the latch)
    if (counter_ != 1 && !autoJoypadPoll_) {
        counter_ = 33;
        return;
    }

    // Counters 2-33: Read/shift 16 bits (one bit per 256 clocks)
    //   Even counter: read serial data from controller shift register
    //   Odd counter:  shift data into joy1-4 registers
    //
    // For a standard gamepad, bit 0 of serial data is the button state.
    // Bit 1 is always 0 (used for multitap, which we don't model).
    // This means joy3 and joy4 are always 0 for standard controllers.
    if (counter_ >= 2) {
        if ((counter_ & 1) == 0) {
            // Read: extract next bit from latched pad data (MSB first)
            if (shiftPos_ < 16) {
                port1Data_ = static_cast<uint8_t>(
                    (port1Pad_ >> (15 - shiftPos_)) & 1);
                port2Data_ = static_cast<uint8_t>(
                    (port2Pad_ >> (15 - shiftPos_)) & 1);
            } else {
                // After 16 bits, controller returns 1
                port1Data_ = 1;
                port2Data_ = 1;
            }
        } else {
            // Shift: push bit into joy registers
            joy1_ = static_cast<uint16_t>((joy1_ << 1) | (port1Data_ & 1));
            joy2_ = static_cast<uint16_t>((joy2_ << 1) | (port2Data_ & 1));
            // Bit 1: always 0 for standard gamepad → joy3/4 stay 0
            joy3_ = static_cast<uint16_t>((joy3_ << 1) | ((port1Data_ >> 1) & 1));
            joy4_ = static_cast<uint16_t>((joy4_ << 1) | ((port2Data_ >> 1) & 1));
            shiftPos_++;
        }
    }
}

} // namespace snes::core
