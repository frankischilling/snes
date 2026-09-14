// IrqController.cpp — SNES NMI / IRQ dispatch implementation
//
// Matches bsnes sfc/cpu/irq.cpp behavior:
//   nmiPoll(), irqPoll(), nmitimenUpdate(), rdnmi(), timeup(),
//   nmiTest(), irqTest(), lastCycle()

#include "snes/core/IrqController.hpp"

namespace snes::core {

// Construction & Reset

IrqController::IrqController() {
    Reset();
}

void IrqController::Reset() {
    nmiEnable_     = false;
    nmiLine_       = false;
    nmiValid_      = false;
    nmiHold_       = false;
    nmiTransition_ = false;

    hirqEnable_    = false;
    virqEnable_    = false;
    irqEnable_     = false;
    irqLine_       = false;
    irqValid_      = false;
    irqHold_       = false;
    irqTransition_ = false;

    htimeRaw_ = 0x1FF;
    htimeMc_  = static_cast<uint16_t>((0x1FF + 1) << 2);  // 2048
    vtime_    = 0x1FF;

    irqLock_  = false;
}

// H/V timer configuration

void IrqController::SetHTime(uint16_t raw9bit) noexcept {
    htimeRaw_ = raw9bit & 0x1FF;
    htimeMc_  = static_cast<uint16_t>((htimeRaw_ + 1) << 2);
}

void IrqController::SetVTime(uint16_t raw9bit) noexcept {
    vtime_ = raw9bit & 0x1FF;
}

// Poll — called every 4 master clocks by the timing system

void IrqController::Poll(uint16_t h, uint16_t v, uint16_t vdisp,
                          uint16_t hperiod) {
    nmiPoll(h, v, vdisp);
    irqPoll(h, v, hperiod);
}

// NMI poll — matches bsnes CPU::nmiPoll()
//
// Logic:
//   1. If nmiHold was active (from previous poll), it expires now.
//      If NMI is enabled, set nmiTransition (the NMI will fire).
//   2. Check if NMI condition (V >= vdisp at 2 clocks ago) changed.
//      On false→true transition: set nmiLine, start hold.
//      On true→false transition: clear nmiLine.

void IrqController::nmiPoll(uint16_t h, uint16_t v, uint16_t vdisp) {
    // Step 1: NMI hold expiration → transition fires
    if (nmiHold_) {
        nmiHold_ = false;
        if (nmiEnable_) {
            nmiTransition_ = true;
        }
    }

    // Step 2: Evaluate NMI condition with 2-clock delay
    // vcounter(2): if h >= 2, use current V; if h < 2, use V-1
    // (at the very start of a scanline, "2 clocks ago" was the previous line)
    uint16_t vDelayed = (h >= 2) ? v : (v > 0 ? static_cast<uint16_t>(v - 1) : uint16_t(0));
    bool newValid = (vDelayed >= vdisp);

    // Edge detection — fire on change
    if (newValid != nmiValid_) {
        nmiValid_ = newValid;
        nmiLine_  = nmiValid_;
        if (nmiLine_) {
            nmiHold_ = true;  // Hold for one poll cycle (4 clocks)
        }
    }
}

// IRQ poll — matches bsnes CPU::irqPoll()
//
// Logic:
//   1. Clear irqHold (it only lasts one poll cycle).
//   2. If irqLine is active and IRQ is enabled → re-assert transition.
//   3. Check composite IRQ condition (with 10-clock delay).
//      On false→true (rising edge): set irqLine and irqHold.

void IrqController::irqPoll(uint16_t h, uint16_t v, uint16_t hperiod) {
    // Step 1: always clear hold
    irqHold_ = false;

    // Step 2: if IRQ line already active and IRQ enabled → keep transition set
    if (irqLine_ && irqEnable_) {
        irqTransition_ = true;
    }

    // Step 3: evaluate composite IRQ condition with 10-clock communication delay
    //
    // hcounter(10): H counter from 10 master clocks ago
    // vcounter(10): V counter from 10 master clocks ago
    uint16_t hd = (h >= 10) ? static_cast<uint16_t>(h - 10)
                             : static_cast<uint16_t>(h + hperiod - 10);
    uint16_t vd = (h >= 10) ? v
                             : (v > 0 ? static_cast<uint16_t>(v - 1) : uint16_t(0));

    // The condition: irqEnable AND (V matches if virq) AND (H matches if hirq)
    // AND not the very last dot of the frame (vcounter(6) || hcounter(6))
    bool active = irqEnable_
        && (!virqEnable_ || vd == vtime_)
        && (!hirqEnable_ || hd == htimeMc_)
        && (vd > 0 || hd > 0);  // Not V=0, H=0 (last-dot guard)

    // Rising edge detection → new assertion
    if (active && !irqValid_) {
        irqValid_ = true;
        irqLine_  = true;
        irqHold_  = true;
    }
    // Falling edge — clear valid (but don't clear irqLine; that's done by Timeup)
    if (!active && irqValid_) {
        irqValid_ = false;
    }
}

// NmitimenUpdate — called when $4200 is written
//
// Matches bsnes CPU::nmitimenUpdate().

void IrqController::NmitimenUpdate(uint8_t data) {
    bool oldNmiEnable = nmiEnable_;

    // Decode enable bits
    hirqEnable_ = (data & 0x10) != 0;
    virqEnable_ = (data & 0x20) != 0;
    irqEnable_  = hirqEnable_ || virqEnable_;
    nmiEnable_  = (data & 0x80) != 0;

    // NMI enable rising edge while NMI line is already active → immediate fire
    if (!oldNmiEnable && nmiEnable_ && nmiLine_) {
        nmiTransition_ = true;
    }

    // IRQ mode changes (matches bsnes)
    if (virqEnable_ && !hirqEnable_ && irqLine_) {
        irqTransition_ = true;
    } else if (!irqEnable_) {
        irqLine_       = false;
        irqTransition_ = false;
    }

    // Lock: prevent interrupt dispatch for one lastCycle test
    irqLock_ = true;
}

// RDNMI ($4210) — read NMI flag with hold-aware clear
//
// Returns current nmiLine value.  If hold is not active, also clears
// nmiLine (the "read-and-clear" behavior).  The hold prevents the flag
// from being cleared during the 4-clock window after assertion.

bool IrqController::Rdnmi() {
    bool result = nmiLine_;
    if (!nmiHold_) {
        nmiLine_ = false;
    }
    return result;
}

// TIMEUP ($4211) — read IRQ flag with hold-aware clear
//
// Returns current irqLine value.  If hold is not active, also clears
// irqLine and irqTransition.

bool IrqController::Timeup() {
    bool result = irqLine_;
    if (!irqHold_) {
        irqLine_       = false;
        irqTransition_ = false;
    }
    return result;
}

// NmiTest — called from CPU lastCycle() equivalent
//
// Returns true if NMI should be dispatched (transition was pending).
// Clears the transition flag.

bool IrqController::NmiTest() {
    if (!nmiTransition_) return false;
    nmiTransition_ = false;
    return true;
}

// IrqTest — called from CPU lastCycle() equivalent
//
// Returns true if IRQ should be dispatched (transition pending AND I=0).
// Clears the transition flag regardless.

bool IrqController::IrqTest(bool iFlagClear) {
    if (!irqTransition_) return false;
    irqTransition_ = false;
    return iFlagClear;
}

} // namespace snes::core
