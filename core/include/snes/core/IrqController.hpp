#pragma once
// IrqController.hpp — SNES NMI / IRQ dispatch state machine
//
// Implements the NMI/IRQ evaluation logic from bsnes's irq.cpp:
//
//   NMI:
//     - Condition: V counter >= VDisp (with 2-clock communication delay)
//     - Edge-detected: fires on false→true transition only
//     - 4-clock hold timer: prevents RDNMI ($4210) from clearing the flag
//       for one poll cycle (4 master clocks) after assertion
//     - NMI transition fires only when NMI is enabled ($4200 bit 7)
//     - If NMI is enabled after the condition is already true (rising edge
//       on enable bit), the transition fires immediately
//
//   IRQ:
//     - H-IRQ:  H counter matches $4207-$4208 target
//     - V-IRQ:  V counter matches $4209-$420A target
//     - HV-IRQ: both match simultaneously
//     - 10-clock communication delay on H/V counter reads
//     - Edge-detected on the composite condition (false→true = one trigger)
//     - 4-clock hold timer protects TIMEUP ($4211) read-and-clear
//     - IRQ line stays asserted until TIMEUP is read (or IRQ disabled)
//     - IRQ cannot trigger on the very last dot of a frame
//
//   IRQ lock:
//     - Set when NMITIMEN ($4200) is written
//     - Prevents NMI/IRQ dispatch for one instruction (one lastCycle test)
//     - Matches bsnes status.irqLock behavior
//
// The IrqController is self-contained — it owns its own copies of all
// enable flags, timer targets, and state machines.  The integration layer
// (Emulator) wires it to CpuIoRegisters, Timing, and SnesCpu.
//
// Usage:
//   1. Timing::onIrqPoll → IrqController::Poll(h, v, vdisp, hperiod)
//   2. CpuIoRegisters $4200 write → IrqController::NmitimenUpdate(data)
//   3. CpuIoRegisters $4207-$420A writes → IrqController::SetHTime/SetVTime
//   4. CpuIoRegisters $4210 read → IrqController::Rdnmi()
//   5. CpuIoRegisters $4211 read → IrqController::Timeup()
//   6. After each CPU Step: check NmiTest/IrqTest → drive SnesCpu interrupts
//
// Reference: bsnes sfc/cpu/irq.cpp

#include <cstdint>

namespace snes::core {

class IrqController {
public:
    IrqController();

    /// Reset all state to power-on defaults.
    void Reset();

    // Periodic polling — called every 4 master clocks by the timing system

    /// Evaluate NMI and IRQ conditions based on current counters.
    /// @param h       Current H counter (master clocks, 0..hperiod-1)
    /// @param v       Current V counter (scanline, 0..vperiod-1)
    /// @param vdisp   First VBlank scanline (225 or 240)
    /// @param hperiod Width of current scanline in master clocks
    void Poll(uint16_t h, uint16_t v, uint16_t vdisp, uint16_t hperiod);

    // NMITIMEN ($4200) write handler

    /// Called when $4200 is written.  Updates enable flags and handles
    /// edge cases (NMI enable rising edge while NMI line is already active,
    /// IRQ mode changes, irqLock).
    void NmitimenUpdate(uint8_t data);

    // Register read hooks — called by CpuIoRegisters

    /// $4210 RDNMI — returns true if NMI flag is set, then performs
    /// read-and-clear with hold protection (flag is NOT cleared if hold
    /// is still active).
    bool Rdnmi();

    /// $4211 TIMEUP — returns true if IRQ flag is set, then performs
    /// read-and-clear with hold protection.
    bool Timeup();

    // CPU lastCycle() interface

    /// Test if NMI should fire.  Returns true and clears the transition
    /// flag.  The caller should then assert NMI on the CPU.
    bool NmiTest();

    /// Test if IRQ should fire.  Returns true if the IRQ transition is
    /// pending AND the I flag is clear.  Clears the transition flag
    /// regardless. @param iFlagClear  true if the CPU's I flag is cleared
    bool IrqTest(bool iFlagClear);

    // IRQ lock

    /// Returns true if the interrupt lock is active (suppresses NmiTest/IrqTest).
    bool IrqLocked() const noexcept { return irqLock_; }

    /// Set/clear the lock.  Typically set by NmitimenUpdate, cleared by the
    /// integration layer after calling NmiTest/IrqTest once.
    void SetIrqLock(bool v) noexcept { irqLock_ = v; }

    // H/V timer target configuration (from $4207-$420A writes)

    /// Set the H-counter IRQ target (9-bit raw value, 0-339 dot range).
    /// Internally converted to master clocks: (raw + 1) << 2.
    void SetHTime(uint16_t raw9bit) noexcept;

    /// Set the V-counter IRQ target (9-bit raw value, scanline number).
    void SetVTime(uint16_t raw9bit) noexcept;

    /// Get raw H-counter target (9-bit, as written to registers).
    uint16_t HTimeRaw() const noexcept { return htimeRaw_; }

    /// Get converted H-counter target in master clocks.
    uint16_t HTimeMasterClocks() const noexcept { return htimeMc_; }

    /// Get V-counter target (9-bit raw scanline number).
    uint16_t VTime() const noexcept { return vtime_; }

    // State queries

    bool NmiEnabled()  const noexcept { return nmiEnable_; }
    bool HIrqEnabled() const noexcept { return hirqEnable_; }
    bool VIrqEnabled() const noexcept { return virqEnable_; }
    bool IrqEnabled()  const noexcept { return irqEnable_; }

    bool NmiLine()       const noexcept { return nmiLine_; }
    bool IrqLine()       const noexcept { return irqLine_; }
    bool NmiTransition() const noexcept { return nmiTransition_; }
    bool IrqTransition() const noexcept { return irqTransition_; }
    bool NmiHold()       const noexcept { return nmiHold_; }
    bool IrqHold()       const noexcept { return irqHold_; }

private:
    void nmiPoll(uint16_t h, uint16_t v, uint16_t vdisp);
    void irqPoll(uint16_t h, uint16_t v, uint16_t hperiod);

    // NMI state machine
    bool nmiEnable_     = false;
    bool nmiLine_       = false;  // NMI flag (returned by RDNMI)
    bool nmiValid_      = false;  // Edge detector input (V >= vdisp)
    bool nmiHold_       = false;  // 4-clock hold (protects RDNMI clear)
    bool nmiTransition_ = false;  // Pending NMI dispatch

    // IRQ state machine
    bool hirqEnable_    = false;
    bool virqEnable_    = false;
    bool irqEnable_     = false;  // hirq || virq
    bool irqLine_       = false;  // IRQ flag (returned by TIMEUP)
    bool irqValid_      = false;  // Edge detector input
    bool irqHold_       = false;  // 4-clock hold (protects TIMEUP clear)
    bool irqTransition_ = false;  // Pending IRQ dispatch

    // H/V timer targets
    uint16_t htimeRaw_  = 0x1FF;  // Raw 9-bit value from $4207/$4208
    uint16_t htimeMc_   = 0;      // Converted to master clocks: (raw+1)<<2
    uint16_t vtime_     = 0x1FF;  // Raw 9-bit value from $4209/$420A

    // Lock
    bool irqLock_       = false;
};

} // namespace snes::core
