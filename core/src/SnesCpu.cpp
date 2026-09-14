// SnesCpu.cpp — SNES-specific CPU wrapper implementation
//
// Implements the Processor65816 virtual bus interface with SNES timing,
// NMI/IRQ state machine, and cycle counting.

#include "snes/core/SnesCpu.hpp"

namespace snes::core {

// Construction

SnesCpu::SnesCpu(ICpuBus& bus)
    : bus_(bus)
{
}

// Reset

void SnesCpu::Reset() {
    power();

    cycles_ = 0;
    instructionClocks_ = 0;

    nmiLine_    = false;
    nmiEdge_    = false;
    nmiPending_ = false;
    irqLine_    = false;
    irqPending_ = false;
    openBus_    = 0;

    // Load the reset vector
    uint8_t pcl = bus_.Read(0xFFFC);
    uint8_t pch = bus_.Read(0xFFFD);
    regs().pc = static_cast<uint16_t>(pcl | (pch << 8));
    regs().pb = 0x00;

    // The reset vector fetch consumes some master clocks
    addClocks(12);
}

// Step — execute one instruction

uint32_t SnesCpu::Step() {
    instructionClocks_ = 0;

    // Check for pending NMI/IRQ before the instruction
    if (nmiPending_) {
        nmiPending_ = false;
        nmiEdge_ = false;
        enterInterrupt(Interrupt::Nmi);
        return instructionClocks_;
    }

    if (irqPending_ && !(regs().p & FlagI)) {
        irqPending_ = false;
        enterInterrupt(Interrupt::Irq);
        return instructionClocks_;
    }

    // WAI: if the CPU is in WAI state, consume an idle cycle and return
    if (regs().wai) {
        idle();
        // WAI clears when NMI or IRQ fires.
        // Promote the edge/level to pending so the interrupt is dispatched
        // on the very next Step() — no instruction after WAI should execute
        // before the interrupt handler runs.
        if (nmiEdge_ || irqLine_) {
            regs().wai = false;
            if (nmiEdge_) {
                nmiPending_ = true;
                nmiEdge_ = false;
            }
            if (irqLine_ && !(regs().p & FlagI)) {
                irqPending_ = true;
            }
        }
        return instructionClocks_;
    }

    // STP: just burn a cycle
    if (regs().stp) {
        idle();
        return instructionClocks_;
    }

    // Execute one instruction
    instruction();

    return instructionClocks_;
}

// External interrupt requests

void SnesCpu::RequestNmi() {
    // NMI is edge-sensitive.  The IrqController already performs the
    // false→true edge detection and only calls us once per VBlank,
    // so we can set the edge flag unconditionally here.
    nmiEdge_ = true;
}

void SnesCpu::SetIrqLevel(bool active) {
    irqLine_ = active;
}

// Bus interface overrides

void SnesCpu::idle() {
    addClocks(6);
}

uint8_t SnesCpu::read(uint32_t address) {
    uint8_t speed = bus_.Speed(address);
    addClocks(speed);

    uint8_t data = bus_.Read(address);
    openBus_ = data;
    regs().mdr = data;
    return data;
}

void SnesCpu::write(uint32_t address, uint8_t data) {
    uint8_t speed = bus_.Speed(address);
    addClocks(speed);

    bus_.Write(address, data);
    openBus_ = data;
    regs().mdr = data;
}

void SnesCpu::lastCycle() {
    // Sample NMI/IRQ edges at the end of the last cycle of each instruction.
    pollInterrupts();
}

bool SnesCpu::interruptPending() const {
    return nmiPending_ || (irqLine_ && !(regs().p & FlagI));
}

// Internal helpers

void SnesCpu::addClocks(uint32_t clocks) {
    cycles_ += clocks;
    instructionClocks_ += clocks;
}

void SnesCpu::pollInterrupts() {
    // NMI edge detection
    if (nmiEdge_) {
        nmiPending_ = true;
        nmiEdge_ = false;
    }

    // IRQ level detection (only fires if I flag is clear, but we record the
    // pending state here; the check at Step() gates on I).
    irqPending_ = irqLine_;
}

// DRAM Refresh — 40-master-clock penalty per scanline
//
// bsnes performs 5 sub-steps of (6 halt + 2 interleave) = 40 total clocks.
// Between each sub-step, the ALU pipeline (multiply/divide) is advanced.
//
// The dramRefreshState_ variable mirrors bsnes status.dramRefresh:
//   0 = not refreshing
//   1 = bus halted (6 clocks)  — coprocessors see the bus as unavailable
//   2 = interleave  (2 clocks) — brief gap between halt phases

void SnesCpu::ApplyDramRefreshPenalty() {
    for (int i = 0; i < 5; i++) {
        dramRefreshState_ = 1;
        addClocks(6);
        dramRefreshState_ = 2;
        addClocks(2);
        if (onAluStep_) onAluStep_();
    }
    dramRefreshState_ = 0;
}

} // namespace snes::core
