#pragma once
// ============================================================================
// SnesCpu.hpp — SNES-specific CPU wrapper
//
// Inherits the pure Processor65816 core and implements:
//   - Virtual bus: read/write/idle with SNES memory-map timing
//   - NMI / IRQ state machine (edge-detect NMI, level-detect IRQ)
//   - lastCycle() for interrupt edge sampling
//   - Cycle counter
//   - DMA hooks (stubbed for now)
//   - DRAM refresh (stubbed for now)
//
// The old ICpuBus interface is preserved as a simple intermediary so that
// existing tests and the Emulator can plug in a bus implementation.
// ============================================================================

#include "snes/core/Processor65816.hpp"
#include "snes/core/Cpu65816.hpp"  // For ICpuBus
#include <cstdint>
#include <functional>

namespace snes::core {

// ---------------------------------------------------------------------------
// SnesCpu — SNES 65816 CPU
// ---------------------------------------------------------------------------
class SnesCpu : public Processor65816 {
public:
    explicit SnesCpu(ICpuBus& bus);
    ~SnesCpu() override = default;

    // Non-copyable
    SnesCpu(const SnesCpu&) = delete;
    SnesCpu& operator=(const SnesCpu&) = delete;

    // -----------------------------------------------------------------------
    // System interface
    // -----------------------------------------------------------------------

    /// Initialize CPU state and load reset vector.
    void Reset();

    /// Execute one instruction and return the number of master clocks consumed.
    uint32_t Step();

    /// External interrupt requests
    void RequestNmi();
    void SetIrqLevel(bool active);

    /// Total master clocks elapsed
    uint64_t Cycles() const noexcept { return cycles_; }

    // -----------------------------------------------------------------------
    // DRAM refresh
    // -----------------------------------------------------------------------

    /// Apply the 40-master-clock DRAM refresh penalty.
    /// Called by the Timing onDramRefresh callback once per scanline.
    /// Internally performed as 5 × (6+2) sub-steps matching bsnes.
    /// An optional ALU step callback is invoked between each sub-step to
    /// advance the multiply/divide hardware pipeline.
    void ApplyDramRefreshPenalty();

    /// Set the callback invoked during each DRAM refresh sub-step to
    /// advance the ALU multiply/divide hardware (CpuIoRegisters::AluStep).
    using AluStepCallback = std::function<void()>;
    void SetAluStepCallback(AluStepCallback cb) { onAluStep_ = std::move(cb); }

    /// Returns true while DRAM refresh bus-halt is active (state 1).
    /// Coprocessors can query this to know when the bus is halted.
    bool DramRefreshActive() const noexcept { return dramRefreshState_ == 1; }

    /// Returns raw DRAM refresh state: 0=idle, 1=bus-halt, 2=interleave.
    uint8_t DramRefreshState() const noexcept { return dramRefreshState_; }

    // -----------------------------------------------------------------------
    // Bus interface overrides  (Processor65816 virtuals)
    // -----------------------------------------------------------------------
    void idle() override;
    uint8_t read(uint32_t address) override;
    void write(uint32_t address, uint8_t data) override;
    void lastCycle() override;
    bool interruptPending() const override;

private:
    // Clock accounting — adds master clocks
    void addClocks(uint32_t clocks);

    // NMI / IRQ state
    void pollInterrupts();

    ICpuBus& bus_;
    uint64_t cycles_ = 0;
    uint32_t instructionClocks_ = 0;  // Clocks for current instruction

    // NMI edge detection
    bool nmiLine_      = false;  // Current NMI pin state
    bool nmiEdge_      = false;  // Pending NMI edge (set on low→high transition)
    bool nmiPending_   = false;  // NMI will be serviced after current instruction

    // IRQ level detection
    bool irqLine_      = false;  // Current IRQ pin state (active = true)
    bool irqPending_   = false;  // IRQ will be serviced after current instruction

    // Open-bus MDR (for unmapped reads)
    uint8_t openBus_   = 0;

    // DRAM refresh state: 0=idle, 1=bus-halt (6 clocks), 2=interleave (2 clocks)
    uint8_t dramRefreshState_ = 0;

    // ALU step callback (called 5× during DRAM refresh, once per sub-step)
    AluStepCallback onAluStep_;
};

} // namespace snes::core
