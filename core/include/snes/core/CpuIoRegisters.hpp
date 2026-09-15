// snes emulator
// core/include/snes/core/CpuIoRegisters.hpp
// CPU I/O register interface.

#pragma once
// CpuIoRegisters.hpp — SNES CPU I/O register state ($4200-$421F, $4016-$4017)
//
// These are the CPU-internal I/O registers, separate from WRAM, DMA, PPU, and
// APU.  Following bsnes's CPU::IO struct and readCPU/writeCPU in io.cpp.
//
// This class owns only the register *state*.  It exposes Read/Write methods
// that the MemoryBus maps to $00-3F,$80-BF:$4016-$4017,$4200-$421F.
//
// Callbacks are used for cross-component interactions (NMI enable changes,
// MEMSEL toggling, DMA/HDMA enable, PPU counter latch, joypad latch/data).
// These are set by the system integrator when wiring subsystems together.
//
// Reference: bsnes sfc/cpu/cpu.hpp (IO struct), sfc/cpu/io.cpp

#include <cstdint>
#include <functional>

namespace snes::core {

// Callback typedefs for cross-component interaction

/// Called when $4200 (NMITIMEN) is written — passes the new data byte
using NmitimenCallback = std::function<void(uint8_t data)>;

/// Called when $420D (MEMSEL) is written — passes new fastROM flag
using MemselCallback = std::function<void(bool fast)>;

/// Called when $420B (MDMAEN) is written — passes the channel enable bitmask
using DmaEnableCallback = std::function<void(uint8_t channels)>;

/// Called when $420C (HDMAEN) is written — passes the channel enable bitmask
using HdmaEnableCallback = std::function<void(uint8_t channels)>;

/// Called when $4201 (WRIO) bit 7 falls — triggers PPU counter latch
using PpuLatchCallback = std::function<void()>;

/// Called after every $4201 write with the new PIO value.
using PioCallback = std::function<void(uint8_t pio)>;

/// Called to read joypad serial data (port 0 or 1)
using JoypadDataCallback = std::function<uint8_t(int port)>;

/// Called to strobe joypad latch ($4016 write)
using JoypadLatchCallback = std::function<void(bool latch)>;

/// Called to query timing state for $4212 HVBJOY
struct TimingQuery {
    uint16_t hcounter = 0;   // horizontal position in master clocks
    uint16_t vcounter = 0;   // current V scanline counter
    uint16_t vblankStart = 225; // first VBlank scanline (225 or 240)
};
using TimingQueryCallback = std::function<TimingQuery()>;

/// Called on $4210 RDNMI read — returns NMI flag (replaces simple flag)
using RdnmiCallback = std::function<bool()>;

/// Called on $4211 TIMEUP read — returns IRQ flag (replaces simple flag)
using TimeupCallback = std::function<bool()>;

/// Called when $4207-$420A H/V timer targets change
using HVTimeChangeCallback = std::function<void(uint16_t htime, uint16_t vtime)>;

// CpuIoRegisters — CPU-internal I/O register state
class CpuIoRegisters {
public:
    CpuIoRegisters();
    ~CpuIoRegisters() = default;

    /// Reset all registers to power-on state
    void Reset();

    // Bus read/write — these are called by MemoryBus handlers
    // The openBus parameter is the CPU I/O MDR value.

    /// Read from CPU I/O register.  addr is the full 24-bit address;
    /// only bits 15:0 are examined.  openBus is the CPU I/O MDR.
    uint8_t Read(uint32_t addr, uint8_t openBus);

    /// Write to CPU I/O register.
    void Write(uint32_t addr, uint8_t data);

    // Callback setters — called by system integrator
    void SetNmitimenCallback(NmitimenCallback cb)   { onNmitimen_ = std::move(cb); }
    void SetMemselCallback(MemselCallback cb)       { onMemsel_ = std::move(cb); }
    void SetDmaEnableCallback(DmaEnableCallback cb) { onDmaEnable_ = std::move(cb); }
    void SetHdmaEnableCallback(HdmaEnableCallback cb) { onHdmaEnable_ = std::move(cb); }
    void SetPpuLatchCallback(PpuLatchCallback cb)   { onPpuLatch_ = std::move(cb); }
    void SetPioCallback(PioCallback cb) { onPio_ = std::move(cb); }
    void SetPioInputCallback(std::function<uint8_t()> cb) { onPioInput_ = std::move(cb); }
    void SetJoypadDataCallback(JoypadDataCallback cb) { onJoypadData_ = std::move(cb); }
    void SetJoypadLatchCallback(JoypadLatchCallback cb) { onJoypadLatch_ = std::move(cb); }
    void SetTimingQueryCallback(TimingQueryCallback cb) { onTimingQuery_ = std::move(cb); }
    void SetRdnmiCallback(RdnmiCallback cb) { onRdnmi_ = std::move(cb); }
    void SetTimeupCallback(TimeupCallback cb) { onTimeup_ = std::move(cb); }
    void SetHVTimeChangeCallback(HVTimeChangeCallback cb) { onHVTimeChange_ = std::move(cb); }

    // Direct access for other subsystems (NMI/IRQ, auto-joypad, timing)

    // $4200 NMITIMEN decomposed flags
    bool nmiEnabled()   const noexcept { return nmiEnable_; }
    bool hIrqEnabled()  const noexcept { return hirqEnable_; }
    bool vIrqEnabled()  const noexcept { return virqEnable_; }
    bool irqEnabled()   const noexcept { return irqEnable_; }
    bool autoJoypadPoll() const noexcept { return autoJoypadPoll_; }

    // $4201 WRIO
    uint8_t pio() const noexcept { return pio_; }

    // $4207-$420A H/V timer targets
    uint16_t htime() const noexcept { return htime_; }
    uint16_t vtime() const noexcept { return vtime_; }

    // $420D MEMSEL
    bool fastRom() const noexcept { return fastRom_; }

    // $4214-$4217 multiply/divide results (readable by other subsystems)
    uint16_t rddiv() const noexcept { return rddiv_; }
    uint16_t rdmpy() const noexcept { return rdmpy_; }

    // $4218-$421F auto-joypad results — writable by the auto-joypad polling logic
    void setJoy1(uint16_t v) noexcept { joy1_ = v; }
    void setJoy2(uint16_t v) noexcept { joy2_ = v; }
    void setJoy3(uint16_t v) noexcept { joy3_ = v; }
    void setJoy4(uint16_t v) noexcept { joy4_ = v; }

    // NMI/IRQ flags — set by timing subsystem, read-and-clear by $4210/$4211
    void setNmiFlag(bool v) noexcept { nmiFlag_ = v; }
    bool nmiFlag() const noexcept { return nmiFlag_; }

    void setIrqFlag(bool v) noexcept { irqFlag_ = v; }
    bool irqFlag() const noexcept { return irqFlag_; }

    // Auto-joypad busy counter — managed by timing subsystem
    void setAutoJoypadCounter(uint8_t v) noexcept { autoJoypadCounter_ = v; }
    uint8_t autoJoypadCounter() const noexcept { return autoJoypadCounter_; }

    // CPU version (for $4210 RDNMI low nibble)
    void setCpuVersion(uint8_t v) noexcept { cpuVersion_ = v; }

    // Clock once per CPU bus/idle cycle or refresh step. For writes, preserve
    // the busy signal sampled before the arithmetic edge completes, including
    // refresh while DMA holds the pending write. Each write replaces the sample.
    void AluStep(bool writeCycle = false);

private:
    // Register state — following bsnes CPU::IO

    // $4200 NMITIMEN
    bool nmiEnable_      = false;
    bool hirqEnable_     = false;
    bool virqEnable_     = false;
    bool irqEnable_      = false; // hirq || virq
    bool autoJoypadPoll_ = false;

    // $4201 WRIO — programmable I/O port
    uint8_t pio_ = 0xFF;

    // $4202-$4203 — multiply operands
    uint8_t wrmpya_ = 0xFF;
    uint8_t wrmpyb_ = 0xFF;

    // $4204-$4206 — divide operands
    uint16_t wrdiva_ = 0xFFFF;
    uint8_t  wrdivb_ = 0xFF;

    // $4207-$420A — H/V timer targets
    // bsnes stores htime as ((9-bit value + 1) << 2) for dot comparison.
    // We store the raw 9-bit values and convert when needed.
    uint16_t htime_ = 0x1FF;
    uint16_t vtime_ = 0x1FF;

    // $420D — MEMSEL (fast ROM)
    bool fastRom_ = false;

    // $4214-$4217 — multiply/divide results
    uint16_t rddiv_ = 0;
    uint16_t rdmpy_ = 0;

    // $4218-$421F — auto-joypad read results
    uint16_t joy1_ = 0;
    uint16_t joy2_ = 0;
    uint16_t joy3_ = 0;
    uint16_t joy4_ = 0;

    // NMI/IRQ flags (read-and-clear via $4210/$4211)
    bool nmiFlag_ = false;
    bool irqFlag_ = false;

    // Auto-joypad busy counter (33 = inactive, <33 = polling)
    uint8_t autoJoypadCounter_ = 33;

    // CPU chip version (1 or 2, typically 2)
    uint8_t cpuVersion_ = 2;

    // ALU state for cycle-accurate multiply/divide
    struct Alu {
        uint32_t mpyctr = 0;  // multiply cycles remaining (8 down to 0)
        uint32_t divctr = 0;  // divide cycles remaining (16 down to 0)
        uint32_t shift  = 0;  // working shift register
        bool busyOnWrite = false;
    } alu_{};

    // Joypad strobe latch state
    bool joypadLatch_ = false;

    // Callbacks
    NmitimenCallback    onNmitimen_;
    MemselCallback      onMemsel_;
    DmaEnableCallback   onDmaEnable_;
    HdmaEnableCallback  onHdmaEnable_;
    PpuLatchCallback    onPpuLatch_;
    PioCallback         onPio_;
    std::function<uint8_t()> onPioInput_;
    JoypadDataCallback  onJoypadData_;
    JoypadLatchCallback onJoypadLatch_;
    TimingQueryCallback onTimingQuery_;
    RdnmiCallback       onRdnmi_;
    TimeupCallback      onTimeup_;
    HVTimeChangeCallback onHVTimeChange_;
};

} // namespace snes::core
