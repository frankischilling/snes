// ============================================================================
// Smp.cpp — SNES Sound Module Processor Implementation
//
// Provides the SPC700's bus interface: 64KB RAM, IPL ROM overlay,
// I/O registers ($00F0-$00FF), and bidirectional CPU communication ports.
//
// Reference: bsnes sfc/smp/io.cpp, sfc/smp/memory.cpp, sfc/smp/smp.cpp
// ============================================================================

#include "snes/core/Smp.hpp"
#include "snes/core/Dsp.hpp"
#include <cstdlib>
#include <cstdio>
#include <cstring>

namespace snes::core {

namespace {
bool SmpPortTraceEnabled() {
    static int enabled = -1;
    if (enabled == -1) {
        const char* v = std::getenv("SNES_SMP_PORT_TRACE");
        enabled = (v && *v && *v != '0') ? 1 : 0;
    }
    return enabled == 1;
}
}

// ============================================================================
// IPL boot ROM — 64 bytes loaded at $FFC0-$FFFF when enabled
// This is the standard SPC700 boot ROM that handles the initial CPU↔APU
// handshake and data transfer protocol.
// ============================================================================
static constexpr uint8_t kIplRom[64] = {
    0xCD, 0xEF, 0xBD, 0xE8, 0x00, 0xC6, 0x1D, 0xD0,
    0xFC, 0x8F, 0xAA, 0xF4, 0x8F, 0xBB, 0xF5, 0x78,
    0xCC, 0xF4, 0xD0, 0xFB, 0x2F, 0x19, 0xEB, 0xF4,
    0xD0, 0xFC, 0x7E, 0xF4, 0xD0, 0x0B, 0xE4, 0xF5,
    0xCB, 0xF4, 0xD7, 0x00, 0xFC, 0xD0, 0xF3, 0xAB,
    0x01, 0x10, 0xEF, 0x7E, 0xF4, 0x10, 0xEB, 0xBA,
    0xF6, 0xDA, 0x00, 0xBA, 0xF4, 0xC4, 0xF4, 0xDD,
    0x5D, 0xD0, 0xDB, 0x1F, 0x00, 0x00, 0xC0, 0xFF,
};

// ============================================================================
// Timer implementation
// ============================================================================

template <unsigned Frequency>
void Smp::Timer<Frequency>::Step(unsigned clocks,
                                 bool timersEnable, bool timersDisable)
{
    // Stage 0: accumulate clocks
    stage0 += static_cast<uint8_t>(clocks);
    if (stage0 < Frequency) return;
    stage0 -= static_cast<uint8_t>(Frequency);

    // Stage 1: toggle pulse (square wave)
    stage1 ^= 1;
    SynchronizeStage1(timersEnable, timersDisable);
}

template <unsigned Frequency>
void Smp::Timer<Frequency>::SynchronizeStage1(bool timersEnable,
                                               bool timersDisable)
{
    // Gate the stage1 signal with global enable/disable from TEST register
    bool level = (stage1 != 0);
    if (!timersEnable) level = false;
    if (timersDisable) level = false;

    // Falling-edge detection: only trigger on 1→0 transition
    bool fallingEdge = line && !level;
    line = level;
    if (!fallingEdge) return;

    // Stage 2: divider counter (only advances when timer is enabled)
    if (!enable) return;
    stage2++;
    if (stage2 != target) return;

    // Stage 3: 4-bit output counter
    stage2 = 0;
    stage3 = (stage3 + 1) & 0x0F;
}

// Explicit template instantiations
template struct Smp::Timer<128>;
template struct Smp::Timer<16>;

// ============================================================================
// Constructor / Power
// ============================================================================

Smp::Smp() {
    std::memcpy(iplRom_.data(), kIplRom, IplRomSize);
    Power();
}

void Smp::Power() {
    // Reset the SPC700 processor core
    Spc700::Power();

    // Clear RAM
    ram_.fill(0);

    // Reset I/O state to power-on defaults
    io_ = IO{};  // uses default member initializers

    // Clear port latches
    apuInput_.fill(0);
    cpuOutput_.fill(0);

    // Reset timers
    timer0_ = {};
    timer1_ = {};
    timer2_ = {};

    // Reset DSP sample clock
    dspClock_ = 0;
}

// ============================================================================
// Bus interface — Spc700 overrides
// ============================================================================

void Smp::Idle() {
    cycles_++;
    // Default idle: step timers by 2 ticks (standard internal wait)
    StepTimers(2);
    tickDsp();
}

uint8_t Smp::Read(uint16_t address) {
    cycles_++;
    // Step timers — internal addresses ($00F0-$00FF, IPL ROM) use internal
    // wait states, external addresses use external wait states.
    // For simplicity, default to 2 ticks (standard wait state 0).
    StepTimers(2);
    tickDsp();

    uint8_t data = readRam(address);
    // I/O registers at $00F0-$00FF override RAM reads
    if ((address & 0xFFF0) == 0x00F0) {
        data = readIO(address);
    }
    return data;
}

void Smp::Write(uint16_t address, uint8_t data) {
    cycles_++;
    // Step timers
    StepTimers(2);
    tickDsp();

    // Writes always go to underlying RAM (even in I/O region)
    writeRam(address, data);
    // I/O registers at $00F0-$00FF get additional handling
    if ((address & 0xFFF0) == 0x00F0) {
        writeIO(address, data);
    }
}

// ============================================================================
// DSP sample clock — called once per bus cycle.
// Every 32 bus cycles, trigger one DSP sample.
// ============================================================================

void Smp::tickDsp() {
    if (++dspClock_ >= kDspSampleInterval) {
        dspClock_ = 0;
        if (dsp_) dsp_->RunSample();
    }
}

// ============================================================================
// Batch execution — run until CycleCount() >= targetCycles
// ============================================================================

void Smp::RunUntil(uint64_t targetCycles) {
    while (cycles_ < targetCycles && !r.wait && !r.stop) {
        Step();
    }
    // If halted but target not reached, advance cycle count to target
    if (cycles_ < targetCycles) {
        cycles_ = targetCycles;
    }
}

// ============================================================================
// Timer stepping — advance all three timers by the given clock ticks
// ============================================================================

void Smp::StepTimers(unsigned clocks) {
    timer0_.Step(clocks, io_.timersEnable, io_.timersDisable);
    timer1_.Step(clocks, io_.timersEnable, io_.timersDisable);
    timer2_.Step(clocks, io_.timersEnable, io_.timersDisable);
}

// ============================================================================
// CPU-side port access
// ============================================================================

uint8_t Smp::PortRead(uint8_t port) const {
    // CPU reads $2140+n → returns what SMP wrote to $F4+n
    return cpuOutput_[port & 3];
}

void Smp::PortWrite(uint8_t port, uint8_t data) {
    // CPU writes $2140+n → sets data for SMP to read from $F4+n
    apuInput_[port & 3] = data;
}

// ============================================================================
// Internal RAM access (with IPL ROM overlay)
// ============================================================================

uint8_t Smp::readRam(uint16_t address) const {
    // IPL ROM overlay: $FFC0-$FFFF when enabled
    if (address >= 0xFFC0 && io_.iplRomEnable) {
        return iplRom_[address & 0x3F];
    }
    if (io_.ramDisable) {
        return 0x5A;  // RAM disabled returns $5A (bsnes behavior)
    }
    return ram_[address];
}

void Smp::writeRam(uint16_t address, uint8_t data) {
    // Writes always go to RAM (even under IPL ROM overlay)
    if (io_.ramWritable && !io_.ramDisable) {
        ram_[address] = data;
    }
}

// ============================================================================
// I/O register read ($00F0-$00FF)
// ============================================================================

uint8_t Smp::readIO(uint16_t address) {
    switch (address) {
    case 0x00F0:  // TEST — write-only
        return 0x00;

    case 0x00F1:  // CONTROL — write-only
        return 0x00;

    case 0x00F2:  // DSPADDR
        return io_.dspAddr;

    case 0x00F3:  // DSPDATA — read DSP register at DSPADDR
        return dsp_ ? dsp_->Read(io_.dspAddr & 0x7F) : 0x00;

    case 0x00F4:  // CPUIO0 — reads what CPU wrote
        return apuInput_[0];

    case 0x00F5:  // CPUIO1
        return apuInput_[1];

    case 0x00F6:  // CPUIO2
        return apuInput_[2];

    case 0x00F7:  // CPUIO3
        return apuInput_[3];

    case 0x00F8:  // AUXIO4
        return io_.aux4;

    case 0x00F9:  // AUXIO5
        return io_.aux5;

    case 0x00FA:  // T0TARGET — write-only
    case 0x00FB:  // T1TARGET — write-only
    case 0x00FC:  // T2TARGET — write-only
        return 0x00;

    case 0x00FD: {  // T0OUT — read-and-clear (4-bit)
        uint8_t val = timer0_.stage3;
        timer0_.stage3 = 0;
        return val;
    }

    case 0x00FE: {  // T1OUT — read-and-clear (4-bit)
        uint8_t val = timer1_.stage3;
        timer1_.stage3 = 0;
        return val;
    }

    case 0x00FF: {  // T2OUT — read-and-clear (4-bit)
        uint8_t val = timer2_.stage3;
        timer2_.stage3 = 0;
        return val;
    }

    default:
        return 0x00;
    }
}

// ============================================================================
// I/O register write ($00F0-$00FF)
// ============================================================================

void Smp::writeIO(uint16_t address, uint8_t data) {
    switch (address) {
    case 0x00F0:  // TEST
        // Writes only valid when P flag is clear (bsnes behavior)
        if (r.p.p) break;
        io_.timersDisable      = (data & 0x01) != 0;
        io_.ramWritable        = (data & 0x02) != 0;
        io_.ramDisable         = (data & 0x04) != 0;
        io_.timersEnable       = (data & 0x08) != 0;
        io_.externalWaitStates = (data >> 4) & 0x03;
        io_.internalWaitStates = (data >> 6) & 0x03;
        // Re-evaluate timer gate signals immediately
        timer0_.SynchronizeStage1(io_.timersEnable, io_.timersDisable);
        timer1_.SynchronizeStage1(io_.timersEnable, io_.timersDisable);
        timer2_.SynchronizeStage1(io_.timersEnable, io_.timersDisable);
        break;

    case 0x00F1:  // CONTROL
        // Bits 4-5: clear input ports (CPU→SMP latches)
        if (data & 0x10) {
            apuInput_[0] = 0x00;
            apuInput_[1] = 0x00;
        }
        if (data & 0x20) {
            apuInput_[2] = 0x00;
            apuInput_[3] = 0x00;
        }

        // Bit 7: IPL ROM enable/disable
        io_.iplRomEnable = (data & 0x80) != 0;

        // Bits 0-2: Timer enable with 0→1 edge detection
        // Rising edge (newly enabled) resets stage2 and stage3
        {
            bool wasEnabled0 = timer0_.enable;
            timer0_.enable = (data & 0x01) != 0;
            if (!wasEnabled0 && timer0_.enable) {
                timer0_.stage2 = 0;
                timer0_.stage3 = 0;
            }

            bool wasEnabled1 = timer1_.enable;
            timer1_.enable = (data & 0x02) != 0;
            if (!wasEnabled1 && timer1_.enable) {
                timer1_.stage2 = 0;
                timer1_.stage3 = 0;
            }

            bool wasEnabled2 = timer2_.enable;
            timer2_.enable = (data & 0x04) != 0;
            if (!wasEnabled2 && timer2_.enable) {
                timer2_.stage2 = 0;
                timer2_.stage3 = 0;
            }
        }
        break;

    case 0x00F2:  // DSPADDR
        io_.dspAddr = data;
        break;

    case 0x00F3:  // DSPDATA — write DSP register at DSPADDR
        if (dsp_ && !(io_.dspAddr & 0x80))  // $80-$FF are read-only mirrors
            dsp_->Write(io_.dspAddr & 0x7F, data);
        break;

    case 0x00F4:  // CPUIO0 — SMP writes output for CPU to read
    {
        static int port0WriteCount = 0;
        if (SmpPortTraceEnabled() && data != cpuOutput_[0] && ++port0WriteCount <= 200) {
            fprintf(stderr, "[SMP-PORT0 #%d cyc=%llu] PC=%04X wrote %02X (was %02X)\n",
                    port0WriteCount,
                    static_cast<unsigned long long>(cycles_), r.pc, data, cpuOutput_[0]);
        }
        cpuOutput_[0] = data;
        break;
    }

    case 0x00F5:  // CPUIO1
        cpuOutput_[1] = data;
        break;

    case 0x00F6:  // CPUIO2
        cpuOutput_[2] = data;
        break;

    case 0x00F7:  // CPUIO3
        cpuOutput_[3] = data;
        break;

    case 0x00F8:  // AUXIO4
        io_.aux4 = data;
        break;

    case 0x00F9:  // AUXIO5
        io_.aux5 = data;
        break;

    case 0x00FA:  // T0TARGET
        timer0_.target = data;
        break;

    case 0x00FB:  // T1TARGET
        timer1_.target = data;
        break;

    case 0x00FC:  // T2TARGET
        timer2_.target = data;
        break;

    case 0x00FD:  // T0OUT — read-only
    case 0x00FE:  // T1OUT — read-only
    case 0x00FF:  // T2OUT — read-only
        break;
    }
}

} // namespace snes::core
