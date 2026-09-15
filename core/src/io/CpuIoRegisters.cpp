// snes emulator
// core/src/io/CpuIoRegisters.cpp
// CPU I/O registers, controller ports, interrupts, and DMA controls.

// CpuIoRegisters.cpp — SNES CPU I/O register implementation
//
// Handles reads/writes to $4016-$4017 and $4200-$421F.
// Reference: bsnes sfc/cpu/io.cpp (readCPU / writeCPU)

#include "snes/core/CpuIoRegisters.hpp"

namespace snes::core {

// Construction

CpuIoRegisters::CpuIoRegisters() {
    Reset();
}

// Reset — power-on state matching bsnes cpu.cpp power()

void CpuIoRegisters::Reset() {
    // $4200 NMITIMEN
    nmiEnable_      = false;
    hirqEnable_     = false;
    virqEnable_     = false;
    irqEnable_      = false;
    autoJoypadPoll_ = false;

    // $4201 WRIO
    pio_ = 0xFF;

    // $4202-$4203
    wrmpya_ = 0xFF;
    wrmpyb_ = 0xFF;

    // $4204-$4206
    wrdiva_ = 0xFFFF;
    wrdivb_ = 0xFF;

    // $4207-$420A  (bsnes stores htime as ((val+1)<<2), we store raw 9-bit)
    htime_ = 0x1FF;
    vtime_ = 0x1FF;

    // $420D
    fastRom_ = false;

    // $4214-$4217
    rddiv_ = 0;
    rdmpy_ = 0;

    // $4218-$421F
    joy1_ = joy2_ = joy3_ = joy4_ = 0;

    // NMI/IRQ flags
    nmiFlag_ = false;
    irqFlag_ = false;

    // Auto-joypad
    autoJoypadCounter_ = 33;

    // ALU
    alu_ = {};

    // Joypad
    joypadLatch_ = false;
}

// Read — $4016-$4017, $4200-$421F
//
// Following bsnes readCPU().  The `openBus` parameter is the CPU I/O MDR;
// registers that only drive some bits preserve the remaining open-bus bits.
// Unrecognized addresses return openBus unchanged (pure open bus).

uint8_t CpuIoRegisters::Read(uint32_t addr, uint8_t openBus) {
    uint8_t data = openBus;

    switch (addr & 0xFFFF) {

    // $4016 JOYSER0 — serial joypad port 0
    case 0x4016:
        data &= 0xFC;   // bits 7-2 = open bus
        if (onJoypadData_) {
            data |= onJoypadData_(0) & 0x03;
        }
        return data;

    // $4017 JOYSER1 — serial joypad port 1
    case 0x4017:
        data &= 0xE0;   // bits 7-5 = open bus
        data |= 0x1C;   // bits 4-2 = pulled high (GND pins)
        if (onJoypadData_) {
            data |= onJoypadData_(1) & 0x03;
        }
        return data;

    // $4210 RDNMI — NMI flag + CPU version (read-and-clear NMI)
    //   bit 7: NMI flag
    //   bits 6-4: open bus
    //   bits 3-0: CPU version (typically 2)
    case 0x4210: {
        bool flag;
        if (onRdnmi_) {
            flag = onRdnmi_();  // Delegate to IrqController with hold-aware clear
        } else {
            flag = nmiFlag_;
            nmiFlag_ = false;   // Simple read-and-clear fallback
        }
        data &= 0x70;   // preserve open-bus bits 6-4
        data |= (flag ? 0x80 : 0x00);
        data |= (cpuVersion_ & 0x0F);
        return data;
    }

    // $4211 TIMEUP — IRQ flag (read-and-clear)
    //   bit 7: IRQ flag
    //   bits 6-0: open bus
    case 0x4211: {
        bool flag;
        if (onTimeup_) {
            flag = onTimeup_();  // Delegate to IrqController with hold-aware clear
        } else {
            flag = irqFlag_;
            irqFlag_ = false;   // Simple read-and-clear fallback
        }
        data &= 0x7F;
        data |= (flag ? 0x80 : 0x00);
        return data;
    }

    // $4212 HVBJOY — H/V blank + auto-joypad busy status
    //   bit 7: VBlank flag
    //   bit 6: HBlank flag
    //   bits 5-1: open bus
    //   bit 0: auto-joypad polling active
    case 0x4212: {
        data &= 0x3E;  // preserve open-bus bits 5-1
        // Auto-joypad active?
        data |= (autoJoypadPoll_ && autoJoypadCounter_ < 33) ? 0x01 : 0x00;
        // H/V blank from timing subsystem
        if (onTimingQuery_) {
            auto t = onTimingQuery_();
            bool hblank = (t.hcounter <= 2 || t.hcounter >= 1096);
            bool vblank = (t.vcounter >= t.vblankStart);
            if (hblank) data |= 0x40;
            if (vblank) data |= 0x80;
        }
        return data;
    }

    // $4213 RDIO — WRIO outputs combined with external input levels.
    case 0x4213:
        return pio_ & (onPioInput_ ? onPioInput_() : 0xff);

    // $4214-$4215 RDDIVL/RDDIVH — division result
    case 0x4214: return static_cast<uint8_t>(rddiv_ >> 0);
    case 0x4215: return static_cast<uint8_t>(rddiv_ >> 8);

    // $4216-$4217 RDMPYL/RDMPYH — multiply result (or division remainder)
    case 0x4216: return static_cast<uint8_t>(rdmpy_ >> 0);
    case 0x4217: return static_cast<uint8_t>(rdmpy_ >> 8);

    // $4218-$421F — Auto-joypad read results
    case 0x4218: return static_cast<uint8_t>(joy1_ >> 0);
    case 0x4219: return static_cast<uint8_t>(joy1_ >> 8);
    case 0x421A: return static_cast<uint8_t>(joy2_ >> 0);
    case 0x421B: return static_cast<uint8_t>(joy2_ >> 8);
    case 0x421C: return static_cast<uint8_t>(joy3_ >> 0);
    case 0x421D: return static_cast<uint8_t>(joy3_ >> 8);
    case 0x421E: return static_cast<uint8_t>(joy4_ >> 0);
    case 0x421F: return static_cast<uint8_t>(joy4_ >> 8);

    default:
        break;
    }

    // Unrecognized register — return open bus
    return data;
}

// Write — $4016, $4200-$421F
//
// Following bsnes writeCPU().
// $4017 writes are ignored (hardware behavior).

void CpuIoRegisters::Write(uint32_t addr, uint8_t data) {
    const bool busyOnWrite = alu_.busyOnWrite;
    alu_.busyOnWrite = false;
    switch (addr & 0xFFFF) {

    // $4016 JOYSER0 — joypad strobe
    //   bit 0 controls the latch line for both controller ports.
    //   $4017 writes are ignored.
    case 0x4016:
        joypadLatch_ = (data & 1) != 0;
        if (onJoypadLatch_) {
            onJoypadLatch_(joypadLatch_);
        }
        return;

    // $4200 NMITIMEN — NMI/IRQ enable, auto-joypad enable
    //   bit 7: NMI enable
    //   bit 5: V-IRQ enable
    //   bit 4: H-IRQ enable
    //   bit 0: auto-joypad enable
    case 0x4200: {
        autoJoypadPoll_ = (data & 0x01) != 0;

        // Decode IRQ/NMI enable bits
        bool oldNmiEnable = nmiEnable_;
        nmiEnable_  = (data & 0x80) != 0;
        hirqEnable_ = (data & 0x10) != 0;
        virqEnable_ = (data & 0x20) != 0;
        irqEnable_  = hirqEnable_ || virqEnable_;
        (void)oldNmiEnable;

        // Notify timing/IRQ subsystem of NMITIMEN change
        if (onNmitimen_) {
            onNmitimen_(data);
        }
        return;
    }

    // $4201 WRIO — programmable I/O port
    //   When bit 7 transitions from 1→0, latch PPU counters.
    case 0x4201:
        if ((pio_ & 0x80) && !(data & 0x80)) {
            if (onPpuLatch_) onPpuLatch_();
        }
        pio_ = data;
        if (onPio_) onPio_(pio_);
        return;

    // $4202 WRMPYA — multiplicand
    case 0x4202:
        wrmpya_ = data;
        return;

    // $4203 WRMPYB — multiplier (triggers multiplication)
    //
    // The result registers expose the shift/add operation over eight cycles.
    // A second trigger clears the accumulator but cannot restart a busy ALU.
    case 0x4203:
        rdmpy_ = 0;
        if (busyOnWrite || alu_.mpyctr || alu_.divctr) return;

        wrmpyb_ = data;
        rddiv_ = static_cast<uint16_t>((wrmpyb_ << 8) | wrmpya_);

        alu_.mpyctr = 8;
        alu_.shift = wrmpyb_;
        return;

    // $4204 WRDIVL — dividend low
    case 0x4204:
        wrdiva_ = (wrdiva_ & 0xFF00) | (static_cast<uint16_t>(data) << 0);
        return;

    // $4205 WRDIVH — dividend high
    case 0x4205:
        wrdiva_ = (wrdiva_ & 0x00FF) | (static_cast<uint16_t>(data) << 8);
        return;

    // $4206 WRDIVB — divisor (triggers division)
    //
    // Division exposes its quotient and remainder over sixteen cycles.
    case 0x4206:
        rdmpy_ = wrdiva_;  // remainder defaults to dividend
        if (busyOnWrite || alu_.mpyctr || alu_.divctr) return;

        wrdivb_ = data;

        alu_.divctr = 16;
        alu_.shift = static_cast<uint32_t>(wrdivb_) << 16;
        return;

    // $4207 HTIMEL — H-counter IRQ target (low 8 bits of 9-bit value)
    case 0x4207:
        htime_ = (htime_ & 0x100) | data;
        if (onHVTimeChange_) onHVTimeChange_(htime_, vtime_);
        return;

    // $4208 HTIMEH — H-counter IRQ target (bit 8)
    case 0x4208:
        htime_ = (htime_ & 0x0FF) | (static_cast<uint16_t>(data & 1) << 8);
        if (onHVTimeChange_) onHVTimeChange_(htime_, vtime_);
        return;

    // $4209 VTIMEL — V-counter IRQ target (low 8 bits)
    case 0x4209:
        vtime_ = (vtime_ & 0x100) | data;
        if (onHVTimeChange_) onHVTimeChange_(htime_, vtime_);
        return;

    // $420A VTIMEH — V-counter IRQ target (bit 8)
    case 0x420A:
        vtime_ = (vtime_ & 0x0FF) | (static_cast<uint16_t>(data & 1) << 8);
        if (onHVTimeChange_) onHVTimeChange_(htime_, vtime_);
        return;

    // $420B MDMAEN — General DMA enable
    //   Each bit enables one of 8 DMA channels.
    case 0x420B:
        if (onDmaEnable_) onDmaEnable_(data);
        return;

    // $420C HDMAEN — HDMA enable
    //   Each bit enables one of 8 HDMA channels.
    case 0x420C:
        if (onHdmaEnable_) onHdmaEnable_(data);
        return;

    // $420D MEMSEL — ROM access speed
    //   bit 0: 0 = 2.68 MHz (slow), 1 = 3.58 MHz (fast) for $80-FF:8000-FFFF
    case 0x420D:
        fastRom_ = (data & 1) != 0;
        if (onMemsel_) onMemsel_(fastRom_);
        return;

    default:
        break;
    }

    // Writes to $4017, $4208-$420F (unused), $4210-$421F (read-only) are ignored
}

// AluStep — advance multiply/divide by one cycle
//
// Reads and idle cycles clock the ALU after the access; writes clock it before
// the access. Refresh contributes five more edges while ordinary DMA pauses it.

void CpuIoRegisters::AluStep(bool writeCycle) {
    // Capture at each CPU write edge. DMA can delay that write across refresh,
    // whose arithmetic edges must not replace its saved acceptance decision.
    // A following CPU write takes a fresh sample even if this one targets RAM.
    if (writeCycle) alu_.busyOnWrite = alu_.mpyctr || alu_.divctr;
    if (alu_.mpyctr) {
        alu_.mpyctr--;
        if (rddiv_ & 1) {
            rdmpy_ += static_cast<uint16_t>(alu_.shift);
        }
        rddiv_ >>= 1;
        alu_.shift <<= 1;
    }

    if (alu_.divctr) {
        alu_.divctr--;
        rddiv_ <<= 1;
        alu_.shift >>= 1;
        if (rdmpy_ >= alu_.shift) {
            rdmpy_ -= static_cast<uint16_t>(alu_.shift);
            rddiv_ |= 1;
        }
    }
}

} // namespace snes::core
