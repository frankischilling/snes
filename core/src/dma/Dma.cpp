// snes emulator
// core/src/dma/Dma.cpp
// General DMA and HDMA channel state and transfers.

// Dma.cpp — SNES DMA controller register I/O + GP-DMA transfer engine
//
// Register read/write follows bsnes:
//   Channel index = (addr >> 4) & 7
//   Register id   = addr & 0xFF8F  (masks out channel bits)
//
// Transfer engine follows bsnes sfc/cpu/dma.cpp:
//   - 8 transfer modes (0-7) with B-bus address pattern
//   - A-bus validation (no B-bus or CPU I/O access)
//   - WRAM-to-WRAM invalid check
//   - Direction: 0 = A→B (CPU→PPU), 1 = B→A (PPU→CPU)
//   - Source address auto-increment/decrement/fixed
//   - Transfer size decrements to 0 (0 means 65536 bytes)
//
// Reference: bsnes sfc/cpu/io.cpp readDMA / writeDMA, sfc/cpu/dma.cpp

#include "snes/core/Dma.hpp"
#include "snes/core/MemoryBus.hpp"
#include "snes/core/Cartridge.hpp"

namespace snes::core {

namespace {
struct RunningGuard {
    bool& running;
    explicit RunningGuard(bool& value) : running(value) { running = true; }
    ~RunningGuard() { running = false; }
};
}

void DmaController::Clock(uint32_t clocks) {
    dmaClocks_ += clocks;
    if (onClock_) onClock_(clocks);
}

uint32_t DmaController::AlignToDma() {
    if (!clockQuery_) return 0;
    const auto clocks = uint32_t(8 - (clockQuery_() & 7));
    Clock(clocks);
    return clocks;
}

uint32_t DmaController::ResumeCpu(uint64_t startClocks) {
    if (!clockQuery_ || cpuCycleClocks_ == 0) return 0;
    const auto clocks = cpuCycleClocks_ - uint32_t((dmaClocks_ - startClocks) % cpuCycleClocks_);
    Clock(clocks);
    return clocks;
}

// Construction / Reset
DmaController::DmaController() {
    Reset();
}

void DmaController::Reset() {
    for (int i = 0; i < 8; ++i) {
        channels_[i].Reset();
    }
    dmaClocks_ = 0;
    cpuCycleClocks_ = 6;
    inDma_ = false;
    inHdma_ = false;
}

// Register read — $4300-$437F
//
// bsnes: addr >> 4 & 7 selects channel, addr & 0xFF8F selects register.
// Unrecognized addresses return open bus.
uint8_t DmaController::Read(uint32_t addr, uint8_t openBus) {
    const int ch = (addr >> 4) & 7;
    const auto& c = channels_[ch];

    switch (addr & 0xFF8F) {
    // $43x0 DMAPx — control
    case 0x4300:
        return c.readControl();

    // $43x1 BBADx — B-bus target address
    case 0x4301:
        return c.targetAddress;

    // $43x2 A1TxL — source address low
    case 0x4302:
        return static_cast<uint8_t>(c.sourceAddress);

    // $43x3 A1TxH — source address high
    case 0x4303:
        return static_cast<uint8_t>(c.sourceAddress >> 8);

    // $43x4 A1Bx — source bank
    case 0x4304:
        return c.sourceBank;

    // $43x5 DASxL — transfer size low / indirect address low
    case 0x4305:
        return static_cast<uint8_t>(c.transferSize);

    // $43x6 DASxH — transfer size high / indirect address high
    case 0x4306:
        return static_cast<uint8_t>(c.transferSize >> 8);

    // $43x7 DASBx — indirect bank
    case 0x4307:
        return c.indirectBank;

    // $43x8 A2AxL — HDMA table address low
    case 0x4308:
        return static_cast<uint8_t>(c.hdmaAddress);

    // $43x9 A2AxH — HDMA table address high
    case 0x4309:
        return static_cast<uint8_t>(c.hdmaAddress >> 8);

    // $43xA NTRLx — HDMA line counter
    case 0x430A:
        return c.lineCounter;

    // $43xB ???x — unknown register (bsnes mirrors this at $43xF too)
    case 0x430B:
    case 0x430F:
        return c.unknown;

    default:
        return openBus;
    }
}

// Register write — $4300-$437F
void DmaController::Write(uint32_t addr, uint8_t data) {
    const int ch = (addr >> 4) & 7;
    auto& c = channels_[ch];

    switch (addr & 0xFF8F) {
    // $43x0 DMAPx — control
    case 0x4300:
        c.writeControl(data);
        break;

    // $43x1 BBADx
    case 0x4301:
        c.targetAddress = data;
        break;

    // $43x2 A1TxL
    case 0x4302:
        c.sourceAddress = (c.sourceAddress & 0xFF00) | data;
        break;

    // $43x3 A1TxH
    case 0x4303:
        c.sourceAddress = (c.sourceAddress & 0x00FF) | (static_cast<uint16_t>(data) << 8);
        break;

    // $43x4 A1Bx
    case 0x4304:
        c.sourceBank = data;
        break;

    // $43x5 DASxL
    case 0x4305:
        c.transferSize = (c.transferSize & 0xFF00) | data;
        break;

    // $43x6 DASxH
    case 0x4306:
        c.transferSize = (c.transferSize & 0x00FF) | (static_cast<uint16_t>(data) << 8);
        break;

    // $43x7 DASBx
    case 0x4307:
        c.indirectBank = data;
        break;

    // $43x8 A2AxL
    case 0x4308:
        c.hdmaAddress = (c.hdmaAddress & 0xFF00) | data;
        break;

    // $43x9 A2AxH
    case 0x4309:
        c.hdmaAddress = (c.hdmaAddress & 0x00FF) | (static_cast<uint16_t>(data) << 8);
        break;

    // $43xA NTRLx
    case 0x430A:
        c.lineCounter = data;
        break;

    // $43xB / $43xF ??? — unknown
    case 0x430B:
    case 0x430F:
        c.unknown = data;
        break;

    default:
        // Writes to unmapped DMA registers are ignored
        break;
    }
}

// Query helpers
bool DmaController::AnyDmaEnabled() const noexcept {
    for (int i = 0; i < 8; ++i) {
        if (channels_[i].dmaEnable) return true;
    }
    return false;
}

bool DmaController::AnyHdmaEnabled() const noexcept {
    for (int i = 0; i < 8; ++i) {
        if (channels_[i].hdmaEnable) return true;
    }
    return false;
}

bool DmaController::AnyHdmaActive() const noexcept {
    for (int i = 0; i < 8; ++i) {
        if (channels_[i].hdmaEnable && !channels_[i].hdmaCompleted) return true;
    }
    return false;
}

// $420B write — enable GP-DMA channels
//
// Each bit enables the corresponding channel.  After setting these flags,
// the caller should invoke RunDma() to execute the transfers.
void DmaController::EnableDma(uint8_t channelMask) {
    for (int i = 0; i < 8; ++i) {
        channels_[i].dmaEnable = (channelMask >> i) & 1;
    }
}

// $420C write — enable HDMA channels
void DmaController::EnableHdma(uint8_t channelMask) {
    for (int i = 0; i < 8; ++i) {
        channels_[i].hdmaEnable = (channelMask >> i) & 1;
    }
}

// HDMA reset — called at the start of each frame (V=0)
void DmaController::HdmaReset() {
    for (int i = 0; i < 8; ++i) {
        channels_[i].hdmaCompleted  = false;
        channels_[i].hdmaDoTransfer = false;
    }
}

// A-bus address validation
//
// DMA A-bus reads/writes cannot access certain address ranges.
// bsnes bit-trick checks from dma.cpp Channel::validA():
bool DmaController::ValidA(uint32_t address) noexcept {
    // B-bus region: $00-3F,$80-BF:$2100-$21FF
    if ((address & 0x40ff00) == 0x2100) return false;
    // Old CPU I/O: $00-3F,$80-BF:$4000-$41FF
    if ((address & 0x40fe00) == 0x4000) return false;
    // CPU I/O regs: $00-3F,$80-BF:$4200-$421F
    if ((address & 0x40ffe0) == 0x4200) return false;
    // DMA regs: $00-3F,$80-BF:$4300-$437F
    if ((address & 0x40ff80) == 0x4300) return false;
    return true;
}

// WRAM-to-WRAM transfer validity
//
// Transfers where B-bus target is $80 (→ $2180 = WMDATA) and the A-bus
// address resolves to WRAM are invalid.  On real hardware, the result is
// undefined; bsnes suppresses the write.
//
// WRAM addresses:
//   $7E0000-$7FFFFF  (banks $7E-$7F full)
//   $00-3F,$80-BF:$0000-$1FFF  (low mirror)
//
// bsnes check:
//   valid = addressB != 0x80
//        || ((addressA & 0xfe0000) != 0x7e0000
//         && (addressA & 0x40e000) != 0x0000)
bool DmaController::ValidWramTransfer(uint8_t bBusAddr, uint32_t aBusAddr) noexcept {
    if (bBusAddr != 0x80) return true;
    // B-bus is $2180 (WMDATA) — check if A-bus also points to WRAM
    bool isWramFull = (aBusAddr & 0xFE0000) == 0x7E0000;
    bool isWramLow  = (aBusAddr & 0x40E000) == 0x0000;
    return !isWramFull && !isWramLow;
}

// A-bus read — reads from the 24-bit A-bus address
//
// If the address is invalid (B-bus or CPU I/O region), returns 0x00.
// Updates the bus MDR (open bus).
// On real hardware this takes 8 master cycles (two 4-cycle half-accesses).
uint8_t DmaController::ReadA(uint32_t address) {
    Clock(4);
    const auto value = ValidA(address) ? bus_->Read(address) : uint8_t(0);
    bus_->SetOpenBus(value);
    Clock(4);
    return value;
}

// A-bus write — writes to the 24-bit A-bus address
// Ignored if address is invalid.
void DmaController::WriteA(uint32_t address, uint8_t data) {
    if (ValidA(address)) {
        bus_->Write(address, data);
    }
}

// B-bus read — reads from $2100 | address (8-bit B-bus address)
// If !valid (WRAM-to-WRAM), returns 0x00.
uint8_t DmaController::ReadB(uint8_t address, bool valid) {
    Clock(4);
    const auto value = valid ? bus_->Read(0x2100 | address) : uint8_t(0);
    bus_->SetOpenBus(value);
    Clock(4);
    return value;
}

// B-bus write — writes to $2100 | address
// Ignored if !valid.
void DmaController::WriteB(uint8_t address, uint8_t data, bool valid) {
    if (valid) {
        bus_->Write(0x2100 | address, data);
    }
}

// Transfer — execute one transfer unit for a channel
//
// The B-bus address is derived from targetAddress + an offset that depends
// on the transfer mode and the current index within the transfer pattern:
//
//   Mode 0: offset = 0               (1 register:  p)
//   Mode 1: offset = index & 1       (2 registers: p, p+1)
//   Mode 2: offset = 0               (1 register:  p, p)       [2 bytes same]
//   Mode 3: offset = (index >> 1)&1  (2 registers: p,p, p+1,p+1)
//   Mode 4: offset = index & 3       (4 registers: p, p+1, p+2, p+3)
//   Mode 5: offset = index & 1       (same as 1)
//   Mode 6: offset = 0               (same as 2)
//   Mode 7: offset = (index >> 1)&1  (same as 3)
//
// Reference: bsnes dma.cpp Channel::transfer()
void DmaController::Transfer(DmaChannel& ch, uint32_t addressA, uint8_t index, const uint8_t* source) {
    uint8_t addressB = ch.targetAddress;

    switch (ch.transferMode) {
    case 0:                                          break; // +0
    case 1: case 5: addressB += (index & 1);         break; // +0, +1
    case 2: case 6:                                  break; // +0, +0
    case 3: case 7: addressB += ((index >> 1) & 1);  break; // +0, +0, +1, +1
    case 4:         addressB += (index & 3);          break; // +0, +1, +2, +3
    default:                                         break;
    }

    // WRAM-to-WRAM validity check
    bool valid = ValidWramTransfer(addressB, addressA);

    if (!ch.direction) {
        // Direction 0: A→B (CPU → PPU / register)
        uint8_t data;
        if (source) {
            Clock(4);
            data = *source;
            bus_->SetOpenBus(data);
            Clock(4);
        } else {
            data = ReadA(addressA);
        }
        WriteB(addressB, data, valid);
    } else {
        // Direction 1: B→A (PPU / register → CPU)
        uint8_t data = ReadB(addressB, valid);
        WriteA(addressA, data);
    }
}

// RunChannelDma — execute GP-DMA for one channel
//
// bsnes dma.cpp Channel::dmaRun():
//   - 8 cycle overhead per channel
//   - Loop: transfer one unit, update source address, decrement transferSize
//   - Continue while dmaEnable && --transferSize != 0
//   - Clear dmaEnable when done
//
// Returns master clock cycles consumed by this channel.
uint32_t DmaController::RunChannelDma(DmaChannel& ch, unsigned channel) {
    if (!ch.dmaEnable) return 0;

    uint32_t cycles = 8; // per-channel overhead
    Clock(8);
    if (onBoundary_) onBoundary_();
    if (!ch.dmaEnable) return cycles;

    const auto decoded = cartridge_ ? cartridge_->BeginDma(channel,
        (uint32_t(ch.sourceBank) << 16) | ch.sourceAddress,
        ch.transferSize, ch.fixedTransfer, ch.direction) : std::vector<uint8_t>{};
    size_t sourceIndex = 0;

    uint8_t index = 0;
    do {
        uint32_t aBusAddr = (static_cast<uint32_t>(ch.sourceBank) << 16) | ch.sourceAddress;
        Transfer(ch, aBusAddr, index, decoded.empty() ? nullptr : &decoded[sourceIndex++]);
        cycles += 8; // 8 master cycles per byte transferred

        // Advance index within the transfer pattern
        index++;

        // Update A-bus source address (unless fixed)
        if (!ch.fixedTransfer) {
            if (!ch.reverseTransfer) {
                ch.sourceAddress++;
            } else {
                ch.sourceAddress--;
            }
        }
        --ch.transferSize;
        if (onBoundary_) onBoundary_();
    } while (ch.dmaEnable && ch.transferSize);

    ch.dmaEnable = false;
    if (!decoded.empty()) cartridge_->EndDma(channel);
    return cycles;
}

// RunDma — execute all enabled GP-DMA channels
//
// bsnes dma.cpp CPU::dmaRun():
//   - 8 cycle global overhead
//   - Process channels 0-7 in priority order
//   - Each channel runs to completion before the next starts
//
// Returns total master clock cycles consumed.
uint32_t DmaController::RunDma() {
    if (!bus_ || !AnyDmaEnabled() || inDma_ || inHdma_) return 0;

    uint32_t totalCycles = 0;
    {
        RunningGuard running(inDma_);
        const auto start = dmaClocks_;
        totalCycles = AlignToDma() + 8;
        Clock(8);
        if (onBoundary_) onBoundary_();
        for (int i = 0; i < 8; ++i) {
            totalCycles += RunChannelDma(channels_[i], unsigned(i));
        }
        totalCycles += ResumeCpu(start);
    }
    return totalCycles;
}

//
//  HDMA — Horizontal-blank DMA (per-scanline automatic transfers)
//

// HdmaFinished — check if all channels after channelIdx are done
//
// bsnes Channel::hdmaFinished(): walks the linked list of channels after
// this one, returns true if none are active.  We use array index instead.
// Used as an optimization in indirect mode reload — if no later channel
// is active, we can skip the second indirect address read on a completed
// channel.  (In practice this is a minor optimization but matches bsnes.)
bool DmaController::HdmaFinished(int channelIdx) const noexcept {
    for (int i = channelIdx + 1; i < 8; ++i) {
        if (channels_[i].hdmaEnable && !channels_[i].hdmaCompleted) {
            return false;
        }
    }
    return true;
}

// HdmaReload — reload HDMA table entry for a channel
//
// bsnes Channel::hdmaReload():
//   1. Read byte from [sourceBank:hdmaAddress] (always — even if not reloading)
//   2. If lower 7 bits of lineCounter == 0:
//      a. lineCounter = data read above
//      b. hdmaAddress++
//      c. If new lineCounter == 0 → hdmaCompleted, doTransfer = false
//      d. Else → hdmaDoTransfer = true
//      e. If indirect mode:
//         - Read low byte of indirect address
//         - Read high byte (unless completed and no later channels active)
//
// Returns master cycles consumed (8 per A-bus read).
uint32_t DmaController::HdmaReload(DmaChannel& ch, int channelIdx) {
    uint32_t cycles = 0;

    // Always read from the HDMA table position
    uint32_t tableAddr = (static_cast<uint32_t>(ch.sourceBank) << 16) | ch.hdmaAddress;
    uint8_t data = ReadA(tableAddr);
    cycles += 8;

    // Only reload if the lower 7 bits of the line counter have reached 0
    if ((ch.lineCounter & 0x7F) == 0) {
        ch.lineCounter = data;
        ch.hdmaAddress++;

        ch.hdmaCompleted  = (ch.lineCounter == 0);
        ch.hdmaDoTransfer = !ch.hdmaCompleted;

        if (ch.indirect) {
            // Read low byte of indirect address
            tableAddr = (static_cast<uint32_t>(ch.sourceBank) << 16) | ch.hdmaAddress;
            ch.hdmaAddress++;
            data = ReadA(tableAddr);
            cycles += 8;

            // A full reload uses this byte as the low half of the address.
            uint16_t indLo = data;

            // Early exit: if completed and no later channel is active, skip
            // the second indirect address read
            if (ch.hdmaCompleted && HdmaFinished(channelIdx)) {
                // The final channel performs only one address read. That
                // byte occupies the high half; the terminator supplies zero.
                ch.setIndirectAddress(static_cast<uint16_t>(indLo << 8));
                return cycles;
            }

            // Read high byte of indirect address
            tableAddr = (static_cast<uint32_t>(ch.sourceBank) << 16) | ch.hdmaAddress;
            ch.hdmaAddress++;
            data = ReadA(tableAddr);
            cycles += 8;

            ch.setIndirectAddress(static_cast<uint16_t>(data << 8) | indLo);
        }
    }

    return cycles;
}

// HdmaTransfer — execute HDMA data transfer for a single channel
//
// bsnes Channel::hdmaTransfer():
//   - Skip if not active (hdmaEnable && !hdmaCompleted)
//   - Cancel any GP-DMA on this channel
//   - Skip if !hdmaDoTransfer (line counter hasn't expired yet)
//   - Transfer kTransferLengths[mode] bytes
//   - Direct mode: data source is [sourceBank:hdmaAddress++]
//   - Indirect mode: data source is [indirectBank:indirectAddress++]
//
// Returns master cycles consumed (8 per byte transferred).
uint32_t DmaController::HdmaTransfer(DmaChannel& ch) {
    if (!(ch.hdmaEnable && !ch.hdmaCompleted)) return 0;

    // HDMA stops any active GP-DMA on this channel
    ch.dmaEnable = false;

    if (!ch.hdmaDoTransfer) return 0;

    uint32_t cycles = 0;
    uint8_t length = kTransferLengths[ch.transferMode & 7];

    for (uint8_t index = 0; index < length; ++index) {
        uint32_t address;
        if (!ch.indirect) {
            // Direct mode: data comes from HDMA table itself
            address = (static_cast<uint32_t>(ch.sourceBank) << 16) | ch.hdmaAddress;
            ch.hdmaAddress++;
        } else {
            // Indirect mode: data comes from [indirectBank:indirectAddress]
            address = (static_cast<uint32_t>(ch.indirectBank) << 16) | ch.indirectAddress();
            ch.setIndirectAddress(ch.indirectAddress() + 1);
        }

        Transfer(ch, address, index);
        cycles += 8;
    }

    return cycles;
}

// HdmaAdvance — advance HDMA to next scanline
//
// bsnes Channel::hdmaAdvance():
//   - Skip if not active
//   - Decrement lineCounter
//   - hdmaDoTransfer = (lineCounter & 0x80) — the repeat flag
//   - Call hdmaReload() — reads next table entry if counter expired
//
// Returns master cycles consumed.
uint32_t DmaController::HdmaAdvance(DmaChannel& ch, int channelIdx) {
    if (!(ch.hdmaEnable && !ch.hdmaCompleted)) return 0;

    ch.lineCounter--;
    ch.hdmaDoTransfer = (ch.lineCounter & 0x80) != 0;

    return HdmaReload(ch, channelIdx);
}

// HdmaSetup — called at frame start (V=0) after HdmaReset
//
// bsnes CPU::hdmaSetup():
//   - 8 cycle overhead
//   - For each channel:
//     - hdmaDoTransfer = true
//     - If not hdmaEnable, skip
//     - Cancel GP-DMA
//     - hdmaAddress = sourceAddress (initialize table pointer)
//     - lineCounter = 0 (force reload on first hdmaReload)
//     - hdmaReload() reads the first table entry
//
// Returns master cycles consumed.
uint32_t DmaController::HdmaSetup() {
    if (!bus_ || inHdma_ || !AnyHdmaEnabled()) return 0;

    RunningGuard running(inHdma_);
    const auto start = dmaClocks_;
    uint32_t totalCycles = (inDma_ ? 0 : AlignToDma()) + 8;
    Clock(8);

    for (int i = 0; i < 8; ++i) {
        auto& ch = channels_[i];

        ch.hdmaDoTransfer = true; // note: set even for disabled channels (bsnes behavior)

        if (!ch.hdmaEnable) continue;

        ch.dmaEnable = false;  // HDMA cancels GP-DMA
        ch.hdmaAddress = ch.sourceAddress;
        ch.lineCounter = 0;    // force a reload

        totalCycles += HdmaReload(ch, i);
    }

    if (!inDma_) totalCycles += ResumeCpu(start);
    return totalCycles;
}

// HdmaRun — called at each visible scanline (~H=1104)
//
// bsnes CPU::hdmaRun():
//   - 8 cycle overhead
//   - First pass: hdmaTransfer() for each channel (priority order 0-7)
//   - Second pass: hdmaAdvance() for each channel (priority order 0-7)
//
// The two-pass approach ensures all transfers happen before any channel
// advances to the next table entry.
//
// Returns master cycles consumed.
uint32_t DmaController::HdmaRun() {
    if (!bus_ || !AnyHdmaActive() || inHdma_) return 0;

    RunningGuard running(inHdma_);
    const auto start = dmaClocks_;
    uint32_t totalCycles = (inDma_ ? 0 : AlignToDma()) + 8;
    Clock(8);

    // Pass 1: transfer data for each active channel
    for (int i = 0; i < 8; ++i) {
        auto& ch = channels_[i];
        totalCycles += HdmaTransfer(ch);
    }

    // Pass 2: advance each active channel to next scanline
    for (int i = 0; i < 8; ++i) {
        totalCycles += HdmaAdvance(channels_[i], i);
    }

    if (!inDma_) totalCycles += ResumeCpu(start);
    return totalCycles;
}

} // namespace snes::core
