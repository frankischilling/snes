// snes emulator
// core/include/snes/core/Dma.hpp
// DMA and HDMA controller interface.

#pragma once
// Dma.hpp — SNES DMA/HDMA controller (8 channels)
//
// The SNES has 8 DMA channels, each with its own set of registers at
// $4300-$437F (16 bytes per channel, though only $43x0-$43xB + $43xF used).
//
// This class owns the channel register state and provides:
//  - Read/Write for the $4300-$437F register range (mapped by MemoryBus)
//  - GP-DMA transfer engine (modes 0-7)
//  - Channel state accessors for the HDMA transfer engine (TODO #15)
//  - Reset to power-on defaults
//
// Reference: bsnes sfc/cpu/cpu.hpp (Channel struct), sfc/cpu/io.cpp
//            (readDMA / writeDMA), sfc/cpu/dma.cpp

#include <cstdint>
#include <functional>

namespace snes::core {

class MemoryBus;  // forward — needed for DMA transfer engine

// DmaChannel — per-channel register state
//
// Matches bsnes CPU::Channel register layout:
//   $43x0  DMAPx    — control (direction, HDMA indirect, pattern, etc.)
//   $43x1  BBADx    — B-bus target address (low byte of $21xx)
//   $43x2  A1TxL    — A-bus source address low
//   $43x3  A1TxH    — A-bus source address high
//   $43x4  A1Bx     — A-bus source bank
//   $43x5  DASxL    — transfer size low  (GP-DMA) / indirect addr low  (HDMA)
//   $43x6  DASxH    — transfer size high (GP-DMA) / indirect addr high (HDMA)
//   $43x7  DASBx    — HDMA indirect bank
//   $43x8  A2AxL    — HDMA table address low
//   $43x9  A2AxH    — HDMA table address high
//   $43xA  NTRLx    — HDMA line counter
//   $43xB  ???x     — unused / unknown register (readable/writable)
//   $43xF           — mirror of $43xB
struct DmaChannel {
    // $43x0 DMAPx — control register
    uint8_t transferMode    = 7;  // bits 2-0: transfer pattern (0-7)
    bool    fixedTransfer   = true;  // bit 3: 1 = A-bus address not incremented
    bool    reverseTransfer = true;  // bit 4: 0 = increment, 1 = decrement (when not fixed)
    bool    unused          = true;  // bit 5: unused (still readable/writable)
    bool    indirect        = true;  // bit 6: HDMA indirect mode
    bool    direction       = true;  // bit 7: 0 = A→B (CPU→PPU), 1 = B→A (PPU→CPU)

    // $43x1 BBADx — B-bus address (low byte; CPU reads/writes $2100|targetAddress)
    uint8_t targetAddress = 0xFF;

    // $43x2-$43x3 A1TxL/H — A-bus source address (16-bit)
    uint16_t sourceAddress = 0xFFFF;

    // $43x4 A1Bx — A-bus source bank
    uint8_t sourceBank = 0xFF;

    // $43x5-$43x6 DASxL/H — byte count (GP-DMA) or indirect address (HDMA)
    //
    // bsnes uses a union; both fields share the same 16 bits.
    // For GP-DMA: transferSize is the number of bytes to transfer (0 = 65536).
    // For HDMA:   indirectAddress is read from the HDMA table.
    uint16_t transferSize = 0xFFFF;  // alias: indirectAddress

    // Convenience accessors for the HDMA alias
    uint16_t indirectAddress() const noexcept { return transferSize; }
    void setIndirectAddress(uint16_t v) noexcept { transferSize = v; }

    // $43x7 DASBx — HDMA indirect bank
    uint8_t indirectBank = 0xFF;

    // $43x8-$43x9 A2AxL/H — HDMA table current address
    uint16_t hdmaAddress = 0xFFFF;

    // $43xA NTRLx — HDMA line counter
    //   bit 7: repeat flag
    //   bits 6-0: line count
    uint8_t lineCounter = 0xFF;

    // $43xB / $43xF — unknown register (readable/writable, $43xF mirrors it)
    uint8_t unknown = 0xFF;

    // Non-register state — managed by $420B/$420C writes and transfer engine
    bool dmaEnable  = false;  // set by $420B, cleared after transfer completes
    bool hdmaEnable = false;  // set by $420C

    // HDMA internal state
    bool hdmaCompleted  = false;  // set when line counter reaches 0
    bool hdmaDoTransfer = false;  // set when a transfer should occur this scanline

    // Pack/unpack $43x0 DMAPx register
    uint8_t readControl() const noexcept {
        return static_cast<uint8_t>(
            (transferMode & 7)
          | (fixedTransfer   ? 0x08 : 0)
          | (reverseTransfer ? 0x10 : 0)
          | (unused          ? 0x20 : 0)
          | (indirect        ? 0x40 : 0)
          | (direction       ? 0x80 : 0)
        );
    }

    void writeControl(uint8_t data) noexcept {
        transferMode    = data & 7;
        fixedTransfer   = (data >> 3) & 1;
        reverseTransfer = (data >> 4) & 1;
        unused          = (data >> 5) & 1;
        indirect        = (data >> 6) & 1;
        direction       = (data >> 7) & 1;
    }

    // Reset to power-on defaults (all 0xFF per bsnes)
    void Reset() noexcept {
        writeControl(0xFF);
        targetAddress = 0xFF;
        sourceAddress = 0xFFFF;
        sourceBank    = 0xFF;
        transferSize  = 0xFFFF;
        indirectBank  = 0xFF;
        hdmaAddress   = 0xFFFF;
        lineCounter   = 0xFF;
        unknown       = 0xFF;
        dmaEnable     = false;
        hdmaEnable    = false;
        hdmaCompleted = false;
        hdmaDoTransfer = false;
    }
};

// DmaController — owns 8 DMA channels + register I/O + transfer engine
class DmaController {
public:
    DmaController();
    ~DmaController() = default;

    /// Reset all 8 channels to power-on state.
    void Reset();

    // Bus connection — must be set before any transfers can occur.
    // The MemoryBus pointer is non-owning and must outlive this object.
    void SetBus(MemoryBus* bus) noexcept { bus_ = bus; }
    MemoryBus* Bus() const noexcept { return bus_; }

    // Register read/write — $4300-$437F
    //
    // These are mapped onto the bus by MemoryBus::MapDma().
    // The openBus parameter is the CPU I/O MDR for unrecognized addresses.
    uint8_t Read(uint32_t addr, uint8_t openBus);
    void    Write(uint32_t addr, uint8_t data);

    // Channel access
    DmaChannel&       Channel(int n)       noexcept { return channels_[n & 7]; }
    const DmaChannel& Channel(int n) const noexcept { return channels_[n & 7]; }

    // Convenience: any channel enabled?
    bool AnyDmaEnabled()  const noexcept;
    bool AnyHdmaEnabled() const noexcept;
    bool AnyHdmaActive()  const noexcept;

    // $420B/$420C write helpers — called by CpuIoRegisters callbacks
    void EnableDma(uint8_t channelMask);
    void EnableHdma(uint8_t channelMask);

    // GP-DMA transfer engine
    //
    // RunDma() executes all enabled DMA channels in priority order (0-7).
    // Returns the total number of master clock cycles consumed.
    //
    // Each byte transferred costs 8 master cycles (two 4-cycle bus accesses).
    // Each channel has an 8-cycle overhead. The entire DMA operation has an
    // 8-cycle initialization overhead.
    //
    // After completion, dmaEnable is cleared for each channel and
    // transferSize is 0x0000.
    uint32_t RunDma();

    // HDMA lifecycle
    //
    // HDMA is horizontal-blank DMA — automatic per-scanline transfers
    // driven by a table in A-bus memory.  The lifecycle is:
    //
    //   Frame start (V=0):
    //     1. HdmaReset()  — clear completed/doTransfer flags
    //     2. HdmaSetup()  — initialize table pointers, read first entry
    //
    //   Each visible scanline (~H=1104):
    //     3. HdmaRun()    — transfer data, then advance to next table entry
    //
    // Each method returns the number of master clock cycles consumed.
    //
    // Reference: bsnes sfc/cpu/dma.cpp

    /// Called at frame start — clears hdmaCompleted and hdmaDoTransfer.
    /// No bus access; returns 0 cycles.
    void HdmaReset();

    /// Called at frame start after HdmaReset — initializes HDMA channels.
    /// For each enabled channel: copies sourceAddress → hdmaAddress,
    /// sets lineCounter to 0, reads the first table entry.
    /// Cancels any active GP-DMA on that channel.
    /// Returns master cycles consumed (8 overhead + per-channel reads).
    uint32_t HdmaSetup();

    /// Called at each visible scanline — performs HDMA transfers and advances.
    /// For each active channel: transfers data bytes based on transfer mode,
    /// then decrements line counter, reloads table entry if count expired.
    /// Returns master cycles consumed (8 overhead + per-channel reads/writes).
    uint32_t HdmaRun();

    // A-bus address validation
    static bool ValidA(uint32_t address) noexcept;

    // WRAM-to-WRAM transfer validity check
    static bool ValidWramTransfer(uint8_t bBusAddr, uint32_t aBusAddr) noexcept;

    // Transfer mode byte counts (used by both GP-DMA and HDMA)
    //
    // Mode: 0  1  2  3  4  5  6  7
    // Len:  1  2  2  4  4  4  2  4
    static constexpr uint8_t kTransferLengths[8] = {1, 2, 2, 4, 4, 4, 2, 4};

private:
    DmaChannel channels_[8];
    MemoryBus* bus_ = nullptr;

    // Internal transfer helpers (shared by GP-DMA and HDMA)

    /// Read from A-bus (24-bit address). Returns 0x00 if address invalid.
    uint8_t ReadA(uint32_t address);

    /// Write to A-bus. Ignored if address invalid.
    void WriteA(uint32_t address, uint8_t data);

    /// Read from B-bus ($2100 | addr8). Returns 0x00 if !valid.
    uint8_t ReadB(uint8_t address, bool valid);

    /// Write to B-bus ($2100 | addr8). Ignored if !valid.
    void WriteB(uint8_t address, uint8_t data, bool valid);

    /// Execute one transfer unit (A→B or B→A) for a channel.
    /// index selects the B-bus address offset based on transfer mode.
    void Transfer(DmaChannel& ch, uint32_t addressA, uint8_t index);

    /// Execute GP-DMA for a single channel. Returns cycles consumed.
    uint32_t RunChannelDma(DmaChannel& ch);

    // HDMA internal helpers

    /// Check if any channel with index > channelIdx is still active.
    /// Used for early-exit optimization in indirect table reload.
    /// Reference: bsnes Channel::hdmaFinished()
    bool HdmaFinished(int channelIdx) const noexcept;

    /// Reload HDMA table entry for a channel.
    /// Reads line counter; if lower 7 bits were 0, loads new counter and
    /// optionally reads indirect address.  Returns cycles consumed.
    /// Reference: bsnes Channel::hdmaReload()
    uint32_t HdmaReload(DmaChannel& ch, int channelIdx);

    /// Execute HDMA transfer for a single channel.
    /// Transfers kTransferLengths[mode] bytes using either direct or
    /// indirect addressing.  Returns cycles consumed.
    /// Reference: bsnes Channel::hdmaTransfer()
    uint32_t HdmaTransfer(DmaChannel& ch);

    /// Advance HDMA to next scanline — decrement line counter, set repeat
    /// flag, reload table if needed.  Returns cycles consumed.
    /// Reference: bsnes Channel::hdmaAdvance()
    uint32_t HdmaAdvance(DmaChannel& ch, int channelIdx);
};

} // namespace snes::core
