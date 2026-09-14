// snes emulator
// core/include/snes/core/MemoryBus.hpp
// Unified 24-bit address bus interface.

#pragma once
// MemoryBus.hpp — Unified 24-bit SNES address space dispatch
//
// The SNES presents a flat 24-bit (16 MB) address space to the CPU.
// Different hardware components (ROM, WRAM, PPU, APU, CPU I/O, DMA) are
// mapped to specific bank:offset regions.
//
// This class uses a 16 MB lookup table (one byte per address) to map each
// address to a handler slot.  Each slot holds a pair of read/write callbacks.
// This makes Read()/Write() O(1) with zero branching — just two array
// lookups and one indirect call.
//
// The bus also owns the 128 KB WRAM, open-bus MDR, CPU I/O registers
// ($4200-$421F), WMDATA ($2180-$2183), and bus-speed timing.
//
// Following bsnes's Bus architecture in sfc/memory/memory.hpp.

#include "snes/core/Cpu65816.hpp"  // ICpuBus
#include <array>
#include <cstdint>
#include <functional>
#include <memory>

namespace snes::core {

class Cartridge;
class CpuIoRegisters;
class DmaController;
class Ppu;
class Smp;

// Handler callback signatures
//   ReadHandler:  (addr24, openBusMdr) → data
//   WriteHandler: (addr24, data) → void
using BusReadHandler  = std::function<uint8_t(uint32_t addr, uint8_t openBus)>;
using BusWriteHandler = std::function<void(uint32_t addr, uint8_t data)>;

// MemoryBus — 24-bit address space with fast lookup dispatch
class MemoryBus : public ICpuBus {
public:
    MemoryBus();
    ~MemoryBus() override;

    // Non-copyable (owns 16 MB+ of heap data)
    MemoryBus(const MemoryBus&) = delete;
    MemoryBus& operator=(const MemoryBus&) = delete;

    // ICpuBus interface — these are what SnesCpu calls
    uint8_t Read(uint32_t address) override;
    void    Write(uint32_t address, uint8_t value) override;
    uint8_t Speed(uint32_t address) const override;

    // Handler registration

    /// Register a read/write handler pair and get back a slot ID (1-255).
    /// Returns 0 on failure (all slots occupied).
    uint8_t RegisterHandler(BusReadHandler reader, BusWriteHandler writer);

    /// Map a rectangular region of the address space to a handler slot.
    /// bankLo/bankHi are inclusive bank numbers (0x00-0xFF).
    /// addrLo/addrHi are inclusive offsets within each bank (0x0000-0xFFFF).
    void MapRange(uint8_t bankLo, uint8_t bankHi,
                  uint16_t addrLo, uint16_t addrHi,
                  uint8_t slotId);

    /// Convenience: register + map in one call.  Returns the slot ID.
    uint8_t Map(uint8_t bankLo, uint8_t bankHi,
                uint16_t addrLo, uint16_t addrHi,
                BusReadHandler reader, BusWriteHandler writer);

    /// Map multiple disjoint bank ranges to the same handler slot.
    /// Useful for WRAM mirrors, PPU, APU, etc.
    void MapRegions(std::initializer_list<
                        std::tuple<uint8_t, uint8_t, uint16_t, uint16_t>
                    > regions, uint8_t slotId);

    // Standard SNES component mapping helpers

    /// Map a Cartridge's ROM and SRAM into the bus.  Must be called after
    /// Reset() or whenever the cartridge changes.
    void MapCartridge(Cartridge& cart);

    /// Map the 128 KB WRAM to its standard bus locations.
    void MapWram();

    /// Map CPU I/O registers ($4016-$4017, $4200-$421F) into the bus.
    /// The CpuIoRegisters object must outlive the bus mapping.
    void MapCpuIo(CpuIoRegisters& cpuIo);

    /// Map DMA channel registers ($4300-$437F) into the bus.
    /// The DmaController object must outlive the bus mapping.
    void MapDma(DmaController& dma);

    /// Map PPU registers ($2100-$213F) into the bus.
    /// The Ppu object must outlive the bus mapping.
    void MapPpu(Ppu& ppu);

    /// Map APU communication ports ($2140-$217F) into the bus.
    /// The Smp object must outlive the bus mapping.
    /// $2140-$217F are all mirrors of 4 ports (addr & 3).
    void MapApu(Smp& smp);

    /// Full reset: clear all mappings, reset WRAM, reset I/O state.
    void Reset();

    // WRAM direct access (for DMA, WMDATA, testing)
    uint8_t* WramData() noexcept { return wram_.data(); }
    const uint8_t* WramData() const noexcept { return wram_.data(); }
    static constexpr size_t WramSize = 0x20000; // 128 KB

    // CPU I/O state — MEMSEL (bus speed) flag
    void SetFastRom(bool enabled) noexcept { fastRom_ = enabled; }
    bool FastRom() const noexcept { return fastRom_; }

    // Open bus MDR
    //
    // The SNES has two open-bus behaviors:
    //  1. Global MDR: updated on every bus read EXCEPT CPU internal I/O
    //     ($00-3F,$80-BF:$4000-$43FF).  Unmapped addresses return this.
    //  2. CPU I/O MDR: updated ONLY when reading $4000-$43FF.  Each
    //     register handler in that range receives this as the open-bus
    //     fallback, and only drives the bits it owns (leaving the rest
    //     as the CPU I/O MDR value).
    //
    // Check: (address & 0x40fc00) == 0x4000  → CPU I/O region
    // Reference: bsnes sfc/cpu/memory.cpp line 44
    uint8_t OpenBus() const noexcept { return mdr_; }
    void SetOpenBus(uint8_t v) noexcept { mdr_ = v; }

    /// CPU I/O open-bus MDR ($4000-$43FF region)
    uint8_t CpuIoMdr() const noexcept { return cpuIoMdr_; }
    void SetCpuIoMdr(uint8_t v) noexcept { cpuIoMdr_ = v; }

    // WMDATA address register ($2181-$2183 write, $2180 read/write)
    uint32_t WmdataAddress() const noexcept { return wmdataAddr_; }

private:
    // 16 MB flat lookup: address → handler slot (0 = unmapped / open bus)
    static constexpr size_t kAddressSpace = 16 * 1024 * 1024; // 0x100'0000
    std::unique_ptr<uint8_t[]> lookup_;

    // Handler slots (slot 0 is the open-bus fallback)
    static constexpr int kMaxSlots = 256;
    BusReadHandler  readers_[kMaxSlots];
    BusWriteHandler writers_[kMaxSlots];
    int nextSlot_ = 1; // next free slot (0 is reserved for open bus)

    // 128 KB WRAM
    std::array<uint8_t, WramSize> wram_{};

    // WMDATA address auto-increment register
    uint32_t wmdataAddr_ = 0;

    // Open bus — global Memory Data Register
    uint8_t mdr_ = 0;

    // CPU I/O MDR — separate latch for $4000-$43FF reads
    uint8_t cpuIoMdr_ = 0;

    // CPU I/O: MEMSEL ($420D) bit 0 — ROM access speed toggle
    bool fastRom_ = false;

    // Internal handler IDs cached for WRAM so we can re-map quickly
    uint8_t wramSlotLo_ = 0;  // for bank mirrors ($00-3F,$80-BF:0000-1FFF)
    uint8_t wramSlotFull_ = 0; // for full banks ($7E-7F)
    uint8_t wmdataSlot_ = 0;   // for WMDATA register

    // Built-in handlers

    // WRAM handlers — low mirror (8KB per bank, 0000-1FFF)
    uint8_t readWramLow(uint32_t addr, uint8_t openBus);
    void    writeWramLow(uint32_t addr, uint8_t data);

    // WRAM handlers — full 128KB banks ($7E-7F)
    uint8_t readWramFull(uint32_t addr, uint8_t openBus);
    void    writeWramFull(uint32_t addr, uint8_t data);

    // WMDATA register ($2180 R/W, $2181-$2183 W)
    uint8_t readWmdata(uint32_t addr, uint8_t openBus);
    void    writeWmdata(uint32_t addr, uint8_t data);
};

} // namespace snes::core
