// MemoryBus.cpp — Unified 24-bit SNES address space dispatch
//
// 16 MB lookup table + handler-slot architecture.
// Implements ICpuBus for SnesCpu, owns 128 KB WRAM, WMDATA register,
// bus-speed calculation, and open-bus MDR.
//
// Reference: bsnes sfc/memory/memory.cpp, sfc/cpu/memory.cpp

#include "snes/core/MemoryBus.hpp"
#include "snes/core/Cartridge.hpp"
#include "snes/core/CpuIoRegisters.hpp"
#include "snes/core/Dma.hpp"
#include "snes/core/Ppu.hpp"
#include "snes/core/Smp.hpp"
#include <algorithm>
#include <cstring>

namespace snes::core {

// Construction / Destruction

MemoryBus::MemoryBus()
    : lookup_(std::make_unique<uint8_t[]>(kAddressSpace))
{
    // Slot 0 = open bus (default for all unmapped addresses)
    readers_[0] = [](uint32_t /*addr*/, uint8_t openBus) -> uint8_t {
        return openBus;
    };
    writers_[0] = [](uint32_t /*addr*/, uint8_t /*data*/) {};

    // Clear lookup table — everything starts as slot 0 (open bus)
    std::memset(lookup_.get(), 0, kAddressSpace);
}

MemoryBus::~MemoryBus() = default;

// ICpuBus interface

uint8_t MemoryBus::Read(uint32_t address) {
    address &= 0x00FFFFFF; // mask to 24 bits
    uint8_t slot = lookup_[address];

    // CPU internal I/O ($00-3F,$80-BF:$4000-$43FF) uses its own MDR.
    // Reads from this region do NOT update the global MDR.
    // bsnes reference: sfc/cpu/memory.cpp line 44
    //   if((address & 0x40fc00) != 0x4000) r.mdr = data;
    const bool isCpuIo = (address & 0x40fc00) == 0x4000;

    // CPU I/O handlers receive cpuIoMdr_ as their open-bus value;
    // all other handlers receive the global mdr_.
    uint8_t data = readers_[slot](address, isCpuIo ? cpuIoMdr_ : mdr_);

    if (isCpuIo) {
        cpuIoMdr_ = data;
    } else {
        mdr_ = data;
    }
    return data;
}

void MemoryBus::Write(uint32_t address, uint8_t value) {
    address &= 0x00FFFFFF;
    uint8_t slot = lookup_[address];
    writers_[slot](address, value);
    mdr_ = value;
}

// Bus speed calculation
//
// Matches bsnes sfc/cpu/memory.cpp speed logic.
// Returns master clock cycles: 6 (fast), 8 (slow), 12 (xslow).
//
//   Region                                          Cycles
//   ─────────────────────────────────────────────── ──────
//   $00-3F,$80-BF : $0000-$1FFF  (WRAM low)         8
//   $00-3F,$80-BF : $2000-$3FFF  (PPU, etc.)        8
//   $00-3F,$80-BF : $4000-$41FF  (joypad/old I/O)  12
//   $00-3F,$80-BF : $4200-$43FF  (CPU I/O, DMA)     6
//   $00-3F,$80-BF : $4400-$5FFF  (unmapped)          8
//   $00-3F,$80-BF : $6000-$7FFF  (SRAM / expn)      8
//   $00-3F,$80-BF : $8000-$FFFF  (ROM)              8 (or 6 if fastROM + bank≥$80)
//   $40-7D         : $0000-$FFFF  (ROM/SRAM)         8
//   $7E-7F         : $0000-$FFFF  (WRAM)             8
//   $80-BF         : see above (mirrors $00-$3F with fast option)
//   $C0-FF         : $0000-$FFFF  (ROM)              8 (or 6 if fastROM)

uint8_t MemoryBus::Speed(uint32_t address) const {
    address &= 0x00FFFFFF;

    // bsnes bit-trick: addr & 0x40'8000 selects the "ROM area"
    // (banks $40-7D/$C0-FF full, or any bank $8000-$FFFF)
    if (address & 0x408000) {
        // ROM region — 8 slow, or 6 if high banks + fastROM enabled
        if ((address & 0x800000) && fastRom_) {
            return 6;
        }
        return 8;
    }

    // Not ROM.  We are in banks $00-3F or $80-BF with offset < $8000,
    // or banks $7E-7F (offset < $8000 — but $7E-7F full is caught above).
    // bsnes: (addr + 0x6000) & 0x4000 → offsets $2000-$3FFF or $6000-$7FFF
    if ((address + 0x6000) & 0x4000) {
        return 8; // slow: PPU/APU registers, SRAM area
    }

    // bsnes: (addr - 0x4000) & 0x7E00 → offsets $4200-$43FF
    if ((address - 0x4000) & 0x7E00) {
        return 6; // fast: CPU I/O and DMA registers
    }

    // Remaining: $4000-$41FF — extra-slow (joypad, old NES-style I/O)
    return 12;
}

// Handler registration

uint8_t MemoryBus::RegisterHandler(BusReadHandler reader, BusWriteHandler writer) {
    if (nextSlot_ >= kMaxSlots) {
        return 0; // no slots available
    }
    uint8_t id = static_cast<uint8_t>(nextSlot_++);
    readers_[id] = std::move(reader);
    writers_[id] = std::move(writer);
    return id;
}

void MemoryBus::MapRange(uint8_t bankLo, uint8_t bankHi,
                          uint16_t addrLo, uint16_t addrHi,
                          uint8_t slotId) {
    for (uint32_t bank = bankLo; bank <= bankHi; ++bank) {
        const uint32_t base = bank << 16;
        for (uint32_t addr = addrLo; addr <= addrHi; ++addr) {
            lookup_[base | addr] = slotId;
        }
    }
}

uint8_t MemoryBus::Map(uint8_t bankLo, uint8_t bankHi,
                        uint16_t addrLo, uint16_t addrHi,
                        BusReadHandler reader, BusWriteHandler writer) {
    uint8_t id = RegisterHandler(std::move(reader), std::move(writer));
    if (id == 0) return 0;
    MapRange(bankLo, bankHi, addrLo, addrHi, id);
    return id;
}

void MemoryBus::MapRegions(
    std::initializer_list<std::tuple<uint8_t, uint8_t, uint16_t, uint16_t>> regions,
    uint8_t slotId)
{
    for (auto& [bankLo, bankHi, addrLo, addrHi] : regions) {
        MapRange(bankLo, bankHi, addrLo, addrHi, slotId);
    }
}

// Reset

void MemoryBus::Reset() {
    // Clear all lookup entries to slot 0 (open bus)
    std::memset(lookup_.get(), 0, kAddressSpace);

    // Reset handler slots (keep slot 0 = open bus)
    for (int i = 1; i < kMaxSlots; ++i) {
        readers_[i] = nullptr;
        writers_[i] = nullptr;
    }
    nextSlot_ = 1;

    // Clear WRAM to 0x55/0xAA pattern (common uninitialized SNES WRAM pattern)
    // bsnes uses random or 0x55 depending on config; we use a deterministic pattern.
    for (size_t i = 0; i < WramSize; ++i) {
        wram_[i] = (i & 1) ? 0xAA : 0x55;
    }

    // Reset WMDATA address
    wmdataAddr_ = 0;

    // Reset MDR
    mdr_ = 0;
    cpuIoMdr_ = 0;

    // Reset bus speed
    fastRom_ = false;

    // Reset cached slot IDs
    wramSlotLo_ = 0;
    wramSlotFull_ = 0;
    wmdataSlot_ = 0;
}

// WRAM mapping

void MemoryBus::MapWram() {
    // Register WRAM low-mirror handler (banks $00-3F,$80-BF : $0000-$1FFF)
    wramSlotLo_ = RegisterHandler(
        [this](uint32_t addr, uint8_t /*ob*/) { return readWramLow(addr, 0); },
        [this](uint32_t addr, uint8_t data)   { writeWramLow(addr, data); }
    );
    MapRegions({
        {uint8_t(0x00), uint8_t(0x3F), uint16_t(0x0000), uint16_t(0x1FFF)},
        {uint8_t(0x80), uint8_t(0xBF), uint16_t(0x0000), uint16_t(0x1FFF)},
    }, wramSlotLo_);

    // Register WRAM full handler (banks $7E-7F : $0000-$FFFF = full 128KB)
    wramSlotFull_ = RegisterHandler(
        [this](uint32_t addr, uint8_t /*ob*/) { return readWramFull(addr, 0); },
        [this](uint32_t addr, uint8_t data)   { writeWramFull(addr, data); }
    );
    MapRange(0x7E, 0x7F, 0x0000, 0xFFFF, wramSlotFull_);

    // Register WMDATA handler ($00-3F,$80-BF : $2180-$2183)
    wmdataSlot_ = RegisterHandler(
        [this](uint32_t addr, uint8_t ob)   { return readWmdata(addr, ob); },
        [this](uint32_t addr, uint8_t data) { writeWmdata(addr, data); }
    );
    MapRegions({
        {uint8_t(0x00), uint8_t(0x3F), uint16_t(0x2180), uint16_t(0x2183)},
        {uint8_t(0x80), uint8_t(0xBF), uint16_t(0x2180), uint16_t(0x2183)},
    }, wmdataSlot_);
}

// CPU I/O register mapping ($4016-$4017, $4200-$421F)
//
// These are mapped to banks $00-3F,$80-BF following bsnes cpu.cpp power():
//   bus.map(readCPU, writeCPU, "00-3f,80-bf:2180-2183,4016-4017,4200-421f")
//
// WMDATA ($2180-$2183) is already mapped by MapWram(), so MapCpuIo() only
// needs to map the joypad ports and the $4200-$421F range.

void MemoryBus::MapCpuIo(CpuIoRegisters& cpuIo) {
    // Register a single handler for all CPU I/O reads/writes.
    uint8_t cpuIoSlot = RegisterHandler(
        [&cpuIo](uint32_t addr, uint8_t ob) -> uint8_t {
            return cpuIo.Read(addr, ob);
        },
        [&cpuIo](uint32_t addr, uint8_t data) {
            cpuIo.Write(addr, data);
        }
    );

    // $00-3F,$80-BF : $4016-$4017 (joypad serial ports)
    MapRegions({
        {uint8_t(0x00), uint8_t(0x3F), uint16_t(0x4016), uint16_t(0x4017)},
        {uint8_t(0x80), uint8_t(0xBF), uint16_t(0x4016), uint16_t(0x4017)},
    }, cpuIoSlot);

    // $00-3F,$80-BF : $4200-$421F (CPU I/O registers)
    MapRegions({
        {uint8_t(0x00), uint8_t(0x3F), uint16_t(0x4200), uint16_t(0x421F)},
        {uint8_t(0x80), uint8_t(0xBF), uint16_t(0x4200), uint16_t(0x421F)},
    }, cpuIoSlot);
}

// DMA channel register mapping ($4300-$437F)
//
// bsnes maps these in cpu.cpp power():
//   bus.map(readDMA, writeDMA, "00-3f,80-bf:4300-437f")

void MemoryBus::MapDma(DmaController& dma) {
    uint8_t dmaSlot = RegisterHandler(
        [&dma](uint32_t addr, uint8_t ob) -> uint8_t {
            return dma.Read(addr, ob);
        },
        [&dma](uint32_t addr, uint8_t data) {
            dma.Write(addr, data);
        }
    );

    // $00-3F,$80-BF : $4300-$437F
    MapRegions({
        {uint8_t(0x00), uint8_t(0x3F), uint16_t(0x4300), uint16_t(0x437F)},
        {uint8_t(0x80), uint8_t(0xBF), uint16_t(0x4300), uint16_t(0x437F)},
    }, dmaSlot);
}

// PPU register mapping ($2100-$213F)
//
// bsnes maps these in ppu power():
//   bus.map(reader, writer, "00-3f,80-bf:2100-213f")

void MemoryBus::MapPpu(Ppu& ppu) {
    uint8_t ppuSlot = RegisterHandler(
        [&ppu](uint32_t addr, uint8_t ob) -> uint8_t {
            return ppu.ReadIO(addr, ob);
        },
        [&ppu](uint32_t addr, uint8_t data) {
            ppu.WriteIO(addr, data);
        }
    );

    // $00-3F,$80-BF : $2100-$213F
    MapRegions({
        {uint8_t(0x00), uint8_t(0x3F), uint16_t(0x2100), uint16_t(0x213F)},
        {uint8_t(0x80), uint8_t(0xBF), uint16_t(0x2100), uint16_t(0x213F)},
    }, ppuSlot);
}

// APU communication port mapping ($2140-$217F)
//
// bsnes maps these in cpu.cpp power():
//   bus.map(readAPU, writeAPU, "00-3f,80-bf:2140-217f")
//
// $2140-$217F are all mirrors of 4 ports (only bits 0-1 matter).

void MemoryBus::MapApu(Smp& smp) {
    uint8_t apuSlot = RegisterHandler(
        [&smp](uint32_t addr, uint8_t /*ob*/) -> uint8_t {
            return smp.PortRead(static_cast<uint8_t>(addr & 3));
        },
        [&smp](uint32_t addr, uint8_t data) {
            smp.PortWrite(static_cast<uint8_t>(addr & 3), data);
        }
    );

    // $00-3F,$80-BF : $2140-$217F
    MapRegions({
        {uint8_t(0x00), uint8_t(0x3F), uint16_t(0x2140), uint16_t(0x217F)},
        {uint8_t(0x80), uint8_t(0xBF), uint16_t(0x2140), uint16_t(0x217F)},
    }, apuSlot);
}

// Cartridge mapping

void MemoryBus::MapCartridge(Cartridge& cart) {
    // Register a single ROM+SRAM handler.
    // The Cartridge class already has full address resolution logic
    // (LoROM/HiROM/ExHiROM), so we delegate to it.
    uint8_t cartSlot = RegisterHandler(
        [&cart](uint32_t addr, uint8_t openBus) -> uint8_t {
            return cart.Read(addr, openBus);
        },
        [&cart](uint32_t addr, uint8_t data) {
            cart.Write(addr, data);
        }
    );

    // Map cartridge ROM based on mapping type.
    // We map the entire ROM-accessible regions; the Cartridge::Read/Write
    // internally resolves the actual offset and handles mirroring.
    const auto mapping = cart.Header().mapping;

    switch (mapping) {
    case MappingType::SufamiTurbo:
        MapRange(0x00, 0x63, 0x8000, 0xffff, cartSlot);
        MapRange(0x80, 0xe3, 0x8000, 0xffff, cartSlot);
        MapRange(0x70, 0x73, 0x8000, 0xffff, cartSlot);
        MapRange(0xf0, 0xf3, 0x8000, 0xffff, cartSlot);
        break;
    case MappingType::ExLoRom:
    case MappingType::LoRomNoMad1:
    case MappingType::LoRomLargeSram:
    case MappingType::LoRom:
        // Both halves of the full-ROM banks select the same 32 KiB page.
        // SRAM below takes precedence in $70-7D/$F0-FF.
        MapRegions({
            {uint8_t(0x00), uint8_t(0x7D), uint16_t(0x8000), uint16_t(0xFFFF)},
            {uint8_t(0x80), uint8_t(0xFF), uint16_t(0x8000), uint16_t(0xFFFF)},
            {uint8_t(0x40), uint8_t(0x7D), uint16_t(0x0000), uint16_t(0x7FFF)},
            {uint8_t(0xC0), uint8_t(0xFF), uint16_t(0x0000), uint16_t(0x7FFF)},
        }, cartSlot);

        // SRAM: $70-7D,$F0-FF:$0000-$7FFF
        // (Cartridge::ResolveSramOffset applies size/mirroring.)
        MapRegions({
            {uint8_t(0x70), uint8_t(0x7D), uint16_t(0x0000), uint16_t(0x7FFF)},
            {uint8_t(0xF0), uint8_t(0xFF), uint16_t(0x0000), uint16_t(0x7FFF)},
        }, cartSlot);
        break;

    case MappingType::LoRom24Mbit:
        MapRange(0x00, 0x3f, 0x8000, 0xffff, cartSlot);
        MapRange(0x80, 0xbf, 0x8000, 0xffff, cartSlot);
        MapRange(0x70, 0x7d, 0x0000, 0x7fff, cartSlot);
        MapRange(0xf0, 0xff, 0x0000, 0x7fff, cartSlot);
        break;

    case MappingType::HiRom:
        // ROM: $40-7D:$0000-$FFFF, $C0-FF:$0000-$FFFF
        MapRegions({
            {uint8_t(0x40), uint8_t(0x7D), uint16_t(0x0000), uint16_t(0xFFFF)},
            {uint8_t(0xC0), uint8_t(0xFF), uint16_t(0x0000), uint16_t(0xFFFF)},
        }, cartSlot);

        // ROM mirror: $00-3F,$80-BF:$8000-$FFFF
        MapRegions({
            {uint8_t(0x00), uint8_t(0x3F), uint16_t(0x8000), uint16_t(0xFFFF)},
            {uint8_t(0x80), uint8_t(0xBF), uint16_t(0x8000), uint16_t(0xFFFF)},
        }, cartSlot);

        // SRAM: $20-3F,$A0-BF:$6000-$7FFF
        // (already handled by cart handler via ResolveSramOffset)
        MapRegions({
            {uint8_t(0x20), uint8_t(0x3F), uint16_t(0x6000), uint16_t(0x7FFF)},
            {uint8_t(0xA0), uint8_t(0xBF), uint16_t(0x6000), uint16_t(0x7FFF)},
        }, cartSlot);
        break;

    case MappingType::ExHiRom:
        // ROM: $C0-FF:$0000-$FFFF (first 4MB)
        MapRange(0xC0, 0xFF, 0x0000, 0xFFFF, cartSlot);

        // ROM: $40-7D:$0000-$FFFF (second 4MB)
        MapRange(0x40, 0x7D, 0x0000, 0xFFFF, cartSlot);

        // ROM mirror: $00-3F,$80-BF:$8000-$FFFF
        MapRegions({
            {uint8_t(0x00), uint8_t(0x3F), uint16_t(0x8000), uint16_t(0xFFFF)},
            {uint8_t(0x80), uint8_t(0xBF), uint16_t(0x8000), uint16_t(0xFFFF)},
        }, cartSlot);

        // SRAM: same as HiROM
        MapRegions({
            {uint8_t(0x20), uint8_t(0x3F), uint16_t(0x6000), uint16_t(0x7FFF)},
            {uint8_t(0xA0), uint8_t(0xBF), uint16_t(0x6000), uint16_t(0x7FFF)},
        }, cartSlot);
        break;

    default:
        // Unknown mapping — just map upper halves
        MapRegions({
            {uint8_t(0x00), uint8_t(0x7D), uint16_t(0x8000), uint16_t(0xFFFF)},
            {uint8_t(0x80), uint8_t(0xFF), uint16_t(0x8000), uint16_t(0xFFFF)},
        }, cartSlot);
        break;
    }
}

// WRAM read/write handlers

// Low mirror: banks $00-3F,$80-BF : $0000-$1FFF → first 8 KB of WRAM
uint8_t MemoryBus::readWramLow(uint32_t addr, uint8_t /*openBus*/) {
    return wram_[addr & 0x1FFF];
}

void MemoryBus::writeWramLow(uint32_t addr, uint8_t data) {
    wram_[addr & 0x1FFF] = data;
}

// Full 128 KB: banks $7E-7F : $0000-$FFFF
// $7E:0000-$7E:FFFF = WRAM[0x00000..0x0FFFF]
// $7F:0000-$7F:FFFF = WRAM[0x10000..0x1FFFF]
uint8_t MemoryBus::readWramFull(uint32_t addr, uint8_t /*openBus*/) {
    uint32_t offset = ((addr >> 16) - 0x7E) * 0x10000 + (addr & 0xFFFF);
    return wram_[offset & 0x1FFFF];
}

void MemoryBus::writeWramFull(uint32_t addr, uint8_t data) {
    uint32_t offset = ((addr >> 16) - 0x7E) * 0x10000 + (addr & 0xFFFF);
    wram_[offset & 0x1FFFF] = data;
}

// WMDATA register ($2180-$2183)
//
//   $2180 — WMDATA read/write  (reads/writes WRAM[wmdataAddr_], auto-increment)
//   $2181 — WMADDL write       (low byte of WRAM address)
//   $2182 — WMADDM write       (mid byte of WRAM address)
//   $2183 — WMADDH write       (high bit of WRAM address, bit 0 only)

uint8_t MemoryBus::readWmdata(uint32_t addr, uint8_t openBus) {
    uint16_t offset = addr & 0xFFFF;

    if (offset == 0x2180) {
        uint8_t data = wram_[wmdataAddr_ & 0x1FFFF];
        wmdataAddr_ = (wmdataAddr_ + 1) & 0x1FFFF;
        return data;
    }

    // $2181-$2183 are write-only; reading returns open bus
    return openBus;
}

void MemoryBus::writeWmdata(uint32_t addr, uint8_t data) {
    uint16_t offset = addr & 0xFFFF;

    switch (offset) {
    case 0x2180:
        wram_[wmdataAddr_ & 0x1FFFF] = data;
        wmdataAddr_ = (wmdataAddr_ + 1) & 0x1FFFF;
        break;
    case 0x2181:
        wmdataAddr_ = (wmdataAddr_ & 0x1FF00) | static_cast<uint32_t>(data);
        break;
    case 0x2182:
        wmdataAddr_ = (wmdataAddr_ & 0x100FF) | (static_cast<uint32_t>(data) << 8);
        break;
    case 0x2183:
        wmdataAddr_ = (wmdataAddr_ & 0x0FFFF) | (static_cast<uint32_t>(data & 0x01) << 16);
        break;
    default:
        break;
    }
}

} // namespace snes::core
