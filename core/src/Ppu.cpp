// ============================================================================
// Ppu.cpp — SNES PPU register model implementation
//
// Implements all PPU registers $2100–$213F following bsnes ppu-fast/io.cpp.
// This file handles the register read/write interface and VRAM/OAM/CGRAM
// access.  Rendering pipeline is in PpuRender.cpp.
//
// Reference: bsnes sfc/ppu-fast/io.cpp, ppu-fast/ppu.cpp
// ============================================================================

#include "snes/core/Ppu.hpp"
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>

namespace snes::core {

namespace {
bool PpuDebugEnabled() {
    static int enabled = -1;
    if (enabled == -1) {
        const char* v = std::getenv("SNES_PPU_IO_TRACE");
        if (!v || !*v) {
            v = std::getenv("SNES_PPU_DEBUG");
        }
        enabled = (v && *v && *v != '0') ? 1 : 0;
    }
    return enabled == 1;
}
}

// ============================================================================
// Constructor / Reset
// ============================================================================
Ppu::Ppu()
    : vram_(std::make_unique<uint16_t[]>(VramWords))
    , lines_(std::make_unique<Line[]>(MaxVisibleLines))
    , output_(std::make_unique<uint32_t[]>(static_cast<size_t>(OutputWidth) * OutputHeight))
{
    BuildLightTable();
    Reset();
}

void Ppu::Reset() {
    // Clear memory
    std::memset(vram_.get(), 0, VramWords * sizeof(uint16_t));
    oam_.fill(0);
    cgram_.fill(0);

    // Reset IO to power-on defaults
    io_ = IO{};
    latch_ = Latch{};

    // Clear rendering state
    lineStart_ = 0;
    lineCount_ = 0;
    std::memset(output_.get(), 0,
                static_cast<size_t>(OutputWidth) * OutputHeight * sizeof(uint32_t));

    currentLine_ = 0;
    currentDot_ = 0;
    cpuPio_ = 0xFF;
    fieldId_ = false;
    frameOverscan_ = false;
    isPal_ = false;

    UpdateVideoMode();
}

// ============================================================================
// Frame / scanline hooks
// ============================================================================
void Ppu::FrameBegin() {
    // Toggle interlace field each frame
    fieldId_ = !fieldId_;
    frameOverscan_ = io_.overscan;

    io_.obj.rangeOver = false;
    io_.obj.timeOver  = false;

    // Reset scanline cache for new frame
    lineStart_ = 0;
    lineCount_ = 0;
}

void Ppu::ScanlineBegin(uint16_t line) {
    bool mosaicEnable = io_.bg1.mosaicEnable || io_.bg2.mosaicEnable ||
                        io_.bg3.mosaicEnable || io_.bg4.mosaicEnable;
    if (line == 1) {
        io_.mosaic.counter = mosaicEnable ? static_cast<uint8_t>(io_.mosaic.size + 1) : 0;
    }
    if (io_.mosaic.counter && !--io_.mosaic.counter) {
        io_.mosaic.counter = mosaicEnable ? io_.mosaic.size : 0;
    }

    // Snapshot IO state and CGRAM for this scanline
    if (line > 0 && line < 240) {
        auto& cache = lines_[line];
        cache.y = line;
        cache.fieldID = fieldId_;

        if (io_.displayDisable || line >= VDisp()) {
            cache.io.displayDisable = true;
        } else {
            cache.io = io_;
            std::memcpy(cache.cgram, cgram_.data(), sizeof(cache.cgram));
        }

        if (lineCount_ == 0) lineStart_ = line;
        lineCount_++;
    }
}

void Ppu::VBlankBegin() {
    if (!io_.displayDisable) {
        OamAddressReset();
    }

    // Render all cached scanlines
    RenderFrame();
}

// ============================================================================
// VRAM helpers
// ============================================================================

uint16_t Ppu::TranslatedVramAddress() const {
    uint16_t address = io_.vramAddress;
    switch (io_.vramMapping) {
    case 0: return address & 0x7FFF;
    case 1: return (address & 0x7F00) | ((address << 3) & 0x00F8) | ((address >> 5) & 7);
    case 2: return (address & 0x7E00) | ((address << 3) & 0x01F8) | ((address >> 6) & 7);
    case 3: return (address & 0x7C00) | ((address << 3) & 0x03F8) | ((address >> 7) & 7);
    default: return address & 0x7FFF;
    }
}

uint16_t Ppu::ReadVram() {
    // During active display (visible scanlines 1..vdisp-1), VRAM reads return 0.
    if (!io_.displayDisable && currentLine_ > 0 && currentLine_ < VDisp()) {
        return 0x0000;
    }
    return vram_[TranslatedVramAddress()];
}

void Ppu::WriteVram(bool highByte, uint8_t data) {
    // During active display (visible scanlines 1..vdisp-1), VRAM writes are blocked.
    if (!io_.displayDisable && currentLine_ > 0 && currentLine_ < VDisp()) {
        return;
    }
    uint16_t addr = TranslatedVramAddress();
    if (!highByte) {
        vram_[addr] = (vram_[addr] & 0xFF00) | static_cast<uint16_t>(data);
    } else {
        vram_[addr] = (vram_[addr] & 0x00FF) | (static_cast<uint16_t>(data) << 8);
    }
}

// ============================================================================
// OAM helpers
// ============================================================================

uint8_t Ppu::ReadOam(uint16_t address) {
    address &= 0x03FF;
    // During active display, use latched address
    if (!io_.displayDisable && currentLine_ > 0 && currentLine_ < VDisp()) {
        address = latch_.oamAddress & 0x03FF;
    }
    if (address < OamSize) return oam_[address];
    return 0;
}

void Ppu::WriteOam(uint16_t address, uint8_t data) {
    address &= 0x03FF;
    // During active display, writes go to a fixed address ($0218)
    if (!io_.displayDisable && currentLine_ > 0 && currentLine_ < VDisp()) {
        address = 0x0218;
    }
    if (address < OamSize) {
        oam_[address] = data;
    }
}

void Ppu::OamAddressReset() {
    io_.oamAddress = io_.oamBaseAddress;
    OamSetFirstObject();
}

void Ppu::OamSetFirstObject() {
    io_.obj.first = 0;
    if (io_.oamPriority) {
        io_.obj.first = (io_.oamAddress >> 2) & 127;
    }
}

// ============================================================================
// CGRAM helpers
// ============================================================================

uint8_t Ppu::ReadCgram(bool highByte, uint8_t address) {
    // During active rendering (display enabled, visible scanline, in active dots),
    // reads are redirected to the latched address.  Outside active dots (HBlank)
    // the CPU can access CGRAM normally even on visible scanlines.
    // bsnes: hcounter >= 88 && hcounter < 1096
    if (!io_.displayDisable && currentLine_ > 0 && currentLine_ < VDisp()
        && currentDot_ >= 88 && currentDot_ < 1096) {
        address = latch_.cgramAddress;
    }
    if (!highByte) {
        return static_cast<uint8_t>(cgram_[address] & 0xFF);
    } else {
        return static_cast<uint8_t>((cgram_[address] >> 8) & 0xFF);
    }
}

void Ppu::WriteCgram(uint8_t address, uint16_t data) {
    if (!io_.displayDisable && currentLine_ > 0 && currentLine_ < VDisp()
        && currentDot_ >= 88 && currentDot_ < 1096) {
        address = latch_.cgramAddress;
    }
    cgram_[address] = data & 0x7FFF; // 15-bit color
}

// ============================================================================
// Counter latching
// ============================================================================

void Ppu::LatchCounters(uint16_t hcounter, uint16_t vcounter) {
    io_.hcounter = hcounter;
    io_.vcounter = vcounter;
    latch_.counters = true;
}

// ============================================================================
// ReadIO — read PPU register ($2134–$213F, plus open-bus for write-only)
//
// Reference: bsnes ppu-fast/io.cpp readIO()
// ============================================================================
uint8_t Ppu::ReadIO(uint32_t addr, uint8_t openBus) {
    switch (addr & 0xFFFF) {

    // Write-only registers — return PPU1 MDR
    case 0x2104: case 0x2105: case 0x2106: case 0x2108:
    case 0x2109: case 0x210A: case 0x2114: case 0x2115:
    case 0x2116: case 0x2118: case 0x2119: case 0x211A:
    case 0x2124: case 0x2125: case 0x2126: case 0x2128:
    case 0x2129: case 0x212A: {
        return latch_.ppu1.mdr;
    }

    // $2134 — MPYL (mode 7 multiply result low)
    case 0x2134: {
        int32_t result = static_cast<int16_t>(io_.mode7.a) *
                         static_cast<int8_t>(io_.mode7.b >> 8);
        return latch_.ppu1.mdr = static_cast<uint8_t>(result);
    }

    // $2135 — MPYM (mode 7 multiply result mid)
    case 0x2135: {
        int32_t result = static_cast<int16_t>(io_.mode7.a) *
                         static_cast<int8_t>(io_.mode7.b >> 8);
        return latch_.ppu1.mdr = static_cast<uint8_t>(result >> 8);
    }

    // $2136 — MPYH (mode 7 multiply result high)
    case 0x2136: {
        int32_t result = static_cast<int16_t>(io_.mode7.a) *
                         static_cast<int8_t>(io_.mode7.b >> 8);
        return latch_.ppu1.mdr = static_cast<uint8_t>(result >> 16);
    }

    // $2137 — SLHV (software latch H/V counters)
    // When PIO bit 7 is high, reading this register latches the current
    // H/V counters.  The actual counter values come from the timing layer
    // via the onCounterLatch_ callback.
    case 0x2137: {
        if ((cpuPio_ & 0x80) && onCounterLatch_) {
            onCounterLatch_();
        }
        return openBus;
    }

    // $2138 — OAMDATAREAD
    case 0x2138: {
        uint8_t data = ReadOam(io_.oamAddress);
        io_.oamAddress = (io_.oamAddress + 1) & 0x03FF;
        OamSetFirstObject();
        return latch_.ppu1.mdr = data;
    }

    // $2139 — VMDATALREAD (VRAM data read low)
    case 0x2139: {
        uint8_t data = static_cast<uint8_t>(latch_.vram);
        if (io_.vramIncrementMode == 0) {
            latch_.vram = ReadVram();
            io_.vramAddress += io_.vramIncrementSize;
        }
        return latch_.ppu1.mdr = data;
    }

    // $213A — VMDATAHREAD (VRAM data read high)
    case 0x213A: {
        uint8_t data = static_cast<uint8_t>(latch_.vram >> 8);
        if (io_.vramIncrementMode == 1) {
            latch_.vram = ReadVram();
            io_.vramAddress += io_.vramIncrementSize;
        }
        return latch_.ppu1.mdr = data;
    }

    // $213B — CGDATAREAD
    case 0x213B: {
        if (!io_.cgramAddressLatch) {
            io_.cgramAddressLatch = true;
            latch_.ppu2.mdr = ReadCgram(false, io_.cgramAddress);
        } else {
            io_.cgramAddressLatch = false;
            latch_.ppu2.mdr = (ReadCgram(true, io_.cgramAddress) & 0x7F) |
                              (latch_.ppu2.mdr & 0x80);
            io_.cgramAddress++;
        }
        return latch_.ppu2.mdr;
    }

    // $213C — OPHCT (horizontal counter read, toggle high/low)
    case 0x213C: {
        if (!latch_.hcounter) {
            latch_.hcounter = true;
            latch_.ppu2.mdr = static_cast<uint8_t>(io_.hcounter);
        } else {
            latch_.hcounter = false;
            latch_.ppu2.mdr = (latch_.ppu2.mdr & 0xFE) |
                              static_cast<uint8_t>((io_.hcounter >> 8) & 1);
        }
        return latch_.ppu2.mdr;
    }

    // $213D — OPVCT (vertical counter read, toggle high/low)
    case 0x213D: {
        if (!latch_.vcounter) {
            latch_.vcounter = true;
            latch_.ppu2.mdr = static_cast<uint8_t>(io_.vcounter);
        } else {
            latch_.vcounter = false;
            latch_.ppu2.mdr = (latch_.ppu2.mdr & 0xFE) |
                              static_cast<uint8_t>((io_.vcounter >> 8) & 1);
        }
        return latch_.ppu2.mdr;
    }

    // $213E — STAT77 (PPU1 status: version + OBJ overflow flags)
    case 0x213E: {
        latch_.ppu1.mdr = 0x01; // PPU1 version = 1
        latch_.ppu1.mdr |= (io_.obj.rangeOver ? 1 : 0) << 6;
        latch_.ppu1.mdr |= (io_.obj.timeOver  ? 1 : 0) << 7;
        return latch_.ppu1.mdr;
    }

    // $213F — STAT78 (PPU2 status: version + region + field + counter latch)
    case 0x213F: {
        // Reset H/V counter toggles
        latch_.hcounter = false;
        latch_.vcounter = false;

        // Preserve bit 5 (open bus)
        latch_.ppu2.mdr &= (1 << 5);
        latch_.ppu2.mdr |= 0x03; // PPU2 version = 3
        latch_.ppu2.mdr |= (isPal_ ? 1 : 0) << 4;
        // Bit 6: counter latch flag
        // When PIO bit 7 is clear, bit 6 always reads high.
        // When PIO bit 7 is set, bit 6 reflects the latch flag (cleared on read).
        if (!(cpuPio_ & 0x80)) {
            latch_.ppu2.mdr |= (1 << 6);
        } else {
            latch_.ppu2.mdr |= (latch_.counters ? 1 : 0) << 6;
            latch_.counters = false;
        }
        // Bit 7: interlace field ID
        latch_.ppu2.mdr |= (fieldId_ ? 1 : 0) << 7;

        return latch_.ppu2.mdr;
    }

    } // switch

    return openBus;
}

// ============================================================================
// WriteIO — write PPU register ($2100–$2133)
//
// Reference: bsnes ppu-fast/io.cpp writeIO()
// ============================================================================
void Ppu::WriteIO(uint32_t addr, uint8_t data) {
    switch (addr & 0xFFFF) {

    // $2100 — INIDISP (display control)
    case 0x2100: {
        if (io_.displayDisable && currentLine_ == VDisp()) {
            OamAddressReset();
        }
        io_.displayBrightness = data & 0x0F;
        io_.displayDisable    = (data >> 7) & 1;
        return;
    }

    // $2101 — OBSEL (object size + base address)
    case 0x2101: {
        uint8_t oldBaseSize = io_.obj.baseSize;
        io_.obj.tiledataAddress = static_cast<uint16_t>((data << 13) & 0x6000);
        io_.obj.nameselect     = (data >> 3) & 3;
        io_.obj.baseSize       = (data >> 5) & 7;
        if (PpuDebugEnabled() && oldBaseSize != io_.obj.baseSize) {
            std::fprintf(stderr,
                         "[PPU-DBG l=%u] OBSEL baseSize=%u nameselect=%u objTileBase=$%04X\n",
                         currentLine_, io_.obj.baseSize, io_.obj.nameselect,
                         io_.obj.tiledataAddress);
        }
        return;
    }

    // $2102 — OAMADDL (OAM address low)
    case 0x2102: {
        io_.oamBaseAddress = (io_.oamBaseAddress & 0x0200) | (static_cast<uint16_t>(data) << 1);
        OamAddressReset();
        return;
    }

    // $2103 — OAMADDH (OAM address high + priority rotation)
    case 0x2103: {
        io_.oamBaseAddress = ((data & 1) << 9) | (io_.oamBaseAddress & 0x01FE);
        io_.oamPriority    = (data >> 7) & 1;
        OamAddressReset();
        return;
    }

    // $2104 — OAMDATA (OAM data write)
    case 0x2104: {
        bool latchBit = io_.oamAddress & 1;
        uint16_t address = io_.oamAddress;
        io_.oamAddress = (io_.oamAddress + 1) & 0x03FF;
        if (!latchBit) latch_.oam = data;
        if (address & 0x200) {
            WriteOam(address, data);
        } else if (latchBit) {
            WriteOam((address & ~1) + 0, latch_.oam);
            WriteOam((address & ~1) + 1, data);
        }
        OamSetFirstObject();
        return;
    }

    // $2105 — BGMODE (BG mode, tile sizes)
    case 0x2105: {
        uint8_t oldMode = io_.bgMode;
        bool oldPri = io_.bgPriority;
        io_.bgMode       = data & 7;
        io_.bgPriority   = (data >> 3) & 1;
        io_.bg1.tileSize = (data >> 4) & 1;
        io_.bg2.tileSize = (data >> 5) & 1;
        io_.bg3.tileSize = (data >> 6) & 1;
        io_.bg4.tileSize = (data >> 7) & 1;
        UpdateVideoMode();
        if (PpuDebugEnabled() && (oldMode != io_.bgMode || oldPri != io_.bgPriority)) {
            std::fprintf(stderr,
                         "[PPU-DBG l=%u] BGMODE mode=%u bgPri=%u tileSize=%u/%u/%u/%u\n",
                         currentLine_, io_.bgMode, io_.bgPriority,
                         io_.bg1.tileSize ? 1 : 0, io_.bg2.tileSize ? 1 : 0,
                         io_.bg3.tileSize ? 1 : 0, io_.bg4.tileSize ? 1 : 0);
        }
        return;
    }

    // $2106 — MOSAIC
    case 0x2106: {
        bool wasMosaicEnabled = io_.bg1.mosaicEnable || io_.bg2.mosaicEnable ||
                                io_.bg3.mosaicEnable || io_.bg4.mosaicEnable;
        io_.bg1.mosaicEnable = (data >> 0) & 1;
        io_.bg2.mosaicEnable = (data >> 1) & 1;
        io_.bg3.mosaicEnable = (data >> 2) & 1;
        io_.bg4.mosaicEnable = (data >> 3) & 1;
        io_.mosaic.size      = ((data >> 4) & 0x0F) + 1;
        if (!wasMosaicEnabled && (data & 0x0F)) {
            io_.mosaic.counter = io_.mosaic.size + 1;
        }
        return;
    }

    // $2107 — BG1SC (BG1 tilemap address + size)
    case 0x2107: {
        io_.bg1.screenSize    = data & 3;
        io_.bg1.screenAddress = static_cast<uint16_t>((data << 8) & 0x7C00);
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG1SC size=%u addr=$%04X\n",
                         currentLine_, io_.bg1.screenSize, io_.bg1.screenAddress);
        }
        return;
    }

    // $2108 — BG2SC
    case 0x2108: {
        io_.bg2.screenSize    = data & 3;
        io_.bg2.screenAddress = static_cast<uint16_t>((data << 8) & 0x7C00);
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG2SC size=%u addr=$%04X\n",
                         currentLine_, io_.bg2.screenSize, io_.bg2.screenAddress);
        }
        return;
    }

    // $2109 — BG3SC
    case 0x2109: {
        io_.bg3.screenSize    = data & 3;
        io_.bg3.screenAddress = static_cast<uint16_t>((data << 8) & 0x7C00);
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG3SC size=%u addr=$%04X\n",
                         currentLine_, io_.bg3.screenSize, io_.bg3.screenAddress);
        }
        return;
    }

    // $210A — BG4SC
    case 0x210A: {
        io_.bg4.screenSize    = data & 3;
        io_.bg4.screenAddress = static_cast<uint16_t>((data << 8) & 0x7C00);
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG4SC size=%u addr=$%04X\n",
                         currentLine_, io_.bg4.screenSize, io_.bg4.screenAddress);
        }
        return;
    }

    // $210B — BG12NBA (BG1/BG2 character data address)
    case 0x210B: {
        io_.bg1.tiledataAddress = static_cast<uint16_t>((data << 12) & 0x7000);
        io_.bg2.tiledataAddress = static_cast<uint16_t>((data <<  8) & 0x7000);
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG12NBA bg1=$%04X bg2=$%04X\n",
                         currentLine_, io_.bg1.tiledataAddress, io_.bg2.tiledataAddress);
        }
        return;
    }

    // $210C — BG34NBA (BG3/BG4 character data address)
    case 0x210C: {
        io_.bg3.tiledataAddress = static_cast<uint16_t>((data << 12) & 0x7000);
        io_.bg4.tiledataAddress = static_cast<uint16_t>((data <<  8) & 0x7000);
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG34NBA bg3=$%04X bg4=$%04X\n",
                         currentLine_, io_.bg3.tiledataAddress, io_.bg4.tiledataAddress);
        }
        return;
    }

    // $210D — BG1HOFS (BG1 horizontal scroll)
    // Also writes Mode 7 horizontal offset
    case 0x210D: {
        io_.mode7.hoffset = static_cast<uint16_t>(data) << 8 | latch_.mode7;
        latch_.mode7 = data;
        io_.bg1.hoffset = static_cast<uint16_t>(((static_cast<uint16_t>(data) << 8) |
                  (latch_.ppu1.bgofs & ~7) | (latch_.ppu2.bgofs & 7)) & 0x03FF);
        latch_.ppu1.bgofs = data;
        latch_.ppu2.bgofs = data;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG1HOFS data=%02X hoffset=$%04X\n",
                         currentLine_, data, io_.bg1.hoffset);
        }
        return;
    }

    // $210E — BG1VOFS (BG1 vertical scroll)
    // Also writes Mode 7 vertical offset
    case 0x210E: {
        io_.mode7.voffset = static_cast<uint16_t>(data) << 8 | latch_.mode7;
        latch_.mode7 = data;
        io_.bg1.voffset = static_cast<uint16_t>(((static_cast<uint16_t>(data) << 8) |
                  latch_.ppu1.bgofs) & 0x03FF);
        latch_.ppu1.bgofs = data;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG1VOFS data=%02X voffset=$%04X\n",
                         currentLine_, data, io_.bg1.voffset);
        }
        return;
    }

    // $210F — BG2HOFS
    case 0x210F: {
        io_.bg2.hoffset = static_cast<uint16_t>(((static_cast<uint16_t>(data) << 8) |
                  (latch_.ppu1.bgofs & ~7) | (latch_.ppu2.bgofs & 7)) & 0x03FF);
        latch_.ppu1.bgofs = data;
        latch_.ppu2.bgofs = data;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG2HOFS data=%02X hoffset=$%04X\n",
                         currentLine_, data, io_.bg2.hoffset);
        }
        return;
    }

    // $2110 — BG2VOFS
    case 0x2110: {
        io_.bg2.voffset = static_cast<uint16_t>(((static_cast<uint16_t>(data) << 8) |
                  latch_.ppu1.bgofs) & 0x03FF);
        latch_.ppu1.bgofs = data;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG2VOFS data=%02X voffset=$%04X\n",
                         currentLine_, data, io_.bg2.voffset);
        }
        return;
    }

    // $2111 — BG3HOFS
    case 0x2111: {
        io_.bg3.hoffset = static_cast<uint16_t>(((static_cast<uint16_t>(data) << 8) |
                  (latch_.ppu1.bgofs & ~7) | (latch_.ppu2.bgofs & 7)) & 0x03FF);
        latch_.ppu1.bgofs = data;
        latch_.ppu2.bgofs = data;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG3HOFS data=%02X hoffset=$%04X\n",
                         currentLine_, data, io_.bg3.hoffset);
        }
        return;
    }

    // $2112 — BG3VOFS
    case 0x2112: {
        io_.bg3.voffset = static_cast<uint16_t>(((static_cast<uint16_t>(data) << 8) |
                  latch_.ppu1.bgofs) & 0x03FF);
        latch_.ppu1.bgofs = data;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG3VOFS data=%02X voffset=$%04X\n",
                         currentLine_, data, io_.bg3.voffset);
        }
        return;
    }

    // $2113 — BG4HOFS
    case 0x2113: {
        io_.bg4.hoffset = static_cast<uint16_t>(((static_cast<uint16_t>(data) << 8) |
                  (latch_.ppu1.bgofs & ~7) | (latch_.ppu2.bgofs & 7)) & 0x03FF);
        latch_.ppu1.bgofs = data;
        latch_.ppu2.bgofs = data;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG4HOFS data=%02X hoffset=$%04X\n",
                         currentLine_, data, io_.bg4.hoffset);
        }
        return;
    }

    // $2114 — BG4VOFS
    case 0x2114: {
        io_.bg4.voffset = static_cast<uint16_t>(((static_cast<uint16_t>(data) << 8) |
                  latch_.ppu1.bgofs) & 0x03FF);
        latch_.ppu1.bgofs = data;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr, "[PPU-DBG l=%u] BG4VOFS data=%02X voffset=$%04X\n",
                         currentLine_, data, io_.bg4.voffset);
        }
        return;
    }

    // $2115 — VMAIN (VRAM address increment mode)
    case 0x2115: {
        static constexpr uint16_t kIncrementSizes[4] = {1, 32, 128, 128};
        io_.vramIncrementSize = kIncrementSizes[data & 3];
        io_.vramMapping       = (data >> 2) & 3;
        io_.vramIncrementMode = (data >> 7) & 1;
        return;
    }

    // $2116 — VMADDL (VRAM address low)
    case 0x2116: {
        io_.vramAddress = (io_.vramAddress & 0xFF00) | data;
        latch_.vram = ReadVram();
        return;
    }

    // $2117 — VMADDH (VRAM address high)
    case 0x2117: {
        io_.vramAddress = (io_.vramAddress & 0x00FF) | (static_cast<uint16_t>(data) << 8);
        latch_.vram = ReadVram();
        return;
    }

    // $2118 — VMDATAL (VRAM data write low)
    case 0x2118: {
        WriteVram(false, data);
        if (!io_.vramIncrementMode) io_.vramAddress += io_.vramIncrementSize;
        return;
    }

    // $2119 — VMDATAH (VRAM data write high)
    case 0x2119: {
        WriteVram(true, data);
        if (io_.vramIncrementMode) io_.vramAddress += io_.vramIncrementSize;
        return;
    }

    // $211A — M7SEL (Mode 7 settings)
    case 0x211A: {
        io_.mode7.hflip  = (data >> 0) & 1;
        io_.mode7.vflip  = (data >> 1) & 1;
        io_.mode7.repeat = (data >> 6) & 3;
        return;
    }

    // $211B — M7A
    case 0x211B: {
        io_.mode7.a = static_cast<uint16_t>(data) << 8 | latch_.mode7;
        latch_.mode7 = data;
        return;
    }

    // $211C — M7B
    case 0x211C: {
        io_.mode7.b = static_cast<uint16_t>(data) << 8 | latch_.mode7;
        latch_.mode7 = data;
        return;
    }

    // $211D — M7C
    case 0x211D: {
        io_.mode7.c = static_cast<uint16_t>(data) << 8 | latch_.mode7;
        latch_.mode7 = data;
        return;
    }

    // $211E — M7D
    case 0x211E: {
        io_.mode7.d = static_cast<uint16_t>(data) << 8 | latch_.mode7;
        latch_.mode7 = data;
        return;
    }

    // $211F — M7X
    case 0x211F: {
        io_.mode7.x = static_cast<uint16_t>(data) << 8 | latch_.mode7;
        latch_.mode7 = data;
        return;
    }

    // $2120 — M7Y
    case 0x2120: {
        io_.mode7.y = static_cast<uint16_t>(data) << 8 | latch_.mode7;
        latch_.mode7 = data;
        return;
    }

    // $2121 — CGADD (CGRAM address)
    case 0x2121: {
        io_.cgramAddress = data;
        io_.cgramAddressLatch = false;
        return;
    }

    // $2122 — CGDATA (CGRAM data write)
    case 0x2122: {
        if (!io_.cgramAddressLatch) {
            io_.cgramAddressLatch = true;
            latch_.cgram = data;
        } else {
            io_.cgramAddressLatch = false;
            WriteCgram(io_.cgramAddress,
                       static_cast<uint16_t>(data & 0x7F) << 8 | latch_.cgram);
            io_.cgramAddress++;
        }
        return;
    }

    // $2123 — W12SEL (window mask settings for BG1/BG2)
    case 0x2123: {
        io_.bg1.window.oneInvert = (data >> 0) & 1;
        io_.bg1.window.oneEnable = (data >> 1) & 1;
        io_.bg1.window.twoInvert = (data >> 2) & 1;
        io_.bg1.window.twoEnable = (data >> 3) & 1;
        io_.bg2.window.oneInvert = (data >> 4) & 1;
        io_.bg2.window.oneEnable = (data >> 5) & 1;
        io_.bg2.window.twoInvert = (data >> 6) & 1;
        io_.bg2.window.twoEnable = (data >> 7) & 1;
        return;
    }

    // $2124 — W34SEL (window mask settings for BG3/BG4)
    case 0x2124: {
        io_.bg3.window.oneInvert = (data >> 0) & 1;
        io_.bg3.window.oneEnable = (data >> 1) & 1;
        io_.bg3.window.twoInvert = (data >> 2) & 1;
        io_.bg3.window.twoEnable = (data >> 3) & 1;
        io_.bg4.window.oneInvert = (data >> 4) & 1;
        io_.bg4.window.oneEnable = (data >> 5) & 1;
        io_.bg4.window.twoInvert = (data >> 6) & 1;
        io_.bg4.window.twoEnable = (data >> 7) & 1;
        return;
    }

    // $2125 — WOBJSEL (window mask settings for OBJ/color)
    case 0x2125: {
        io_.obj.window.oneInvert = (data >> 0) & 1;
        io_.obj.window.oneEnable = (data >> 1) & 1;
        io_.obj.window.twoInvert = (data >> 2) & 1;
        io_.obj.window.twoEnable = (data >> 3) & 1;
        io_.col.window.oneInvert = (data >> 4) & 1;
        io_.col.window.oneEnable = (data >> 5) & 1;
        io_.col.window.twoInvert = (data >> 6) & 1;
        io_.col.window.twoEnable = (data >> 7) & 1;
        return;
    }

    // $2126 — WH0 (window 1 left position)
    case 0x2126: { io_.window.oneLeft  = data; return; }

    // $2127 — WH1 (window 1 right position)
    case 0x2127: { io_.window.oneRight = data; return; }

    // $2128 — WH2 (window 2 left position)
    case 0x2128: { io_.window.twoLeft  = data; return; }

    // $2129 — WH3 (window 2 right position)
    case 0x2129: { io_.window.twoRight = data; return; }

    // $212A — WBGLOG (window mask logic for BG layers)
    case 0x212A: {
        io_.bg1.window.mask = (data >> 0) & 3;
        io_.bg2.window.mask = (data >> 2) & 3;
        io_.bg3.window.mask = (data >> 4) & 3;
        io_.bg4.window.mask = (data >> 6) & 3;
        return;
    }

    // $212B — WOBJLOG (window mask logic for OBJ/color)
    case 0x212B: {
        io_.obj.window.mask = (data >> 0) & 3;
        io_.col.window.mask = (data >> 2) & 3;
        return;
    }

    // $212C — TM (main screen designation)
    case 0x212C: {
        io_.bg1.aboveEnable = (data >> 0) & 1;
        io_.bg2.aboveEnable = (data >> 1) & 1;
        io_.bg3.aboveEnable = (data >> 2) & 1;
        io_.bg4.aboveEnable = (data >> 3) & 1;
        io_.obj.aboveEnable = (data >> 4) & 1;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr,
                         "[PPU-DBG l=%u] TM above bg=%u%u%u%u obj=%u\n",
                         currentLine_, io_.bg4.aboveEnable ? 1 : 0,
                         io_.bg3.aboveEnable ? 1 : 0,
                         io_.bg2.aboveEnable ? 1 : 0,
                         io_.bg1.aboveEnable ? 1 : 0,
                         io_.obj.aboveEnable ? 1 : 0);
        }
        return;
    }

    // $212D — TS (sub screen designation)
    case 0x212D: {
        io_.bg1.belowEnable = (data >> 0) & 1;
        io_.bg2.belowEnable = (data >> 1) & 1;
        io_.bg3.belowEnable = (data >> 2) & 1;
        io_.bg4.belowEnable = (data >> 3) & 1;
        io_.obj.belowEnable = (data >> 4) & 1;
        if (PpuDebugEnabled()) {
            std::fprintf(stderr,
                         "[PPU-DBG l=%u] TS below bg=%u%u%u%u obj=%u\n",
                         currentLine_, io_.bg4.belowEnable ? 1 : 0,
                         io_.bg3.belowEnable ? 1 : 0,
                         io_.bg2.belowEnable ? 1 : 0,
                         io_.bg1.belowEnable ? 1 : 0,
                         io_.obj.belowEnable ? 1 : 0);
        }
        return;
    }

    // $212E — TMW (window mask for main screen)
    case 0x212E: {
        io_.bg1.window.aboveEnable = (data >> 0) & 1;
        io_.bg2.window.aboveEnable = (data >> 1) & 1;
        io_.bg3.window.aboveEnable = (data >> 2) & 1;
        io_.bg4.window.aboveEnable = (data >> 3) & 1;
        io_.obj.window.aboveEnable = (data >> 4) & 1;
        return;
    }

    // $212F — TSW (window mask for sub screen)
    case 0x212F: {
        io_.bg1.window.belowEnable = (data >> 0) & 1;
        io_.bg2.window.belowEnable = (data >> 1) & 1;
        io_.bg3.window.belowEnable = (data >> 2) & 1;
        io_.bg4.window.belowEnable = (data >> 3) & 1;
        io_.obj.window.belowEnable = (data >> 4) & 1;
        return;
    }

    // $2130 — CGWSEL (color addition select)
    case 0x2130: {
        io_.col.directColor      = (data >> 0) & 1;
        io_.col.blendMode        = (data >> 1) & 1;
        io_.col.window.belowMask = (data >> 4) & 3;
        io_.col.window.aboveMask = (data >> 6) & 3;
        return;
    }

    // $2131 — CGADDSUB (color math designation)
    case 0x2131: {
        io_.col.enable[Source::BG1 ] = (data >> 0) & 1;
        io_.col.enable[Source::BG2 ] = (data >> 1) & 1;
        io_.col.enable[Source::BG3 ] = (data >> 2) & 1;
        io_.col.enable[Source::BG4 ] = (data >> 3) & 1;
        io_.col.enable[Source::OBJ1] = false;
        io_.col.enable[Source::OBJ2] = (data >> 4) & 1;
        io_.col.enable[Source::COL ] = (data >> 5) & 1;
        io_.col.halve               = (data >> 6) & 1;
        io_.col.mathMode            = (data >> 7) & 1;
        return;
    }

    // $2132 — COLDATA (fixed color data)
    case 0x2132: {
        uint16_t intensity = data & 0x1F;
        if (data & 0x20) io_.col.fixedColor = (io_.col.fixedColor & 0b11111'11111'00000) | (intensity <<  0);
        if (data & 0x40) io_.col.fixedColor = (io_.col.fixedColor & 0b11111'00000'11111) | (intensity <<  5);
        if (data & 0x80) io_.col.fixedColor = (io_.col.fixedColor & 0b00000'11111'11111) | (intensity << 10);
        return;
    }

    // $2133 — SETINI (screen mode / scroll settings)
    case 0x2133: {
        bool oldInterlace = io_.interlace;
        bool oldObjInterlace = io_.obj.interlace;
        bool oldOverscan = io_.overscan;
        bool oldPseudoHires = io_.pseudoHires;
        io_.interlace     = (data >> 0) & 1;
        io_.obj.interlace = (data >> 1) & 1;
        io_.overscan      = (data >> 2) & 1;
        io_.pseudoHires   = (data >> 3) & 1;
        io_.extbg         = (data >> 6) & 1;
        UpdateVideoMode();
        if (PpuDebugEnabled() && (oldInterlace != io_.interlace
            || oldObjInterlace != io_.obj.interlace
            || oldOverscan != io_.overscan
            || oldPseudoHires != io_.pseudoHires)) {
            std::fprintf(stderr,
                         "[PPU-DBG l=%u] SETINI interlace=%u objInterlace=%u overscan=%u pseudoHires=%u extbg=%u\n",
                         currentLine_, io_.interlace ? 1 : 0,
                         io_.obj.interlace ? 1 : 0,
                         io_.overscan ? 1 : 0,
                         io_.pseudoHires ? 1 : 0,
                         io_.extbg ? 1 : 0);
        }
        return;
    }

    } // switch
}

// ============================================================================
// UpdateVideoMode — set tile modes + priority per BG mode
//
// Reference: bsnes ppu-fast/io.cpp updateVideoMode()
// ============================================================================
void Ppu::UpdateVideoMode() {
    auto assign2 = [](uint8_t (&arr)[2], uint8_t a, uint8_t b) { arr[0] = a; arr[1] = b; };
    auto assign4 = [](uint8_t (&arr)[4], uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
        arr[0] = a; arr[1] = b; arr[2] = c; arr[3] = d;
    };

    switch (io_.bgMode) {
    case 0:
        io_.bg1.tileMode = TileMode::BPP2;
        io_.bg2.tileMode = TileMode::BPP2;
        io_.bg3.tileMode = TileMode::BPP2;
        io_.bg4.tileMode = TileMode::BPP2;
        assign2(io_.bg1.priority, 8, 11);
        assign2(io_.bg2.priority, 7, 10);
        assign2(io_.bg3.priority, 2,  5);
        assign2(io_.bg4.priority, 1,  4);
        assign4(io_.obj.priority, 3,  6, 9, 12);
        break;
    case 1:
        io_.bg1.tileMode = TileMode::BPP4;
        io_.bg2.tileMode = TileMode::BPP4;
        io_.bg3.tileMode = TileMode::BPP2;
        io_.bg4.tileMode = TileMode::Inactive;
        if (io_.bgPriority) {
            assign2(io_.bg1.priority, 5,  8);
            assign2(io_.bg2.priority, 4,  7);
            assign2(io_.bg3.priority, 1, 10);
            assign4(io_.obj.priority, 2,  3, 6,  9);
        } else {
            assign2(io_.bg1.priority, 6,  9);
            assign2(io_.bg2.priority, 5,  8);
            assign2(io_.bg3.priority, 1,  3);
            assign4(io_.obj.priority, 2,  4, 7, 10);
        }
        break;
    case 2:
        io_.bg1.tileMode = TileMode::BPP4;
        io_.bg2.tileMode = TileMode::BPP4;
        io_.bg3.tileMode = TileMode::Inactive;
        io_.bg4.tileMode = TileMode::Inactive;
        assign2(io_.bg1.priority, 3, 7);
        assign2(io_.bg2.priority, 1, 5);
        assign4(io_.obj.priority, 2, 4, 6, 8);
        break;
    case 3:
        io_.bg1.tileMode = TileMode::BPP8;
        io_.bg2.tileMode = TileMode::BPP4;
        io_.bg3.tileMode = TileMode::Inactive;
        io_.bg4.tileMode = TileMode::Inactive;
        assign2(io_.bg1.priority, 3, 7);
        assign2(io_.bg2.priority, 1, 5);
        assign4(io_.obj.priority, 2, 4, 6, 8);
        break;
    case 4:
        io_.bg1.tileMode = TileMode::BPP8;
        io_.bg2.tileMode = TileMode::BPP2;
        io_.bg3.tileMode = TileMode::Inactive;
        io_.bg4.tileMode = TileMode::Inactive;
        assign2(io_.bg1.priority, 3, 7);
        assign2(io_.bg2.priority, 1, 5);
        assign4(io_.obj.priority, 2, 4, 6, 8);
        break;
    case 5:
        io_.bg1.tileMode = TileMode::BPP4;
        io_.bg2.tileMode = TileMode::BPP2;
        io_.bg3.tileMode = TileMode::Inactive;
        io_.bg4.tileMode = TileMode::Inactive;
        assign2(io_.bg1.priority, 3, 7);
        assign2(io_.bg2.priority, 1, 5);
        assign4(io_.obj.priority, 2, 4, 6, 8);
        break;
    case 6:
        io_.bg1.tileMode = TileMode::BPP4;
        io_.bg2.tileMode = TileMode::Inactive;
        io_.bg3.tileMode = TileMode::Inactive;
        io_.bg4.tileMode = TileMode::Inactive;
        assign2(io_.bg1.priority, 2, 5);
        assign4(io_.obj.priority, 1, 3, 4, 6);
        break;
    case 7:
        if (!io_.extbg) {
            io_.bg1.tileMode = TileMode::Mode7;
            io_.bg2.tileMode = TileMode::Inactive;
            io_.bg3.tileMode = TileMode::Inactive;
            io_.bg4.tileMode = TileMode::Inactive;
            assign2(io_.bg1.priority, 2, 2); // Mode 7 has only 1 priority; set both same
            assign4(io_.obj.priority, 1, 3, 4, 5);
        } else {
            io_.bg1.tileMode = TileMode::Mode7;
            io_.bg2.tileMode = TileMode::Mode7;
            io_.bg3.tileMode = TileMode::Inactive;
            io_.bg4.tileMode = TileMode::Inactive;
            assign2(io_.bg1.priority, 3, 3);
            assign2(io_.bg2.priority, 1, 5);
            assign4(io_.obj.priority, 2, 4, 6, 7);
        }
        break;
    default:
        break;
    }
}

} // namespace snes::core
