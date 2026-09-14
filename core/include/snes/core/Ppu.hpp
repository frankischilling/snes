#pragma once
// Ppu.hpp — SNES PPU register model + scanline renderer
//
// Covers all PPU I/O registers $2100–$213F and a scanline-based rendering
// pipeline following bsnes ppu-fast.
//
// Write registers: $2100–$2133
// Read  registers: $2134–$213F
//
// Internal memory:
//   VRAM  — 64 KB (32K × 16-bit words)
//   OAM   — 544 bytes (128 objects × 4 bytes + 32 bytes high table)
//   CGRAM — 512 bytes (256 × 15-bit colors stored as uint16_t)
//
// Rendering:
//   Per-scanline pipeline: snapshot IO → render BG/OBJ → window → composite
//   Output: 256×224(240) pixels as uint32_t RGBA8888
//
// Reference: bsnes sfc/ppu-fast/ppu.hpp, line.cpp, background.cpp, object.cpp

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>

namespace snes::core {

// Ppu — PPU register model
class Ppu {
public:
    Ppu();
    ~Ppu() = default;

    // Non-copyable
    Ppu(const Ppu&) = delete;
    Ppu& operator=(const Ppu&) = delete;

    // Reset — power-on defaults (clear VRAM/OAM/CGRAM, reset latches)
    void Reset();

    // Bus interface — called from MemoryBus via handler callbacks
    //   addr is the full 24-bit address (only $2100-$213F are dispatched here)
    //   openBus is the current bus MDR (for reads of write-only registers)
    uint8_t ReadIO(uint32_t addr, uint8_t openBus);
    void    WriteIO(uint32_t addr, uint8_t data);

    // Scanline / frame hooks (called by the timing layer)

    /// Called at the start of each frame (V=0).
    void FrameBegin();

    /// Called at the start of each visible scanline (V=1..vdisp-1).
    /// Snapshots IO state for the scanline renderer.
    void ScanlineBegin(uint16_t line);

    /// Called at V=vdisp (start of vblank).
    void VBlankBegin();

    // Display state queries
    bool Interlace()    const noexcept { return io_.interlace; }
    bool Overscan()     const noexcept { return io_.overscan; }
    bool FrameOverscan() const noexcept { return frameOverscan_; }
    uint16_t VDisp()    const noexcept { return io_.overscan ? 240 : 225; }
    bool DisplayDisable() const noexcept { return io_.displayDisable; }
    uint8_t Brightness() const noexcept { return io_.displayBrightness; }
    bool FieldID()       const noexcept { return fieldId_; }

    // Counters (for H/V counter latching by CPU I/O $2137, $213C-$213F)

    /// Latch current H/V counters — called when SLHV ($2137) read or via
    /// programmable I/O pin.  The timing layer provides the actual values.
    void LatchCounters(uint16_t hcounter, uint16_t vcounter);

    /// Set the current V counter (for vblank detection in OAM/VRAM access).
    /// The timing layer calls this each scanline so the PPU knows the current
    /// scanline position for display-disable checks.
    void SetCurrentLine(uint16_t vcounter) noexcept { currentLine_ = vcounter; }

    /// Set the current H counter (for CGRAM mid-scanline gating).
    /// The timing layer calls this before PPU register accesses so the PPU
    /// knows whether the dot is in active rendering or HBlank.
    void SetCurrentDot(uint16_t hcounter) noexcept { currentDot_ = hcounter; }

    /// Set the CPU PIO register ($4201) value — needed for STAT78 counter
    /// latch bit behavior. The CPU I/O layer updates this on WRIO writes.
    void SetCpuPio(uint8_t pio) noexcept { cpuPio_ = pio; }

    /// Set callback for $2137 SLHV reads — the emulator provides
    /// LatchCounters(hcounter, vcounter) through this.
    using LatchCallback = std::function<void()>;
    void SetCounterLatchCallback(LatchCallback cb) { onCounterLatch_ = std::move(cb); }

    /// Notify timing when SETINI changes the first VBlank scanline.
    using VDispCallback = std::function<void(uint16_t vdisp)>;
    void SetVDispCallback(VDispCallback cb) { onVDisp_ = std::move(cb); }

    // Direct memory access — for DMA, testing, rendering
    uint16_t* VramData() noexcept { return vram_.get(); }
    const uint16_t* VramData() const noexcept { return vram_.get(); }
    static constexpr size_t VramWords = 32768; // 64KB / 2

    uint8_t*  OamData() noexcept { return oam_.data(); }
    const uint8_t* OamData() const noexcept { return oam_.data(); }
    static constexpr size_t OamSize = 544; // 512 + 32

    uint16_t* CgramData() noexcept { return cgram_.data(); }
    const uint16_t* CgramData() const noexcept { return cgram_.data(); }
    static constexpr size_t CgramColors = 256;

    // UpdateVideoMode — recalculate tile modes and priority per BG mode.
    // Called after writing BGMODE ($2105) or SETINI ($2133).
    void UpdateVideoMode();

    // PPU I/O register state — public for rendering / test inspection

    /// Tile depth mode per BG layer (set by UpdateVideoMode)
    enum class TileMode : uint8_t { BPP2, BPP4, BPP8, Mode7, Inactive };

    /// Source layer IDs for color math / priority
    enum Source : uint8_t { BG1, BG2, BG3, BG4, OBJ1, OBJ2, COL, SourceCount };

    // Window layer enable/invert/mask
    struct WindowLayer {
        bool oneEnable  = false;
        bool oneInvert  = false;
        bool twoEnable  = false;
        bool twoInvert  = false;
        uint8_t mask    = 0;   // 0=OR, 1=AND, 2=XOR, 3=XNOR
        bool aboveEnable = false;
        bool belowEnable = false;
    };

    struct WindowColor {
        bool oneEnable  = false;
        bool oneInvert  = false;
        bool twoEnable  = false;
        bool twoInvert  = false;
        uint8_t mask    = 0;
        uint8_t aboveMask = 0; // 2 bits
        uint8_t belowMask = 0; // 2 bits
    };

    // Per-background state
    struct Background {
        WindowLayer window;
        bool aboveEnable    = false;
        bool belowEnable    = false;
        bool mosaicEnable   = false;
        uint16_t tiledataAddress = 0;
        uint16_t screenAddress   = 0;
        uint8_t  screenSize      = 0;
        bool     tileSize        = false; // 0=8×8, 1=16×16
        uint16_t hoffset         = 0;
        uint16_t voffset         = 0;
        TileMode tileMode        = TileMode::Inactive;
        uint8_t  priority[2]     = {0, 0};
    };

    // Sprite / OBJ state
    struct ObjectIO {
        WindowLayer window;
        bool aboveEnable    = false;
        bool belowEnable    = false;
        bool interlace      = false;
        uint8_t  baseSize   = 0;
        uint8_t  nameselect = 0;
        uint16_t tiledataAddress = 0;
        uint8_t  first      = 0; // first sprite index (for priority rotation)
        bool rangeOver       = false;
        bool timeOver        = false;
        uint8_t priority[4]  = {0, 0, 0, 0};
    };

    // Color math state
    struct ColorIO {
        WindowColor window;
        bool enable[SourceCount] = {};
        bool directColor = false;
        bool blendMode   = false; // 0=fixed color, 1=sub-screen pixel
        bool halve       = false;
        bool mathMode    = false; // 0=add, 1=subtract
        uint16_t fixedColor = 0;  // 15-bit BGR555
    };

    // Mosaic
    struct Mosaic {
        uint8_t size    = 1;  // 1–16
        uint8_t counter = 0;
    };

    // Mode 7
    struct Mode7 {
        bool hflip      = false;
        bool vflip      = false;
        uint8_t repeat  = 0;   // 2 bits
        uint16_t a = 0, b = 0, c = 0, d = 0;
        uint16_t x = 0, y = 0;
        uint16_t hoffset = 0;
        uint16_t voffset = 0;
    };

    // Window position
    struct WindowPos {
        uint8_t oneLeft   = 0;
        uint8_t oneRight  = 0;
        uint8_t twoLeft   = 0;
        uint8_t twoRight  = 0;
    };

    // Aggregate IO structure
    struct IO {
        // $2100 INIDISP
        bool displayDisable     = true;
        uint8_t displayBrightness = 0;

        // $2102-$2103 OAM address
        uint16_t oamBaseAddress = 0; // 10-bit
        uint16_t oamAddress     = 0; // 10-bit, current address
        bool oamPriority        = false;

        // $2105 BGMODE
        uint8_t bgMode          = 0;
        bool bgPriority         = false;

        // $2115 VMAIN
        bool vramIncrementMode  = false; // 0=after $2118, 1=after $2119
        uint8_t vramMapping     = 0;     // address translation mode (0-3)
        uint16_t vramIncrementSize = 1;  // 1,32,128

        // $2116-$2117 VRAM address
        uint16_t vramAddress    = 0;

        // $2121 CGRAM address
        uint8_t cgramAddress    = 0;
        bool cgramReadLatch     = false;
        bool cgramWriteLatch    = false;

        // $2133 SETINI
        bool interlace          = false;
        bool overscan           = false;
        bool pseudoHires        = false;
        bool extbg              = false;

        // H/V counters (latched by SLHV)
        uint16_t hcounter       = 0;
        uint16_t vcounter       = 0;

        // Nested state
        Mosaic  mosaic;
        Mode7   mode7;
        WindowPos window;
        Background bg1, bg2, bg3, bg4;
        ObjectIO   obj;
        ColorIO    col;
    };

    // Public access to IO for rendering / tests
    IO&       GetIO() noexcept { return io_; }
    const IO& GetIO() const noexcept { return io_; }

    // Latch state — internal buffers for write-twice behavior, etc.
    struct Latch {
        uint16_t vram        = 0;   // VRAM prefetch latch (16-bit)
        uint8_t  oam         = 0;   // OAM write latch (low byte)
        uint8_t  cgram       = 0;   // CGRAM write latch (low byte)
        uint16_t oamAddress  = 0;   // latched OAM address for active display
        uint8_t  cgramAddress= 0;   // latched CGRAM address for active display
        uint8_t  mode7       = 0;   // Mode 7 / scroll write-twice latch
        bool     counters    = false;
        bool     hcounter    = false; // toggle for $213C read
        bool     vcounter    = false; // toggle for $213D read

        struct PpuMdr {
            uint8_t mdr    = 0;
            uint8_t bgofs  = 0;   // BG scroll offset latch
        } ppu1, ppu2;
    };

    Latch&       GetLatch() noexcept { return latch_; }
    const Latch& GetLatch() const noexcept { return latch_; }

    // Rendering — pixel format and line buffers

    /// A single rendered pixel in the above/below line buffers.
    struct Pixel {
        uint8_t  source   = COL;   // Source layer that produced this pixel
        uint8_t  priority = 0;     // Priority value (higher wins)
        uint16_t color    = 0;     // 15-bit BGR555 color
    };

    /// Parsed OAM object for rendering.
    struct Object {
        uint16_t x         = 0;
        uint8_t  y         = 0;
        uint8_t  character = 0;
        bool     nameselect = false;
        bool     vflip     = false;
        bool     hflip     = false;
        uint8_t  priority  = 0;   // 0-3
        uint8_t  palette   = 0;   // 0-7
        bool     size      = false; // 0=small, 1=large
    };

    /// Sprite evaluation result — which OAM entries are on this line.
    struct ObjectItem {
        bool    valid  = false;
        uint8_t index  = 0;
        uint8_t width  = 0;
        uint8_t height = 0;
    };

    /// Sprite tile data for rendering.
    struct ObjectTile {
        bool     valid    = false;
        uint16_t x        = 0;
        uint8_t  priority = 0;
        uint8_t  palette  = 0;
        bool     hflip    = false;
        uint32_t data     = 0; // packed 4bpp tile row (32 bits = 2 VRAM words)
    };

    /// Per-scanline rendering state — snapshotted at cache time.
    struct Line {
        uint16_t y       = 0; // Hardware vcounter; first visible line is 1.
        bool     fieldID = false;

        IO       io;                              // Snapshot of PPU IO
        uint16_t cgram[CgramColors] = {};         // Snapshot of palette

        ObjectItem items[128]  = {};
        ObjectTile tiles[128]  = {};

        Pixel    above[256] = {};                 // Main screen line buffer
        Pixel    below[256] = {};                 // Sub screen line buffer
        bool     windowAbove[256] = {};           // Color window for main screen
        bool     windowBelow[256] = {};           // Color window for sub screen
    };

    /// Sprite size lookup tables (indexed by ObjectIO::baseSize 0-7).
    static constexpr uint8_t kObjSmallWidth [8] = { 8,  8,  8, 16, 16, 32, 16, 16};
    static constexpr uint8_t kObjSmallHeight[8] = { 8,  8,  8, 16, 16, 32, 32, 32};
    static constexpr uint8_t kObjLargeWidth [8] = {16, 32, 64, 32, 64, 64, 32, 32};
    static constexpr uint8_t kObjLargeHeight[8] = {16, 32, 64, 32, 64, 64, 64, 32};

    // Rendering API

    /// Render all cached scanlines (call at VBlank).  Writes to the output
    /// framebuffer.
    void RenderFrame();

    /// Get the output framebuffer.  Layout: 512×480 uint32_t RGBA8888.
    /// Standard 256×224 image occupies the first 256 pixels of each row
    /// for the first 224 rows (no interlace).
    uint32_t*       OutputData() noexcept { return output_.get(); }
    const uint32_t* OutputData() const noexcept { return output_.get(); }
    static constexpr int OutputWidth  = 512;
    static constexpr int OutputHeight = 480;

    /// Access parsed OAM objects (populated from raw OAM by ParseOam).
    const Object* Objects() const noexcept { return objects_.data(); }

    /// Parse raw OAM bytes into structured Object array.
    void ParseOam();

    /// Number of valid rendering lines cached this frame.
    int CachedLineCount() const noexcept { return lineCount_; }

private:
    // Internal VRAM / OAM / CGRAM helpers

    /// VRAM address with translation mapping applied
    uint16_t TranslatedVramAddress() const;

    /// Read VRAM word — returns 0 during active rendering if display enabled
    uint16_t ReadVram();

    /// Write VRAM byte (lowByte=false → low, true → high)
    void WriteVram(bool highByte, uint8_t data);

    /// Read a byte from OAM (10-bit address space, 544 bytes)
    uint8_t ReadOam(uint16_t address);

    /// Write a byte to OAM
    void WriteOam(uint16_t address, uint8_t data);

    /// Read a byte from CGRAM (lowByte=false → bits 0-7, true → bits 8-14)
    uint8_t ReadCgram(bool highByte, uint8_t address);

    /// Write a 15-bit color to CGRAM
    void WriteCgram(uint8_t address, uint16_t data);

    /// OAM address reset (on OAMADDL/OAMADDH write, at VBlank start, etc.)
    void OamAddressReset();

    /// Set first sprite for priority rotation
    void OamSetFirstObject();

    // Scanline rendering internals

    /// Render a single cached scanline.
    void RenderLine(Line& line);

    /// Initialize above/below buffers with backdrop color.
    void InitLineBuffers(Line& line);

    /// Render one background layer into the line buffers.
    void RenderBackground(Line& line, const Background& bg, uint8_t source);

    /// Render Mode 7 background.
    void RenderMode7(Line& line, const Background& bg, uint8_t source);

    /// Render all sprites into the line buffers.
    void RenderObjects(Line& line, const ObjectIO& obj);

    /// Compute window mask for a BG/OBJ layer.
    void RenderWindow(const Line& line, const WindowLayer& wl, bool enable,
                      bool output[256]);

    /// Compute window mask for color math.
    void RenderWindowColor(const Line& line, const WindowColor& wc,
                           uint8_t mask, bool output[256]);

    /// Tilemap lookup — returns 16-bit tilemap word.
    uint16_t GetTile(const Line& line, const Background& bg,
                     uint32_t hoffset, uint32_t voffset);

    /// Priority-based pixel plot.
    static void PlotAbove(Line& line, int x, uint8_t source,
                          uint8_t priority, uint16_t color);
    static void PlotBelow(Line& line, int x, uint8_t source,
                          uint8_t priority, uint16_t color);

    /// Color math compositing for a single pixel.
    uint16_t CompositePixel(const Line& line, int x,
                            Pixel above, Pixel below) const;

    /// Color math blend (add/subtract with saturation).
    uint16_t Blend(const Line& line, uint16_t x, uint16_t y,
                   bool halve) const;

    /// Direct color mode conversion.
    static uint16_t DirectColor(uint8_t paletteIndex, uint8_t paletteColor);

    /// Build the brightness lookup table.
    void BuildLightTable();

    // State
    IO    io_;
    Latch latch_;

    // VRAM — 64 KB = 32K × 16-bit words (heap-allocated to avoid stack overflow)
    std::unique_ptr<uint16_t[]> vram_;

    // OAM — 544 bytes raw (128 objects × 4 bytes + 32 bytes high table)
    std::array<uint8_t, OamSize> oam_{};

    // CGRAM — 256 × 15-bit colors
    std::array<uint16_t, CgramColors> cgram_{};

    // Parsed OAM objects (128 entries, built from raw OAM before rendering)
    std::array<Object, 128> objects_{};

    // Per-scanline rendering cache (max 240 visible lines)
    // Heap-allocated to avoid stack overflow (~1.3 MB)
    static constexpr int MaxVisibleLines = 240;
    std::unique_ptr<Line[]> lines_;
    int lineStart_ = 0;
    int lineCount_ = 0;

    // Output framebuffer — 512×480 RGBA8888
    std::unique_ptr<uint32_t[]> output_;

    // Brightness lookup table: lightTable_[brightness][bgr555] → bgr555 dimmed
    // 16 brightness levels × 32768 colors
    std::unique_ptr<uint16_t[]> lightTableData_;
    uint16_t* lightTable_[16] = {};

    // Current scanline (set by timing layer for display-disable checks)
    uint16_t currentLine_ = 0;

    // Current H dot (set by timing layer for CGRAM mid-scanline gating)
    uint16_t currentDot_ = 0;

    // CPU PIO register ($4201) — needed for STAT78 counter latch behavior
    uint8_t cpuPio_ = 0xFF;

    // Callback for $2137 SLHV reads (triggers counter latch via Emulator)
    LatchCallback onCounterLatch_;
    VDispCallback onVDisp_;

    // Interlace field ID (toggled each frame)
    bool fieldId_ = false;

    // Overscan state latched at frame start (matches render-time frame state)
    bool frameOverscan_ = false;

    // NTSC / PAL flag (affects STAT78 readback)
    bool isPal_ = false;
};

} // namespace snes::core
