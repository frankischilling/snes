// snes emulator
// core/include/snes/core/Timing.hpp
// Master clock and video-region timing interface.

#pragma once
// Timing.hpp — SNES dot / scanline / frame timing subsystem
//
// Tracks horizontal master clocks and vertical scanlines in two-clock steps.
// Provides the dot-position, scanline, and field state that every other
// subsystem depends on:
//
//   PPU   — FrameBegin / ScanlineBegin / VBlankBegin / SetCurrentLine/Dot
//   CPU   — NMI edge at V=vblank, H≈2;  DRAM refresh once per line
//   DMA   — HDMA setup at V=0 / HDMA run per visible scanline
//
// Timing parameters (from Fullsnes & bsnes PPUcounter):
//
//   NTSC: 262 scanlines per frame, 1364 master clocks per scanline
//         Exception: on non-interlace odd fields scanline 240 is 1360 clocks
//         (the "short scanline" aligns the color burst phase).
//
//   PAL:  312 scanlines per frame, 1364 master clocks per scanline
//         Exception: on interlace odd fields scanline 311 is 1368 clocks.
//
//   Master clocks per frame (NTSC non-interlace even field):
//         262 × 1364 = 357,368
//         Odd field: 261 × 1364 + 1 × 1360 = 357,364
//
//   VBlank: begins at scanline V = vdisp (225 normal, 240 overscan).
//   HBlank: dots 274–340 (master clocks 1096–1364) per scanline.
//
// The counter is paced externally: call Tick(masterClocks) to advance time.
// Callbacks fire at the correct position.
//
// Reference: bsnes/sfc/ppu/counter/counter-inline.hpp
//            https://problemkaputt.de/fullsnes.htm#sabortsnesframecyclecounts

#include <cstdint>
#include <functional>

namespace snes::core {

// Region — NTSC vs PAL
enum class Region : uint8_t {
    NTSC = 0,
    PAL  = 1,
};

// Timing — master-clock-accurate H/V counter
class Timing {
public:
    // Constants

    /// Master clock frequency (≈21.477 MHz crystal ÷ 1)
    static constexpr uint32_t kMasterClockHz = 21'477'272;
    static constexpr uint32_t kMasterClockHzPal = 21'281'370;

    /// Normal scanline width in master clocks.
    static constexpr uint16_t kDotsPerLine = 1364;

    /// Short scanline width (NTSC non-interlace odd-field line 240).
    static constexpr uint16_t kDotsPerLineShort = 1360;

    /// Long scanline width (PAL interlace odd-field line 311).
    static constexpr uint16_t kDotsPerLineLong = 1368;

    /// Scanlines per frame.
    static constexpr uint16_t kScanlinesNTSC = 262;
    static constexpr uint16_t kScanlinesPAL  = 312;

    /// Master clocks per "normal" frame (even field, NTSC non-interlace).
    static constexpr uint32_t kMasterClocksPerFrameNTSC = kScanlinesNTSC * kDotsPerLine;  // 357,368
    static constexpr uint32_t kMasterClocksPerFramePAL  = kScanlinesPAL  * kDotsPerLine;  // 425,568

    /// HBlank begins at this H counter value (dot 274 × 4 = 1096).
    static constexpr uint16_t kHBlankStart = 1096;

    /// Initial DRAM refresh position; later lines follow the DMA divider.
    static constexpr uint16_t kDramRefreshPos = 538;

    /// DRAM refresh penalty in master clocks (40 clocks = 5 × 8-cycle slots).
    static constexpr uint16_t kDramRefreshClocks = 40;

    /// HDMA horizontal trigger position.
    static constexpr uint16_t kHdmaPosition = 1104;

    /// Approximate PPU render sampling point used by bsnes fast PPU.
    static constexpr uint16_t kRenderCycle = 512;

    /// NMI trigger position (H counter = 2, at the start of vblank scanline).
    static constexpr uint16_t kNmiHPos = 2;

    // Construction

    explicit Timing(Region region = Region::NTSC);

    // Configuration

    /// Get/set region (changes scanline count + short/long line rules).
    Region GetRegion() const noexcept { return region_; }
    uint32_t MasterClockHz() const noexcept { return region_ == Region::PAL ? kMasterClockHzPal : kMasterClockHz; }
    void SetRegion(Region r) noexcept;

    /// Get/set interlace mode (mirrors PPU SETINI bit 0).
    bool Interlace() const noexcept { return interlace_; }
    void SetInterlace(bool v) noexcept { interlace_ = v; }

    /// VDisp — number of active (visible) scanlines.
    /// 225 in normal mode, 240 with overscan.
    uint16_t VDisp() const noexcept { return vdisp_; }
    void SetVDisp(uint16_t v) noexcept { vdisp_ = v; }

    // Counter state

    /// Current horizontal position (0 .. hPeriod-1), in master clocks.
    uint16_t HCounter() const noexcept { return hcounter_; }
    uint16_t DramRefreshPosition() const noexcept { return dramRefreshPosition_; }
    uint16_t HdmaSetupPosition() const noexcept { return hdmaSetupPosition_; }

    /// Horizontal dot number for the latched PPU beam counter.
    uint16_t HDot() const noexcept;

    /// Current vertical scanline counter (0 .. vPeriod-1).
    uint16_t VCounter() const noexcept { return vcounter_; }

    /// Current field (0 = even, 1 = odd).  Toggles every frame.
    bool Field() const noexcept { return field_; }

    /// Master clocks remaining in the current scanline.
    uint16_t HPeriod() const noexcept { return hperiod_; }

    /// Scanlines in the current frame.
    uint16_t VPeriod() const noexcept { return vperiod_; }

    /// Total master clocks elapsed since power-on / reset.
    uint64_t MasterClocksElapsed() const noexcept { return masterClocksElapsed_; }

    /// Is the current dot position in HBlank?
    bool InHBlank() const noexcept { return hcounter_ >= kHBlankStart; }

    /// Is the current scanline in VBlank?
    bool InVBlank() const noexcept { return vcounter_ >= vdisp_; }

    /// Frame index (incremented every time V wraps to 0).
    uint64_t FrameCount() const noexcept { return frameCount_; }

    // Core tick interface

    /// Advance the timing subsystem by `clocks` master clocks.
    /// Fires callbacks (onScanline, onVBlank, onHBlank, etc.) at the
    /// appropriate positions as H/V counters wrap.
    void Tick(uint32_t clocks);

    /// Reset all counters to power-on state.
    void Reset();

    // Callbacks — set by the integration layer (Emulator / StepFrame)

    /// Called at V=0, H=0 (start of a new frame).
    std::function<void()> onFrameBegin;

    /// Initialize HDMA during scanline zero after the divider-dependent delay.
    std::function<void()> onHdmaSetup;

    /// Called when V counter enters a new scanline (H wraps to 0).
    /// Parameter: new V counter value.
    std::function<void(uint16_t vcounter)> onScanline;

    /// Called when H counter reaches the render sampling point on a visible
    /// scanline. The PPU line cache should snapshot register state here.
    std::function<void(uint16_t vcounter)> onRenderCycle;

    /// Called when V counter reaches vdisp (VBlank start), H=0.
    /// This is the moment to fire NMI (after a few-dot delay).
    std::function<void()> onVBlankBegin;

    /// Called when H counter crosses kHBlankStart on a visible scanline.
    /// Useful for signaling HBlank-related behavior.
    std::function<void(uint16_t vcounter)> onHBlank;

    /// Called when H counter reaches kNmiHPos on the VBlank scanline.
    /// This is the actual NMI assertion point.
    std::function<void()> onNmiPoint;

    /// Called when H counter reaches kDramRefreshPos on any scanline.
    /// The caller should deduct kDramRefreshClocks from the CPU budget.
    std::function<void()> onDramRefresh;

    /// Called when H counter reaches kHdmaPosition on a visible scanline.
    /// The caller should run HDMA transfers.
    std::function<void(uint16_t vcounter)> onHdmaTransfer;

    /// Called every 4 master clocks for NMI/IRQ condition evaluation.
    /// Parameters: (hcounter, vcounter, vdisp, hperiod).
    /// Matches bsnes stepOnce() polling frequency.
    std::function<void(uint16_t, uint16_t, uint16_t, uint16_t)> onIrqPoll;

    /// Called every 128 master clocks for auto-joypad polling.
    /// Parameters: (hcounter, vcounter, vdisp).
    /// Matches bsnes joypadEdge() frequency.
    std::function<void(uint16_t, uint16_t, uint16_t)> onJoypadPoll;

    // Utility

    /// Compute the total master clocks in one complete frame for the
    /// current region/interlace/field configuration.
    uint32_t MasterClocksThisFrame() const;

    /// Compute the H period for a given scanline in the current configuration.
    uint16_t HPeriodForScanline(uint16_t scanline) const;

private:
    void tickOnce(uint8_t clocks = 2);
    void tickScanline();      // Called when H counter wraps
    void updateHPeriod();     // Recompute hperiod_ for the current scanline

    Region   region_    = Region::NTSC;
    bool     interlace_ = false;
    uint16_t vdisp_     = 225;

    uint16_t hcounter_  = 0;       // 0 .. hperiod_-1
    uint16_t vcounter_  = 0;       // 0 .. vperiod_-1
    bool     field_     = false;   // 0=even, 1=odd
    uint16_t hperiod_   = kDotsPerLine;
    uint16_t vperiod_   = kScanlinesNTSC;

    uint64_t masterClocksElapsed_ = 0;
    uint64_t frameCount_ = 0;
    uint16_t dramRefreshPosition_ = kDramRefreshPos;
    uint16_t hdmaSetupPosition_ = 12;
    bool hdmaSetupFired_ = false;
    bool halfClockPending_ = false;

    // Per-scanline event flags (reset on each new scanline)
    bool dramRefreshFired_ = false;
    bool hblankFired_      = false;
    bool hdmaFired_        = false;
    bool renderCycleFired_ = false;
    bool nmiFired_         = false;

    // Joypad poll divider (fires every 128 master clocks)
    uint8_t joypadDivider_ = 0;
};

} // namespace snes::core
