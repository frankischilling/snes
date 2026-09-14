// snes emulator
// core/src/ppu/PpuRender.cpp
// Scanline rendering for backgrounds, sprites, and color math.

// PpuRender.cpp — SNES PPU scanline rendering pipeline
//
// Implements the scanline-based rendering pipeline following bsnes ppu-fast:
//   - IO state snapshot per visible scanline
//   - Background rendering (modes 0-7, 2bpp/4bpp/8bpp, offset-per-tile)
//   - Sprite rendering (128 OAM evaluation, 32-per-line, 4bpp tiles)
//   - Window masking (BG/OBJ layer windows + color math windows)
//   - Priority-based compositing with above/below (main/sub) screens
//   - Color math (add/subtract with saturation, halve, per-layer enable)
//   - Brightness application via lookup table
//   - Output to RGBA8888 framebuffer
//
// Reference: bsnes sfc/ppu-fast/line.cpp, background.cpp, object.cpp

#include "snes/core/Ppu.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace snes::core {

namespace {
int gRenderFrame = 0;

int ProbeFrame() {
    static int probeFrame = -2;
    if (probeFrame == -2) {
        const char* v = std::getenv("SNES_PROBE_FRAME");
        probeFrame = (v && *v) ? std::atoi(v) : -1;
    }
    return probeFrame;
}

bool OamTraceEnabled() {
    static int enabled = -1;
    if (enabled == -1) {
        const char* v = std::getenv("SNES_OAM_TRACE");
        enabled = (v && *v && *v != '0') ? 1 : 0;
    }
    return enabled == 1;
}
}

// Brightness lookup table

void Ppu::BuildLightTable() {
    // 16 brightness levels × 32768 BGR555 entries
    // Index and output are both plain BGR555 — no channel swapping.
    lightTableData_ = std::make_unique<uint16_t[]>(16 * 32768);
    for (int l = 0; l < 16; l++) {
        lightTable_[l] = lightTableData_.get() + l * 32768;
        double luma = static_cast<double>(l) / 15.0;
        for (uint32_t color = 0; color < 32768; color++) {
            uint32_t r = (color >>  0) & 31;
            uint32_t g = (color >>  5) & 31;
            uint32_t b = (color >> 10) & 31;
            uint32_t ar = static_cast<uint32_t>(luma * r + 0.5);
            uint32_t ag = static_cast<uint32_t>(luma * g + 0.5);
            uint32_t ab = static_cast<uint32_t>(luma * b + 0.5);
            lightTable_[l][color] =
                static_cast<uint16_t>((ab << 10) | (ag << 5) | ar);
        }
    }
}

// OAM parsing — raw bytes → structured Object array

void Ppu::ParseOam() {
    for (int n = 0; n < 128; n++) {
        auto& obj = objects_[n];

        // Low table: 4 bytes per object at offset n*4
        uint16_t base = static_cast<uint16_t>(n) * 4;
        uint8_t b0 = oam_[base + 0];
        uint8_t b1 = oam_[base + 1];
        uint8_t b2 = oam_[base + 2];
        uint8_t b3 = oam_[base + 3];

        // High table: 2 bits per object at offset 512 + n/4
        uint8_t hiByte = oam_[512 + (n >> 2)];
        uint8_t hiBits = (hiByte >> ((n & 3) * 2)) & 3;

        obj.x         = static_cast<uint16_t>(b0) | ((hiBits & 1) << 8); // 9-bit x
        // bsnes stores OBJ Y as (OAM Y + 1) in visible vcounter space.
        obj.y         = static_cast<uint8_t>(b1 + 1);
        obj.character = b2;
        obj.nameselect = (b3 >> 0) & 1;
        obj.palette    = (b3 >> 1) & 7;
        obj.priority   = (b3 >> 4) & 3;
        obj.hflip      = (b3 >> 6) & 1;
        obj.vflip      = (b3 >> 7) & 1;
        obj.size       = (hiBits >> 1) & 1;
    }
}

// Scanline snapshot — called each visible scanline during emulation

// ScanlineBegin is defined in Ppu.cpp; we update it here with caching.
// The actual caching is done via the new body (see wire-up below).

// Frame rendering — renders all cached scanlines at VBlank

void Ppu::RenderFrame() {
    if (lineCount_ == 0) return;

    // Parse OAM once per frame
    ParseOam();

    if (OamTraceEnabled() && gRenderFrame == ProbeFrame()) {
        std::fprintf(stderr, "[OAM frame=%d] baseSize=%u first=%u\n",
                     gRenderFrame, io_.obj.baseSize, io_.obj.first);
        for (int i = 0; i < 128; i++) {
            const auto& obj = objects_[i];
            uint8_t w = obj.size ? kObjLargeWidth[io_.obj.baseSize]
                                 : kObjSmallWidth[io_.obj.baseSize];
            uint8_t h = obj.size ? kObjLargeHeight[io_.obj.baseSize]
                                 : kObjSmallHeight[io_.obj.baseSize];
            bool onscreenX = !(obj.x > 256 && static_cast<int>(obj.x) + w - 1 < 512);
            bool onscreenY = obj.y <= 224 || static_cast<uint16_t>(obj.y) + h >= 256;
            if (!onscreenX || !onscreenY) continue;
            std::fprintf(stderr,
                         "  idx=%3d x=%3u y=%3u chr=%3u size=%u w=%2u h=%2u pri=%u pal=%u ns=%u hf=%u vf=%u\n",
                         i, obj.x, obj.y, obj.character, obj.size ? 1 : 0, w, h,
                         obj.priority, obj.palette, obj.nameselect ? 1 : 0,
                         obj.hflip ? 1 : 0, obj.vflip ? 1 : 0);
        }
    }

    // Render each cached scanline
    for (int i = 0; i < lineCount_; i++) {
        RenderLine(lines_[lineStart_ + i]);
    }

    lineStart_ = 0;
    lineCount_ = 0;
    gRenderFrame++;
}

// RenderLine — the core per-scanline pipeline

void Ppu::RenderLine(Line& line) {
    uint16_t y = line.y;

    // Export visible rows directly: visible vcounter 1 maps to output row 0.
    uint16_t outputY = static_cast<uint16_t>(y - 1);

    auto* outRow = output_.get() + static_cast<size_t>(outputY) * OutputWidth;

    // Display disabled → black
    if (line.io.displayDisable) {
        std::memset(outRow, 0, 256 * sizeof(uint32_t));
        return;
    }

    // Step 1: Initialize line buffers with backdrop
    InitLineBuffers(line);

    // Step 2: Render all layers; order follows bsnes
    RenderBackground(line, line.io.bg1, Source::BG1);
    if (!line.io.extbg)
        RenderBackground(line, line.io.bg2, Source::BG2);
    RenderBackground(line, line.io.bg3, Source::BG3);
    RenderBackground(line, line.io.bg4, Source::BG4);
    RenderObjects(line, line.io.obj);
    if (line.io.extbg)
        RenderBackground(line, line.io.bg2, Source::BG2); // EXTBG: BG2 after sprites

    // Step 3: Compute color windows
    RenderWindowColor(line, line.io.col.window,
                      line.io.col.window.aboveMask, line.windowAbove);
    RenderWindowColor(line, line.io.col.window,
                      line.io.col.window.belowMask, line.windowBelow);

    // Step 4: Final compositing with brightness
    auto* luma = lightTable_[line.io.displayBrightness];

    for (int x = 0; x < 256; x++) {
        uint16_t color555 = CompositePixel(line, x, line.above[x], line.below[x]);
        uint16_t dimmed = luma[color555];

        // Convert BGR555 → RGBA8888
        // dimmed is BGR555: bbbbb_ggggg_rrrrr
        // 5→8 bit expansion: (c << 3) | (c >> 2)  [matches bsnes]
        uint32_t r5 = (dimmed >>  0) & 31;
        uint32_t g5 = (dimmed >>  5) & 31;
        uint32_t b5 = (dimmed >> 10) & 31;
        uint32_t r8 = (r5 << 3) | (r5 >> 2);
        uint32_t g8 = (g5 << 3) | (g5 >> 2);
        uint32_t b8 = (b5 << 3) | (b5 >> 2);
        outRow[x] = (0xFFu << 24) | (b8 << 16) | (g8 << 8) | r8;
    }
}

// InitLineBuffers — fill with backdrop color

void Ppu::InitLineBuffers(Line& line) {
    bool hires = line.io.pseudoHires || line.io.bgMode == 5 || line.io.bgMode == 6;
    uint16_t aboveColor = line.cgram[0]; // palette[0] always
    uint16_t belowColor = hires ? line.cgram[0] : line.io.col.fixedColor;

    for (int x = 0; x < 256; x++) {
        line.above[x] = {Source::COL, 0, aboveColor};
        line.below[x] = {Source::COL, 0, belowColor};
    }
}

// PlotAbove / PlotBelow — priority-based pixel merge

void Ppu::PlotAbove(Line& line, int x, uint8_t source,
                    uint8_t priority, uint16_t color) {
    if (priority > line.above[x].priority) {
        line.above[x] = {source, priority, color};
    }
}

void Ppu::PlotBelow(Line& line, int x, uint8_t source,
                    uint8_t priority, uint16_t color) {
    if (priority > line.below[x].priority) {
        line.below[x] = {source, priority, color};
    }
}

// DirectColor — BG1 direct color mode (modes 3/4)

uint16_t Ppu::DirectColor(uint8_t paletteIndex, uint8_t paletteColor) {
    // paletteIndex = bgr bits from palette number (3 bits)
    // paletteColor = BBGGGRRR (8-bit pixel value)
    return static_cast<uint16_t>(
        ((paletteColor << 2) & 0x001C) + ((paletteIndex <<  1) & 0x0002) +  // R
        ((paletteColor << 4) & 0x0380) + ((paletteIndex <<  5) & 0x0040) +  // G
        ((paletteColor << 7) & 0x6000) + ((paletteIndex << 10) & 0x1000));  // B
}

// GetTile — tilemap word lookup
//
// The tilemap contains 16-bit entries: vhopppcc cccccccc
//   v=vflip h=hflip o=priority ppp=palette cccccccccc=character

uint16_t Ppu::GetTile(const Line& line, const Background& bg,
                      uint32_t hoffset, uint32_t voffset) {
    bool hires = line.io.bgMode == 5 || line.io.bgMode == 6;
    uint32_t tileHeight = 3 + static_cast<uint32_t>(bg.tileSize);
    uint32_t tileWidth  = !hires ? tileHeight : 4;

    uint32_t screenX = (bg.screenSize & 1) ? (32 << 5) : 0;
    uint32_t screenY = (bg.screenSize & 2) ? (32 << (5 + (bg.screenSize & 1))) : 0;

    uint32_t tileX = hoffset >> tileWidth;
    uint32_t tileY = voffset >> tileHeight;

    uint32_t offset = ((tileY & 0x1F) << 5) | (tileX & 0x1F);
    if (tileX & 0x20) offset += screenX;
    if (tileY & 0x20) offset += screenY;

    return vram_[(bg.screenAddress + offset) & 0x7FFF];
}

// RenderBackground — tiled BG rendering (modes 0-6, 2/4/8bpp)
//
// Reference: bsnes ppu-fast/background.cpp

void Ppu::RenderBackground(Line& line, const Background& bg, uint8_t source) {
    // Skip inactive layers
    if (bg.tileMode == TileMode::Inactive) return;
    if (!bg.aboveEnable && !bg.belowEnable) return;

    // Mode 7 → separate path
    if (bg.tileMode == TileMode::Mode7) {
        RenderMode7(line, bg, source);
        return;
    }

    // Compute per-layer window masks
    bool winAbove[256], winBelow[256];
    RenderWindow(line, bg.window, bg.window.aboveEnable, winAbove);
    RenderWindow(line, bg.window, bg.window.belowEnable, winBelow);

    // Tile parameters
    bool hires = line.io.bgMode == 5 || line.io.bgMode == 6;
    bool offsetPerTile = line.io.bgMode == 2 || line.io.bgMode == 4 ||
                         line.io.bgMode == 6;
    bool directColorMode = line.io.col.directColor && source == Source::BG1 &&
                           (line.io.bgMode == 3 || line.io.bgMode == 4);

    int tileMode = static_cast<int>(bg.tileMode); // BPP2=0, BPP4=1, BPP8=2
    int colorShift = 3 + tileMode;
    int width = 256 << static_cast<int>(hires);
    uint32_t tileHeight = 3 + static_cast<uint32_t>(bg.tileSize);
    uint32_t tileWidth = !hires ? tileHeight : 4;
    uint32_t tileMask = 0x0FFF >> tileMode;
    uint32_t tiledataIndex = bg.tiledataAddress >> colorShift;
    uint32_t paletteBase = (line.io.bgMode == 0) ? (static_cast<uint32_t>(source) << 5) : 0;
    uint32_t paletteShift = 2 << tileMode;

    uint32_t hscroll = bg.hoffset;
    uint32_t vscroll = bg.voffset;

    uint32_t hmask = (static_cast<uint32_t>(width) <<
                      static_cast<uint32_t>(bg.tileSize) <<
                      static_cast<uint32_t>(!!(bg.screenSize & 1))) - 1;
    uint32_t vmask = (static_cast<uint32_t>(width) <<
                      static_cast<uint32_t>(bg.tileSize) <<
                      static_cast<uint32_t>(!!(bg.screenSize & 2))) - 1;

    // line.y is the hardware vcounter: output row + 1.
    uint32_t y = line.y;
    if (hires) {
        hscroll <<= 1;
        if (line.io.interlace) {
            // bsnes: y = (y << 1) | (field && !mosaicEnable)
            y = (y << 1) | static_cast<uint32_t>(line.fieldID && !bg.mosaicEnable);
        }
    }

    // Mosaic: snap Y position
    if (bg.mosaicEnable) {
        int mosaicDelta = line.io.mosaic.size - line.io.mosaic.counter;
        if (hires && line.io.interlace) mosaicDelta <<= 1;
        y = static_cast<uint32_t>(static_cast<int>(y) - mosaicDelta);
    }

    // Mosaic state for X
    uint16_t mosaicColor = 0;
    uint8_t  mosaicPriority = 0;
    int      mosaicCounter = 0;
    bool     mosaicTransparent = true;

    // Iterate tile columns
    int x = 0 - static_cast<int>(hscroll & 7);
    while (x < width) {
        uint32_t hoffset2 = static_cast<uint32_t>(x) + hscroll;
        uint32_t voffset2 = y + vscroll;

        // Offset-per-tile (modes 2, 4, 6)
        if (offsetPerTile) {
            uint32_t validBit = 0x2000 << source;
            uint32_t offsetX = static_cast<uint32_t>(x) + (hscroll & 7);
            if (offsetX >= 8) {
                uint32_t hLookup = GetTile(line, line.io.bg3,
                    (offsetX - 8) + (line.io.bg3.hoffset & ~7u),
                    line.io.bg3.voffset + 0);

                if (line.io.bgMode == 4) {
                    // Mode 4: single BG3 word, bit 15 → H or V
                    if (hLookup & validBit) {
                        if (!(hLookup & 0x8000))
                            hoffset2 = offsetX + (hLookup & ~7u);
                        else
                            voffset2 = y + hLookup;
                    }
                } else {
                    // Modes 2/6: separate H+V lookups
                    uint32_t vLookup = GetTile(line, line.io.bg3,
                        (offsetX - 8) + (line.io.bg3.hoffset & ~7u),
                        line.io.bg3.voffset + 8);
                    if (hLookup & validBit)
                        hoffset2 = offsetX + (hLookup & ~7u);
                    if (vLookup & validBit)
                        voffset2 = y + vLookup;
                }
            }
        }

        hoffset2 &= hmask;
        voffset2 &= vmask;

        // Fetch tilemap entry
        uint16_t tileEntry = GetTile(line, bg, hoffset2, voffset2);

        uint32_t mirrorY = (tileEntry & 0x8000) ? 7 : 0;
        uint32_t mirrorX = (tileEntry & 0x4000) ? 7 : 0;
        uint8_t  tilePriority = bg.priority[(tileEntry >> 13) & 1];
        uint32_t paletteNumber = (tileEntry >> 10) & 7;
        uint32_t paletteIndex = (paletteBase + (paletteNumber << paletteShift)) & 0xFF;

        uint32_t tileNumber = tileEntry & 0x03FF;

        // 16×16 tile handling — note: additions happen BEFORE 10-bit mask,
        // matching bsnes: (tileNumber & 0x03ff) is applied after +1/+16
        if (tileWidth == 4 && (static_cast<bool>(hoffset2 & 8) ^ static_cast<bool>(mirrorX)))
            tileNumber += 1;
        if (tileHeight == 4 && (static_cast<bool>(voffset2 & 8) ^ static_cast<bool>(mirrorY)))
            tileNumber += 16;

        tileNumber = (((tileNumber & 0x03FF) + tiledataIndex) & tileMask);

        // Fetch character data (bitplane format)
        uint16_t address = static_cast<uint16_t>(
            ((tileNumber << colorShift) + ((voffset2 & 7) ^ mirrorY)) & 0x7FFF);

        uint64_t data = 0;
        data  = static_cast<uint64_t>(vram_[address +  0]) <<  0; // planes 0-1
        if (tileMode >= 1)
            data |= static_cast<uint64_t>(vram_[address +  8]) << 16; // planes 2-3
        if (tileMode >= 2) {
            data |= static_cast<uint64_t>(vram_[address + 16]) << 32; // planes 4-5
            data |= static_cast<uint64_t>(vram_[address + 24]) << 48; // planes 6-7
        }

        // Decode 8 pixels from the packed bitplane data
        for (int tileX = 0; tileX < 8; tileX++, x++) {
            if (x < 0 || x >= width) continue;

            uint32_t shift = mirrorX ? tileX : (7 - tileX);
            uint32_t color = 0;

            color  = static_cast<uint32_t>((data >> (shift +  0)) &   1);
            color += static_cast<uint32_t>((data >> (shift +  7)) &   2);
            if (tileMode >= 1) {
                color += static_cast<uint32_t>((data >> (shift + 14)) &   4);
                color += static_cast<uint32_t>((data >> (shift + 21)) &   8);
            }
            if (tileMode >= 2) {
                color += static_cast<uint32_t>((data >> (shift + 28)) &  16);
                color += static_cast<uint32_t>((data >> (shift + 35)) &  32);
                color += static_cast<uint32_t>((data >> (shift + 42)) &  64);
                color += static_cast<uint32_t>((data >> (shift + 49)) & 128);
            }

            // Mosaic
            if (bg.mosaicEnable && line.io.mosaic.size > 1) {
                if (mosaicCounter == 0) {
                    mosaicCounter = line.io.mosaic.size;
                    mosaicTransparent = (color == 0);
                    if (!mosaicTransparent) {
                        if (!directColorMode) {
                            mosaicColor = line.cgram[(paletteIndex + color) & 0xFF];
                        } else {
                            mosaicColor = DirectColor(
                                static_cast<uint8_t>(paletteNumber),
                                static_cast<uint8_t>(color));
                        }
                    }
                    mosaicPriority = tilePriority;
                }
                mosaicCounter--;

                if (mosaicTransparent) continue;

                if (!hires) {
                    if (bg.aboveEnable && !winAbove[x])
                        PlotAbove(line, x, source, mosaicPriority, mosaicColor);
                    if (bg.belowEnable && !winBelow[x])
                        PlotBelow(line, x, source, mosaicPriority, mosaicColor);
                } else {
                    int X = x >> 1;
                    if (x & 1) {
                        if (bg.aboveEnable && !winAbove[X])
                            PlotAbove(line, X, source, mosaicPriority, mosaicColor);
                    } else {
                        if (bg.belowEnable && !winBelow[X])
                            PlotBelow(line, X, source, mosaicPriority, mosaicColor);
                    }
                }
                continue;
            }

            // Color 0 = transparent
            if (color == 0) continue;

            // Resolve final color
            uint16_t finalColor;
            if (!directColorMode) {
                finalColor = line.cgram[(paletteIndex + color) & 0xFF];
            } else {
                finalColor = DirectColor(
                    static_cast<uint8_t>(paletteNumber),
                    static_cast<uint8_t>(color));
            }

            // Plot to above/below based on window and enable flags
            if (!hires) {
                if (bg.aboveEnable && !winAbove[x])
                    PlotAbove(line, x, source, tilePriority, finalColor);
                if (bg.belowEnable && !winBelow[x])
                    PlotBelow(line, x, source, tilePriority, finalColor);
            } else {
                int X = x >> 1;
                if (x & 1) {
                    if (bg.aboveEnable && !winAbove[X])
                        PlotAbove(line, X, source, tilePriority, finalColor);
                } else {
                    if (bg.belowEnable && !winBelow[X])
                        PlotBelow(line, X, source, tilePriority, finalColor);
                }
            }
        }
    }
}

// RenderMode7 — Mode 7 affine-transformed background
//
// Reference: bsnes ppu-fast/mode7.cpp

void Ppu::RenderMode7(Line& line, const Background& bg, uint8_t source) {
    if (!bg.aboveEnable && !bg.belowEnable) return;

    bool winAbove[256], winBelow[256];
    RenderWindow(line, bg.window, bg.window.aboveEnable, winAbove);
    RenderWindow(line, bg.window, bg.window.belowEnable, winBelow);

    // Mode 7 matrix parameters (signed)
    int a = static_cast<int16_t>(line.io.mode7.a);
    int b = static_cast<int16_t>(line.io.mode7.b);
    int c = static_cast<int16_t>(line.io.mode7.c);
    int d = static_cast<int16_t>(line.io.mode7.d);

    // 13-bit sign extension
    auto signExtend13 = [](uint16_t v) -> int {
        return static_cast<int>(v & 0x1FFF) - ((v & 0x1000) ? 0x2000 : 0);
    };

    int hcenter = signExtend13(line.io.mode7.x);
    int vcenter = signExtend13(line.io.mode7.y);
    int hoffset = signExtend13(line.io.mode7.hoffset);
    int voffset = signExtend13(line.io.mode7.voffset);

    auto clip = [](int n) -> int {
        return (n & 0x2000) ? (n | ~1023) : (n & 1023);
    };

    // Apply mosaic to the visible row before the Mode 7 vertical flip.
    int y = line.y;
    if (bg.mosaicEnable) y -= line.io.mosaic.size - line.io.mosaic.counter;
    if (line.io.mode7.vflip) y = 255 - y;

    int originX = (a * clip(hoffset - hcenter) & ~63)
                + (b * clip(voffset - vcenter) & ~63)
                + (b * y & ~63)
                + hcenter * 256;
    int originY = (c * clip(hoffset - hcenter) & ~63)
                + (d * clip(voffset - vcenter) & ~63)
                + (d * y & ~63)
                + vcenter * 256;

    // Mosaic state
    uint16_t mosaicColor = 0;
    uint8_t  mosaicPriority = 0;
    int      mosaicCounter = 0;
    bool     mosaicTransparent = true;

    for (int X = 0; X < 256; X++) {
        int sx = line.io.mode7.hflip ? (255 - X) : X;

        int pixelX = (originX + a * sx) >> 8;
        int pixelY = (originY + c * sx) >> 8;

        int tileX = (pixelX >> 3) & 127;
        int tileY = (pixelY >> 3) & 127;
        bool outOfBounds = (pixelX | pixelY) & ~1023;

        // Mode 7 VRAM layout:
        // Low bytes of VRAM[0..16383] = tile numbers (128×128 tilemap)
        // High bytes of VRAM[tile*64 + pixel_offset] = pixel data
        uint16_t tileAddress = static_cast<uint16_t>(tileY * 128 + tileX);
        uint8_t tile = (line.io.mode7.repeat == 3 && outOfBounds)
                           ? 0
                           : static_cast<uint8_t>(vram_[tileAddress & 0x7FFF] & 0xFF);

        uint16_t palAddress = static_cast<uint16_t>(
            (tile << 6) | ((pixelY & 7) << 3) | (pixelX & 7));
        uint8_t palette = (line.io.mode7.repeat == 2 && outOfBounds)
                              ? 0
                              : static_cast<uint8_t>((vram_[palAddress & 0x7FFF] >> 8) & 0xFF);

        uint8_t tilePriority;
        if (source == Source::BG1) {
            tilePriority = bg.priority[0];
        } else {
            // EXTBG BG2: bit 7 of palette = priority select
            tilePriority = bg.priority[palette >> 7];
            palette &= 0x7F;
        }

        // Mosaic
        if (bg.mosaicEnable && line.io.mosaic.size > 1) {
            if (mosaicCounter == 0) {
                mosaicCounter = line.io.mosaic.size;
                mosaicTransparent = (palette == 0);
                if (!mosaicTransparent) {
                    mosaicColor = line.cgram[palette];
                }
                mosaicPriority = tilePriority;
            }
            mosaicCounter--;

            if (mosaicTransparent) continue;
            if (bg.aboveEnable && !winAbove[X])
                PlotAbove(line, X, source, mosaicPriority, mosaicColor);
            if (bg.belowEnable && !winBelow[X])
                PlotBelow(line, X, source, mosaicPriority, mosaicColor);
            continue;
        }

        if (palette == 0) continue;

        uint16_t finalColor = line.cgram[palette];

        if (bg.aboveEnable && !winAbove[X])
            PlotAbove(line, X, source, tilePriority, finalColor);
        if (bg.belowEnable && !winBelow[X])
            PlotBelow(line, X, source, tilePriority, finalColor);
    }
}

// RenderObjects — sprite rendering
//
// Reference: bsnes ppu-fast/object.cpp

void Ppu::RenderObjects(Line& line, const ObjectIO& obj) {
    if (!obj.aboveEnable && !obj.belowEnable) return;

    // Phase 1: Sprite evaluation — find which sprites are on this scanline
    bool winAbove[256], winBelow[256];
    RenderWindow(line, obj.window, obj.window.aboveEnable, winAbove);
    RenderWindow(line, obj.window, obj.window.belowEnable, winBelow);

    int itemCount = 0;
    int tileCount = 0;
    constexpr int ItemLimit = 32;
    constexpr int TileLimit = 34;
    bool rangeOver = false;
    bool timeOver  = false;

    // Clear items/tiles
    for (int i = 0; i < 128; i++) {
        line.items[i] = {false, 0, 0, 0};
        line.tiles[i] = {};
    }

    uint16_t y = static_cast<uint16_t>(line.y & 0xFF);

    for (int n = 0; n < 128; n++) {
        uint8_t idx = static_cast<uint8_t>((obj.first + n) & 127);
        const auto& object = objects_[idx];

        // Determine sprite dimensions
        uint8_t w, h;
        if (!object.size) {
            w = kObjSmallWidth[obj.baseSize];
            h = kObjSmallHeight[obj.baseSize];
            if (obj.interlace && obj.baseSize >= 6) h = 16; // hardware quirk
        } else {
            w = kObjLargeWidth[obj.baseSize];
            h = kObjLargeHeight[obj.baseSize];
        }

        // Off-screen X check: if sprite starts beyond 256 and doesn't wrap onto screen
        if (object.x > 256 && static_cast<int>(object.x) + w - 1 < 512) continue;

        // Y range check in 8-bit space (raw OAM Y and wrapped scanline).
        uint32_t effectiveH = h >> static_cast<uint32_t>(obj.interlace);
        uint32_t objY = object.y;
        uint32_t lineY = y;

        // Check Y range (with 8-bit wrapping)
        bool inRange = false;
        if (lineY >= objY && lineY < objY + effectiveH) {
            inRange = true;
        } else if (objY + effectiveH >= 256 && lineY < ((objY + effectiveH) & 0xFF)) {
            inRange = true;
        }

        if (!inRange) continue;

        if (itemCount >= ItemLimit) { rangeOver = true; break; }
        line.items[itemCount] = {true, idx, w, h};
        itemCount++;
    }

    // Phase 2: Tile fetching (reverse order — lower index = higher priority)
    for (int n = itemCount - 1; n >= 0; n--) {
        const auto& item = line.items[n];
        if (!item.valid) continue;

        const auto& object = objects_[item.index];

        uint32_t w = item.width;
        uint32_t h = item.height;
        uint32_t tileWidth = w >> 3; // tiles across

        // Compute Y within sprite
        int spriteY = static_cast<int>((y - object.y) & 0xFF);
        if (obj.interlace) spriteY <<= 1;

        // V-flip: non-square sprites flip within each width-sized block
        if (object.vflip) {
            if (w == h) {
                spriteY = static_cast<int>(h - 1) - spriteY;
            } else if (spriteY < static_cast<int>(w)) {
                spriteY = static_cast<int>(w - 1) - spriteY;
            } else {
                spriteY = static_cast<int>(w) + static_cast<int>(w - 1)
                        - (spriteY - static_cast<int>(w));
            }
        }

        // Interlace field offset
        if (obj.interlace) {
            spriteY = !object.vflip ? spriteY + static_cast<int>(line.fieldID)
                                    : spriteY - static_cast<int>(line.fieldID);
        }

        uint32_t sx = static_cast<uint32_t>(object.x) & 0x1FF;
        spriteY &= 0xFF;

        // Tile data base address
        uint16_t tiledataAddr = obj.tiledataAddress;
        if (object.nameselect) {
            // OBSEL nameselect selects an alternate character page at
            // ((1 + nameselect) << 12) words from the base tile data address.
            tiledataAddr += static_cast<uint16_t>((1u + obj.nameselect) << 12);
        }

        uint16_t characterX = object.character & 0x0F;
        uint16_t characterY = static_cast<uint16_t>(
            (((object.character >> 4) + (static_cast<uint32_t>(spriteY) >> 3)) & 0x0F) << 4);

        for (uint32_t tileX = 0; tileX < tileWidth; tileX++) {
            uint32_t objectX = (sx + (tileX << 3)) & 0x1FF;
            // Skip tiles entirely off-screen (x=256 sprites are not skipped)
            if (sx != 256 && objectX >= 256 && objectX + 7 < 512) continue;

            if (tileCount >= TileLimit) { timeOver = true; break; }

            uint32_t mirrorX = object.hflip ? (tileWidth - 1 - tileX) : tileX;
            uint16_t address = tiledataAddr +
                static_cast<uint16_t>(
                    ((characterY + ((characterX + mirrorX) & 15)) << 4));
            address = (address & 0x7FF0) + static_cast<uint16_t>(spriteY & 7);

            ObjectTile tile;
            tile.valid    = true;
            tile.x        = static_cast<uint16_t>(objectX);
            tile.priority = object.priority;
            tile.palette  = static_cast<uint8_t>(128 + (object.palette << 4));
            tile.hflip    = object.hflip;

            // 4bpp tile data (2 VRAM words = 32 bits)
            uint32_t d = 0;
            d  = static_cast<uint32_t>(vram_[(address + 0) & 0x7FFF]) <<  0;
            d |= static_cast<uint32_t>(vram_[(address + 8) & 0x7FFF]) << 16;
            tile.data = d;

            line.tiles[tileCount] = tile;
            tileCount++;
        }
    }

    // Set overflow flags (|= because flags are sticky until read)
    io_.obj.rangeOver |= rangeOver;
    io_.obj.timeOver  |= timeOver;

    // Phase 3: Pixel decode into temporary buffers
    uint8_t paletteArr[256] = {};
    uint8_t priorityArr[256] = {};

    for (int n = 0; n < tileCount && n < TileLimit; n++) {
        const auto& tile = line.tiles[n];
        if (!tile.valid) continue;

        uint32_t tileX = tile.x;
        for (int px = 0; px < 8; px++) {
            tileX &= 511;
            if (tileX < 256) {
                uint32_t shift = tile.hflip ? static_cast<uint32_t>(px)
                                            : (7 - static_cast<uint32_t>(px));
                uint32_t color = 0;
                color  = (tile.data >> (shift +  0)) & 1;
                color += (tile.data >> (shift +  7)) & 2;
                color += (tile.data >> (shift + 14)) & 4;
                color += (tile.data >> (shift + 21)) & 8;

                if (color != 0) {
                    paletteArr[tileX] = tile.palette + static_cast<uint8_t>(color);
                    priorityArr[tileX] = obj.priority[tile.priority];
                }
            }
            tileX++;
        }
    }

    // Phase 4: Plot to above/below
    for (int x = 0; x < 256; x++) {
        if (priorityArr[x] == 0) continue;

        // OBJ1 = palettes 0-3 (indices < 192), OBJ2 = palettes 4-7 (indices >= 192)
        uint8_t src = (paletteArr[x] < 192) ? Source::OBJ1 : Source::OBJ2;
        uint16_t color = line.cgram[paletteArr[x]];

        if (obj.aboveEnable && !winAbove[x])
            PlotAbove(line, x, src, priorityArr[x], color);
        if (obj.belowEnable && !winBelow[x])
            PlotBelow(line, x, src, priorityArr[x], color);
    }
}

// RenderWindow — compute window mask for a BG/OBJ layer
//
// output[x] = true means pixel is INSIDE the window → will be clipped

void Ppu::RenderWindow(const Line& line, const WindowLayer& wl, bool enable,
                       bool output[256]) {
    if (!enable || (!wl.oneEnable && !wl.twoEnable)) {
        std::memset(output, 0, 256 * sizeof(bool));
        return;
    }

    if (wl.oneEnable && !wl.twoEnable) {
        // Only window 1
        for (int x = 0; x < 256; x++) {
            bool inside = (x >= line.io.window.oneLeft && x <= line.io.window.oneRight);
            output[x] = inside ^ wl.oneInvert;
        }
        return;
    }

    if (!wl.oneEnable && wl.twoEnable) {
        // Only window 2
        for (int x = 0; x < 256; x++) {
            bool inside = (x >= line.io.window.twoLeft && x <= line.io.window.twoRight);
            output[x] = inside ^ wl.twoInvert;
        }
        return;
    }

    // Both windows enabled — combine with mask logic
    for (int x = 0; x < 256; x++) {
        bool one = (x >= line.io.window.oneLeft && x <= line.io.window.oneRight) ^ wl.oneInvert;
        bool two = (x >= line.io.window.twoLeft && x <= line.io.window.twoRight) ^ wl.twoInvert;
        switch (wl.mask) {
        case 0: output[x] = one | two;   break; // OR
        case 1: output[x] = one & two;   break; // AND
        case 2: output[x] = one ^ two;   break; // XOR
        case 3: output[x] = !(one ^ two); break; // XNOR
        }
    }
}

// RenderWindowColor — compute window mask for color math
//
// mask: 0=always on, 1=inside window only, 2=outside window only, 3=never
// output[x] = true means color math/display is ENABLED at this pixel

void Ppu::RenderWindowColor(const Line& line, const WindowColor& wc,
                            uint8_t mask, bool output[256]) {
    switch (mask) {
    case 0: // Always
        for (int x = 0; x < 256; x++) output[x] = true;
        return;
    case 3: // Never
        for (int x = 0; x < 256; x++) output[x] = false;
        return;
    default:
        break;
    }

    // Compute window region first
    bool windowRegion[256];

    if (!wc.oneEnable && !wc.twoEnable) {
        std::memset(windowRegion, 0, 256 * sizeof(bool));
    } else if (wc.oneEnable && !wc.twoEnable) {
        for (int x = 0; x < 256; x++) {
            bool inside = (x >= line.io.window.oneLeft && x <= line.io.window.oneRight);
            windowRegion[x] = inside ^ wc.oneInvert;
        }
    } else if (!wc.oneEnable && wc.twoEnable) {
        for (int x = 0; x < 256; x++) {
            bool inside = (x >= line.io.window.twoLeft && x <= line.io.window.twoRight);
            windowRegion[x] = inside ^ wc.twoInvert;
        }
    } else {
        for (int x = 0; x < 256; x++) {
            bool one = (x >= line.io.window.oneLeft && x <= line.io.window.oneRight) ^ wc.oneInvert;
            bool two = (x >= line.io.window.twoLeft && x <= line.io.window.twoRight) ^ wc.twoInvert;
            switch (wc.mask) {
            case 0: windowRegion[x] = one | two;   break;
            case 1: windowRegion[x] = one & two;   break;
            case 2: windowRegion[x] = one ^ two;   break;
            case 3: windowRegion[x] = !(one ^ two); break;
            }
        }
    }

    // mask=1: inside window → enabled
    // mask=2: outside window → enabled
    for (int x = 0; x < 256; x++) {
        output[x] = (mask == 1) ? windowRegion[x] : !windowRegion[x];
    }
}

// CompositePixel — color math compositing per pixel
//
// Reference: bsnes ppu-fast/line.cpp pixel()

uint16_t Ppu::CompositePixel(const Line& line, int x,
                             Pixel above, Pixel below) const {
    // If color window masks main screen → black
    if (!line.windowAbove[x]) above.color = 0x0000;

    // If color window masks sub screen → no blending, return main screen
    if (!line.windowBelow[x]) return above.color;

    // If color math not enabled for this source layer → return main screen
    if (!line.io.col.enable[above.source]) return above.color;

    // Blend with fixed color or sub screen
    if (!line.io.col.blendMode) {
        // Fixed color mode
        return Blend(line, above.color, line.io.col.fixedColor,
                     line.io.col.halve && line.windowAbove[x]);
    }

    // Sub screen mode
    return Blend(line, above.color, below.color,
                 line.io.col.halve && line.windowAbove[x] &&
                 below.source != Source::COL);
}

// Blend — saturating add/subtract on packed BGR555
//
// Reference: bsnes ppu-fast/line.cpp blend()

uint16_t Ppu::Blend(const Line& line, uint16_t x, uint16_t y,
                    bool halve) const {
    if (!line.io.col.mathMode) {
        // ADD
        if (!halve) {
            uint32_t sum   = static_cast<uint32_t>(x) + static_cast<uint32_t>(y);
            uint32_t carry = (sum - ((x ^ y) & 0x0421)) & 0x8420;
            return static_cast<uint16_t>((sum - carry) | (carry - (carry >> 5)));
        } else {
            return static_cast<uint16_t>(
                (static_cast<uint32_t>(x) + static_cast<uint32_t>(y)
                 - ((x ^ y) & 0x0421)) >> 1);
        }
    } else {
        // SUBTRACT
        uint32_t diff   = static_cast<uint32_t>(x) - static_cast<uint32_t>(y) + 0x8420;
        uint32_t borrow = (diff - ((x ^ y) & 0x8420)) & 0x8420;
        if (!halve) {
            return static_cast<uint16_t>((diff - borrow) & (borrow - (borrow >> 5)));
        } else {
            return static_cast<uint16_t>(
                (((diff - borrow) & (borrow - (borrow >> 5))) & 0x7BDE) >> 1);
        }
    }
}

} // namespace snes::core
