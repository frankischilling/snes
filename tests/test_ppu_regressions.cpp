// snes emulator
// tests/test_ppu_regressions.cpp
// Regression coverage for PPU registers and rendering.

#include "snes/core/Ppu.hpp"

#include <cstdio>
#include <memory>

using snes::core::Ppu;

namespace {
int failures = 0;

void Expect(bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

void Write16(Ppu& ppu, uint16_t reg, uint16_t value) {
    ppu.WriteIO(reg, static_cast<uint8_t>(value));
    ppu.WriteIO(reg, static_cast<uint8_t>(value >> 8));
}

uint16_t Pixel(const Ppu& ppu, int x, int y) {
    const uint32_t pixel = ppu.OutputData()[y * Ppu::OutputWidth + x];
    return static_cast<uint16_t>(((pixel >> 3) & 31) |
                                ((pixel >> 6) & 0x3e0) |
                                ((pixel >> 9) & 0x7c00));
}

void Render(Ppu& ppu) {
    ppu.FrameBegin();
    for (uint16_t v = 1; v <= 224; ++v) ppu.ScanlineBegin(v);
    ppu.VBlankBegin();
}

void RenderLines(Ppu& ppu, uint16_t count) {
    ppu.FrameBegin();
    for (uint16_t v = 1; v <= count; ++v) ppu.ScanlineBegin(v);
    ppu.VBlankBegin();
}

uint16_t RasterClock(int x) {
    return static_cast<uint16_t>(88 + x * 4);
}

void Fill2bppTile(Ppu& ppu, uint16_t base, uint16_t tile, uint8_t color) {
    const uint16_t row = static_cast<uint16_t>(((color & 1) ? 0x00ff : 0) |
                                               ((color & 2) ? 0xff00 : 0));
    for (int y = 0; y < 8; ++y) ppu.VramData()[base + tile * 8 + y] = row;
}

void SetupBg1(Ppu& ppu) {
    ppu.WriteIO(0x2100, 0x0f);
    ppu.WriteIO(0x2105, 0);
    ppu.WriteIO(0x210b, 1);
    ppu.WriteIO(0x212c, 1);
    ppu.CgramData()[1] = 0x001f;
    ppu.CgramData()[2] = 0x03e0;
    Fill2bppTile(ppu, 0x1000, 0, 1);
    Fill2bppTile(ppu, 0x1000, 1, 2);
}

void BackgroundRows() {
    // Source Y = output row + BG VOFS + 1.
    for (uint8_t mode = 0; mode <= 4; ++mode) {
        for (uint16_t scroll : {0, 3, 255, 1023}) {
            auto ppu = std::make_unique<Ppu>();
            ppu->WriteIO(0x2100, 0x0f);
            ppu->WriteIO(0x2105, mode);
            ppu->WriteIO(0x210b, 1);
            ppu->WriteIO(0x212c, 1);
            Write16(*ppu, 0x210e, scroll);
            ppu->CgramData()[1] = 0x001f;
            ppu->CgramData()[2] = 0x03e0;
            for (int row = 0; row < 8; ++row)
                ppu->VramData()[0x1000 + row] = row % 2 ? 0xff00 : 0x00ff;
            Render(*ppu);
            for (int y : {0, 1, 6, 7, 8, 222, 223}) {
                const auto expected = (y + scroll + 1) % 2 ? 0x03e0 : 0x001f;
                Expect(Pixel(*ppu, 0, y) == expected, "BG source row includes one visible-line offset");
            }
        }
    }
}

void SpriteFloorAlignment() {
    for (bool large : {false, true}) {
        const int height = large ? 16 : 8;
        for (uint8_t top : {0, 16, 208, 248, 255}) {
            auto ppu = std::make_unique<Ppu>();
            ppu->WriteIO(0x2100, 0x0f);
            ppu->WriteIO(0x2105, 1);
            ppu->WriteIO(0x210b, 1);
            ppu->WriteIO(0x212c, 0x11);
            ppu->CgramData()[1] = 0x03e0;
            ppu->CgramData()[129] = 0x001f;
            // A solid floor tile starts at map Y=32. Put it just below the sprite.
            Write16(*ppu, 0x210e, static_cast<uint16_t>((32 - top - height - 1) & 1023));
            for (int x = 0; x < 32; ++x) ppu->VramData()[4 * 32 + x] = 1;
            for (int r = 0; r < 8; ++r) {
                ppu->VramData()[0x1010 + r] = 0x00ff;
                ppu->VramData()[0x4000 + r] = 0x00ff;
                ppu->VramData()[0x4010 + r] = 0x00ff;
                ppu->VramData()[0x4100 + r] = 0x00ff;
                ppu->VramData()[0x4110 + r] = 0x00ff;
            }
            ppu->WriteIO(0x2101, 2); // OBJ tile data at word address $4000.
            for (int i = 0; i < 128; ++i) ppu->OamData()[4 * i + 1] = 240;
            ppu->OamData()[1] = top;
            ppu->OamData()[3] = 0x30;
            ppu->OamData()[512] = large ? 2 : 0;
            Render(*ppu);
            for (int y = 0; y < 224; ++y) {
                const bool inSprite = ((y - top) & 255) < height;
                Expect((Pixel(*ppu, 0, y) == 0x001f) == inSprite,
                       "OBJ top, bottom and Y wrap match raw OAM coordinates");
            }
            const int floorY = (top + height) & 255;
            if (floorY < 224) {
                Expect(Pixel(*ppu, 0, floorY) == 0x03e0, "floor starts immediately below sprite feet");
                if (floorY > 0)
                    Expect(Pixel(*ppu, 32, floorY - 1) == 0, "floor beside sprite stays below the feet");
                if (floorY > 0)
                    Expect(Pixel(*ppu, 7, floorY - 1) == 0x001f, "feet stay above the floor");
            }
        }
    }
}

void Mode7Coordinates() {
    // Offsets are signed 13-bit values; Y is Line+1, optionally flipped.
    for (uint8_t flips : {0, 1, 2, 3}) {
        for (uint16_t scroll : {0x0000, 0x1fff, 0xffff}) {
            auto ppu = std::make_unique<Ppu>();
            ppu->WriteIO(0x2100, 0x0f);
            ppu->WriteIO(0x2105, 7);
            ppu->WriteIO(0x212c, 1);
            ppu->WriteIO(0x211a, static_cast<uint8_t>(0x80 | flips)); // transparent outside map
            Write16(*ppu, 0x211b, 0x0100);
            Write16(*ppu, 0x211e, 0x0100);
            Write16(*ppu, 0x210d, scroll);
            Write16(*ppu, 0x210e, scroll);
            for (int color = 1; color <= 64; ++color) ppu->CgramData()[color] = color;
            // Tile 0 repeats over the map; each pixel has a distinct color.
            for (int i = 0; i < 64; ++i) ppu->VramData()[i] = static_cast<uint16_t>((i + 1) << 8);
            Render(*ppu);
            const int offset = scroll == 0 ? 0 : -1;
            for (int y : {0, 1, 6, 7, 223}) {
                for (int x : {0, 1, 7, 255}) {
                    const int sx = ((flips & 1) ? 255 - x : x) + offset;
                    const int sy = ((flips & 2) ? 255 - (y + 1) : y + 1) + offset;
                    const uint16_t expected = sx < 0 || sy < 0 ? 0 : (sy % 8) * 8 + sx % 8 + 1;
                    Expect(Pixel(*ppu, x, y) == expected, "Mode 7 row, flip and signed scroll coordinates");
                }
            }
        }
    }
}

void Mode7VerticalMosaic() {
    auto ppu = std::make_unique<Ppu>();
    ppu->WriteIO(0x2100, 0x0f);
    ppu->WriteIO(0x2105, 7);
    ppu->WriteIO(0x2106, 0x31); // BG1 mosaic, four pixels.
    ppu->WriteIO(0x212c, 1);
    Write16(*ppu, 0x211b, 0x0100);
    Write16(*ppu, 0x211e, 0x0100);
    for (int i = 0; i < 64; ++i) ppu->VramData()[i] = static_cast<uint16_t>((i / 8 + 1) << 8);
    for (int i = 1; i <= 8; ++i) ppu->CgramData()[i] = i;
    Render(*ppu);
    for (int y = 0; y < 12; ++y)
        Expect(Pixel(*ppu, 0, y) == ((y / 4 * 4 + 1) % 8) + 1,
               "Mode 7 mosaic repeats the first source row of each block");
}
void HighResolutionOutput() {
    for (uint8_t mode : {0, 5, 6}) {
        auto ppu = std::make_unique<Ppu>();
        ppu->WriteIO(0x2100, 0x0f);
        ppu->WriteIO(0x2105, mode);
        ppu->WriteIO(0x210b, 1);
        ppu->WriteIO(0x2133, mode == 0 ? 8 : 0);
        ppu->WriteIO(0x212c, 1);
        ppu->WriteIO(0x212d, mode == 0 ? 0 : 1);
        ppu->CgramData()[0] = 0x7c00;
        ppu->CgramData()[1] = 0x001f;
        ppu->CgramData()[2] = 0x03e0;
        for (int row = 0; row < 8; ++row) {
            ppu->VramData()[0x1000 + row] = mode == 0 ? 0xff : 0x55aa;
            ppu->VramData()[0x1010 + row] = 0x55aa;
        }
        Render(*ppu);
        Expect(ppu->FrameWidth() == 512, "Hires output retains 512 dots");
        for (int x : {0, 2, 128, 254, 510}) {
            Expect(Pixel(*ppu, x, 0) == (mode == 0 ? 0x7c00 : 0x001f), "Even output dot comes from sub screen");
            Expect(Pixel(*ppu, x + 1, 0) == (mode == 0 ? 0x001f : 0x03e0), "Odd output dot comes from main screen");
        }
        // A forced blank line must clear the entire wide output row.
        ppu->WriteIO(0x2100, 0x80);
        Render(*ppu);
        Expect(Pixel(*ppu, 511, 0) == 0, "Forced blank clears right edge of hires frame");
    }
}

void MixedWidthsAndFields() {
    auto ppu = std::make_unique<Ppu>();
    ppu->WriteIO(0x2100, 0x0f);
    ppu->WriteIO(0x2105, 0);
    ppu->WriteIO(0x210b, 1);
    ppu->WriteIO(0x212c, 1);
    ppu->CgramData()[1] = 0x001f;
    ppu->CgramData()[2] = 0x03e0;
    for (int row = 0; row < 8; ++row) ppu->VramData()[0x1000 + row] = 0x55aa;
    ppu->FrameBegin();
    ppu->ScanlineBegin(1);
    ppu->WriteIO(0x2133, 8);
    ppu->ScanlineBegin(2);
    ppu->VBlankBegin();
    Expect(ppu->FrameWidth() == 512, "One hires line widens the output frame");
    for (int x = 0; x < 512; ++x)
        Expect(Pixel(*ppu, x, 0) == ((x / 2) % 2 ? 0x03e0 : 0x001f), "Low resolution lines duplicate dots in mixed frames");

    ppu->WriteIO(0x212c, 0);
    ppu->WriteIO(0x2133, 1);
    ppu->CgramData()[0] = 0x001f;
    Render(*ppu);
    const bool firstField = ppu->FieldID();
    Expect(ppu->FrameHeight() == 448, "Interlace exposes both fields");
    Expect(Pixel(*ppu, 0, firstField) == 0x001f, "First field uses its own row parity");
    ppu->CgramData()[0] = 0x03e0;
    Render(*ppu);
    Expect(Pixel(*ppu, 0, firstField) == 0x001f && Pixel(*ppu, 0, !firstField) == 0x03e0,
           "Second field preserves the preceding field");
    ppu->WriteIO(0x2133, 0);
    Render(*ppu);
    Expect(ppu->FrameWidth() == 256 && ppu->FrameHeight() == 224, "Leaving interlace restores normal frame dimensions");
}
void RasterMemoryChanges() {
    for (bool tilemap : {false, true}) {
        auto ppu = std::make_unique<Ppu>();
        ppu->WriteIO(0x2100, 0x0f);
        ppu->WriteIO(0x210b, 1);
        ppu->WriteIO(0x212c, 1);
        ppu->CgramData()[1] = 0x001f;
        ppu->CgramData()[2] = 0x03e0;
        for (int row = 0; row < 8; ++row) {
            ppu->VramData()[0x1000 + row] = 0x00ff;
            ppu->VramData()[0x1008 + row] = 0xff00;
        }
        ppu->FrameBegin();
        ppu->ScanlineBegin(1);
        if (tilemap) ppu->VramData()[0] = 1;
        else for (int row = 0; row < 8; ++row) ppu->VramData()[0x1000 + row] = 0xff00;
        ppu->ScanlineBegin(2);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 0, 0) == 0x001f && Pixel(*ppu, 0, 1) == 0x03e0,
               "Tile and tilemap updates preserve earlier scanlines");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        ppu->WriteIO(0x2100, 0x0f);
        ppu->WriteIO(0x2105, 7);
        ppu->WriteIO(0x212c, 1);
        Write16(*ppu, 0x211b, 0x0100);
        Write16(*ppu, 0x211e, 0x0100);
        ppu->CgramData()[1] = 0x001f;
        ppu->CgramData()[2] = 0x03e0;
        for (int i = 0; i < 64; ++i) ppu->VramData()[i] = 0x0100;
        ppu->FrameBegin();
        ppu->ScanlineBegin(1);
        for (int i = 0; i < 64; ++i) ppu->VramData()[i] = 0x0200;
        ppu->ScanlineBegin(2);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 0, 0) == 0x001f && Pixel(*ppu, 0, 1) == 0x03e0,
               "Mode 7 reads each line's tile memory");
    }
    for (bool tileData : {false, true}) {
        auto ppu = std::make_unique<Ppu>();
        ppu->WriteIO(0x2100, 0x0f);
        ppu->WriteIO(0x2101, 2);
        ppu->WriteIO(0x212c, 0x10);
        ppu->CgramData()[129] = 0x001f;
        ppu->CgramData()[130] = 0x03e0;
        for (int i = 0; i < 128; ++i) ppu->OamData()[4 * i + 1] = 240;
        ppu->OamData()[1] = 0;
        ppu->OamData()[3] = 0x30;
        for (int row = 0; row < 8; ++row) ppu->VramData()[0x4000 + row] = 0x00ff;
        ppu->FrameBegin();
        ppu->ScanlineBegin(1);
        if (tileData) for (int row = 0; row < 8; ++row) ppu->VramData()[0x4000 + row] = 0xff00;
        else ppu->OamData()[0] = 16;
        ppu->ScanlineBegin(2);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 0, 0) == 0x001f, "Later OBJ changes preserve earlier sprite rows");
        Expect(Pixel(*ppu, tileData ? 0 : 16, 1) == (tileData ? 0x03e0 : 0x001f),
               "Later sprite rows use updated OAM and tile memory");
        if (!tileData) Expect(Pixel(*ppu, 0, 1) == 0, "Moved sprite leaves its old position");
    }
}

void RasterRegisterChanges() {
    {
        auto ppu = std::make_unique<Ppu>();
        ppu->WriteIO(0x2100, 0x0f);
        ppu->CgramData()[0] = 0x001f;
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->SetCurrentHClock(RasterClock(64));
        ppu->WriteIO(0x2100, 0x07); // Before the H=512 line snapshot.
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(192));
        ppu->WriteIO(0x2100, 0x0f);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 32, 0) == 0x001f, "Raster brightness preserves the pre-write left span");
        Expect(Pixel(*ppu, 100, 0) == 14, "Raster brightness changes only the timestamped middle span");
        Expect(Pixel(*ppu, 220, 0) == 0x001f, "Raster brightness restores on a later write");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        ppu->WriteIO(0x2100, 0x0f);
        ppu->CgramData()[0] = 0x03e0;
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->SetCurrentHClock(RasterClock(80));
        ppu->WriteIO(0x2100, 0x8f);
        ppu->ScanlineBegin(1); // Memory must still be cached while the sample is blanked.
        ppu->SetCurrentHClock(RasterClock(160));
        ppu->WriteIO(0x2100, 0x0f);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 40, 0) == 0x03e0, "Forced blank keeps pixels before the write");
        Expect(Pixel(*ppu, 120, 0) == 0, "Forced blank blacks only its active span");
        Expect(Pixel(*ppu, 200, 0) == 0x03e0, "Display resumes after a mid-line forced-blank clear");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        for (int x = 0; x < 32; ++x) ppu->VramData()[x] = static_cast<uint16_t>(x & 1);
        Write16(*ppu, 0x210d, 0);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        Write16(*ppu, 0x210d, 8);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 100, 0) == 0x001f, "Raster scroll leaves the earlier tile columns unchanged");
        Expect(Pixel(*ppu, 132, 0) == 0x03e0, "Raster scroll uses the new offset after the write");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        ppu->WriteIO(0x2126, 0);
        ppu->WriteIO(0x2127, 255);
        ppu->WriteIO(0x212e, 1);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x2123, 0x02);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 64, 0) == 0x001f, "Window selection keeps the earlier span unmasked");
        Expect(Pixel(*ppu, 192, 0) == 0, "Window selection masks only pixels after its timestamp");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        ppu->WriteIO(0x2100, 0x0f);
        ppu->WriteIO(0x2105, 0);
        ppu->WriteIO(0x210b, 1);
        ppu->WriteIO(0x212c, 1);
        ppu->CgramData()[1] = 0x001f;
        ppu->CgramData()[2] = 0x03e0;
        for (int row = 0; row < 8; ++row) ppu->VramData()[0x1000 + row] = 0x55aa;
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x2106, 0x31);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 126, 0) != Pixel(*ppu, 127, 0), "Pixels before the mosaic write stay unmosaiced");
        Expect(Pixel(*ppu, 128, 0) == Pixel(*ppu, 129, 0) &&
               Pixel(*ppu, 128, 0) == Pixel(*ppu, 131, 0),
               "Mosaic starts in the span following its timestamp");
    }
}

uint16_t Mode7DirectColor(uint8_t color) {
    return static_cast<uint16_t>(((color << 2) & 0x001c) |
                                 ((color << 4) & 0x0380) |
                                 ((color << 7) & 0x6000));
}

void SetupMode7(Ppu& ppu) {
    ppu.WriteIO(0x2100, 0x0f);
    ppu.WriteIO(0x2105, 7);
    Write16(ppu, 0x211b, 0x0100);
    Write16(ppu, 0x211e, 0x0100);
}

void Mode7DirectColorAndExtbgMosaic() {
    for (bool mosaic : {false, true}) {
        auto ppu = std::make_unique<Ppu>();
        SetupMode7(*ppu);
        ppu->WriteIO(0x212c, 1);
        ppu->WriteIO(0x2130, 1);
        if (mosaic) ppu->WriteIO(0x2106, 0x31);
        constexpr uint8_t raw = 0xe7;
        ppu->CgramData()[raw] = 0x001f;
        for (int i = 0; i < 64; ++i) ppu->VramData()[i] = static_cast<uint16_t>(raw << 8);
        RenderLines(*ppu, 1);
        Expect(Pixel(*ppu, 0, 0) == Mode7DirectColor(raw),
               mosaic ? "Mode 7 mosaic BG1 uses direct color" : "Mode 7 BG1 uses direct color");
    }

    {
        auto ppu = std::make_unique<Ppu>();
        SetupMode7(*ppu);
        ppu->WriteIO(0x2133, 0x40);
        ppu->WriteIO(0x2130, 1);
        ppu->WriteIO(0x212c, 2);
        ppu->CgramData()[1] = 0x03e0;
        for (int i = 0; i < 64; ++i) ppu->VramData()[i] = 0x8100;
        RenderLines(*ppu, 1);
        Expect(Pixel(*ppu, 0, 0) == 0x03e0,
               "EXTBG BG2 remains palette indexed when BG1 direct color is enabled");
    }

    {
        auto ppu = std::make_unique<Ppu>();
        SetupMode7(*ppu);
        ppu->WriteIO(0x2133, 0x40);
        ppu->WriteIO(0x212c, 2);
        ppu->WriteIO(0x2106, 0x31); // BG1 alone controls Mode 7 vertical mosaic.
        for (int color = 1; color <= 8; ++color) ppu->CgramData()[color] = static_cast<uint16_t>(color);
        for (int row = 0; row < 8; ++row) {
            for (int x = 0; x < 8; ++x) {
                ppu->VramData()[row * 8 + x] = static_cast<uint16_t>((0x81 + row) << 8);
            }
        }
        RenderLines(*ppu, 8);
        for (int y = 0; y < 4; ++y)
            Expect(Pixel(*ppu, 0, y) == 2, "BG1 mosaic holds EXTBG BG2 on the same source row vertically");
        Expect(Pixel(*ppu, 0, 4) == 6, "EXTBG BG2 advances when the BG1 vertical mosaic block ends");
    }
}

void SetupHiresPair(Ppu& ppu, bool mode5) {
    ppu.WriteIO(0x2100, 0x0f);
    ppu.WriteIO(0x2105, mode5 ? 5 : 0);
    if (!mode5) ppu.WriteIO(0x2133, 0x08);
    ppu.WriteIO(0x2107, 0x00);
    ppu.WriteIO(0x2108, 0x04);
    ppu.WriteIO(0x210b, 0x21);
    ppu.WriteIO(0x212c, 0x01);
    ppu.WriteIO(0x212d, 0x02);
    for (int x = 0; x < 32; ++x) {
        ppu.VramData()[x] = 0;
        ppu.VramData()[0x0400 + x] = mode5 ? 0x0400 : 0;
    }
    for (int row = 0; row < 8; ++row) {
        ppu.VramData()[0x1000 + row] = 0x00ff;
        ppu.VramData()[0x1008 + row] = 0;
        if (mode5) {
            ppu.VramData()[0x1010 + row] = 0x00ff;
            ppu.VramData()[0x1018 + row] = 0;
        }
        ppu.VramData()[0x2000 + row] = 0x00ff;
        if (mode5) ppu.VramData()[0x2008 + row] = 0x00ff;
    }
    ppu.CgramData()[1] = 5;
    ppu.CgramData()[mode5 ? 5 : 33] = static_cast<uint16_t>(5 << 5);
}

void HiresAdjacentMathAndWindows() {
    for (uint8_t mode : {uint8_t(0), uint8_t(5), uint8_t(6)}) {
        for (bool subtract : {false, true}) {
            for (bool halve : {false, true}) {
                auto ppu = std::make_unique<Ppu>();
                SetupHiresPair(*ppu, mode != 0);
                if (mode == 6) ppu->WriteIO(0x2105, 6);
                ppu->CgramData()[0] = 4 << 5;
                ppu->CgramData()[1] = 10;
                ppu->WriteIO(0x212d, 0); // No sub-screen layer at this pixel.
                ppu->WriteIO(0x2130, 2);
                ppu->WriteIO(0x2131, uint8_t(1 | (subtract ? 0x80 : 0) | (halve ? 0x40 : 0)));
                ppu->WriteIO(0x2132, 0x22); // Fixed red = 2, distinct from the green backdrop.
                RenderLines(*ppu, 1);
                Expect(Pixel(*ppu, 3, 0) == (subtract ? 8 : 12),
                       "Hires main math uses fixed color without halving when no sub-screen layer exists");
                Expect(Pixel(*ppu, 2, 0) == uint16_t((4 << 5) | (subtract ? 0 : 2)),
                       "Hires sub display retains its palette backdrop while using fixed-color math");
            }
        }
    }
    for (bool mode5 : {false, true}) {
        {
            auto ppu = std::make_unique<Ppu>();
            SetupHiresPair(*ppu, mode5);
            ppu->WriteIO(0x2130, 0x02);
            ppu->WriteIO(0x2131, 0x01);
            RenderLines(*ppu, 1);
            const uint16_t combined = static_cast<uint16_t>((5 << 5) | 5);
            Expect(Pixel(*ppu, 2, 0) == combined,
                   mode5 ? "Hires Sub(x+1) uses Main(x) color math" :
                           "Pseudo-hires Sub(x+1) uses Main(x) color math");
        }
        {
            auto ppu = std::make_unique<Ppu>();
            SetupHiresPair(*ppu, mode5);
            ppu->WriteIO(0x2126, 0);
            ppu->WriteIO(0x2127, 0);
            ppu->WriteIO(0x2125, 0x20);
            ppu->WriteIO(0x2130, 0x80);
            ppu->WriteIO(0x2131, 0x00);
            RenderLines(*ppu, 1);
            Expect(Pixel(*ppu, 2, 0) == 0,
                   mode5 ? "Hires Sub(x+1) inherits Main(x) color-window clipping" :
                           "Pseudo-hires Sub(x+1) inherits Main(x) color-window clipping");
        }
        {
            auto ppu = std::make_unique<Ppu>();
            SetupHiresPair(*ppu, mode5);
            ppu->WriteIO(0x2130, 0x02);
            ppu->WriteIO(0x2131, 0x01);
            ppu->WriteIO(0x2126, 0);
            ppu->WriteIO(0x2127, 255);
            ppu->WriteIO(0x212e, 1);
            ppu->FrameBegin();
            ppu->SetCurrentLine(1);
            ppu->ScanlineBegin(1);
            ppu->SetCurrentHClock(RasterClock(128));
            Write16(*ppu, 0x210d, 8);
            ppu->WriteIO(0x2123, 0x02);
            ppu->VBlankBegin();

            const uint16_t paired = static_cast<uint16_t>((5 << 5) | 5);
            Expect(Pixel(*ppu, 256, 0) == paired,
                   mode5 ? "Hires span boundary pairs Sub(x) with the preceding rendered Main(x-1)" :
                           "Pseudo-hires span boundary pairs Sub(x) with the preceding rendered Main(x-1)");
            Expect(Pixel(*ppu, 258, 0) == static_cast<uint16_t>(5 << 5),
                   mode5 ? "Hires pixels after the boundary use the new scroll/window state" :
                           "Pseudo-hires pixels after the boundary use the new scroll/window state");
        }
    }
}

void ObjectOverflowAndStat77() {
    {
        Ppu ppu;
        ppu.WriteIO(0x2100, 0x8f); // Forced blank; OBJ is absent from TM/TS.
        for (int i = 0; i < 128; ++i) ppu.OamData()[4 * i + 1] = 240;
        for (int i = 0; i < 33; ++i) {
            ppu.OamData()[4 * i + 0] = static_cast<uint8_t>((i * 7) & 0xff);
            ppu.OamData()[4 * i + 1] = 0;
        }
        ppu.FrameBegin();
        ppu.SetCurrentLine(1);
        Expect((ppu.ReadIO(0x213e, 0) & 0x40) != 0,
               "Range-over updates live even with forced blank and OBJ display disabled");
    }
    {
        Ppu ppu;
        ppu.WriteIO(0x2100, 0x8f);
        ppu.WriteIO(0x2101, static_cast<uint8_t>(3 << 5)); // 16x16 small objects, two tiles each.
        for (int i = 0; i < 128; ++i) ppu.OamData()[4 * i + 1] = 240;
        for (int i = 0; i < 18; ++i) {
            ppu.OamData()[4 * i + 0] = static_cast<uint8_t>((i * 13) & 0xff);
            ppu.OamData()[4 * i + 1] = 0;
        }
        ppu.FrameBegin();
        ppu.SetCurrentLine(1);
        Expect((ppu.ReadIO(0x213e, 0) & 0x80) != 0,
               "Time-over updates live after the 34th fetched OBJ tile");
    }
    {
        Ppu ppu;
        ppu.GetLatch().ppu1.mdr = 0x10;
        const uint8_t status = ppu.ReadIO(0x213e, 0);
        Expect((status & 0x10) != 0 && (status & 0x0f) == 1,
               "STAT77 preserves PPU1 open-bus bit 4 while reporting its version");
    }
}
} // namespace

int main() {
    BackgroundRows();
    SpriteFloorAlignment();
    Mode7Coordinates();
    Mode7VerticalMosaic();
    HighResolutionOutput();
    MixedWidthsAndFields();
    RasterMemoryChanges();
    RasterRegisterChanges();
    Mode7DirectColorAndExtbgMosaic();
    HiresAdjacentMathAndWindows();
    ObjectOverflowAndStat77();
    std::printf("PPU regression failures: %d\n", failures);
    return failures ? 1 : 0;
}
