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

void SetTilePixel(Ppu& ppu, uint16_t base, int tile, int depth,
                  int x, int y, uint8_t color) {
    for (int plane = 0; plane < depth; ++plane) {
        auto& word = ppu.VramData()[base + tile * depth * 4 + (plane / 2) * 8 + y];
        const uint16_t mask = static_cast<uint16_t>(1u << (7 - x + (plane & 1) * 8));
        word = static_cast<uint16_t>((word & ~mask) | ((color & (1u << plane)) ? mask : 0));
    }
}

int Tilemap64Address(int x, int y) {
    return (y & 31) * 32 + (x & 31) + ((x & 32) ? 1024 : 0) + ((y & 32) ? 2048 : 0);
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

void HiresMosaicSampling() {
    constexpr uint8_t pattern[4] = {1, 2, 0, 3};
    for (uint8_t mode : {5, 6}) {
        for (int layer : {0, 1}) {
            if (mode == 6 && layer == 1) continue;
            for (int size : {0, 1, 2, 3, 16}) {
                for (bool flip : {false, true}) {
                    for (uint16_t scroll : {0, 3}) {
                        auto ppu = std::make_unique<Ppu>();
                        const uint8_t enable = static_cast<uint8_t>(1u << layer);
                        ppu->WriteIO(0x2100, 0x0f);
                        ppu->WriteIO(0x2105, mode);
                        ppu->WriteIO(0x210b, layer == 0 ? 0x01 : 0x10);
                        ppu->WriteIO(0x212c, enable);
                        ppu->WriteIO(0x212d, enable);
                        Write16(*ppu, static_cast<uint16_t>(0x210d + layer * 2), scroll);
                        ppu->WriteIO(0x2106, size ? static_cast<uint8_t>(((size - 1) << 4) | enable) : 0);
                        // Mask only the main screen; mosaic sampling continues behind the window.
                        ppu->WriteIO(0x2123, layer == 0 ? 0x02 : 0x20);
                        ppu->WriteIO(0x2126, 5);
                        ppu->WriteIO(0x2127, 9);
                        ppu->WriteIO(0x212e, enable);
                        ppu->CgramData()[1] = 0x001f;
                        ppu->CgramData()[2] = 0x03e0;
                        ppu->CgramData()[3] = 0x7c00;
                        for (int tile = 0; tile < 32; ++tile)
                            ppu->VramData()[tile] = flip ? 0x4000 : 0;
                        for (int y = 0; y < 8; ++y) {
                            for (int x = 0; x < 16; ++x)
                                SetTilePixel(*ppu, 0x1000, x / 8, layer == 0 ? 4 : 2,
                                             x & 7, y, pattern[x & 3]);
                        }
                        RenderLines(*ppu, 1);
                        bool matches = true;
                        for (int x = 0; x < 512; ++x) {
                            const int dot = x / 2;
                            // A block is measured in 256-dot coordinates. Even size one
                            // sends its even source sample to both main and sub screens.
                            int source = size ? (dot / size) * size * 2 : x;
                            source = (source + scroll * 2) & 15;
                            if (flip) source ^= 15;
                            uint16_t expected = ppu->CgramData()[pattern[source & 3]];
                            if ((x & 1) && dot >= 5 && dot <= 9) expected = 0;
                            matches &= Pixel(*ppu, x, 0) == expected;
                        }
                        Expect(matches, "Hires mosaic samples even dots at logical block widths before per-screen windows");
                    }
                }
            }
        }
    }
}

void Mode6TileOffsets() {
    for (bool tall : {false, true}) {
        for (uint16_t scroll : {0, 3, 7}) {
            for (uint16_t lookupScroll : {0, 248}) {
                for (uint8_t offsets : {0, 1, 2, 3}) {
                    auto ppu = std::make_unique<Ppu>();
                    ppu->WriteIO(0x2100, 0x0f);
                    ppu->WriteIO(0x2105, static_cast<uint8_t>(6 | (tall ? 0x10 : 0)));
                    ppu->WriteIO(0x2107, 0x03); // BG1: 64x64 map at word $0000.
                    ppu->WriteIO(0x2109, 0x23); // BG3: 64x64 offset map at word $2000.
                    ppu->WriteIO(0x210b, 0x04);
                    ppu->WriteIO(0x212c, 1);
                    ppu->WriteIO(0x212d, 1);
                    Write16(*ppu, 0x210d, scroll);
                    Write16(*ppu, 0x210e, 5);
                    Write16(*ppu, 0x2111, lookupScroll);
                    for (int y = 0; y < 64; ++y) {
                        for (int x = 0; x < 64; ++x)
                            ppu->VramData()[Tilemap64Address(x, y)] = static_cast<uint16_t>(2 * ((x + y * 3) & 7));
                    }
                    for (int x = 0; x < 64; ++x) {
                        // Disabled entries carry BG2's enable bit to check BG1 isolation.
                        ppu->VramData()[0x2000 + Tilemap64Address(x, 0)] =
                            static_cast<uint16_t>(((offsets & 1) ? 0x2000 : 0x4000) | ((16 + x * 24) & 1023));
                        ppu->VramData()[0x2000 + Tilemap64Address(x, 1)] =
                            static_cast<uint16_t>(((offsets & 2) ? 0x2000 : 0x4000) | ((11 + x * 5) & 1023));
                    }
                    for (int color = 1; color < 16; ++color)
                        ppu->CgramData()[color] = static_cast<uint16_t>(color * 0x421);
                    for (int tile = 0; tile < 32; ++tile) {
                        for (int y = 0; y < 8; ++y) {
                            for (int x = 0; x < 8; ++x)
                                SetTilePixel(*ppu, 0x4000, tile, 4, x, y,
                                             static_cast<uint8_t>((tile + y * 2) % 15 + 1));
                        }
                    }
                    RenderLines(*ppu, 3);
                    bool matches = true;
                    for (int y = 0; y < 3; ++y) {
                        for (int x = 0; x < 512; ++x) {
                            int sourceX = x + scroll * 2;
                            int sourceY = y + 1 + 5;
                            const int offsetDot = x / 2 + (scroll & 7);
                            // The first eight logical dots, shortened by fine scroll,
                            // have no offset entry. Each later entry covers 16 source pixels.
                            if (offsetDot >= 8) {
                                const int column = (lookupScroll / 8 + (offsetDot - 8) / 8) & 63;
                                if (offsets & 1)
                                    sourceX = x + (scroll & 7) * 2 + ((16 + column * 24) & 1023) * 2;
                                if (offsets & 2) sourceY = y + 1 + ((11 + column * 5) & 1023);
                            }
                            const int mapX = (sourceX / 16) & 63;
                            const int mapY = (sourceY / (tall ? 16 : 8)) & 63;
                            int tile = 2 * ((mapX + mapY * 3) & 7) + ((sourceX & 8) ? 1 : 0);
                            if (tall && (sourceY & 8)) tile += 16;
                            const uint16_t expected = static_cast<uint16_t>(((tile + (sourceY & 7) * 2) % 15 + 1) * 0x421);
                            matches &= Pixel(*ppu, x, y) == expected;
                        }
                    }
                    Expect(matches, "Mode 6 offsets preserve logical columns, fine scroll, BG3 scrolling and independent H/V enables");
                }
            }
        }
    }
}

void RasterSpanSampling() {
    constexpr int boundaries[] = {1, 5, 13, 47, 95, 108, 127, 128, 171, 223, 250};
    for (uint8_t mode = 0; mode < 8; ++mode) {
        for (uint8_t mosaic : {0, 3, 5}) {
            for (bool interlace : {false, true}) {
                auto reference = std::make_unique<Ppu>();
                auto raster = std::make_unique<Ppu>();
                for (Ppu* ppu : {reference.get(), raster.get()}) {
                    ppu->WriteIO(0x2100, 0x0f);
                    ppu->WriteIO(0x2105, mode);
                    ppu->WriteIO(0x2107, 1);
                    ppu->WriteIO(0x2109, 0x20);
                    ppu->WriteIO(0x210b, 1);
                    ppu->WriteIO(0x2101, 0x63); // 16x16 objects at word $6000.
                    ppu->WriteIO(0x2133, static_cast<uint8_t>((interlace ? 9 : 0) | (mode == 7 ? 0x40 : 0)));
                    ppu->WriteIO(0x212c, 0x13);
                    ppu->WriteIO(0x212d, 0x13);
                    ppu->WriteIO(0x2130, 0x02);
                    ppu->WriteIO(0x2131, 0x73);
                    ppu->WriteIO(0x2132, 0xe7);
                    ppu->WriteIO(0x2106, mosaic ? static_cast<uint8_t>(((mosaic - 1) << 4) | 3) : 0);
                    Write16(*ppu, 0x210d, 3);
                    Write16(*ppu, 0x210e, 5);
                    ppu->WriteIO(0x2123, 0x02);
                    ppu->WriteIO(0x2125, 0x2a);
                    ppu->WriteIO(0x2126, 9);
                    ppu->WriteIO(0x2127, 61);
                    ppu->WriteIO(0x2128, 159);
                    ppu->WriteIO(0x2129, 207);
                    ppu->WriteIO(0x212e, 0x11);
                    ppu->WriteIO(0x212f, 0x10);
                    for (int color = 0; color < 256; ++color)
                        ppu->CgramData()[color] = static_cast<uint16_t>((color * 137) & 0x7fff);
                    if (mode == 7) {
                        Write16(*ppu, 0x211b, 0x0140);
                        Write16(*ppu, 0x211c, 0x0080);
                        Write16(*ppu, 0x211d, 0xffc0);
                        Write16(*ppu, 0x211e, 0x0100);
                        for (int word = 0; word < 0x4000; ++word)
                            ppu->VramData()[word] = static_cast<uint16_t>(((word / 7) & 7) | (((word * 13 + word / 8) & 255) << 8));
                    } else {
                        const int depth = 2 << static_cast<int>(ppu->GetIO().bg1.tileMode);
                        for (int tile = 0; tile < 2048; ++tile)
                            ppu->VramData()[tile] = static_cast<uint16_t>((tile & 6) | ((tile & 3) << 13));
                        for (int tile = 0; tile < 8; ++tile) {
                            for (int y = 0; y < 8; ++y) {
                                for (int x = 0; x < 8; ++x)
                                    SetTilePixel(*ppu, 0x1000, tile, depth, x, y,
                                                 static_cast<uint8_t>((x + y + tile) & ((1 << depth) - 1)));
                            }
                        }
                    }
                    for (int i = 0; i < 128; ++i) ppu->OamData()[i * 4 + 1] = 240;
                    for (int i = 0; i < 20; ++i) {
                        ppu->OamData()[i * 4] = static_cast<uint8_t>(i * 11);
                        ppu->OamData()[i * 4 + 1] = 0;
                        ppu->OamData()[i * 4 + 2] = static_cast<uint8_t>((i & 7) * 2);
                        ppu->OamData()[i * 4 + 3] = static_cast<uint8_t>(((i & 3) << 4) | ((i & 7) << 1));
                    }
                    for (int tile = 0; tile < 32; ++tile) {
                        for (int y = 0; y < 8; ++y) {
                            for (int x = 0; x < 8; ++x)
                                SetTilePixel(*ppu, 0x6000, tile, 4, x, y, static_cast<uint8_t>((x + tile) & 15));
                        }
                    }
                }
                RenderLines(*reference, 3);
                raster->FrameBegin();
                for (uint16_t line = 1; line <= 3; ++line) {
                    raster->SetCurrentLine(line);
                    raster->SetCurrentHClock(0);
                    raster->WriteIO(0x2100, 15);
                    for (int i = 0; i < 11; ++i) {
                        if (i == 5) raster->ScanlineBegin(line);
                        raster->SetCurrentHClock(RasterClock(boundaries[i]));
                        raster->WriteIO(0x2100, static_cast<uint8_t>((i * 7 + line) % 15 + 1));
                    }
                }
                raster->VBlankBegin();
                bool matches = true;
                for (int line = 1; line <= 3; ++line) {
                    const int row = (line - 1) * (interlace ? 2 : 1) + (interlace ? 1 : 0);
                    for (int x = 0; x < reference->FrameWidth(); ++x) {
                        const int dot = x / (reference->FrameWidth() == 512 ? 2 : 1);
                        int brightness = 15;
                        for (int i = 0; i < 11 && dot >= boundaries[i]; ++i)
                            brightness = (i * 7 + line) % 15 + 1;
                        const uint16_t original = Pixel(*reference, x, row);
                        uint16_t expected = 0;
                        for (int shift : {0, 5, 10})
                            expected |= static_cast<uint16_t>(((((original >> shift) & 31) * brightness + 7) / 15) << shift);
                        matches &= Pixel(*raster, x, row) == expected;
                    }
                }
                Expect(matches, "Raster brightness spans preserve tile, mosaic, Mode 7 and OBJ samples with interlace and both screens");
            }
        }
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

void VramSnapshotLifetime() {
    auto ppu = std::make_unique<Ppu>();
    ppu->WriteIO(0x210b, 1);
    ppu->WriteIO(0x212c, 1);
    ppu->WriteIO(0x2133, 4);
    ppu->WriteIO(0x2115, 0x80);
    ppu->CgramData()[1] = 0x001f;
    ppu->CgramData()[2] = 0x03e0;

    auto writeTile = [&](uint16_t row) {
        ppu->WriteIO(0x2116, 0x00);
        ppu->WriteIO(0x2117, 0x10);
        for (int y = 0; y < 8; ++y) {
            ppu->WriteIO(0x2118, static_cast<uint8_t>(row));
            ppu->WriteIO(0x2119, static_cast<uint8_t>(row >> 8));
        }
    };

    // Exercise both unchanged runs and a distinct memory revision on every
    // overscan line, then reuse that storage over several further frames.
    for (unsigned block : {239u, 13u, 1u, 239u}) {
        ppu->FrameBegin();
        for (uint16_t line = 1; line < 240; ++line) {
            ppu->SetCurrentLine(line);
            ppu->SetCurrentHClock(0);
            if ((line - 1) % block == 0) {
                ppu->WriteIO(0x2100, 0x8f);
                writeTile(((line - 1) / block) % 2 ? 0xff00 : 0x00ff);
                ppu->WriteIO(0x2100, 0x0f);
            }
            ppu->ScanlineBegin(line);
            // Active-display writes must not alter any line's tile memory.
            writeTile(0xffff);
        }
        ppu->VBlankBegin();
        for (int y = 0; y < 239; ++y) {
            const uint16_t expected = (y / block) % 2 ? 0x03e0 : 0x001f;
            Expect(Pixel(*ppu, 0, y) == expected && Pixel(*ppu, 255, y) == expected,
                   "VRAM port writes preserve every captured row across frames");
        }
    }

    // A debugger can retain a writable pointer, including across a reset.
    auto* memory = ppu->VramData();
    for (int pass = 0; pass < 2; ++pass) {
        ppu->Reset();
        ppu->WriteIO(0x2100, 0x0f);
        ppu->WriteIO(0x210b, 1);
        ppu->WriteIO(0x212c, 1);
        ppu->CgramData()[1] = 0x001f;
        ppu->CgramData()[2] = 0x03e0;
        ppu->FrameBegin();
        for (uint16_t line = 1; line <= 16; ++line) {
            for (int row = 0; row < 8; ++row)
                memory[0x1000 + row] = line <= 8 ? 0x00ff : 0xff00;
            ppu->ScanlineBegin(line);
        }
        ppu->VBlankBegin();
        for (int y = 0; y < 16; ++y)
            Expect(Pixel(*ppu, 0, y) == (y < 8 ? 0x001f : 0x03e0),
                   "Retained mutable VRAM pointers remain coherent after reset");
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
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        ppu->WriteIO(0x2123, 0x02);
        ppu->WriteIO(0x2126, 0);
        ppu->WriteIO(0x2127, 255);
        ppu->WriteIO(0x212e, 1);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x2127, 127);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 64, 0) == 0, "Raster window position preserves the earlier masked span");
        Expect(Pixel(*ppu, 192, 0) == 0x001f, "Raster window position applies only after its timestamp");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        ppu->WriteIO(0x2123, 0x0a);
        ppu->WriteIO(0x2126, 0);
        ppu->WriteIO(0x2127, 127);
        ppu->WriteIO(0x2128, 128);
        ppu->WriteIO(0x2129, 255);
        ppu->WriteIO(0x212e, 1);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x212a, 0x01);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 64, 0) == 0, "Raster window logic preserves the earlier OR-masked span");
        Expect(Pixel(*ppu, 192, 0) == 0x001f, "Raster window logic applies the new AND rule after its timestamp");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        ppu->WriteIO(0x212c, 0);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->SetCurrentHClock(RasterClock(64));
        ppu->WriteIO(0x212c, 1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(192));
        ppu->WriteIO(0x212c, 0);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 32, 0) == 0, "Raster main-screen selection reconstructs the pre-snapshot disabled span");
        Expect(Pixel(*ppu, 100, 0) == 0x001f, "Raster main-screen selection keeps the state sampled at H=512");
        Expect(Pixel(*ppu, 220, 0) == 0, "Raster main-screen selection applies a later write to the final span");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        ppu->WriteIO(0x2123, 0x02);
        ppu->WriteIO(0x2126, 0);
        ppu->WriteIO(0x2127, 255);
        ppu->WriteIO(0x212e, 0);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x212e, 1);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 64, 0) == 0x001f, "Raster main-window enable preserves the earlier visible span");
        Expect(Pixel(*ppu, 192, 0) == 0, "Raster main-window enable masks the layer after its timestamp");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x2130, 0xc0);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 64, 0) == 0x001f, "Raster color-window control preserves the earlier main pixel");
        Expect(Pixel(*ppu, 192, 0) == 0, "Raster color-window control clips main pixels after its timestamp");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        ppu->WriteIO(0x2125, 0xa0);
        ppu->WriteIO(0x2126, 0);
        ppu->WriteIO(0x2127, 127);
        ppu->WriteIO(0x2128, 128);
        ppu->WriteIO(0x2129, 255);
        ppu->WriteIO(0x2130, 0x40);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x212b, 0x04);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 64, 0) == 0x001f, "Raster color-window logic preserves the earlier OR span");
        Expect(Pixel(*ppu, 192, 0) == 0, "Raster color-window logic applies the later AND span");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        ppu->CgramData()[1] = 5;
        ppu->WriteIO(0x2132, 0x44);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x2131, 0x01);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 64, 0) == 5, "Raster color-math designation preserves the earlier unblended span");
        Expect(Pixel(*ppu, 192, 0) == static_cast<uint16_t>(5 | (4 << 5)),
               "Raster color-math designation blends only after its timestamp");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupBg1(*ppu);
        ppu->CgramData()[1] = 5;
        ppu->WriteIO(0x2131, 0x01);
        ppu->WriteIO(0x2132, 0x21);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x2132, 0x22);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 64, 0) == 6, "Raster fixed color preserves the earlier blend operand");
        Expect(Pixel(*ppu, 192, 0) == 7, "Raster fixed color changes only the later blend span");
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
        ppu->WriteIO(0x212c, 1);
        constexpr uint8_t raw = 0xe7;
        ppu->CgramData()[raw] = 0x001f;
        for (int i = 0; i < 64; ++i) ppu->VramData()[i] = static_cast<uint16_t>(raw << 8);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x2130, 0x01);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 64, 0) == 0x001f, "Raster direct-color control preserves the earlier indexed span");
        Expect(Pixel(*ppu, 192, 0) == Mode7DirectColor(raw),
               "Raster direct-color control changes Mode 7 pixels after its timestamp");
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
    {
        auto ppu = std::make_unique<Ppu>();
        SetupHiresPair(*ppu, false);
        ppu->WriteIO(0x212d, 0);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x212d, 2);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 128, 0) == 0, "Raster sub-screen selection preserves the earlier backdrop sub pixel");
        Expect(Pixel(*ppu, 384, 0) == static_cast<uint16_t>(5 << 5),
               "Raster sub-screen selection enables the later pseudo-hires sub pixel");
    }
    {
        auto ppu = std::make_unique<Ppu>();
        SetupHiresPair(*ppu, false);
        ppu->WriteIO(0x2123, 0x20);
        ppu->WriteIO(0x2126, 0);
        ppu->WriteIO(0x2127, 255);
        ppu->WriteIO(0x212f, 0);
        ppu->FrameBegin();
        ppu->SetCurrentLine(1);
        ppu->ScanlineBegin(1);
        ppu->SetCurrentHClock(RasterClock(128));
        ppu->WriteIO(0x212f, 2);
        ppu->VBlankBegin();
        Expect(Pixel(*ppu, 128, 0) == static_cast<uint16_t>(5 << 5),
               "Raster sub-window enable preserves the earlier pseudo-hires sub pixel");
        Expect(Pixel(*ppu, 384, 0) == 0,
               "Raster sub-window enable masks the later pseudo-hires sub pixel");
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
    HiresMosaicSampling();
    Mode6TileOffsets();
    RasterSpanSampling();
    MixedWidthsAndFields();
    RasterMemoryChanges();
    VramSnapshotLifetime();
    RasterRegisterChanges();
    Mode7DirectColorAndExtbgMosaic();
    HiresAdjacentMathAndWindows();
    ObjectOverflowAndStat77();
    std::printf("PPU regression failures: %d\n", failures);
    return failures ? 1 : 0;
}
