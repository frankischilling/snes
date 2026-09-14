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

void BackgroundRows() {
    // Snes9x gfx.cpp RenderLine: source Y = output row + BG VOFS + 1.
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
    // Snes9x tileimpl.h: signed 13-bit offsets; Y is Line+1, optionally flipped.
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
} // namespace

int main() {
    BackgroundRows();
    SpriteFloorAlignment();
    Mode7Coordinates();
    Mode7VerticalMosaic();
    std::printf("PPU regression failures: %d\n", failures);
    return failures ? 1 : 0;
}
