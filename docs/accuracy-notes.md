# Rendering and LoROM checks

| Behavior | Emulator behavior |
| --- | --- |
| Background Y | Output row Y samples background row Y + VOFS + 1. Cached scanlines already include the +1. |
| Sprite Y | Raw OAM Y is the first output row. Parsed sprite Y includes the hardware scanline offset, with 8-bit wrapping. |
| Mode 7 coordinates | Use signed 13-bit offsets and centers. Apply vertical flip to the hardware scanline. |
| Mode 7 mosaic | Repeat the first source row of each vertical mosaic block before applying vertical flip. |
| LoROM mirrors | Both halves of banks $40-$7D and $C0-$FF map 32KB ROM pages. SRAM and WRAM take precedence in their regions. |

The background renderer previously added the visible-line offset twice. This moved the floor one pixel upward relative to Mario. Mode 7 had the same extra offset, ignored vertical flip and vertical mosaic, and used bit 13 instead of bit 12 to extend the sign of 13-bit values. LoROM decoding also omitted the lower halves of full-ROM banks.

`snes_ppu_regression_tests` checks background rows in modes 0 through 4, sprite edges and wrapping, floor alignment, and Mode 7 scrolling, flips and mosaic. Its checks run in every build configuration. The tests reported 346 failed pixel checks before the rendering fixes and zero afterward.

The existing core suite now keeps assertions enabled in Release builds. Rendering fixtures that expect tile row 0 at output row 0 explicitly set VOFS to -1. Coordinate regression tests retain zero and nonzero offsets to verify the hardware row convention. Core tests also cover LoROM mirrors, SRAM writes and WRAM precedence.

Run the suites from the repository directory:

```powershell
cmake --build --preset build-release
ctest --test-dir out/build/release --output-on-failure
```

These checks cover the rendering and mapping paths listed above. They do not establish compatibility with every game.
