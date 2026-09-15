# Original SNES hardware support

The core now executes SA-1 and Super FX programs, processes the supported DSP-3/4 and Cx4 commands, and exposes SPC7110 decompression and clock hardware. BS-X uses its own memory controller and packet receivers. These devices connect to the console bus, cartridge storage, and interrupt sampler; detection alone is not considered an implementation.

Existing LoROM, HiROM, extended and special boards, DSP-1/2, OBC1, S-RTC, S-DD1, ST010, Sufami Turbo, controllers, and PPU rendering paths remain in use. This work concerns original hardware compatibility. It does not add frontend features or change the renderer into a per-dot model.

## Cartridge implementations

| Device | Implemented behavior | Important limits |
| --- | --- | --- |
| SA-1 | Reuses the 65816 instruction core; reset/vector registers, mailbox IRQ/NMI, timers, 2 KiB IRAM and protection, shared BW-RAM and bitmap modes, ROM banking, arithmetic, variable-length bit reads, progressive DMA, both character-conversion modes, and master-clock advancement. | CPU advancement finishes whole instructions. ROM cache and bus contention penalties, exact character-conversion latency, DMA priority arbitration, and short/interlaced scanline coupling need further work. |
| Super FX/FX2 | GSU instruction and ALT encodings, prefetched pipeline and delay slots, register file, 512-byte cache, plotting/pixel rows, ROM/RAM buffers, screen modes, bus ownership, clock modes, IRQ, 4 MiB GSU2 images, and the extended CPU ROM/RAM board layout. | Cache fills and pixel writes publish at transaction completion rather than each constituent byte. Commercial GSU game and hardware traces remain unverified. |
| DSP-3 | Byte/word status protocol, coordinate and hex-grid operations, planar conversion, diagnostic coefficient ROM, resumable prefix/LZ token decoding, and terrain/path result streams. | Command-level model. Several diagnostic commands have only provisional observed behavior; the internal DSP program and clock timing are not modeled. |
| DSP-4 | Command/output protocol, signed multiplication, road projection variants, incremental lighting, polygon windows, vehicle/terrain objects, sprite row budgets, and OAM attributes. | Command-level model. Reciprocal rounding and edge inputs need hardware measurements; there is no internal DSP instruction or busy-time model. |
| Cx4 | Work RAM, ROM transfer, arithmetic/vector commands, sprite/OAM conversion, affine and dissolve graphics, transformed vertices, wireframes, waves, window edges, and diagnostic responses. | Commands complete immediately. Internal processor execution and arbitration are not modeled. Malformed graphics dimensions are bounded to physical RAM. |
| SPC7110/RTC | Three decompression modes, banked data access, pointer/adjust/increment modes, multiply/divide, SRAM write gating, cartridge mapping, and an injectable persistent calendar. | Decoder/MMIO behavior is modeled at command level. Chip-internal decompression latency and bus contention are not established. |
| BS-X | BIOS overlay, latched MMC bits, 512 KiB PSRAM, 32 KiB battery SRAM, flash mapping and commands, ready IRQ, radio registers, calendar packets, and two independent recorded-data receivers. | Flash completion is synchronous. Block-lock recovery, physical /WP/CIC behavior, undocumented hidden-bit remapping, live reception, and SoundLink audio remain unimplemented. |
| ST011 | Command packets, the 128-byte board download, ready status, known result-byte writes, and separate work/battery RAM. | No internal processor or shogi calculations. Commands with unknown computations retain their limited observed protocol behavior. |
| ST018 | Three-byte startup commands `$000100` and `$00FF00`, parameter rounds, status, and `$81` replies. | No ARM processor, firmware, or game logic. Passing the startup handshake is not full chip emulation. |

DSP-3 uses banks `$20-$3F/$A0-$BF:$8000-$FFFF`; DSP-4 uses `$30-$3F/$B0-$BF` in the same offset range. `$8000-$BFFF` carries data and `$C000-$FFFF` reads status. Their mirrors share one command stream.

SA-1 slot boards with extended IDs `ZX3J` or `ZBPJ` accept a separate 512 KiB expansion ROM through the existing broadcast loader. MMC pages 4-7 select that ROM, an empty slot returns open bus, and the expanded console BW-RAM windows preserve WRAM at `$7E-$7F`. Expansion ROM is never exposed as writable flash storage.

The SA-1 bitmap window selects a physical 2 KiB BW-RAM page before unpacking two-bit or four-bit pixels. Division uses a signed quotient truncated toward zero and a nonnegative remainder, including the measured divide-by-zero register values. The bit reader discards address bit zero, reads only ROM, mirrors non-ROM 32 KiB blocks to bank-zero LoROM, and follows Super MMC projection. Writing `$225B` loads the address and clears `$2258`, so automatic advance must be enabled afterward. Regression vectors follow [SA-1 hardware measurements](https://sneslab.net/wiki/SA-1_Hardware_Behavior). Those division measurements cover `$2306-$2309`; the existing upper-byte behavior at `$230A` remains unverified.

Super FX CPU RAM windows include `$72-$73` when the board has 256 KiB of RAM. Larger RAM declarations extend the CPU map through `$7D`; `$7E-$7F` remain console WRAM. The extended ROM layout, selected by a ROM-size header value of 14 or greater, places successive ROM regions at `$00-$3F:$8000-$FFFF`, `$80-$BF:$8000-$FFFF`, `$C0-$FF`, and `$40-$6F`, covering up to 11 MiB. These CPU extensions do not widen the GSU's own 2 MiB ROM view or one-bit RAM bank register. Missing ROM regions return open bus, and ROM writes cannot alias save RAM.

The GSU RAMB instruction selects bank `$70` or `$71`, and `$303C` returns zero in its reserved upper bits, as specified in [Nintendo's development manual, Book II, section 4.7](https://archive.org/stream/SNESDevManual/book2_djvu.txt). Tests exercise all 256 low-byte values supplied to RAMB and verify that banked loads/stores cannot reach CPU expansion RAM. Invalid internal GSU banks are not treated as extra ROM aliases.

SPC7110 calendar state uses a separate 24-byte `.rtc` representation: sixteen nibble-valued clock registers followed by an eight-byte little-endian timestamp. This differs from S-RTC's format; the loaded cartridge selects the decoder. Tests cover leap-day advancement, stopped clocks, rollback, and rejection of invalid state.

## CPU, DMA, and audio timing

CPU reads, writes, and internal cycles advance hardware individually. Registers can observe an event reached during an earlier access in the same instruction. IRQ entry uses the interrupt-mask value sampled before the final cycle, including CLI, SEI, REP, SEP, and PLP. Masked IRQs still release WAI.

Multiply and divide run for eight and sixteen CPU cycles. The result registers expose intermediate values, reads sample before the next arithmetic step, and trigger writes cannot restart an active operation, including its final busy cycle. Ordinary DMA pauses arithmetic; refresh contributes five steps. PLB uses a full-width stack address increment: in emulation mode, a stack pointer of `$01FF` reads `$0200` before returning to page one.

DMA requests allow a CPU cycle before bus takeover. Transfers align to the DMA divider, charge startup, place source and destination accesses on separate phases, and resume the CPU at its cycle boundary. HDMA table and data accesses advance hardware as they occur and can interrupt general DMA. Refresh positions follow the divider instead of staying fixed on every scanline. The existing eight-channel modes, source-bank wrapping, and same-channel HDMA cancellation remain covered.

The S-DSP advances through 32 phases per output sample. Register latches, BRR/directory reads, envelopes, noise, pitch modulation, voice mixing, echo RAM, and output timing have separate phase behavior. Each elapsed 1.024 MHz SMP clock advances it once, including clocks spent waiting for a memory access.

The SPC700 can pause inside an instruction or a stretched memory access. `RunUntil` stops at its requested clock count. Communication reads hold their sampled input until the read completes; pending stores do not reach the console early. CONTROL clears and timer read-and-clear operations occur on their access boundary. `Step` still completes one instruction for direct processor users. Reset cancels pending work, and coroutine storage belongs to the processor so sequential thread handoff does not invalidate a suspended instruction.

The SMP TEST register selects internal and external wait states separately. I/O, idle cycles, and the enabled IPL overlay use the internal setting; other RAM accesses use the external setting. The implemented clock model uses 1, 2, 5, or 10 SMP clocks and advances the timer divider by 2, 4, 8, or 16 ticks. A TEST write finishes under the old setting. The model samples port reads midway through a stretched access, rounded to the next SMP clock, and ordinary RAM reads at its end.

The frame collector drains full DSP blocks during long DMA operations. Transfers spanning several fields retain all generated audio instead of stopping at the ordinary 2,048-sample buffer limit. Video presentation still returns the latest completed field after such an operation.

A refresh penalty is drained after the current clock interval, and SA-1 still advances at instruction boundaries. SA-1 cache/contention and DMA arbitration, finer coprocessor transactions, and analog audio behavior remain accuracy work. [Hardware measurements of SMP TEST](https://forums.nesdev.org/viewtopic.php?start=75&t=16140) show much longer internal waits and lockups on some revisions. Those differences, sub-clock port sampling, and read/write collisions are not modeled.

## Raster rendering

Visible-line writes to brightness/forced blank, BG scroll, mosaic, window selection/positions/logic, main and sub-screen enables, window masks, and color-math controls carry their horizontal timestamps into rendering. This includes direct-color selection and fixed-color changes. The renderer reconstructs the state before the H=512 snapshot and draws successive spans, preserving pixels to the left of each change. VRAM, CGRAM, and OAM still use per-line snapshots; this is not a per-dot memory-fetch model. Register-specific fetch delays, active CGRAM address timing, and midline mode/geometry changes remain outside the timestamped path. HBlank writes affect the next line.

Mode 7 BG1 supports direct color with and without mosaic. EXTBG BG2 continues to use CGRAM, and BG1's mosaic enable controls vertical sampling for both Mode 7 layers. Hires and pseudo-hires output pair each sub pixel with the preceding main pixel's color-math and window decisions, including at a raster-state boundary. An empty sub screen displays its palette backdrop, while main color math uses the fixed-color register without halving.

Sprite overflow is evaluated during the current frame, even when OBJ is absent from the main and sub screens or the display is blanked. The 32-object and 34-tile flags remain sticky until the next frame, and `$213E` preserves PPU1 open-bus bit 4. Deferred drawing no longer changes the global overflow flags.

## BS-X host interface

The ordinary loader recognizes a 1 MiB BS-X BIOS and supplies a blank 1 MiB pack. `LoadBroadcastCartridge` also accepts a separate 1 or 2 MiB pack. BIOS and broadcast files are not included.

```cpp
std::string error;
if (!emulator.LoadBroadcastCartridge(biosBytes, packBytes, &error)) {
    throw std::runtime_error(error);
}
if (!emulator.LoadBroadcastStream(0x0123, 0, recordedPayload)) {
    throw std::runtime_error("Invalid broadcast channel");
}
emulator.SetBroadcastTimeSource([] {
    return snes::core::BroadcastTime{2026, 9, 14, 2, 12, 34, 56};
});
```

Channel zero supplies the clock. Other installed channels expose 22-byte packets, including padding on a partial final packet. Receivers keep independent cursors and prefix flags. A missing channel has no packets; it does not synthesize a broadcast. The default cartridge clock uses host local time, while the standalone device and tests can inject a deterministic clock.

The memory controller follows the documented [MCC register and mapping behavior](https://wiki.superfamicom.org/bs-x-mmio), including D7-only registers, pending versus applied bits, BIOS mirrors, PSRAM snippets, and expansion holes. The implementation keeps these independently tested cases separate from the common-path differential comparison.

## Validation scope

All 34 configured CTest targets pass in Release and AddressSanitizer builds on Windows with Clang 21: 31 bundled tests and three optional DMA/IRQ, CPU, and SPC diagnostics. The Windows legacy test executable reserves an 8 MiB stack so sanitizer instrumentation can retain its large fixtures.

Run the complete test set with:

```powershell
cmake --build --preset build-release
ctest --test-dir out/build/release --output-on-failure
```

Committed tests use generated programs, command streams, known output vectors, and deterministic clocks. Native 65816 programs exercise cartridge ports through the real bus. Processor tests run SA-1 and GSU programs, check their IRQs, and verify that save loading retains shared storage. Bus tests place events within instructions and verify DMA timestamps. Audio tests change registers and shared RAM across individual DSP phases.

The CPU arithmetic tests check all 65,536 eight-bit products and 657,664 division cases, plus partial results, busy writes, reset, refresh, and DMA behavior. Final-cycle trigger tests cover six-, eight-, and twelve-master-clock accesses and a DMA transfer that holds the pending write across refresh. SMP tests cover all sixteen internal/external wait combinations, timer and DSP clocks, held port reads, delayed stores, IPL mapping, reset during a wait, and halted execution. PAL and NTSC console tests also verify stretched port-store deadlines.

The sound-port integration tests execute console loads and stores through `$2140-$2143` and their mirrors under both PAL and NTSC clock ratios. They check deadlines in both communication directions. In the earlier audit, these tests exposed an SMP store becoming visible too early and console accesses failing to reach Super FX RAM banks `$72-$73`.

The earlier audit compared SPC700 traces for 2,048 cases: all 256 opcodes, four deterministic RAM/register/flag seeds, and two branch paths. Ordered read/write/idle events, cycle counts, registers, halt state, and complete RAM hashes matched. The committed segmented-execution tests also cover reset during an instruction, CONTROL clears, timer reads, halt deadlines, DSP phase boundaries, and sequential cross-thread resume/destruction.

For external text diagnostics, `snes_rom_diagnostic` reads a 32-column ASCII tilemap directly from backing VRAM without touching PPU read latches. It prints stable result pages. `--expect` requires exact result lines and returns a failing exit status when any are missing; execution alone is not a diagnostic pass. For example, with an existing [SnesTests checkout](https://github.com/SourMesen/SnesTests):

```powershell
cmake --preset release -DSNES_DIAGNOSTIC_ROM_DIR=C:/path/to/SnesTests
cmake --build --preset build-release
ctest --test-dir out/build/release -R snes_external_dma_irq --output-on-failure
```

The DMA/IRQ fixture checks all 19 source-defined results from checkout `71c50e3272cb43d9042b1630e4dbbe7ac2f0ca29`. Its two SEI sentinels are `$00FF`, as written by the ROM's 8-bit test routine. The opcode timing ROM produced 839 measurements across 54 result pages. Comparisons exposed differences in stack-relative indexed and PER timing, so dedicated tests retain the seven/eight-cycle indexed accesses and six-cycle PER specified by the [W65C816S datasheet, table 5-4](https://www.westerndesigncenter.com/wdc/documentation/w65c816s.pdf). Absolute counter/startup-phase differences remain unclassified; the timing ROMs are not claimed as fully passing.

The independent [CPU/SPC diagnostic release v1.4](https://github.com/gilyon/snes-tests/releases/tag/v1.4) adds 1,610 65C816 cases and 1,368 SPC700 cases. Both ROMs reach their final `Success` line. The full CPU ROM exposed the PLB stack-boundary error, which now has a separate regression. These ROMs test instruction results, flags, and addressing behavior; they do not establish cycle timing, hardware interrupts, or DSP accuracy. Their SLEEP/STOP and WAI/STP exclusions have separate core tests.

With that release extracted locally, enable both ROMs without downloading anything during the build:

```powershell
cmake --preset release -DSNES_PROCESSOR_DIAGNOSTIC_DIR=C:/path/to/snes-tests
cmake --build --preset build-release
ctest --test-dir out/build/release -R 'snes_external_(cpu|spc)' --output-on-failure
```

The earlier cartridge and DSP audit exercised these paths:

| Device | Comparison scope and result |
| --- | --- |
| SA-1 | 77,493 byte/result comparisons across arithmetic, bit reads, mapping, DMA, and character conversion at the time of that audit. Current bitmap, division, and bit-reader regressions cover the later corrections described above. Seven accumulator overflow-flag disagreements remain deliberate: the implementation detects signed 40-bit overflow. Hardware confirmation is still needed. Processor instruction execution was tested separately. |
| Super FX | 8,660 programs; register, flag, bank, and RAM results matched at the time of that audit. The current RAMBR reserved-bit behavior follows the hardware specification above. Plotting, cache storage, and timing also have independent tests. |
| DSP-3 | 935,231 byte/status/result comparisons matched, including coefficient ROM, geometry, planar data, 1,500 generated decode streams, and 250 terrain/path streams. |
| DSP-4 | 1,620,513 comparisons matched, including fixed commands, OAM budgets, 500 road/lighting streams, 300 polygon streams, and 300 object streams. |
| Cx4 | 8,956 command runs matched complete work-RAM snapshots across arithmetic and graphics paths. |
| SPC7110 | 180 compressed streams and 1,661,952 byte/register comparisons matched across three ROM sizes, all decoder modes, pointer controls, arithmetic, and banking. RTC behavior has separate deterministic tests. |
| S-DSP | 1,228,800 phases in 32 scenarios matched registers, decoder state, shared RAM, and 38,400 stereo output frames. |
| BS-X | 256 mapping configurations, 1,068 flash transactions, 371,783 reads, and 36,222 writes matched on common paths, including two packet receivers. The independently documented MCC differences, injected calendar, and malformed/exhausted stream handling were checked separately. |
| ST011/ST018 | 11,527,200 RAM/status/result comparisons matched on the implemented board-packet and startup-handshake paths. No internal processor or game-logic behavior was compared. |

The temporary comparison adapters and external source builds are local validation tools under `out/`; they are not dependencies of the committed tests. These comparisons do not cover every possible input or establish complete chip accuracy.

Each supplied game completed a 9,000-frame integration run from reset, with Start held for 20 frames beginning at frame 600:

| Game | Visible frames | Non-silent sample values | Captured state |
| --- | ---: | ---: | --- |
| Super Mario World | 8,833 | 8,052,131 | File selection menu |
| Contra IV - The Alien Wars prototype | 8,141 | 7,012,696 | Attract-mode action |
| Kirby Super Star | 8,010 | 7,922,048 | SA-1 attract-mode action |
| Super Mario RPG | 7,945 | 7,279,463 | SA-1 opening sequence |
| Super Mario All-Stars + Super Mario World | 8,877 | 8,477,772 | Game selection menu |

In the earlier audit, the four retail titles also ran in an independent comparison build. Their captures reached corresponding menus or sequences, with timing and animation differences; these were not pixel-identical comparisons. That build selected HiROM for the Contra prototype and produced no output, so it provided no valid game comparison for that image. The local core's prototype run was checked separately.

These runs check boot, sustained execution, output generation, and limited Start input. They do not establish complete playthroughs, every controller interaction, or commercial compatibility for every enhancement chip. GSU and other chips without supplied game images retain the synthetic and command-level coverage described above. Diagnostic BMP capture now converts the core's ABGR pixels to BMP channel order, so capture colors can be compared correctly.
