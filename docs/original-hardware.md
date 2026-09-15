# Original SNES hardware support

The core now executes SA-1 and Super FX programs, processes the supported DSP-3/4 and Cx4 commands, and exposes SPC7110 decompression and clock hardware. BS-X uses its own memory controller and packet receivers. These devices connect to the console bus, cartridge storage, and interrupt sampler; detection alone is not considered an implementation.

Existing LoROM, HiROM, extended and special boards, DSP-1/2, OBC1, S-RTC, S-DD1, ST010, Sufami Turbo, controllers, and PPU rendering paths remain in use. This work concerns original hardware compatibility. It does not add frontend features or change the renderer into a per-dot model.

## Cartridge implementations

| Device | Implemented behavior | Important limits |
| --- | --- | --- |
| SA-1 | Reuses the 65816 instruction core; reset/vector registers, mailbox IRQ/NMI, timers, 2 KiB IRAM and protection, shared BW-RAM and bitmap modes, ROM banking, arithmetic, variable-length bit reads, progressive DMA, both character-conversion modes, and master-clock advancement. | CPU advancement finishes whole instructions. ROM cache and bus contention penalties, exact character-conversion latency, DMA priority arbitration, and short/interlaced scanline coupling need further work. |
| Super FX/FX2 | GSU instruction and ALT encodings, prefetched pipeline and delay slots, register file, 512-byte cache, plotting/pixel rows, ROM/RAM buffers, screen modes, bus ownership, clock modes, IRQ, and cartridge maps including 4 MiB GSU2 images. | Cache fills and pixel writes publish at transaction completion rather than each constituent byte. Commercial game and hardware traces remain unverified. |
| DSP-3 | Byte/word status protocol, coordinate and hex-grid operations, planar conversion, diagnostic coefficient ROM, resumable prefix/LZ token decoding, and terrain/path result streams. | Command-level model. Several diagnostic commands have only provisional observed behavior; the internal DSP program and clock timing are not modeled. |
| DSP-4 | Command/output protocol, signed multiplication, road projection variants, incremental lighting, polygon windows, vehicle/terrain objects, sprite row budgets, and OAM attributes. | Command-level model. Reciprocal rounding and edge inputs need hardware measurements; there is no internal DSP instruction or busy-time model. |
| Cx4 | Work RAM, ROM transfer, arithmetic/vector commands, sprite/OAM conversion, affine and dissolve graphics, transformed vertices, wireframes, waves, window edges, and diagnostic responses. | Commands complete immediately. Internal processor execution and arbitration are not modeled. Malformed graphics dimensions are bounded to physical RAM. |
| SPC7110/RTC | Three decompression modes, banked data access, pointer/adjust/increment modes, multiply/divide, SRAM write gating, cartridge mapping, and an injectable persistent calendar. | Decoder/MMIO behavior is modeled at command level. Chip-internal decompression latency and bus contention are not established. |
| BS-X | BIOS overlay, latched MMC bits, 512 KiB PSRAM, 32 KiB battery SRAM, flash mapping and commands, ready IRQ, radio registers, calendar packets, and two independent recorded-data receivers. | Flash completion is synchronous. Block-lock recovery, physical /WP/CIC behavior, undocumented hidden-bit remapping, live reception, and SoundLink audio remain unimplemented. |
| ST011 | Command packets, the 128-byte board download, ready status, known result-byte writes, and separate work/battery RAM. | No internal processor or shogi calculations. Commands with unknown computations retain their limited observed protocol behavior. |
| ST018 | Three-byte startup commands `$000100` and `$00FF00`, parameter rounds, status, and `$81` replies. | No ARM processor, firmware, or game logic. Passing the startup handshake is not full chip emulation. |

DSP-3 uses banks `$20-$3F/$A0-$BF:$8000-$FFFF`; DSP-4 uses `$30-$3F/$B0-$BF` in the same offset range. `$8000-$BFFF` carries data and `$C000-$FFFF` reads status. Their mirrors share one command stream.

SA-1 slot boards with extended IDs `ZX3J` or `ZBPJ` accept a separate 512 KiB expansion ROM through the existing broadcast loader. MMC pages 4-7 select that ROM, an empty slot returns open bus, and the expanded console BW-RAM windows preserve WRAM at `$7E-$7F`. Expansion ROM is never exposed as writable flash storage.

SPC7110 calendar state uses a separate 24-byte `.rtc` representation: sixteen nibble-valued clock registers followed by an eight-byte little-endian timestamp. This differs from S-RTC's format; the loaded cartridge selects the decoder. Tests cover leap-day advancement, stopped clocks, rollback, and rejection of invalid state.

## CPU, DMA, and audio timing

CPU reads, writes, and internal cycles advance hardware individually. Registers can observe an event reached during an earlier access in the same instruction. IRQ entry uses the interrupt-mask value sampled before the final cycle, including CLI, SEI, REP, SEP, and PLP. Masked IRQs still release WAI.

DMA requests allow a CPU cycle before bus takeover. Transfers align to the DMA divider, charge startup, place source and destination accesses on separate phases, and resume the CPU at its cycle boundary. HDMA table and data accesses advance hardware as they occur and can interrupt general DMA. Refresh positions follow the divider instead of staying fixed on every scanline. The existing eight-channel modes, source-bank wrapping, and same-channel HDMA cancellation remain covered.

The S-DSP advances through 32 phases per output sample. Register latches, BRR/directory reads, envelopes, noise, pitch modulation, voice mixing, echo RAM, and output timing have separate phase behavior. Each SMP bus or idle cycle advances it once.

The frame collector drains full DSP blocks during long DMA operations. Transfers spanning several fields retain all generated audio instead of stopping at the ordinary 2,048-sample buffer limit. Video presentation still returns the latest completed field after such an operation.

This is not a fully cycle-exact machine. The PPU retains scanline snapshots, CPU multiply/divide results remain immediate, and a refresh penalty is drained after the current clock interval. SA-1 and SMP synchronization can finish an instruction beyond the requested timestamp. For the SMP, that can order a port read ahead of a console write which should have arrived first; per-phase DSP timing does not solve this separate instruction-scheduling limit. SMP TEST speed/wait-state bits and analog audio behavior remain outside the model.

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

All 29 CTest targets passed in Release and AddressSanitizer builds on Windows with Clang 21. Selected command-engine tests also passed undefined-behavior trap instrumentation. The Windows legacy test executable reserves an 8 MiB stack so sanitizer instrumentation can retain its large fixtures.

Run the complete test set with:

```powershell
cmake --build --preset build-release
ctest --test-dir out/build/release --output-on-failure
```

Committed tests use generated programs, command streams, known output vectors, and deterministic clocks. Native 65816 programs exercise cartridge ports through the real bus. Processor tests run SA-1 and GSU programs, check their IRQs, and verify that save loading retains shared storage. Bus tests place events within instructions and verify DMA timestamps. Audio tests change registers and shared RAM across individual DSP phases.

Local differential checks exercised these paths:

| Device | Comparison scope and result |
| --- | --- |
| SA-1 | 77,493 byte/result comparisons across arithmetic, bit reads, mapping, DMA, and character conversion. Seven accumulator overflow-flag disagreements remain deliberate: the implementation detects signed 40-bit overflow. Hardware confirmation is still needed. Processor instruction execution was tested separately, not by this comparison. |
| Super FX | 8,660 programs; register, flag, bank, and RAM results matched. Plotting, cache storage, and timing also have independent tests. |
| DSP-3 | 935,231 byte/status/result comparisons matched, including coefficient ROM, geometry, planar data, 1,500 generated decode streams, and 250 terrain/path streams. |
| DSP-4 | 1,620,513 comparisons matched, including fixed commands, OAM budgets, 500 road/lighting streams, 300 polygon streams, and 300 object streams. |
| Cx4 | 8,956 command runs matched complete work-RAM snapshots across arithmetic and graphics paths. |
| SPC7110 | 180 compressed streams and 1,661,952 byte/register comparisons matched across three ROM sizes, all decoder modes, pointer controls, arithmetic, and banking. RTC behavior has separate deterministic tests. |
| S-DSP | 1,228,800 phases in 32 scenarios matched registers, decoder state, shared RAM, and 38,400 stereo output frames. |
| BS-X | 256 mapping configurations, 1,068 flash transactions, 371,783 reads, and 36,222 writes matched on common paths, including two packet receivers. The independently documented MCC differences, injected calendar, and malformed/exhausted stream handling were checked separately. |
| ST011/ST018 | 11,527,200 RAM/status/result comparisons matched on the implemented board-packet and startup-handshake paths. No internal processor or game-logic behavior was compared. |

The temporary comparison adapters and external source builds are local validation tools under `out/`; they are not dependencies of the committed tests. These comparisons do not cover every possible input or establish complete chip accuracy.

After the timing changes and interrupt-lock correction, each supplied game completed a 1,800-frame smoke run with Start at frame 600. Super Mario World produced 1,633 visible frames and 1,439,993 non-silent sample values; the Contra prototype produced 1,448 visible frames and 357,509 non-silent sample values. These are checks of boot and early execution, not playthroughs or validation of games using the added cartridge processors.
