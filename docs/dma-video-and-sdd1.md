# S-DD1, DMA timing, and video output

S-DD1 cartridges load with a decompressor, bank registers, and board-specific SRAM mapping. CPU, DMA, and HDMA accesses advance connected hardware during their bus cycles. Video output retains the 512 dots of high-resolution modes and both interlaced fields.

## S-DD1 cartridge support

Registers $4800 and $4801 enable decompression per DMA channel. A transfer must read from $C0-$FF with fixed A-bus addressing and both channel bits enabled. CPU reads continue to return compressed ROM bytes. Completion clears that channel's bit in $4801. Reverse transfers, incrementing transfers, and disabled channels use ordinary bus reads and writes.

Registers $4804-$4807 select four independent 1 MiB ROM windows. The decoder reads through those windows, including when compressed input crosses a window boundary. Register aliases occupy banks $00-$3F and $80-$BF. Loading a cartridge resets the four bank selections to 0, 1, 2, and 3.

| Address | Mapping |
| --- | --- |
| $00-$3F/$80-$BF:$8000-$FFFF | Fixed LoROM pages |
| $60-$6F:$0000-$FFFF | Fixed 64 KiB ROM pages |
| $70-$7D:$8000-$FFFF | Upper halves of fixed ROM pages |
| $70-$7D:$0000-$7FFF and $A0-$BF:$6000-$7FFF | SRAM, mirrored to its populated size |
| $C0-$FF:$0000-$FFFF | Four switchable ROM windows |

WRAM retains banks $7E-$7F. Unpopulated SRAM returns open bus. Battery RAM uses the existing `.srm` load and save path.

The decompressor handles all four bitplane layouts and all four context models described in the [S-DD1 algorithm documentation](https://wiki.superfamicom.org/s-dd1). Each transfer has independent prediction and run state. A zero DMA count requests 65,536 output bytes; odd counts end at the requested byte.

Expanded, pre-decompressed images of at least 8 MiB use a separate mapper. Detection uses the S-DD1 chip header or the supported game titles. The mapper exposes the expanded ROM pages directly, keeps missing banks open, and does not expose chip registers or SRAM. Synthetic tests cover 8 MiB and 12 MiB layouts.

## DMA and field timing

Writing $420B queues DMA. The controller allows one CPU bus cycle before taking the bus, aligns to its eight-clock divider, and charges global and per-channel startup intervals. Each byte has two four-clock read phases followed by the destination write. HDMA table reads and data transfers use the same clock path. WRAM refresh pauses transfers, and HDMA can interrupt general DMA between bytes. HDMA on the same channel cancels that general DMA transfer; another channel's HDMA lets it resume. Source offsets wrap within their original bank.

The scheduler returns DMA control at a CPU cycle boundary and samples interrupts through the CPU's normal instruction boundary logic. HDMA initializes on scanline zero at a divider-dependent position. DRAM refresh follows that divider, alternating positions 538 and 534 on ordinary scanlines and preserving its phase across a short line. Odd-sized clock batches retain their unfinished half tick. SETINI controls field timing, and `StepFrame` targets the next field boundary.

CPU register accesses now see hardware events reached during earlier accesses of the same instruction. This does not make the whole machine cycle exact: the PPU still samples one scanline, coprocessor synchronization has device-specific limits, and the scheduler drains a refresh penalty at the end of the current clock interval. An instruction or DMA operation crossing a field boundary can make `StepFrame` return after that boundary.

## Video output

Modes 5 and 6 and pseudo-hires produce 512 output dots. Even dots use the sub screen and odd dots use the main screen, with color math and brightness applied during composition. If a frame mixes low and high resolution lines, each low resolution dot is repeated horizontally. Forced blanking clears the full output width.

Interlace writes alternating row parities and retains the preceding field. Normal and overscan output heights are 224 and 239 rows, or 448 and 478 in interlace. The SDL frontend uploads the complete frame and keeps the same display proportions when its pixel dimensions change.

## Validation and remaining gaps

The decoder passed 3,072 local comparison cases spanning all 16 header modes, 32 deterministic input streams, and six output lengths from one byte through 64 KiB. Committed tests retain output digests for every header mode and check bank boundaries, register aliases, save round trips, all eight DMA channels and modes, enable masks, reverse DMA, and expanded images.

Timing tests check divider alignment, separate source/destination timestamps, startup and CPU resumption, refresh stalls, HDMA interruption on the same and different channels, APU progress, bank wrapping, and NTSC/PAL interlaced field lengths. CPU bus tests place HBlank and NMI edges inside instructions. PPU tests check alternating main/sub dots, mixed resolutions, blanking, field retention, and returning to normal dimensions.

The full test command and the current cartridge validation scope are listed in [original hardware support](original-hardware.md). Tests keep the transfer data-path checks separate from integration checks that advance the complete machine.

Both supplied games completed 1,800-frame runs with visible video and non-silent audio. The final Contra prototype frame showed gameplay. These runs do not validate S-DD1 games, later levels, or sound accuracy. No game images are included in the tests.

[Original hardware support](original-hardware.md) describes the cartridge processors, command engines, broadcast hardware, and their remaining gaps. [Light guns and raster memory](light-guns-and-raster.md) covers gun protocols and scanline VRAM/OAM snapshots. CPU multiply/divide results remain immediate, and the renderer samples state once per scanline. S-DD1 decoding does not model its internal input FIFO timing or hardware underflow behavior.
