# Cartridge devices and region timing

The loader distinguishes cartridge processors using the mapping byte, cartridge type, and, for DSP-3, maker code. An unsupported device produces an error with its name. A failed load leaves the current cartridge, RAM, and CPU bus intact. A successful load resets the machine and its frame counters.

## DSP-2

DSP-2 cartridges use data ports at $20-$3F/$A0-$BF:$6000-$6FFF and $8000-$BFFF. The ports share one command stream. Reads consume queued result bytes and return $FF when the result is exhausted. Addresses $C000-$FFFF retain ROM access.

The implementation handles packed-pixel conversion to SNES bitplanes, transparent color selection, bitmap overlays, pixel reversal, unsigned 16-bit multiplication, and fixed-point bitmap scaling. Parameter RAM survives between commands. Empty bitmap requests complete without leaving the parser waiting for a payload. This defines recovery for empty requests; it does not reproduce undocumented hardware behavior for them.

DSP-1 remains available through its existing board windows. DSP-3 and DSP-4 have separate command engines and narrower cartridge windows, described in [original hardware support](original-hardware.md).

## OBC1

OBC1 has 8 KiB of RAM at $00-$3F/$80-$BF:$6000-$7FFF. Registers $7FF0-$7FF3 read and write the selected object's four bytes. Register $7FF4 reads its packed attribute byte and writes only the selected object's two bits. Registers $7FF5 and $7FF6 select the object table and object index. Writes also update the register locations in RAM.

RAM and selectors reset to $FF when a cartridge loads. [Super Scope input](light-guns-and-raster.md) is available separately. The object controller and gun protocol have synthetic tests, but games requiring both have not been tested.

## S-RTC

The clock reads through $2800 and accepts commands through $2801 in banks $00-$3F/$80-$BF. Commands select read mode, program the calendar, or clear it. Reads contain a leading $F, thirteen calendar nibbles, and a trailing $F. A read sequence keeps one calendar snapshot even if a save occurs during the read.

A new clock starts with the host's local date and time. Games can program another date. Calendar advancement uses elapsed host seconds, accounts for Gregorian leap years, and ignores clock rollback. The clock also advances across emulator restarts.

The frontend stores clock data in a `.rtc` file beside the ROM, using the same temporary-file replacement as battery RAM. It saves every 300 frames and on normal exit. The file contains thirteen calendar bytes, the three ASCII bytes `RTC`, and an eight-byte unsigned Unix timestamp in little-endian order. This 24-byte format is specific to this emulator. Invalid signatures and sizes stop loading without replacing the file.

## Sufami Turbo

Supply a 256 KiB BIOS and one or two game images:

```powershell
.\out\build\release\frontend\snes_frontend.exe --sufami "C:\games\bios.bin" "C:\games\slot-a.st" "C:\games\slot-b.st"
```

Omit the last argument to leave slot B empty. Use `-` for either empty slot, including both slots for BIOS-only operation. BIOS and game signatures are checked, and 512-byte copier headers are removed. Game images must be between 512 KiB and 1 MiB. This command takes separate files. The ordinary ROM command also loads [combined cartridge images](controllers-and-slots.md#combined-sufami-turbo-images).

| Address window | Device |
| --- | --- |
| $00-$1F/$80-$9F:$8000-$FFFF | BIOS, mirrored within 256 KiB |
| $20-$3F/$A0-$BF:$8000-$FFFF | Slot A ROM |
| $40-$5F/$C0-$DF:$8000-$FFFF | Slot B ROM |
| $60-$63/$E0-$E3:$8000-$FFFF | Slot A SRAM, mirrored within 16 KiB |
| $70-$73/$F0-$F3:$8000-$FFFF | Slot B SRAM, mirrored within 16 KiB |

Empty slots return open bus and have no save file. Each populated slot loads and saves a `.srm` beside its game image. A save follows the game when it moves between slots. The frontend rejects slots that resolve to the same save path. BIOS and game images are not included.

The core also exposes `LoadSufamiTurbo`, `LoadSufamiTurboFromFiles`, `SlotSramData`, and `LoadSlotSram` for callers that manage files themselves.

## Region timing and loader behavior

Country codes 2 through 12 and 18 select PAL. Other country codes select NTSC. PAL uses 312 scanlines and a 21,281,370 Hz master clock; NTSC uses 262 scanlines and the existing 21,477,272 Hz clock. The PPU's region flag and SMP synchronization use the selected region. Audio output remains 32 kHz.

Small LoROM boards expose SRAM in both bank halves when the ROM-size header is at most 2 MiB and SRAM is at most 32 KiB. Larger layouts retain ROM in the upper half. An absent SRAM window returns open bus. Database overrides can explicitly disable SRAM with a size of zero. MEMSEL controls fast accesses independently of the speed advertised in the ROM header.

The loader rejects erased headers and headers without a cartridge-space reset vector. Reserved mode bytes cannot override the physical header layout. SA-1, Super FX, S-DD1 and SPC7110 retain their boot headers even when data in an extended ROM bank resembles another cartridge header.

Type-1 interleaved LoROM and HiROM images are normalized when their displaced headers identify that layout. ExHiROM also accepts independently interleaved boot and data chips in either physical order. A native HiROM header of equal or better quality prevents speculative deinterleaving. Each transformed chip must contain complete pairs of 32 KiB blocks.

For 5–8 MiB ExLoROM and ExHiROM images, the loader accepts either physical chip order and places the boot chip after the four-MiB data region. Reset windows, ROM mirrors and the cartridge CRC then use the same canonical bytes. A HiROM header at $40FFC0 selects ExHiROM. GD24 and type-2 forced dump formats do not yet have a format-selection interface; their bank permutations cannot be inferred safely from a plausible header alone.

## Validation and limits

`snes_cartridge_device_tests` covers device detection, rejected loads, bank boundaries, save separation, DSP-2 command results, OBC1 attribute packing, clock persistence and calendar rollover, and PAL frame/audio timing. Loader regressions compare every normalized ROM byte and CRC, verify both reset windows, and preserve native images with competing headers. Tests use generated ROMs and an injected clock, so they need no game files.

Local 1,800-frame checks produced visible output and non-silent audio for Super Mario World and the Contra prototype. The final Contra frame showed gameplay. These checks do not prove complete gameplay, correct sound, or compatibility with commercial games that use the new devices.

[Controllers and cartridge slots](controllers-and-slots.md) describes mouse, multitap, BS slot boards, and combined Sufami Turbo support. [S-DD1, DMA timing, and video output](dma-video-and-sdd1.md) covers decompression, bank switching, expanded ROM layouts, and video changes.

[Original hardware support](original-hardware.md) covers SA-1, Super FX/FX2, DSP-3/4, Cx4, SPC7110 and its RTC, and BS-X. ST011 has a limited packet interface and ST018 has startup handshakes; neither has its internal program processor. Register and mapper coverage does not establish full system compatibility.
