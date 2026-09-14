# Cartridge compatibility and saves

## Contra prototype

The supplied Contra prototype has an intact reset vector and an incomplete header whose mapping byte is $FF. The loader infers the layout from the selected header's position when the mode byte is unrecognized. LoROM, HiROM, ExLoROM, and ExHiROM headers are considered, including images with a 512-byte copier header.

Before the loader fix, a 120-frame run produced no visible frames or audio. Afterward, a 1,800-frame run with Start pressed at frame 600 reached gameplay, produced 1,448 visible frames, and generated audio. This checks boot and early gameplay; it does not establish that every level works.

The frontend displays the selected mapping by name. Frame execution no longer reads memory to print hang diagnostics. Those reads could change bus state, and long block moves could trigger false hang reports.

## Additional cartridge layouts

| Layout | Address behavior | Detection |
| --- | --- | --- |
| ExLoROM | Banks $80-$FF select the first 4 MiB; lower banks select the remaining ROM. Each populated section mirrors independently. | Header at file offset $407FC0. |
| NoMAD-1 LoROM | SRAM occupies both halves of banks $70-$7D and $F0-$FF. | `WANDERERS FROM YS` header title. |
| 24 Mbit board | Banks $00-$1F select the first MiB, $20-$3F and $A0-$BF the second, and $80-$9F the third. | `SOUND NOVEL-TCOOL` or `DERBY STALLION 96`. |
| Large SRAM board | Banks $70-$73 expose overlapping 64 KiB windows at 32 KiB SRAM offsets. | `THOROUGHBRED BREEDER3` or `RPG-TCOOL 2`. |

WRAM retains priority in banks $7E-$7F. Synthetic tests cover the added layouts, ROM mirrors, independent SRAM banks, and upper-half SRAM access. ExLoROM fixtures cover 5, 7, and 8 MiB images. These are address-decoding tests, not game compatibility tests for those boards.

## Battery saves

The frontend loads a `.srm` file beside the ROM when the cartridge has SRAM. Changed SRAM is saved every 300 frames and on normal exit. Unchanged data does not rewrite the file, and cartridges without SRAM create no save file.

Each save is written to a temporary file beside the destination, then renamed over the previous save. A save with the wrong size stops loading with an error and remains untouched. Tests cover loading, replacement, unchanged saves, size mismatches, and failed replacements.

## Headless game checks

The smoke tool runs a supplied ROM without opening a window. It reports visible frames and non-silent audio samples, presses Start for 20 frames, and can save the final frame as a BMP:

```powershell
.\out\build\release\tools\snes_rom_smoke.exe ".\game.sfc" 1800 ".\out\game.bmp" 600
```

The arguments are ROM path, frame count, optional output path, and optional Start frame (default 240). Exit code 0 means at least one frame contained nonblack pixels; code 1 means none did; code 2 means loading, argument parsing, or output failed. A successful result does not prove correct gameplay or sound. The tool does not load or save battery files.

## Remaining work

[Cartridge devices and region timing](cartridge-devices.md) covers DSP-2, OBC1, S-RTC, Sufami Turbo loading and saves, PAL timing, and loader regression tests.

[Controllers and cartridge slots](controllers-and-slots.md) covers mouse and multitap input, BS slot board mappings and flash saves, and combined Sufami Turbo images.

SA-1, Super FX, DSP-3/4, S-DD1, SPC7110 and its RTC, Cx4, ST010/011/018, the full BS-X broadcast system, and MSU-1 remain unimplemented. These need processors, register models, data paths, or peripheral behavior in addition to address mapping. Super Scope, Justifiers, MACS rifle, save states, and rewind also remain unfinished.
