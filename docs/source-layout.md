# Source layout

The core implementation is grouped by the hardware area it models:

| Folder | Contents |
| --- | --- |
| `core/src/audio` | S-DSP, SMP, SPC700, DSP-1, DSP-2, and the cartridge clock |
| `core/src/cartridge` | ROM headers, mappers, save RAM, SA-1, Super FX, DSP-3/4, Cx4, S-DD1, SPC7110/RTC, ST commands, and BS slot/controller hardware |
| `core/src/cpu` | 65816 processor and SNES CPU wrapper |
| `core/src/dma` | General DMA and HDMA |
| `core/src/io` | CPU I/O registers, interrupts, controller protocols, and automatic polling |
| `core/src/memory` | The unified 24-bit memory bus |
| `core/src/ppu` | PPU registers and scanline rendering |
| `core/src/system` | Emulator lifecycle and logging |
| `core/src/timing` | Master clock and video-region timing |

Public headers remain under `core/include/snes/core`, so callers keep the same include paths. CMake lists the implementation files by subsystem. The frontend and command-line tools keep their own `src` folders because they are separate targets.
