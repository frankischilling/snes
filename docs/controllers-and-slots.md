# Controllers and cartridge slots

## Controller setup

Each console port accepts `pad`, `mouse`, `multitap`, or `none`. Both default to `pad`. Port two also accepts `scope`, `justifier`, `justifiers`, and `rifle`; see [light-gun setup and controls](light-guns-and-raster.md).

```powershell
.\out\build\release\frontend\snes_frontend.exe --port1=mouse "C:\games\game.sfc"
.\out\build\release\frontend\snes_frontend.exe --port2=multitap "C:\games\game.sfc"
```

The keyboard and first host gamepad control player one. Additional host gamepads control subsequent players in connection order. The keyboard uses arrow keys, Z/X for B/A, A/S for Y/X, Q/W for L/R, Enter for Start, and Right Shift for Select. Host gamepad face buttons map by position: south/east/west/north become B/A/Y/X. The left stick also controls the directional pad.

A multitap takes four consecutive player slots. With a pad on port one and a multitap on port two, players two through five use the multitap. Two multitaps expose eight players. A mouse or empty port reserves one player slot. Disconnecting a host gamepad releases its buttons and preserves the other assignments; a new device takes the first vacant slot.

Mouse mode captures relative movement in the window. Escape exits. The frontend supports one host mouse on either console port; the core can receive separate mouse input for both ports.

## Serial behavior

Manual reads at $4016/$4017 and automatic polling at $4218-$421F share the same controller packets and read positions. Automatic polling consumes sixteen bits per port. A subsequent manual read continues where polling stopped, which lets software read a mouse's movement after its button word.

Gamepads serialize B, Y, Select, Start, Up, Down, Left, Right, A, X, L, R, then four zero bits. Further reads return one. Opposing directions cancel. Each port receives independent input, including when an older input provider only supplies player one.

The CPU latch and automatic latch are combined. A rising edge captures input; reads while the latch is high do not advance the packet. Multitaps return a two-bit identification value of 2 while latched. WRIO bit 6 selects the pad pair on port one, and bit 7 selects it on port two. Each pair keeps its own read position. An empty multitap socket returns zero even after the other socket finishes its packet.

Mouse packets contain eight zero bits, a button/sensitivity/signature byte, signed Y movement, and signed X movement. Each movement byte uses a sign bit and a seven-bit magnitude. Movement beyond 127 counts carries into later packets. Reads with the latch high cycle the three sensitivity settings. These settings appear in the packet; the frontend passes host movement counts without applying an additional sensitivity curve.

Core callers configure devices through `GetAutoJoypad().Ports().Configure(...)`. An explicit array assigns up to four player IDs to a port, with `-1` for an empty socket. Input providers can override `PollController(player, frame)` and `PollMouse(port, frame)`. Mouse deltas must be consumed once per call. Reset preserves device assignments and clears protocol state.

## BS cartridge slots

```powershell
.\out\build\release\frontend\snes_frontend.exe --broadcast "C:\games\base.sfc" "C:\games\pack.bs"
```

Use `-` for the pack to insert a blank 1 MiB flash pack. The loader checks the base cartridge's extended header and accepts LoROM, HiROM, and the 24-Mbit LoROM board. Bases requiring an enhancement processor are rejected. Pack images must contain 1 MiB; a 512-byte copier header is accepted.

LoROM boards expose the pack at $C0-$EF, with 32 KiB bank addressing and mirrored bank halves. HiROM boards expose pack data in $20-$3F/$A0-$BF:$8000-$FFFF, $60-$7D, and $E0-$FF. Only $E0-$FF accepts HiROM flash commands. Base ROM, save RAM, and console WRAM retain their own address windows.

Flash supports byte programming ($10 or $40 followed by data), 64 KiB block erase ($20 followed by $D0), compatible status ($70), extended status ($71), vendor identification ($75), status clear ($50), and return to array reads ($00 or $FF). Programming clears bits; erasing restores them to one. Compatible status returns $80 once. Operations complete immediately. Type-7 mask ROM packs are read-only and have no flash save.

The frontend saves flash contents to the supplied pack filename with `.flash` appended, such as `pack.bs.flash`. A blank pack uses the base filename with the same suffix. Saves load before execution and flush every 300 frames and on normal exit. They use the existing temporary-file replacement mechanism. A save of the wrong size stops loading. Base cartridge SRAM remains in its `.srm` file.

This implements the slot boards and the listed flash commands. The BS-X broadcast BIOS, satellite streams, PSRAM/MMC remapping, other pack sizes, flash program/erase timing, and additional pack command sets remain unsupported.

## Combined Sufami Turbo images

The ordinary ROM command also accepts a combined image containing a base BIOS at offset zero, slot A at $100000, and optional slot B at $200000. The image must be 2 or 3 MiB. The loader checks the BIOS and game signatures and treats a slot filled entirely with $00 or $FF as empty. Truncated slots are rejected before replacing the loaded cartridge.

Combined images use one 32 KiB `.srm`: the first 16 KiB belongs to slot A and the second to slot B. The separate-file `--sufami` command still saves each game beside its own image. A standalone 256 KiB base BIOS maps the BIOS windows and leaves both game slots empty.

## Validation

The controller tests exercise manual/automatic polling through CPU I/O, mouse packets and sensitivity, independent multitap pair counters, empty sockets, and reads past packet end. SDL tests attach virtual gamepads and verify input, disconnects, and replacement devices. Cartridge tests cover the slot maps, flash commands, save round trips, combined images, and failed-load preservation. These tests use generated data and do not require game files.

Local 1,800-frame runs with Start at frame 600 produced 1,633 visible frames for Super Mario World and 1,448 for the Contra prototype. Both generated non-silent audio; the saved Contra frame showed gameplay. These runs check existing game behavior, not commercial mouse, multitap, or slot-cartridge compatibility.

SA-1, Super FX, DSP-3/4, SPC7110 and its RTC, Cx4, ST010/011/018, and MSU-1 still need implementations. [S-DD1, DMA timing, and video output](dma-video-and-sdd1.md) describes the added decompressor and mapper. This change does not establish full CPU, video, audio, or timing accuracy.
