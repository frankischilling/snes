# Light guns and raster memory

The frontend accepts Super Scope, one or two Justifiers, and MACS rifle on console port two:

```powershell
.\out\build\release\frontend\snes_frontend.exe --port2=scope "C:\games\game.sfc"
.\out\build\release\frontend\snes_frontend.exe --port2=justifier "C:\games\game.sfc"
.\out\build\release\frontend\snes_frontend.exe --port2=justifiers "C:\games\game.sfc"
.\out\build\release\frontend\snes_frontend.exe --port2=rifle "C:\games\game.sfc"
```

## Controls

| Action | First gun | Second Justifier |
| --- | --- | --- |
| Aim | Mouse pointer | I/J/K/L |
| Trigger | Left mouse button | Right Ctrl |
| Start | Enter | Backspace |
| Offscreen aim | Hold O or move outside the picture | Hold U |
| Scope cursor | Right mouse button | — |
| Scope turbo switch | T toggles | — |
| Scope pause | P | — |

Green and magenta crosshairs show the two aim positions. They are drawn over the displayed picture and are absent from saved probe frames. Unfocused windows release the gun buttons and report offscreen aim. The frontend rejects a relative mouse on port one when port two uses a gun, since both would need the same host pointer.

Aiming follows the centered picture when the window resizes. Letterbox borders count as offscreen. High resolution uses the same 256 horizontal aim positions; interlaced output maps back to 224 or 239 visible rows. Windows smaller than the native picture scale down to fit.

## Controller and beam behavior

Scope packets contain eight button/status bits followed by ones. Normal fire, cursor, and pause are reported once per press. Turbo repeats fire and cursor while held; pause remains a single event. A high strobe holds the first packet bit.

Justifier packets contain the identification prefix, separate trigger and Start bits, and a selection bit that alternates on each rising strobe. A pair shares one serial port, with only the selected gun providing beam coordinates. Selecting the absent second gun in single-gun mode produces no beam latch. Reads after the 32-bit packet return one. The rifle exposes its live trigger level without a shift register.

Manual reads and automatic joypad polling share the same packets and positions. Gun hits pull RDIO bit 7 low at the beam position and latch the PPU counters when WRIO bit 7 allows input. Only console port two has this PPU connection. Scope shots arm the beam independently of packet consumption, so reading a shot before the beam arrives does not discard its position. Offscreen positions do not latch.

Core input providers override `PollLightGun(port, gun, frame)` and return `LightGunState`. Coordinates use 256 horizontal positions and noninterlaced visible rows. Return the current button levels on every call; polling can happen more than once per frame.

## Raster fixes

The renderer previously saved registers and palettes per scanline but read tile memory and sprite attributes at VBlank. A later VRAM or OAM update could therefore change rows that had already passed. Each visible scanline retains its tile memory and parsed objects. Backgrounds, Mode 7, and sprites render from that saved state.

Unchanged lines share an immutable VRAM snapshot. The renderer copies 64 KiB only when a visible line first uses a new VRAM revision, then reuses the allocated storage in later frames. A frame with no visible VRAM changes needs one tile-memory snapshot instead of 224. Forced-blank uploads still create a new snapshot, preserving earlier lines. A retained mutable `VramData()` pointer enables memory comparisons before sharing snapshots, including after reset. Worst-case storage remains roughly 15 MiB for a distinct revision on each overscan line.

Raster register writes are kept in separate lists for each scanline. Rendering a line no longer searches every event in the frame or allocates a temporary event list. Event order and horizontal positions are unchanged.

Palette access also used incompatible timing units: the scheduler supplied horizontal dots to comparisons expressed in master clocks. It now supplies master clocks, allowing CGRAM access during HBlank and redirecting active-display accesses through the rendering latch. The PPU setter names the clock unit explicitly.

## Validation and limits

Tests cover packet endings, strobe behavior, turbo transitions, gun selection, WRIO gating, beam arrival, offscreen input, automatic polling, viewport edges, and graphics-memory changes between scanlines. Snapshot tests also change VRAM on all 239 overscan lines, reuse the storage across frames, reject blocked display-time writes, and keep a mutable pointer across reset. An integration test transfers palette data during active display and HBlank to check the clock-unit fix. The full test command and current hardware scope are listed in [original hardware support](original-hardware.md).

Both supplied games completed 1,800 frames with visible video and non-silent audio. The saved Contra prototype frame showed gameplay. No light-gun game was available for this check; synthetic protocol tests do not establish retail-game compatibility.

Beam input uses the scheduled aim position without simulating optical brightness thresholds or sensor persistence. CPU register accesses advance hardware per bus cycle. Graphics memory is sampled once per scanline; supported display, scroll, mosaic, window and color-math register writes split the rendered line at their recorded horizontal positions. Individual tile-fetch timing and optical sensor behavior remain unmodeled.
