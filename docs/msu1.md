# MSU-1 data and audio

The file loader enables MSU-1 when it finds an `.msu` file beside the ROM with the same stem. For example:

```text
game.sfc
game.msu
game-1.pcm
game-2.pcm
```

Run `game.sfc` through the normal frontend command. No extra flag is needed. An empty `.msu` file is valid for an audio-only pack. Track filenames use decimal numbers without leading zeroes, including track 0 and track 65535. Loading another cartridge closes the old pack and removes its register mapping. Loading a ROM from an in-memory byte array does not search for sidecars.

## Registers and files

The eight registers occupy `$2000-$2007` in banks `$00-$3F` and `$80-$BF`. Reads expose revision 2, playback status, the data stream and the `S-MSU1` signature. Writes select a 32-bit data position, a 16-bit track, volume, play/repeat state and a resume point. The highest byte commits a data seek or track selection. Data reads advance the stream, including repeated DMA reads from a fixed source address. Reads past EOF return zero; a later seek can read the file again.

PCM files start with `MSU1`, followed by a little-endian 32-bit loop position measured in stereo frames. The payload is signed little-endian 16-bit stereo at 44,100 frames per second. Missing tracks, invalid headers, empty payloads and incomplete final frames set the audio-error flag. Loop positions outside the payload restart at its first frame. Stopping with the resume bit set saves one track position; selecting that track consumes the saved position.

Data and PCM each use an 8 KiB file cache. Seeks use 64-bit file offsets, so large data files do not require equally large allocations. File operations complete synchronously and leave the busy flags clear.

## Audio timing

PCM advances at each S-DSP output sample. Integer interval averaging converts 44.1 kHz PCM to the core's 32 kHz stream without accumulating rate error. The mixer applies the current volume, combines signed stereo samples with DSP output and clamps the result to 16 bits. Volume or playback writes during a video frame affect subsequent samples. PCM also advances without an attached audio output and through DMA transfers that cross several video frames.

The resampling filter is a simple averaging filter. It is not a high-order reconstruction filter, and the 32 kHz mixed output limits audio bandwidth. Loose files are supported; compressed `.msu1` archives and alternate manifest naming are not.

## Checks

`snes_msu1_tests` uses generated data and PCM files. It checks sparse data through the 4 GiB boundary, cache boundaries, register mirrors, malformed tracks, loop fallback, pause/resume, signed volume and an independent sample-interval oracle at several output rates.

`snes_msu1_integration_tests` runs generated console programs. It checks automatic discovery, cartridge replacement, data DMA and mid-frame audio controls in NTSC and PAL. A varying waveform is compared sample by sample through three DMA buffer flushes and through audio attachment during headless playback. A paired native-DSP check covers addition and both clipping limits. These tests require no commercial ROM or soundtrack files.
