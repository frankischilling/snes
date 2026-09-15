# Audio playback

The DSP produces 32,000 stereo frames per second. It decodes BRR samples, applies Gaussian interpolation and envelopes, and mixes voices and echo with signed 16-bit saturation. The frontend converts these samples to floats and lets SDL resample them for the playback device.

The frontend prepares each picture ahead of its presentation deadline. Deadlines follow elapsed console master clocks, including PAL, NTSC, short scanlines, interlace and transfers that span several fields. Computation can finish early without displaying the next picture early. A stall during computation, event handling or presentation starts a fresh cadence instead of releasing a burst of overdue pictures.

The audio queue adjusts the wall-clock pace gradually, by at most 0.5%, to follow the playback device's clock. Queue measurements are averaged across frames so individual audio callbacks do not dictate when pictures appear. This adjustment does not change CPU, PPU or sound-chip clock relationships inside the core. VSync is disabled; visible tearing is still possible.

Queue depth uses bytes in the input format through [SDL_GetAudioStreamQueued](https://wiki.libsdl.org/SDL3/SDL_GetAudioStreamQueued). Converted output can have a different sample rate or channel count, so its byte count cannot be compared with the input buffer target. The controller targets 40 ms after submission. If an unusual transfer or device stall leaves at least 80 ms queued, production waits for the device. Every submitted block stays in order.

Playback starts after at least 40 ms has accumulated. If the stream runs out of playable output, it pauses and refills before restarting. During priming and recovery, the frontend keeps the latest prepared picture and presents it when playback starts. Resampler padding stays queued. An explicit pause remains in effect until `Resume()` is called. Device errors are reported, and SDL resources are destroyed before SDL shuts down.

Audio and video output buffers retain their capacity across frames. Output callbacks borrow these buffers only for the duration of the call; a consumer that retains their contents must copy them. Diagnostic screenshots are disabled by default. Set `SNES_PROBE_FRAME` to a nonnegative frame index to write that frame as a BMP.

The sound CPU's timers and DSP keep advancing during SLEEP and STOP. Each SMP bus or idle cycle advances one DSP phase. The 32-phase schedule captures voice registers, reads BRR and directory data, decodes samples, updates envelopes, mixes voices, and accesses echo RAM at their individual phases. A CPU write between phases can therefore affect the next latch without changing an earlier one. Power cycling clears the key-on latch.

A long DMA can advance several video fields before `StepFrame` returns. The core drains full DSP output blocks during those transfers and submits the collected samples together. The normal 2,048-sample buffer therefore does not truncate that operation's audio. A maximal eight-channel DMA test compares the submitted count with the DSP phase count and checks that the following frame contains no stale samples.

[MSU-1 packs](msu1.md) add file-backed stereo PCM. The core resamples PCM to 32 kHz and mixes it at each DSP output sample, before the next console bus operation. Playback continues without a host audio sink, and full buffers are drained during long transfers. Mid-frame volume, pause and track changes retain samples that have already played.

## Checks

`snes_audio_regression_tests` checks sound CPU halts and DSP reset behavior. `snes_dsp_phase_tests` checks register latch boundaries, shared-RAM echo writes, output timing, key-on delay, and SMP timer/port interactions. `snes_sdl_audio_tests` uses SDL's dummy audio device at 32, 44.1 and 48 kHz to check queue accounting, preservation of submitted blocks, startup buffering, underrun recovery and pause/resume behavior.

`snes_frame_pacing_tests` checks actual region and field lengths, fractional deadlines, delayed computation and presentation, and batched audio consumption with device-clock drift. `snes_sdl_video_tests` checks immediate and deferred rendering and resolution changes with SDL's dummy/software backend. These tests use synthetic samples and pictures; they do not measure physical display latency or GPU backbuffer behavior.

A local SDL dummy-audio probe compared queue-only production with scheduled presentation markers. Each run measured 300 intervals after 20 warm-up frames, using a requested 48 kHz device, 32 kHz input and simulated computation taking 3–5 ms per frame. Queue-only intervals ranged from 3.002 to 25.839 ms, with 85 below 10 ms and 16 above 25 ms. Scheduled intervals ranged from 15.736 to 17.510 ms, with none outside those thresholds. Neither run needed an underrun refill. These are scheduler measurements from one run of each mode, not physical display measurements.

```powershell
cmake --build --preset build-release
ctest --test-dir out/build/release --output-on-failure
```

The phase implementation matched 1,228,800 individual steps in 32 local differential scenarios, including DSP registers, decoder state, all shared RAM, and 38,400 stereo output frames. This checks the digital DSP behavior exercised by those scenarios. `Smp::RunUntil` stops at the requested clock, including inside an instruction or stretched access. TEST wait controls use the deterministic ratios described in [original hardware support](original-hardware.md#cpu-dma-and-audio-timing). Revision-specific long waits, sub-clock port collisions and analog output transients remain unmodeled. Buffering cannot compensate for a machine that consistently emulates slower than playback.
