# Audio playback

The DSP produces 32,000 stereo frames per second. It decodes BRR samples, applies Gaussian interpolation and envelopes, and mixes voices and echo with signed 16-bit saturation. The frontend converts these samples to floats and lets SDL resample them for the playback device.

The audio queue controls emulation speed. The frontend runs another video frame when less than 40 ms of input is queued, then waits while the device consumes it. Queue depth can exceed that target by one emulated frame. VSync is disabled so the monitor's refresh rate cannot delay audio production; this can cause visible tearing.

Queue depth uses bytes in the input format through [SDL_GetAudioStreamQueued](https://wiki.libsdl.org/SDL3/SDL_GetAudioStreamQueued). Converted output can have a different sample rate or channel count, so its byte count cannot be compared with the input buffer target. Every submitted block stays in order. The frontend waits instead of discarding audio when the queue is full.

Playback starts after at least 40 ms has accumulated. If the stream runs out of playable output, it pauses and refills before restarting. Resampler padding stays queued. An explicit pause remains in effect until `Resume()` is called. Device errors are reported, and SDL resources are destroyed before SDL shuts down.

The sound CPU's timers and DSP keep advancing during SLEEP and STOP. Each SMP bus or idle cycle advances one DSP phase. The 32-phase schedule captures voice registers, reads BRR and directory data, decodes samples, updates envelopes, mixes voices, and accesses echo RAM at their individual phases. A CPU write between phases can therefore affect the next latch without changing an earlier one. Power cycling clears the key-on latch.

A long DMA can advance several video fields before `StepFrame` returns. The core drains full DSP output blocks during those transfers and submits the collected samples together. The normal 2,048-sample buffer therefore does not truncate that operation's audio. A maximal eight-channel DMA test compares the submitted count with the DSP phase count and checks that the following frame contains no stale samples.

## Checks

`snes_audio_regression_tests` checks sound CPU halts and DSP reset behavior. `snes_dsp_phase_tests` checks register latch boundaries, shared-RAM echo writes, output timing, key-on delay, and SMP timer/port interactions. `snes_sdl_audio_tests` uses SDL's dummy audio device at 32, 44.1 and 48 kHz to check queue accounting, preservation of submitted blocks, the frame pacing threshold, startup buffering, underrun recovery and pause/resume behavior. The tests use synthetic samples and require no game files.

```powershell
cmake --build --preset build-release
ctest --test-dir out/build/release --output-on-failure
```

The phase implementation matched 1,228,800 individual steps in 32 local differential scenarios, including DSP registers, decoder state, all shared RAM, and 38,400 stereo output frames. This checks the digital DSP behavior exercised by those scenarios. `Smp::RunUntil` still finishes whole SPC700 instructions and can overshoot a requested synchronization time, allowing a port read to run before an earlier console write is delivered. SMP TEST speed/wait-state controls and analog output transients remain unmodeled. Buffering cannot compensate for a machine that consistently emulates slower than playback.
