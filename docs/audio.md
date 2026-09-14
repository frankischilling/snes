# Audio playback

The DSP produces 32,000 stereo frames per second. It decodes BRR samples, applies Gaussian interpolation and envelopes, and mixes voices and echo with signed 16-bit saturation. The frontend converts these samples to floats and lets SDL resample them for the playback device.

The audio queue controls emulation speed. The frontend runs another video frame when less than 40 ms of input is queued, then waits while the device consumes it. Queue depth can exceed that target by one emulated frame. VSync is disabled so the monitor's refresh rate cannot delay audio production; this can cause visible tearing.

Queue depth uses bytes in the input format through [SDL_GetAudioStreamQueued](https://wiki.libsdl.org/SDL3/SDL_GetAudioStreamQueued). Converted output can have a different sample rate or channel count, so its byte count cannot be compared with the input buffer target. Every submitted block stays in order. The frontend waits instead of discarding audio when the queue is full.

Playback starts after at least 40 ms has accumulated. If the stream runs out of playable output, it pauses and refills before restarting. Resampler padding stays queued. An explicit pause remains in effect until `Resume()` is called. Device errors are reported, and SDL resources are destroyed before SDL shuts down.

The sound CPU's timers and DSP keep advancing during SLEEP and STOP. Power cycling clears the DSP's key-on latch, so a note started immediately after reset behaves the same as on a fresh instance.

## Checks

`snes_audio_regression_tests` checks sound CPU halts and DSP reset behavior. `snes_sdl_audio_tests` uses SDL's dummy audio device at 32, 44.1 and 48 kHz to check queue accounting, preservation of submitted blocks, the frame pacing threshold, startup buffering, underrun recovery and pause/resume behavior. The tests use synthetic samples and require no game files.

```powershell
cmake --build --preset build-release
ctest --test-dir out/build/release --output-on-failure
```

The DSP processes all voices once per sample. Register writes are not modeled at each of the hardware's 32 DSP phases, so these checks do not establish exact audio timing for every game. Buffering also cannot compensate for a machine that consistently takes longer to emulate a frame than the audio device takes to play it.
