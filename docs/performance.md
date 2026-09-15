# Performance checks

The renderer shares unchanged VRAM snapshots across scanlines and keeps raster events in separate lists for each line. This removes repeated 64 KiB copies and searches through unrelated register writes. The CPU bus fix and optional MSU-1 mixer were included in the same comparison.

## Local measurements, 2026-09-15

The baseline was commit `8224ace`, with the same timing instrumentation added to `snes_rom_smoke`. Both binaries used Clang 21.1.5, CMake Release and the same Windows host with an Intel Core i7-13700H. Each game ran 1,200 frames with Start held for 20 frames beginning at frame 240. The run order was baseline, changed, changed, baseline. The workstation was in use during the measurements.

Throughput below is 2,400 frames divided by the combined duration of each pair of runs. These are headless measurements with the tool's video and audio collectors attached. They exclude ROM loading and image-file writing, and do not measure display presentation or audio-device latency. None of these runs used MSU-1 tracks.

| Game | Baseline frames/s | Changed frames/s | Throughput change |
| --- | ---: | ---: | ---: |
| Super Mario World | 137.88 | 160.34 | +16.3% |
| Kirby Super Star | 94.66 | 109.36 | +15.5% |
| Super Mario RPG | 97.83 | 113.91 | +16.4% |
| Super Mario All-Stars + Super Mario World | 140.96 | 165.27 | +17.2% |

The 95th-percentile frame-time ranges below show both individual runs; they are not percentiles pooled across the two runs.

| Game | Baseline p95, ms | Changed p95, ms |
| --- | ---: | ---: |
| Super Mario World | 8.633–8.669 | 7.385–7.735 |
| Kirby Super Star | 11.697–12.823 | 10.518–11.018 |
| Super Mario RPG | 12.044–12.536 | 10.401–10.437 |
| Super Mario All-Stars + Super Mario World | 8.079–10.020 | 7.147–8.585 |

Timing varied between runs. A preliminary unpaired Mario World run reached 49.001 ms for its slowest frame; the paired changed runs peaked at 9.433 and 11.562 ms. Mario RPG still reached 22.078 ms in one changed run. The reduced typical cost does not eliminate occasional spikes or establish the same performance on another machine.

## Output checks

Visible-frame counts, audible-sample counts and final CPU positions matched the baseline in every paired run. Separate 1,200-frame captures for these four games and the supplied Contra prototype had identical BMP hashes before and after the changes. The images were also inspected for their expected title and demo scenes.

The Contra prototype needs a later scripted Start to exercise gameplay in this check. At 1,800 frames with Start at frame 600, both binaries produced 1,441 visible frames and 353,459 audible channel samples, ended at `01:A61E`, and saved identical gameplay images. A shorter 1,200-frame run with Start at frame 240 produced no audible samples in either binary. That script difference is not evidence of an audio regression.

These checks cover the scripted boot and demo paths. Matching one final frame and output counts does not establish complete game compatibility or pixel-for-pixel equivalence throughout every run.

The smoke tool prints elapsed time, throughput, median, p95, p99 and maximum frame time. Its arguments are documented in [Cartridge compatibility and saves](compatibility.md).
