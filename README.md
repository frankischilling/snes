# snes

A SNES emulator written in C++.

Build and test from the repository directory with CMake, Ninja and a C++20 compiler:

```powershell
cmake --preset release
cmake --build --preset build-release
ctest --test-dir out/build/release --output-on-failure
```

Run a game on Windows:

```powershell
.\out\build\release\frontend\snes_frontend.exe "C:\path\to\game.sfc"
```

[Rendering and LoROM checks](docs/accuracy-notes.md) describes the fixes checked against Snes9x and their regression tests.
