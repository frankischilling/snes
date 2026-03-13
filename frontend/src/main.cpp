// ============================================================================
// main.cpp — SNES emulator SDL3 frontend
//
// Usage:  snes_frontend <rom_file.smc>
//
// SDL init → load ROM → run StepFrame() per vsync → event pump → quit.
// ============================================================================

#include "SdlVideoOutput.hpp"
#include "SdlAudioOutput.hpp"
#include "SdlInputProvider.hpp"

#include "snes/core/Emulator.hpp"
#include "snes/core/Logging.hpp"

#include <SDL3/SDL.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

int main(int argc, char* argv[]) {
    // ------------------------------------------------------------------
    // 1. Parse command-line arguments
    // ------------------------------------------------------------------
    if (argc < 2) {
        std::cerr << "Usage: snes_frontend <rom_file.smc>\n";
        return EXIT_FAILURE;
    }
    const std::string romPath = argv[1];

    // ------------------------------------------------------------------
    // 2. Initialise SDL (video + audio + events)
    // ------------------------------------------------------------------
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return EXIT_FAILURE;
    }

    // ------------------------------------------------------------------
    // 3. Create SDL platform objects
    // ------------------------------------------------------------------
    std::unique_ptr<snes::frontend::SdlVideoOutput> video;
    std::unique_ptr<snes::frontend::SdlAudioOutput> audio;
    std::unique_ptr<snes::frontend::SdlInputProvider> input;

    try {
        video = std::make_unique<snes::frontend::SdlVideoOutput>();
        audio = std::make_unique<snes::frontend::SdlAudioOutput>();
        input = std::make_unique<snes::frontend::SdlInputProvider>();
    } catch (const std::exception& e) {
        std::cerr << "Failed to create SDL subsystem: " << e.what() << '\n';
        SDL_Quit();
        return EXIT_FAILURE;
    }

    // ------------------------------------------------------------------
    // 4. Create emulator & load ROM
    // ------------------------------------------------------------------
    auto emulator = std::make_unique<snes::core::Emulator>();

    emulator->AttachVideoOutput(video.get());
    emulator->AttachAudioOutput(audio.get());
    emulator->AttachInputProvider(input.get());

    std::string loadError;
    if (!emulator->LoadCartridgeFromFile(romPath, &loadError)) {
        std::cerr << "Failed to load ROM: " << loadError << '\n';
        SDL_Quit();
        return EXIT_FAILURE;
    }

    const auto* cart = emulator->LoadedCartridge();
    if (cart) {
        const auto& hdr = cart->Header();
        bool isHiROM = (hdr.mapping == snes::core::MappingType::HiRom);
        std::cout << "Loaded: " << hdr.title
                  << " (" << (isHiROM ? "HiROM" : "LoROM") << ")\n";
    }

    // ------------------------------------------------------------------
    // 5. Un-pause audio and enter main loop
    // ------------------------------------------------------------------
    audio->Resume();

    // Explicit frame pacing for NTSC (~59.94 Hz) prevents the emulation loop
    // from outrunning real-time audio on systems where VSync is unavailable.
    constexpr uint64_t kFrameNs = (1000000000ull * 1001ull) / 60000ull;
    uint64_t nextFrameTime = SDL_GetTicksNS();

    bool running = true;
    while (running) {
        // --- Event pump (also updates keyboard state for SdlInputProvider) ---
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_EVENT_QUIT:
                    running = false;
                    break;
                case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
                    running = false;
                    break;
                case SDL_EVENT_KEY_DOWN:
                    // Escape quits
                    if (event.key.scancode == SDL_SCANCODE_ESCAPE)
                        running = false;
                    break;
                default:
                    break;
            }
        }

        if (!running) break;

        // --- Run one emulated frame ---
        emulator->StepFrame();

        // Pace to real time to keep audio queue stable.
        nextFrameTime += kFrameNs;
        uint64_t now = SDL_GetTicksNS();
        if (now + (kFrameNs * 4) < nextFrameTime) {
            // Large clock jump/system sleep: resync pacing anchor.
            nextFrameTime = now;
        } else if (now < nextFrameTime) {
            const uint64_t remainingNs = nextFrameTime - now;
            const uint32_t delayMs = static_cast<uint32_t>(remainingNs / 1000000ull);
            if (delayMs > 0) {
                SDL_Delay(delayMs);
            }
        } else if (now - nextFrameTime > kFrameNs) {
            // Running behind: avoid accumulating lag.
            nextFrameTime = now;
        }
    }

    // ------------------------------------------------------------------
    // 6. Cleanup (order matters: destroy SDL objects before SDL_Quit)
    // ------------------------------------------------------------------
    audio->Pause();
    emulator.reset();
    input.reset();
    audio.reset();
    video.reset();
    SDL_Quit();

    return EXIT_SUCCESS;
}
