// main.cpp — SNES emulator SDL3 frontend
//
// Usage:  snes_frontend <rom_file.smc>
//
// SDL init → load ROM → pace StepFrame() from audio demand → event pump → quit.

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
    // 1. Parse command-line arguments
    if (argc < 2) {
        std::cerr << "Usage: snes_frontend <rom_file.smc>\n";
        return EXIT_FAILURE;
    }
    const std::string romPath = argv[1];

    // 2. Initialise SDL (video + audio + events)
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return EXIT_FAILURE;
    }
    struct SdlLifetime {
        ~SdlLifetime() { SDL_Quit(); }
    } sdlLifetime;

    // 3. Create SDL platform objects
    std::unique_ptr<snes::frontend::SdlVideoOutput> video;
    std::unique_ptr<snes::frontend::SdlAudioOutput> audio;
    std::unique_ptr<snes::frontend::SdlInputProvider> input;

    try {
        snes::frontend::SdlVideoOutput::Config videoConfig;
        // Audio consumption sets emulation speed, independent of monitor refresh.
        videoConfig.vsync = false;
        video = std::make_unique<snes::frontend::SdlVideoOutput>(videoConfig);
        audio = std::make_unique<snes::frontend::SdlAudioOutput>();
        input = std::make_unique<snes::frontend::SdlInputProvider>();
    } catch (const std::exception& e) {
        std::cerr << "Failed to create SDL subsystem: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    // 4. Create emulator & load ROM
    auto emulator = std::make_unique<snes::core::Emulator>();

    emulator->AttachVideoOutput(video.get());
    emulator->AttachAudioOutput(audio.get());
    emulator->AttachInputProvider(input.get());

    std::string loadError;
    if (!emulator->LoadCartridgeFromFile(romPath, &loadError)) {
        std::cerr << "Failed to load ROM: " << loadError << '\n';
        return EXIT_FAILURE;
    }

    const auto* cart = emulator->LoadedCartridge();
    if (cart) {
        const auto& hdr = cart->Header();
        bool isHiROM = (hdr.mapping == snes::core::MappingType::HiRom);
        std::cout << "Loaded: " << hdr.title
                  << " (" << (isHiROM ? "HiROM" : "LoROM") << ")\n";
    }

    // 5. Prime audio and enter main loop
    try {
        audio->Resume();
        bool running = true;
        while (running) {
            // Event pump (also updates keyboard state for SdlInputProvider)
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

            // Wait for the device to consume input instead of dropping samples.
            // Keep polling events during the wait so the window stays responsive.
            if (!audio->NeedsSamples()) {
                SDL_Delay(1);
                continue;
            }
            emulator->StepFrame();
        }
        audio->Pause();
    } catch (const std::exception& e) {
        std::cerr << "Playback failed: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    // 6. Cleanup (order matters: destroy SDL objects before SDL_Quit)
    emulator.reset();
    input.reset();
    audio.reset();
    video.reset();

    return EXIT_SUCCESS;
}
