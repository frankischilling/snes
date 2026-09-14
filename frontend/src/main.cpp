// snes emulator
// frontend/src/main.cpp
// SDL frontend entry point and save-file lifecycle.

// main.cpp — SNES emulator SDL3 frontend
//
// Usage:  snes_frontend <rom_file.smc>
//
// SDL init → load ROM → pace StepFrame() from audio demand → event pump → quit.

#include "SdlVideoOutput.hpp"
#include "SdlAudioOutput.hpp"
#include "SdlInputProvider.hpp"
#include "SaveRamFile.hpp"

#include "snes/core/Emulator.hpp"
#include "snes/core/Logging.hpp"

#include <SDL3/SDL.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <array>
#include <stdexcept>

int main(int argc, char* argv[]) {
    // 1. Parse command-line arguments
    const bool sufami = argc >= 2 && std::string_view(argv[1]) == "--sufami";
    if ((!sufami && argc != 2) || (sufami && (argc < 4 || argc > 5))) {
        std::cerr << "Usage: snes_frontend <rom_file.smc>\n"
                  << "       snes_frontend --sufami <bios.bin> <slot-a.st|-> [slot-b.st|-]\n";
        return EXIT_FAILURE;
    }
    const std::string romPath = argv[sufami ? 2 : 1];
    const std::array<std::string, 2> slotPaths{sufami ? argv[3] : "", sufami && argc == 5 ? argv[4] : ""};

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
    const bool loaded = sufami ? emulator->LoadSufamiTurboFromFiles(romPath, slotPaths[0], slotPaths[1], &loadError) :
                                emulator->LoadCartridgeFromFile(romPath, &loadError);
    if (!loaded) {
        std::cerr << "Failed to load ROM: " << loadError << '\n';
        return EXIT_FAILURE;
    }

    const auto* cart = emulator->LoadedCartridge();
    auto savePath = std::filesystem::path(romPath);
    savePath.replace_extension(".srm");
    snes::frontend::SaveRamFile saveRam(savePath);
    auto rtcPath = std::filesystem::path(romPath);
    rtcPath.replace_extension(".rtc");
    snes::frontend::SaveRamFile saveRtc(rtcPath);
    std::array<std::unique_ptr<snes::frontend::SaveRamFile>, 2> slotSaves;
    try {
        if (sufami) {
            std::array<std::filesystem::path, 2> paths;
            for (unsigned i = 0; i < 2; ++i) {
                if (cart->SlotSramData(i).empty()) continue;
                paths[i] = std::filesystem::weakly_canonical(slotPaths[i]);
                paths[i].replace_extension(".srm");
            }
            if (!paths[0].empty() && paths[0] == paths[1])
                throw std::runtime_error("Both slots would write the same save file; use separate game paths");
            for (unsigned i = 0; i < 2; ++i) {
                if (paths[i].empty()) continue;
                slotSaves[i] = std::make_unique<snes::frontend::SaveRamFile>(paths[i]);
                emulator->LoadSlotSram(i, slotSaves[i]->Load(cart->SlotSramData(i).size()));
            }
        } else {
            emulator->LoadSram(saveRam.Load(cart->SramData().size()));
            const auto rtc = saveRtc.Load(emulator->SaveRtc().size());
            if (!rtc.empty() && !emulator->LoadRtc(rtc))
                throw std::runtime_error("Invalid cartridge clock save: " + rtcPath.string());
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to load save RAM: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
    if (cart) {
        const auto& hdr = cart->Header();
        std::cout << "Loaded: " << hdr.title
                  << " (" << snes::core::MappingName(hdr.mapping) << ")\n";
    }

    const auto flushSaves = [&] {
        if (sufami) {
            for (unsigned i = 0; i < 2; ++i)
                if (slotSaves[i]) slotSaves[i]->Flush(cart->SlotSramData(i));
        } else {
            saveRam.Flush(cart->SramData());
            saveRtc.Flush(emulator->SaveRtc());
        }
    };

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
            if (emulator->CurrentFrame() % 300 == 0) flushSaves();
        }
        audio->Pause();
        flushSaves();
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
