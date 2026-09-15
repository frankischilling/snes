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
#include "FramePacer.hpp"

#include "snes/core/Emulator.hpp"
#include "snes/core/Logging.hpp"

#include <SDL3/SDL.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <vector>

int main(int argc, char* argv[]) {
    // 1. Parse command-line arguments
    using snes::core::ControllerDevice;
    std::array devices{ControllerDevice::Gamepad, ControllerDevice::Gamepad};
    std::vector<std::string> arguments;
    for (int i = 1; i < argc; ++i) {
        const std::string_view value(argv[i]);
        if (value.starts_with("--port1=") || value.starts_with("--port2=")) {
            auto& device = devices[value[6] - '1'];
            const auto name = value.substr(8);
            if (name == "pad") device = ControllerDevice::Gamepad;
            else if (name == "mouse") device = ControllerDevice::Mouse;
            else if (name == "multitap") device = ControllerDevice::Multitap;
            else if (name == "none") device = ControllerDevice::None;
            else if (name == "scope") device = ControllerDevice::SuperScope;
            else if (name == "justifier") device = ControllerDevice::Justifier;
            else if (name == "justifiers") device = ControllerDevice::Justifiers;
            else if (name == "rifle") device = ControllerDevice::MacsRifle;
            else {
                std::cerr << "Unknown controller device: " << name << '\n';
                return EXIT_FAILURE;
            }
        } else arguments.emplace_back(value);
    }
    const bool sufami = !arguments.empty() && arguments[0] == "--sufami";
    const bool broadcast = !arguments.empty() && arguments[0] == "--broadcast";
    if ((!sufami && !broadcast && arguments.size() != 1) ||
        (sufami && (arguments.size() < 3 || arguments.size() > 4)) ||
        (broadcast && arguments.size() != 3)) {
        std::cerr << "Usage: snes_frontend [--port1=pad|mouse|multitap|none] [--port2=...] <rom_file.smc>\n"
                  << "       snes_frontend [controller options] --sufami <bios.bin> <slot-a.st|-> [slot-b.st|-]\n"
                  << "       snes_frontend [controller options] --broadcast <base.sfc> <pack.bs|->\n";
        std::cerr << "       Port 2 also supports scope, justifier, justifiers, and rifle\n";
        return EXIT_FAILURE;
    }
    if (devices[0] == ControllerDevice::Mouse && devices[1] == ControllerDevice::Mouse) {
        std::cerr << "The frontend supports one host mouse; choose one console port\n";
        return EXIT_FAILURE;
    }
    if (snes::core::IsLightGun(devices[0])) {
        std::cerr << "Connect light guns to port 2 for beam-counter input\n";
        return EXIT_FAILURE;
    }
    if (devices[0] == ControllerDevice::Mouse && snes::core::IsLightGun(devices[1])) {
        std::cerr << "The host pointer cannot control a relative mouse and a light gun together\n";
        return EXIT_FAILURE;
    }
    const std::string romPath = arguments[sufami || broadcast ? 1 : 0];
    const std::array<std::string, 2> slotPaths{sufami ? arguments[2] : "",
                                             sufami && arguments.size() == 4 ? arguments[3] : ""};

    // 2. Initialise SDL (video + audio + events)
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMEPAD)) {
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
        // Console clocks pace presentation, with gradual audio queue feedback.
        videoConfig.vsync = false;
        videoConfig.deferPresentation = true;
        video = std::make_unique<snes::frontend::SdlVideoOutput>(videoConfig);
        audio = std::make_unique<snes::frontend::SdlAudioOutput>();
        input = std::make_unique<snes::frontend::SdlInputProvider>();
        if ((devices[0] == ControllerDevice::Mouse || devices[1] == ControllerDevice::Mouse) &&
            !SDL_SetWindowRelativeMouseMode(video->Window(), true))
            throw std::runtime_error(std::string("Cannot capture mouse: ") + SDL_GetError());
    } catch (const std::exception& e) {
        std::cerr << "Failed to create SDL subsystem: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    // 4. Create emulator & load ROM
    auto emulator = std::make_unique<snes::core::Emulator>();

    emulator->AttachVideoOutput(video.get());
    emulator->AttachAudioOutput(audio.get());
    emulator->AttachInputProvider(input.get());
    int nextPlayer = 0;
    for (unsigned port = 0; port < devices.size(); ++port) {
        std::array<int, 4> players{-1, -1, -1, -1};
        const int count = devices[port] == ControllerDevice::Multitap ? 4 : 1;
        for (int i = 0; i < count; ++i) players[i] = nextPlayer++;
        emulator->GetAutoJoypad().Ports().Configure(port, devices[port], players);
    }

    std::string loadError;
    const bool loaded = sufami ? emulator->LoadSufamiTurboFromFiles(romPath, slotPaths[0], slotPaths[1], &loadError) :
                        broadcast ? emulator->LoadBroadcastCartridgeFromFiles(romPath, arguments[2], &loadError) :
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
    auto packPath = std::filesystem::path(broadcast && arguments[2] != "-" ? arguments[2] : romPath);
    packPath += ".flash";
    snes::frontend::SaveRamFile savePack(packPath);
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
            const auto pack = savePack.Load(cart->MemoryPackData().size());
            if (!pack.empty() && !emulator->LoadMemoryPack(pack))
                throw std::runtime_error("Invalid memory pack save: " + packPath.string());
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
            savePack.Flush(cart->MemoryPackData());
        }
    };

    // 5. Prime audio and enter main loop
    try {
        audio->Resume();
        bool running = true;
        bool picturePending = false;
        uint64_t presentationTime = 0;
        snes::frontend::FramePacer pacer;
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

            if (picturePending) {
                const auto now = SDL_GetTicksNS();
                if (now < presentationTime) {
                    SDL_DelayPrecise(std::min<uint64_t>(presentationTime - now, 2'000'000));
                    continue;
                }
                video->PresentPending();
                pacer.Presented(SDL_GetTicksNS());
                picturePending = false;
                continue;
            }

            // Preserve all samples after unusually long transfers or a device
            // stall. Normal playback has enough headroom for batched callbacks.
            if (audio->IsPlaying() && audio->QueuedMilliseconds() >= 80.0) {
                pacer.Reset();
                SDL_Delay(1);
                continue;
            }
            if (snes::core::IsLightGun(devices[1])) {
                input->SetGunViewport(video->InputViewport(), (SDL_GetWindowFlags(video->Window()) & SDL_WINDOW_INPUT_FOCUS) != 0);
                const unsigned count = devices[1] == ControllerDevice::Justifiers ? 2 : 1;
                std::array<snes::core::LightGunState, 2> aim;
                for (unsigned gun = 0; gun < count; ++gun)
                    aim[gun] = input->PollLightGun(1, gun, emulator->CurrentFrame());
                video->SetGunAim(aim, count);
            }
            const auto before = emulator->CurrentMasterCycles();
            const auto result = emulator->StepFrame();
            if (audio->IsPlaying()) {
                presentationTime = pacer.Schedule(SDL_GetTicksNS(), result.masterCycles - before,
                    emulator->GetTiming().MasterClockHz(), audio->QueuedMilliseconds());
                picturePending = true;
            } else {
                // Prime/refill audio promptly; the last prepared picture is
                // presented when playback starts instead of flashing each one.
                pacer.Reset();
            }
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
