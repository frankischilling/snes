#include "SdlInputProvider.hpp"

#include <SDL3/SDL.h>
#include <cstdio>
#include <stdexcept>

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

struct VirtualPad {
    SDL_JoystickID id = 0;
    SDL_Joystick* joystick = nullptr;
    VirtualPad() {
        SDL_VirtualJoystickDesc desc;
        SDL_INIT_INTERFACE(&desc);
        desc.type = SDL_JOYSTICK_TYPE_GAMEPAD;
        desc.naxes = SDL_GAMEPAD_AXIS_COUNT;
        desc.nbuttons = SDL_GAMEPAD_BUTTON_COUNT;
        desc.axis_mask = (1u << SDL_GAMEPAD_AXIS_COUNT) - 1;
        desc.button_mask = (1u << SDL_GAMEPAD_BUTTON_COUNT) - 1;
        desc.name = "Controller regression device";
        id = SDL_AttachVirtualJoystick(&desc);
        Check(id != 0, "Attach virtual gamepad");
        joystick = SDL_OpenJoystick(id);
        Check(joystick != nullptr, "Open virtual gamepad");
    }
    void Disconnect() {
        if (joystick) SDL_CloseJoystick(joystick);
        joystick = nullptr;
        if (id) SDL_DetachVirtualJoystick(id);
        id = 0;
    }
    ~VirtualPad() { Disconnect(); }
};
}

int main() {
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    if (!SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_EVENTS)) return 1;
    struct Lifetime { ~Lifetime() { SDL_Quit(); } } lifetime;
    try {
        VirtualPad first, second;
        SDL_SetJoystickVirtualButton(first.joystick, SDL_GAMEPAD_BUTTON_SOUTH, true);
        SDL_SetJoystickVirtualButton(second.joystick, SDL_GAMEPAD_BUTTON_EAST, true);
        SDL_SetJoystickVirtualAxis(second.joystick, SDL_GAMEPAD_AXIS_LEFTX, 20000);
        SDL_UpdateJoysticks();
        SDL_PumpEvents();
        snes::frontend::SdlInputProvider input;
        int firstPlayer = -1, secondPlayer = -1;
        for (int player = 0; player < 8; ++player) {
            const auto state = input.PollController(player, 0);
            if (state.b && !state.a) firstPlayer = player;
            if (state.a && state.right && !state.b) secondPlayer = player;
        }
        Check(firstPlayer >= 0 && secondPlayer >= 0 && firstPlayer != secondPlayer,
              "Host gamepads map to independent players");
        SDL_SetJoystickVirtualButton(first.joystick, SDL_GAMEPAD_BUTTON_SOUTH, false);
        SDL_SetJoystickVirtualButton(first.joystick, SDL_GAMEPAD_BUTTON_NORTH, true);
        SDL_SetJoystickVirtualButton(first.joystick, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER, true);
        SDL_SetJoystickVirtualButton(first.joystick, SDL_GAMEPAD_BUTTON_START, true);
        SDL_SetJoystickVirtualButton(first.joystick, SDL_GAMEPAD_BUTTON_BACK, true);
        SDL_UpdateJoysticks();
        SDL_PumpEvents();
        const auto changed = input.PollController(firstPlayer, 1);
        Check(changed.x && changed.l && changed.start && changed.select && !changed.b,
              "SDL face, shoulder, start and select buttons update");
        first.Disconnect();
        SDL_PumpEvents();
        Check(!input.PollController(firstPlayer, 2).x, "Disconnected gamepad releases buttons");
        Check(input.PollController(secondPlayer, 2).a, "Disconnect preserves other player assignments");
        VirtualPad replacement;
        SDL_SetJoystickVirtualButton(replacement.joystick, SDL_GAMEPAD_BUTTON_WEST, true);
        SDL_UpdateJoysticks();
        SDL_PumpEvents();
        Check(input.PollController(firstPlayer, 3).y, "New gamepad takes the vacant player slot");
        Check(!input.PollController(8, 3).a && !input.PollController(-1, 3).a,
              "Out-of-range player input is empty");
        std::puts("SDL controller connection and input checks passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s: %s\n", error.what(), SDL_GetError());
        return 1;
    }
}
