#include "SdlInputProvider.hpp"

#include <SDL3/SDL.h>
#include <cstdio>
#include <stdexcept>
#include <limits>

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void GunViewport() {
    using snes::frontend::MakeVideoViewport;
    for (unsigned height : {224u, 239u, 448u, 478u}) {
        const auto view = MakeVideoViewport(1000, 800, height);
        const auto top = view.Aim(view.x, view.y);
        const auto center = view.Aim(view.x + view.width / 2, view.y + view.height / 2);
        const auto bottom = view.Aim(view.x + view.width - 0.01f, view.y + view.height - 0.01f);
        Check(!top.offscreen && top.x == 0 && top.y == 0, "Viewport top-left maps to first dot");
        Check(!center.offscreen && center.x == 128 && center.y == int(view.visibleLines / 2),
              "Aiming uses visible rows in both progressive and interlaced modes");
        Check(!bottom.offscreen && bottom.x == 255 && bottom.y == int(view.visibleLines - 1),
              "Viewport bottom-right maps to last dot");
        Check(view.Aim(view.x - 1, view.y).offscreen && view.Aim(view.x, view.y - 1).offscreen &&
              view.Aim(view.x + view.width, view.y).offscreen &&
              view.Aim(view.x, view.y + view.height).offscreen, "Letterbox and excluded edges are offscreen");
        Check(view.Aim(std::numeric_limits<float>::quiet_NaN(), view.y).offscreen,
              "Invalid aim coordinates are offscreen");
    }
    const auto small = MakeVideoViewport(128, 100, 224);
    Check(small.width <= 128 && small.height <= 100 &&
          small.Aim(64, 50).x == 128, "Small windows scale down without cropping aim");
    Check(MakeVideoViewport(0, 0, 224).Aim(0, 0).offscreen, "Empty viewport is offscreen");
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
        GunViewport();
        VirtualPad first, second;
        SDL_SetJoystickVirtualButton(first.joystick, SDL_GAMEPAD_BUTTON_SOUTH, true);
        SDL_SetJoystickVirtualButton(second.joystick, SDL_GAMEPAD_BUTTON_EAST, true);
        SDL_SetJoystickVirtualAxis(second.joystick, SDL_GAMEPAD_AXIS_LEFTX, 20000);
        SDL_UpdateJoysticks();
        SDL_PumpEvents();
        snes::frontend::SdlInputProvider input;
        Check(input.PollLightGun(1, 0, 0).offscreen, "Unfocused gun input is offscreen");
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
