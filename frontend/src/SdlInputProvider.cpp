// snes emulator
// frontend/src/SdlInputProvider.cpp
// SDL keyboard to controller state translation.

// SdlInputProvider.cpp — SDL3-based keyboard input implementation

#include "SdlInputProvider.hpp"

#include <cstdlib>
#include <string>
#include <algorithm>
#include <cmath>

namespace snes::frontend {

SdlInputProvider::SdlInputProvider() : SdlInputProvider(KeyMap{}) {}

SdlInputProvider::SdlInputProvider(const KeyMap& keyMap)
    : keyMap_(keyMap)
    , scriptedRanges_(ParseScript())
{}

SdlInputProvider::~SdlInputProvider() {
    for (auto* gamepad : gamepads_) if (gamepad) SDL_CloseGamepad(gamepad);
}

void SdlInputProvider::RefreshGamepads(uint64_t frameIndex) {
    if (lastDeviceFrame_ == frameIndex) return;
    lastDeviceFrame_ = frameIndex;
    for (auto& gamepad : gamepads_) {
        if (gamepad && !SDL_GamepadConnected(gamepad)) {
            SDL_CloseGamepad(gamepad);
            gamepad = nullptr;
        }
    }
    int count = 0;
    auto* ids = SDL_GetGamepads(&count);
    for (int i = 0; ids && i < count; ++i) {
        const auto known = std::find_if(gamepads_.begin(), gamepads_.end(),
            [&](SDL_Gamepad* pad) { return pad && SDL_GetGamepadID(pad) == ids[i]; });
        if (known != gamepads_.end()) continue;
        auto empty = std::find(gamepads_.begin(), gamepads_.end(), nullptr);
        if (empty == gamepads_.end()) break;
        *empty = SDL_OpenGamepad(ids[i]);
    }
    SDL_free(ids);
}

snes::core::InputState SdlInputProvider::PollController(int player, uint64_t frameIndex) {
    if (player < 0 || player >= static_cast<int>(gamepads_.size())) return {};
    RefreshGamepads(frameIndex);
    auto state = player == 0 ? Poll(frameIndex) : snes::core::InputState{};
    auto* pad = gamepads_[player];
    if (!pad) return state;
    const auto button = [&](SDL_GamepadButton id) { return SDL_GetGamepadButton(pad, id); };
    const auto x = SDL_GetGamepadAxis(pad, SDL_GAMEPAD_AXIS_LEFTX);
    const auto y = SDL_GetGamepadAxis(pad, SDL_GAMEPAD_AXIS_LEFTY);
    state.up |= button(SDL_GAMEPAD_BUTTON_DPAD_UP) || y < -16000;
    state.down |= button(SDL_GAMEPAD_BUTTON_DPAD_DOWN) || y > 16000;
    state.left |= button(SDL_GAMEPAD_BUTTON_DPAD_LEFT) || x < -16000;
    state.right |= button(SDL_GAMEPAD_BUTTON_DPAD_RIGHT) || x > 16000;
    state.b |= button(SDL_GAMEPAD_BUTTON_SOUTH);
    state.a |= button(SDL_GAMEPAD_BUTTON_EAST);
    state.y |= button(SDL_GAMEPAD_BUTTON_WEST);
    state.x |= button(SDL_GAMEPAD_BUTTON_NORTH);
    state.l |= button(SDL_GAMEPAD_BUTTON_LEFT_SHOULDER);
    state.r |= button(SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER);
    state.start |= button(SDL_GAMEPAD_BUTTON_START);
    state.select |= button(SDL_GAMEPAD_BUTTON_BACK);
    return state;
}

snes::core::MouseState SdlInputProvider::PollMouse(int /*port*/, uint64_t /*frameIndex*/) {
    float x = 0, y = 0;
    const auto buttons = SDL_GetRelativeMouseState(&x, &y);
    mouseFractionX_ += x;
    mouseFractionY_ += y;
    // Bound conversions even if a backend reports an extreme motion event.
    const auto dx = static_cast<int32_t>(std::clamp(std::trunc(mouseFractionX_), -1000000.0f, 1000000.0f));
    const auto dy = static_cast<int32_t>(std::clamp(std::trunc(mouseFractionY_), -1000000.0f, 1000000.0f));
    mouseFractionX_ -= dx;
    mouseFractionY_ -= dy;
    return {dx, dy, (buttons & SDL_BUTTON_LMASK) != 0, (buttons & SDL_BUTTON_RMASK) != 0};
}

snes::core::InputState SdlInputProvider::Poll(uint64_t frameIndex) {
    // SDL_GetKeyboardState returns a pointer to an internal array indexed by
    // SDL_Scancode.  The array is updated by SDL_PumpEvents(), which the main
    // loop must call before Poll().
    int numKeys = 0;
    const bool* keys = SDL_GetKeyboardState(&numKeys);
    if (!keys) return {};

    // Helper: safely check a scancode against the array bounds.
    auto pressed = [&](SDL_Scancode sc) -> bool {
        return static_cast<int>(sc) < numKeys && keys[sc];
    };

    snes::core::InputState state{};
    state.up     = pressed(keyMap_.up);
    state.down   = pressed(keyMap_.down);
    state.left   = pressed(keyMap_.left);
    state.right  = pressed(keyMap_.right);
    state.a      = pressed(keyMap_.a);
    state.b      = pressed(keyMap_.b);
    state.x      = pressed(keyMap_.x);
    state.y      = pressed(keyMap_.y);
    state.l      = pressed(keyMap_.l);
    state.r      = pressed(keyMap_.r);
    state.start  = pressed(keyMap_.start);
    state.select = pressed(keyMap_.select);

    state.up     = state.up     || ScriptPressed(Button::Up, frameIndex);
    state.down   = state.down   || ScriptPressed(Button::Down, frameIndex);
    state.left   = state.left   || ScriptPressed(Button::Left, frameIndex);
    state.right  = state.right  || ScriptPressed(Button::Right, frameIndex);
    state.a      = state.a      || ScriptPressed(Button::A, frameIndex);
    state.b      = state.b      || ScriptPressed(Button::B, frameIndex);
    state.x      = state.x      || ScriptPressed(Button::X, frameIndex);
    state.y      = state.y      || ScriptPressed(Button::Y, frameIndex);
    state.l      = state.l      || ScriptPressed(Button::L, frameIndex);
    state.r      = state.r      || ScriptPressed(Button::R, frameIndex);
    state.start  = state.start  || ScriptPressed(Button::Start, frameIndex);
    state.select = state.select || ScriptPressed(Button::Select, frameIndex);

    return state;
}

SdlInputProvider::ScriptRanges SdlInputProvider::ParseScript() {
    ScriptRanges ranges;

    const char* raw = std::getenv("SNES_INPUT_SCRIPT");
    if (!raw || !*raw) {
        return ranges;
    }

    auto parseButton = [](const std::string& name) -> Button {
        if (name == "up") return Button::Up;
        if (name == "down") return Button::Down;
        if (name == "left") return Button::Left;
        if (name == "right") return Button::Right;
        if (name == "a") return Button::A;
        if (name == "b") return Button::B;
        if (name == "x") return Button::X;
        if (name == "y") return Button::Y;
        if (name == "l") return Button::L;
        if (name == "r") return Button::R;
        if (name == "start") return Button::Start;
        if (name == "select") return Button::Select;
        return Button::Count;
    };

    std::string script(raw);
    size_t cursor = 0;
    while (cursor < script.size()) {
        size_t next = script.find(',', cursor);
        std::string token = script.substr(cursor, next == std::string::npos
                                                     ? std::string::npos
                                                     : next - cursor);
        cursor = (next == std::string::npos) ? script.size() : next + 1;
        if (token.empty()) continue;

        size_t at = token.find('@');
        size_t dash = token.find('-', at == std::string::npos ? 0 : at + 1);
        if (at == std::string::npos || dash == std::string::npos) continue;

        Button button = parseButton(token.substr(0, at));
        if (button == Button::Count) continue;

        try {
            uint64_t first = std::stoull(token.substr(at + 1, dash - at - 1));
            uint64_t last = std::stoull(token.substr(dash + 1));
            if (last < first) continue;
            ranges[static_cast<size_t>(button)].push_back({first, last});
        } catch (...) {
            continue;
        }
    }

    return ranges;
}

bool SdlInputProvider::ScriptPressed(Button button, uint64_t frameIndex) const noexcept {
    const auto& ranges = scriptedRanges_[static_cast<size_t>(button)];
    for (const auto& range : ranges) {
        if (frameIndex >= range.first && frameIndex <= range.last) {
            return true;
        }
    }
    return false;
}

} // namespace snes::frontend
