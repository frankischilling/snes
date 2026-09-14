// snes emulator
// frontend/src/SdlInputProvider.cpp
// SDL keyboard to controller state translation.

// SdlInputProvider.cpp — SDL3-based keyboard input implementation

#include "SdlInputProvider.hpp"

#include <cstdlib>
#include <string>

namespace snes::frontend {

SdlInputProvider::SdlInputProvider() : SdlInputProvider(KeyMap{}) {}

SdlInputProvider::SdlInputProvider(const KeyMap& keyMap)
    : keyMap_(keyMap)
    , scriptedRanges_(ParseScript())
{}

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
