// snes emulator
// frontend/src/SdlInputProvider.hpp
// SDL input provider interface.

#pragma once
// SdlInputProvider.hpp — SDL3-based keyboard input for the SNES emulator
//
// Reads the SDL keyboard state array each frame and maps scancodes to
// SNES controller buttons.  Default layout (easily reconfigurable):
//
//   D-pad    → Arrow keys
//   A        → X          B → Z
//   X        → S          Y → A
//   L        → Q          R → W
//   Start    → Enter      Select → Right Shift
//
// Call SDL_PumpEvents() before Poll() so the keyboard state is current.

#include "snes/core/Platform.hpp"

#include <SDL3/SDL.h>
#include <array>
#include <vector>

namespace snes::frontend {

class SdlInputProvider final : public snes::core::IInputProvider {
public:
    /// Per-button scancode bindings (all configurable).
    struct KeyMap {
        SDL_Scancode up     = SDL_SCANCODE_UP;
        SDL_Scancode down   = SDL_SCANCODE_DOWN;
        SDL_Scancode left   = SDL_SCANCODE_LEFT;
        SDL_Scancode right  = SDL_SCANCODE_RIGHT;
        SDL_Scancode a      = SDL_SCANCODE_X;
        SDL_Scancode b      = SDL_SCANCODE_Z;
        SDL_Scancode x      = SDL_SCANCODE_S;
        SDL_Scancode y      = SDL_SCANCODE_A;
        SDL_Scancode l      = SDL_SCANCODE_Q;
        SDL_Scancode r      = SDL_SCANCODE_W;
        SDL_Scancode start  = SDL_SCANCODE_RETURN;
        SDL_Scancode select = SDL_SCANCODE_RSHIFT;
    };

    explicit SdlInputProvider();
    explicit SdlInputProvider(const KeyMap& keyMap);
    ~SdlInputProvider() override = default;

    // IInputProvider
    snes::core::InputState Poll(uint64_t frameIndex) override;

    /// Replace the current key mapping at runtime.
    void SetKeyMap(const KeyMap& keyMap) noexcept { keyMap_ = keyMap; }

    /// Access the current key mapping.
    const KeyMap& GetKeyMap() const noexcept { return keyMap_; }

private:
    struct FrameRange {
        uint64_t first = 0;
        uint64_t last = 0;
    };

    enum class Button : size_t {
        Up,
        Down,
        Left,
        Right,
        A,
        B,
        X,
        Y,
        L,
        R,
        Start,
        Select,
        Count,
    };

    using ScriptRanges = std::array<std::vector<FrameRange>, static_cast<size_t>(Button::Count)>;

    static ScriptRanges ParseScript();
    bool ScriptPressed(Button button, uint64_t frameIndex) const noexcept;

    KeyMap keyMap_;
    ScriptRanges scriptedRanges_;
};

} // namespace snes::frontend
