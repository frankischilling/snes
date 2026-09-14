#pragma once
// ============================================================================
// SdlVideoOutput.hpp — SDL3-based video output for the SNES emulator
//
// Creates an SDL window + renderer + streaming texture.  Each call to
// Present() uploads the PPU framebuffer and draws it scaled to the window.
//
// The visible SNES frame is 256×224 RGBA8888 pixels (non-overscan).
// The window defaults to 3× the base resolution (256×224) = 768×672.
// The texture always matches the incoming VideoFrame dimensions so
// that varying frame sizes (e.g. overscan 256×239) are handled.
//
// SDL3 API (not SDL2) — uses SDL_CreateWindow / SDL_CreateRenderer /
// SDL_CreateTexture / SDL_UpdateTexture / SDL_RenderTexture.
// ============================================================================

#include "snes/core/Platform.hpp"

#include <SDL3/SDL.h>
#include <cstdint>
#include <string>

namespace snes::frontend {

class SdlVideoOutput final : public snes::core::IVideoOutput {
public:
    /// Configuration for window creation.
    struct Config {
        std::string title   = "SNES Emulator";
        int         scale   = 3;           // Window = base × scale
        int         baseW   = 256;         // Base width  (SNES visible)
        int         baseH   = 224;         // Base height (non-overscan)
        bool        vsync   = true;
    };

    explicit SdlVideoOutput();
    explicit SdlVideoOutput(const Config& config);
    ~SdlVideoOutput() override;

    // Non-copyable / non-movable (owns SDL resources)
    SdlVideoOutput(const SdlVideoOutput&) = delete;
    SdlVideoOutput& operator=(const SdlVideoOutput&) = delete;

    // IVideoOutput
    void Present(const snes::core::VideoFrame& frame) override;

    // -----------------------------------------------------------------------
    // State queries
    // -----------------------------------------------------------------------

    /// Returns true if Init() succeeded (window + renderer created).
    bool IsValid() const noexcept { return renderer_ != nullptr; }

    /// Access the underlying SDL window (e.g. for event handling).
    SDL_Window*   Window()   const noexcept { return window_; }
    SDL_Renderer* Renderer() const noexcept { return renderer_; }

    /// Returns the SDL window ID (for matching SDL_WindowEvent).
    uint32_t WindowID() const noexcept;

private:
    /// (Re-)create the streaming texture to match the given dimensions.
    void EnsureTexture(uint32_t w, uint32_t h);

    SDL_Window*   window_   = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    SDL_Texture*  texture_  = nullptr;

    // Current texture dimensions (recreated if frame size changes)
    uint32_t texW_ = 0;
    uint32_t texH_ = 0;
};

} // namespace snes::frontend
