// snes emulator
// frontend/src/SdlVideoOutput.cpp
// SDL texture upload, scaling, and window presentation.

// SdlVideoOutput.cpp — SDL3-based video output implementation

#include "SdlVideoOutput.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace snes::frontend {

namespace {
int ProbeFrameIndex() {
    static int probeFrame = -2;
    if (probeFrame == -2) {
        const char* v = std::getenv("SNES_PROBE_FRAME");
        probeFrame = (v && *v) ? std::atoi(v) : 500;
    }
    return probeFrame;
}
}

// Construction / Destruction

SdlVideoOutput::SdlVideoOutput() : SdlVideoOutput(Config{}) {}

SdlVideoOutput::SdlVideoOutput(const Config& config) {
    int windowW = config.baseW * config.scale;
    int windowH = config.baseH * config.scale;

    if (!SDL_CreateWindowAndRenderer(
            config.title.c_str(),
            windowW, windowH,
            0,                       // no extra window flags
            &window_, &renderer_))
    {
        throw std::runtime_error(
            std::string("SDL_CreateWindowAndRenderer failed: ") + SDL_GetError());
    }

    if (config.vsync) {
        SDL_SetRenderVSync(renderer_, 1);
    }

    // Clear to black initially
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);
    SDL_RenderPresent(renderer_);
}

SdlVideoOutput::~SdlVideoOutput() {
    if (texture_)  SDL_DestroyTexture(texture_);
    if (renderer_) SDL_DestroyRenderer(renderer_);
    if (window_)   SDL_DestroyWindow(window_);
}

// IVideoOutput::Present

void SdlVideoOutput::Present(const snes::core::VideoFrame& frame) {
    if (!renderer_) return;
    if (frame.pixels.empty()) return;

    EnsureTexture(frame.width, frame.height);
    if (!texture_) return;

    // Upload pixel data.
    // PPU output is ABGR8888 (as a uint32_t: alpha MSB, red LSB).
    // The texture was created with SDL_PIXELFORMAT_ABGR8888 to match.
    int pitch = static_cast<int>(frame.width * sizeof(uint32_t));
    SDL_UpdateTexture(texture_, nullptr, frame.pixels.data(), pitch);

    // Fit the picture to the window, using integer scaling when it fits.
    int winW = 0, winH = 0;
    SDL_GetWindowSize(window_, &winW, &winH);
    const auto viewport = MakeVideoViewport(winW, winH, frame.height);
    SDL_FRect dst{viewport.x, viewport.y, viewport.width, viewport.height};

    // One-shot: log presentation dimensions
    static bool logged = false;
    if (!logged) {
        fprintf(stderr, "[VIDEO] frame=%ux%u window=%dx%d dst=%.0fx%.0f+%.0f+%.0f\n",
                frame.width, frame.height, winW, winH,
                dst.w, dst.h, dst.x, dst.y);
        logged = true;
    }

    // Draw the picture and gun sights over a black letterbox.
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);
    SDL_RenderTexture(renderer_, texture_, nullptr, &dst);
    for (unsigned gun = 0; gun < std::min(gunCount_, 2u); ++gun) {
        const auto& aim = gunAim_[gun];
        if (aim.offscreen || viewport.visibleLines == 0) continue;
        const float x = dst.x + (aim.x + 0.5f) * dst.w / 256;
        const float y = dst.y + (aim.y + 0.5f) * dst.h / viewport.visibleLines;
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
        const SDL_FRect horizontal{x - 8, y - 1, 17, 3}, vertical{x - 1, y - 8, 3, 17};
        SDL_RenderFillRect(renderer_, &horizontal); SDL_RenderFillRect(renderer_, &vertical);
        SDL_SetRenderDrawColor(renderer_, gun ? 255 : 0, gun ? 0 : 255, gun ? 255 : 0, 255);
        SDL_RenderLine(renderer_, x - 7, y, x + 7, y);
        SDL_RenderLine(renderer_, x, y - 7, x, y + 7);
    }
    SDL_RenderPresent(renderer_);

    // Save a probe frame as BMP for diagnostic comparison.
    static int frameCounter = 0;
    const int probeFrame = ProbeFrameIndex();
    if (probeFrame >= 0 && frameCounter == probeFrame) {
        SDL_Surface* surf = SDL_CreateSurface(
            static_cast<int>(frame.width), static_cast<int>(frame.height),
            SDL_PIXELFORMAT_ABGR8888);
        if (surf) {
            char filename[32];
            std::snprintf(filename, sizeof(filename), "frame%d.bmp", probeFrame);
            memcpy(surf->pixels, frame.pixels.data(),
                   frame.width * frame.height * sizeof(uint32_t));
            SDL_SaveBMP(surf, filename);
            SDL_DestroySurface(surf);
            fprintf(stderr, "[VIDEO] Saved %s (%ux%u)\n",
                    filename, frame.width, frame.height);
        }
    }
    frameCounter++;
}

// WindowID

uint32_t SdlVideoOutput::WindowID() const noexcept {
    return window_ ? SDL_GetWindowID(window_) : 0;
}

VideoViewport SdlVideoOutput::InputViewport() const {
    int width = 0, height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    return MakeVideoViewport(width, height, texH_ ? texH_ : 224);
}

// EnsureTexture — (re)create texture when frame dimensions change

void SdlVideoOutput::EnsureTexture(uint32_t w, uint32_t h) {
    if (texture_ && texW_ == w && texH_ == h) return;

    if (texture_) {
        SDL_DestroyTexture(texture_);
        texture_ = nullptr;
    }

    // PPU pixel layout: uint32_t = (0xFF << 24) | (B << 16) | (G << 8) | R
    // This is SDL_PIXELFORMAT_ABGR8888 (named MSB→LSB in the uint32_t).
    texture_ = SDL_CreateTexture(
        renderer_,
        SDL_PIXELFORMAT_ABGR8888,
        SDL_TEXTUREACCESS_STREAMING,
        static_cast<int>(w),
        static_cast<int>(h));

    if (texture_) {
        texW_ = w;
        texH_ = h;
        // Use nearest-neighbour scaling for crisp pixels
        SDL_SetTextureScaleMode(texture_, SDL_SCALEMODE_NEAREST);
    } else {
        texW_ = texH_ = 0;
    }
}

} // namespace snes::frontend
