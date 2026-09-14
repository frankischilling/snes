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

    // Compute integer-scaled destination rect, centered in window.
    int winW = 0, winH = 0;
    SDL_GetWindowSize(window_, &winW, &winH);
    int scaleX = winW / static_cast<int>(frame.width);
    int scaleY = winH / static_cast<int>(frame.height);
    int scale  = (scaleX < scaleY) ? scaleX : scaleY;
    if (scale < 1) scale = 1;
    int dstW = static_cast<int>(frame.width)  * scale;
    int dstH = static_cast<int>(frame.height) * scale;
    SDL_FRect dst;
    dst.x = static_cast<float>((winW - dstW) / 2);
    dst.y = static_cast<float>((winH - dstH) / 2);
    dst.w = static_cast<float>(dstW);
    dst.h = static_cast<float>(dstH);

    // One-shot: log presentation dimensions
    static bool logged = false;
    if (!logged) {
        fprintf(stderr, "[VIDEO] frame=%ux%u window=%dx%d scale=%d dst=%.0fx%.0f+%.0f+%.0f\n",
                frame.width, frame.height, winW, winH, scale,
                dst.w, dst.h, dst.x, dst.y);
        logged = true;
    }

    // Draw — black letterbox + integer-scaled texture
    SDL_RenderClear(renderer_);
    SDL_RenderTexture(renderer_, texture_, nullptr, &dst);
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
