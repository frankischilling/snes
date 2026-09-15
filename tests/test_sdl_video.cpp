#include "SdlVideoOutput.hpp"

#include <cstdio>
#include <memory>
#include <stdexcept>

using snes::core::VideoFrame;
using snes::frontend::SdlVideoOutput;

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

Uint32 ReadPixel(SDL_Renderer* renderer) {
    // These checks deliberately use the dummy/software backend, whose surface
    // persists after presentation. GPU backbuffer retention is not tested here.
    std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface(
        SDL_RenderReadPixels(renderer, nullptr), SDL_DestroySurface);
    Check(surface != nullptr, SDL_GetError());
    Uint8 r = 0, g = 0, b = 0, a = 0;
    Check(SDL_ReadSurfacePixel(surface.get(), surface->w / 2, surface->h / 2, &r, &g, &b, &a), SDL_GetError());
    return Uint32(r) | (Uint32(g) << 8) | (Uint32(b) << 16);
}

void DeferredPresentation() {
    SdlVideoOutput::Config config;
    config.scale = 1;
    config.vsync = false;
    config.deferPresentation = true;
    SdlVideoOutput video(config);
    VideoFrame frame;
    frame.pixels.assign(frame.width * frame.height, 0xff0000ff);
    const auto initial = ReadPixel(video.Renderer());
    video.Present(frame);
    Check(ReadPixel(video.Renderer()) == initial, "Preparing a frame leaves the displayed picture intact");
    video.PresentPending();
    Check(ReadPixel(video.Renderer()) == 0xff, "The scheduled presentation displays the prepared pixels");

    frame.width = 512;
    frame.height = 478;
    frame.pixels.assign(frame.width * frame.height, 0xff00ff00);
    video.Present(frame);
    Check(ReadPixel(video.Renderer()) == 0xff, "Preparing a new resolution does not display it early");
    video.PresentPending();
    Check(ReadPixel(video.Renderer()) == 0xff00, "Deferred presentation handles hires interlaced output");
    video.PresentPending();
    Check(ReadPixel(video.Renderer()) == 0xff00, "Presenting without a new picture keeps the last frame");
}

void ImmediatePresentation() {
    SdlVideoOutput::Config config;
    config.scale = 1;
    config.vsync = false;
    SdlVideoOutput video(config);
    VideoFrame frame;
    frame.pixels.assign(frame.width * frame.height, 0xffff0000);
    video.Present(frame);
    Check(ReadPixel(video.Renderer()) == 0xff0000, "Default output still presents synchronously");
}
}

int main() {
    SDL_SetHint(SDL_HINT_VIDEO_DRIVER, "dummy");
    SDL_SetHint(SDL_HINT_RENDER_DRIVER, "software");
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "%s\n", SDL_GetError());
        return 1;
    }
    int result = 0;
    try {
        DeferredPresentation();
        ImmediatePresentation();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        result = 1;
    }
    SDL_Quit();
    return result;
}
