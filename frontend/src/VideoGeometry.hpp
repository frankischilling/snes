#pragma once

#include "snes/core/Platform.hpp"
#include <algorithm>
#include <cmath>

namespace snes::frontend {

struct VideoViewport {
    float x = 0, y = 0, width = 0, height = 0;
    unsigned visibleLines = 224;

    snes::core::LightGunState Aim(float windowX, float windowY) const {
        snes::core::LightGunState state;
        if (!std::isfinite(windowX) || !std::isfinite(windowY) || width <= 0 || height <= 0 ||
            windowX < x || windowY < y || windowX >= x + width || windowY >= y + height) return state;
        state.x = std::clamp(int((windowX - x) * 256 / width), 0, 255);
        state.y = std::clamp(int((windowY - y) * visibleLines / height), 0, int(visibleLines) - 1);
        state.offscreen = false;
        return state;
    }
};

inline VideoViewport MakeVideoViewport(int windowWidth, int windowHeight, unsigned frameHeight) {
    VideoViewport viewport;
    viewport.visibleLines = frameHeight > 240 ? frameHeight / 2 : frameHeight;
    if (!viewport.visibleLines || windowWidth <= 0 || windowHeight <= 0) return viewport;
    float scale = std::min(windowWidth / 256.0f, windowHeight / float(viewport.visibleLines));
    if (scale >= 1) scale = std::floor(scale);
    viewport.width = 256 * scale;
    viewport.height = viewport.visibleLines * scale;
    viewport.x = (windowWidth - viewport.width) / 2;
    viewport.y = (windowHeight - viewport.height) / 2;
    return viewport;
}

} // namespace snes::frontend
