#include "snes/core/Dsp3.hpp"

#include <algorithm>
#include <bit>

namespace snes::core {

void Dsp3::Move(int direction, int16_t& x, int16_t& y, bool wrap) {
    int dx, dy;
    if (wrap) {
        dy = std::bit_cast<int16_t>(DataRom(unsigned(0x3b2 + direction * 2)));
        dx = std::bit_cast<int16_t>(DataRom(unsigned(0x3b3 + direction * 2)));
    } else {
        // The search sequencer widens these byte deltas before adding. Its
        // perimeter is deliberately different from the wrapping transfer walk.
        constexpr uint8_t xDelta[]{0,0,1,1,0,255,255,0};
        constexpr uint8_t yDelta[2][8]{{0,255,0,1,1,1,0,0}, {0,255,255,0,1,0,255,0}};
        dx = xDelta[unsigned(direction) & 7];
        dy = yDelta[x & 1][unsigned(direction) & 7];
    }
    const int oldX = uint8_t(x);
    int nextX = oldX + dx;
    int nextY = uint8_t(y) + dy + ((oldX & 1) ? (dx & 1) : 0);
    if (wrap) {
        if (nextX < 0) nextX += width_;
        else if (nextX >= width_) nextX -= width_;
        if (nextY < 0) nextY += height_;
        else if (nextY >= height_) nextY -= height_;
    }
    x = int16_t(nextX);
    y = int16_t(nextY);
}

void Dsp3::StartRing(bool results) {
    unsigned& previous = results ? returnedRadius_ : searchedRadius_;
    ring_.low = std::max(1u, unsigned(uint8_t(data_)));
    ring_.high = data_ >> 8;
    if (previous >= ring_.low) ring_.low = previous + 1;
    previous = std::max(previous, ring_.high);
    ring_.radius = ring_.steps = ring_.low;
    ring_.side = 0;
    ring_.x = originX_;
    ring_.y = originY_;
    for (unsigned i = 0; i < ring_.radius; ++i) Move(0, ring_.x, ring_.y, true);
    EmitCell(results);
}

bool Dsp3::NextRing() {
    if (!ring_.steps) {
        ++ring_.radius;
        ring_.steps = ring_.radius;
        ring_.x = originX_;
        ring_.y = originY_;
        for (unsigned i = 0; i < ring_.radius; ++i) Move(int(ring_.side), ring_.x, ring_.y, true);
    }
    if (ring_.radius > ring_.high) {
        ++ring_.side;
        ring_.radius = ring_.steps = ring_.low;
        ring_.x = originX_;
        ring_.y = originY_;
        for (unsigned i = 0; i < ring_.radius; ++i) Move(int(ring_.side), ring_.x, ring_.y, true);
    }
    return ring_.side < 6;
}

void Dsp3::EmitCell(bool results) {
    status_ = 0x80;
    if (!NextRing()) {
        data_ = 0xffff;
        phase_ = results ? Phase::Finish : Phase::Solve;
        return;
    }
    cell_ = data_ = Linear(uint8_t(ring_.x), uint8_t(ring_.y));
    phase_ = results ? Phase::ResultCell : Phase::SearchCell;
}

void Dsp3::SolvePaths() {
    int16_t x = originX_, y = originY_;
    const auto inside = [this](int px, int py) {
        return px >= 0 && px < width_ && py >= 0 && py < height_;
    };
    for (unsigned radius = 1; radius < ring_.high; ++radius) {
        --y;
        for (int side : {5,4,3,2,1,6}) {
            for (unsigned step = 0; step < radius; ++step) {
                Move(side, x, y, false);
                if (!inside(x, y)) continue;
                auto& target = cells_[Linear(uint8_t(x), uint8_t(y)) & 8191];
                if (target.cost >= 128 || target.terrain >= 64) continue;
                int16_t best = 255;
                for (int edge = 6; edge >= 1; --edge) {
                    int16_t nx = x, ny = y;
                    Move(edge, nx, ny, false);
                    if (!inside(nx, ny)) continue;
                    const auto& neighbor = cells_[Linear(uint8_t(nx), uint8_t(ny)) & 8191];
                    if (neighbor.terrain < 128 || neighbor.weight == 0)
                        best = std::min(best, neighbor.weight);
                }
                if (best != 255) target.weight = int16_t(best + target.cost);
            }
        }
    }
}

} // namespace snes::core
