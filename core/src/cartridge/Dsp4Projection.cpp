#include "snes/core/Dsp4.hpp"

#include <algorithm>

namespace snes::core {

void Dsp4::RoadCommand() {
    const bool guide = command_ == 7 || command_ == 0x10;
    const bool lit = command_ == 0x0f || command_ == 0x10;
    switch (stage_) {
    case Stage::Start: {
        if (lit) Word();
        road_.y = Long();
        auto& edge = polygons_[0].edges[0];
        edge.bottom = Word();
        edge.top = Word();
        polygons_[1].edges[0].center = Word();
        viewport_.bottom = Word();
        road_.x = Long();
        edge.center = Word();
        edge.pointer = Word();
        road_.vertical = Word();
        if (guide) {
            distance_ = Word();
            road_.next.y = Word();
            road_.delta.y = Project(Word(), distance_);
            road_.next.x = Word();
            road_.delta.x = Project(Word(), distance_);
        } else {
            road_.dy = Long();
            road_.dx = Long();
            distance_ = Word();
            Word();
            road_.envelope = command_ == 0x0d ? int32_t(Word()) * 256 : Long();
            road_.ddy = Word();
            road_.ddx = Word();
            if (command_ != 0x0d) road_.turn = road_.turnDelta = 0;
        }
        road_.verticalEnvelope = Word();
        road_.previous.x = Wrap16((guide ? road_.x : Wrap32(int64_t(road_.x) + road_.envelope)) >> 16);
        road_.previous.y = Wrap16(road_.y >> 16);
        road_.previousScroll = {Wrap16(road_.x >> 16), road_.vertical};
        edge.raster = edge.bottom;
        ProjectRoad();
        return;
    }
    case Stage::RoadDistance:
        distance_ = Word();
        if (distance_ == -32768) { Finish(); return; }
        if ((command_ == 1 || command_ == 0x0f) && uint16_t(distance_) == 0x8001)
            Need(6, Stage::RoadSplice);
        else Need(guide ? 10 : 6, Stage::RoadUpdate);
        return;
    case Stage::RoadSplice: {
        distance_ = Word();
        road_.turn = Word();
        road_.turnDelta = Word();
        const auto displacement = Project(road_.turn, distance_);
        road_.previous.x = Wrap16(road_.previous.x + displacement);
        road_.previousScroll.x = Wrap16(road_.previousScroll.x + displacement);
        road_.turn = Wrap16(road_.turn + road_.turnDelta);
        Need(2, Stage::RoadDistance);
        return;
    }
    case Stage::RoadUpdate:
        if (guide) {
            road_.next.y = Word();
            road_.delta.y = Project(Word(), distance_);
            road_.next.x = Word();
            road_.delta.x = Project(Word(), distance_);
            if (command_ == 7) road_.verticalEnvelope = Word();
        } else {
            road_.ddy = Word();
            road_.ddx = Word();
            road_.verticalEnvelope = Word();
            road_.envelope = 0;
        }
        ProjectRoad();
        return;
    case Stage::Lighting: {
        const int16_t intensity = Word();
        const uint16_t color = uint16_t(Word());
        unsigned shaded = 0;
        for (unsigned shift = 0; shift < 15; shift += 5)
            shaded |= unsigned((int32_t((color >> shift) & 31) * intensity >> 15) & 31) << shift;
        ClearOutput();
        WordOut(int(shaded));
        if (++road_.light < 4) Need(4, Stage::Lighting);
        else { RasterRoad(); AdvanceRoad(); }
        return;
    }
    default: Finish(); return;
    }
}

void Dsp4::ProjectRoad() {
    const bool guide = command_ == 7 || command_ == 0x10;
    auto& edge = polygons_[0].edges[0];
    ClearOutput();
    if (guide) {
        road_.next.x = Wrap16(road_.next.x + road_.delta.x);
        road_.next.y = Wrap16(road_.next.y + road_.delta.y);
    } else {
        const int32_t horizontal = Wrap32(int64_t(road_.x) + road_.envelope) >> 16;
        road_.next.x = Project(horizontal, distance_);
        if (command_ != 0x0f) road_.next.x = Wrap16(road_.next.x + Project(road_.turn, distance_));
        road_.next.y = Project(road_.y >> 16, distance_);
        WordOut(horizontal);
    }
    road_.nextScroll = {road_.next.x,
        Wrap16(Project(road_.vertical, distance_) + edge.bottom - road_.next.y)};
    WordOut(road_.next.x);
    if (!guide) WordOut(road_.y >> 16);
    WordOut(road_.next.y);
    const int baseline = command_ == 1 || command_ == 0x0f ? edge.raster : road_.previous.y;
    road_.lines = Wrap16(baseline - road_.next.y);
    if (road_.next.y >= edge.raster) road_.lines = 0;
    else edge.raster = road_.next.y;
    if (road_.next.y < edge.top)
        road_.lines = road_.previous.y >= edge.top ? Wrap16(road_.previous.y - edge.top) : 0;
    WordOut(road_.lines);
    if (road_.lines && (command_ == 0x0f || command_ == 0x10)) {
        road_.light = 0;
        Need(4, Stage::Lighting);
        return;
    }
    RasterRoad();
    AdvanceRoad();
}

void Dsp4::RasterRoad() {
    auto& edge = polygons_[0].edges[0];
    const int32_t dx = Slope(road_.nextScroll.x - road_.previousScroll.x, road_.lines);
    const int32_t dy = Slope(road_.nextScroll.y - road_.previousScroll.y, road_.lines);
    int32_t x = Fixed(edge.center + road_.previousScroll.x);
    int32_t y = Fixed(-viewport_.bottom + road_.previousScroll.y + road_.verticalEnvelope +
                      polygons_[1].edges[0].center - road_.vertical);
    for (int line = 0; line < road_.lines; ++line) {
        WordOut(edge.pointer);
        WordOut(Wrap32(int64_t(y) + 32768) >> 16);
        WordOut(Wrap32(int64_t(x) + 32768) >> 16);
        edge.pointer = Wrap16(edge.pointer - 4);
        x = Wrap32(int64_t(x) + dx);
        y = Wrap32(int64_t(y) + dy);
    }
}

void Dsp4::AdvanceRoad() {
    road_.previous = road_.next;
    road_.previousScroll = road_.nextScroll;
    if (command_ != 7 && command_ != 0x10) {
        road_.dx = Wrap32(int64_t(road_.dx) + int32_t(road_.ddx) * 256);
        road_.dy = Wrap32(int64_t(road_.dy) + int32_t(road_.ddy) * 256);
        road_.x = Wrap32(int64_t(road_.x) + road_.dx + road_.envelope);
        road_.y = Wrap32(int64_t(road_.y) + road_.dy);
    }
    if (command_ == 1 || command_ == 0x0f) road_.turn = Wrap16(road_.turn + road_.turnDelta);
    Need(2, Stage::RoadDistance);
}

void Dsp4::WindowCommand() {
    std::array<Point, 2> view{};
    int16_t envelopes[2][2]{};
    if (stage_ == Stage::WindowDistance) {
        distance_ = Word();
        if (distance_ == -32768) { ClearOutput(); WordOut(0); Finish(); }
        else Need(16, Stage::WindowUpdate);
        return;
    }
    if (stage_ == Stage::Start) {
        for (auto& polygon : polygons_) for (auto& edge : polygon.edges) edge.right = Word();
        for (auto& polygon : polygons_) for (auto& edge : polygon.edges) edge.left = Word();
        for (unsigned i = 0; i < 8; ++i) Word();
        for (auto& polygon : polygons_) for (auto& edge : polygon.edges) edge.center = Word();
        for (auto& polygon : polygons_) for (auto& edge : polygon.edges) edge.pointer = Word();
        for (auto& polygon : polygons_) for (auto& edge : polygon.edges) edge.bottom = Word();
        for (auto& polygon : polygons_) for (auto& edge : polygon.edges) edge.top = Word();
        for (unsigned i = 0; i < 4; ++i) Word();
        distance_ = Word();
    }
    for (auto& point : view) { point.x = Word(); point.y = Word(); }
    for (auto& pair : envelopes) for (auto& value : pair) value = Word();
    ClearOutput();
    const auto clip = [](int value, const Edge& edge) {
        // Preserve the order even when a host supplies crossed clip limits.
        return std::min(std::max(value, int(edge.left)), int(edge.right));
    };
    if (stage_ == Stage::Start) {
        for (unsigned p = 0; p < 2; ++p) {
            polygons_[p].start = view[p].x;
            polygons_[p].plane = distance_;
            for (auto& edge : polygons_[p].edges) edge.raster = view[p].y;
        }
        for (unsigned side = 0; side < 2; ++side) {
            const auto& edge = polygons_[0].edges[side];
            ByteOut(uint8_t(clip(Wrap16(edge.center - view[0].x + envelopes[0][side]), edge)));
        }
    } else {
        for (unsigned p = 0; p < 2; ++p) {
            auto& polygon = polygons_[p];
            int16_t lines = Wrap16(polygon.edges[0].raster - view[p].y);
            if (lines > 0) for (auto& edge : polygon.edges) edge.raster = view[p].y;
            else lines = 0;
            if (view[p].y < polygon.edges[0].top) lines = 0;
            WordOut(lines);
            unsigned source = p;
            if (lines) {
                if (uint16_t(envelopes[p][0]) == 0xc001 || envelopes[p][1] == 0x3fff) source = 1;
                std::array<int32_t, 2> position{}, increment{};
                for (unsigned side = 0; side < 2; ++side) {
                    const int16_t before = Project(envelopes[p][side], polygons_[source].plane);
                    const int16_t after = Project(envelopes[p][side], distance_);
                    const int16_t a = Wrap16(view[source].x + before);
                    const int16_t b = Wrap16(polygons_[source].start + after);
                    increment[side] = Slope(b - a, lines);
                    if (lines == 1) increment[side] = Wrap32(-int64_t(increment[side]));
                    position[side] = Fixed(polygon.edges[side].center - polygons_[source].start + before);
                }
                polygon.plane = distance_;
                for (int line = 0; line < lines; ++line) {
                    WordOut(polygon.edges[0].pointer);
                    for (unsigned side = 0; side < 2; ++side) {
                        position[side] = Wrap32(int64_t(position[side]) + increment[side]);
                        auto& edge = polygon.edges[side];
                        ByteOut(uint8_t(clip(Wrap16(position[side] >> 16), edge)));
                        edge.pointer = Wrap16(edge.pointer - 4);
                    }
                }
            }
            polygon.start = view[source].x;
        }
    }
    Need(2, Stage::WindowDistance);
}

} // namespace snes::core
