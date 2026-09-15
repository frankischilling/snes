#include "snes/core/Dsp4.hpp"

namespace snes::core {

void Dsp4::EmitObject(bool& draw, int16_t x, int16_t y, int16_t attr,
                     bool large, bool stop) {
    const unsigned row = (y >> 3) & 31;
    const unsigned next = (row + 1) & 31;
    if (y >= 0 && (y & 511) >= 235) draw = false;
    if (large) {
        if (rows_[row] + 1 >= rowLimit_ || rows_[next] + 1 >= rowLimit_) draw = false;
    } else if (rows_[row] >= rowLimit_) draw = false;
    if (objectCount_ >= 128) draw = false;
    if (!draw) {
        if (stop) WordOut(0);
        return;
    }
    rows_[row] += large ? 2 : 1;
    if (large) rows_[next] += 2;
    WordOut(1);
    ByteOut(uint8_t(x));
    ByteOut(uint8_t(y));
    WordOut(attr);
    const unsigned flags = unsigned(x < 0 || x > 255) | (unsigned(large) << 1);
    attributes_[objectCount_ / 8] |= uint16_t(flags << ((objectCount_ % 8) * 2));
    ++objectCount_;
}

void Dsp4::ObjectCommand() {
    auto& edge = polygons_[0].edges[0];
    switch (stage_) {
    case Stage::Start:
        viewport_.x = Word(); viewport_.y = Word(); Word();
        viewport_.left = Word(); viewport_.right = Word();
        viewport_.top = Word(); viewport_.bottom = Word();
        edge.bottom = Wrap16(viewport_.bottom - viewport_.y);
        edge.raster = 256;
        Need(4, Stage::ObjectHeader);
        return;
    case Stage::ObjectHeader:
        raster_ = Word();
        if (raster_ < edge.raster) {
            object_.clip = Wrap16(viewport_.bottom - (edge.bottom - raster_));
            edge.raster = raster_;
        }
        distance_ = Word();
        if (distance_ == -32768) { Finish(); return; }
        if (!distance_) { Need(4, Stage::ObjectHeader); return; }
        if (uint16_t(distance_) == 0x9000) Need(14, Stage::Vehicle);
        else Need(10, Stage::TerrainObject);
        return;
    case Stage::Vehicle: {
        const uint16_t energy = uint16_t(Word());
        const int16_t impactY = Word(), back = Word(), impactX = Word(), left = Word();
        distance_ = Word();
        const int16_t right = Word();
        const int16_t x = Wrap16(Wrap16(right - left) - (Wrap32(int64_t(energy) * (impactX - left)) >> 16));
        const int16_t y = Wrap16(back - (Wrap32(int64_t(energy) * (back - impactY)) >> 16));
        object_.x = Wrap16(viewport_.x + Project(x, distance_));
        object_.y = Wrap16(viewport_.bottom - (edge.bottom - Project(y, distance_)));
        ClearOutput(); WordOut(x);
        Need(4, Stage::VehicleTail);
        return;
    }
    case Stage::VehicleTail:
        object_.y = Wrap16(object_.y + Word());
        object_.attributes = Word();
        object_.large = true;
        Need(2, Stage::TileHeader);
        return;
    case Stage::TerrainObject: {
        edge.center = Word();
        polygons_[0].edges[1].raster = Word();
        const int16_t x = Word(), y = Word();
        const int16_t lines = Wrap16(edge.bottom - raster_);
        object_.x = Wrap16(viewport_.x + Project(x, distance_) - edge.center);
        object_.y = Wrap16(viewport_.bottom - lines + Project(y, distance_));
        object_.attributes = Word();
        object_.large = true;
        Need(2, Stage::TileHeader);
        return;
    }
    case Stage::TileHeader: {
        raster_ = object_.header = Word();
        if (object_.header == -32768) { Finish(); return; }
        if (!object_.header) {
            if (object_.large) { object_.large = false; Need(2, Stage::TileHeader); }
            else Need(4, Stage::ObjectHeader);
            return;
        }
        switch (uint16_t(object_.header) >> 8) {
        case 0x20: case 0x2e: case 0x40: case 0x60: case 0xa0: case 0xc0: case 0xe0:
            Need(4, Stage::Tile);
            break;
        default: Need(4, Stage::ObjectHeader); break;
        }
        return;
    }
    case Stage::Tile: {
        const int16_t dy = Word(), dx = Word();
        const int16_t x = Wrap16(object_.x + dx), y = Wrap16(object_.y + dy);
        const int16_t attr = Wrap16(object_.attributes + object_.header);
        const int margin = object_.large ? 15 : 7;
        const bool horizontal = x >= viewport_.left - margin && x <= viewport_.right;
        bool draw = true;
        ClearOutput();
        if (horizontal && y >= object_.clip - margin && y <= object_.clip &&
            object_.clip >= viewport_.top - margin && object_.clip <= viewport_.bottom)
            EmitObject(draw, x, object_.clip, 0x00ee, object_.large, false);
        if (horizontal && y >= viewport_.top - margin && y <= viewport_.bottom && y <= object_.clip)
            EmitObject(draw, x, y, attr, object_.large, false);
        EmitObject(draw, 0, 256, 0, false, true);
        Need(2, Stage::TileHeader);
        return;
    }
    default: Finish(); return;
    }
}

} // namespace snes::core
