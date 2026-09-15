#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace snes::core {

// Stateful byte port for road, window and object projection commands.
class Dsp4 {
public:
    Dsp4() = default;
    void Reset();
    uint8_t Read(uint16_t address);
    void Write(uint16_t address, uint8_t value);
    static bool Selects(uint32_t address) noexcept;

private:
    enum class Stage : uint8_t {
        Start, RoadDistance, RoadSplice, RoadUpdate, Lighting,
        WindowDistance, WindowUpdate, ObjectHeader, Vehicle,
        VehicleTail, TerrainObject, TileHeader, Tile
    };
    struct Point { int16_t x = 0, y = 0; };
    struct Edge {
        int16_t left = 0, right = 0, center = 0, pointer = 0;
        int16_t top = 0, bottom = 0, raster = 0;
    };
    struct Polygon {
        std::array<Edge, 2> edges{};
        int16_t start = 0, plane = 0;
    };
    struct Road {
        int32_t x = 0, y = 0, dx = 0, dy = 0, envelope = 0;
        int16_t ddx = 0, ddy = 0, vertical = 0, verticalEnvelope = 0;
        Point previous{}, next{}, previousScroll{}, nextScroll{}, delta{};
        int16_t turn = 0, turnDelta = 0, lines = 0;
        unsigned light = 0;
    };
    struct Viewport {
        int16_t x = 0, y = 0, left = 0, right = 0, top = 0, bottom = 0;
    };
    struct Object {
        int16_t x = 0, y = 0, attributes = 0, clip = 0, header = 0;
        bool large = false;
    };

    void Execute();
    void Need(size_t bytes, Stage stage);
    void Finish();
    int16_t Word();
    int32_t Long();
    void ByteOut(uint8_t value);
    void WordOut(int value);
    void ClearOutput();
    static int16_t Wrap16(int64_t value);
    static int32_t Wrap32(int64_t value);
    static int32_t Fixed(int value);
    static int16_t Project(int value, int16_t distance);
    static int32_t Slope(int delta, int16_t lines);

    void RoadCommand();
    void ProjectRoad();
    void RasterRoad();
    void AdvanceRoad();
    void WindowCommand();
    void ObjectCommand();
    void EmitObject(bool& draw, int16_t x, int16_t y, int16_t attr,
                    bool large, bool stop);

    std::array<uint8_t, 96> input_{};
    std::vector<uint8_t> output_;
    size_t received_ = 0, needed_ = 0, cursor_ = 0, outputCursor_ = 0;
    unsigned commandBytes_ = 0;
    uint16_t command_ = 0;
    bool commandReady_ = true;
    Stage stage_ = Stage::Start;
    int16_t distance_ = 0, raster_ = 0;
    std::array<Polygon, 2> polygons_{};
    Road road_{};
    Viewport viewport_{};
    Object object_{};
    std::array<uint16_t, 16> attributes_{};
    std::array<uint16_t, 32> rows_{};
    unsigned objectCount_ = 0;
    uint16_t rowLimit_ = 0;
};

} // namespace snes::core
