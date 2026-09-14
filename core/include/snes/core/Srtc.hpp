#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <span>

namespace snes::core {

class Srtc {
public:
    using Clock = std::function<int64_t()>;
    static constexpr size_t SaveSize = 24;
    explicit Srtc(Clock clock = {});
    uint8_t Read();
    void Write(uint8_t value);
    std::array<uint8_t, SaveSize> Save();
    bool Load(std::span<const uint8_t> data);

private:
    void Update();
    void SetWeekday();
    enum class Mode { Read, Command, Write, Idle };
    Clock clock_;
    std::array<uint8_t, 13> digits_{};
    std::array<uint8_t, 13> latched_{};
    int64_t timestamp_ = 0;
    int index_ = -1;
    Mode mode_ = Mode::Read;
};

}
