#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace snes::core {

// Byte-oriented command interface for the cartridge bitmap processor.
class Dsp2 {
public:
    uint8_t Read();
    void Write(uint8_t value);

private:
    void Execute();
    std::array<uint8_t, 512> input_{};
    std::array<uint8_t, 512> output_{};
    size_t received_ = 0;
    size_t needed_ = 0;
    size_t read_ = 0;
    size_t available_ = 0;
    size_t length_ = 0;
    size_t scaledLength_ = 0;
    uint8_t command_ = 0;
    uint8_t transparent_ = 0;
    bool payload_ = false;
};

}
