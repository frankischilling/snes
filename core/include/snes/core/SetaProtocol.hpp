// Observed ST011 command packets and ST018 startup handshakes.
// These interfaces do not emulate either device's internal program processor.
#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace snes::core {

class St011 {
public:
    explicit St011(std::span<uint8_t> ram) : ram_(ram) {}
    void Reset();
    uint8_t Read(uint32_t address) const;
    void Write(uint32_t address, uint8_t value);
    std::span<const uint8_t, 81> Board() const { return board_; }
private:
    std::span<uint8_t> ram_;
    std::array<uint8_t, 65536> work_{};
    std::array<uint8_t, 128> packet_{};
    std::array<uint8_t, 81> board_{};
    uint8_t command_ = 0;
    unsigned received_ = 0, required_ = 0;
    void Store(unsigned offset, uint8_t value);
    void Complete();
};

class St018 {
public:
    explicit St018(std::span<uint8_t> ram) : ram_(ram) {}
    void Reset();
    uint8_t Read(uint32_t address);
    void Write(uint32_t address, uint8_t value);
    unsigned PendingOutput() const { return outputCount_; }
private:
    std::span<uint8_t> ram_;
    uint32_t command_ = 0;
    unsigned commandBytes_ = 0, pass_ = 0, outputCount_ = 0;
    bool awaitingParameter_ = false;
};

} // namespace snes::core
