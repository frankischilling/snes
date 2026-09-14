#pragma once

#include <cstdint>
#include <span>

namespace snes::core {

// Command-level model with a shared 4 KiB cartridge RAM window.
class St010 {
public:
    static bool Selects(uint32_t address) noexcept;
    uint8_t Read(uint32_t address, std::span<const uint8_t> ram) const;
    void Write(uint32_t address, uint8_t value, std::span<uint8_t> ram);

private:
    void Execute(std::span<uint8_t> ram);
    bool enabled_ = false;
    uint8_t command_ = 0, execute_ = 0;
};

} // namespace snes::core
