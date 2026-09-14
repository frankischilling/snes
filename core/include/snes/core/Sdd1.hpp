#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace snes::core {

// The reader supplies mapped cartridge bytes without CPU bus side effects.
std::vector<uint8_t> DecompressSdd1(
    const std::function<uint8_t(uint32_t)>& reader, size_t outputSize);

} // namespace snes::core
