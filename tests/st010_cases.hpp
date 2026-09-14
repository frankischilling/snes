#pragma once

#include <array>
#include <cstdint>

namespace st010_test {
constexpr unsigned Cases = 2048;
inline std::array<uint8_t, 4096> Input(unsigned command, unsigned index) {
    std::array<uint8_t, 4096> ram;
    uint32_t state = 0x721c49a3u ^ (command << 24) ^ index;
    for (auto& byte : ram) {
        state ^= state << 13; state ^= state >> 17; state ^= state << 5;
        byte = uint8_t(state);
    }
    const auto word = [&](unsigned offset, uint16_t value) {
        ram[offset] = uint8_t(value); ram[offset + 1] = uint8_t(value >> 8);
    };
    constexpr std::array<uint16_t, 12> edges{0, 1, 2, 31, 32, 255, 256, 0x7fff, 0x8000, 0x8001, 0xfffe, 0xffff};
    if (index < edges.size() * edges.size()) {
        word(0, edges[index % edges.size()]); word(2, edges[index / edges.size()]);
        word(4, edges[(index / 3) % edges.size()]);
    }
    if (command == 2) {
        word(0x24, uint16_t(index % 33));
        // Include ties and track stable driver ordering.
        for (unsigned i = 0; i < 32; ++i) word(0x40 + i * 2, uint16_t(ram[0x40 + i * 2] & 15));
    }
    if (command == 5 && index < 1024) {
        word(0xc0, uint16_t((index & 15) - 8));
        word(0xc2, uint16_t(((index >> 4) & 15) - 8));
        word(0xc4, 0); word(0xc6, 0); word(0xc8, 0); word(0xca, 0);
        word(0xcc, uint16_t(index << 8));
        word(0xd4, uint16_t(index)); word(0xd6, 100); word(0xd8, 1000);
        word(0xda, index & 1 ? 0xffff : 0);
    }
    if (command == 7 || command == 8) word(command == 7 ? 0 : 4, uint16_t(index * 257));
    return ram;
}
inline uint64_t Digest(uint64_t hash, const std::array<uint8_t, 4096>& ram) {
    for (auto byte : ram) hash = (hash ^ byte) * 1099511628211ull;
    return hash;
}
} // namespace st010_test
