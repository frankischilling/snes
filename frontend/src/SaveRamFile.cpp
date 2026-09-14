// snes emulator
// frontend/src/SaveRamFile.cpp
// Atomic battery RAM and clock sidecar persistence.

#include "SaveRamFile.hpp"
#include <SDL3/SDL.h>
#include <algorithm>
#include <atomic>
#include <fstream>
#include <stdexcept>
#include <string>

namespace snes::frontend {
std::vector<uint8_t> SaveRamFile::Load(size_t expectedSize) {
    if (expectedSize == 0 || !std::filesystem::exists(path_)) return {};
    if (std::filesystem::file_size(path_) != expectedSize)
        throw std::runtime_error("Save RAM file has the wrong size: " + path_.string());
    std::vector<uint8_t> data(expectedSize);
    std::ifstream file(path_, std::ios::binary);
    if (!file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size())))
        throw std::runtime_error("Cannot read save RAM: " + path_.string());
    saved_ = data;
    return data;
}

void SaveRamFile::Flush(std::span<const uint8_t> data) {
    if (data.empty() || std::equal(data.begin(), data.end(), saved_.begin(), saved_.end())) return;
    static std::atomic<unsigned> serial{0};
    auto temporary = path_;
    temporary += ".tmp-" + std::to_string(SDL_GetTicksNS()) + "-" + std::to_string(serial++);
    std::ofstream file(temporary, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot create save RAM file: " + temporary.string());
    file.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    file.close();
    if (!file) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        throw std::runtime_error("Cannot write save RAM: " + path_.string());
    }
    const auto source = temporary.u8string();
    const auto destination = path_.u8string();
    if (!SDL_RenamePath(reinterpret_cast<const char*>(source.c_str()),
                        reinterpret_cast<const char*>(destination.c_str()))) {
        const std::string error = SDL_GetError();
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        throw std::runtime_error("Cannot replace save RAM: " + error);
    }
    saved_.assign(data.begin(), data.end());
}
}
