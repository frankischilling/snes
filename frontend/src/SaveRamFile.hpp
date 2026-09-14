// snes emulator
// frontend/src/SaveRamFile.hpp
// Save-file persistence interface.

#pragma once
#include <cstdint>
#include <filesystem>
#include <span>
#include <utility>
#include <vector>

namespace snes::frontend {
class SaveRamFile {
public:
    explicit SaveRamFile(std::filesystem::path path) : path_(std::move(path)) {}
    std::vector<uint8_t> Load(size_t expectedSize);
    void Flush(std::span<const uint8_t> data);
private:
    std::filesystem::path path_;
    std::vector<uint8_t> saved_;
};
}
