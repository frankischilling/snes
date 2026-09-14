#include "SaveRamFile.hpp"
#include <SDL3/SDL.h>
#include <cstdio>
#include <chrono>
#include <fstream>
#include <stdexcept>

using snes::frontend::SaveRamFile;
namespace fs = std::filesystem;
int main() {
    const auto directory = fs::temp_directory_path() / ("snes-save-test-" + std::to_string(SDL_GetTicksNS()));
    if (!fs::create_directory(directory)) return 2;
    struct Cleanup {
        fs::path path;
        ~Cleanup() { std::error_code ignored; fs::remove_all(path, ignored); }
    } cleanup{directory};
    try {
        auto check = [](bool condition, const char* label) {
            if (!condition) throw std::runtime_error(label);
        };
        const auto path = directory / "game.srm";
        SaveRamFile save(path);
        check(save.Load(8192).empty(), "A missing save starts empty");
        save.Flush({});
        check(!fs::exists(path), "A cartridge without SRAM creates no save");
        std::vector<uint8_t> data(8192, 0x57);
        save.Flush(data);
        check(SaveRamFile(path).Load(data.size()) == data, "Save round trip");
        data[4096] = 0x63;
        save.Flush(data);
        check(SaveRamFile(path).Load(data.size()) == data, "Replace an existing save");
        const auto timestamp = fs::last_write_time(path) - std::chrono::hours(1);
        fs::last_write_time(path, timestamp);
        save.Flush(data);
        check(fs::last_write_time(path) == timestamp, "Unchanged SRAM does not rewrite the save");
        bool rejected = false;
        try { SaveRamFile(path).Load(1024); } catch (const std::runtime_error&) { rejected = true; }
        check(rejected, "Reject a save with the wrong size");
        check(SaveRamFile(path).Load(data.size()) == data, "Rejecting a save preserves its contents");
        const auto blocked = directory / "directory.srm";
        fs::create_directory(blocked);
        std::ofstream(blocked / "keep.txt") << "preserve";
        rejected = false;
        try { SaveRamFile(blocked).Flush(data); } catch (const std::runtime_error&) { rejected = true; }
        check(rejected && fs::exists(blocked / "keep.txt"), "A failed replacement preserves the destination");
        for (const auto& entry : fs::directory_iterator(directory))
            check(entry.path().filename().string().find(".tmp-") == std::string::npos,
                  "Save operations clean up temporary files");
        std::printf("Save RAM persistence checks passed\n");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
