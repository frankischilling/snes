#include "snes/core/Emulator.hpp"
#include "snes/core/SnesCpu.hpp"

#include <charconv>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
unsigned Number(std::string_view value, int base, unsigned limit) {
    if (base == 16 && value.starts_with("0x")) value.remove_prefix(2);
    unsigned result = 0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result, base);
    if (value.empty() || parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || result > limit)
        throw std::runtime_error("Invalid numeric argument");
    return result;
}

// Text diagnostics commonly use ASCII tile numbers in a 32-column BG map.
// Inspect backing VRAM directly so reporting cannot alter a PPU read latch.
std::string TextPage(const uint16_t* vram, unsigned base) {
    std::string page;
    for (unsigned y = 0; y < 28; ++y) {
        std::string row;
        for (unsigned x = 0; x < 32; ++x) {
            const unsigned tile = vram[(base + y * 32 + x) & 0x7fff] & 0x3ff;
            row += tile >= 32 && tile <= 126 ? char(tile) : ' ';
        }
        const auto last = row.find_last_not_of(' ');
        if (last != std::string::npos) page += row.substr(0, last + 1) + '\n';
    }
    return page;
}
}

int main(int argc, char** argv) {
    if (argc < 3 || argc > 6) {
        std::cerr << "Usage: snes_rom_diagnostic ROM frames [tilemap-word-address-hex] [--expect text-file]\n";
        return 2;
    }
    try {
        const unsigned frames = Number(argv[2], 10, 1000000);
        unsigned tilemap = 0x6000;
        int argument = 3;
        if (argument < argc && std::string_view(argv[argument]) != "--expect")
            tilemap = Number(argv[argument++], 16, 0x7fff);
        std::vector<std::string> expected;
        if (argument < argc) {
            if (std::string_view(argv[argument]) != "--expect" || argument + 2 != argc)
                throw std::runtime_error("Expected --expect followed by a text file");
            std::ifstream file(argv[argument + 1]);
            if (!file) throw std::runtime_error("Cannot open expected-result file");
            std::string row;
            while (std::getline(file, row)) {
                if (!row.empty() && row.back() == '\r') row.pop_back();
                if (!row.empty()) expected.push_back(row);
            }
            if (expected.empty()) throw std::runtime_error("Expected-result file is empty");
        }
        if (!frames) throw std::runtime_error("Frame count must be positive");
        auto machine = std::make_unique<snes::core::Emulator>();
        std::string error;
        if (!machine->LoadCartridgeFromFile(argv[1], &error)) throw std::runtime_error(error);
        std::string candidate, previous;
        std::vector<bool> matched(expected.size(), false);
        unsigned stable = 0, pages = 0;
        for (unsigned frame = 1; frame <= frames; ++frame) {
            machine->StepFrame({false, false, false});
            const auto page = TextPage(machine->GetPpu().VramData(), tilemap);
            if (page != candidate) { candidate = page; stable = 0; }
            if (!machine->GetPpu().DisplayDisable() && ++stable >= 2 &&
                !candidate.empty() && candidate != previous) {
                std::cout << "frame=" << frame << '\n' << candidate << std::flush;
                const std::string bounded = '\n' + candidate;
                for (size_t i = 0; i < expected.size(); ++i)
                    matched[i] = matched[i] || bounded.find('\n' + expected[i] + '\n') != std::string::npos;
                previous = candidate;
                ++pages;
            }
        }
        std::printf("completed_frames=%u text_pages=%u PC=%02X:%04X master_clocks=%llu smp_cycles=%llu\n",
            frames, pages, unsigned(machine->GetCpu()->regs().pb), unsigned(machine->GetCpu()->regs().pc),
            static_cast<unsigned long long>(machine->GetTiming().MasterClocksElapsed()),
            static_cast<unsigned long long>(machine->GetSmp().CycleCount()));
        bool success = true;
        for (size_t i = 0; i < expected.size(); ++i) {
            if (matched[i]) continue;
            std::cerr << "Missing expected result: " << expected[i] << '\n';
            success = false;
        }
        if (!expected.empty()) {
            std::cout << "expected_results=" << expected.size() << " status=" << (success ? "PASS" : "FAIL") << '\n';
            return success ? 0 : 1;
        }
        // Without expectations, completion reports execution only.
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
