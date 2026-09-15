#include "snes/core/Dsp3.hpp"

#include <array>
#include <cstdio>
#include <stdexcept>
#include <vector>

using snes::core::Dsp3;
namespace {
void Check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void Put(Dsp3& chip, uint16_t word) { chip.Write(0x8000, uint8_t(word)); chip.Write(0xbfff, uint8_t(word >> 8)); }
uint16_t Get(Dsp3& chip) { const unsigned low = chip.Read(0x8000); return uint16_t(low | (unsigned(chip.Read(0xbfff)) << 8)); }
void Command(Dsp3& chip, uint8_t command) { chip.Write(0x8000, command); }
void Window(Dsp3& chip, unsigned width, unsigned height) { Command(chip, 6); Put(chip, uint16_t(width | (height << 8))); }

struct Bitstream {
    std::vector<uint16_t> words;
    unsigned size = 0;
    void Add(unsigned value, unsigned bits) {
        for (unsigned i = bits; i; --i) {
            if (!(size & 15)) words.push_back(0);
            words.back() |= uint16_t(((value >> (i - 1)) & 1) << (15 - (size++ & 15)));
        }
    }
};

void ProtocolAndCoordinates() {
    Dsp3 chip;
    Check(chip.Read(0xc000) == 0x84, "Reset selects 8-bit commands");
    chip.Write(0xffff, 3);
    Check(chip.Read(0xc000) == 0x84, "Status writes do not change the command phase");
    Command(chip, 0x7f);
    Check(chip.Read(0xc000) == 0x84, "Unrecognized byte leaves command reception active");
    Command(chip, 6);
    Check(chip.Read(0xc000) == 0x80, "Window command accepts words");
    chip.Write(0x8000, 16);
    Check(chip.Read(0xffff) == 0x90 && chip.Read(0xc000) == 0x90, "Status is nondestructive during a half-word");
    chip.Write(0xbfff, 16);
    Check(chip.Read(0xc000) == 0x84, "Window completes after the high byte");
    Command(chip, 3); Put(chip, 0x0509);
    Check(chip.Read(0x8000) == 89 && chip.Read(0xc000) == 0x90, "Linear address returns its low byte first");
    Check(chip.Read(0x8000) == 0 && chip.Read(0xc000) == 0x84, "High-byte read completes a response");
    constexpr std::array<uint16_t, 6> adjacent{0x0708,0x0709,0x0809,0x0908,0x0807,0x0707};
    for (unsigned direction = 0; direction < 6; ++direction) {
        Command(chip, 7); chip.Write(0x8000, uint8_t(direction)); Put(chip, 0x0808);
        Check(Get(chip) == adjacent[direction], "Hex neighbor coordinate");
        Check(Get(chip) == (adjacent[direction] & 255) + (adjacent[direction] >> 8) * 16, "Hex neighbor linear index");
    }
    Command(chip, 7); chip.Write(0x8000, 0); Put(chip, 0);
    Check(Get(chip) == 0x0f00 && Get(chip) == 240, "Neighbor walk wraps north across the map");
    Window(chip, 255, 255); Command(chip, 3); Put(chip, 0xffff);
    Check(Get(chip) == 0xff00, "Coordinate arithmetic truncates before signed division");
    Command(chip, 6); chip.Write(0x8000, 13); chip.Reset();
    Check(chip.Read(0xc000) == 0x84, "Reset abandons an incomplete word");
    Check(Dsp3::Selects(0x208000) && Dsp3::Selects(0xbfffff) && !Dsp3::Selects(0x1f8000) &&
          !Dsp3::Selects(0x207fff) && !Dsp3::Selects(0xc08000), "Cartridge bank selection");
}

void ConversionAndDiagnostics() {
    Dsp3 chip;
    Command(chip, 0x18); Put(chip, 2);
    for (unsigned row = 0; row < 2; ++row) {
        for (unsigned pair = 0; pair < 4; ++pair) Put(chip, uint16_t((1u << (pair * 2)) | (1u << (pair * 2 + 9))));
        for (unsigned pair = 0; pair < 4; ++pair)
            Check(Get(chip) == ((0x80u >> (pair * 2)) | ((0x40u >> (pair * 2)) << 8)), "Eight pixels transpose into eight planes");
    }
    Check(chip.Read(0xc000) == 0x84, "Multi-row conversion completes");
    Command(chip, 0x0f); Put(chip, 0x1234); Check(Get(chip) == 0, "RAM test response");
    Command(chip, 0x0c); Put(chip, 0xabcd); Check(Get(chip) == 0, "Zero-result diagnostic response");
    Command(chip, 0x10); Put(chip, 0); Put(chip, 0x4567); Put(chip, 0xffff);
    Check(chip.Read(0xc000) == 0x84, "Absorb stream ends at its sentinel");
    Command(chip, 0x1c); Put(chip, 0x1111); Put(chip, 0x2222);
    Check(Get(chip) == 0x2222 && Get(chip) == 0 && Get(chip) == 0, "Diagnostic staged read sequence");
    Command(chip, 2); Put(chip, 0); Put(chip, 0); Put(chip, 0); Put(chip, 123); Put(chip, 456);
    Check(Get(chip) == 1 && Get(chip) == 123 && Get(chip) == 456, "Coordinate record echoes X/Y after acknowledgement");
    Put(chip, 0); Put(chip, 0xffff);
    Check(chip.Read(0xc000) == 0x84, "Coordinate record termination");
    Command(chip, 0x1f); Put(chip, 0);
    for (unsigned i = 0; i < 1024; ++i) {
        const auto value = Get(chip);
        if (i < 16) Check(value == (0x8000u >> i), "Diagnostic ROM bit masks");
        if (i == 0x2b) Check(value == 0, "Wave origin");
        if (i == 0x4b) Check(value == 256, "Wave positive peak");
        if (i == 0x8b) Check(value == 0xff00, "Wave negative peak");
        if (i == 0x3b2) Check(value == 0xffff, "Hex direction coefficient");
        if (i >= 0x3d2) Check(value == 0xffff, "Unpopulated coefficient region");
    }
    Check(chip.Read(0xc000) == 0x84, "ROM dump finishes after exactly 1024 words");
}

void DecodeTokens() {
    Bitstream bits;
    bits.Add(0,2); bits.Add(0x41,9);
    bits.Add(1,2);
    bits.Add(2,2); bits.Add(0,1);
    bits.Add(3,2); bits.Add(0,4);
    bits.Add(0,2); bits.Add(0x100,9);
    bits.Add(1,2); bits.Add(1,2); bits.Add(1,2);
    bits.Add(0,1);
    for (unsigned i = 0; i < 4; ++i) bits.Add(0,3);
    for (unsigned i = 0; i < 4; ++i) bits.Add(i,3);
    bits.Add(4,3); bits.Add(0,1); bits.Add(0x35,8);
    bits.Add(5,3); bits.Add(1,1); bits.Add(0xabc,12);
    bits.Add(0,3);
    Dsp3 chip;
    Command(chip, 0x38); Put(chip, 8); Put(chip, 7);
    std::vector<uint16_t> actual;
    size_t input = 0;
    for (unsigned steps = 0; steps < 100; ++steps) {
        const auto status = chip.Read(0xc000);
        if (status == 0x84) break;
        if (status & 0x40) {
            Check(input < bits.words.size(), "Decoder does not consume padding beyond the supplied stream");
            Put(chip, bits.words[input++]);
        } else actual.push_back(Get(chip));
    }
    Check(actual == std::vector<uint16_t>({0x41,0x42,0x44,0x48,0x8002,0x35,0x8003,0xabc,0x41}),
          "Decoder handles delta symbols, prefix tables and both back-reference offset widths");
    Check(chip.Read(0xc000) == 0x84, "Decoder returns to commands at the logical output length");
    Command(chip, 0x38); Put(chip, 513); Put(chip, 1);
    Check(chip.Read(0xc000) == 0x84, "Invalid symbol table length cannot overrun private RAM");
}

void PathsAndIsolation() {
    Dsp3 chip, other;
    Window(chip, 16, 16); Window(other, 7, 7);
    Command(chip, 0x3e); Put(chip, 0x0808); Check(Get(chip) == 136, "Path origin index");
    Command(chip, 0x1e); Put(chip, 0x0101);
    constexpr std::array<uint16_t, 6> neighbors{120,121,137,152,135,119};
    for (unsigned i = 0; i < 6; ++i) {
        Check(Get(chip) == neighbors[i], "Path terrain collection walks each side in order");
        Check(chip.Read(0xc000) == 0x84, "Terrain and cost use byte transfers");
        chip.Write(0x8000, i == 2 ? 1 : 0);
        chip.Write(0x8000, uint8_t(i + 3));
        Command(other, 3); Put(other, 0x0203); Check(Get(other) == 17, "Another instance preserves independent geometry");
    }
    Check(Get(chip) == 0xffff, "Terrain collection sentinel");
    Put(chip, 0x0101);
    for (unsigned i = 0; i < 6; ++i) {
        Check(Get(chip) == neighbors[i], "Path result cell order");
        Check(chip.Read(0x8000) == (i == 2 ? 255 : i + 3), "Blocked first-ring cells have inaccessible weight");
    }
    Check(Get(chip) == 0xffff && chip.Read(0xc000) == 0x84, "Path result sentinel ends the command");
}
}

int main() {
    try { ProtocolAndCoordinates(); ConversionAndDiagnostics(); DecodeTokens(); PathsAndIsolation(); }
    catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
    std::puts("DSP-3 handshakes, geometry, conversion, diagnostics, decoding and path streams passed");
}
