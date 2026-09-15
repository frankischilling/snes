#include "snes/core/Dsp4.hpp"

#include <array>
#include <cstdio>
#include <stdexcept>
#include <vector>

using snes::core::Dsp4;
namespace {
void Check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void Put(Dsp4& chip, int word) { chip.Write(0x8000, uint8_t(word)); chip.Write(0xbfff, uint8_t(unsigned(word) >> 8)); }
uint16_t Get(Dsp4& chip) { const unsigned low = chip.Read(0x8000); return uint16_t(low | (unsigned(chip.Read(0xbfff)) << 8)); }
void Command(Dsp4& chip, unsigned command, std::initializer_list<int> words = {}) {
    Put(chip, int(command)); for (auto word : words) Put(chip, word);
}
void Long(std::vector<int>& words, int32_t value) { words.push_back(uint16_t(value)); words.push_back(uint16_t(uint32_t(value) >> 16)); }
void Send(Dsp4& chip, unsigned command, const std::vector<int>& words) { Put(chip, int(command)); for (auto word : words) Put(chip, word); }
std::vector<int> Road(bool guide = false, bool lit = false, bool multiplayer = false) {
    std::vector<int> words;
    if (lit) words.push_back(0);
    Long(words, 10 * 65536);
    for (int value : {10,0,0,10}) words.push_back(value);
    Long(words, 4 * 65536);
    for (int value : {100,0x1000,0}) words.push_back(value);
    if (guide) {
        for (int value : {0x4000,8,-2,6,2,0}) words.push_back(value);
    } else {
        Long(words, -2 * 65536); Long(words, 0);
        words.push_back(0x4000); words.push_back(0);
        if (multiplayer) words.push_back(0); else Long(words, 0);
        for (int value : {0,0,0}) words.push_back(value);
    }
    return words;
}

void ArithmeticAndPackets() {
    Dsp4 chip;
    Check(chip.Read(0x8000) == 255 && chip.Read(0xc000) == 0x80, "Empty port and status values");
    for (int a : {0,1,-1,32767,-32768}) for (int b : {0,1,-1,32767,-32768}) {
        Command(chip, 0, {a,b});
        const uint32_t expected = (a == -32768 && b == -32768) ? 0xc0000000u : uint32_t(a * b);
        const uint32_t low = Get(chip), high = Get(chip);
        Check((low | (high << 16)) == expected, "Signed 31-bit product including positive overflow");
        Check(chip.Read(0x8000) == 255, "Response length is four bytes");
    }
    Command(chip, 0x0a, {0,0xf18e,0});
    for (int value : {48,-48,-96,-384}) Check(Get(chip) == uint16_t(value), "Signed nibble mapping order");
    Command(chip, 0x11, {16384,8192,4096,2048});
    Check(Get(chip) == 0xa5a5, "Horizontal nibble quantization");
    Command(chip, 0, {7,9});
    for (unsigned i = 0; i < 4; ++i) chip.Write(0x8000, 0xff);
    Command(chip, 0, {3,5}); Check(Get(chip) == 15 && Get(chip) == 0, "Writes consume outstanding reply bytes before a new command");
    chip.Write(0x8000, 0); chip.Write(0xffff, 0x66); chip.Reset();
    Command(chip, 0, {8,9}); Check(Get(chip) == 72 && Get(chip) == 0, "Reset abandons half a command");
    Put(chip, 0xffff); Command(chip, 0, {2,3}); Check(Get(chip) == 6 && Get(chip) == 0, "Unknown commands allow a new packet");
    Check(Dsp4::Selects(0x308000) && Dsp4::Selects(0xbfffff) && !Dsp4::Selects(0x2f8000) &&
          !Dsp4::Selects(0x307fff) && !Dsp4::Selects(0xc08000), "Device address selection");
}

void OamLimits() {
    Dsp4 chip;
    Command(chip, 3); Command(chip, 5);
    Command(chip, 0x0b, {-1,10,0x1234});
    Check(Get(chip) == 1 && Get(chip) == 0x0aff && Get(chip) == 0x1234, "OAM position and attributes");
    Command(chip, 6);
    for (unsigned i = 0; i < 16; ++i) Check(Get(chip) == (i ? 0 : 1), "OAM high X bit is retained");
    Command(chip, 0x0e); Command(chip, 5);
    for (unsigned i = 0; i < 17; ++i) {
        Command(chip, 0x0b, {0,10,0});
        Check(Get(chip) == (i < 16 ? 1 : 0), "Multiplayer row budget");
        if (i < 16) { Get(chip); Get(chip); }
    }
    Command(chip, 3); Command(chip, 5);
    for (unsigned i = 0; i < 129; ++i) {
        Command(chip, 0x0b, {int(i),int((i % 28) * 8),0});
        Check(Get(chip) == (i < 128 ? 1 : 0), "OAM frame limit protects the attribute buffer");
        if (i < 128) { Get(chip); Get(chip); }
    }
    Command(chip, 5); Command(chip, 0x0b, {0,240,0}); Check(Get(chip) == 0, "Offscreen OAM row is rejected");
}

void ProjectionStreams() {
    Dsp4 chip;
    Send(chip, 1, Road());
    for (int value : {4,2,10,5,5}) Check(Get(chip) == value, "Road projection header");
    constexpr std::array<int, 5> horizontal{104,104,103,103,102};
    for (unsigned i = 0; i < 5; ++i) {
        Check(Get(chip) == 0x1000 - i * 4, "HDMA pointer walks backward");
        Check(Get(chip) == uint16_t(-10 + int(i)) && Get(chip) == horizontal[i], "Fixed-point interpolation and rounding");
    }
    Put(chip, 0x4000); Put(chip, 0); Put(chip, 0); Put(chip, 0);
    for (int value : {4,2,8,4,1}) Check(Get(chip) == value, "Incremental world position survives packet boundaries");
    Get(chip); Get(chip); Get(chip); Put(chip, 0x8000);
    Command(chip, 0, {6,7}); Check(Get(chip) == 42 && Get(chip) == 0, "Termination releases command parsing");
    for (unsigned command : {7u,0x0du,0x0fu,0x10u}) {
        const bool guide = command == 7 || command == 0x10;
        const bool lit = command == 0x0f || command == 0x10;
        Send(chip, command, Road(guide,lit,command == 0x0d));
        if (guide) { Check(Get(chip) == 7 && Get(chip) == 7, "Guideline projection applies deltas"); }
        else { Check(Get(chip) == 4 && Get(chip) == 2 && Get(chip) == 10 && Get(chip) == 5, "World projection variant"); }
        const unsigned lines = Get(chip);
        Check(lines == (guide ? 3u : 5u), "Variant raster count");
        if (lit) {
            for (unsigned color = 0; color < 4; ++color) {
                Put(chip, 0x4000); Put(chip, 0x7fff);
                Check(Get(chip) == 0x3def, "Lighting scales each five-bit color component");
                if (color != 3) Check(chip.Read(0x8000) == 255, "Lighting waits for the next color packet");
            }
        }
        for (unsigned i = 0; i < lines * 3; ++i) Get(chip);
        Put(chip, 0x8000);
    }
}

void LargeReplyAndReset() {
    Dsp4 chip;
    auto words = Road();
    words[0] = 0; words[1] = 200;
    words[2] = words[5] = 200;
    words[6] = words[7] = words[8] = 0;
    words[9] = 4;
    words[15] = 0;
    Send(chip, 1, words);
    for (int value : {0,0,200,0,200})
        Check(Get(chip) == value, "Large raster response header");
    for (unsigned line = 0; line < 200; ++line) {
        Check(Get(chip) == uint16_t(4 - int(line * 4)), "Long reply retains each wrapped HDMA pointer");
        Get(chip);
        Check(Get(chip) == 0, "Long replies preserve horizontal scroll beyond 512 bytes");
    }
    Check(chip.Read(0x8000) == 255, "Large response ends at its exact byte count");
    Put(chip, 0x8000);
    Send(chip, 0x0f, Road(false, true));
    Get(chip);
    chip.Reset();
    Command(chip, 0, {12,13});
    Check(Get(chip) == 156 && Get(chip) == 0, "Reset clears both an unfinished reply and pending lighting input");
}

void WindowStream() {
    Dsp4 chip;
    std::vector<int> words;
    for (int i = 0; i < 4; ++i) words.push_back(255);
    for (int i = 0; i < 12; ++i) words.push_back(0);
    for (int value : {80,160,100,120,0x1000,0x1000,0x2000,0x2000,10,10,10,10,0,0,0,0,0,0,0,0,
                     0x4000,0,10,0,10,-20,20,-20,20}) words.push_back(value);
    Check(words.size() == 45, "Polygon fixture size");
    Send(chip, 8, words);
    Check(chip.Read(0x8000) == 60 && chip.Read(0x8000) == 180, "Initial polygon limits");
    for (int value : {0x4000,0,8,0,8,-20,20,-20,20}) Put(chip, value);
    for (unsigned p = 0; p < 2; ++p) {
        Check(Get(chip) == 2, "Each polygon emits two raster records");
        for (unsigned i = 0; i < 2; ++i) {
            Check(Get(chip) == (p ? 0x2000 : 0x1000) - i * 4, "Polygon HDMA pointer");
            Check(chip.Read(0x8000) == (p ? 90 : 70) && chip.Read(0x8000) == (p ? 130 : 170), "Projected polygon boundaries");
        }
    }
    Put(chip, 0x8000); Check(Get(chip) == 0, "Polygon terminator emits its final zero");
}

void ObjectStream() {
    Dsp4 chip;
    Command(chip, 3); Command(chip, 5);
    Command(chip, 9, {128,0,0,0,255,0,224});
    for (int value : {100,0x4000,0,0,0,-20,0x10,0x2000,0,0}) Put(chip, value);
    Check(Get(chip) == 1 && Get(chip) == 0x6480 && Get(chip) == 0x00ee, "Sprite mask clips against the road");
    Check(Get(chip) == 1 && Get(chip) == 0x5a80 && Get(chip) == 0x2010, "Terrain object produces a projected OAM tile");
    Check(Get(chip) == 0, "Object output has an explicit end marker");
    Put(chip, 0x8000);
    Command(chip, 6);
    for (unsigned i = 0; i < 16; ++i) Check(Get(chip) == (i ? 0 : 0x0a), "Projected large-object flags are packed");
    Command(chip, 5);
    Command(chip, 9, {128,0,0,0,255,0,224});
    for (int value : {150,0x9000,0,0,20,0,-10,0x4000,10}) Put(chip, value);
    Check(Get(chip) == 20, "Vehicle projection exposes its collision-adjusted world X");
    for (int value : {0,0x20,0x2000,0,0}) Put(chip, value);
    Check(Get(chip) == 1 && Get(chip) == 0x0a8a && Get(chip) == 0x2020 && Get(chip) == 0, "Vehicle object stream continues through tile packets");
    Put(chip, 0x8000);
    Dsp4 independent;
    Command(independent, 0, {5,5}); Check(Get(independent) == 25 && Get(independent) == 0, "Instances own their command state");
}
}

int main() {
    try { ArithmeticAndPackets(); OamLimits(); ProjectionStreams(); LargeReplyAndReset(); WindowStream(); ObjectStream(); }
    catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
    std::puts("DSP-4 arithmetic, packet phases, OAM budgets, road, lighting, window and object streams passed");
}
