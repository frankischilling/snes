#pragma once

#include <array>
#include <cstdint>

namespace snes::core::detail {

// The diagnostic dump exposes coefficients as data. Repeated regions are
// generated from their row layout instead of storing an entire ROM image.
constexpr auto Dsp3Coefficients() {
    std::array<uint16_t, 1024> words{};
    for (unsigned i = 0; i < 16; ++i) words[i] = uint16_t(0x8000u >> i);
    for (unsigned i = 0; i < 8; ++i) words[16 + i] = uint16_t(2u << i);
    constexpr uint16_t configuration[]{
        0,15,1024,512,320,1024,512,64,125,126,126,123,124,125,123,124,2,32,48
    };
    for (unsigned i = 0; i < 19; ++i) words[0x18 + i] = configuration[i];
    constexpr int16_t wave[]{
        0,13,25,38,50,62,74,86,98,109,121,132,142,152,162,172,
        181,190,198,206,213,220,226,231,236,241,245,248,251,253,255,256,
        256,256,255,253,251,248,245,241,237,231,226,220,213,206,198,190,
        181,172,162,153,142,132,121,110,98,86,74,62,50,38,25,13,
        0,-13,-25,-37,-50,-62,-74,-86,-98,-109,-121,-131,-142,-152,-162,-172,
        -181,-190,-198,-206,-213,-219,-226,-231,-236,-241,-245,-248,-251,-253,-255,-256,
        -256,-256,-255,-253,-251,-248,-245,-241,-237,-232,-226,-220,-213,-206,-198,-190,
        -181,-172,-163,-153,-142,-132,-121,-110,-98,-87,-75,-62,-50,-38,-25,-13
    };
    for (unsigned i = 0; i < 128; ++i) words[0x2b + i] = uint16_t(wave[i]);
    constexpr uint16_t masks[]{43,127,32,255,0xff00};
    for (unsigned i = 0; i < 5; ++i) words[0xab + i] = masks[i];
    constexpr int16_t headers[]{-66,-63,-59,-54,-48,-41,-33,-24,-14,-3,-57,-44,-30,-15,-53,-36,-18,-18};
    constexpr unsigned bases[]{0,1,3,6,10,15,21,28,36,45,55,0,12,25,39,0,16,33};
    for (unsigned bank = 0; bank < 2; ++bank) {
        for (unsigned row = 0; row < 18; ++row) {
            const unsigned at = 0xb0 + bank * 360 + row * 20;
            words[at] = uint16_t(headers[row]);
            for (unsigned col = 0; col <= row; ++col)
                words[at + 1 + col] = uint16_t(bases[row] + col);
            words[at + row + 2] = uint16_t(int(bases[row]) + (bank ? -340 : 68));
        }
    }
    constexpr uint16_t tail[]{
        340,536,272,176,204,176,136,176,68,176,0,176,
        254,0xff07,2,255,248,7,254,238,2047,512,239,0xf800,0x700,238
    };
    for (unsigned i = 0; i < 26; ++i) words[0x380 + i] = tail[i];
    constexpr int16_t neighborhood[]{-1,-1,-1,0,0,1,1,1,1,0,0,-1};
    constexpr int16_t hexSteps[]{-1,0,-1,1,0,1,1,0,0,-1,-1,-1};
    for (unsigned i = 0; i < 24; ++i) {
        words[0x39a + i] = uint16_t(neighborhood[i % 12]);
        words[0x3b2 + i] = uint16_t(hexSteps[i % 12]);
    }
    for (unsigned i = 0; i < 6; ++i) words[0x3cc + i] = uint16_t(68 * i);
    for (unsigned i = 0x3d2; i < 1024; ++i) words[i] = 0xffff;
    return words;
}

inline constexpr auto Dsp3Rom = Dsp3Coefficients();

} // namespace snes::core::detail
