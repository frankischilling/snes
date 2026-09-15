// snes emulator
// core/src/audio/Dsp.cpp
// S-DSP voice mixing, envelopes, echo, and sample generation.

// Dsp.cpp — SNES S-DSP Implementation
//
// Shared voice latches and echo RAM accesses advance on individual DSP clocks.

#include "snes/core/Dsp.hpp"
#include <cstring>

namespace snes::core {

// Static tables

// 512-entry half-Gaussian interpolation table
static constexpr int16_t kGauss[512] = {
       0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
       1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,
       2,   2,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,
       6,   6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  10,
      11,  11,  11,  12,  12,  13,  13,  14,  14,  15,  15,  15,  16,  16,  17,  17,
      18,  19,  19,  20,  20,  21,  21,  22,  23,  23,  24,  24,  25,  26,  27,  27,
      28,  29,  29,  30,  31,  32,  32,  33,  34,  35,  36,  36,  37,  38,  39,  40,
      41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,
      58,  59,  60,  61,  62,  64,  65,  66,  67,  69,  70,  71,  73,  74,  76,  77,
      78,  80,  81,  83,  84,  86,  87,  89,  90,  92,  94,  95,  97,  99, 100, 102,
     104, 106, 107, 109, 111, 113, 115, 117, 118, 120, 122, 124, 126, 128, 130, 132,
     134, 137, 139, 141, 143, 145, 147, 150, 152, 154, 156, 159, 161, 163, 166, 168,
     171, 173, 175, 178, 180, 183, 186, 188, 191, 193, 196, 199, 201, 204, 207, 210,
     212, 215, 218, 221, 224, 227, 230, 233, 236, 239, 242, 245, 248, 251, 254, 257,
     260, 263, 267, 270, 273, 276, 280, 283, 286, 290, 293, 297, 300, 304, 307, 311,
     314, 318, 321, 325, 328, 332, 336, 339, 343, 347, 351, 354, 358, 362, 366, 370,
     374, 378, 381, 385, 389, 393, 397, 401, 405, 410, 414, 418, 422, 426, 430, 434,
     439, 443, 447, 451, 456, 460, 464, 469, 473, 477, 482, 486, 491, 495, 499, 504,
     508, 513, 517, 522, 527, 531, 536, 540, 545, 550, 554, 559, 563, 568, 573, 577,
     582, 587, 592, 596, 601, 606, 611, 615, 620, 625, 630, 635, 640, 644, 649, 654,
     659, 664, 669, 674, 678, 683, 688, 693, 698, 703, 708, 713, 718, 723, 728, 732,
     737, 742, 747, 752, 757, 762, 767, 772, 777, 782, 787, 792, 797, 802, 806, 811,
     816, 821, 826, 831, 836, 841, 846, 851, 855, 860, 865, 870, 875, 880, 884, 889,
     894, 899, 904, 908, 913, 918, 923, 927, 932, 937, 941, 946, 951, 955, 960, 965,
     969, 974, 978, 983, 988, 992, 997,1001,1005,1010,1014,1019,1023,1027,1032,1036,
    1040,1045,1049,1053,1057,1061,1066,1070,1074,1078,1082,1086,1090,1094,1098,1102,
    1106,1109,1113,1117,1121,1125,1128,1132,1136,1139,1143,1146,1150,1153,1157,1160,
    1164,1167,1170,1174,1177,1180,1183,1186,1190,1193,1196,1199,1202,1205,1207,1210,
    1213,1216,1219,1221,1224,1227,1229,1232,1234,1237,1239,1241,1244,1246,1248,1251,
    1253,1255,1257,1259,1261,1263,1265,1267,1269,1270,1272,1274,1275,1277,1279,1280,
    1282,1283,1284,1286,1287,1288,1290,1291,1292,1293,1294,1295,1296,1297,1297,1298,
    1299,1300,1300,1301,1302,1302,1303,1303,1303,1304,1304,1304,1304,1304,1305,1305,
};

// Counter rate table: period in counter ticks for each rate (0-31)
// Rate 0 = never fires (period > counter range)
static constexpr unsigned kCounterRates[32] = {
    30721, 2048, 1536,
    1280, 1024,  768,
     640,  512,  384,
     320,  256,  192,
     160,  128,   96,
      80,   64,   48,
      40,   32,   24,
      20,   16,   12,
      10,    8,    6,
       5,    4,    3,
             2,
             1
};

// Counter offset table: phase alignment for each rate
static constexpr unsigned kCounterOffsets[32] = {
       1, 0, 1040,
     536, 0, 1040,
     536, 0, 1040,
     536, 0, 1040,
     536, 0, 1040,
     536, 0, 1040,
     536, 0, 1040,
     536, 0, 1040,
     536, 0, 1040,
     536, 0, 1040,
          0,
          0
};

static constexpr int kSimpleCounterRange = 2048 * 5 * 3;  // 30720

// Initial register values (all 0 except FLG at 0x6C = 0xE0)
static constexpr uint8_t kInitialRegs[128] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

// Constructor / Power / SoftReset

Dsp::Dsp() {
    Power();
}

void Dsp::Power() {
    std::memcpy(regs_.data(), kInitialRegs, RegisterCount);

    // Reset all voices
    for (int i = 0; i < VoiceCount; i++) {
        voices_[i] = Voice{};
        voices_[i].brrOffset = 1;
    }

    newKon_ = regs_[kKon];
    kon_ = 0;
    endxBuf_ = envxBuf_ = outxBuf_ = 0;
    samplesWritten_ = 0;
    echoLength_ = 0;
    pipe_ = {};
    std::memset(echoHist_, 0, sizeof(echoHist_));

    SoftReset();
}

void Dsp::SoftReset() {
    regs_[kFlg] = 0xE0;  // Soft reset + mute + echo write disable
    noise_       = 0x4000;
    echoHistIdx_ = 0;
    everyOther_  = 1;
    echoOffset_  = 0;
    counter_     = 0;
    phase_       = 0;
}

// Register access

uint8_t Dsp::Read(uint8_t addr) const {
    return regs_[addr & 0x7F];
}

void Dsp::Write(uint8_t addr, uint8_t data) {
    addr &= 0x7F;
    regs_[addr] = data;

    switch (addr & 0x0F) {
    case kEnvX:  // ENVX writes are buffered and overwritten by DSP
        envxBuf_ = data;
        break;
    case kOutX:  // OUTX writes are buffered and overwritten by DSP
        outxBuf_ = data;
        break;
    case 0x0C:   // Could be KON (0x4C) or ENDX (0x7C)
        if (addr == kKon) {
            newKon_ = data;
        }
        if (addr == kEndx) {
            // Writing any value to ENDX clears it
            endxBuf_ = 0;
            regs_[kEndx] = 0;
        }
        break;
    }
}

// Output buffer management

void Dsp::SetOutput(int16_t* buffer, int maxSamples) {
    outBuf_     = buffer;
    outBufSize_ = maxSamples;
    samplesWritten_ = 0;
}

// Counter system

void Dsp::runCounters() {
    if (--counter_ < 0)
        counter_ = kSimpleCounterRange - 1;
}

bool Dsp::readCounter(int rate) const {
    return ((unsigned)counter_ + kCounterOffsets[rate]) % kCounterRates[rate] == 0;
}

// Helpers

int Dsp::clamp16(int value) {
    if (static_cast<int16_t>(value) != value)
        value = (value >> 31) ^ 0x7FFF;
    return value;
}

uint8_t Dsp::ramRead(uint16_t addr) const {
    return ram_ ? ram_[addr] : 0;
}

uint16_t Dsp::ramReadWord(uint16_t addr) const {
    if (!ram_) return 0;
    return static_cast<uint16_t>(ram_[addr] | (ram_[(addr + 1) & 0xFFFF] << 8));
}

void Dsp::ramWriteWord(uint16_t addr, int16_t value) {
    if (!ram_) return;
    ram_[addr] = static_cast<uint8_t>(value);
    ram_[(addr + 1) & 0xFFFF] = static_cast<uint8_t>(value >> 8);
}

// BRR decoding — decode 4 samples from 2 BRR data bytes
//
// header:    BRR block header byte (shift in bits 7-4, filter in bits 3-2)
// brrByte1:  first data byte (high nybbles)
// brrByte2:  second data byte (low nybbles)

void Dsp::decodeBrr(Voice& v, int header, int brrByte1, int brrByte2) {
    // Arrange 4 nybbles in 0xABCD order
    const unsigned nybbles = unsigned(brrByte1) * 0x100 + unsigned(brrByte2);

    // Write to next 4 samples in circular buffer
    int* pos = &v.buf[v.bufPos];
    if ((v.bufPos += 4) >= BrrBufSize)
        v.bufPos = 0;

    int const shift  = header >> 4;
    int const filter = header & 0x0C;

    for (int i = 0; i < 4; i++) {
        // Extract top nybble, sign-extend
        int s = int((nybbles >> (12 - i * 4)) & 15);
        if (s & 8) s -= 16;

        // Apply shift
        s = shift <= 12 ? (s * (1 << shift)) >> 1 : s < 0 ? -2048 : 0;

        // IIR filter using previous 2 samples
        int const p1 = pos[BrrBufSize - 1];
        int const p2 = pos[BrrBufSize - 2] >> 1;

        if (filter >= 8) {
            s += p1;
            s -= p2;
            if (filter == 8) {
                // Filter 2: s += p1 * 0.953125 - p2 * 0.46875
                s += p2 >> 4;
                s += (p1 * -3) >> 6;
            } else {
                // Filter 3: s += p1 * 0.8984375 - p2 * 0.40625
                s += (p1 * -13) >> 7;
                s += (p2 * 3) >> 4;
            }
        } else if (filter) {
            // Filter 1: s += p1 * 0.46875
            s += p1 >> 1;
            s += (-p1) >> 5;
        }
        // Filter 0: no filter (s unchanged)

        // Clamp to 16-bit, then double
        s = clamp16(s);
        s = static_cast<int16_t>(s * 2);

        // Write to both halves of doubled buffer
        pos[BrrBufSize] = pos[0] = s;
        pos++;
    }
}

// Gaussian interpolation — 4-point interpolation using half-Gaussian table

int Dsp::interpolate(const Voice& v) const {
    int const* in = &v.buf[(v.interpPos >> 12) + v.bufPos];

    // Index into Gaussian table from fractional position
    int offset = (v.interpPos >> 4) & 0xFF;
    int16_t const* fwd = kGauss + 255 - offset;
    int16_t const* rev = kGauss + offset;

    int out;
    out  = (fwd[  0] * in[0]) >> 11;
    out += (fwd[256] * in[1]) >> 11;
    out += (rev[256] * in[2]) >> 11;
    out  = static_cast<int16_t>(out);     // Truncate after 3 taps
    out += (rev[  0] * in[3]) >> 11;

    out = clamp16(out);
    out &= ~1;  // Clear LSB
    return out;
}

// ADSR/GAIN envelope processing

void Dsp::runEnvelope(Voice& v, int adsr0, int adsr1, int gain) {
    int env = v.env;

    if (v.envMode == Release) {
        // Release: always decrements by 8, no counter gating
        if ((env -= 0x8) < 0)
            env = 0;
        v.env = env;
        return;
    }

    int rate;
    int envData = adsr1;

    if (adsr0 & 0x80) {
        // ADSR mode
        if (v.envMode >= Decay) {
            // Decay or Sustain: exponential decrease
            env--;
            env -= env >> 8;
            rate = envData & 0x1F;
            if (v.envMode == Decay)
                rate = (adsr0 >> 3 & 0x0E) + 0x10;
        } else {
            // Attack
            rate = (adsr0 & 0x0F) * 2 + 1;
            env += rate < 31 ? 0x20 : 0x400;
        }
    } else {
        // GAIN mode
        envData = gain;
        int mode = envData >> 5;
        if (mode < 4) {
            // Direct set
            env  = envData * 0x10;
            rate = 31;
        } else {
            rate = envData & 0x1F;
            if (mode == 4) {
                // Linear decrease
                env -= 0x20;
            } else if (mode < 6) {
                // Exponential decrease
                env--;
                env -= env >> 8;
            } else {
                // Linear increase (mode 6) or bent-line increase (mode 7)
                env += 0x20;
                if (mode > 6 && static_cast<unsigned>(v.hiddenEnv) >= 0x600)
                    env += 0x8 - 0x20;  // Bent line: slower above 0x600
            }
        }
    }

    // Sustain level check (ADSR mode): transition decay → sustain
    if ((env >> 8) == (envData >> 5) && v.envMode == Decay)
        v.envMode = Sustain;

    v.hiddenEnv = env;

    // Clamp to 0x000-0x7FF
    if (static_cast<unsigned>(env) > 0x7FF) {
        env = (env < 0 ? 0 : 0x7FF);
        if (v.envMode == Attack)
            v.envMode = Decay;
    }

    // Only update envelope when counter fires
    if (readCounter(rate))
        v.env = env;
}

// Voice pipeline operations. Adjacent voices use the same latches at staggered
// phases, so sampling a register early must not read it again at a later phase.

void Dsp::captureDirectory(unsigned voice) {
    pipe_.directoryAddress = uint16_t(unsigned(pipe_.directoryPage) * 256 + unsigned(pipe_.sourceNumber) * 4);
    pipe_.sourceNumber = vreg(voice, kSrcn);
}

void Dsp::captureVoiceRegisters(unsigned voice) {
    const uint16_t entry = uint16_t(pipe_.directoryAddress + (voices_[voice].konDelay ? 0 : 2));
    pipe_.nextBrrAddress = ramReadWord(entry);
    pipe_.adsr0 = vreg(voice, kAdsr0);
    pipe_.pitch = vreg(voice, kPitchL);
}

void Dsp::capturePitchHigh(unsigned voice) {
    pipe_.pitch += (vreg(voice, kPitchH) & 0x3f) * 256;
}

void Dsp::captureBrrBytes(unsigned voice) {
    const auto& v = voices_[voice];
    pipe_.brrByte = ramRead(uint16_t(v.brrAddr + v.brrOffset));
    pipe_.brrHeader = ramRead(uint16_t(v.brrAddr));
}

void Dsp::renderVoice(unsigned voice) {
    auto& v = voices_[voice];
    const unsigned bit = 1u << voice;
    if (pipe_.pitchMod & bit) pipe_.pitch += ((pipe_.output >> 5) * pipe_.pitch) >> 10;
    if (v.konDelay) {
        if (v.konDelay == 5) {
            v.brrAddr = pipe_.nextBrrAddress;
            v.brrOffset = 1;
            v.bufPos = 0;
            pipe_.brrHeader = 0;
        }
        v.env = v.hiddenEnv = 0;
        --v.konDelay;
        v.interpPos = (v.konDelay & 3) ? 0x4000 : 0;
        pipe_.pitch = 0;
    }

    const int sample = (pipe_.noiseEnable & bit) ? int(int16_t(noise_ * 2)) : interpolate(v);
    pipe_.output = ((sample * v.env) >> 11) & ~1;
    v.output = pipe_.output;
    v.envSnapshot = uint8_t(v.env >> 4);

    if ((reg(kFlg) & 0x80) || (pipe_.brrHeader & 3) == 1) {
        v.envMode = Release;
        v.env = 0;
    }
    if (everyOther_) {
        if (pipe_.keyOff & bit) v.envMode = Release;
        if (kon_ & bit) { v.konDelay = 5; v.envMode = Attack; }
    }
    if (!v.konDelay) runEnvelope(v, pipe_.adsr0, vreg(voice, kAdsr1), vreg(voice, kGain));
}

void Dsp::prepareVoice(unsigned voice) {
    capturePitchHigh(voice);
    captureBrrBytes(voice);
    renderVoice(voice);
}

void Dsp::mixVoice(unsigned voice, unsigned channel) {
    const int scaled = (pipe_.output * int8_t(vreg(voice, kVolL + channel))) >> 7;
    pipe_.mainMix[channel] = clamp16(pipe_.mainMix[channel] + scaled);
    if (pipe_.echoEnable & (1u << voice)) pipe_.echoMix[channel] = clamp16(pipe_.echoMix[channel] + scaled);
}

void Dsp::decodeAndMixLeft(unsigned voice) {
    auto& v = voices_[voice];
    pipe_.looped = 0;
    if (v.interpPos >= 0x4000) {
        decodeBrr(v, pipe_.brrHeader, pipe_.brrByte, ramRead(uint16_t(v.brrAddr + v.brrOffset + 1)));
        v.brrOffset += 2;
        if (v.brrOffset == BrrBlockSize) {
            v.brrAddr = (v.brrAddr + BrrBlockSize) & 0xffff;
            if (pipe_.brrHeader & 1) {
                v.brrAddr = pipe_.nextBrrAddress;
                pipe_.looped = 1 << voice;
            }
            v.brrOffset = 1;
        }
    }
    const int advanced = (v.interpPos & 0x3fff) + pipe_.pitch;
    v.interpPos = advanced > 0x7fff ? 0x7fff : advanced;
    mixVoice(voice, 0);
}

void Dsp::mixRightAndCaptureEnd(unsigned voice) {
    mixVoice(voice, 1);
    endxBuf_ = uint8_t(reg(kEndx) | pipe_.looped);
    if (voices_[voice].konDelay == 5) endxBuf_ &= uint8_t(~(1u << voice));
}

void Dsp::captureOutput() { outxBuf_ = uint8_t(pipe_.output >> 8); }
void Dsp::publishEndAndCaptureEnv(unsigned voice) {
    regs_[kEndx] = endxBuf_;
    envxBuf_ = voices_[voice].envSnapshot;
}
void Dsp::publishOutput(unsigned voice) { regs_[voice * 16 + kOutX] = outxBuf_; }
void Dsp::publishEnvelope(unsigned voice) { regs_[voice * 16 + kEnvX] = envxBuf_; }

void Dsp::clockGlobalLatches() {
    switch (phase_) {
    case 27: pipe_.pitchMod = reg(kPmon) & 0xfe; break;
    case 28:
        pipe_.noiseEnable = reg(kNon);
        pipe_.echoEnable = reg(kEon);
        pipe_.directoryPage = reg(kDir);
        break;
    case 29:
        everyOther_ ^= 1;
        if (everyOther_) newKon_ &= uint8_t(~kon_);
        break;
    case 30:
        if (everyOther_) { kon_ = newKon_; pipe_.keyOff = reg(kKoff); }
        runCounters();
        if (readCounter(reg(kFlg) & 31)) {
            const int feedback = (noise_ ^ (noise_ >> 1)) & 1;
            noise_ = (noise_ >> 1) | (feedback << 14);
        }
        break;
    }
}

void Dsp::readEcho(unsigned channel) {
    const int sample = int16_t(ramReadWord(uint16_t(pipe_.echoAddress + channel * 2)));
    echoHist_[echoHistIdx_][channel] = sample >> 1;
    echoHist_[echoHistIdx_ + EchoHistSize][channel] = sample >> 1;
}

void Dsp::writeEcho(unsigned channel) {
    if (!(pipe_.echoFlags & 0x20)) ramWriteWord(uint16_t(pipe_.echoAddress + channel * 2), int16_t(pipe_.echoMix[channel]));
    pipe_.echoMix[channel] = 0;
}

int Dsp::firTap(unsigned tap, unsigned channel) const {
    return (echoHist_[echoHistIdx_ + tap + 1][channel] * int8_t(reg(0x0f + tap * 16))) >> 6;
}

int Dsp::mixFinal(unsigned channel) const {
    const int dry = int16_t((pipe_.mainMix[channel] * int8_t(reg(kMVolL + channel * 16))) >> 7);
    const int wet = int16_t((pipe_.filteredEcho[channel] * int8_t(reg(kEVolL + channel * 16))) >> 7);
    return clamp16(dry + wet);
}

void Dsp::clockEcho() {
    switch (phase_) {
    case 22:
        echoHistIdx_ = (echoHistIdx_ + 1) % EchoHistSize;
        pipe_.echoAddress = uint16_t(unsigned(pipe_.echoPage) * 256 + echoOffset_);
        readEcho(0);
        for (unsigned ch = 0; ch < 2; ++ch) pipe_.filteredEcho[ch] = firTap(0, ch);
        break;
    case 23:
        for (unsigned ch = 0; ch < 2; ++ch) pipe_.filteredEcho[ch] += firTap(1, ch) + firTap(2, ch);
        readEcho(1);
        break;
    case 24:
        for (unsigned ch = 0; ch < 2; ++ch) pipe_.filteredEcho[ch] += firTap(3, ch) + firTap(4, ch) + firTap(5, ch);
        break;
    case 25:
        for (unsigned ch = 0; ch < 2; ++ch) {
            const int first = int16_t(pipe_.filteredEcho[ch] + firTap(6, ch));
            pipe_.filteredEcho[ch] = clamp16(first + int16_t(firTap(7, ch))) & ~1;
        }
        break;
    case 26:
        pipe_.mainMix[0] = mixFinal(0);
        for (unsigned ch = 0; ch < 2; ++ch) {
            const int feedback = int16_t((pipe_.filteredEcho[ch] * int8_t(reg(kEfb))) >> 7);
            pipe_.echoMix[ch] = clamp16(pipe_.echoMix[ch] + feedback) & ~1;
        }
        break;
    case 27: {
        const int left = (reg(kFlg) & 0x40) ? 0 : pipe_.mainMix[0];
        const int right = (reg(kFlg) & 0x40) ? 0 : mixFinal(1);
        pipe_.mainMix = {};
        if (outBuf_ && samplesWritten_ < outBufSize_) {
            outBuf_[samplesWritten_ * 2] = int16_t(left);
            outBuf_[samplesWritten_ * 2 + 1] = int16_t(right);
            ++samplesWritten_;
        }
        break;
    }
    case 28: pipe_.echoFlags = reg(kFlg); break;
    case 29:
        pipe_.echoPage = reg(kEsa);
        if (!echoOffset_) echoLength_ = (reg(kEdl) & 15) * 0x800;
        echoOffset_ += 4;
        if (echoOffset_ >= echoLength_) echoOffset_ = 0;
        writeEcho(0);
        pipe_.echoFlags = reg(kFlg);
        break;
    case 30: writeEcho(1); break;
    }
}

void Dsp::Tick() {
    // Five regularly spaced groups overlap three neighboring voices. The last
    // groups leave room for echo; voice zero's preparation crosses that interval.
    if (phase_ >= 2 && phase_ <= 16) {
        const unsigned group = (phase_ - 2) / 3;
        switch ((phase_ - 2) % 3) {
        case 0:
            publishEndAndCaptureEnv(group);
            captureDirectory(group + 3);
            decodeAndMixLeft(group + 1);
            break;
        case 1:
            publishOutput(group);
            mixRightAndCaptureEnd(group + 1);
            captureVoiceRegisters(group + 2);
            break;
        case 2:
            publishEnvelope(group);
            captureOutput();
            prepareVoice(group + 2);
            break;
        }
    } else {
        switch (phase_) {
        case 0: mixRightAndCaptureEnd(0); captureVoiceRegisters(1); break;
        case 1: captureOutput(); prepareVoice(1); break;
        case 17: captureDirectory(0); publishEndAndCaptureEnv(5); decodeAndMixLeft(6); break;
        case 18: publishOutput(5); mixRightAndCaptureEnd(6); captureVoiceRegisters(7); break;
        case 19: publishEnvelope(5); captureOutput(); prepareVoice(7); break;
        case 20: captureDirectory(1); publishEndAndCaptureEnv(6); decodeAndMixLeft(7); break;
        case 21: publishOutput(6); mixRightAndCaptureEnd(7); captureVoiceRegisters(0); break;
        case 22: capturePitchHigh(0); publishEnvelope(6); captureOutput(); break;
        case 23: publishEndAndCaptureEnv(7); break;
        case 24: publishOutput(7); break;
        case 25: captureBrrBytes(0); publishEnvelope(7); break;
        case 30: clockGlobalLatches(); renderVoice(0); break;
        case 31: decodeAndMixLeft(0); captureDirectory(2); break;
        default: clockGlobalLatches(); break;
        }
        clockEcho();
    }
    phase_ = (phase_ + 1) & 31;
}

void Dsp::RunSample() {
    for (unsigned clocks = 0; clocks < 32; ++clocks) Tick();
}

} // namespace snes::core
