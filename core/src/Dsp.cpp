// ============================================================================
// Dsp.cpp — SNES S-DSP Implementation
//
// Implements the complete SNES audio DSP in "fast" (batch) mode:
// all 8 voices processed per sample, then echo, then mix.
// Register writes take effect at sample boundaries.
//
// Reference: bsnes sfc/dsp/SPC_DSP.cpp
// ============================================================================

#include "snes/core/Dsp.hpp"
#include <cstring>

namespace snes::core {

// ============================================================================
// Static tables
// ============================================================================

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

// ============================================================================
// Constructor / Power / SoftReset
// ============================================================================

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

    SoftReset();
}

void Dsp::SoftReset() {
    regs_[kFlg] = 0xE0;  // Soft reset + mute + echo write disable
    noise_       = 0x4000;
    echoHistIdx_ = 0;
    everyOther_  = 1;
    echoOffset_  = 0;
    counter_     = 0;

    std::memset(echoHist_, 0, sizeof(echoHist_));
}

// ============================================================================
// Register access
// ============================================================================

uint8_t Dsp::Read(uint8_t addr) const {
    return regs_[addr & 0x7F];
}

void Dsp::Write(uint8_t addr, uint8_t data) {
    regs_[addr & 0x7F] = data;

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

// ============================================================================
// Output buffer management
// ============================================================================

void Dsp::SetOutput(int16_t* buffer, int maxSamples) {
    outBuf_     = buffer;
    outBufSize_ = maxSamples;
    samplesWritten_ = 0;
}

// ============================================================================
// Counter system
// ============================================================================

void Dsp::runCounters() {
    if (--counter_ < 0)
        counter_ = kSimpleCounterRange - 1;
}

bool Dsp::readCounter(int rate) const {
    return ((unsigned)counter_ + kCounterOffsets[rate]) % kCounterRates[rate] == 0;
}

// ============================================================================
// Helpers
// ============================================================================

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

// ============================================================================
// BRR decoding — decode 4 samples from 2 BRR data bytes
//
// header:    BRR block header byte (shift in bits 7-4, filter in bits 3-2)
// brrByte1:  first data byte (high nybbles)
// brrByte2:  second data byte (low nybbles)
// ============================================================================

void Dsp::decodeBrr(Voice& v, int header, int brrByte1, int brrByte2) {
    // Arrange 4 nybbles in 0xABCD order
    int nybbles = brrByte1 * 0x100 + brrByte2;

    // Write to next 4 samples in circular buffer
    int* pos = &v.buf[v.bufPos];
    if ((v.bufPos += 4) >= BrrBufSize)
        v.bufPos = 0;

    int const shift  = header >> 4;
    int const filter = header & 0x0C;

    for (int i = 0; i < 4; i++, nybbles <<= 4) {
        // Extract top nybble, sign-extend
        int s = static_cast<int16_t>(nybbles) >> 12;

        // Apply shift
        s = (s << shift) >> 1;
        if (shift >= 0x0D)
            s = (s >> 25) << 11;  // Invalid shift: clamp to sign

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

// ============================================================================
// Gaussian interpolation — 4-point interpolation using half-Gaussian table
// ============================================================================

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

// ============================================================================
// ADSR/GAIN envelope processing
// ============================================================================

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

// ============================================================================
// Echo processing — FIR filter, feedback, write, mix, output
// ============================================================================

void Dsp::processEcho(int mainOut[2], int echoOut[2]) {
    // --- echo_22: advance history, read left echo, start FIR ---
    echoHistIdx_ = (echoHistIdx_ + 1) % EchoHistSize;

    uint16_t echoAddr = static_cast<uint16_t>(
        (reg(kEsa) * 0x100 + echoOffset_) & 0xFFFF);

    // Read echo buffer into history (both copies for wrap)
    for (int ch = 0; ch < 2; ch++) {
        int16_t s = static_cast<int16_t>(ramReadWord(
            static_cast<uint16_t>(echoAddr + ch * 2)));
        echoHist_[echoHistIdx_][ch] = s >> 1;
        echoHist_[echoHistIdx_ + EchoHistSize][ch] = s >> 1;
    }

    // --- echo_22 through echo_25: 8-tap FIR filter ---
    // Compute FIR exactly as bsnes does (with int16_t truncation between taps 6 & 7)
    auto calcFir = [&](int i, int ch) -> int {
        return (echoHist_[echoHistIdx_ + i + 1][ch]
                * static_cast<int8_t>(reg(0x0F + i * 0x10))) >> 6;
    };

    int echoIn[2];
    for (int ch = 0; ch < 2; ch++) {
        int sum = calcFir(0, ch);                                   // echo_22
        sum += calcFir(1, ch) + calcFir(2, ch);                    // echo_23
        sum += calcFir(3, ch) + calcFir(4, ch) + calcFir(5, ch);  // echo_24
        sum += calcFir(6, ch);                                      // echo_25
        sum  = static_cast<int16_t>(sum);                           // Truncate
        sum += static_cast<int16_t>(calcFir(7, ch));               // Last tap
        sum  = clamp16(sum);
        echoIn[ch] = sum & ~1;
    }

    // --- echo_26: mix output + echo feedback ---
    int finalOut[2];
    for (int ch = 0; ch < 2; ch++) {
        // Main output: (mainOut * MVOL >> 7) + (echoIn * EVOL >> 7)
        int8_t mvol = static_cast<int8_t>(reg(ch == 0 ? kMVolL : kMVolR));
        int8_t evol = static_cast<int8_t>(reg(ch == 0 ? kEVolL : kEVolR));
        int out = static_cast<int16_t>((mainOut[ch] * mvol) >> 7)
                + static_cast<int16_t>((echoIn[ch]  * evol) >> 7);
        finalOut[ch] = clamp16(out);
    }

    // Echo feedback
    int8_t efb = static_cast<int8_t>(reg(kEfb));
    for (int ch = 0; ch < 2; ch++) {
        int sum = echoOut[ch]
                + static_cast<int16_t>((echoIn[ch] * efb) >> 7);
        sum = clamp16(sum);
        echoOut[ch] = sum & ~1;
    }

    // --- echo_27: apply global mute ---
    if (reg(kFlg) & 0x40) {
        finalOut[0] = 0;
        finalOut[1] = 0;
    }

    // --- echo_29: advance echo offset, write echo ---
    if (!echoOffset_)
        echoLength_ = (reg(kEdl) & 0x0F) * 0x800;

    echoOffset_ += 4;
    if (echoOffset_ >= echoLength_)
        echoOffset_ = 0;

    // Write echo to buffer (unless write-protected: FLG bit 5)
    if (!(reg(kFlg) & 0x20)) {
        for (int ch = 0; ch < 2; ch++) {
            ramWriteWord(static_cast<uint16_t>(echoAddr + ch * 2),
                         static_cast<int16_t>(echoOut[ch]));
        }
    }

    // --- Write to output buffer ---
    if (outBuf_ && samplesWritten_ < outBufSize_) {
        outBuf_[samplesWritten_ * 2 + 0] = static_cast<int16_t>(finalOut[0]);
        outBuf_[samplesWritten_ * 2 + 1] = static_cast<int16_t>(finalOut[1]);
        samplesWritten_++;
    }
}

// ============================================================================
// RunSample — process all 8 voices + echo + output for one stereo sample
// ============================================================================

void Dsp::RunSample() {
    // === misc_27-28: read shared registers ===
    int tPmon = reg(kPmon) & 0xFE;  // Voice 0 cannot be pitch-modulated
    int tNon  = reg(kNon);
    int tEon  = reg(kEon);
    int tDir  = reg(kDir);

    // === misc_29: toggle every-other-sample flag ===
    if ((everyOther_ ^= 1) != 0)
        newKon_ &= ~kon_;

    // === misc_30: latch KON/KOFF, run counters, noise ===
    int tKoff = 0;
    if (everyOther_) {
        kon_  = newKon_;
        tKoff = reg(kKoff);
    }

    runCounters();

    // Noise LFSR update
    if (readCounter(reg(kFlg) & 0x1F)) {
        int feedback = (noise_ << 13) ^ (noise_ << 14);
        noise_ = (feedback & 0x4000) ^ (noise_ >> 1);
    }

    // === Process all 8 voices ===
    int mainOut[2] = {0, 0};
    int echoOut[2] = {0, 0};

    for (int vi = 0; vi < VoiceCount; vi++) {
        Voice& v = voices_[vi];
        int vbit = 1 << vi;

        // --- V1/V2: read source directory, pitch, adsr0 ---
        int tSrcn   = vreg(vi, kSrcn);
        int tDirAddr = (tDir * 0x100 + tSrcn * 4) & 0xFFFF;

        // Read sample start/loop address
        uint16_t entryAddr = static_cast<uint16_t>(tDirAddr);
        if (!v.konDelay)
            entryAddr += 2;  // Use loop address when not starting
        int tBrrNextAddr = ramReadWord(entryAddr);

        int tAdsr0 = vreg(vi, kAdsr0);
        int tPitch = vreg(vi, kPitchL);

        // --- V3a: complete pitch ---
        tPitch += (vreg(vi, kPitchH) & 0x3F) << 8;

        // --- V3b: read BRR header and data byte ---
        int tBrrHeader = ramRead(static_cast<uint16_t>(v.brrAddr));
        int tBrrByte   = ramRead(static_cast<uint16_t>((v.brrAddr + v.brrOffset) & 0xFFFF));

        // --- V3c: main voice processing ---

        // Pitch modulation (using previous voice's output)
        if (tPmon & vbit)
            tPitch += ((voices_[vi > 0 ? vi - 1 : 0].output >> 5) * tPitch) >> 10;

        // KON delay processing
        if (v.konDelay) {
            if (v.konDelay == 5) {
                v.brrAddr   = tBrrNextAddr;
                v.brrOffset = 1;
                v.bufPos    = 0;
                tBrrHeader  = 0;  // Header ignored on first sample
            }

            v.env       = 0;
            v.hiddenEnv = 0;
            v.interpPos = 0;
            if (--v.konDelay & 3)
                v.interpPos = 0x4000;

            tPitch = 0;
        }

        // Gaussian interpolation
        int output = interpolate(v);

        // Noise substitution
        if (tNon & vbit)
            output = static_cast<int16_t>(noise_ * 2);

        // Apply envelope to output
        int tOutput = (output * v.env) >> 11 & ~1;
        uint8_t envxOut = static_cast<uint8_t>(v.env >> 4);

        // Immediate silence on end-of-sample or soft reset
        if ((reg(kFlg) & 0x80) || (tBrrHeader & 3) == 1) {
            v.envMode = Release;
            v.env     = 0;
        }

        // KON/KOFF (on every other sample)
        if (everyOther_) {
            if (tKoff & vbit)
                v.envMode = Release;
            if (kon_ & vbit) {
                v.konDelay = 5;
                v.envMode  = Attack;
            }
        }

        // Run envelope for next sample
        if (!v.konDelay) {
            runEnvelope(v, tAdsr0, vreg(vi, kAdsr1), vreg(vi, kGain));
        }

        // --- V4: BRR decode + advance interpPos + output left ---
        int tLooped = 0;
        if (v.interpPos >= 0x4000) {
            int brrByte2 = ramRead(
                static_cast<uint16_t>((v.brrAddr + v.brrOffset + 1) & 0xFFFF));
            decodeBrr(v, tBrrHeader, tBrrByte, brrByte2);

            v.brrOffset += 2;
            if (v.brrOffset >= BrrBlockSize) {
                // Advance to next BRR block
                v.brrAddr = (v.brrAddr + BrrBlockSize) & 0xFFFF;
                if (tBrrHeader & 1) {
                    // Loop/end: jump to loop address
                    v.brrAddr = tBrrNextAddr;
                    tLooped   = vbit;
                }
                v.brrOffset = 1;
            }
        }

        // Advance interpolation position
        v.interpPos = (v.interpPos & 0x3FFF) + tPitch;
        if (v.interpPos > 0x7FFF)
            v.interpPos = 0x7FFF;

        // Voice output: apply volume and accumulate to main/echo sums
        for (int ch = 0; ch < 2; ch++) {
            int amp = (tOutput * static_cast<int8_t>(vreg(vi, kVolL + ch))) >> 7;
            mainOut[ch] += amp;
            mainOut[ch] = clamp16(mainOut[ch]);

            if (tEon & vbit) {
                echoOut[ch] += amp;
                echoOut[ch] = clamp16(echoOut[ch]);
            }
        }

        // --- V5-V9: update registers ---
        // ENDX
        int endx = regs_[kEndx] | tLooped;
        if (v.konDelay == 5)
            endx &= ~vbit;
        regs_[kEndx] = static_cast<uint8_t>(endx);

        // OUTX (signed 8-bit: output >> 8)
        regs_[vi * 0x10 + kOutX] = static_cast<uint8_t>(tOutput >> 8);

        // ENVX
        regs_[vi * 0x10 + kEnvX] = envxOut;

        // Store output for pitch modulation of next voice
        v.output = tOutput;
    }

    // === Echo processing + mixing + output ===
    processEcho(mainOut, echoOut);
}

} // namespace snes::core
