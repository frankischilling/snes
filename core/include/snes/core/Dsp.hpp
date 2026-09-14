// snes emulator
// core/include/snes/core/Dsp.hpp
// S-DSP audio state and processing interface.

// Dsp.hpp — SNES S-DSP (Sony S-DSP / μPD77C25)
//
// The DSP is responsible for all SNES audio output:
//   - 8 voices, each playing BRR-compressed samples
//   - ADSR/GAIN envelope per voice
//   - Gaussian interpolation
//   - Per-voice pitch modulation and noise
//   - 8-tap FIR echo filter with feedback
//   - Stereo mixing at ~32 kHz
//
// Register map: 128 bytes (0x00-0x7F)
//   - 8 voices × 16 bytes each (with 6 unused per voice)
//   - Global registers at specific offsets
//
// This is a "fast" (batch) DSP — all 8 voices are processed per sample,
// with register writes taking effect at sample boundaries.
//
// Reference: bsnes sfc/dsp/SPC_DSP.h, sfc/dsp/SPC_DSP.cpp

#pragma once

#include <array>
#include <cstdint>

namespace snes::core {

class Dsp {
public:
    // Constants
    static constexpr int RegisterCount = 128;
    static constexpr int VoiceCount    = 8;
    static constexpr int BrrBufSize    = 12;   // Decoded sample buffer (3 groups of 4)
    static constexpr int BrrBlockSize  = 9;    // 1 header + 8 data bytes
    static constexpr int EchoHistSize  = 8;    // FIR tap count

    // Per-voice register offsets (within each voice's 16-byte block)
    static constexpr uint8_t kVolL   = 0x00;
    static constexpr uint8_t kVolR   = 0x01;
    static constexpr uint8_t kPitchL = 0x02;
    static constexpr uint8_t kPitchH = 0x03;
    static constexpr uint8_t kSrcn   = 0x04;
    static constexpr uint8_t kAdsr0  = 0x05;
    static constexpr uint8_t kAdsr1  = 0x06;
    static constexpr uint8_t kGain   = 0x07;
    static constexpr uint8_t kEnvX   = 0x08;  // Read-only (DSP writes)
    static constexpr uint8_t kOutX   = 0x09;  // Read-only (DSP writes)

    // Global register addresses
    static constexpr uint8_t kMVolL = 0x0C;
    static constexpr uint8_t kMVolR = 0x1C;
    static constexpr uint8_t kEVolL = 0x2C;
    static constexpr uint8_t kEVolR = 0x3C;
    static constexpr uint8_t kKon   = 0x4C;
    static constexpr uint8_t kKoff  = 0x5C;
    static constexpr uint8_t kFlg   = 0x6C;
    static constexpr uint8_t kEndx  = 0x7C;
    static constexpr uint8_t kEfb   = 0x0D;
    static constexpr uint8_t kPmon  = 0x2D;
    static constexpr uint8_t kNon   = 0x3D;
    static constexpr uint8_t kEon   = 0x4D;
    static constexpr uint8_t kDir   = 0x5D;
    static constexpr uint8_t kEsa   = 0x6D;
    static constexpr uint8_t kEdl   = 0x7D;
    // FIR coefficients at 0x0F, 0x1F, 0x2F, 0x3F, 0x4F, 0x5F, 0x6F, 0x7F

    // Envelope modes
    enum EnvMode { Release = 0, Attack = 1, Decay = 2, Sustain = 3 };

    // Voice state
    struct Voice {
        int      buf[BrrBufSize * 2]{};  // Decoded BRR samples (doubled for wrap)
        int      bufPos    = 0;          // Write position in decode buffer
        int      interpPos = 0;          // Fractional position (12.4 fixed-point)
        int      brrAddr   = 0;          // Address of current BRR block in RAM
        int      brrOffset = 1;          // Offset within BRR block (1-based)
        int      konDelay  = 0;          // KON setup phase (5..0)
        EnvMode  envMode   = Release;
        int      env       = 0;          // Current envelope (0x000-0x7FF)
        int      hiddenEnv = 0;          // For bent-line GAIN mode 7
        int      output    = 0;          // Last voice output (for pitch mod)
    };

    // Public interface
    Dsp();

    // Register access (called by SMP for $F3 DSPDATA)
    uint8_t Read(uint8_t addr) const;
    void    Write(uint8_t addr, uint8_t data);

    // Generate one stereo sample (~32 kHz).
    // Call once per 32 DSP clocks (= once per SMP sample period).
    void    RunSample();

    // Set pointer to SMP's 64 KB RAM (DSP reads BRR/directory/echo from here)
    void    SetRam(uint8_t* ram) { ram_ = ram; }

    // Power-on full reset
    void    Power();

    // Soft reset (FLG bit 7)
    void    SoftReset();

    // Output buffer management
    void    SetOutput(int16_t* buffer, int maxSamples);
    int     SamplesWritten() const { return samplesWritten_; }
    void    ResetSamplesWritten() { samplesWritten_ = 0; }

    // Direct state access (for testing / inspection)
    const std::array<uint8_t, RegisterCount>& Regs() const { return regs_; }
    std::array<uint8_t, RegisterCount>&       Regs()       { return regs_; }

    const Voice& GetVoice(int i) const { return voices_[i]; }
    Voice&       GetVoice(int i)       { return voices_[i]; }

    int  Noise()   const { return noise_; }
    int  Counter() const { return counter_; }

private:
    // Internal processing
    void decodeBrr(Voice& v, int header, int brrByte1, int brrByte2);
    int  interpolate(const Voice& v) const;
    void runEnvelope(Voice& v, int adsr0, int adsr1, int gain);
    void processEcho(int mainOut[2], int echoOut[2]);

    // Counter system (shared rate counter for envelopes/noise)
    void runCounters();
    bool readCounter(int rate) const;

    // Helpers
    static int clamp16(int value);
    uint8_t    ramRead(uint16_t addr) const;
    uint16_t   ramReadWord(uint16_t addr) const;
    void       ramWriteWord(uint16_t addr, int16_t value);

    uint8_t reg(int addr) const { return regs_[addr]; }
    uint8_t vreg(int voice, int offset) const { return regs_[voice * 0x10 + offset]; }

    // State
    std::array<uint8_t, RegisterCount> regs_{};
    Voice voices_[VoiceCount]{};

    uint8_t* ram_ = nullptr;

    // KON buffering
    uint8_t newKon_     = 0;
    uint8_t kon_        = 0;
    int     everyOther_ = 1;    // Toggles each sample (bsnes starts at 1)

    // Noise LFSR (15-bit)
    int     noise_      = 0x4000;

    // Shared rate counter (decrements from 30719 to 0, wrapping)
    int     counter_    = 0;

    // Echo state
    int     echoHist_[EchoHistSize * 2][2]{};  // Doubled for wrap-around
    int     echoHistIdx_ = 0;                   // Current position (0..7)
    int     echoOffset_  = 0;                   // Byte offset in echo buffer
    int     echoLength_  = 0;                   // Echo buffer size in bytes

    // Output buffer
    int16_t* outBuf_         = nullptr;
    int      outBufSize_     = 0;
    int      samplesWritten_ = 0;

    // Buffered register values (for ENVX/OUTX/ENDX write timing)
    uint8_t endxBuf_ = 0;
    uint8_t envxBuf_ = 0;
    uint8_t outxBuf_ = 0;
};

} // namespace snes::core
