#pragma once
// Dsp1.hpp — DSP-1 (uPD77C25) coprocessor HLE
//
// High-level emulation of the DSP-1 math coprocessor used in SNES games
// like Super Mario Kart. Ported from bsnes's dsp1emu (by Overload, The
// Dumper, Neviksti, Andreas Naive).
//
// The cartridge board selects the data and status address windows.

#include <cstdint>
#include <cstring>

namespace snes::core {

class Dsp1 {
public:
    enum SrFlags { DRC = 0x04, DRS = 0x10, RQM = 0x80 };

    Dsp1();

    uint8_t GetSr() const;
    uint8_t GetDr();
    void    SetDr(uint8_t value);
    void    Reset();

private:
    enum FsmMajorState { WAIT_COMMAND, READ_DATA, WRITE_DATA };
    static constexpr int MAX_READS  = 7;
    static constexpr int MAX_WRITES = 1024;

    struct Command {
        void (Dsp1::*callback)(int16_t*, int16_t*);
        unsigned int reads;
        unsigned int writes;
    };

    static const Command kCommandTable[0x40];
    static const int16_t kMaxAZS_Exp[16];
    static const int16_t kSinTable[256];
    static const int16_t kMulTable[256];
    static const uint16_t kDataRom[1024];

    struct SharedData {
        int16_t MatrixA[3][3];
        int16_t MatrixB[3][3];
        int16_t MatrixC[3][3];
        int16_t CentreX, CentreY, CentreZ;
        int16_t CentreZ_C, CentreZ_E;
        int16_t VOffset;
        int16_t Les, C_Les, E_Les;
        int16_t SinAas, CosAas;
        int16_t SinAzs, CosAzs;
        int16_t SinAZS, CosAZS;
        int16_t SecAZS_C1, SecAZS_E1;
        int16_t SecAZS_C2, SecAZS_E2;
        int16_t Nx, Ny, Nz;
        int16_t Gx, Gy, Gz;
        int16_t Hx, Hy;
        int16_t Vx, Vy, Vz;
    } shared_;

    uint8_t  sr_;
    uint16_t dr_;
    unsigned fsmMajorState_;
    uint8_t  command_;
    uint8_t  dataCounter_;
    int16_t  readBuffer_[MAX_READS];
    int16_t  writeBuffer_[MAX_WRITES];
    bool     freeze_;

    void fsmStep(bool read, uint8_t& data);

    // DSP-1 commands
    void memoryTest(int16_t* input, int16_t* output);
    void memoryDump(int16_t* input, int16_t* output);
    void memorySize(int16_t* input, int16_t* output);
    void multiply(int16_t* input, int16_t* output);
    void multiply2(int16_t* input, int16_t* output);
    void inverseCmd(int16_t* input, int16_t* output);
    void triangle(int16_t* input, int16_t* output);
    void radius(int16_t* input, int16_t* output);
    void range(int16_t* input, int16_t* output);
    void range2(int16_t* input, int16_t* output);
    void distance(int16_t* input, int16_t* output);
    void rotate(int16_t* input, int16_t* output);
    void polar(int16_t* input, int16_t* output);
    void attitudeA(int16_t* input, int16_t* output);
    void attitudeB(int16_t* input, int16_t* output);
    void attitudeC(int16_t* input, int16_t* output);
    void objectiveA(int16_t* input, int16_t* output);
    void objectiveB(int16_t* input, int16_t* output);
    void objectiveC(int16_t* input, int16_t* output);
    void subjectiveA(int16_t* input, int16_t* output);
    void subjectiveB(int16_t* input, int16_t* output);
    void subjectiveC(int16_t* input, int16_t* output);
    void scalarA(int16_t* input, int16_t* output);
    void scalarB(int16_t* input, int16_t* output);
    void scalarC(int16_t* input, int16_t* output);
    void gyrate(int16_t* input, int16_t* output);
    void parameter(int16_t* input, int16_t* output);
    void raster(int16_t* input, int16_t* output);
    void target(int16_t* input, int16_t* output);
    void project(int16_t* input, int16_t* output);

    // Auxiliary math
    int16_t sinLut(int16_t angle);
    int16_t cosLut(int16_t angle);
    void inverse(int16_t coefficient, int16_t exponent, int16_t& iCoefficient, int16_t& iExponent);
    int16_t denormalizeAndClip(int16_t c, int16_t e);
    void normalize(int16_t m, int16_t& coefficient, int16_t& exponent);
    void normalizeDouble(int32_t product, int16_t& coefficient, int16_t& exponent);
    int16_t shiftR(int16_t c, int16_t e);
};

} // namespace snes::core
