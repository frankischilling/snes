// Dsp1.cpp — DSP-1 (uPD77C25) coprocessor HLE implementation
//
// Ported from bsnes dsp1emu.cpp.
// Original research by Overload, The Dumper, Neviksti, Andreas Naive.

#include "snes/core/Dsp1.hpp"

namespace snes::core {

// Construction / Reset

Dsp1::Dsp1() { Reset(); }

void Dsp1::Reset() {
    sr_ = DRC | RQM;
    dr_ = 0x0080;
    freeze_ = false;
    fsmMajorState_ = WAIT_COMMAND;
    std::memset(&shared_, 0, sizeof(SharedData));
}

// Register access

uint8_t Dsp1::GetSr() const {
    // The cartridge exposes the status byte containing the ready flag.
    return sr_;
}

uint8_t Dsp1::GetDr() {
    uint8_t data;
    fsmStep(true, data);
    return data;
}

void Dsp1::SetDr(uint8_t value) {
    fsmStep(false, value);
}

// Finite State Machine

void Dsp1::fsmStep(bool read, uint8_t& data) {
    if (0 == (sr_ & RQM)) return;

    // Binding between external byte and internal 16-bit DR
    if (read) {
        if (sr_ & DRS)
            data = static_cast<uint8_t>(dr_ >> 8);
        else
            data = static_cast<uint8_t>(dr_);
    } else {
        if (sr_ & DRS) {
            dr_ &= 0x00ff;
            dr_ |= static_cast<uint16_t>(data) << 8;
        } else {
            dr_ &= 0xff00;
            dr_ |= data;
        }
    }

    switch (fsmMajorState_) {
    case WAIT_COMMAND:
        command_ = static_cast<uint8_t>(dr_);
        if (!(command_ & 0xc0)) {
            switch (command_) {
            case 0x1a:
            case 0x2a:
            case 0x3a:
                freeze_ = true;
                break;
            default:
                dataCounter_ = 0;
                fsmMajorState_ = READ_DATA;
                sr_ &= ~DRC;
                break;
            }
        }
        break;

    case READ_DATA:
        sr_ ^= DRS;
        if (!(sr_ & DRS)) {
            readBuffer_[dataCounter_++] = static_cast<int16_t>(dr_);
            if (dataCounter_ >= kCommandTable[command_].reads) {
                (this->*kCommandTable[command_].callback)(readBuffer_, writeBuffer_);
                if (0 != kCommandTable[command_].writes) {
                    dataCounter_ = 0;
                    dr_ = static_cast<uint16_t>(writeBuffer_[dataCounter_]);
                    fsmMajorState_ = WRITE_DATA;
                } else {
                    dr_ = 0x0080;
                    fsmMajorState_ = WAIT_COMMAND;
                    sr_ |= DRC;
                }
            }
        }
        break;

    case WRITE_DATA:
        sr_ ^= DRS;
        if (!(sr_ & DRS)) {
            ++dataCounter_;
            if (dataCounter_ >= kCommandTable[command_].writes) {
                if ((command_ == 0x0a) && (dr_ != 0x8000)) {
                    readBuffer_[0]++;
                    (this->*kCommandTable[command_].callback)(readBuffer_, writeBuffer_);
                    dataCounter_ = 0;
                    dr_ = static_cast<uint16_t>(writeBuffer_[dataCounter_]);
                } else {
                    dr_ = 0x0080;
                    fsmMajorState_ = WAIT_COMMAND;
                    sr_ |= DRC;
                }
            } else {
                dr_ = static_cast<uint16_t>(writeBuffer_[dataCounter_]);
            }
        }
        break;
    }

    if (freeze_)
        sr_ &= ~RQM;
}

// Command table

const Dsp1::Command Dsp1::kCommandTable[0x40] = {
    {&Dsp1::multiply, 2, 1},       // 0x00
    {&Dsp1::attitudeA, 4, 0},      // 0x01
    {&Dsp1::parameter, 7, 4},      // 0x02
    {&Dsp1::subjectiveA, 3, 3},    // 0x03
    {&Dsp1::triangle, 2, 2},       // 0x04
    {&Dsp1::attitudeA, 4, 0},      // 0x05
    {&Dsp1::project, 3, 3},        // 0x06
    {&Dsp1::memoryTest, 1, 1},     // 0x07
    {&Dsp1::radius, 3, 2},         // 0x08
    {&Dsp1::objectiveA, 3, 3},     // 0x09
    {&Dsp1::raster, 1, 4},         // 0x0a
    {&Dsp1::scalarA, 3, 1},        // 0x0b
    {&Dsp1::rotate, 3, 2},         // 0x0c
    {&Dsp1::objectiveA, 3, 3},     // 0x0d
    {&Dsp1::target, 2, 2},         // 0x0e
    {&Dsp1::memoryTest, 1, 1},     // 0x0f

    {&Dsp1::inverseCmd, 2, 2},     // 0x10
    {&Dsp1::attitudeB, 4, 0},      // 0x11
    {&Dsp1::parameter, 7, 4},      // 0x12
    {&Dsp1::subjectiveB, 3, 3},    // 0x13
    {&Dsp1::gyrate, 6, 3},         // 0x14
    {&Dsp1::attitudeB, 4, 0},      // 0x15
    {&Dsp1::project, 3, 3},        // 0x16
    {&Dsp1::memoryDump, 1, 1024},  // 0x17
    {&Dsp1::range, 4, 1},          // 0x18
    {&Dsp1::objectiveB, 3, 3},     // 0x19
    {nullptr, 0, 0},               // 0x1a (freeze)
    {&Dsp1::scalarB, 3, 1},        // 0x1b
    {&Dsp1::polar, 6, 3},          // 0x1c
    {&Dsp1::objectiveB, 3, 3},     // 0x1d
    {&Dsp1::target, 2, 2},         // 0x1e
    {&Dsp1::memoryDump, 1, 1024},  // 0x1f

    {&Dsp1::multiply2, 2, 1},      // 0x20
    {&Dsp1::attitudeC, 4, 0},      // 0x21
    {&Dsp1::parameter, 7, 4},      // 0x22
    {&Dsp1::subjectiveC, 3, 3},    // 0x23
    {&Dsp1::triangle, 2, 2},       // 0x24
    {&Dsp1::attitudeC, 4, 0},      // 0x25
    {&Dsp1::project, 3, 3},        // 0x26
    {&Dsp1::memorySize, 1, 1},     // 0x27
    {&Dsp1::distance, 3, 1},       // 0x28
    {&Dsp1::objectiveC, 3, 3},     // 0x29
    {nullptr, 0, 0},               // 0x2a (freeze)
    {&Dsp1::scalarC, 3, 1},        // 0x2b
    {&Dsp1::rotate, 3, 2},         // 0x2c
    {&Dsp1::objectiveC, 3, 3},     // 0x2d
    {&Dsp1::target, 2, 2},         // 0x2e
    {&Dsp1::memorySize, 1, 1},     // 0x2f

    {&Dsp1::inverseCmd, 2, 2},     // 0x30
    {&Dsp1::attitudeA, 4, 0},      // 0x31
    {&Dsp1::parameter, 7, 4},      // 0x32
    {&Dsp1::subjectiveA, 3, 3},    // 0x33
    {&Dsp1::gyrate, 6, 3},         // 0x34
    {&Dsp1::attitudeA, 4, 0},      // 0x35
    {&Dsp1::project, 3, 3},        // 0x36
    {&Dsp1::memoryDump, 1, 1024},  // 0x37
    {&Dsp1::range2, 4, 1},         // 0x38
    {&Dsp1::objectiveA, 3, 3},     // 0x39
    {nullptr, 0, 0},               // 0x3a (freeze)
    {&Dsp1::scalarA, 3, 1},        // 0x3b
    {&Dsp1::polar, 6, 3},          // 0x3c
    {&Dsp1::objectiveA, 3, 3},     // 0x3d
    {&Dsp1::target, 2, 2},         // 0x3e
    {&Dsp1::memoryDump, 1, 1024},  // 0x3f
};

// DSP-1 Commands

void Dsp1::memoryTest(int16_t* input, int16_t* output) {
    output[0] = 0x0000;
}

void Dsp1::memoryDump(int16_t* input, int16_t* output) {
    std::memcpy(output, kDataRom, 1024 * sizeof(int16_t));
}

void Dsp1::memorySize(int16_t* input, int16_t* output) {
    output[0] = 0x0100;
}

void Dsp1::multiply(int16_t* input, int16_t* output) {
    int16_t& Multiplicand = input[0];
    int16_t& Multiplier = input[1];
    output[0] = static_cast<int16_t>(Multiplicand * Multiplier >> 15);
}

void Dsp1::multiply2(int16_t* input, int16_t* output) {
    int16_t& Multiplicand = input[0];
    int16_t& Multiplier = input[1];
    output[0] = static_cast<int16_t>((Multiplicand * Multiplier >> 15) + 1);
}

void Dsp1::inverseCmd(int16_t* input, int16_t* output) {
    inverse(input[0], input[1], output[0], output[1]);
}

void Dsp1::triangle(int16_t* input, int16_t* output) {
    int16_t& Angle  = input[0];
    int16_t& Radius = input[1];
    output[0] = static_cast<int16_t>(sinLut(Angle) * Radius >> 15); // Y
    output[1] = static_cast<int16_t>(cosLut(Angle) * Radius >> 15); // X
}

void Dsp1::radius(int16_t* input, int16_t* output) {
    int16_t& X = input[0];
    int16_t& Y = input[1];
    int16_t& Z = input[2];
    int32_t Rad = (X * X + Y * Y + Z * Z) << 1;
    output[0] = static_cast<int16_t>(Rad);
    output[1] = static_cast<int16_t>(Rad >> 16);
}

void Dsp1::range(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1];
    int16_t& Z = input[2]; int16_t& R = input[3];
    output[0] = static_cast<int16_t>((X * X + Y * Y + Z * Z - R * R) >> 15);
}

void Dsp1::range2(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1];
    int16_t& Z = input[2]; int16_t& R = input[3];
    output[0] = static_cast<int16_t>(((X * X + Y * Y + Z * Z - R * R) >> 15) + 1);
}

void Dsp1::distance(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1]; int16_t& Z = input[2];

    int32_t Rad = X * X + Y * Y + Z * Z;
    if (Rad == 0) { output[0] = 0; return; }

    int16_t C, E;
    normalizeDouble(Rad, C, E);
    if (E & 1) C = static_cast<int16_t>(C * 0x4000 >> 15);

    int16_t Pos = static_cast<int16_t>(C * 0x0040 >> 15);
    int16_t Node1 = static_cast<int16_t>(kDataRom[0x00d5 + Pos]);
    int16_t Node2 = static_cast<int16_t>(kDataRom[0x00d6 + Pos]);
    int16_t Dist = static_cast<int16_t>(((Node2 - Node1) * (C & 0x1ff) >> 9) + Node1);
    Dist >>= (E >> 1);
    output[0] = Dist;
}

void Dsp1::rotate(int16_t* input, int16_t* output) {
    int16_t& Angle = input[0]; int16_t& X1 = input[1]; int16_t& Y1 = input[2];
    output[0] = static_cast<int16_t>((Y1 * sinLut(Angle) >> 15) + (X1 * cosLut(Angle) >> 15));
    output[1] = static_cast<int16_t>((Y1 * cosLut(Angle) >> 15) - (X1 * sinLut(Angle) >> 15));
}

void Dsp1::polar(int16_t* input, int16_t* output) {
    int16_t& Az = input[0]; int16_t& Ay = input[1]; int16_t& Ax = input[2];
    int16_t& X1 = input[3]; int16_t& Y1 = input[4]; int16_t& Z1 = input[5];

    int16_t X, Y, Z;
    // Rotate around Z
    X = static_cast<int16_t>((Y1 * sinLut(Az) >> 15) + (X1 * cosLut(Az) >> 15));
    Y = static_cast<int16_t>((Y1 * cosLut(Az) >> 15) - (X1 * sinLut(Az) >> 15));
    X1 = X; Y1 = Y;
    // Rotate around Y
    Z = static_cast<int16_t>((X1 * sinLut(Ay) >> 15) + (Z1 * cosLut(Ay) >> 15));
    X = static_cast<int16_t>((X1 * cosLut(Ay) >> 15) - (Z1 * sinLut(Ay) >> 15));
    output[0] = X; Z1 = Z;
    // Rotate around X
    Y = static_cast<int16_t>((Z1 * sinLut(Ax) >> 15) + (Y1 * cosLut(Ax) >> 15));
    Z = static_cast<int16_t>((Z1 * cosLut(Ax) >> 15) - (Y1 * sinLut(Ax) >> 15));
    output[1] = Y; output[2] = Z;
}

// Attitude matrices

void Dsp1::attitudeA(int16_t* input, int16_t* output) {
    int16_t& S = input[0]; int16_t& Rz = input[1]; int16_t& Ry = input[2]; int16_t& Rx = input[3];
    int16_t SinRz = sinLut(Rz), CosRz = cosLut(Rz);
    int16_t SinRy = sinLut(Ry), CosRy = cosLut(Ry);
    int16_t SinRx = sinLut(Rx), CosRx = cosLut(Rx);
    S >>= 1;
    shared_.MatrixA[0][0] = static_cast<int16_t>((S * CosRz >> 15) * CosRy >> 15);
    shared_.MatrixA[0][1] = static_cast<int16_t>(((S * SinRz >> 15) * CosRx >> 15) + (((S * CosRz >> 15) * SinRx >> 15) * SinRy >> 15));
    shared_.MatrixA[0][2] = static_cast<int16_t>(((S * SinRz >> 15) * SinRx >> 15) - (((S * CosRz >> 15) * CosRx >> 15) * SinRy >> 15));
    shared_.MatrixA[1][0] = static_cast<int16_t>(-((S * SinRz >> 15) * CosRy >> 15));
    shared_.MatrixA[1][1] = static_cast<int16_t>(((S * CosRz >> 15) * CosRx >> 15) - (((S * SinRz >> 15) * SinRx >> 15) * SinRy >> 15));
    shared_.MatrixA[1][2] = static_cast<int16_t>(((S * CosRz >> 15) * SinRx >> 15) + (((S * SinRz >> 15) * CosRx >> 15) * SinRy >> 15));
    shared_.MatrixA[2][0] = static_cast<int16_t>(S * SinRy >> 15);
    shared_.MatrixA[2][1] = static_cast<int16_t>(-((S * SinRx >> 15) * CosRy >> 15));
    shared_.MatrixA[2][2] = static_cast<int16_t>((S * CosRx >> 15) * CosRy >> 15);
}

void Dsp1::attitudeB(int16_t* input, int16_t* output) {
    int16_t& S = input[0]; int16_t& Rz = input[1]; int16_t& Ry = input[2]; int16_t& Rx = input[3];
    int16_t SinRz = sinLut(Rz), CosRz = cosLut(Rz);
    int16_t SinRy = sinLut(Ry), CosRy = cosLut(Ry);
    int16_t SinRx = sinLut(Rx), CosRx = cosLut(Rx);
    S >>= 1;
    shared_.MatrixB[0][0] = static_cast<int16_t>((S * CosRz >> 15) * CosRy >> 15);
    shared_.MatrixB[0][1] = static_cast<int16_t>(((S * SinRz >> 15) * CosRx >> 15) + (((S * CosRz >> 15) * SinRx >> 15) * SinRy >> 15));
    shared_.MatrixB[0][2] = static_cast<int16_t>(((S * SinRz >> 15) * SinRx >> 15) - (((S * CosRz >> 15) * CosRx >> 15) * SinRy >> 15));
    shared_.MatrixB[1][0] = static_cast<int16_t>(-((S * SinRz >> 15) * CosRy >> 15));
    shared_.MatrixB[1][1] = static_cast<int16_t>(((S * CosRz >> 15) * CosRx >> 15) - (((S * SinRz >> 15) * SinRx >> 15) * SinRy >> 15));
    shared_.MatrixB[1][2] = static_cast<int16_t>(((S * CosRz >> 15) * SinRx >> 15) + (((S * SinRz >> 15) * CosRx >> 15) * SinRy >> 15));
    shared_.MatrixB[2][0] = static_cast<int16_t>(S * SinRy >> 15);
    shared_.MatrixB[2][1] = static_cast<int16_t>(-((S * SinRx >> 15) * CosRy >> 15));
    shared_.MatrixB[2][2] = static_cast<int16_t>((S * CosRx >> 15) * CosRy >> 15);
}

void Dsp1::attitudeC(int16_t* input, int16_t* output) {
    int16_t& S = input[0]; int16_t& Rz = input[1]; int16_t& Ry = input[2]; int16_t& Rx = input[3];
    int16_t SinRz = sinLut(Rz), CosRz = cosLut(Rz);
    int16_t SinRy = sinLut(Ry), CosRy = cosLut(Ry);
    int16_t SinRx = sinLut(Rx), CosRx = cosLut(Rx);
    S >>= 1;
    shared_.MatrixC[0][0] = static_cast<int16_t>((S * CosRz >> 15) * CosRy >> 15);
    shared_.MatrixC[0][1] = static_cast<int16_t>(((S * SinRz >> 15) * CosRx >> 15) + (((S * CosRz >> 15) * SinRx >> 15) * SinRy >> 15));
    shared_.MatrixC[0][2] = static_cast<int16_t>(((S * SinRz >> 15) * SinRx >> 15) - (((S * CosRz >> 15) * CosRx >> 15) * SinRy >> 15));
    shared_.MatrixC[1][0] = static_cast<int16_t>(-((S * SinRz >> 15) * CosRy >> 15));
    shared_.MatrixC[1][1] = static_cast<int16_t>(((S * CosRz >> 15) * CosRx >> 15) - (((S * SinRz >> 15) * SinRx >> 15) * SinRy >> 15));
    shared_.MatrixC[1][2] = static_cast<int16_t>(((S * CosRz >> 15) * SinRx >> 15) + (((S * SinRz >> 15) * CosRx >> 15) * SinRy >> 15));
    shared_.MatrixC[2][0] = static_cast<int16_t>(S * SinRy >> 15);
    shared_.MatrixC[2][1] = static_cast<int16_t>(-((S * SinRx >> 15) * CosRy >> 15));
    shared_.MatrixC[2][2] = static_cast<int16_t>((S * CosRx >> 15) * CosRy >> 15);
}

// Objective (global → object coordinates)

void Dsp1::objectiveA(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1]; int16_t& Z = input[2];
    output[0] = static_cast<int16_t>((shared_.MatrixA[0][0] * X >> 15) + (shared_.MatrixA[1][0] * Y >> 15) + (shared_.MatrixA[2][0] * Z >> 15));
    output[1] = static_cast<int16_t>((shared_.MatrixA[0][1] * X >> 15) + (shared_.MatrixA[1][1] * Y >> 15) + (shared_.MatrixA[2][1] * Z >> 15));
    output[2] = static_cast<int16_t>((shared_.MatrixA[0][2] * X >> 15) + (shared_.MatrixA[1][2] * Y >> 15) + (shared_.MatrixA[2][2] * Z >> 15));
}

void Dsp1::objectiveB(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1]; int16_t& Z = input[2];
    output[0] = static_cast<int16_t>((shared_.MatrixB[0][0] * X >> 15) + (shared_.MatrixB[1][0] * Y >> 15) + (shared_.MatrixB[2][0] * Z >> 15));
    output[1] = static_cast<int16_t>((shared_.MatrixB[0][1] * X >> 15) + (shared_.MatrixB[1][1] * Y >> 15) + (shared_.MatrixB[2][1] * Z >> 15));
    output[2] = static_cast<int16_t>((shared_.MatrixB[0][2] * X >> 15) + (shared_.MatrixB[1][2] * Y >> 15) + (shared_.MatrixB[2][2] * Z >> 15));
}

void Dsp1::objectiveC(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1]; int16_t& Z = input[2];
    output[0] = static_cast<int16_t>((shared_.MatrixC[0][0] * X >> 15) + (shared_.MatrixC[1][0] * Y >> 15) + (shared_.MatrixC[2][0] * Z >> 15));
    output[1] = static_cast<int16_t>((shared_.MatrixC[0][1] * X >> 15) + (shared_.MatrixC[1][1] * Y >> 15) + (shared_.MatrixC[2][1] * Z >> 15));
    output[2] = static_cast<int16_t>((shared_.MatrixC[0][2] * X >> 15) + (shared_.MatrixC[1][2] * Y >> 15) + (shared_.MatrixC[2][2] * Z >> 15));
}

// Subjective (object → global coordinates)

void Dsp1::subjectiveA(int16_t* input, int16_t* output) {
    int16_t& F = input[0]; int16_t& L = input[1]; int16_t& U = input[2];
    output[0] = static_cast<int16_t>((shared_.MatrixA[0][0] * F >> 15) + (shared_.MatrixA[0][1] * L >> 15) + (shared_.MatrixA[0][2] * U >> 15));
    output[1] = static_cast<int16_t>((shared_.MatrixA[1][0] * F >> 15) + (shared_.MatrixA[1][1] * L >> 15) + (shared_.MatrixA[1][2] * U >> 15));
    output[2] = static_cast<int16_t>((shared_.MatrixA[2][0] * F >> 15) + (shared_.MatrixA[2][1] * L >> 15) + (shared_.MatrixA[2][2] * U >> 15));
}

void Dsp1::subjectiveB(int16_t* input, int16_t* output) {
    int16_t& F = input[0]; int16_t& L = input[1]; int16_t& U = input[2];
    output[0] = static_cast<int16_t>((shared_.MatrixB[0][0] * F >> 15) + (shared_.MatrixB[0][1] * L >> 15) + (shared_.MatrixB[0][2] * U >> 15));
    output[1] = static_cast<int16_t>((shared_.MatrixB[1][0] * F >> 15) + (shared_.MatrixB[1][1] * L >> 15) + (shared_.MatrixB[1][2] * U >> 15));
    output[2] = static_cast<int16_t>((shared_.MatrixB[2][0] * F >> 15) + (shared_.MatrixB[2][1] * L >> 15) + (shared_.MatrixB[2][2] * U >> 15));
}

void Dsp1::subjectiveC(int16_t* input, int16_t* output) {
    int16_t& F = input[0]; int16_t& L = input[1]; int16_t& U = input[2];
    output[0] = static_cast<int16_t>((shared_.MatrixC[0][0] * F >> 15) + (shared_.MatrixC[0][1] * L >> 15) + (shared_.MatrixC[0][2] * U >> 15));
    output[1] = static_cast<int16_t>((shared_.MatrixC[1][0] * F >> 15) + (shared_.MatrixC[1][1] * L >> 15) + (shared_.MatrixC[1][2] * U >> 15));
    output[2] = static_cast<int16_t>((shared_.MatrixC[2][0] * F >> 15) + (shared_.MatrixC[2][1] * L >> 15) + (shared_.MatrixC[2][2] * U >> 15));
}

// Scalar products

void Dsp1::scalarA(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1]; int16_t& Z = input[2];
    output[0] = static_cast<int16_t>((X * shared_.MatrixA[0][0] + Y * shared_.MatrixA[1][0] + Z * shared_.MatrixA[2][0]) >> 15);
}

void Dsp1::scalarB(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1]; int16_t& Z = input[2];
    output[0] = static_cast<int16_t>((X * shared_.MatrixB[0][0] + Y * shared_.MatrixB[1][0] + Z * shared_.MatrixB[2][0]) >> 15);
}

void Dsp1::scalarC(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1]; int16_t& Z = input[2];
    output[0] = static_cast<int16_t>((X * shared_.MatrixC[0][0] + Y * shared_.MatrixC[1][0] + Z * shared_.MatrixC[2][0]) >> 15);
}

// Gyrate

void Dsp1::gyrate(int16_t* input, int16_t* output) {
    int16_t& Az = input[0]; int16_t& Ax = input[1]; int16_t& Ay = input[2];
    int16_t& U = input[3]; int16_t& F = input[4]; int16_t& L = input[5];

    int16_t CSec, ESec, CSin, C, E;
    int16_t SinAy = sinLut(Ay);
    int16_t CosAy = cosLut(Ay);

    inverse(cosLut(Ax), 0, CSec, ESec);

    // Rotation around Z
    normalizeDouble(U * CosAy - F * SinAy, C, E);
    E = static_cast<int16_t>(ESec - E);
    normalize(static_cast<int16_t>(C * CSec >> 15), C, E);
    output[0] = static_cast<int16_t>(Az + denormalizeAndClip(C, E)); // Rz

    // Rotation around X
    output[1] = static_cast<int16_t>(Ax + (U * SinAy >> 15) + (F * CosAy >> 15)); // Rx

    // Rotation around Y
    normalizeDouble(U * CosAy + F * SinAy, C, E);
    E = static_cast<int16_t>(ESec - E);
    normalize(sinLut(Ax), CSin, E);
    normalize(static_cast<int16_t>(-(C * (CSec * CSin >> 15) >> 15)), C, E);
    output[2] = static_cast<int16_t>(Ay + denormalizeAndClip(C, E) + L); // Ry
}

// Projection: parameter, raster, target, project

const int16_t Dsp1::kMaxAZS_Exp[16] = {
    0x38b4, 0x38b7, 0x38ba, 0x38be, 0x38c0, 0x38c4, 0x38c7, 0x38ca,
    0x38ce, 0x38d0, 0x38d4, 0x38d7, 0x38da, 0x38dd, 0x38e0, 0x38e4
};

void Dsp1::parameter(int16_t* input, int16_t* output) {
    int16_t& Fx = input[0]; int16_t& Fy = input[1]; int16_t& Fz = input[2];
    int16_t& Lfe = input[3]; int16_t& Les = input[4];
    int16_t& Aas = input[5]; int16_t& Azs = input[6];

    int16_t CSec, C, E;
    int16_t LfeNx, LfeNy, LfeNz;
    int16_t LesNx, LesNy, LesNz;
    int16_t AZS = Azs;

    shared_.Les = Les;
    shared_.E_Les = 0;
    normalize(Les, shared_.C_Les, shared_.E_Les);

    shared_.SinAas = sinLut(Aas);
    shared_.CosAas = cosLut(Aas);
    shared_.SinAzs = sinLut(Azs);
    shared_.CosAzs = cosLut(Azs);

    shared_.Nx = static_cast<int16_t>(shared_.SinAzs * -shared_.SinAas >> 15);
    shared_.Ny = static_cast<int16_t>(shared_.SinAzs * shared_.CosAas >> 15);
    shared_.Nz = static_cast<int16_t>(shared_.CosAzs * 0x7fff >> 15);

    shared_.Hx = static_cast<int16_t>(shared_.CosAas * 0x7fff >> 15);
    shared_.Hy = static_cast<int16_t>(shared_.SinAas * 0x7fff >> 15);

    shared_.Vx = static_cast<int16_t>(shared_.CosAzs * -shared_.SinAas >> 15);
    shared_.Vy = static_cast<int16_t>(shared_.CosAzs * shared_.CosAas >> 15);
    shared_.Vz = static_cast<int16_t>(-shared_.SinAzs * 0x7fff >> 15);

    LfeNx = static_cast<int16_t>(Lfe * shared_.Nx >> 15);
    LfeNy = static_cast<int16_t>(Lfe * shared_.Ny >> 15);
    LfeNz = static_cast<int16_t>(Lfe * shared_.Nz >> 15);

    shared_.CentreX = static_cast<int16_t>(Fx + LfeNx);
    shared_.CentreY = static_cast<int16_t>(Fy + LfeNy);
    shared_.CentreZ = static_cast<int16_t>(Fz + LfeNz);

    LesNx = static_cast<int16_t>(Les * shared_.Nx >> 15);
    LesNy = static_cast<int16_t>(Les * shared_.Ny >> 15);
    LesNz = static_cast<int16_t>(Les * shared_.Nz >> 15);

    shared_.Gx = static_cast<int16_t>(shared_.CentreX - LesNx);
    shared_.Gy = static_cast<int16_t>(shared_.CentreY - LesNy);
    shared_.Gz = static_cast<int16_t>(shared_.CentreZ - LesNz);

    E = 0;
    normalize(shared_.CentreZ, C, E);
    shared_.CentreZ_C = C;
    shared_.CentreZ_E = E;

    int16_t MaxAZS = kMaxAZS_Exp[-E];
    if (AZS < 0) {
        MaxAZS = static_cast<int16_t>(-MaxAZS);
        if (AZS < MaxAZS + 1) AZS = static_cast<int16_t>(MaxAZS + 1);
    } else {
        if (AZS > MaxAZS) AZS = MaxAZS;
    }

    shared_.SinAZS = sinLut(AZS);
    shared_.CosAZS = cosLut(AZS);

    inverse(shared_.CosAZS, 0, shared_.SecAZS_C1, shared_.SecAZS_E1);
    normalize(static_cast<int16_t>(C * shared_.SecAZS_C1 >> 15), C, E);
    E = static_cast<int16_t>(E + shared_.SecAZS_E1);
    C = static_cast<int16_t>(denormalizeAndClip(C, E) * shared_.SinAZS >> 15);

    shared_.CentreX = static_cast<int16_t>(shared_.CentreX + (C * shared_.SinAas >> 15));
    shared_.CentreY = static_cast<int16_t>(shared_.CentreY - (C * shared_.CosAas >> 15));

    output[2] = shared_.CentreX; // Cx
    output[3] = shared_.CentreY; // Cy

    int16_t Vof = 0;
    if ((Azs != AZS) || (Azs == MaxAZS)) {
        if (Azs == -32768) Azs = -32767;
        C = static_cast<int16_t>(Azs - MaxAZS);
        if (C >= 0) C--;
        int16_t Aux = static_cast<int16_t>(~(C << 2));

        C = static_cast<int16_t>(Aux * static_cast<int16_t>(kDataRom[0x0328]) >> 15);
        C = static_cast<int16_t>((C * Aux >> 15) + static_cast<int16_t>(kDataRom[0x0327]));
        Vof = static_cast<int16_t>(Vof - static_cast<int16_t>((C * Aux >> 15) * Les >> 15));

        C = static_cast<int16_t>(Aux * Aux >> 15);
        Aux = static_cast<int16_t>((C * static_cast<int16_t>(kDataRom[0x0324]) >> 15) + static_cast<int16_t>(kDataRom[0x0325]));
        shared_.CosAZS = static_cast<int16_t>(shared_.CosAZS + ((C * Aux >> 15) * shared_.CosAZS >> 15));
    }

    shared_.VOffset = static_cast<int16_t>(Les * shared_.CosAZS >> 15);

    inverse(shared_.SinAZS, 0, CSec, E);
    normalize(shared_.VOffset, C, E);
    normalize(static_cast<int16_t>(C * CSec >> 15), C, E);

    if (C == -32768) { C >>= 1; E++; }

    output[0] = Vof;                                    // Vof
    output[1] = denormalizeAndClip(static_cast<int16_t>(-C), E); // Vva

    inverse(shared_.CosAZS, 0, shared_.SecAZS_C2, shared_.SecAZS_E2);
}

void Dsp1::raster(int16_t* input, int16_t* output) {
    int16_t& Vs = input[0];
    int16_t C, E, C1, E1;

    inverse(static_cast<int16_t>((Vs * shared_.SinAzs >> 15) + shared_.VOffset), 7, C, E);
    E = static_cast<int16_t>(E + shared_.CentreZ_E);
    C1 = static_cast<int16_t>(C * shared_.CentreZ_C >> 15);

    E1 = static_cast<int16_t>(E + shared_.SecAZS_E2);

    normalize(C1, C, E);
    C = denormalizeAndClip(C, E);

    output[0] = static_cast<int16_t>(C * shared_.CosAas >> 15); // An
    output[2] = static_cast<int16_t>(C * shared_.SinAas >> 15); // Cn

    normalize(static_cast<int16_t>(C1 * shared_.SecAZS_C2 >> 15), C, E1);
    C = denormalizeAndClip(C, E1);

    output[1] = static_cast<int16_t>(C * -shared_.SinAas >> 15); // Bn
    output[3] = static_cast<int16_t>(C * shared_.CosAas >> 15);  // Dn
}

void Dsp1::target(int16_t* input, int16_t* output) {
    int16_t& H = input[0]; int16_t& V = input[1];
    int16_t C, E, C1, E1;

    inverse(static_cast<int16_t>((V * shared_.SinAzs >> 15) + shared_.VOffset), 8, C, E);
    E = static_cast<int16_t>(E + shared_.CentreZ_E);
    C1 = static_cast<int16_t>(C * shared_.CentreZ_C >> 15);
    E1 = static_cast<int16_t>(E + shared_.SecAZS_E1);

    int16_t Hsh = static_cast<int16_t>(H << 8);
    normalize(C1, C, E);
    C = static_cast<int16_t>(denormalizeAndClip(C, E) * Hsh >> 15);

    output[0] = static_cast<int16_t>(shared_.CentreX + (C * shared_.CosAas >> 15));
    output[1] = static_cast<int16_t>(shared_.CentreY - (C * shared_.SinAas >> 15));

    int16_t Vsh = static_cast<int16_t>(V << 8);
    normalize(static_cast<int16_t>(C1 * shared_.SecAZS_C1 >> 15), C, E1);
    C = static_cast<int16_t>(denormalizeAndClip(C, E1) * Vsh >> 15);

    output[0] = static_cast<int16_t>(output[0] + (C * -shared_.SinAas >> 15));
    output[1] = static_cast<int16_t>(output[1] + (C * shared_.CosAas >> 15));
}

void Dsp1::project(int16_t* input, int16_t* output) {
    int16_t& X = input[0]; int16_t& Y = input[1]; int16_t& Z = input[2];

    int32_t aux, aux4;
    int16_t E, E2, E3, E4, E5, refE, E6, E7;
    int16_t C2, C4, C6, C8, C9, C10, C11, C12, C16, C17, C18, C19, C20, C21, C22, C23, C24, C25, C26;
    int16_t Px, Py, Pz;

    E4 = E3 = E2 = E = E5 = 0;

    normalizeDouble(static_cast<int32_t>(X) - shared_.Gx, Px, E4);
    normalizeDouble(static_cast<int32_t>(Y) - shared_.Gy, Py, E);
    normalizeDouble(static_cast<int32_t>(Z) - shared_.Gz, Pz, E3);
    Px >>= 1; E4--;
    Py >>= 1; E--;
    Pz >>= 1; E3--;

    refE = (E < E3) ? E : E3;
    refE = (refE < E4) ? refE : E4;

    Px = shiftR(Px, static_cast<int16_t>(E4 - refE));
    Py = shiftR(Py, static_cast<int16_t>(E - refE));
    Pz = shiftR(Pz, static_cast<int16_t>(E3 - refE));

    C11 = static_cast<int16_t>(-(Px * shared_.Nx >> 15));
    C8  = static_cast<int16_t>(-(Py * shared_.Ny >> 15));
    C9  = static_cast<int16_t>(-(Pz * shared_.Nz >> 15));
    C12 = static_cast<int16_t>(C11 + C8 + C9);

    aux4 = C12;
    refE = static_cast<int16_t>(16 - refE);
    if (refE >= 0)
        aux4 <<= refE;
    else
        aux4 >>= (-refE);
    if (aux4 == -1) aux4 = 0;
    aux4 >>= 1;

    aux = static_cast<uint16_t>(shared_.Les) + aux4;
    normalizeDouble(static_cast<int32_t>(aux), C10, E2);
    E2 = static_cast<int16_t>(15 - E2);

    inverse(C10, 0, C4, E4);
    C2 = static_cast<int16_t>(C4 * shared_.C_Les >> 15);

    // H
    E7 = 0;
    C16 = static_cast<int16_t>(Px * shared_.Hx >> 15);
    C20 = static_cast<int16_t>(Py * shared_.Hy >> 15);
    C17 = static_cast<int16_t>(C16 + C20);
    C18 = static_cast<int16_t>(C17 * C2 >> 15);
    normalize(C18, C19, E7);
    output[0] = denormalizeAndClip(C19, static_cast<int16_t>(shared_.E_Les - E2 + refE + E7)); // H

    // V
    E6 = 0;
    C21 = static_cast<int16_t>(Px * shared_.Vx >> 15);
    C22 = static_cast<int16_t>(Py * shared_.Vy >> 15);
    C23 = static_cast<int16_t>(Pz * shared_.Vz >> 15);
    C24 = static_cast<int16_t>(C21 + C22 + C23);
    C26 = static_cast<int16_t>(C24 * C2 >> 15);
    normalize(C26, C25, E6);
    output[1] = denormalizeAndClip(C25, static_cast<int16_t>(shared_.E_Les - E2 + refE + E6)); // V

    // M
    normalize(C2, C6, E4);
    output[2] = denormalizeAndClip(C6, static_cast<int16_t>(E4 + shared_.E_Les - E2 - 7)); // M
}

// Auxiliary math functions

int16_t Dsp1::sinLut(int16_t Angle) {
    if (Angle < 0) {
        if (Angle == -32768) return 0;
        return static_cast<int16_t>(-sinLut(static_cast<int16_t>(-Angle)));
    }
    int32_t S = kSinTable[Angle >> 8] + (kMulTable[Angle & 0xff] * kSinTable[0x40 + (Angle >> 8)] >> 15);
    if (S > 32767) S = 32767;
    return static_cast<int16_t>(S);
}

int16_t Dsp1::cosLut(int16_t Angle) {
    if (Angle < 0) {
        if (Angle == -32768) return -32768;
        Angle = static_cast<int16_t>(-Angle);
    }
    int32_t S = kSinTable[0x40 + (Angle >> 8)] - (kMulTable[Angle & 0xff] * kSinTable[Angle >> 8] >> 15);
    if (S < -32768) S = -32767;
    return static_cast<int16_t>(S);
}

void Dsp1::inverse(int16_t Coefficient, int16_t Exponent, int16_t& iCoefficient, int16_t& iExponent) {
    if (Coefficient == 0x0000) {
        iCoefficient = 0x7fff;
        iExponent = 0x002f;
    } else {
        int16_t Sign = 1;
        if (Coefficient < 0) {
            if (Coefficient < -32767) Coefficient = -32767;
            Coefficient = static_cast<int16_t>(-Coefficient);
            Sign = -1;
        }
        while (Coefficient < 0x4000) {
            Coefficient = static_cast<int16_t>(Coefficient << 1);
            Exponent--;
        }
        if (Coefficient == 0x4000) {
            if (Sign == 1)
                iCoefficient = 0x7fff;
            else {
                iCoefficient = -0x4000;
                Exponent--;
            }
        } else {
            int16_t i = static_cast<int16_t>(kDataRom[((Coefficient - 0x4000) >> 7) + 0x0065]);
            i = static_cast<int16_t>((i + (-i * (Coefficient * i >> 15) >> 15)) << 1);
            i = static_cast<int16_t>((i + (-i * (Coefficient * i >> 15) >> 15)) << 1);
            iCoefficient = static_cast<int16_t>(i * Sign);
        }
        iExponent = static_cast<int16_t>(1 - Exponent);
    }
}

int16_t Dsp1::denormalizeAndClip(int16_t C, int16_t E) {
    if (E > 0) {
        if (C > 0) return 32767; else if (C < 0) return -32767;
    } else {
        if (E < 0) return static_cast<int16_t>(C * static_cast<int16_t>(kDataRom[0x0031 + E]) >> 15);
    }
    return C;
}

void Dsp1::normalize(int16_t m, int16_t& Coefficient, int16_t& Exponent) {
    int16_t i = 0x4000;
    int16_t e = 0;

    if (m < 0) {
        while ((m & i) && i) { i >>= 1; e++; }
    } else {
        while (!(m & i) && i) { i >>= 1; e++; }
    }

    if (e > 0)
        Coefficient = static_cast<int16_t>(m * static_cast<int16_t>(kDataRom[0x21 + e]) << 1);
    else
        Coefficient = m;

    Exponent = static_cast<int16_t>(Exponent - e);
}

void Dsp1::normalizeDouble(int32_t Product, int16_t& Coefficient, int16_t& Exponent) {
    int16_t n = static_cast<int16_t>(Product & 0x7fff);
    int16_t m = static_cast<int16_t>(Product >> 15);
    int16_t i = 0x4000;
    int16_t e = 0;

    if (m < 0) {
        while ((m & i) && i) { i >>= 1; e++; }
    } else {
        while (!(m & i) && i) { i >>= 1; e++; }
    }

    if (e > 0) {
        Coefficient = static_cast<int16_t>(m * static_cast<int16_t>(kDataRom[0x0021 + e]) << 1);
        if (e < 15)
            Coefficient = static_cast<int16_t>(Coefficient + static_cast<int16_t>(n * static_cast<int16_t>(kDataRom[0x0040 - e]) >> 15));
        else {
            i = 0x4000;
            if (m < 0) {
                while ((n & i) && i) { i >>= 1; e++; }
            } else {
                while (!(n & i) && i) { i >>= 1; e++; }
            }
            if (e > 15)
                Coefficient = static_cast<int16_t>(n * static_cast<int16_t>(kDataRom[0x0012 + e]) << 1);
            else
                Coefficient = static_cast<int16_t>(Coefficient + n);
        }
    } else {
        Coefficient = m;
    }

    Exponent = e;
}

int16_t Dsp1::shiftR(int16_t C, int16_t E) {
    return static_cast<int16_t>(C * static_cast<int16_t>(kDataRom[0x0031 + E]) >> 15);
}

// Lookup tables

const int16_t Dsp1::kSinTable[256] = {
    0x0000,  0x0324,  0x0647,  0x096a,  0x0c8b,  0x0fab,  0x12c8,  0x15e2,
    0x18f8,  0x1c0b,  0x1f19,  0x2223,  0x2528,  0x2826,  0x2b1f,  0x2e11,
    0x30fb,  0x33de,  0x36ba,  0x398c,  0x3c56,  0x3f17,  0x41ce,  0x447a,
    0x471c,  0x49b4,  0x4c3f,  0x4ebf,  0x5133,  0x539b,  0x55f5,  0x5842,
    0x5a82,  0x5cb4,  0x5ed7,  0x60ec,  0x62f2,  0x64e8,  0x66cf,  0x68a6,
    0x6a6d,  0x6c24,  0x6dca,  0x6f5f,  0x70e2,  0x7255,  0x73b5,  0x7504,
    0x7641,  0x776c,  0x7884,  0x798a,  0x7a7d,  0x7b5d,  0x7c29,  0x7ce3,
    0x7d8a,  0x7e1d,  0x7e9d,  0x7f09,  0x7f62,  0x7fa7,  0x7fd8,  0x7ff6,
    0x7fff,  0x7ff6,  0x7fd8,  0x7fa7,  0x7f62,  0x7f09,  0x7e9d,  0x7e1d,
    0x7d8a,  0x7ce3,  0x7c29,  0x7b5d,  0x7a7d,  0x798a,  0x7884,  0x776c,
    0x7641,  0x7504,  0x73b5,  0x7255,  0x70e2,  0x6f5f,  0x6dca,  0x6c24,
    0x6a6d,  0x68a6,  0x66cf,  0x64e8,  0x62f2,  0x60ec,  0x5ed7,  0x5cb4,
    0x5a82,  0x5842,  0x55f5,  0x539b,  0x5133,  0x4ebf,  0x4c3f,  0x49b4,
    0x471c,  0x447a,  0x41ce,  0x3f17,  0x3c56,  0x398c,  0x36ba,  0x33de,
    0x30fb,  0x2e11,  0x2b1f,  0x2826,  0x2528,  0x2223,  0x1f19,  0x1c0b,
    0x18f8,  0x15e2,  0x12c8,  0x0fab,  0x0c8b,  0x096a,  0x0647,  0x0324,
    -0x0000, -0x0324, -0x0647, -0x096a, -0x0c8b, -0x0fab, -0x12c8, -0x15e2,
    -0x18f8, -0x1c0b, -0x1f19, -0x2223, -0x2528, -0x2826, -0x2b1f, -0x2e11,
    -0x30fb, -0x33de, -0x36ba, -0x398c, -0x3c56, -0x3f17, -0x41ce, -0x447a,
    -0x471c, -0x49b4, -0x4c3f, -0x4ebf, -0x5133, -0x539b, -0x55f5, -0x5842,
    -0x5a82, -0x5cb4, -0x5ed7, -0x60ec, -0x62f2, -0x64e8, -0x66cf, -0x68a6,
    -0x6a6d, -0x6c24, -0x6dca, -0x6f5f, -0x70e2, -0x7255, -0x73b5, -0x7504,
    -0x7641, -0x776c, -0x7884, -0x798a, -0x7a7d, -0x7b5d, -0x7c29, -0x7ce3,
    -0x7d8a, -0x7e1d, -0x7e9d, -0x7f09, -0x7f62, -0x7fa7, -0x7fd8, -0x7ff6,
    -0x7fff, -0x7ff6, -0x7fd8, -0x7fa7, -0x7f62, -0x7f09, -0x7e9d, -0x7e1d,
    -0x7d8a, -0x7ce3, -0x7c29, -0x7b5d, -0x7a7d, -0x798a, -0x7884, -0x776c,
    -0x7641, -0x7504, -0x73b5, -0x7255, -0x70e2, -0x6f5f, -0x6dca, -0x6c24,
    -0x6a6d, -0x68a6, -0x66cf, -0x64e8, -0x62f2, -0x60ec, -0x5ed7, -0x5cb4,
    -0x5a82, -0x5842, -0x55f5, -0x539b, -0x5133, -0x4ebf, -0x4c3f, -0x49b4,
    -0x471c, -0x447a, -0x41ce, -0x3f17, -0x3c56, -0x398c, -0x36ba, -0x33de,
    -0x30fb, -0x2e11, -0x2b1f, -0x2826, -0x2528, -0x2223, -0x1f19, -0x1c0b,
    -0x18f8, -0x15e2, -0x12c8, -0x0fab, -0x0c8b, -0x096a, -0x0647, -0x0324
};

const int16_t Dsp1::kMulTable[256] = {
    0x0000,  0x0003,  0x0006,  0x0009,  0x000c,  0x000f,  0x0012,  0x0015,
    0x0019,  0x001c,  0x001f,  0x0022,  0x0025,  0x0028,  0x002b,  0x002f,
    0x0032,  0x0035,  0x0038,  0x003b,  0x003e,  0x0041,  0x0045,  0x0048,
    0x004b,  0x004e,  0x0051,  0x0054,  0x0057,  0x005b,  0x005e,  0x0061,
    0x0064,  0x0067,  0x006a,  0x006d,  0x0071,  0x0074,  0x0077,  0x007a,
    0x007d,  0x0080,  0x0083,  0x0087,  0x008a,  0x008d,  0x0090,  0x0093,
    0x0096,  0x0099,  0x009d,  0x00a0,  0x00a3,  0x00a6,  0x00a9,  0x00ac,
    0x00af,  0x00b3,  0x00b6,  0x00b9,  0x00bc,  0x00bf,  0x00c2,  0x00c5,
    0x00c9,  0x00cc,  0x00cf,  0x00d2,  0x00d5,  0x00d8,  0x00db,  0x00df,
    0x00e2,  0x00e5,  0x00e8,  0x00eb,  0x00ee,  0x00f1,  0x00f5,  0x00f8,
    0x00fb,  0x00fe,  0x0101,  0x0104,  0x0107,  0x010b,  0x010e,  0x0111,
    0x0114,  0x0117,  0x011a,  0x011d,  0x0121,  0x0124,  0x0127,  0x012a,
    0x012d,  0x0130,  0x0133,  0x0137,  0x013a,  0x013d,  0x0140,  0x0143,
    0x0146,  0x0149,  0x014d,  0x0150,  0x0153,  0x0156,  0x0159,  0x015c,
    0x015f,  0x0163,  0x0166,  0x0169,  0x016c,  0x016f,  0x0172,  0x0175,
    0x0178,  0x017c,  0x017f,  0x0182,  0x0185,  0x0188,  0x018b,  0x018e,
    0x0192,  0x0195,  0x0198,  0x019b,  0x019e,  0x01a1,  0x01a4,  0x01a8,
    0x01ab,  0x01ae,  0x01b1,  0x01b4,  0x01b7,  0x01ba,  0x01be,  0x01c1,
    0x01c4,  0x01c7,  0x01ca,  0x01cd,  0x01d0,  0x01d4,  0x01d7,  0x01da,
    0x01dd,  0x01e0,  0x01e3,  0x01e6,  0x01ea,  0x01ed,  0x01f0,  0x01f3,
    0x01f6,  0x01f9,  0x01fc,  0x0200,  0x0203,  0x0206,  0x0209,  0x020c,
    0x020f,  0x0212,  0x0216,  0x0219,  0x021c,  0x021f,  0x0222,  0x0225,
    0x0228,  0x022c,  0x022f,  0x0232,  0x0235,  0x0238,  0x023b,  0x023e,
    0x0242,  0x0245,  0x0248,  0x024b,  0x024e,  0x0251,  0x0254,  0x0258,
    0x025b,  0x025e,  0x0261,  0x0264,  0x0267,  0x026a,  0x026e,  0x0271,
    0x0274,  0x0277,  0x027a,  0x027d,  0x0280,  0x0284,  0x0287,  0x028a,
    0x028d,  0x0290,  0x0293,  0x0296,  0x029a,  0x029d,  0x02a0,  0x02a3,
    0x02a6,  0x02a9,  0x02ac,  0x02b0,  0x02b3,  0x02b6,  0x02b9,  0x02bc,
    0x02bf,  0x02c2,  0x02c6,  0x02c9,  0x02cc,  0x02cf,  0x02d2,  0x02d5,
    0x02d8,  0x02db,  0x02df,  0x02e2,  0x02e5,  0x02e8,  0x02eb,  0x02ee,
    0x02f1,  0x02f5,  0x02f8,  0x02fb,  0x02fe,  0x0301,  0x0304,  0x0307,
    0x030b,  0x030e,  0x0311,  0x0314,  0x0317,  0x031a,  0x031d,  0x0321
};

const uint16_t Dsp1::kDataRom[1024] = {
    0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,
    0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,
    0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,
    0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,
    0x0000,  0x0000,  0x0001,  0x0002,  0x0004,  0x0008,  0x0010,  0x0020,
    0x0040,  0x0080,  0x0100,  0x0200,  0x0400,  0x0800,  0x1000,  0x2000,
    0x4000,  0x7fff,  0x4000,  0x2000,  0x1000,  0x0800,  0x0400,  0x0200,
    0x0100,  0x0080,  0x0040,  0x0020,  0x0010,  0x0008,  0x0004,  0x0002,
    0x0001,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,
    0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,
    0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,
    0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,  0x0000,
    0x0000,  0x0000,  0x8000,  0xffe5,  0x0100,  0x7fff,  0x7f02,  0x7e08,
    0x7d12,  0x7c1f,  0x7b30,  0x7a45,  0x795d,  0x7878,  0x7797,  0x76ba,
    0x75df,  0x7507,  0x7433,  0x7361,  0x7293,  0x71c7,  0x70fe,  0x7038,
    0x6f75,  0x6eb4,  0x6df6,  0x6d3a,  0x6c81,  0x6bca,  0x6b16,  0x6a64,
    0x69b4,  0x6907,  0x685b,  0x67b2,  0x670b,  0x6666,  0x65c4,  0x6523,
    0x6484,  0x63e7,  0x634c,  0x62b3,  0x621c,  0x6186,  0x60f2,  0x6060,
    0x5fd0,  0x5f41,  0x5eb5,  0x5e29,  0x5d9f,  0x5d17,  0x5c91,  0x5c0c,
    0x5b88,  0x5b06,  0x5a85,  0x5a06,  0x5988,  0x590b,  0x5890,  0x5816,
    0x579d,  0x5726,  0x56b0,  0x563b,  0x55c8,  0x5555,  0x54e4,  0x5474,
    0x5405,  0x5398,  0x532b,  0x52bf,  0x5255,  0x51ec,  0x5183,  0x511c,
    0x50b6,  0x5050,  0x4fec,  0x4f89,  0x4f26,  0x4ec5,  0x4e64,  0x4e05,
    0x4da6,  0x4d48,  0x4cec,  0x4c90,  0x4c34,  0x4bda,  0x4b81,  0x4b28,
    0x4ad0,  0x4a79,  0x4a23,  0x49cd,  0x4979,  0x4925,  0x48d1,  0x487f,
    0x482d,  0x47dc,  0x478c,  0x473c,  0x46ed,  0x469f,  0x4651,  0x4604,
    0x45b8,  0x456c,  0x4521,  0x44d7,  0x448d,  0x4444,  0x43fc,  0x43b4,
    0x436d,  0x4326,  0x42e0,  0x429a,  0x4255,  0x4211,  0x41cd,  0x4189,
    0x4146,  0x4104,  0x40c2,  0x4081,  0x4040,  0x3fff,  0x41f7,  0x43e1,
    0x45bd,  0x478d,  0x4951,  0x4b0b,  0x4cbb,  0x4e61,  0x4fff,  0x5194,
    0x5322,  0x54a9,  0x5628,  0x57a2,  0x5914,  0x5a81,  0x5be9,  0x5d4a,
    0x5ea7,  0x5fff,  0x6152,  0x62a0,  0x63ea,  0x6530,  0x6672,  0x67b0,
    0x68ea,  0x6a20,  0x6b53,  0x6c83,  0x6daf,  0x6ed9,  0x6fff,  0x7122,
    0x7242,  0x735f,  0x747a,  0x7592,  0x76a7,  0x77ba,  0x78cb,  0x79d9,
    0x7ae5,  0x7bee,  0x7cf5,  0x7dfa,  0x7efe,  0x7fff,  0x0000,  0x0324,
    0x0647,  0x096a,  0x0c8b,  0x0fab,  0x12c8,  0x15e2,  0x18f8,  0x1c0b,
    0x1f19,  0x2223,  0x2528,  0x2826,  0x2b1f,  0x2e11,  0x30fb,  0x33de,
    0x36ba,  0x398c,  0x3c56,  0x3f17,  0x41ce,  0x447a,  0x471c,  0x49b4,
    0x4c3f,  0x4ebf,  0x5133,  0x539b,  0x55f5,  0x5842,  0x5a82,  0x5cb4,
    0x5ed7,  0x60ec,  0x62f2,  0x64e8,  0x66cf,  0x68a6,  0x6a6d,  0x6c24,
    0x6dca,  0x6f5f,  0x70e2,  0x7255,  0x73b5,  0x7504,  0x7641,  0x776c,
    0x7884,  0x798a,  0x7a7d,  0x7b5d,  0x7c29,  0x7ce3,  0x7d8a,  0x7e1d,
    0x7e9d,  0x7f09,  0x7f62,  0x7fa7,  0x7fd8,  0x7ff6,  0x7fff,  0x7ff6,
    0x7fd8,  0x7fa7,  0x7f62,  0x7f09,  0x7e9d,  0x7e1d,  0x7d8a,  0x7ce3,
    0x7c29,  0x7b5d,  0x7a7d,  0x798a,  0x7884,  0x776c,  0x7641,  0x7504,
    0x73b5,  0x7255,  0x70e2,  0x6f5f,  0x6dca,  0x6c24,  0x6a6d,  0x68a6,
    0x66cf,  0x64e8,  0x62f2,  0x60ec,  0x5ed7,  0x5cb4,  0x5a82,  0x5842,
    0x55f5,  0x539b,  0x5133,  0x4ebf,  0x4c3f,  0x49b4,  0x471c,  0x447a,
    0x41ce,  0x3f17,  0x3c56,  0x398c,  0x36ba,  0x33de,  0x30fb,  0x2e11,
    0x2b1f,  0x2826,  0x2528,  0x2223,  0x1f19,  0x1c0b,  0x18f8,  0x15e2,
    0x12c8,  0x0fab,  0x0c8b,  0x096a,  0x0647,  0x0324,  0x7fff,  0x7ff6,
    0x7fd8,  0x7fa7,  0x7f62,  0x7f09,  0x7e9d,  0x7e1d,  0x7d8a,  0x7ce3,
    0x7c29,  0x7b5d,  0x7a7d,  0x798a,  0x7884,  0x776c,  0x7641,  0x7504,
    0x73b5,  0x7255,  0x70e2,  0x6f5f,  0x6dca,  0x6c24,  0x6a6d,  0x68a6,
    0x66cf,  0x64e8,  0x62f2,  0x60ec,  0x5ed7,  0x5cb4,  0x5a82,  0x5842,
    0x55f5,  0x539b,  0x5133,  0x4ebf,  0x4c3f,  0x49b4,  0x471c,  0x447a,
    0x41ce,  0x3f17,  0x3c56,  0x398c,  0x36ba,  0x33de,  0x30fb,  0x2e11,
    0x2b1f,  0x2826,  0x2528,  0x2223,  0x1f19,  0x1c0b,  0x18f8,  0x15e2,
    0x12c8,  0x0fab,  0x0c8b,  0x096a,  0x0647,  0x0324,  0x0000,  0xfcdc,
    0xf9b9,  0xf696,  0xf375,  0xf055,  0xed38,  0xea1e,  0xe708,  0xe3f5,
    0xe0e7,  0xdddd,  0xdad8,  0xd7da,  0xd4e1,  0xd1ef,  0xcf05,  0xcc22,
    0xc946,  0xc674,  0xc3aa,  0xc0e9,  0xbe32,  0xbb86,  0xb8e4,  0xb64c,
    0xb3c1,  0xb141,  0xaecd,  0xac65,  0xaa0b,  0xa7be,  0xa57e,  0xa34c,
    0xa129,  0x9f14,  0x9d0e,  0x9b18,  0x9931,  0x975a,  0x9593,  0x93dc,
    0x9236,  0x90a1,  0x8f1e,  0x8dab,  0x8c4b,  0x8afc,  0x89bf,  0x8894,
    0x877c,  0x8676,  0x8583,  0x84a3,  0x83d7,  0x831d,  0x8276,  0x81e3,
    0x8163,  0x80f7,  0x809e,  0x8059,  0x8028,  0x800a,  0x6488,  0x0080,
    0x03ff,  0x0116,  0x0002,  0x0080,  0x4000,  0x3fd7,  0x3faf,  0x3f86,
    0x3f5d,  0x3f34,  0x3f0c,  0x3ee3,  0x3eba,  0x3e91,  0x3e68,  0x3e40,
    0x3e17,  0x3dee,  0x3dc5,  0x3d9c,  0x3d74,  0x3d4b,  0x3d22,  0x3cf9,
    0x3cd0,  0x3ca7,  0x3c7f,  0x3c56,  0x3c2d,  0x3c04,  0x3bdb,  0x3bb2,
    0x3b89,  0x3b60,  0x3b37,  0x3b0e,  0x3ae5,  0x3abc,  0x3a93,  0x3a69,
    0x3a40,  0x3a17,  0x39ee,  0x39c5,  0x399c,  0x3972,  0x3949,  0x3920,
    0x38f6,  0x38cd,  0x38a4,  0x387a,  0x3851,  0x3827,  0x37fe,  0x37d4,
    0x37aa,  0x3781,  0x3757,  0x372d,  0x3704,  0x36da,  0x36b0,  0x3686,
    0x365c,  0x3632,  0x3609,  0x35df,  0x35b4,  0x358a,  0x3560,  0x3536,
    0x350c,  0x34e1,  0x34b7,  0x348d,  0x3462,  0x3438,  0x340d,  0x33e3,
    0x33b8,  0x338d,  0x3363,  0x3338,  0x330d,  0x32e2,  0x32b7,  0x328c,
    0x3261,  0x3236,  0x320b,  0x31df,  0x31b4,  0x3188,  0x315d,  0x3131,
    0x3106,  0x30da,  0x30ae,  0x3083,  0x3057,  0x302b,  0x2fff,  0x2fd2,
    0x2fa6,  0x2f7a,  0x2f4d,  0x2f21,  0x2ef4,  0x2ec8,  0x2e9b,  0x2e6e,
    0x2e41,  0x2e14,  0x2de7,  0x2dba,  0x2d8d,  0x2d60,  0x2d32,  0x2d05,
    0x2cd7,  0x2ca9,  0x2c7b,  0x2c4d,  0x2c1f,  0x2bf1,  0x2bc3,  0x2b94,
    0x2b66,  0x2b37,  0x2b09,  0x2ada,  0x2aab,  0x2a7c,  0x2a4c,  0x2a1d,
    0x29ed,  0x29be,  0x298e,  0x295e,  0x292e,  0x28fe,  0x28ce,  0x289d,
    0x286d,  0x283c,  0x280b,  0x27da,  0x27a9,  0x2777,  0x2746,  0x2714,
    0x26e2,  0x26b0,  0x267e,  0x264c,  0x2619,  0x25e7,  0x25b4,  0x2581,
    0x254d,  0x251a,  0x24e6,  0x24b2,  0x247e,  0x244a,  0x2415,  0x23e1,
    0x23ac,  0x2376,  0x2341,  0x230b,  0x22d6,  0x229f,  0x2269,  0x2232,
    0x21fc,  0x21c4,  0x218d,  0x2155,  0x211d,  0x20e5,  0x20ad,  0x2074,
    0x203b,  0x2001,  0x1fc7,  0x1f8d,  0x1f53,  0x1f18,  0x1edd,  0x1ea1,
    0x1e66,  0x1e29,  0x1ded,  0x1db0,  0x1d72,  0x1d35,  0x1cf6,  0x1cb8,
    0x1c79,  0x1c39,  0x1bf9,  0x1bb8,  0x1b77,  0x1b36,  0x1af4,  0x1ab1,
    0x1a6e,  0x1a2a,  0x19e6,  0x19a1,  0x195c,  0x1915,  0x18ce,  0x1887,
    0x183f,  0x17f5,  0x17ac,  0x1761,  0x1715,  0x16c9,  0x167c,  0x162e,
    0x15df,  0x158e,  0x153d,  0x14eb,  0x1497,  0x1442,  0x13ec,  0x1395,
    0x133c,  0x12e2,  0x1286,  0x1228,  0x11c9,  0x1167,  0x1104,  0x109e,
    0x1036,  0x0fcc,  0x0f5f,  0x0eef,  0x0e7b,  0x0e04,  0x0d89,  0x0d0a,
    0x0c86,  0x0bfd,  0x0b6d,  0x0ad6,  0x0a36,  0x098d,  0x08d7,  0x0811,
    0x0736,  0x063e,  0x0519,  0x039a,  0x0000,  0x7fff,  0x0100,  0x0080,
    0x021d,  0x00c8,  0x00ce,  0x0048,  0x0a26,  0x277a,  0x00ce,  0x6488,
    0x14ac,  0x0001,  0x00f9,  0x00fc,  0x00ff,  0x00fc,  0x00f9,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,
    0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff,  0xffff
};

} // namespace snes::core
