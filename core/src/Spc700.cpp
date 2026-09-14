// Spc700.cpp — Sony SPC700 Audio Processor Core Implementation
//
// All 256 opcodes, ALU algorithms, and addressing mode instruction helpers.
//
// Reference: bsnes processor/spc700/
//   - algorithms.cpp, memory.cpp, instruction.cpp, instructions.cpp

#include "snes/core/Spc700.hpp"
#include <cstring>

namespace snes::core {

// Constructor / Power

Spc700::Spc700() {
    Power();
}

void Spc700::Power() {
    r.pc   = 0xFFC0;  // IPL ROM entry point
    r.a    = 0x00;
    r.y    = 0x00;
    r.x    = 0x00;
    r.s    = 0xEF;
    r.p.Unpack(0x02);  // Z flag set
    r.wait = false;
    r.stop = false;
    cycles_ = 0;
}

// Memory access helpers

uint8_t Spc700::Fetch() {
    return Read(r.pc++);
}

uint8_t Spc700::Load(uint8_t addr) {
    return Read(static_cast<uint16_t>(r.p.p ? 0x0100 : 0x0000) | addr);
}

void Spc700::Store(uint8_t addr, uint8_t data) {
    Write(static_cast<uint16_t>(r.p.p ? 0x0100 : 0x0000) | addr, data);
}

uint8_t Spc700::Pull() {
    return Read(0x0100 | static_cast<uint16_t>(++r.s));
}

void Spc700::Push(uint8_t data) {
    Write(0x0100 | static_cast<uint16_t>(r.s--), data);
}

// ALU Algorithms — 8-bit

uint8_t Spc700::AlgADC(uint8_t x, uint8_t y) {
    int z = x + y + (r.p.c ? 1 : 0);
    r.p.c = z > 0xFF;
    r.p.z = static_cast<uint8_t>(z) == 0;
    r.p.h = ((x ^ y ^ z) & 0x10) != 0;
    r.p.v = (~(x ^ y) & (x ^ z) & 0x80) != 0;
    r.p.n = (z & 0x80) != 0;
    return static_cast<uint8_t>(z);
}

uint8_t Spc700::AlgSBC(uint8_t x, uint8_t y) {
    return AlgADC(x, ~y);
}

uint8_t Spc700::AlgAND(uint8_t x, uint8_t y) {
    x &= y;
    r.p.z = x == 0;
    r.p.n = (x & 0x80) != 0;
    return x;
}

uint8_t Spc700::AlgOR(uint8_t x, uint8_t y) {
    x |= y;
    r.p.z = x == 0;
    r.p.n = (x & 0x80) != 0;
    return x;
}

uint8_t Spc700::AlgEOR(uint8_t x, uint8_t y) {
    x ^= y;
    r.p.z = x == 0;
    r.p.n = (x & 0x80) != 0;
    return x;
}

uint8_t Spc700::AlgCMP(uint8_t x, uint8_t y) {
    int z = x - y;
    r.p.c = z >= 0;
    r.p.z = static_cast<uint8_t>(z) == 0;
    r.p.n = (z & 0x80) != 0;
    return x; // CMP returns x unchanged
}

uint8_t Spc700::AlgASL(uint8_t x) {
    r.p.c = (x & 0x80) != 0;
    x <<= 1;
    r.p.z = x == 0;
    r.p.n = (x & 0x80) != 0;
    return x;
}

uint8_t Spc700::AlgLSR(uint8_t x) {
    r.p.c = (x & 0x01) != 0;
    x >>= 1;
    r.p.z = x == 0;
    r.p.n = false; // bit 7 always 0 after LSR
    return x;
}

uint8_t Spc700::AlgROL(uint8_t x) {
    bool c = r.p.c;
    r.p.c = (x & 0x80) != 0;
    x = static_cast<uint8_t>((x << 1) | (c ? 1 : 0));
    r.p.z = x == 0;
    r.p.n = (x & 0x80) != 0;
    return x;
}

uint8_t Spc700::AlgROR(uint8_t x) {
    bool c = r.p.c;
    r.p.c = (x & 0x01) != 0;
    x = static_cast<uint8_t>((x >> 1) | (c ? 0x80 : 0));
    r.p.z = x == 0;
    r.p.n = (x & 0x80) != 0;
    return x;
}

uint8_t Spc700::AlgINC(uint8_t x) {
    x++;
    r.p.z = x == 0;
    r.p.n = (x & 0x80) != 0;
    return x;
}

uint8_t Spc700::AlgDEC(uint8_t x) {
    x--;
    r.p.z = x == 0;
    r.p.n = (x & 0x80) != 0;
    return x;
}

uint8_t Spc700::AlgLD(uint8_t /*x*/, uint8_t y) {
    r.p.z = y == 0;
    r.p.n = (y & 0x80) != 0;
    return y;
}

// ALU Algorithms — 16-bit

uint16_t Spc700::AlgADW(uint16_t x, uint16_t y) {
    // Two chained ADC calls: low byte first, then high byte
    r.p.c = false;
    uint16_t z;
    uint8_t lo = AlgADC(static_cast<uint8_t>(x), static_cast<uint8_t>(y));
    uint8_t hi = AlgADC(static_cast<uint8_t>(x >> 8), static_cast<uint8_t>(y >> 8));
    z = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    r.p.z = z == 0;
    return z;
}

uint16_t Spc700::AlgSBW(uint16_t x, uint16_t y) {
    // Two chained SBC calls: low byte first, then high byte
    r.p.c = true;
    uint8_t lo = AlgSBC(static_cast<uint8_t>(x), static_cast<uint8_t>(y));
    uint8_t hi = AlgSBC(static_cast<uint8_t>(x >> 8), static_cast<uint8_t>(y >> 8));
    uint16_t z = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    r.p.z = z == 0;
    return z;
}

uint16_t Spc700::AlgCPW(uint16_t x, uint16_t y) {
    int z = static_cast<int>(x) - static_cast<int>(y);
    r.p.c = z >= 0;
    r.p.z = static_cast<uint16_t>(z) == 0;
    r.p.n = (z & 0x8000) != 0;
    return x; // compare returns x unchanged
}

uint16_t Spc700::AlgLDW(uint16_t /*x*/, uint16_t y) {
    r.p.z = y == 0;
    r.p.n = (y & 0x8000) != 0;
    return y;
}

// Instruction implementations — addressing mode groups

// Immediate: op A, #imm
void Spc700::InstrImmediateRead(AlgOp op, uint8_t& target) {
    uint8_t imm = Fetch();
    target = (this->*op)(target, imm);
}

// Direct page: op A, dp
void Spc700::InstrDirectRead(AlgOp op, uint8_t& target) {
    uint8_t dp = Fetch();
    uint8_t val = Load(dp);
    target = (this->*op)(target, val);
}

// Direct page modify: op dp
void Spc700::InstrDirectModify(ModOp op) {
    uint8_t dp = Fetch();
    uint8_t val = Load(dp);
    val = (this->*op)(val);
    Store(dp, val);
}

// Direct page write: MOV dp, reg
void Spc700::InstrDirectWrite(uint8_t data) {
    uint8_t dp = Fetch();
    Load(dp); // dummy read (bus timing)
    Store(dp, data);
}

// Direct page indexed: op A, dp+X
void Spc700::InstrDirectIndexedRead(AlgOp op, uint8_t& target, uint8_t index) {
    uint8_t dp = Fetch();
    Idle();
    uint8_t val = Load(static_cast<uint8_t>(dp + index));
    target = (this->*op)(target, val);
}

// Direct page indexed modify: op dp+X
void Spc700::InstrDirectIndexedModify(ModOp op, uint8_t index) {
    uint8_t dp = Fetch();
    Idle();
    uint8_t val = Load(static_cast<uint8_t>(dp + index));
    val = (this->*op)(val);
    Store(static_cast<uint8_t>(dp + index), val);
}

// Direct page indexed write: MOV dp+X, reg
void Spc700::InstrDirectIndexedWrite(uint8_t data, uint8_t index) {
    uint8_t dp = Fetch();
    Idle();
    Load(static_cast<uint8_t>(dp + index)); // dummy read
    Store(static_cast<uint8_t>(dp + index), data);
}

// Absolute: op A, abs
void Spc700::InstrAbsoluteRead(AlgOp op, uint8_t& target) {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    uint16_t addr = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint8_t val = Read(addr);
    target = (this->*op)(target, val);
}

// Absolute modify: op abs
void Spc700::InstrAbsoluteModify(ModOp op) {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    uint16_t addr = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint8_t val = Read(addr);
    val = (this->*op)(val);
    Write(addr, val);
}

// Absolute write: MOV abs, reg
void Spc700::InstrAbsoluteWrite(uint8_t data) {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    uint16_t addr = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    Read(addr); // dummy read
    Write(addr, data);
}

// Absolute indexed: op A, abs+X
void Spc700::InstrAbsoluteIndexedRead(AlgOp op, uint8_t index) {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    Idle();
    uint16_t addr = (static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8)) + index;
    uint8_t val = Read(addr);
    r.a = (this->*op)(r.a, val);
}

// Absolute indexed write: MOV abs+X, A
void Spc700::InstrAbsoluteIndexedWrite(uint8_t index) {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    Idle();
    uint16_t addr = (static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8)) + index;
    Read(addr); // dummy read
    Write(addr, r.a);
}

// Indexed indirect: op A, [dp+X]
void Spc700::InstrIndexedIndirectRead(AlgOp op, uint8_t index) {
    uint8_t dp = Fetch();
    Idle();
    uint8_t ptrL = Load(static_cast<uint8_t>(dp + index));
    uint8_t ptrH = Load(static_cast<uint8_t>(dp + index + 1));
    uint16_t addr = static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8);
    uint8_t val = Read(addr);
    r.a = (this->*op)(r.a, val);
}

// Indexed indirect write: MOV [dp+X], A
void Spc700::InstrIndexedIndirectWrite(uint8_t data, uint8_t index) {
    uint8_t dp = Fetch();
    Idle();
    uint8_t ptrL = Load(static_cast<uint8_t>(dp + index));
    uint8_t ptrH = Load(static_cast<uint8_t>(dp + index + 1));
    uint16_t addr = static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8);
    Read(addr); // dummy read
    Write(addr, data);
}

// Indirect indexed: op A, [dp]+Y
void Spc700::InstrIndirectIndexedRead(AlgOp op, uint8_t index) {
    uint8_t dp = Fetch();
    uint8_t ptrL = Load(dp);
    uint8_t ptrH = Load(static_cast<uint8_t>(dp + 1));
    Idle();
    uint16_t addr = (static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8)) + index;
    uint8_t val = Read(addr);
    r.a = (this->*op)(r.a, val);
}

// Indirect indexed write: MOV [dp]+Y, A
void Spc700::InstrIndirectIndexedWrite(uint8_t data, uint8_t index) {
    uint8_t dp = Fetch();
    uint8_t ptrL = Load(dp);
    uint8_t ptrH = Load(static_cast<uint8_t>(dp + 1));
    Idle();
    uint16_t addr = (static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8)) + index;
    Read(addr); // dummy read
    Write(addr, data);
}

// Indirect X: op A, (X)
void Spc700::InstrIndirectXRead(AlgOp op) {
    Idle();
    uint8_t val = Load(r.x);
    r.a = (this->*op)(r.a, val);
}

// Indirect X write: MOV (X), A
void Spc700::InstrIndirectXWrite(uint8_t data) {
    Idle();
    Load(r.x); // dummy read
    Store(r.x, data);
}

// MOV A,(X++)
void Spc700::InstrIndirectXIncrementRead(uint8_t& target) {
    Idle();
    target = AlgLD(target, Load(r.x++));
}

// MOV (X++),A
void Spc700::InstrIndirectXIncrementWrite(uint8_t data) {
    Idle();
    Idle();
    Store(r.x++, data);
}

// CMP (X),(Y)
void Spc700::InstrIndirectXCompareIndirectY(AlgOp op) {
    Idle();
    uint8_t yVal = Load(r.y);
    uint8_t xVal = Load(r.x);
    (this->*op)(xVal, yVal);
}

// op (X)=(Y) — e.g. ADC (X)=(Y), OR (X)=(Y)
void Spc700::InstrIndirectXWriteIndirectY(AlgOp op) {
    Idle();
    uint8_t yVal = Load(r.y);
    uint8_t xVal = Load(r.x);
    xVal = (this->*op)(xVal, yVal);
    Store(r.x, xVal);
}

// Direct-Direct, Direct-Immediate instruction groups

// CMP dp, dp
void Spc700::InstrDirectDirectCompare(AlgOp op) {
    uint8_t srcDp = Fetch();
    uint8_t srcVal = Load(srcDp);
    uint8_t dstDp = Fetch();
    uint8_t dstVal = Load(dstDp);
    (this->*op)(dstVal, srcVal);
}

// op dp, dp (e.g. ADC dp,dp)
void Spc700::InstrDirectDirectModify(AlgOp op) {
    uint8_t srcDp = Fetch();
    uint8_t srcVal = Load(srcDp);
    uint8_t dstDp = Fetch();
    uint8_t dstVal = Load(dstDp);
    dstVal = (this->*op)(dstVal, srcVal);
    Store(dstDp, dstVal);
}

// MOV dp, dp
void Spc700::InstrDirectDirectWrite() {
    uint8_t srcDp = Fetch();
    uint8_t srcVal = Load(srcDp);
    uint8_t dstDp = Fetch();
    Store(dstDp, srcVal);
}

// CMP dp, #imm
void Spc700::InstrDirectImmediateCompare(AlgOp op) {
    uint8_t imm = Fetch();
    uint8_t dp = Fetch();
    uint8_t val = Load(dp);
    (this->*op)(val, imm);
}

// op dp, #imm (e.g. ADC dp,#imm)
void Spc700::InstrDirectImmediateModify(AlgOp op) {
    uint8_t imm = Fetch();
    uint8_t dp = Fetch();
    uint8_t val = Load(dp);
    val = (this->*op)(val, imm);
    Store(dp, val);
}

// MOV dp, #imm
void Spc700::InstrDirectImmediateWrite() {
    uint8_t imm = Fetch();
    uint8_t dp = Fetch();
    Load(dp); // dummy read
    Store(dp, imm);
}

// 16-bit word operations

// CMPW YA, dp
void Spc700::InstrDirectCompareWord(AlgOp16 op) {
    uint8_t dp = Fetch();
    uint8_t lo = Load(dp);
    uint8_t hi = Load(static_cast<uint8_t>(dp + 1));
    uint16_t val = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    (this->*op)(r.ya(), val);
}

// ADDW/SUBW/MOVW YA, dp
void Spc700::InstrDirectReadWord(AlgOp16 op) {
    uint8_t dp = Fetch();
    uint8_t lo = Load(dp);
    Idle();
    uint8_t hi = Load(static_cast<uint8_t>(dp + 1));
    uint16_t val = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint16_t result = (this->*op)(r.ya(), val);
    r.setYA(result);
}

// INCW/DECW dp
void Spc700::InstrDirectModifyWord(int16_t adjust) {
    uint8_t dp = Fetch();
    uint8_t lo = Load(dp);
    lo += static_cast<uint8_t>(adjust);
    Store(dp, lo);
    uint8_t hi = Load(static_cast<uint8_t>(dp + 1));
    // Carry from low byte: if adjust=+1 and lo overflowed (now 0), or adjust=-1 and lo underflowed (now 0xFF)
    hi += (adjust >= 0 ? (lo == 0 ? 1 : 0) : (lo == 0xFF ? static_cast<uint8_t>(-1) : 0));
    Store(static_cast<uint8_t>(dp + 1), hi);
    uint16_t result = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    r.p.z = result == 0;
    r.p.n = (result & 0x8000) != 0;
}

// MOVW dp, YA
void Spc700::InstrDirectWriteWord() {
    uint8_t dp = Fetch();
    Load(dp); // dummy read
    Store(dp, r.a);
    Store(static_cast<uint8_t>(dp + 1), r.y);
}

// Bit operations

// Absolute bit modify — 8 sub-modes via opcode bits
void Spc700::InstrAbsoluteBitModify(uint8_t mode) {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    uint16_t raw = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint8_t bit = static_cast<uint8_t>(raw >> 13);
    uint16_t addr = raw & 0x1FFF;
    uint8_t val = Read(addr);
    uint8_t mask = static_cast<uint8_t>(1 << bit);

    switch (mode) {
    case 0: // OR1  C, mem.bit
        Idle();
        r.p.c = r.p.c | ((val & mask) != 0);
        break;
    case 1: // OR1  C, !mem.bit
        Idle();
        r.p.c = r.p.c | ((val & mask) == 0);
        break;
    case 2: // AND1 C, mem.bit
        r.p.c = r.p.c & ((val & mask) != 0);
        break;
    case 3: // AND1 C, !mem.bit
        r.p.c = r.p.c & ((val & mask) == 0);
        break;
    case 4: // EOR1 C, mem.bit
        Idle();
        r.p.c = r.p.c ^ ((val & mask) != 0);
        break;
    case 5: // MOV1 C, mem.bit (load)
        r.p.c = (val & mask) != 0;
        break;
    case 6: // MOV1 mem.bit, C (store)
        Idle();
        if (r.p.c) val |= mask; else val &= ~mask;
        Write(addr, val);
        break;
    case 7: // NOT1 mem.bit
        val ^= mask;
        Write(addr, val);
        break;
    }
}

// SET1/CLR1 dp.bit
void Spc700::InstrAbsoluteBitSet(uint8_t bit, bool value) {
    uint8_t dp = Fetch();
    uint8_t val = Load(dp);
    uint8_t mask = static_cast<uint8_t>(1 << bit);
    if (value) val |= mask; else val &= ~mask;
    Store(dp, val);
}

// TSET1/TCLR1 abs
void Spc700::InstrTestSetBitsAbsolute(bool set) {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    uint16_t addr = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint8_t val = Read(addr);
    Read(addr); // dummy read
    // Flags: N/Z set from (A - val) (like CMP)
    uint8_t diff = static_cast<uint8_t>(r.a - val);
    r.p.z = diff == 0;
    r.p.n = (diff & 0x80) != 0;
    // Modify memory
    if (set) val |= r.a; else val &= ~r.a;
    Write(addr, val);
}

// Branch instructions

void Spc700::InstrBranch(bool take) {
    uint8_t offset = Fetch();
    if (!take) return;
    Idle();
    Idle();
    r.pc += static_cast<int8_t>(offset);
}

void Spc700::InstrBranchBit(uint8_t bit, bool match) {
    uint8_t dp = Fetch();
    uint8_t val = Load(dp);
    Idle();
    uint8_t offset = Fetch();
    bool bitSet = (val & (1 << bit)) != 0;
    if (bitSet != match) return;
    Idle();
    Idle();
    r.pc += static_cast<int8_t>(offset);
}

// CBNE dp, rel — compare A != mem, branch
void Spc700::InstrBranchNotDirect() {
    uint8_t dp = Fetch();
    uint8_t val = Load(dp);
    Idle();
    uint8_t offset = Fetch();
    if (r.a == val) return;
    Idle();
    Idle();
    r.pc += static_cast<int8_t>(offset);
}

// CBNE dp+X, rel
void Spc700::InstrBranchNotDirectIndexed(uint8_t index) {
    uint8_t dp = Fetch();
    Idle();
    uint8_t val = Load(static_cast<uint8_t>(dp + index));
    Idle();
    uint8_t offset = Fetch();
    if (r.a == val) return;
    Idle();
    Idle();
    r.pc += static_cast<int8_t>(offset);
}

// DBNZ dp, rel — decrement mem, branch if non-zero
void Spc700::InstrBranchNotDirectDecrement() {
    uint8_t dp = Fetch();
    uint8_t val = Load(dp);
    val--;
    Store(dp, val);
    uint8_t offset = Fetch();
    if (val == 0) return;
    Idle();
    Idle();
    r.pc += static_cast<int8_t>(offset);
}

// DBNZ Y, rel
void Spc700::InstrBranchNotYDecrement() {
    Idle();
    Idle();
    uint8_t offset = Fetch();
    if (--r.y == 0) return;
    Idle();
    Idle();
    r.pc += static_cast<int8_t>(offset);
}

// Flow control

// CALL abs
void Spc700::InstrCallAbsolute() {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    Idle();
    Push(static_cast<uint8_t>(r.pc >> 8));
    Push(static_cast<uint8_t>(r.pc));
    Idle();
    Idle();
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// PCALL $FFxx
void Spc700::InstrCallPage() {
    uint8_t lo = Fetch();
    Idle();
    Push(static_cast<uint8_t>(r.pc >> 8));
    Push(static_cast<uint8_t>(r.pc));
    Idle();
    r.pc = 0xFF00 | lo;
}

// TCALL n — vector at $FFDE - (n * 2)
void Spc700::InstrCallTable(uint8_t vector) {
    Idle();
    Idle();
    Push(static_cast<uint8_t>(r.pc >> 8));
    Push(static_cast<uint8_t>(r.pc));
    Idle();
    uint16_t vecAddr = static_cast<uint16_t>(0xFFDE - (vector << 1));
    uint8_t lo = Read(vecAddr);
    uint8_t hi = Read(static_cast<uint16_t>(vecAddr + 1));
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// JMP abs
void Spc700::InstrJumpAbsolute() {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// JMP [abs+X]
void Spc700::InstrJumpIndirectX() {
    uint8_t lo = Fetch();
    uint8_t hi = Fetch();
    Idle();
    uint16_t addr = (static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8)) + r.x;
    uint8_t ptrL = Read(addr);
    uint8_t ptrH = Read(static_cast<uint16_t>(addr + 1));
    r.pc = static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8);
}

// RET
void Spc700::InstrReturnSubroutine() {
    Idle();
    Idle();
    uint8_t lo = Pull();
    uint8_t hi = Pull();
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// RETI
void Spc700::InstrReturnInterrupt() {
    Idle();
    Idle();
    r.p.Unpack(Pull());
    uint8_t lo = Pull();
    uint8_t hi = Pull();
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// BRK
void Spc700::InstrBreak() {
    uint8_t lo = Read(0xFFDE);
    uint8_t hi = Read(0xFFDF);
    Idle();
    Push(static_cast<uint8_t>(r.pc >> 8));
    Push(static_cast<uint8_t>(r.pc));
    Push(r.p.Pack());
    Idle();
    r.p.i = false;
    r.p.b = true;
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// Register transfer

void Spc700::InstrTransfer(uint8_t from, uint8_t& to) {
    Idle();
    to = from;
    // No flag changes if target is SP
    if (&to != &r.s) {
        r.p.z = to == 0;
        r.p.n = (to & 0x80) != 0;
    }
}

// Push / Pull

void Spc700::InstrPush(uint8_t data) {
    Idle();
    Idle();
    Push(data);
}

void Spc700::InstrPull(uint8_t& target) {
    Idle();
    Idle();
    target = Pull();
}

void Spc700::InstrPushP() {
    Idle();
    Idle();
    Push(r.p.Pack());
}

void Spc700::InstrPullP() {
    Idle();
    Idle();
    r.p.Unpack(Pull());
}

// Flag manipulation

void Spc700::InstrFlagSet(bool& flag, bool value) {
    Idle();
    Idle();
    flag = value;
}

void Spc700::InstrOverflowClear() {
    Idle();
    Idle();
    r.p.h = false;
    r.p.v = false;
}

void Spc700::InstrComplementCarry() {
    Idle();
    Idle();
    r.p.c = !r.p.c;
}

// Implied register single-operand modify

void Spc700::InstrImpliedModify(ModOp op, uint8_t& target) {
    Idle();
    target = (this->*op)(target);
}

// Special instructions

// MUL YA — Y*A → YA (16-bit result), flags on Y
void Spc700::InstrMultiply() {
    Idle();
    for (int i = 0; i < 8; i++) Idle();
    uint16_t result = static_cast<uint16_t>(r.y) * static_cast<uint16_t>(r.a);
    r.setYA(result);
    r.p.z = r.y == 0;
    r.p.n = (r.y & 0x80) != 0;
}

// DIV YA,X — YA / X → A (quotient), Y (remainder)
void Spc700::InstrDivide() {
    Idle();
    for (int i = 0; i < 11; i++) Idle();

    // Save original values for overflow detection
    uint16_t ya = r.ya();
    uint8_t x = r.x;

    r.p.h = ((r.y & 0x0F) >= (r.x & 0x0F));  // half-carry heuristic
    r.p.v = r.y >= r.x;  // overflow if Y >= X

    if (r.y < (r.x << 1)) {
        // Normal division
        r.a = static_cast<uint8_t>(ya / x);
        r.y = static_cast<uint8_t>(ya % x);
    } else {
        // Overflow case — match bsnes behavior
        r.a = static_cast<uint8_t>(255 - (ya - (static_cast<uint16_t>(x) << 9)) / (256 - x));
        r.y = static_cast<uint8_t>(x + (ya - (static_cast<uint16_t>(x) << 9)) % (256 - x));
    }

    r.p.z = r.a == 0;
    r.p.n = (r.a & 0x80) != 0;
}

// DAA — BCD adjust after addition
void Spc700::InstrDecimalAdjustAdd() {
    Idle();
    Idle();
    if (r.p.c || r.a > 0x99) { r.a += 0x60; r.p.c = true; }
    if (r.p.h || (r.a & 0x0F) > 0x09) { r.a += 0x06; }
    r.p.z = r.a == 0;
    r.p.n = (r.a & 0x80) != 0;
}

// DAS — BCD adjust after subtraction
void Spc700::InstrDecimalAdjustSub() {
    Idle();
    Idle();
    if (!r.p.c || r.a > 0x99) { r.a -= 0x60; r.p.c = false; }
    if (!r.p.h || (r.a & 0x0F) > 0x09) { r.a -= 0x06; }
    r.p.z = r.a == 0;
    r.p.n = (r.a & 0x80) != 0;
}

// XCN — swap nibbles of A
void Spc700::InstrExchangeNibble() {
    Idle();
    Idle();
    Idle();
    Idle();
    r.a = static_cast<uint8_t>((r.a >> 4) | (r.a << 4));
    r.p.z = r.a == 0;
    r.p.n = (r.a & 0x80) != 0;
}

void Spc700::InstrNoOperation() {
    Idle();
}

void Spc700::InstrSleep() {
    Idle();
    Idle();
    r.wait = true;
}

void Spc700::InstrStop() {
    Idle();
    Idle();
    r.stop = true;
}

// Step — Opcode dispatch (all 256 opcodes)

void Spc700::Step() {
    if (r.wait || r.stop) {
        Idle();
        return;
    }

    uint8_t opcode = Fetch();

    switch (opcode) {
    // 0x0X
    case 0x00: InstrNoOperation(); break;
    case 0x01: InstrCallTable(0); break;
    case 0x02: InstrAbsoluteBitSet(0, true); break;
    case 0x03: InstrBranchBit(0, true); break;
    case 0x04: InstrDirectRead(&Spc700::AlgOR, r.a); break;
    case 0x05: InstrAbsoluteRead(&Spc700::AlgOR, r.a); break;
    case 0x06: InstrIndirectXRead(&Spc700::AlgOR); break;
    case 0x07: InstrIndexedIndirectRead(&Spc700::AlgOR, r.x); break;
    case 0x08: InstrImmediateRead(&Spc700::AlgOR, r.a); break;
    case 0x09: InstrDirectDirectModify(&Spc700::AlgOR); break;
    case 0x0A: InstrAbsoluteBitModify(0); break;  // OR1 C,mem.bit
    case 0x0B: InstrDirectModify(&Spc700::AlgASL); break;
    case 0x0C: InstrAbsoluteModify(&Spc700::AlgASL); break;
    case 0x0D: InstrPushP(); break;
    case 0x0E: InstrTestSetBitsAbsolute(true); break;   // TSET1
    case 0x0F: InstrBreak(); break;

    // 0x1X
    case 0x10: InstrBranch(!r.p.n); break;  // BPL
    case 0x11: InstrCallTable(1); break;
    case 0x12: InstrAbsoluteBitSet(0, false); break;  // CLR1 dp.0
    case 0x13: InstrBranchBit(0, false); break;        // BBC dp.0
    case 0x14: InstrDirectIndexedRead(&Spc700::AlgOR, r.a, r.x); break;
    case 0x15: InstrAbsoluteIndexedRead(&Spc700::AlgOR, r.x); break;
    case 0x16: InstrAbsoluteIndexedRead(&Spc700::AlgOR, r.y); break;
    case 0x17: InstrIndirectIndexedRead(&Spc700::AlgOR, r.y); break;
    case 0x18: InstrDirectImmediateModify(&Spc700::AlgOR); break;
    case 0x19: InstrIndirectXWriteIndirectY(&Spc700::AlgOR); break;
    case 0x1A: InstrDirectModifyWord(-1); break;  // DECW
    case 0x1B: InstrDirectIndexedModify(&Spc700::AlgASL, r.x); break;
    case 0x1C: InstrImpliedModify(&Spc700::AlgASL, r.a); break;
    case 0x1D: InstrImpliedModify(&Spc700::AlgDEC, r.x); break;
    case 0x1E: InstrAbsoluteRead(&Spc700::AlgCMP, r.x); break;  // CMP X,abs
    case 0x1F: InstrJumpIndirectX(); break;

    // 0x2X
    case 0x20: InstrFlagSet(r.p.p, false); break;  // CLRP
    case 0x21: InstrCallTable(2); break;
    case 0x22: InstrAbsoluteBitSet(1, true); break;
    case 0x23: InstrBranchBit(1, true); break;
    case 0x24: InstrDirectRead(&Spc700::AlgAND, r.a); break;
    case 0x25: InstrAbsoluteRead(&Spc700::AlgAND, r.a); break;
    case 0x26: InstrIndirectXRead(&Spc700::AlgAND); break;
    case 0x27: InstrIndexedIndirectRead(&Spc700::AlgAND, r.x); break;
    case 0x28: InstrImmediateRead(&Spc700::AlgAND, r.a); break;
    case 0x29: InstrDirectDirectModify(&Spc700::AlgAND); break;
    case 0x2A: InstrAbsoluteBitModify(1); break;  // OR1 C,!mem.bit
    case 0x2B: InstrDirectModify(&Spc700::AlgROL); break;
    case 0x2C: InstrAbsoluteModify(&Spc700::AlgROL); break;
    case 0x2D: InstrPush(r.a); break;
    case 0x2E: InstrBranchNotDirect(); break;  // CBNE dp
    case 0x2F: InstrBranch(true); break;       // BRA

    // 0x3X
    case 0x30: InstrBranch(r.p.n); break;  // BMI
    case 0x31: InstrCallTable(3); break;
    case 0x32: InstrAbsoluteBitSet(1, false); break;
    case 0x33: InstrBranchBit(1, false); break;
    case 0x34: InstrDirectIndexedRead(&Spc700::AlgAND, r.a, r.x); break;
    case 0x35: InstrAbsoluteIndexedRead(&Spc700::AlgAND, r.x); break;
    case 0x36: InstrAbsoluteIndexedRead(&Spc700::AlgAND, r.y); break;
    case 0x37: InstrIndirectIndexedRead(&Spc700::AlgAND, r.y); break;
    case 0x38: InstrDirectImmediateModify(&Spc700::AlgAND); break;
    case 0x39: InstrIndirectXWriteIndirectY(&Spc700::AlgAND); break;
    case 0x3A: InstrDirectModifyWord(+1); break;  // INCW
    case 0x3B: InstrDirectIndexedModify(&Spc700::AlgROL, r.x); break;
    case 0x3C: InstrImpliedModify(&Spc700::AlgROL, r.a); break;
    case 0x3D: InstrImpliedModify(&Spc700::AlgINC, r.x); break;
    case 0x3E: InstrDirectRead(&Spc700::AlgCMP, r.x); break;  // CMP X,dp
    case 0x3F: InstrCallAbsolute(); break;

    // 0x4X
    case 0x40: InstrFlagSet(r.p.p, true); break;  // SETP
    case 0x41: InstrCallTable(4); break;
    case 0x42: InstrAbsoluteBitSet(2, true); break;
    case 0x43: InstrBranchBit(2, true); break;
    case 0x44: InstrDirectRead(&Spc700::AlgEOR, r.a); break;
    case 0x45: InstrAbsoluteRead(&Spc700::AlgEOR, r.a); break;
    case 0x46: InstrIndirectXRead(&Spc700::AlgEOR); break;
    case 0x47: InstrIndexedIndirectRead(&Spc700::AlgEOR, r.x); break;
    case 0x48: InstrImmediateRead(&Spc700::AlgEOR, r.a); break;
    case 0x49: InstrDirectDirectModify(&Spc700::AlgEOR); break;
    case 0x4A: InstrAbsoluteBitModify(2); break;  // AND1 C,mem.bit
    case 0x4B: InstrDirectModify(&Spc700::AlgLSR); break;
    case 0x4C: InstrAbsoluteModify(&Spc700::AlgLSR); break;
    case 0x4D: InstrPush(r.x); break;
    case 0x4E: InstrTestSetBitsAbsolute(false); break;  // TCLR1
    case 0x4F: InstrCallPage(); break;  // PCALL

    // 0x5X
    case 0x50: InstrBranch(!r.p.v); break;  // BVC
    case 0x51: InstrCallTable(5); break;
    case 0x52: InstrAbsoluteBitSet(2, false); break;
    case 0x53: InstrBranchBit(2, false); break;
    case 0x54: InstrDirectIndexedRead(&Spc700::AlgEOR, r.a, r.x); break;
    case 0x55: InstrAbsoluteIndexedRead(&Spc700::AlgEOR, r.x); break;
    case 0x56: InstrAbsoluteIndexedRead(&Spc700::AlgEOR, r.y); break;
    case 0x57: InstrIndirectIndexedRead(&Spc700::AlgEOR, r.y); break;
    case 0x58: InstrDirectImmediateModify(&Spc700::AlgEOR); break;
    case 0x59: InstrIndirectXWriteIndirectY(&Spc700::AlgEOR); break;
    case 0x5A: InstrDirectCompareWord(&Spc700::AlgCPW); break;  // CMPW
    case 0x5B: InstrDirectIndexedModify(&Spc700::AlgLSR, r.x); break;
    case 0x5C: InstrImpliedModify(&Spc700::AlgLSR, r.a); break;
    case 0x5D: InstrTransfer(r.a, r.x); break;  // MOV X,A
    case 0x5E: InstrAbsoluteRead(&Spc700::AlgCMP, r.y); break;  // CMP Y,abs
    case 0x5F: InstrJumpAbsolute(); break;

    // 0x6X
    case 0x60: InstrFlagSet(r.p.c, false); break;  // CLRC
    case 0x61: InstrCallTable(6); break;
    case 0x62: InstrAbsoluteBitSet(3, true); break;
    case 0x63: InstrBranchBit(3, true); break;
    case 0x64: InstrDirectRead(&Spc700::AlgCMP, r.a); break;
    case 0x65: InstrAbsoluteRead(&Spc700::AlgCMP, r.a); break;
    case 0x66: InstrIndirectXRead(&Spc700::AlgCMP); break;
    case 0x67: InstrIndexedIndirectRead(&Spc700::AlgCMP, r.x); break;
    case 0x68: InstrImmediateRead(&Spc700::AlgCMP, r.a); break;
    case 0x69: InstrDirectDirectCompare(&Spc700::AlgCMP); break;
    case 0x6A: InstrAbsoluteBitModify(3); break;  // AND1 C,!mem.bit
    case 0x6B: InstrDirectModify(&Spc700::AlgROR); break;
    case 0x6C: InstrAbsoluteModify(&Spc700::AlgROR); break;
    case 0x6D: InstrPush(r.y); break;
    case 0x6E: InstrBranchNotDirectDecrement(); break;  // DBNZ dp
    case 0x6F: InstrReturnSubroutine(); break;

    // 0x7X
    case 0x70: InstrBranch(r.p.v); break;  // BVS
    case 0x71: InstrCallTable(7); break;
    case 0x72: InstrAbsoluteBitSet(3, false); break;
    case 0x73: InstrBranchBit(3, false); break;
    case 0x74: InstrDirectIndexedRead(&Spc700::AlgCMP, r.a, r.x); break;
    case 0x75: InstrAbsoluteIndexedRead(&Spc700::AlgCMP, r.x); break;
    case 0x76: InstrAbsoluteIndexedRead(&Spc700::AlgCMP, r.y); break;
    case 0x77: InstrIndirectIndexedRead(&Spc700::AlgCMP, r.y); break;
    case 0x78: InstrDirectImmediateCompare(&Spc700::AlgCMP); break;
    case 0x79: InstrIndirectXCompareIndirectY(&Spc700::AlgCMP); break;
    case 0x7A: InstrDirectReadWord(&Spc700::AlgADW); break;  // ADDW
    case 0x7B: InstrDirectIndexedModify(&Spc700::AlgROR, r.x); break;
    case 0x7C: InstrImpliedModify(&Spc700::AlgROR, r.a); break;
    case 0x7D: InstrTransfer(r.x, r.a); break;  // MOV A,X
    case 0x7E: InstrDirectRead(&Spc700::AlgCMP, r.y); break;  // CMP Y,dp
    case 0x7F: InstrReturnInterrupt(); break;

    // 0x8X
    case 0x80: InstrFlagSet(r.p.c, true); break;  // SETC
    case 0x81: InstrCallTable(8); break;
    case 0x82: InstrAbsoluteBitSet(4, true); break;
    case 0x83: InstrBranchBit(4, true); break;
    case 0x84: InstrDirectRead(&Spc700::AlgADC, r.a); break;
    case 0x85: InstrAbsoluteRead(&Spc700::AlgADC, r.a); break;
    case 0x86: InstrIndirectXRead(&Spc700::AlgADC); break;
    case 0x87: InstrIndexedIndirectRead(&Spc700::AlgADC, r.x); break;
    case 0x88: InstrImmediateRead(&Spc700::AlgADC, r.a); break;
    case 0x89: InstrDirectDirectModify(&Spc700::AlgADC); break;
    case 0x8A: InstrAbsoluteBitModify(4); break;  // EOR1
    case 0x8B: InstrDirectModify(&Spc700::AlgDEC); break;
    case 0x8C: InstrAbsoluteModify(&Spc700::AlgDEC); break;
    case 0x8D: InstrImmediateRead(&Spc700::AlgLD, r.y); break;  // MOV Y,#imm
    case 0x8E: InstrPullP(); break;  // POP PSW
    case 0x8F: InstrDirectImmediateWrite(); break;  // MOV dp,#imm

    // 0x9X
    case 0x90: InstrBranch(!r.p.c); break;  // BCC
    case 0x91: InstrCallTable(9); break;
    case 0x92: InstrAbsoluteBitSet(4, false); break;
    case 0x93: InstrBranchBit(4, false); break;
    case 0x94: InstrDirectIndexedRead(&Spc700::AlgADC, r.a, r.x); break;
    case 0x95: InstrAbsoluteIndexedRead(&Spc700::AlgADC, r.x); break;
    case 0x96: InstrAbsoluteIndexedRead(&Spc700::AlgADC, r.y); break;
    case 0x97: InstrIndirectIndexedRead(&Spc700::AlgADC, r.y); break;
    case 0x98: InstrDirectImmediateModify(&Spc700::AlgADC); break;
    case 0x99: InstrIndirectXWriteIndirectY(&Spc700::AlgADC); break;
    case 0x9A: InstrDirectReadWord(&Spc700::AlgSBW); break;  // SUBW
    case 0x9B: InstrDirectIndexedModify(&Spc700::AlgDEC, r.x); break;
    case 0x9C: InstrImpliedModify(&Spc700::AlgDEC, r.a); break;
    case 0x9D: InstrTransfer(r.s, r.x); break;  // MOV X,SP (TSX)
    case 0x9E: InstrDivide(); break;
    case 0x9F: InstrExchangeNibble(); break;

    // 0xAX
    case 0xA0: InstrFlagSet(r.p.i, true); break;  // EI
    case 0xA1: InstrCallTable(10); break;
    case 0xA2: InstrAbsoluteBitSet(5, true); break;
    case 0xA3: InstrBranchBit(5, true); break;
    case 0xA4: InstrDirectRead(&Spc700::AlgSBC, r.a); break;
    case 0xA5: InstrAbsoluteRead(&Spc700::AlgSBC, r.a); break;
    case 0xA6: InstrIndirectXRead(&Spc700::AlgSBC); break;
    case 0xA7: InstrIndexedIndirectRead(&Spc700::AlgSBC, r.x); break;
    case 0xA8: InstrImmediateRead(&Spc700::AlgSBC, r.a); break;
    case 0xA9: InstrDirectDirectModify(&Spc700::AlgSBC); break;
    case 0xAA: InstrAbsoluteBitModify(5); break;  // MOV1 C,mem.bit
    case 0xAB: InstrDirectModify(&Spc700::AlgINC); break;
    case 0xAC: InstrAbsoluteModify(&Spc700::AlgINC); break;
    case 0xAD: InstrImmediateRead(&Spc700::AlgCMP, r.y); break;  // CMP Y,#imm
    case 0xAE: InstrPull(r.a); break;  // POP A
    case 0xAF: InstrIndirectXIncrementWrite(r.a); break;  // MOV (X++),A

    // 0xBX
    case 0xB0: InstrBranch(r.p.c); break;  // BCS
    case 0xB1: InstrCallTable(11); break;
    case 0xB2: InstrAbsoluteBitSet(5, false); break;
    case 0xB3: InstrBranchBit(5, false); break;
    case 0xB4: InstrDirectIndexedRead(&Spc700::AlgSBC, r.a, r.x); break;
    case 0xB5: InstrAbsoluteIndexedRead(&Spc700::AlgSBC, r.x); break;
    case 0xB6: InstrAbsoluteIndexedRead(&Spc700::AlgSBC, r.y); break;
    case 0xB7: InstrIndirectIndexedRead(&Spc700::AlgSBC, r.y); break;
    case 0xB8: InstrDirectImmediateModify(&Spc700::AlgSBC); break;
    case 0xB9: InstrIndirectXWriteIndirectY(&Spc700::AlgSBC); break;
    case 0xBA: InstrDirectReadWord(&Spc700::AlgLDW); break;  // MOVW YA,dp
    case 0xBB: InstrDirectIndexedModify(&Spc700::AlgINC, r.x); break;
    case 0xBC: InstrImpliedModify(&Spc700::AlgINC, r.a); break;
    case 0xBD: InstrTransfer(r.x, r.s); break;  // MOV SP,X (TXS)
    case 0xBE: InstrDecimalAdjustSub(); break;  // DAS
    case 0xBF: InstrIndirectXIncrementRead(r.a); break;  // MOV A,(X++)

    // 0xCX
    case 0xC0: InstrFlagSet(r.p.i, false); break;  // DI
    case 0xC1: InstrCallTable(12); break;
    case 0xC2: InstrAbsoluteBitSet(6, true); break;
    case 0xC3: InstrBranchBit(6, true); break;
    case 0xC4: InstrDirectWrite(r.a); break;  // MOV dp,A
    case 0xC5: InstrAbsoluteWrite(r.a); break;  // MOV abs,A
    case 0xC6: InstrIndirectXWrite(r.a); break;  // MOV (X),A
    case 0xC7: InstrIndexedIndirectWrite(r.a, r.x); break;
    case 0xC8: InstrImmediateRead(&Spc700::AlgCMP, r.x); break;  // CMP X,#imm
    case 0xC9: InstrAbsoluteWrite(r.x); break;  // MOV abs,X
    case 0xCA: InstrAbsoluteBitModify(6); break;  // MOV1 mem.bit,C
    case 0xCB: InstrDirectWrite(r.y); break;  // MOV dp,Y
    case 0xCC: InstrAbsoluteWrite(r.y); break;  // MOV abs,Y
    case 0xCD: InstrImmediateRead(&Spc700::AlgLD, r.x); break;  // MOV X,#imm
    case 0xCE: InstrPull(r.x); break;  // POP X
    case 0xCF: InstrMultiply(); break;

    // 0xDX
    case 0xD0: InstrBranch(!r.p.z); break;  // BNE
    case 0xD1: InstrCallTable(13); break;
    case 0xD2: InstrAbsoluteBitSet(6, false); break;
    case 0xD3: InstrBranchBit(6, false); break;
    case 0xD4: InstrDirectIndexedWrite(r.a, r.x); break;
    case 0xD5: InstrAbsoluteIndexedWrite(r.x); break;  // MOV abs+X,A
    case 0xD6: InstrAbsoluteIndexedWrite(r.y); break;  // MOV abs+Y,A
    case 0xD7: InstrIndirectIndexedWrite(r.a, r.y); break;
    case 0xD8: InstrDirectWrite(r.x); break;  // MOV dp,X
    case 0xD9: InstrDirectIndexedWrite(r.x, r.y); break;  // MOV dp+Y,X
    case 0xDA: InstrDirectWriteWord(); break;  // MOVW dp,YA
    case 0xDB: InstrDirectIndexedWrite(r.y, r.x); break;  // MOV dp+X,Y
    case 0xDC: InstrImpliedModify(&Spc700::AlgDEC, r.y); break;
    case 0xDD: InstrTransfer(r.y, r.a); break;  // MOV A,Y
    case 0xDE: InstrBranchNotDirectIndexed(r.x); break;  // CBNE dp+X
    case 0xDF: InstrDecimalAdjustAdd(); break;  // DAA

    // 0xEX
    case 0xE0: InstrOverflowClear(); break;  // CLRV
    case 0xE1: InstrCallTable(14); break;
    case 0xE2: InstrAbsoluteBitSet(7, true); break;
    case 0xE3: InstrBranchBit(7, true); break;
    case 0xE4: InstrDirectRead(&Spc700::AlgLD, r.a); break;  // MOV A,dp
    case 0xE5: InstrAbsoluteRead(&Spc700::AlgLD, r.a); break;  // MOV A,abs
    case 0xE6: { // MOV A,(X)
        Idle();
        r.a = AlgLD(r.a, Load(r.x));
        break;
    }
    case 0xE7: InstrIndexedIndirectRead(&Spc700::AlgLD, r.x); break;  // MOV A,[dp+X]
    case 0xE8: InstrImmediateRead(&Spc700::AlgLD, r.a); break;  // MOV A,#imm
    case 0xE9: InstrAbsoluteRead(&Spc700::AlgLD, r.x); break;  // MOV X,abs
    case 0xEA: InstrAbsoluteBitModify(7); break;  // NOT1
    case 0xEB: InstrDirectRead(&Spc700::AlgLD, r.y); break;  // MOV Y,dp
    case 0xEC: InstrAbsoluteRead(&Spc700::AlgLD, r.y); break;  // MOV Y,abs
    case 0xED: InstrComplementCarry(); break;  // CMC
    case 0xEE: InstrPull(r.y); break;  // POP Y
    case 0xEF: InstrSleep(); break;

    // 0xFX
    case 0xF0: InstrBranch(r.p.z); break;  // BEQ
    case 0xF1: InstrCallTable(15); break;
    case 0xF2: InstrAbsoluteBitSet(7, false); break;
    case 0xF3: InstrBranchBit(7, false); break;
    case 0xF4: InstrDirectIndexedRead(&Spc700::AlgLD, r.a, r.x); break;  // MOV A,dp+X
    case 0xF5: InstrAbsoluteIndexedRead(&Spc700::AlgLD, r.x); break;  // MOV A,abs+X
    case 0xF6: InstrAbsoluteIndexedRead(&Spc700::AlgLD, r.y); break;  // MOV A,abs+Y
    case 0xF7: InstrIndirectIndexedRead(&Spc700::AlgLD, r.y); break;  // MOV A,[dp]+Y
    case 0xF8: InstrDirectRead(&Spc700::AlgLD, r.x); break;  // MOV X,dp
    case 0xF9: InstrDirectIndexedRead(&Spc700::AlgLD, r.x, r.y); break;  // MOV X,dp+Y
    case 0xFA: InstrDirectDirectWrite(); break;  // MOV dp,dp
    case 0xFB: InstrDirectIndexedRead(&Spc700::AlgLD, r.y, r.x); break;  // MOV Y,dp+X
    case 0xFC: InstrImpliedModify(&Spc700::AlgINC, r.y); break;
    case 0xFD: InstrTransfer(r.a, r.y); break;  // MOV Y,A
    case 0xFE: InstrBranchNotYDecrement(); break;  // DBNZ Y
    case 0xFF: InstrStop(); break;
    }
}

} // namespace snes::core
