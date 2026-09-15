// snes emulator
// core/src/audio/Spc700.cpp
// The SPC700 instruction core and audio processor state.

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

void* Spc700::AllocateCoroutineFrame(std::size_t size) {
    const std::size_t allocationSize = sizeof(CoroutineFrameHeader) + size;
    void* allocation = framePool_.allocate(allocationSize, alignof(CoroutineFrameHeader));
    auto* header = new (allocation) CoroutineFrameHeader{&framePool_, allocationSize};
    return header + 1;
}

void Spc700::ReleaseCoroutineFrame(void* frame) noexcept {
    if (!frame) return;
    auto* header = static_cast<CoroutineFrameHeader*>(frame) - 1;
    auto* resource = header->resource;
    const std::size_t allocationSize = header->allocationSize;
    header->~CoroutineFrameHeader();
    resource->deallocate(header, allocationSize, alignof(CoroutineFrameHeader));
}

void Spc700::Power() {
    executor_.Reset();
    activeCoroutine_ = {};
    pendingCycle_ = {};
    atInstructionBoundary_ = true;
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

void Spc700::IdleCycle::await_suspend(std::coroutine_handle<> handle) const noexcept {
    cpu->activeCoroutine_ = handle;
    cpu->pendingCycle_ = PendingCycle{PendingCycleKind::Idle};
}

void Spc700::ReadCycle::await_suspend(std::coroutine_handle<> handle) noexcept {
    cpu->activeCoroutine_ = handle;
    cpu->pendingCycle_ = PendingCycle{PendingCycleKind::Read, address, 0, &result};
}

void Spc700::WriteCycle::await_suspend(std::coroutine_handle<> handle) const noexcept {
    cpu->activeCoroutine_ = handle;
    cpu->pendingCycle_ = PendingCycle{PendingCycleKind::Write, address, data, nullptr};
}

void Spc700::InstructionBoundary::await_suspend(std::coroutine_handle<> handle) const noexcept {
    cpu->activeCoroutine_ = handle;
    cpu->atInstructionBoundary_ = true;
}

Spc700::ReadCycle Spc700::Fetch() {
    return ReadCycleAt(r.pc++);
}

Spc700::ReadCycle Spc700::Load(uint8_t addr) {
    return ReadCycleAt(static_cast<uint16_t>(r.p.p ? 0x0100 : 0x0000) | addr);
}

Spc700::WriteCycle Spc700::Store(uint8_t addr, uint8_t data) {
    return WriteCycleAt(static_cast<uint16_t>(r.p.p ? 0x0100 : 0x0000) | addr, data);
}

Spc700::ReadCycle Spc700::Pull() {
    return ReadCycleAt(0x0100 | static_cast<uint16_t>(++r.s));
}

Spc700::WriteCycle Spc700::Push(uint8_t data) {
    return WriteCycleAt(0x0100 | static_cast<uint16_t>(r.s--), data);
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
Spc700::Routine Spc700::InstrImmediateRead(AlgOp op, uint8_t& target) {
    uint8_t imm = co_await Fetch();
    target = (this->*op)(target, imm);
}

// Direct page: op A, dp
Spc700::Routine Spc700::InstrDirectRead(AlgOp op, uint8_t& target) {
    uint8_t dp = co_await Fetch();
    uint8_t val = co_await Load(dp);
    target = (this->*op)(target, val);
}

// Direct page modify: op dp
Spc700::Routine Spc700::InstrDirectModify(ModOp op) {
    uint8_t dp = co_await Fetch();
    uint8_t val = co_await Load(dp);
    val = (this->*op)(val);
    co_await Store(dp, val);
}

// Direct page write: MOV dp, reg
Spc700::Routine Spc700::InstrDirectWrite(uint8_t data) {
    uint8_t dp = co_await Fetch();
    (void)co_await Load(dp); // dummy read (bus timing)
    co_await Store(dp, data);
}

// Direct page indexed: op A, dp+X
Spc700::Routine Spc700::InstrDirectIndexedRead(AlgOp op, uint8_t& target, uint8_t index) {
    uint8_t dp = co_await Fetch();
    co_await WaitCycle();
    uint8_t val = co_await Load(static_cast<uint8_t>(dp + index));
    target = (this->*op)(target, val);
}

// Direct page indexed modify: op dp+X
Spc700::Routine Spc700::InstrDirectIndexedModify(ModOp op, uint8_t index) {
    uint8_t dp = co_await Fetch();
    co_await WaitCycle();
    uint8_t val = co_await Load(static_cast<uint8_t>(dp + index));
    val = (this->*op)(val);
    co_await Store(static_cast<uint8_t>(dp + index), val);
}

// Direct page indexed write: MOV dp+X, reg
Spc700::Routine Spc700::InstrDirectIndexedWrite(uint8_t data, uint8_t index) {
    uint8_t dp = co_await Fetch();
    co_await WaitCycle();
    (void)co_await Load(static_cast<uint8_t>(dp + index)); // dummy read
    co_await Store(static_cast<uint8_t>(dp + index), data);
}

// Absolute: op A, abs
Spc700::Routine Spc700::InstrAbsoluteRead(AlgOp op, uint8_t& target) {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    uint16_t addr = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint8_t val = co_await ReadCycleAt(addr);
    target = (this->*op)(target, val);
}

// Absolute modify: op abs
Spc700::Routine Spc700::InstrAbsoluteModify(ModOp op) {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    uint16_t addr = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint8_t val = co_await ReadCycleAt(addr);
    val = (this->*op)(val);
    co_await WriteCycleAt(addr, val);
}

// Absolute write: MOV abs, reg
Spc700::Routine Spc700::InstrAbsoluteWrite(uint8_t data) {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    uint16_t addr = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    (void)co_await ReadCycleAt(addr); // dummy read
    co_await WriteCycleAt(addr, data);
}

// Absolute indexed: op A, abs+X
Spc700::Routine Spc700::InstrAbsoluteIndexedRead(AlgOp op, uint8_t index) {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    co_await WaitCycle();
    uint16_t addr = (static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8)) + index;
    uint8_t val = co_await ReadCycleAt(addr);
    r.a = (this->*op)(r.a, val);
}

// Absolute indexed write: MOV abs+X, A
Spc700::Routine Spc700::InstrAbsoluteIndexedWrite(uint8_t index) {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    co_await WaitCycle();
    uint16_t addr = (static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8)) + index;
    (void)co_await ReadCycleAt(addr); // dummy read
    co_await WriteCycleAt(addr, r.a);
}

// Indexed indirect: op A, [dp+X]
Spc700::Routine Spc700::InstrIndexedIndirectRead(AlgOp op, uint8_t index) {
    uint8_t dp = co_await Fetch();
    co_await WaitCycle();
    uint8_t ptrL = co_await Load(static_cast<uint8_t>(dp + index));
    uint8_t ptrH = co_await Load(static_cast<uint8_t>(dp + index + 1));
    uint16_t addr = static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8);
    uint8_t val = co_await ReadCycleAt(addr);
    r.a = (this->*op)(r.a, val);
}

// Indexed indirect write: MOV [dp+X], A
Spc700::Routine Spc700::InstrIndexedIndirectWrite(uint8_t data, uint8_t index) {
    uint8_t dp = co_await Fetch();
    co_await WaitCycle();
    uint8_t ptrL = co_await Load(static_cast<uint8_t>(dp + index));
    uint8_t ptrH = co_await Load(static_cast<uint8_t>(dp + index + 1));
    uint16_t addr = static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8);
    (void)co_await ReadCycleAt(addr); // dummy read
    co_await WriteCycleAt(addr, data);
}

// Indirect indexed: op A, [dp]+Y
Spc700::Routine Spc700::InstrIndirectIndexedRead(AlgOp op, uint8_t index) {
    uint8_t dp = co_await Fetch();
    uint8_t ptrL = co_await Load(dp);
    uint8_t ptrH = co_await Load(static_cast<uint8_t>(dp + 1));
    co_await WaitCycle();
    uint16_t addr = (static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8)) + index;
    uint8_t val = co_await ReadCycleAt(addr);
    r.a = (this->*op)(r.a, val);
}

// Indirect indexed write: MOV [dp]+Y, A
Spc700::Routine Spc700::InstrIndirectIndexedWrite(uint8_t data, uint8_t index) {
    uint8_t dp = co_await Fetch();
    uint8_t ptrL = co_await Load(dp);
    uint8_t ptrH = co_await Load(static_cast<uint8_t>(dp + 1));
    co_await WaitCycle();
    uint16_t addr = (static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8)) + index;
    (void)co_await ReadCycleAt(addr); // dummy read
    co_await WriteCycleAt(addr, data);
}

// Indirect X: op A, (X)
Spc700::Routine Spc700::InstrIndirectXRead(AlgOp op) {
    co_await WaitCycle();
    uint8_t val = co_await Load(r.x);
    r.a = (this->*op)(r.a, val);
}

// Indirect X write: MOV (X), A
Spc700::Routine Spc700::InstrIndirectXWrite(uint8_t data) {
    co_await WaitCycle();
    (void)co_await Load(r.x); // dummy read
    co_await Store(r.x, data);
}

// MOV A,(X++)
Spc700::Routine Spc700::InstrIndirectXIncrementRead(uint8_t& target) {
    co_await WaitCycle();
    target = AlgLD(target, co_await Load(r.x++));
}

// MOV (X++),A
Spc700::Routine Spc700::InstrIndirectXIncrementWrite(uint8_t data) {
    co_await WaitCycle();
    co_await WaitCycle();
    co_await Store(r.x++, data);
}

// CMP (X),(Y)
Spc700::Routine Spc700::InstrIndirectXCompareIndirectY(AlgOp op) {
    co_await WaitCycle();
    uint8_t yVal = co_await Load(r.y);
    uint8_t xVal = co_await Load(r.x);
    (this->*op)(xVal, yVal);
}

// op (X)=(Y) — e.g. ADC (X)=(Y), OR (X)=(Y)
Spc700::Routine Spc700::InstrIndirectXWriteIndirectY(AlgOp op) {
    co_await WaitCycle();
    uint8_t yVal = co_await Load(r.y);
    uint8_t xVal = co_await Load(r.x);
    xVal = (this->*op)(xVal, yVal);
    co_await Store(r.x, xVal);
}

// Direct-Direct, Direct-Immediate instruction groups

// CMP dp, dp
Spc700::Routine Spc700::InstrDirectDirectCompare(AlgOp op) {
    uint8_t srcDp = co_await Fetch();
    uint8_t srcVal = co_await Load(srcDp);
    uint8_t dstDp = co_await Fetch();
    uint8_t dstVal = co_await Load(dstDp);
    (this->*op)(dstVal, srcVal);
}

// op dp, dp (e.g. ADC dp,dp)
Spc700::Routine Spc700::InstrDirectDirectModify(AlgOp op) {
    uint8_t srcDp = co_await Fetch();
    uint8_t srcVal = co_await Load(srcDp);
    uint8_t dstDp = co_await Fetch();
    uint8_t dstVal = co_await Load(dstDp);
    dstVal = (this->*op)(dstVal, srcVal);
    co_await Store(dstDp, dstVal);
}

// MOV dp, dp
Spc700::Routine Spc700::InstrDirectDirectWrite() {
    uint8_t srcDp = co_await Fetch();
    uint8_t srcVal = co_await Load(srcDp);
    uint8_t dstDp = co_await Fetch();
    co_await Store(dstDp, srcVal);
}

// CMP dp, #imm
Spc700::Routine Spc700::InstrDirectImmediateCompare(AlgOp op) {
    uint8_t imm = co_await Fetch();
    uint8_t dp = co_await Fetch();
    uint8_t val = co_await Load(dp);
    (this->*op)(val, imm);
}

// op dp, #imm (e.g. ADC dp,#imm)
Spc700::Routine Spc700::InstrDirectImmediateModify(AlgOp op) {
    uint8_t imm = co_await Fetch();
    uint8_t dp = co_await Fetch();
    uint8_t val = co_await Load(dp);
    val = (this->*op)(val, imm);
    co_await Store(dp, val);
}

// MOV dp, #imm
Spc700::Routine Spc700::InstrDirectImmediateWrite() {
    uint8_t imm = co_await Fetch();
    uint8_t dp = co_await Fetch();
    (void)co_await Load(dp); // dummy read
    co_await Store(dp, imm);
}

// 16-bit word operations

// CMPW YA, dp
Spc700::Routine Spc700::InstrDirectCompareWord(AlgOp16 op) {
    uint8_t dp = co_await Fetch();
    uint8_t lo = co_await Load(dp);
    uint8_t hi = co_await Load(static_cast<uint8_t>(dp + 1));
    uint16_t val = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    (this->*op)(r.ya(), val);
}

// ADDW/SUBW/MOVW YA, dp
Spc700::Routine Spc700::InstrDirectReadWord(AlgOp16 op) {
    uint8_t dp = co_await Fetch();
    uint8_t lo = co_await Load(dp);
    co_await WaitCycle();
    uint8_t hi = co_await Load(static_cast<uint8_t>(dp + 1));
    uint16_t val = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint16_t result = (this->*op)(r.ya(), val);
    r.setYA(result);
}

// INCW/DECW dp
Spc700::Routine Spc700::InstrDirectModifyWord(int16_t adjust) {
    uint8_t dp = co_await Fetch();
    uint8_t lo = co_await Load(dp);
    lo += static_cast<uint8_t>(adjust);
    co_await Store(dp, lo);
    uint8_t hi = co_await Load(static_cast<uint8_t>(dp + 1));
    // Carry from low byte: if adjust=+1 and lo overflowed (now 0), or adjust=-1 and lo underflowed (now 0xFF)
    hi += (adjust >= 0 ? (lo == 0 ? 1 : 0) : (lo == 0xFF ? static_cast<uint8_t>(-1) : 0));
    co_await Store(static_cast<uint8_t>(dp + 1), hi);
    uint16_t result = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    r.p.z = result == 0;
    r.p.n = (result & 0x8000) != 0;
}

// MOVW dp, YA
Spc700::Routine Spc700::InstrDirectWriteWord() {
    uint8_t dp = co_await Fetch();
    (void)co_await Load(dp); // dummy read
    co_await Store(dp, r.a);
    co_await Store(static_cast<uint8_t>(dp + 1), r.y);
}

// Bit operations

// Absolute bit modify — 8 sub-modes via opcode bits
Spc700::Routine Spc700::InstrAbsoluteBitModify(uint8_t mode) {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    uint16_t raw = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint8_t bit = static_cast<uint8_t>(raw >> 13);
    uint16_t addr = raw & 0x1FFF;
    uint8_t val = co_await ReadCycleAt(addr);
    uint8_t mask = static_cast<uint8_t>(1 << bit);

    switch (mode) {
    case 0: // OR1  C, mem.bit
        co_await WaitCycle();
        r.p.c = r.p.c | ((val & mask) != 0);
        break;
    case 1: // OR1  C, !mem.bit
        co_await WaitCycle();
        r.p.c = r.p.c | ((val & mask) == 0);
        break;
    case 2: // AND1 C, mem.bit
        r.p.c = r.p.c & ((val & mask) != 0);
        break;
    case 3: // AND1 C, !mem.bit
        r.p.c = r.p.c & ((val & mask) == 0);
        break;
    case 4: // EOR1 C, mem.bit
        co_await WaitCycle();
        r.p.c = r.p.c ^ ((val & mask) != 0);
        break;
    case 5: // MOV1 C, mem.bit (load)
        r.p.c = (val & mask) != 0;
        break;
    case 6: // MOV1 mem.bit, C (store)
        co_await WaitCycle();
        if (r.p.c) val |= mask; else val &= ~mask;
        co_await WriteCycleAt(addr, val);
        break;
    case 7: // NOT1 mem.bit
        val ^= mask;
        co_await WriteCycleAt(addr, val);
        break;
    }
}

// SET1/CLR1 dp.bit
Spc700::Routine Spc700::InstrAbsoluteBitSet(uint8_t bit, bool value) {
    uint8_t dp = co_await Fetch();
    uint8_t val = co_await Load(dp);
    uint8_t mask = static_cast<uint8_t>(1 << bit);
    if (value) val |= mask; else val &= ~mask;
    co_await Store(dp, val);
}

// TSET1/TCLR1 abs
Spc700::Routine Spc700::InstrTestSetBitsAbsolute(bool set) {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    uint16_t addr = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    uint8_t val = co_await ReadCycleAt(addr);
    (void)co_await ReadCycleAt(addr); // dummy read
    // Flags: N/Z set from (A - val) (like CMP)
    uint8_t diff = static_cast<uint8_t>(r.a - val);
    r.p.z = diff == 0;
    r.p.n = (diff & 0x80) != 0;
    // Modify memory
    if (set) val |= r.a; else val &= ~r.a;
    co_await WriteCycleAt(addr, val);
}

// Branch instructions

Spc700::Routine Spc700::InstrBranch(bool take) {
    uint8_t offset = co_await Fetch();
    if (!take) co_return;
    co_await WaitCycle();
    co_await WaitCycle();
    r.pc += static_cast<int8_t>(offset);
}

Spc700::Routine Spc700::InstrBranchBit(uint8_t bit, bool match) {
    uint8_t dp = co_await Fetch();
    uint8_t val = co_await Load(dp);
    co_await WaitCycle();
    uint8_t offset = co_await Fetch();
    bool bitSet = (val & (1 << bit)) != 0;
    if (bitSet != match) co_return;
    co_await WaitCycle();
    co_await WaitCycle();
    r.pc += static_cast<int8_t>(offset);
}

// CBNE dp, rel — compare A != mem, branch
Spc700::Routine Spc700::InstrBranchNotDirect() {
    uint8_t dp = co_await Fetch();
    uint8_t val = co_await Load(dp);
    co_await WaitCycle();
    uint8_t offset = co_await Fetch();
    if (r.a == val) co_return;
    co_await WaitCycle();
    co_await WaitCycle();
    r.pc += static_cast<int8_t>(offset);
}

// CBNE dp+X, rel
Spc700::Routine Spc700::InstrBranchNotDirectIndexed(uint8_t index) {
    uint8_t dp = co_await Fetch();
    co_await WaitCycle();
    uint8_t val = co_await Load(static_cast<uint8_t>(dp + index));
    co_await WaitCycle();
    uint8_t offset = co_await Fetch();
    if (r.a == val) co_return;
    co_await WaitCycle();
    co_await WaitCycle();
    r.pc += static_cast<int8_t>(offset);
}

// DBNZ dp, rel — decrement mem, branch if non-zero
Spc700::Routine Spc700::InstrBranchNotDirectDecrement() {
    uint8_t dp = co_await Fetch();
    uint8_t val = co_await Load(dp);
    val--;
    co_await Store(dp, val);
    uint8_t offset = co_await Fetch();
    if (val == 0) co_return;
    co_await WaitCycle();
    co_await WaitCycle();
    r.pc += static_cast<int8_t>(offset);
}

// DBNZ Y, rel
Spc700::Routine Spc700::InstrBranchNotYDecrement() {
    co_await WaitCycle();
    co_await WaitCycle();
    uint8_t offset = co_await Fetch();
    if (--r.y == 0) co_return;
    co_await WaitCycle();
    co_await WaitCycle();
    r.pc += static_cast<int8_t>(offset);
}

// Flow control

// CALL abs
Spc700::Routine Spc700::InstrCallAbsolute() {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    co_await WaitCycle();
    co_await Push(static_cast<uint8_t>(r.pc >> 8));
    co_await Push(static_cast<uint8_t>(r.pc));
    co_await WaitCycle();
    co_await WaitCycle();
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// PCALL $FFxx
Spc700::Routine Spc700::InstrCallPage() {
    uint8_t lo = co_await Fetch();
    co_await WaitCycle();
    co_await Push(static_cast<uint8_t>(r.pc >> 8));
    co_await Push(static_cast<uint8_t>(r.pc));
    co_await WaitCycle();
    r.pc = 0xFF00 | lo;
}

// TCALL n — vector at $FFDE - (n * 2)
Spc700::Routine Spc700::InstrCallTable(uint8_t vector) {
    co_await WaitCycle();
    co_await WaitCycle();
    co_await Push(static_cast<uint8_t>(r.pc >> 8));
    co_await Push(static_cast<uint8_t>(r.pc));
    co_await WaitCycle();
    uint16_t vecAddr = static_cast<uint16_t>(0xFFDE - (vector << 1));
    uint8_t lo = co_await ReadCycleAt(vecAddr);
    uint8_t hi = co_await ReadCycleAt(static_cast<uint16_t>(vecAddr + 1));
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// JMP abs
Spc700::Routine Spc700::InstrJumpAbsolute() {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// JMP [abs+X]
Spc700::Routine Spc700::InstrJumpIndirectX() {
    uint8_t lo = co_await Fetch();
    uint8_t hi = co_await Fetch();
    co_await WaitCycle();
    uint16_t addr = (static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8)) + r.x;
    uint8_t ptrL = co_await ReadCycleAt(addr);
    uint8_t ptrH = co_await ReadCycleAt(static_cast<uint16_t>(addr + 1));
    r.pc = static_cast<uint16_t>(ptrL) | (static_cast<uint16_t>(ptrH) << 8);
}

// RET
Spc700::Routine Spc700::InstrReturnSubroutine() {
    co_await WaitCycle();
    co_await WaitCycle();
    uint8_t lo = co_await Pull();
    uint8_t hi = co_await Pull();
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// RETI
Spc700::Routine Spc700::InstrReturnInterrupt() {
    co_await WaitCycle();
    co_await WaitCycle();
    r.p.Unpack(co_await Pull());
    uint8_t lo = co_await Pull();
    uint8_t hi = co_await Pull();
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// BRK
Spc700::Routine Spc700::InstrBreak() {
    uint8_t lo = co_await ReadCycleAt(0xFFDE);
    uint8_t hi = co_await ReadCycleAt(0xFFDF);
    co_await WaitCycle();
    co_await Push(static_cast<uint8_t>(r.pc >> 8));
    co_await Push(static_cast<uint8_t>(r.pc));
    co_await Push(r.p.Pack());
    co_await WaitCycle();
    r.p.i = false;
    r.p.b = true;
    r.pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// Register transfer

Spc700::Routine Spc700::InstrTransfer(uint8_t from, uint8_t& to) {
    co_await WaitCycle();
    to = from;
    // No flag changes if target is SP
    if (&to != &r.s) {
        r.p.z = to == 0;
        r.p.n = (to & 0x80) != 0;
    }
}

// Push / Pull

Spc700::Routine Spc700::InstrPush(uint8_t data) {
    co_await WaitCycle();
    co_await WaitCycle();
    co_await Push(data);
}

Spc700::Routine Spc700::InstrPull(uint8_t& target) {
    co_await WaitCycle();
    co_await WaitCycle();
    target = co_await Pull();
}

Spc700::Routine Spc700::InstrPushP() {
    co_await WaitCycle();
    co_await WaitCycle();
    co_await Push(r.p.Pack());
}

Spc700::Routine Spc700::InstrPullP() {
    co_await WaitCycle();
    co_await WaitCycle();
    r.p.Unpack(co_await Pull());
}

// Flag manipulation

Spc700::Routine Spc700::InstrFlagSet(bool& flag, bool value) {
    co_await WaitCycle();
    co_await WaitCycle();
    flag = value;
}

Spc700::Routine Spc700::InstrOverflowClear() {
    co_await WaitCycle();
    co_await WaitCycle();
    r.p.h = false;
    r.p.v = false;
}

Spc700::Routine Spc700::InstrComplementCarry() {
    co_await WaitCycle();
    co_await WaitCycle();
    r.p.c = !r.p.c;
}

// Implied register single-operand modify

Spc700::Routine Spc700::InstrImpliedModify(ModOp op, uint8_t& target) {
    co_await WaitCycle();
    target = (this->*op)(target);
}

// Special instructions

// MUL YA — Y*A → YA (16-bit result), flags on Y
Spc700::Routine Spc700::InstrMultiply() {
    co_await WaitCycle();
    for (int i = 0; i < 8; i++) co_await WaitCycle();
    uint16_t result = static_cast<uint16_t>(r.y) * static_cast<uint16_t>(r.a);
    r.setYA(result);
    r.p.z = r.y == 0;
    r.p.n = (r.y & 0x80) != 0;
}

// DIV YA,X — YA / X → A (quotient), Y (remainder)
Spc700::Routine Spc700::InstrDivide() {
    co_await WaitCycle();
    for (int i = 0; i < 11; i++) co_await WaitCycle();

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
Spc700::Routine Spc700::InstrDecimalAdjustAdd() {
    co_await WaitCycle();
    co_await WaitCycle();
    if (r.p.c || r.a > 0x99) { r.a += 0x60; r.p.c = true; }
    if (r.p.h || (r.a & 0x0F) > 0x09) { r.a += 0x06; }
    r.p.z = r.a == 0;
    r.p.n = (r.a & 0x80) != 0;
}

// DAS — BCD adjust after subtraction
Spc700::Routine Spc700::InstrDecimalAdjustSub() {
    co_await WaitCycle();
    co_await WaitCycle();
    if (!r.p.c || r.a > 0x99) { r.a -= 0x60; r.p.c = false; }
    if (!r.p.h || (r.a & 0x0F) > 0x09) { r.a -= 0x06; }
    r.p.z = r.a == 0;
    r.p.n = (r.a & 0x80) != 0;
}

// XCN — swap nibbles of A
Spc700::Routine Spc700::InstrExchangeNibble() {
    co_await WaitCycle();
    co_await WaitCycle();
    co_await WaitCycle();
    co_await WaitCycle();
    r.a = static_cast<uint8_t>((r.a >> 4) | (r.a << 4));
    r.p.z = r.a == 0;
    r.p.n = (r.a & 0x80) != 0;
}

Spc700::Routine Spc700::InstrNoOperation() {
    co_await WaitCycle();
}

Spc700::Routine Spc700::InstrSleep() {
    co_await WaitCycle();
    co_await WaitCycle();
    r.wait = true;
}

Spc700::Routine Spc700::InstrStop() {
    co_await WaitCycle();
    co_await WaitCycle();
    r.stop = true;
}

// Step — cycle-resumable opcode dispatch (all 256 opcodes)

bool Spc700::InstructionInProgress() const noexcept {
    return static_cast<bool>(executor_) && !atInstructionBoundary_;
}

void Spc700::BeginInstruction() {
    if (!executor_) {
        executor_ = ExecuteInstructions();
        activeCoroutine_ = executor_.GetHandle();
        atInstructionBoundary_ = true;
    }
    if (!atInstructionBoundary_) return;

    atInstructionBoundary_ = false;
    if (activeCoroutine_ && !activeCoroutine_.done()) activeCoroutine_.resume();
    CheckExecutor();
    ResumeToCycleBoundary();
}

void Spc700::CheckExecutor() {
    if (!executor_ || !executor_.Done()) return;
    executor_.RethrowIfFailed();
    executor_.Reset();
    activeCoroutine_ = {};
    pendingCycle_ = {};
    atInstructionBoundary_ = true;
}

void Spc700::ResumeToCycleBoundary() {
    while (executor_ && !atInstructionBoundary_ &&
           pendingCycle_.kind == PendingCycleKind::None) {
        if (executor_.Done()) {
            CheckExecutor();
            return;
        }
        if (!activeCoroutine_) activeCoroutine_ = executor_.GetHandle();
        activeCoroutine_.resume();
        CheckExecutor();
    }
}

void Spc700::StepCycle() {
    BeginInstruction();
    if (!executor_ || pendingCycle_.kind == PendingCycleKind::None) return;

    const PendingCycle cycle = pendingCycle_;
    pendingCycle_ = {};

    switch (cycle.kind) {
    case PendingCycleKind::Idle:
        Idle();
        break;
    case PendingCycleKind::Read:
        *cycle.readResult = Read(cycle.address);
        break;
    case PendingCycleKind::Write:
        Write(cycle.address, cycle.data);
        break;
    case PendingCycleKind::None:
        return;
    }

    if (activeCoroutine_ && !activeCoroutine_.done()) activeCoroutine_.resume();
    CheckExecutor();
    ResumeToCycleBoundary();
}

void Spc700::Step() {
    do {
        StepCycle();
    } while (InstructionInProgress());
}

Spc700::Routine Spc700::ExecuteInstructions() {
    for (;;) {
        if (r.wait || r.stop) {
            co_await WaitCycle();
            co_await InstructionBoundary{this};
            continue;
        }

        uint8_t opcode = co_await Fetch();

        switch (opcode) {
    // 0x0X
    case 0x00: co_await InstrNoOperation(); break;
    case 0x01: co_await InstrCallTable(0); break;
    case 0x02: co_await InstrAbsoluteBitSet(0, true); break;
    case 0x03: co_await InstrBranchBit(0, true); break;
    case 0x04: co_await InstrDirectRead(&Spc700::AlgOR, r.a); break;
    case 0x05: co_await InstrAbsoluteRead(&Spc700::AlgOR, r.a); break;
    case 0x06: co_await InstrIndirectXRead(&Spc700::AlgOR); break;
    case 0x07: co_await InstrIndexedIndirectRead(&Spc700::AlgOR, r.x); break;
    case 0x08: co_await InstrImmediateRead(&Spc700::AlgOR, r.a); break;
    case 0x09: co_await InstrDirectDirectModify(&Spc700::AlgOR); break;
    case 0x0A: co_await InstrAbsoluteBitModify(0); break;  // OR1 C,mem.bit
    case 0x0B: co_await InstrDirectModify(&Spc700::AlgASL); break;
    case 0x0C: co_await InstrAbsoluteModify(&Spc700::AlgASL); break;
    case 0x0D: co_await InstrPushP(); break;
    case 0x0E: co_await InstrTestSetBitsAbsolute(true); break;   // TSET1
    case 0x0F: co_await InstrBreak(); break;

    // 0x1X
    case 0x10: co_await InstrBranch(!r.p.n); break;  // BPL
    case 0x11: co_await InstrCallTable(1); break;
    case 0x12: co_await InstrAbsoluteBitSet(0, false); break;  // CLR1 dp.0
    case 0x13: co_await InstrBranchBit(0, false); break;        // BBC dp.0
    case 0x14: co_await InstrDirectIndexedRead(&Spc700::AlgOR, r.a, r.x); break;
    case 0x15: co_await InstrAbsoluteIndexedRead(&Spc700::AlgOR, r.x); break;
    case 0x16: co_await InstrAbsoluteIndexedRead(&Spc700::AlgOR, r.y); break;
    case 0x17: co_await InstrIndirectIndexedRead(&Spc700::AlgOR, r.y); break;
    case 0x18: co_await InstrDirectImmediateModify(&Spc700::AlgOR); break;
    case 0x19: co_await InstrIndirectXWriteIndirectY(&Spc700::AlgOR); break;
    case 0x1A: co_await InstrDirectModifyWord(-1); break;  // DECW
    case 0x1B: co_await InstrDirectIndexedModify(&Spc700::AlgASL, r.x); break;
    case 0x1C: co_await InstrImpliedModify(&Spc700::AlgASL, r.a); break;
    case 0x1D: co_await InstrImpliedModify(&Spc700::AlgDEC, r.x); break;
    case 0x1E: co_await InstrAbsoluteRead(&Spc700::AlgCMP, r.x); break;  // CMP X,abs
    case 0x1F: co_await InstrJumpIndirectX(); break;

    // 0x2X
    case 0x20: co_await InstrFlagSet(r.p.p, false); break;  // CLRP
    case 0x21: co_await InstrCallTable(2); break;
    case 0x22: co_await InstrAbsoluteBitSet(1, true); break;
    case 0x23: co_await InstrBranchBit(1, true); break;
    case 0x24: co_await InstrDirectRead(&Spc700::AlgAND, r.a); break;
    case 0x25: co_await InstrAbsoluteRead(&Spc700::AlgAND, r.a); break;
    case 0x26: co_await InstrIndirectXRead(&Spc700::AlgAND); break;
    case 0x27: co_await InstrIndexedIndirectRead(&Spc700::AlgAND, r.x); break;
    case 0x28: co_await InstrImmediateRead(&Spc700::AlgAND, r.a); break;
    case 0x29: co_await InstrDirectDirectModify(&Spc700::AlgAND); break;
    case 0x2A: co_await InstrAbsoluteBitModify(1); break;  // OR1 C,!mem.bit
    case 0x2B: co_await InstrDirectModify(&Spc700::AlgROL); break;
    case 0x2C: co_await InstrAbsoluteModify(&Spc700::AlgROL); break;
    case 0x2D: co_await InstrPush(r.a); break;
    case 0x2E: co_await InstrBranchNotDirect(); break;  // CBNE dp
    case 0x2F: co_await InstrBranch(true); break;       // BRA

    // 0x3X
    case 0x30: co_await InstrBranch(r.p.n); break;  // BMI
    case 0x31: co_await InstrCallTable(3); break;
    case 0x32: co_await InstrAbsoluteBitSet(1, false); break;
    case 0x33: co_await InstrBranchBit(1, false); break;
    case 0x34: co_await InstrDirectIndexedRead(&Spc700::AlgAND, r.a, r.x); break;
    case 0x35: co_await InstrAbsoluteIndexedRead(&Spc700::AlgAND, r.x); break;
    case 0x36: co_await InstrAbsoluteIndexedRead(&Spc700::AlgAND, r.y); break;
    case 0x37: co_await InstrIndirectIndexedRead(&Spc700::AlgAND, r.y); break;
    case 0x38: co_await InstrDirectImmediateModify(&Spc700::AlgAND); break;
    case 0x39: co_await InstrIndirectXWriteIndirectY(&Spc700::AlgAND); break;
    case 0x3A: co_await InstrDirectModifyWord(+1); break;  // INCW
    case 0x3B: co_await InstrDirectIndexedModify(&Spc700::AlgROL, r.x); break;
    case 0x3C: co_await InstrImpliedModify(&Spc700::AlgROL, r.a); break;
    case 0x3D: co_await InstrImpliedModify(&Spc700::AlgINC, r.x); break;
    case 0x3E: co_await InstrDirectRead(&Spc700::AlgCMP, r.x); break;  // CMP X,dp
    case 0x3F: co_await InstrCallAbsolute(); break;

    // 0x4X
    case 0x40: co_await InstrFlagSet(r.p.p, true); break;  // SETP
    case 0x41: co_await InstrCallTable(4); break;
    case 0x42: co_await InstrAbsoluteBitSet(2, true); break;
    case 0x43: co_await InstrBranchBit(2, true); break;
    case 0x44: co_await InstrDirectRead(&Spc700::AlgEOR, r.a); break;
    case 0x45: co_await InstrAbsoluteRead(&Spc700::AlgEOR, r.a); break;
    case 0x46: co_await InstrIndirectXRead(&Spc700::AlgEOR); break;
    case 0x47: co_await InstrIndexedIndirectRead(&Spc700::AlgEOR, r.x); break;
    case 0x48: co_await InstrImmediateRead(&Spc700::AlgEOR, r.a); break;
    case 0x49: co_await InstrDirectDirectModify(&Spc700::AlgEOR); break;
    case 0x4A: co_await InstrAbsoluteBitModify(2); break;  // AND1 C,mem.bit
    case 0x4B: co_await InstrDirectModify(&Spc700::AlgLSR); break;
    case 0x4C: co_await InstrAbsoluteModify(&Spc700::AlgLSR); break;
    case 0x4D: co_await InstrPush(r.x); break;
    case 0x4E: co_await InstrTestSetBitsAbsolute(false); break;  // TCLR1
    case 0x4F: co_await InstrCallPage(); break;  // PCALL

    // 0x5X
    case 0x50: co_await InstrBranch(!r.p.v); break;  // BVC
    case 0x51: co_await InstrCallTable(5); break;
    case 0x52: co_await InstrAbsoluteBitSet(2, false); break;
    case 0x53: co_await InstrBranchBit(2, false); break;
    case 0x54: co_await InstrDirectIndexedRead(&Spc700::AlgEOR, r.a, r.x); break;
    case 0x55: co_await InstrAbsoluteIndexedRead(&Spc700::AlgEOR, r.x); break;
    case 0x56: co_await InstrAbsoluteIndexedRead(&Spc700::AlgEOR, r.y); break;
    case 0x57: co_await InstrIndirectIndexedRead(&Spc700::AlgEOR, r.y); break;
    case 0x58: co_await InstrDirectImmediateModify(&Spc700::AlgEOR); break;
    case 0x59: co_await InstrIndirectXWriteIndirectY(&Spc700::AlgEOR); break;
    case 0x5A: co_await InstrDirectCompareWord(&Spc700::AlgCPW); break;  // CMPW
    case 0x5B: co_await InstrDirectIndexedModify(&Spc700::AlgLSR, r.x); break;
    case 0x5C: co_await InstrImpliedModify(&Spc700::AlgLSR, r.a); break;
    case 0x5D: co_await InstrTransfer(r.a, r.x); break;  // MOV X,A
    case 0x5E: co_await InstrAbsoluteRead(&Spc700::AlgCMP, r.y); break;  // CMP Y,abs
    case 0x5F: co_await InstrJumpAbsolute(); break;

    // 0x6X
    case 0x60: co_await InstrFlagSet(r.p.c, false); break;  // CLRC
    case 0x61: co_await InstrCallTable(6); break;
    case 0x62: co_await InstrAbsoluteBitSet(3, true); break;
    case 0x63: co_await InstrBranchBit(3, true); break;
    case 0x64: co_await InstrDirectRead(&Spc700::AlgCMP, r.a); break;
    case 0x65: co_await InstrAbsoluteRead(&Spc700::AlgCMP, r.a); break;
    case 0x66: co_await InstrIndirectXRead(&Spc700::AlgCMP); break;
    case 0x67: co_await InstrIndexedIndirectRead(&Spc700::AlgCMP, r.x); break;
    case 0x68: co_await InstrImmediateRead(&Spc700::AlgCMP, r.a); break;
    case 0x69: co_await InstrDirectDirectCompare(&Spc700::AlgCMP); break;
    case 0x6A: co_await InstrAbsoluteBitModify(3); break;  // AND1 C,!mem.bit
    case 0x6B: co_await InstrDirectModify(&Spc700::AlgROR); break;
    case 0x6C: co_await InstrAbsoluteModify(&Spc700::AlgROR); break;
    case 0x6D: co_await InstrPush(r.y); break;
    case 0x6E: co_await InstrBranchNotDirectDecrement(); break;  // DBNZ dp
    case 0x6F: co_await InstrReturnSubroutine(); break;

    // 0x7X
    case 0x70: co_await InstrBranch(r.p.v); break;  // BVS
    case 0x71: co_await InstrCallTable(7); break;
    case 0x72: co_await InstrAbsoluteBitSet(3, false); break;
    case 0x73: co_await InstrBranchBit(3, false); break;
    case 0x74: co_await InstrDirectIndexedRead(&Spc700::AlgCMP, r.a, r.x); break;
    case 0x75: co_await InstrAbsoluteIndexedRead(&Spc700::AlgCMP, r.x); break;
    case 0x76: co_await InstrAbsoluteIndexedRead(&Spc700::AlgCMP, r.y); break;
    case 0x77: co_await InstrIndirectIndexedRead(&Spc700::AlgCMP, r.y); break;
    case 0x78: co_await InstrDirectImmediateCompare(&Spc700::AlgCMP); break;
    case 0x79: co_await InstrIndirectXCompareIndirectY(&Spc700::AlgCMP); break;
    case 0x7A: co_await InstrDirectReadWord(&Spc700::AlgADW); break;  // ADDW
    case 0x7B: co_await InstrDirectIndexedModify(&Spc700::AlgROR, r.x); break;
    case 0x7C: co_await InstrImpliedModify(&Spc700::AlgROR, r.a); break;
    case 0x7D: co_await InstrTransfer(r.x, r.a); break;  // MOV A,X
    case 0x7E: co_await InstrDirectRead(&Spc700::AlgCMP, r.y); break;  // CMP Y,dp
    case 0x7F: co_await InstrReturnInterrupt(); break;

    // 0x8X
    case 0x80: co_await InstrFlagSet(r.p.c, true); break;  // SETC
    case 0x81: co_await InstrCallTable(8); break;
    case 0x82: co_await InstrAbsoluteBitSet(4, true); break;
    case 0x83: co_await InstrBranchBit(4, true); break;
    case 0x84: co_await InstrDirectRead(&Spc700::AlgADC, r.a); break;
    case 0x85: co_await InstrAbsoluteRead(&Spc700::AlgADC, r.a); break;
    case 0x86: co_await InstrIndirectXRead(&Spc700::AlgADC); break;
    case 0x87: co_await InstrIndexedIndirectRead(&Spc700::AlgADC, r.x); break;
    case 0x88: co_await InstrImmediateRead(&Spc700::AlgADC, r.a); break;
    case 0x89: co_await InstrDirectDirectModify(&Spc700::AlgADC); break;
    case 0x8A: co_await InstrAbsoluteBitModify(4); break;  // EOR1
    case 0x8B: co_await InstrDirectModify(&Spc700::AlgDEC); break;
    case 0x8C: co_await InstrAbsoluteModify(&Spc700::AlgDEC); break;
    case 0x8D: co_await InstrImmediateRead(&Spc700::AlgLD, r.y); break;  // MOV Y,#imm
    case 0x8E: co_await InstrPullP(); break;  // POP PSW
    case 0x8F: co_await InstrDirectImmediateWrite(); break;  // MOV dp,#imm

    // 0x9X
    case 0x90: co_await InstrBranch(!r.p.c); break;  // BCC
    case 0x91: co_await InstrCallTable(9); break;
    case 0x92: co_await InstrAbsoluteBitSet(4, false); break;
    case 0x93: co_await InstrBranchBit(4, false); break;
    case 0x94: co_await InstrDirectIndexedRead(&Spc700::AlgADC, r.a, r.x); break;
    case 0x95: co_await InstrAbsoluteIndexedRead(&Spc700::AlgADC, r.x); break;
    case 0x96: co_await InstrAbsoluteIndexedRead(&Spc700::AlgADC, r.y); break;
    case 0x97: co_await InstrIndirectIndexedRead(&Spc700::AlgADC, r.y); break;
    case 0x98: co_await InstrDirectImmediateModify(&Spc700::AlgADC); break;
    case 0x99: co_await InstrIndirectXWriteIndirectY(&Spc700::AlgADC); break;
    case 0x9A: co_await InstrDirectReadWord(&Spc700::AlgSBW); break;  // SUBW
    case 0x9B: co_await InstrDirectIndexedModify(&Spc700::AlgDEC, r.x); break;
    case 0x9C: co_await InstrImpliedModify(&Spc700::AlgDEC, r.a); break;
    case 0x9D: co_await InstrTransfer(r.s, r.x); break;  // MOV X,SP (TSX)
    case 0x9E: co_await InstrDivide(); break;
    case 0x9F: co_await InstrExchangeNibble(); break;

    // 0xAX
    case 0xA0: co_await InstrFlagSet(r.p.i, true); break;  // EI
    case 0xA1: co_await InstrCallTable(10); break;
    case 0xA2: co_await InstrAbsoluteBitSet(5, true); break;
    case 0xA3: co_await InstrBranchBit(5, true); break;
    case 0xA4: co_await InstrDirectRead(&Spc700::AlgSBC, r.a); break;
    case 0xA5: co_await InstrAbsoluteRead(&Spc700::AlgSBC, r.a); break;
    case 0xA6: co_await InstrIndirectXRead(&Spc700::AlgSBC); break;
    case 0xA7: co_await InstrIndexedIndirectRead(&Spc700::AlgSBC, r.x); break;
    case 0xA8: co_await InstrImmediateRead(&Spc700::AlgSBC, r.a); break;
    case 0xA9: co_await InstrDirectDirectModify(&Spc700::AlgSBC); break;
    case 0xAA: co_await InstrAbsoluteBitModify(5); break;  // MOV1 C,mem.bit
    case 0xAB: co_await InstrDirectModify(&Spc700::AlgINC); break;
    case 0xAC: co_await InstrAbsoluteModify(&Spc700::AlgINC); break;
    case 0xAD: co_await InstrImmediateRead(&Spc700::AlgCMP, r.y); break;  // CMP Y,#imm
    case 0xAE: co_await InstrPull(r.a); break;  // POP A
    case 0xAF: co_await InstrIndirectXIncrementWrite(r.a); break;  // MOV (X++),A

    // 0xBX
    case 0xB0: co_await InstrBranch(r.p.c); break;  // BCS
    case 0xB1: co_await InstrCallTable(11); break;
    case 0xB2: co_await InstrAbsoluteBitSet(5, false); break;
    case 0xB3: co_await InstrBranchBit(5, false); break;
    case 0xB4: co_await InstrDirectIndexedRead(&Spc700::AlgSBC, r.a, r.x); break;
    case 0xB5: co_await InstrAbsoluteIndexedRead(&Spc700::AlgSBC, r.x); break;
    case 0xB6: co_await InstrAbsoluteIndexedRead(&Spc700::AlgSBC, r.y); break;
    case 0xB7: co_await InstrIndirectIndexedRead(&Spc700::AlgSBC, r.y); break;
    case 0xB8: co_await InstrDirectImmediateModify(&Spc700::AlgSBC); break;
    case 0xB9: co_await InstrIndirectXWriteIndirectY(&Spc700::AlgSBC); break;
    case 0xBA: co_await InstrDirectReadWord(&Spc700::AlgLDW); break;  // MOVW YA,dp
    case 0xBB: co_await InstrDirectIndexedModify(&Spc700::AlgINC, r.x); break;
    case 0xBC: co_await InstrImpliedModify(&Spc700::AlgINC, r.a); break;
    case 0xBD: co_await InstrTransfer(r.x, r.s); break;  // MOV SP,X (TXS)
    case 0xBE: co_await InstrDecimalAdjustSub(); break;  // DAS
    case 0xBF: co_await InstrIndirectXIncrementRead(r.a); break;  // MOV A,(X++)

    // 0xCX
    case 0xC0: co_await InstrFlagSet(r.p.i, false); break;  // DI
    case 0xC1: co_await InstrCallTable(12); break;
    case 0xC2: co_await InstrAbsoluteBitSet(6, true); break;
    case 0xC3: co_await InstrBranchBit(6, true); break;
    case 0xC4: co_await InstrDirectWrite(r.a); break;  // MOV dp,A
    case 0xC5: co_await InstrAbsoluteWrite(r.a); break;  // MOV abs,A
    case 0xC6: co_await InstrIndirectXWrite(r.a); break;  // MOV (X),A
    case 0xC7: co_await InstrIndexedIndirectWrite(r.a, r.x); break;
    case 0xC8: co_await InstrImmediateRead(&Spc700::AlgCMP, r.x); break;  // CMP X,#imm
    case 0xC9: co_await InstrAbsoluteWrite(r.x); break;  // MOV abs,X
    case 0xCA: co_await InstrAbsoluteBitModify(6); break;  // MOV1 mem.bit,C
    case 0xCB: co_await InstrDirectWrite(r.y); break;  // MOV dp,Y
    case 0xCC: co_await InstrAbsoluteWrite(r.y); break;  // MOV abs,Y
    case 0xCD: co_await InstrImmediateRead(&Spc700::AlgLD, r.x); break;  // MOV X,#imm
    case 0xCE: co_await InstrPull(r.x); break;  // POP X
    case 0xCF: co_await InstrMultiply(); break;

    // 0xDX
    case 0xD0: co_await InstrBranch(!r.p.z); break;  // BNE
    case 0xD1: co_await InstrCallTable(13); break;
    case 0xD2: co_await InstrAbsoluteBitSet(6, false); break;
    case 0xD3: co_await InstrBranchBit(6, false); break;
    case 0xD4: co_await InstrDirectIndexedWrite(r.a, r.x); break;
    case 0xD5: co_await InstrAbsoluteIndexedWrite(r.x); break;  // MOV abs+X,A
    case 0xD6: co_await InstrAbsoluteIndexedWrite(r.y); break;  // MOV abs+Y,A
    case 0xD7: co_await InstrIndirectIndexedWrite(r.a, r.y); break;
    case 0xD8: co_await InstrDirectWrite(r.x); break;  // MOV dp,X
    case 0xD9: co_await InstrDirectIndexedWrite(r.x, r.y); break;  // MOV dp+Y,X
    case 0xDA: co_await InstrDirectWriteWord(); break;  // MOVW dp,YA
    case 0xDB: co_await InstrDirectIndexedWrite(r.y, r.x); break;  // MOV dp+X,Y
    case 0xDC: co_await InstrImpliedModify(&Spc700::AlgDEC, r.y); break;
    case 0xDD: co_await InstrTransfer(r.y, r.a); break;  // MOV A,Y
    case 0xDE: co_await InstrBranchNotDirectIndexed(r.x); break;  // CBNE dp+X
    case 0xDF: co_await InstrDecimalAdjustAdd(); break;  // DAA

    // 0xEX
    case 0xE0: co_await InstrOverflowClear(); break;  // CLRV
    case 0xE1: co_await InstrCallTable(14); break;
    case 0xE2: co_await InstrAbsoluteBitSet(7, true); break;
    case 0xE3: co_await InstrBranchBit(7, true); break;
    case 0xE4: co_await InstrDirectRead(&Spc700::AlgLD, r.a); break;  // MOV A,dp
    case 0xE5: co_await InstrAbsoluteRead(&Spc700::AlgLD, r.a); break;  // MOV A,abs
    case 0xE6: { // MOV A,(X)
        co_await WaitCycle();
        r.a = AlgLD(r.a, co_await Load(r.x));
        break;
    }
    case 0xE7: co_await InstrIndexedIndirectRead(&Spc700::AlgLD, r.x); break;  // MOV A,[dp+X]
    case 0xE8: co_await InstrImmediateRead(&Spc700::AlgLD, r.a); break;  // MOV A,#imm
    case 0xE9: co_await InstrAbsoluteRead(&Spc700::AlgLD, r.x); break;  // MOV X,abs
    case 0xEA: co_await InstrAbsoluteBitModify(7); break;  // NOT1
    case 0xEB: co_await InstrDirectRead(&Spc700::AlgLD, r.y); break;  // MOV Y,dp
    case 0xEC: co_await InstrAbsoluteRead(&Spc700::AlgLD, r.y); break;  // MOV Y,abs
    case 0xED: co_await InstrComplementCarry(); break;  // CMC
    case 0xEE: co_await InstrPull(r.y); break;  // POP Y
    case 0xEF: co_await InstrSleep(); break;

    // 0xFX
    case 0xF0: co_await InstrBranch(r.p.z); break;  // BEQ
    case 0xF1: co_await InstrCallTable(15); break;
    case 0xF2: co_await InstrAbsoluteBitSet(7, false); break;
    case 0xF3: co_await InstrBranchBit(7, false); break;
    case 0xF4: co_await InstrDirectIndexedRead(&Spc700::AlgLD, r.a, r.x); break;  // MOV A,dp+X
    case 0xF5: co_await InstrAbsoluteIndexedRead(&Spc700::AlgLD, r.x); break;  // MOV A,abs+X
    case 0xF6: co_await InstrAbsoluteIndexedRead(&Spc700::AlgLD, r.y); break;  // MOV A,abs+Y
    case 0xF7: co_await InstrIndirectIndexedRead(&Spc700::AlgLD, r.y); break;  // MOV A,[dp]+Y
    case 0xF8: co_await InstrDirectRead(&Spc700::AlgLD, r.x); break;  // MOV X,dp
    case 0xF9: co_await InstrDirectIndexedRead(&Spc700::AlgLD, r.x, r.y); break;  // MOV X,dp+Y
    case 0xFA: co_await InstrDirectDirectWrite(); break;  // MOV dp,dp
    case 0xFB: co_await InstrDirectIndexedRead(&Spc700::AlgLD, r.y, r.x); break;  // MOV Y,dp+X
    case 0xFC: co_await InstrImpliedModify(&Spc700::AlgINC, r.y); break;
    case 0xFD: co_await InstrTransfer(r.a, r.y); break;  // MOV Y,A
    case 0xFE: co_await InstrBranchNotYDecrement(); break;  // DBNZ Y
    case 0xFF: co_await InstrStop(); break;
        }

        co_await InstructionBoundary{this};
    }
}

} // namespace snes::core
