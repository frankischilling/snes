// Spc700.hpp — Sony SPC700 Audio Processor Core
//
// Pure processor implementation with virtual bus interface.
// The SNES SMP wrapper (future) will inherit this and provide the bus.
//
// Reference: bsnes processor/spc700/spc700.hpp

#pragma once

#include <array>
#include <cstdint>

namespace snes::core {

class Spc700 {
public:
    Spc700();
    virtual ~Spc700() = default;

    // Virtual bus interface — override in SMP wrapper
    virtual void    Idle()                          = 0;
    virtual uint8_t Read(uint16_t address)          = 0;
    virtual void    Write(uint16_t address, uint8_t data) = 0;

    // PSW (Processor Status Word) — bit layout:
    //   7  6  5  4  3  2  1  0
    //   N  V  P  B  H  I  Z  C
    struct Flags {
        bool c = false;  // bit 0 — Carry
        bool z = false;  // bit 1 — Zero
        bool i = false;  // bit 2 — Interrupt disable
        bool h = false;  // bit 3 — Half-carry
        bool b = false;  // bit 4 — Break
        bool p = false;  // bit 5 — Direct page select (0=$0000, 1=$0100)
        bool v = false;  // bit 6 — Overflow
        bool n = false;  // bit 7 — Negative

        // Pack flags → byte
        [[nodiscard]] uint8_t Pack() const {
            return static_cast<uint8_t>(
                (c ? 0x01 : 0) | (z ? 0x02 : 0) | (i ? 0x04 : 0) |
                (h ? 0x08 : 0) | (b ? 0x10 : 0) | (p ? 0x20 : 0) |
                (v ? 0x40 : 0) | (n ? 0x80 : 0));
        }

        // Unpack byte → flags
        void Unpack(uint8_t val) {
            c = (val & 0x01) != 0;
            z = (val & 0x02) != 0;
            i = (val & 0x04) != 0;
            h = (val & 0x08) != 0;
            b = (val & 0x10) != 0;
            p = (val & 0x20) != 0;
            v = (val & 0x40) != 0;
            n = (val & 0x80) != 0;
        }
    };

    // Registers
    struct Registers {
        uint16_t pc = 0;
        uint8_t  a  = 0;   // Accumulator  (low byte of YA)
        uint8_t  y  = 0;   // Y register   (high byte of YA)
        uint8_t  x  = 0;   // X register
        uint8_t  s  = 0;   // Stack pointer (stack is always page 1: $0100-$01FF)
        Flags    p;         // PSW flags

        bool     wait = false;  // SLEEP state
        bool     stop = false;  // STOP state

        // 16-bit YA accessor
        [[nodiscard]] uint16_t ya() const {
            return static_cast<uint16_t>(a) | (static_cast<uint16_t>(y) << 8);
        }
        void setYA(uint16_t val) {
            a = static_cast<uint8_t>(val);
            y = static_cast<uint8_t>(val >> 8);
        }
    };

    Registers r;

    // Execution
    void Power();             // Reset to power-on state
    void Step();              // Execute one instruction
    uint64_t CycleCount() const { return cycles_; }

protected:
    uint64_t cycles_ = 0;    // Total cycles consumed

    // Memory access helpers
    uint8_t Fetch();                              // read(PC++)
    uint8_t Load(uint8_t addr);                   // read(dp | addr)
    void    Store(uint8_t addr, uint8_t data);    // write(dp | addr, data)
    uint8_t Pull();                               // read(0x100 | ++S)
    void    Push(uint8_t data);                    // write(0x100 | S--, data)

    // ALU algorithms — 8-bit
    uint8_t AlgADC(uint8_t x, uint8_t y);
    uint8_t AlgSBC(uint8_t x, uint8_t y);
    uint8_t AlgAND(uint8_t x, uint8_t y);
    uint8_t AlgOR (uint8_t x, uint8_t y);
    uint8_t AlgEOR(uint8_t x, uint8_t y);
    uint8_t AlgCMP(uint8_t x, uint8_t y);   // returns x (flags only)
    uint8_t AlgASL(uint8_t x);
    uint8_t AlgLSR(uint8_t x);
    uint8_t AlgROL(uint8_t x);
    uint8_t AlgROR(uint8_t x);
    uint8_t AlgINC(uint8_t x);
    uint8_t AlgDEC(uint8_t x);
    uint8_t AlgLD (uint8_t x, uint8_t y);   // returns y, sets Z/N

    // ALU algorithms — 16-bit
    uint16_t AlgADW(uint16_t x, uint16_t y);
    uint16_t AlgSBW(uint16_t x, uint16_t y);
    uint16_t AlgCPW(uint16_t x, uint16_t y);
    uint16_t AlgLDW(uint16_t x, uint16_t y);

    // Instruction implementations

    // ALU function pointer type for parameterized instructions
    using AlgOp = uint8_t (Spc700::*)(uint8_t, uint8_t);
    using ModOp = uint8_t (Spc700::*)(uint8_t);
    using AlgOp16 = uint16_t (Spc700::*)(uint16_t, uint16_t);

    // Addressing mode instruction groups
    void InstrImmediateRead(AlgOp op, uint8_t& target);
    void InstrDirectRead(AlgOp op, uint8_t& target);
    void InstrDirectModify(ModOp op);
    void InstrDirectWrite(uint8_t data);
    void InstrDirectIndexedRead(AlgOp op, uint8_t& target, uint8_t index);
    void InstrDirectIndexedModify(ModOp op, uint8_t index);
    void InstrDirectIndexedWrite(uint8_t data, uint8_t index);
    void InstrAbsoluteRead(AlgOp op, uint8_t& target);
    void InstrAbsoluteModify(ModOp op);
    void InstrAbsoluteWrite(uint8_t data);
    void InstrAbsoluteIndexedRead(AlgOp op, uint8_t index);
    void InstrAbsoluteIndexedWrite(uint8_t index);
    void InstrIndexedIndirectRead(AlgOp op, uint8_t index);
    void InstrIndexedIndirectWrite(uint8_t data, uint8_t index);
    void InstrIndirectIndexedRead(AlgOp op, uint8_t index);
    void InstrIndirectIndexedWrite(uint8_t data, uint8_t index);
    void InstrIndirectXRead(AlgOp op);
    void InstrIndirectXWrite(uint8_t data);
    void InstrIndirectXIncrementRead(uint8_t& target);
    void InstrIndirectXIncrementWrite(uint8_t data);
    void InstrIndirectXCompareIndirectY(AlgOp op);
    void InstrIndirectXWriteIndirectY(AlgOp op);

    // Direct-Direct, Direct-Immediate
    void InstrDirectDirectCompare(AlgOp op);
    void InstrDirectDirectModify(AlgOp op);
    void InstrDirectDirectWrite();
    void InstrDirectImmediateCompare(AlgOp op);
    void InstrDirectImmediateModify(AlgOp op);
    void InstrDirectImmediateWrite();

    // 16-bit word operations
    void InstrDirectCompareWord(AlgOp16 op);
    void InstrDirectReadWord(AlgOp16 op);
    void InstrDirectModifyWord(int16_t adjust);
    void InstrDirectWriteWord();

    // Bit operations
    void InstrAbsoluteBitModify(uint8_t mode);
    void InstrAbsoluteBitSet(uint8_t bit, bool value);
    void InstrTestSetBitsAbsolute(bool set);

    // Branches
    void InstrBranch(bool take);
    void InstrBranchBit(uint8_t bit, bool match);
    void InstrBranchNotDirect();
    void InstrBranchNotDirectIndexed(uint8_t index);
    void InstrBranchNotDirectDecrement();
    void InstrBranchNotYDecrement();

    // Flow control
    void InstrCallAbsolute();
    void InstrCallPage();
    void InstrCallTable(uint8_t vector);
    void InstrJumpAbsolute();
    void InstrJumpIndirectX();
    void InstrReturnSubroutine();
    void InstrReturnInterrupt();
    void InstrBreak();

    // Register transfer
    void InstrTransfer(uint8_t from, uint8_t& to);

    // Push/Pull
    void InstrPush(uint8_t data);
    void InstrPull(uint8_t& target);
    void InstrPushP();
    void InstrPullP();

    // Flag manipulation
    void InstrFlagSet(bool& flag, bool value);
    void InstrOverflowClear();
    void InstrComplementCarry();

    // Implied register ops
    void InstrImpliedModify(ModOp op, uint8_t& target);

    // Special instructions
    void InstrMultiply();
    void InstrDivide();
    void InstrDecimalAdjustAdd();
    void InstrDecimalAdjustSub();
    void InstrExchangeNibble();
    void InstrNoOperation();
    void InstrSleep();
    void InstrStop();
};

} // namespace snes::core
