#pragma once
//
// Processor65816 — Pure WDC 65C816 CPU core
//
// This is a platform-independent 65816 instruction-set implementation.
// It knows nothing about the SNES; subclasses (SnesCpu, and later SA-1)
// implement the virtual bus interface with system-specific timing.
//
// Design mirrors bsnes processor/wdc65816/wdc65816.hpp:
//   * Virtual read()/write()/idle() for bus access
//   * Virtual lastCycle() for interrupt edge detection
//   * Register set with MDR, MAR, temporaries
//   * ALU algorithms as member functions (8-bit and 16-bit variants)
//   * Instruction dispatch via 256-entry opcode table
//

#include <cstdint>
#include <functional>

namespace snes::core {

// Processor65816  — reusable 65C816 CPU core
class Processor65816 {
public:
    // Register file
    struct Registers {
        uint16_t a  = 0;       // Accumulator (C)
        uint16_t x  = 0;       // Index X
        uint16_t y  = 0;       // Index Y
        uint16_t s  = 0x01FF;  // Stack pointer
        uint16_t d  = 0;       // Direct-page register
        uint16_t pc = 0;       // Program counter (16-bit offset)
        uint8_t  db = 0;       // Data bank register
        uint8_t  pb = 0;       // Program bank register
        uint8_t  p  = 0x34;    // Processor status (NV-MX-DIZC packed)
        bool     e  = true;    // Emulation mode flag

        // Extra state
        bool     irq = false;  // IRQ pin level (active-low; true = asserted)
        bool     wai = false;  // Waiting for interrupt (WAI executed)
        bool     stp = false;  // Stopped (STP executed)

        uint16_t vector = 0xFFFC; // Current interrupt vector address

        // Memory Address Register / Memory Data Register
        uint32_t mar = 0;      // 24-bit address latch
        uint8_t  mdr = 0;      // Open-bus data latch

        // Temporaries used during multi-cycle instructions
        uint32_t u = 0;
        uint32_t v = 0;
        uint32_t w = 0;
    };

    // Status flag constants (bit positions in p register)
    static constexpr uint8_t FlagC = 0x01;
    static constexpr uint8_t FlagZ = 0x02;
    static constexpr uint8_t FlagI = 0x04;
    static constexpr uint8_t FlagD = 0x08;
    static constexpr uint8_t FlagX = 0x10;
    static constexpr uint8_t FlagM = 0x20;
    static constexpr uint8_t FlagV = 0x40;
    static constexpr uint8_t FlagN = 0x80;

    // Interrupt types
    enum class Interrupt {
        Reset,
        Nmi,
        Irq,
        Brk,
        Cop,
        Abort
    };

    // Construction / destruction
    Processor65816() = default;
    virtual ~Processor65816() = default;

    // Non-copyable, movable
    Processor65816(const Processor65816&) = delete;
    Processor65816& operator=(const Processor65816&) = delete;
    Processor65816(Processor65816&&) = default;
    Processor65816& operator=(Processor65816&&) = default;

    // Virtual bus interface — subclasses implement these
    virtual void idle() = 0;                                   // Internal operation cycle
    virtual uint8_t read(uint32_t address) = 0;                // Bus read
    virtual void write(uint32_t address, uint8_t data) = 0;    // Bus write
    virtual void lastCycle() = 0;                              // Called on final cycle of instruction (IRQ/NMI edge)
    virtual bool interruptPending() const = 0;                 // Is NMI or IRQ about to fire?

    // Power / reset
    void power();

    // Instruction execution — runs one full instruction
    void instruction();

    // Register accessors
    const Registers& regs() const noexcept { return r; }
    Registers& regs() noexcept { return r; }

    // Flag helpers (inline, used by CPU core and subclasses)
    bool flagC() const { return (r.p & FlagC) != 0; }
    bool flagZ() const { return (r.p & FlagZ) != 0; }
    bool flagI() const { return (r.p & FlagI) != 0; }
    bool flagD() const { return (r.p & FlagD) != 0; }
    bool flagX() const { return (r.p & FlagX) != 0; }
    bool flagM() const { return (r.p & FlagM) != 0; }
    bool flagV() const { return (r.p & FlagV) != 0; }
    bool flagN() const { return (r.p & FlagN) != 0; }

    bool mf() const { return r.e || flagM(); }  // 8-bit accumulator?
    bool xf() const { return r.e || flagX(); }  // 8-bit index?

protected:
    // Register file
    Registers r{};

    // Flag manipulation
    void setFlag(uint8_t flag, bool set);
    void setNZ8(uint8_t value);
    void setNZ16(uint16_t value);
    void setE(bool enabled);

    // Memory access helpers (following bsnes memory.cpp)
    uint8_t fetch();                                     // [PB:PC++]
    void idleIRQ();                                      // idle, but reads bus if IRQ pending
    void idle2();                                        // extra cycle when D.l != 0
    void idle4(uint16_t x, uint16_t y);                  // extra cycle on page cross (X=1 or page differs)
    void idle6(uint16_t address);                        // extra cycle on E-mode page cross

    uint8_t pull();                                      // stack pull (E-mode aware)
    void    push(uint8_t data);                          // stack push (E-mode aware)
    uint8_t pullN();                                     // stack pull (native mode, no E wrapping)
    void    pushN(uint8_t data);                         // stack push (native mode, no E wrapping)

    uint8_t readDirect(uint32_t address);                // direct page read (E-mode page wrap)
    void    writeDirect(uint32_t address, uint8_t data); // direct page write
    uint8_t readDirectX(uint32_t address, uint32_t offset); // (direct,X) with E-mode wrapping bug
    uint8_t readDirectN(uint32_t address);               // direct page read (no E-mode wrap)
    uint8_t readBank(uint32_t address);                  // [DB:addr]
    void    writeBank(uint32_t address, uint8_t data);
    uint8_t readLong(uint32_t address);                  // [24-bit addr]
    void    writeLong(uint32_t address, uint8_t data);
    uint8_t readStack(uint32_t address);                 // [S+offset]
    void    writeStack(uint32_t address, uint8_t data);

    // ALU algorithms — 8-bit and 16-bit variants
    using Alu8  = uint8_t  (Processor65816::*)(uint8_t);
    using Alu16 = uint16_t (Processor65816::*)(uint16_t);

    uint8_t  algorithmADC8(uint8_t data);
    uint16_t algorithmADC16(uint16_t data);
    uint8_t  algorithmAND8(uint8_t data);
    uint16_t algorithmAND16(uint16_t data);
    uint8_t  algorithmASL8(uint8_t data);
    uint16_t algorithmASL16(uint16_t data);
    uint8_t  algorithmBIT8(uint8_t data);
    uint16_t algorithmBIT16(uint16_t data);
    uint8_t  algorithmCMP8(uint8_t data);
    uint16_t algorithmCMP16(uint16_t data);
    uint8_t  algorithmCPX8(uint8_t data);
    uint16_t algorithmCPX16(uint16_t data);
    uint8_t  algorithmCPY8(uint8_t data);
    uint16_t algorithmCPY16(uint16_t data);
    uint8_t  algorithmDEC8(uint8_t data);
    uint16_t algorithmDEC16(uint16_t data);
    uint8_t  algorithmEOR8(uint8_t data);
    uint16_t algorithmEOR16(uint16_t data);
    uint8_t  algorithmINC8(uint8_t data);
    uint16_t algorithmINC16(uint16_t data);
    uint8_t  algorithmLDA8(uint8_t data);
    uint16_t algorithmLDA16(uint16_t data);
    uint8_t  algorithmLDX8(uint8_t data);
    uint16_t algorithmLDX16(uint16_t data);
    uint8_t  algorithmLDY8(uint8_t data);
    uint16_t algorithmLDY16(uint16_t data);
    uint8_t  algorithmLSR8(uint8_t data);
    uint16_t algorithmLSR16(uint16_t data);
    uint8_t  algorithmORA8(uint8_t data);
    uint16_t algorithmORA16(uint16_t data);
    uint8_t  algorithmROL8(uint8_t data);
    uint16_t algorithmROL16(uint16_t data);
    uint8_t  algorithmROR8(uint8_t data);
    uint16_t algorithmROR16(uint16_t data);
    uint8_t  algorithmSBC8(uint8_t data);
    uint16_t algorithmSBC16(uint16_t data);
    uint8_t  algorithmTRB8(uint8_t data);
    uint16_t algorithmTRB16(uint16_t data);
    uint8_t  algorithmTSB8(uint8_t data);
    uint16_t algorithmTSB16(uint16_t data);

    // Instruction implementations — read operations
    void instructionImmediateRead8(Alu8 op);
    void instructionImmediateRead16(Alu16 op);
    void instructionBankRead8(Alu8 op);
    void instructionBankRead16(Alu16 op);
    void instructionBankRead8(Alu8 op, uint16_t index);
    void instructionBankRead16(Alu16 op, uint16_t index);
    void instructionLongRead8(Alu8 op, uint16_t index = 0);
    void instructionLongRead16(Alu16 op, uint16_t index = 0);
    void instructionDirectRead8(Alu8 op);
    void instructionDirectRead16(Alu16 op);
    void instructionDirectRead8(Alu8 op, uint16_t index);
    void instructionDirectRead16(Alu16 op, uint16_t index);
    void instructionIndirectRead8(Alu8 op);
    void instructionIndirectRead16(Alu16 op);
    void instructionIndexedIndirectRead8(Alu8 op);
    void instructionIndexedIndirectRead16(Alu16 op);
    void instructionIndirectIndexedRead8(Alu8 op);
    void instructionIndirectIndexedRead16(Alu16 op);
    void instructionIndirectLongRead8(Alu8 op, uint16_t index = 0);
    void instructionIndirectLongRead16(Alu16 op, uint16_t index = 0);
    void instructionStackRead8(Alu8 op);
    void instructionStackRead16(Alu16 op);
    void instructionIndirectStackRead8(Alu8 op);
    void instructionIndirectStackRead16(Alu16 op);

    // Instruction implementations — write operations
    void instructionBankWrite8(uint16_t reg);
    void instructionBankWrite16(uint16_t reg);
    void instructionBankWrite8(uint16_t reg, uint16_t index);
    void instructionBankWrite16(uint16_t reg, uint16_t index);
    void instructionLongWrite8(uint16_t index = 0);
    void instructionLongWrite16(uint16_t index = 0);
    void instructionDirectWrite8(uint16_t reg);
    void instructionDirectWrite16(uint16_t reg);
    void instructionDirectWrite8(uint16_t reg, uint16_t index);
    void instructionDirectWrite16(uint16_t reg, uint16_t index);
    void instructionIndirectWrite8();
    void instructionIndirectWrite16();
    void instructionIndexedIndirectWrite8();
    void instructionIndexedIndirectWrite16();
    void instructionIndirectIndexedWrite8();
    void instructionIndirectIndexedWrite16();
    void instructionIndirectLongWrite8(uint16_t index = 0);
    void instructionIndirectLongWrite16(uint16_t index = 0);
    void instructionStackWrite8();
    void instructionStackWrite16();
    void instructionIndirectStackWrite8();
    void instructionIndirectStackWrite16();

    // Instruction implementations — read-modify-write
    void instructionImpliedModify8(Alu8 op, uint16_t& reg);
    void instructionImpliedModify16(Alu16 op, uint16_t& reg);
    void instructionBankModify8(Alu8 op);
    void instructionBankModify16(Alu16 op);
    void instructionBankIndexedModify8(Alu8 op);
    void instructionBankIndexedModify16(Alu16 op);
    void instructionDirectModify8(Alu8 op);
    void instructionDirectModify16(Alu16 op);
    void instructionDirectIndexedModify8(Alu8 op);
    void instructionDirectIndexedModify16(Alu16 op);

    // Instruction implementations — program counter control
    void instructionBranch(bool take = true);
    void instructionBranchLong();
    void instructionJumpShort();
    void instructionJumpLong();
    void instructionJumpIndirect();
    void instructionJumpIndexedIndirect();
    void instructionJumpIndirectLong();
    void instructionCallShort();
    void instructionCallLong();
    void instructionCallIndexedIndirect();
    void instructionReturnInterrupt();
    void instructionReturnShort();
    void instructionReturnLong();

    // Instruction implementations — miscellaneous
    void instructionBitImmediate8();
    void instructionBitImmediate16();
    void instructionNoOperation();
    void instructionPrefix();             // WDM (2-byte NOP)
    void instructionExchangeBA();         // XBA
    void instructionBlockMove8(int adjust);   // MVN/MVP
    void instructionBlockMove16(int adjust);
    void instructionInterrupt(uint16_t vector); // BRK/COP
    void instructionStop();               // STP
    void instructionWait();               // WAI
    void instructionExchangeCE();         // XCE
    void instructionSetFlag(uint8_t flag);
    void instructionClearFlag(uint8_t flag);
    void instructionResetP();             // REP
    void instructionSetP();               // SEP
    void instructionTransfer8(uint16_t src, uint16_t& dst);
    void instructionTransfer16(uint16_t src, uint16_t& dst);
    void instructionTransferCS();         // TCS
    void instructionTransferSX8();        // TSX 8-bit
    void instructionTransferSX16();       // TSX 16-bit
    void instructionTransferXS();         // TXS
    void instructionPush8(uint16_t reg);
    void instructionPush16(uint16_t reg);
    void instructionPushD();
    void instructionPull8(uint16_t& reg);
    void instructionPull16(uint16_t& reg);
    void instructionPullD();
    void instructionPullB();
    void instructionPullP();
    void instructionPushEffectiveAddress();         // PEA
    void instructionPushEffectiveIndirectAddress();  // PEI
    void instructionPushEffectiveRelativeAddress();  // PER

    // Interrupt entry
    void enterInterrupt(Interrupt type);
    uint16_t vectorAddress(Interrupt type) const;

private:
    // Helpers for accessing register as lo/hi bytes
    static uint8_t lo(uint16_t v) { return static_cast<uint8_t>(v); }
    static uint8_t hi(uint16_t v) { return static_cast<uint8_t>(v >> 8); }
    static uint16_t makeWord(uint8_t lo, uint8_t hi) { return static_cast<uint16_t>(lo | (hi << 8)); }
};

} // namespace snes::core
