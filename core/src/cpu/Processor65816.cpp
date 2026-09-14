// snes emulator
// core/src/cpu/Processor65816.cpp
// The reusable 65816 processor core.

// Processor65816.cpp — Pure WDC 65C816 CPU core implementation
//
// Contains:
//   - power() — initialization
//   - Memory access helpers (fetch, push/pull, readDirect, readBank, etc.)
//   - ALU algorithms (ADC, SBC, AND, ORA, EOR, ASL, LSR, ROL, ROR, etc.)
//   - Instruction implementations for all addressing modes
//   - 256-entry opcode dispatch
//
// Logic follows bsnes processor/wdc65816/ as reference but is written clean-
// room in standard C++20 with no external dependencies.

#include "snes/core/Processor65816.hpp"
#include <cstring>

namespace snes::core {

//  Power / Reset

void Processor65816::power() {
    r = Registers{};
    r.a  = 0x0000;
    r.x  = 0x0000;
    r.y  = 0x0000;
    r.s  = 0x01FF;
    r.d  = 0x0000;
    r.pc = 0x0000;
    r.db = 0x00;
    r.pb = 0x00;
    r.p  = 0x34;  // M=1, X=1, I=1
    r.e  = true;

    r.irq = false;
    r.wai = false;
    r.stp = false;
    r.mar = 0;
    r.mdr = 0;
    r.vector = 0xFFFC;

    r.u = 0;
    r.v = 0;
    r.w = 0;
}

//  Flag helpers

void Processor65816::setFlag(uint8_t flag, bool set) {
    if (set) r.p |= flag;
    else     r.p &= ~flag;
    if (r.e) r.p |= (FlagM | FlagX);
}

void Processor65816::setNZ8(uint8_t value) {
    setFlag(FlagZ, value == 0);
    setFlag(FlagN, (value & 0x80) != 0);
}

void Processor65816::setNZ16(uint16_t value) {
    setFlag(FlagZ, value == 0);
    setFlag(FlagN, (value & 0x8000) != 0);
}

void Processor65816::setE(bool enabled) {
    bool oldE = r.e;
    r.e = enabled;
    if (enabled) {
        r.p |= (FlagM | FlagX);
        r.x &= 0x00FF;
        r.y &= 0x00FF;
        r.s = static_cast<uint16_t>(0x0100 | (r.s & 0xFF));
    } else if (oldE && !enabled) {
        // Transitioning from emulation to native: keep S as-is (some docs note 0x01FF)
    }
}

//  Memory access helpers  (following bsnes processor/wdc65816/memory.cpp)

uint8_t Processor65816::fetch() {
    uint32_t addr = (static_cast<uint32_t>(r.pb) << 16) | r.pc;
    r.pc++;
    return read(addr);
}

// Immediate-mode 2-cycle opcodes: if an IRQ is pending, convert the idle cycle
// to a bus read (real 65816 behaviour).
void Processor65816::idleIRQ() {
    if (interruptPending()) {
        read((static_cast<uint32_t>(r.pb) << 16) | r.pc);
    } else {
        idle();
    }
}

// Extra idle cycle when D register low byte is non-zero.
void Processor65816::idle2() {
    if (r.d & 0xFF) idle();
}

// Extra idle cycle on page crossing when index is 8-bit or pages differ.
void Processor65816::idle4(uint16_t x, uint16_t y) {
    if (!xf() || (x >> 8) != (y >> 8)) idle();
}

// Extra idle cycle on page crossing in emulation mode.
void Processor65816::idle6(uint16_t address) {
    if (r.e && (hi(r.pc) != static_cast<uint8_t>(address >> 8))) idle();
}

uint8_t Processor65816::pull() {
    if (r.e) {
        uint8_t lo_s = static_cast<uint8_t>(r.s + 1);
        r.s = static_cast<uint16_t>(0x0100 | lo_s);
    } else {
        r.s++;
    }
    return read(r.s);
}

void Processor65816::push(uint8_t data) {
    write(r.s, data);
    if (r.e) {
        uint8_t lo_s = static_cast<uint8_t>(r.s - 1);
        r.s = static_cast<uint16_t>(0x0100 | lo_s);
    } else {
        r.s--;
    }
}

uint8_t Processor65816::pullN() {
    r.s++;
    return read(r.s);
}

void Processor65816::pushN(uint8_t data) {
    write(r.s, data);
    r.s--;
}

uint8_t Processor65816::readDirect(uint32_t address) {
    if (r.e && (r.d & 0xFF) == 0) {
        return read((r.d & 0xFF00) | (address & 0xFF));
    }
    return read(static_cast<uint16_t>(r.d + address));
}

void Processor65816::writeDirect(uint32_t address, uint8_t data) {
    if (r.e && (r.d & 0xFF) == 0) {
        write((r.d & 0xFF00) | (address & 0xFF), data);
        return;
    }
    write(static_cast<uint16_t>(r.d + address), data);
}

uint8_t Processor65816::readDirectX(uint32_t address, uint32_t offset) {
    // The (direct,X) addressing mode has a bug:
    // when E=1 and D&0xFF!=0, the high byte wraps within the page.
    if (r.e && (r.d & 0xFF)) {
        uint16_t base = static_cast<uint16_t>(r.d + address);
        return read((base & 0xFFFF00) | ((base + offset) & 0xFF));
    }
    return readDirect(address + offset);
}

uint8_t Processor65816::readDirectN(uint32_t address) {
    return read(static_cast<uint16_t>(r.d + address));
}

uint8_t Processor65816::readBank(uint32_t address) {
    return read(((static_cast<uint32_t>(r.db) << 16) + address) & 0xFFFFFF);
}

void Processor65816::writeBank(uint32_t address, uint8_t data) {
    write(((static_cast<uint32_t>(r.db) << 16) + address) & 0xFFFFFF, data);
}

uint8_t Processor65816::readLong(uint32_t address) {
    return read(address & 0xFFFFFF);
}

void Processor65816::writeLong(uint32_t address, uint8_t data) {
    write(address & 0xFFFFFF, data);
}

uint8_t Processor65816::readStack(uint32_t address) {
    return read(static_cast<uint16_t>(r.s + address));
}

void Processor65816::writeStack(uint32_t address, uint8_t data) {
    write(static_cast<uint16_t>(r.s + address), data);
}

//  ALU algorithms  (following bsnes processor/wdc65816/algorithms.cpp)

uint8_t Processor65816::algorithmADC8(uint8_t data) {
    int result;
    if (!flagD()) {
        result = (r.a & 0xFF) + data + flagC();
    } else {
        result = (r.a & 0x0F) + (data & 0x0F) + flagC();
        if (result > 0x09) result += 0x06;
        result = (r.a & 0xF0) + (data & 0xF0) + (result > 0x0F ? 0x10 : 0) + (result & 0x0F);
    }

    setFlag(FlagV, (~((r.a & 0xFF) ^ data) & ((r.a & 0xFF) ^ result) & 0x80) != 0);
    if (flagD() && result > 0x9F) result += 0x60;
    setFlag(FlagC, result > 0xFF);
    uint8_t r8 = static_cast<uint8_t>(result);
    setNZ8(r8);
    r.a = (r.a & 0xFF00) | r8;
    return r8;
}

uint16_t Processor65816::algorithmADC16(uint16_t data) {
    int result;
    if (!flagD()) {
        result = r.a + data + flagC();
    } else {
        result = (r.a & 0x000F) + (data & 0x000F) + flagC();
        if (result >  0x0009) result += 0x0006;
        result = (r.a & 0x00F0) + (data & 0x00F0) + (result > 0x000F ? 0x0010 : 0) + (result & 0x000F);
        if (result >  0x009F) result += 0x0060;
        result = (r.a & 0x0F00) + (data & 0x0F00) + (result > 0x00FF ? 0x0100 : 0) + (result & 0x00FF);
        if (result >  0x09FF) result += 0x0600;
        result = (r.a & 0xF000) + (data & 0xF000) + (result > 0x0FFF ? 0x1000 : 0) + (result & 0x0FFF);
    }

    setFlag(FlagV, (~(r.a ^ data) & (r.a ^ result) & 0x8000) != 0);
    if (flagD() && result > 0x9FFF) result += 0x6000;
    setFlag(FlagC, result > 0xFFFF);
    uint16_t r16 = static_cast<uint16_t>(result);
    setNZ16(r16);
    r.a = r16;
    return r16;
}

uint8_t Processor65816::algorithmSBC8(uint8_t data) {
    int result;
    data ^= 0xFF;
    if (!flagD()) {
        result = (r.a & 0xFF) + data + flagC();
    } else {
        result = (r.a & 0x0F) + (data & 0x0F) + flagC();
        if (result <= 0x0F) result -= 0x06;
        result = (r.a & 0xF0) + (data & 0xF0) + (result > 0x0F ? 0x10 : 0) + (result & 0x0F);
    }

    setFlag(FlagV, (~((r.a & 0xFF) ^ data) & ((r.a & 0xFF) ^ result) & 0x80) != 0);
    if (flagD() && result <= 0xFF) result -= 0x60;
    setFlag(FlagC, result > 0xFF);
    uint8_t r8 = static_cast<uint8_t>(result);
    setNZ8(r8);
    r.a = (r.a & 0xFF00) | r8;
    return r8;
}

uint16_t Processor65816::algorithmSBC16(uint16_t data) {
    int result;
    data ^= 0xFFFF;
    if (!flagD()) {
        result = r.a + data + flagC();
    } else {
        result = (r.a & 0x000F) + (data & 0x000F) + flagC();
        if (result <=  0x000F) result -= 0x0006;
        result = (r.a & 0x00F0) + (data & 0x00F0) + (result > 0x000F ? 0x0010 : 0) + (result & 0x000F);
        if (result <=  0x00FF) result -= 0x0060;
        result = (r.a & 0x0F00) + (data & 0x0F00) + (result > 0x00FF ? 0x0100 : 0) + (result & 0x00FF);
        if (result <=  0x0FFF) result -= 0x0600;
        result = (r.a & 0xF000) + (data & 0xF000) + (result > 0x0FFF ? 0x1000 : 0) + (result & 0x0FFF);
    }

    setFlag(FlagV, (~(r.a ^ data) & (r.a ^ result) & 0x8000) != 0);
    if (flagD() && result <= 0xFFFF) result -= 0x6000;
    setFlag(FlagC, result > 0xFFFF);
    uint16_t r16 = static_cast<uint16_t>(result);
    setNZ16(r16);
    r.a = r16;
    return r16;
}

uint8_t Processor65816::algorithmAND8(uint8_t data) {
    uint8_t v = static_cast<uint8_t>((r.a & 0xFF) & data);
    setNZ8(v);
    r.a = (r.a & 0xFF00) | v;
    return v;
}

uint16_t Processor65816::algorithmAND16(uint16_t data) {
    r.a &= data;
    setNZ16(r.a);
    return r.a;
}

uint8_t Processor65816::algorithmASL8(uint8_t data) {
    setFlag(FlagC, (data & 0x80) != 0);
    data <<= 1;
    setNZ8(data);
    return data;
}

uint16_t Processor65816::algorithmASL16(uint16_t data) {
    setFlag(FlagC, (data & 0x8000) != 0);
    data <<= 1;
    setNZ16(data);
    return data;
}

uint8_t Processor65816::algorithmBIT8(uint8_t data) {
    setFlag(FlagN, (data & 0x80) != 0);
    setFlag(FlagV, (data & 0x40) != 0);
    setFlag(FlagZ, ((r.a & 0xFF) & data) == 0);
    return data;
}

uint16_t Processor65816::algorithmBIT16(uint16_t data) {
    setFlag(FlagN, (data & 0x8000) != 0);
    setFlag(FlagV, (data & 0x4000) != 0);
    setFlag(FlagZ, (r.a & data) == 0);
    return data;
}

uint8_t Processor65816::algorithmCMP8(uint8_t data) {
    int result = (r.a & 0xFF) - data;
    setFlag(FlagC, result >= 0);
    setNZ8(static_cast<uint8_t>(result));
    return data;
}

uint16_t Processor65816::algorithmCMP16(uint16_t data) {
    int result = r.a - data;
    setFlag(FlagC, result >= 0);
    setNZ16(static_cast<uint16_t>(result));
    return data;
}

uint8_t Processor65816::algorithmCPX8(uint8_t data) {
    int result = (r.x & 0xFF) - data;
    setFlag(FlagC, result >= 0);
    setNZ8(static_cast<uint8_t>(result));
    return data;
}

uint16_t Processor65816::algorithmCPX16(uint16_t data) {
    int result = r.x - data;
    setFlag(FlagC, result >= 0);
    setNZ16(static_cast<uint16_t>(result));
    return data;
}

uint8_t Processor65816::algorithmCPY8(uint8_t data) {
    int result = (r.y & 0xFF) - data;
    setFlag(FlagC, result >= 0);
    setNZ8(static_cast<uint8_t>(result));
    return data;
}

uint16_t Processor65816::algorithmCPY16(uint16_t data) {
    int result = r.y - data;
    setFlag(FlagC, result >= 0);
    setNZ16(static_cast<uint16_t>(result));
    return data;
}

uint8_t Processor65816::algorithmDEC8(uint8_t data) {
    data--;
    setNZ8(data);
    return data;
}

uint16_t Processor65816::algorithmDEC16(uint16_t data) {
    data--;
    setNZ16(data);
    return data;
}

uint8_t Processor65816::algorithmEOR8(uint8_t data) {
    uint8_t v = static_cast<uint8_t>((r.a & 0xFF) ^ data);
    setNZ8(v);
    r.a = (r.a & 0xFF00) | v;
    return v;
}

uint16_t Processor65816::algorithmEOR16(uint16_t data) {
    r.a ^= data;
    setNZ16(r.a);
    return r.a;
}

uint8_t Processor65816::algorithmINC8(uint8_t data) {
    data++;
    setNZ8(data);
    return data;
}

uint16_t Processor65816::algorithmINC16(uint16_t data) {
    data++;
    setNZ16(data);
    return data;
}

uint8_t Processor65816::algorithmLDA8(uint8_t data) {
    r.a = (r.a & 0xFF00) | data;
    setNZ8(data);
    return data;
}

uint16_t Processor65816::algorithmLDA16(uint16_t data) {
    r.a = data;
    setNZ16(data);
    return data;
}

uint8_t Processor65816::algorithmLDX8(uint8_t data) {
    r.x = data;
    setNZ8(data);
    return data;
}

uint16_t Processor65816::algorithmLDX16(uint16_t data) {
    r.x = data;
    setNZ16(data);
    return data;
}

uint8_t Processor65816::algorithmLDY8(uint8_t data) {
    r.y = data;
    setNZ8(data);
    return data;
}

uint16_t Processor65816::algorithmLDY16(uint16_t data) {
    r.y = data;
    setNZ16(data);
    return data;
}

uint8_t Processor65816::algorithmLSR8(uint8_t data) {
    setFlag(FlagC, (data & 0x01) != 0);
    data >>= 1;
    setNZ8(data);
    return data;
}

uint16_t Processor65816::algorithmLSR16(uint16_t data) {
    setFlag(FlagC, (data & 0x0001) != 0);
    data >>= 1;
    setNZ16(data);
    return data;
}

uint8_t Processor65816::algorithmORA8(uint8_t data) {
    uint8_t v = static_cast<uint8_t>((r.a & 0xFF) | data);
    setNZ8(v);
    r.a = (r.a & 0xFF00) | v;
    return v;
}

uint16_t Processor65816::algorithmORA16(uint16_t data) {
    r.a |= data;
    setNZ16(r.a);
    return r.a;
}

uint8_t Processor65816::algorithmROL8(uint8_t data) {
    bool carry = flagC();
    setFlag(FlagC, (data & 0x80) != 0);
    data = static_cast<uint8_t>((data << 1) | (carry ? 1 : 0));
    setNZ8(data);
    return data;
}

uint16_t Processor65816::algorithmROL16(uint16_t data) {
    bool carry = flagC();
    setFlag(FlagC, (data & 0x8000) != 0);
    data = static_cast<uint16_t>((data << 1) | (carry ? 1 : 0));
    setNZ16(data);
    return data;
}

uint8_t Processor65816::algorithmROR8(uint8_t data) {
    bool carry = flagC();
    setFlag(FlagC, (data & 0x01) != 0);
    data = static_cast<uint8_t>((data >> 1) | (carry ? 0x80 : 0));
    setNZ8(data);
    return data;
}

uint16_t Processor65816::algorithmROR16(uint16_t data) {
    bool carry = flagC();
    setFlag(FlagC, (data & 0x0001) != 0);
    data = static_cast<uint16_t>((data >> 1) | (carry ? 0x8000 : 0));
    setNZ16(data);
    return data;
}

uint8_t Processor65816::algorithmTRB8(uint8_t data) {
    setFlag(FlagZ, ((r.a & 0xFF) & data) == 0);
    return data & ~(r.a & 0xFF);
}

uint16_t Processor65816::algorithmTRB16(uint16_t data) {
    setFlag(FlagZ, (r.a & data) == 0);
    return data & ~r.a;
}

uint8_t Processor65816::algorithmTSB8(uint8_t data) {
    setFlag(FlagZ, ((r.a & 0xFF) & data) == 0);
    return data | (r.a & 0xFF);
}

uint16_t Processor65816::algorithmTSB16(uint16_t data) {
    setFlag(FlagZ, (r.a & data) == 0);
    return data | r.a;
}

//  Instruction implementations — Read operations
//  (following bsnes processor/wdc65816/instructions-read.cpp)

// Immediate — 8-bit
void Processor65816::instructionImmediateRead8(Alu8 op) {
    lastCycle();
    uint8_t data = fetch();
    (this->*op)(data);
}

// Immediate — 16-bit
void Processor65816::instructionImmediateRead16(Alu16 op) {
    uint8_t dl = fetch();
    lastCycle();
    uint8_t dh = fetch();
    (this->*op)(makeWord(dl, dh));
}

// Absolute — 8-bit   addr = [DB:abs]
void Processor65816::instructionBankRead8(Alu8 op) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    lastCycle();
    uint8_t data = readBank(makeWord(al, ah));
    (this->*op)(data);
}

// Absolute — 16-bit
void Processor65816::instructionBankRead16(Alu16 op) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    uint8_t dl = readBank(addr + 0);
    lastCycle();
    uint8_t dh = readBank(addr + 1);
    (this->*op)(makeWord(dl, dh));
}

// Absolute,X/Y — 8-bit   addr = [DB:abs+index]
void Processor65816::instructionBankRead8(Alu8 op, uint16_t index) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    idle4(addr, static_cast<uint16_t>(addr + index));
    lastCycle();
    uint8_t data = readBank(addr + index);
    (this->*op)(data);
}

// Absolute,X/Y — 16-bit
void Processor65816::instructionBankRead16(Alu16 op, uint16_t index) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    idle4(addr, static_cast<uint16_t>(addr + index));
    uint8_t dl = readBank(addr + index + 0);
    lastCycle();
    uint8_t dh = readBank(addr + index + 1);
    (this->*op)(makeWord(dl, dh));
}

// Absolute Long — 8-bit   addr = [long]  or  [long+index]
void Processor65816::instructionLongRead8(Alu8 op, uint16_t index) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint8_t ab = fetch();
    lastCycle();
    uint32_t addr = (static_cast<uint32_t>(ab) << 16) | makeWord(al, ah);
    uint8_t data = readLong(addr + index);
    (this->*op)(data);
}

// Absolute Long — 16-bit
void Processor65816::instructionLongRead16(Alu16 op, uint16_t index) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint8_t ab = fetch();
    uint32_t addr = (static_cast<uint32_t>(ab) << 16) | makeWord(al, ah);
    uint8_t dl = readLong(addr + index + 0);
    lastCycle();
    uint8_t dh = readLong(addr + index + 1);
    (this->*op)(makeWord(dl, dh));
}

// Direct Page — 8-bit   addr = [D+dp]
void Processor65816::instructionDirectRead8(Alu8 op) {
    uint8_t dp = fetch();
    idle2();
    lastCycle();
    uint8_t data = readDirect(dp);
    (this->*op)(data);
}

// Direct Page — 16-bit
void Processor65816::instructionDirectRead16(Alu16 op) {
    uint8_t dp = fetch();
    idle2();
    uint8_t dl = readDirect(dp + 0);
    lastCycle();
    uint8_t dh = readDirect(dp + 1);
    (this->*op)(makeWord(dl, dh));
}

// Direct Page,X/Y — 8-bit   addr = [D+dp+index]
void Processor65816::instructionDirectRead8(Alu8 op, uint16_t index) {
    uint8_t dp = fetch();
    idle2();
    idle();
    lastCycle();
    uint8_t data = readDirect(dp + index);
    (this->*op)(data);
}

// Direct Page,X/Y — 16-bit
void Processor65816::instructionDirectRead16(Alu16 op, uint16_t index) {
    uint8_t dp = fetch();
    idle2();
    idle();
    uint8_t dl = readDirect(dp + index + 0);
    lastCycle();
    uint8_t dh = readDirect(dp + index + 1);
    (this->*op)(makeWord(dl, dh));
}

// (Direct) — 8-bit   addr = [DB:[D+dp]]
void Processor65816::instructionIndirectRead8(Alu8 op) {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirect(dp + 0);
    uint8_t ah = readDirect(dp + 1);
    lastCycle();
    uint8_t data = readBank(makeWord(al, ah));
    (this->*op)(data);
}

// (Direct) — 16-bit
void Processor65816::instructionIndirectRead16(Alu16 op) {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirect(dp + 0);
    uint8_t ah = readDirect(dp + 1);
    uint16_t addr = makeWord(al, ah);
    uint8_t dl = readBank(addr + 0);
    lastCycle();
    uint8_t dh = readBank(addr + 1);
    (this->*op)(makeWord(dl, dh));
}

// (Direct,X) — 8-bit   addr = [DB:[D+dp+X]]
void Processor65816::instructionIndexedIndirectRead8(Alu8 op) {
    uint8_t dp = fetch();
    idle2();
    idle();
    uint8_t al = readDirectX(dp + r.x, 0);
    uint8_t ah = readDirectX(dp + r.x, 1);
    lastCycle();
    uint8_t data = readBank(makeWord(al, ah));
    (this->*op)(data);
}

// (Direct,X) — 16-bit
void Processor65816::instructionIndexedIndirectRead16(Alu16 op) {
    uint8_t dp = fetch();
    idle2();
    idle();
    uint8_t al = readDirectX(dp + r.x, 0);
    uint8_t ah = readDirectX(dp + r.x, 1);
    uint16_t addr = makeWord(al, ah);
    uint8_t dl = readBank(addr + 0);
    lastCycle();
    uint8_t dh = readBank(addr + 1);
    (this->*op)(makeWord(dl, dh));
}

// (Direct),Y — 8-bit   addr = [DB:[D+dp]+Y]
void Processor65816::instructionIndirectIndexedRead8(Alu8 op) {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirect(dp + 0);
    uint8_t ah = readDirect(dp + 1);
    uint16_t addr = makeWord(al, ah);
    idle4(addr, static_cast<uint16_t>(addr + r.y));
    lastCycle();
    uint8_t data = readBank(addr + r.y);
    (this->*op)(data);
}

// (Direct),Y — 16-bit
void Processor65816::instructionIndirectIndexedRead16(Alu16 op) {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirect(dp + 0);
    uint8_t ah = readDirect(dp + 1);
    uint16_t addr = makeWord(al, ah);
    idle4(addr, static_cast<uint16_t>(addr + r.y));
    uint8_t dl = readBank(addr + r.y + 0);
    lastCycle();
    uint8_t dh = readBank(addr + r.y + 1);
    (this->*op)(makeWord(dl, dh));
}

// [Direct] — 8-bit   addr = [[D+dp]] (long indirect)
void Processor65816::instructionIndirectLongRead8(Alu8 op, uint16_t index) {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirectN(dp + 0);
    uint8_t ah = readDirectN(dp + 1);
    uint8_t ab = readDirectN(dp + 2);
    uint32_t addr = (static_cast<uint32_t>(ab) << 16) | makeWord(al, ah);
    lastCycle();
    uint8_t data = readLong(addr + index);
    (this->*op)(data);
}

// [Direct] — 16-bit
void Processor65816::instructionIndirectLongRead16(Alu16 op, uint16_t index) {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirectN(dp + 0);
    uint8_t ah = readDirectN(dp + 1);
    uint8_t ab = readDirectN(dp + 2);
    uint32_t addr = (static_cast<uint32_t>(ab) << 16) | makeWord(al, ah);
    uint8_t dl = readLong(addr + index + 0);
    lastCycle();
    uint8_t dh = readLong(addr + index + 1);
    (this->*op)(makeWord(dl, dh));
}

// Stack Relative — 8-bit   addr = [S+offset]
void Processor65816::instructionStackRead8(Alu8 op) {
    uint8_t offset = fetch();
    idle();
    lastCycle();
    uint8_t data = readStack(offset);
    (this->*op)(data);
}

// Stack Relative — 16-bit
void Processor65816::instructionStackRead16(Alu16 op) {
    uint8_t offset = fetch();
    idle();
    uint8_t dl = readStack(offset + 0);
    lastCycle();
    uint8_t dh = readStack(offset + 1);
    (this->*op)(makeWord(dl, dh));
}

// (Stack Relative),Y — 8-bit   addr = [DB:[[S+offset]]+Y]
void Processor65816::instructionIndirectStackRead8(Alu8 op) {
    uint8_t offset = fetch();
    idle();
    uint8_t al = readStack(offset + 0);
    uint8_t ah = readStack(offset + 1);
    idle();
    lastCycle();
    uint8_t data = readBank(makeWord(al, ah) + r.y);
    (this->*op)(data);
}

// (Stack Relative),Y — 16-bit
void Processor65816::instructionIndirectStackRead16(Alu16 op) {
    uint8_t offset = fetch();
    idle();
    uint8_t al = readStack(offset + 0);
    uint8_t ah = readStack(offset + 1);
    idle();
    uint8_t dl = readBank(makeWord(al, ah) + r.y + 0);
    lastCycle();
    uint8_t dh = readBank(makeWord(al, ah) + r.y + 1);
    (this->*op)(makeWord(dl, dh));
}

//  Instruction implementations — Write operations
//  (following bsnes processor/wdc65816/instructions-write.cpp)

// STA/STX/STY/STZ Absolute — 8-bit
void Processor65816::instructionBankWrite8(uint16_t reg) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    lastCycle();
    writeBank(makeWord(al, ah), static_cast<uint8_t>(reg));
}

// Absolute — 16-bit
void Processor65816::instructionBankWrite16(uint16_t reg) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    writeBank(addr + 0, lo(reg));
    lastCycle();
    writeBank(addr + 1, hi(reg));
}

// Absolute,X/Y — 8-bit
void Processor65816::instructionBankWrite8(uint16_t reg, uint16_t index) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    idle();
    lastCycle();
    writeBank(makeWord(al, ah) + index, static_cast<uint8_t>(reg));
}

// Absolute,X/Y — 16-bit
void Processor65816::instructionBankWrite16(uint16_t reg, uint16_t index) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    idle();
    uint16_t addr = makeWord(al, ah);
    writeBank(addr + index + 0, lo(reg));
    lastCycle();
    writeBank(addr + index + 1, hi(reg));
}

// Absolute Long — 8-bit
void Processor65816::instructionLongWrite8(uint16_t index) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint8_t ab = fetch();
    uint32_t addr = (static_cast<uint32_t>(ab) << 16) | makeWord(al, ah);
    lastCycle();
    writeLong(addr + index, static_cast<uint8_t>(r.a));
}

// Absolute Long — 16-bit
void Processor65816::instructionLongWrite16(uint16_t index) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint8_t ab = fetch();
    uint32_t addr = (static_cast<uint32_t>(ab) << 16) | makeWord(al, ah);
    writeLong(addr + index + 0, lo(r.a));
    lastCycle();
    writeLong(addr + index + 1, hi(r.a));
}

// Direct Page — 8-bit
void Processor65816::instructionDirectWrite8(uint16_t reg) {
    uint8_t dp = fetch();
    idle2();
    lastCycle();
    writeDirect(dp, static_cast<uint8_t>(reg));
}

// Direct Page — 16-bit
void Processor65816::instructionDirectWrite16(uint16_t reg) {
    uint8_t dp = fetch();
    idle2();
    writeDirect(dp + 0, lo(reg));
    lastCycle();
    writeDirect(dp + 1, hi(reg));
}

// Direct Page,X/Y — 8-bit
void Processor65816::instructionDirectWrite8(uint16_t reg, uint16_t index) {
    uint8_t dp = fetch();
    idle2();
    idle();
    lastCycle();
    writeDirect(dp + index, static_cast<uint8_t>(reg));
}

// Direct Page,X/Y — 16-bit
void Processor65816::instructionDirectWrite16(uint16_t reg, uint16_t index) {
    uint8_t dp = fetch();
    idle2();
    idle();
    writeDirect(dp + index + 0, lo(reg));
    lastCycle();
    writeDirect(dp + index + 1, hi(reg));
}

// (Direct) — 8-bit
void Processor65816::instructionIndirectWrite8() {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirect(dp + 0);
    uint8_t ah = readDirect(dp + 1);
    lastCycle();
    writeBank(makeWord(al, ah), static_cast<uint8_t>(r.a));
}

// (Direct) — 16-bit
void Processor65816::instructionIndirectWrite16() {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirect(dp + 0);
    uint8_t ah = readDirect(dp + 1);
    uint16_t addr = makeWord(al, ah);
    writeBank(addr + 0, lo(r.a));
    lastCycle();
    writeBank(addr + 1, hi(r.a));
}

// (Direct,X) — 8-bit
void Processor65816::instructionIndexedIndirectWrite8() {
    uint8_t dp = fetch();
    idle2();
    idle();
    uint8_t al = readDirectX(dp + r.x, 0);
    uint8_t ah = readDirectX(dp + r.x, 1);
    lastCycle();
    writeBank(makeWord(al, ah), static_cast<uint8_t>(r.a));
}

// (Direct,X) — 16-bit
void Processor65816::instructionIndexedIndirectWrite16() {
    uint8_t dp = fetch();
    idle2();
    idle();
    uint8_t al = readDirectX(dp + r.x, 0);
    uint8_t ah = readDirectX(dp + r.x, 1);
    uint16_t addr = makeWord(al, ah);
    writeBank(addr + 0, lo(r.a));
    lastCycle();
    writeBank(addr + 1, hi(r.a));
}

// (Direct),Y — 8-bit
void Processor65816::instructionIndirectIndexedWrite8() {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirect(dp + 0);
    uint8_t ah = readDirect(dp + 1);
    idle();
    lastCycle();
    writeBank(makeWord(al, ah) + r.y, static_cast<uint8_t>(r.a));
}

// (Direct),Y — 16-bit
void Processor65816::instructionIndirectIndexedWrite16() {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirect(dp + 0);
    uint8_t ah = readDirect(dp + 1);
    idle();
    uint16_t addr = makeWord(al, ah);
    writeBank(addr + r.y + 0, lo(r.a));
    lastCycle();
    writeBank(addr + r.y + 1, hi(r.a));
}

// [Direct] — 8-bit
void Processor65816::instructionIndirectLongWrite8(uint16_t index) {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirectN(dp + 0);
    uint8_t ah = readDirectN(dp + 1);
    uint8_t ab = readDirectN(dp + 2);
    uint32_t addr = (static_cast<uint32_t>(ab) << 16) | makeWord(al, ah);
    lastCycle();
    writeLong(addr + index, static_cast<uint8_t>(r.a));
}

// [Direct] — 16-bit
void Processor65816::instructionIndirectLongWrite16(uint16_t index) {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirectN(dp + 0);
    uint8_t ah = readDirectN(dp + 1);
    uint8_t ab = readDirectN(dp + 2);
    uint32_t addr = (static_cast<uint32_t>(ab) << 16) | makeWord(al, ah);
    writeLong(addr + index + 0, lo(r.a));
    lastCycle();
    writeLong(addr + index + 1, hi(r.a));
}

// Stack Relative — 8-bit
void Processor65816::instructionStackWrite8() {
    uint8_t offset = fetch();
    idle();
    lastCycle();
    writeStack(offset, static_cast<uint8_t>(r.a));
}

// Stack Relative — 16-bit
void Processor65816::instructionStackWrite16() {
    uint8_t offset = fetch();
    idle();
    writeStack(offset + 0, lo(r.a));
    lastCycle();
    writeStack(offset + 1, hi(r.a));
}

// (Stack Relative),Y — 8-bit
void Processor65816::instructionIndirectStackWrite8() {
    uint8_t offset = fetch();
    idle();
    uint8_t al = readStack(offset + 0);
    uint8_t ah = readStack(offset + 1);
    idle();
    lastCycle();
    writeBank(makeWord(al, ah) + r.y, static_cast<uint8_t>(r.a));
}

// (Stack Relative),Y — 16-bit
void Processor65816::instructionIndirectStackWrite16() {
    uint8_t offset = fetch();
    idle();
    uint8_t al = readStack(offset + 0);
    uint8_t ah = readStack(offset + 1);
    idle();
    writeBank(makeWord(al, ah) + r.y + 0, lo(r.a));
    lastCycle();
    writeBank(makeWord(al, ah) + r.y + 1, hi(r.a));
}

//  Instruction implementations — Read-Modify-Write
//  (following bsnes processor/wdc65816/instructions-modify.cpp)

// Implied (accumulator or register) — 8-bit
void Processor65816::instructionImpliedModify8(Alu8 op, uint16_t& reg) {
    lastCycle();
    idleIRQ();
    uint8_t val = static_cast<uint8_t>(reg);
    val = (this->*op)(val);
    reg = (reg & 0xFF00) | val;
}

// Implied — 16-bit
void Processor65816::instructionImpliedModify16(Alu16 op, uint16_t& reg) {
    lastCycle();
    idleIRQ();
    reg = (this->*op)(reg);
}

// Absolute — 8-bit
void Processor65816::instructionBankModify8(Alu8 op) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    uint8_t data = readBank(addr);
    idle();
    lastCycle();
    data = (this->*op)(data);
    writeBank(addr, data);
}

// Absolute — 16-bit
void Processor65816::instructionBankModify16(Alu16 op) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    uint8_t dl = readBank(addr + 0);
    uint8_t dh = readBank(addr + 1);
    idle();
    uint16_t data = (this->*op)(makeWord(dl, dh));
    writeBank(addr + 1, hi(data));
    lastCycle();
    writeBank(addr + 0, lo(data));
}

// Absolute,X — 8-bit
void Processor65816::instructionBankIndexedModify8(Alu8 op) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    idle();
    uint8_t data = readBank(addr + r.x);
    idle();
    lastCycle();
    data = (this->*op)(data);
    writeBank(addr + r.x, data);
}

// Absolute,X — 16-bit
void Processor65816::instructionBankIndexedModify16(Alu16 op) {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    idle();
    uint8_t dl = readBank(addr + r.x + 0);
    uint8_t dh = readBank(addr + r.x + 1);
    idle();
    uint16_t data = (this->*op)(makeWord(dl, dh));
    writeBank(addr + r.x + 1, hi(data));
    lastCycle();
    writeBank(addr + r.x + 0, lo(data));
}

// Direct Page — 8-bit
void Processor65816::instructionDirectModify8(Alu8 op) {
    uint8_t dp = fetch();
    idle2();
    uint8_t data = readDirect(dp);
    idle();
    lastCycle();
    data = (this->*op)(data);
    writeDirect(dp, data);
}

// Direct Page — 16-bit
void Processor65816::instructionDirectModify16(Alu16 op) {
    uint8_t dp = fetch();
    idle2();
    uint8_t dl = readDirect(dp + 0);
    uint8_t dh = readDirect(dp + 1);
    idle();
    uint16_t data = (this->*op)(makeWord(dl, dh));
    writeDirect(dp + 1, hi(data));
    lastCycle();
    writeDirect(dp + 0, lo(data));
}

// Direct Page,X — 8-bit
void Processor65816::instructionDirectIndexedModify8(Alu8 op) {
    uint8_t dp = fetch();
    idle2();
    idle();
    uint8_t data = readDirect(dp + r.x);
    idle();
    lastCycle();
    data = (this->*op)(data);
    writeDirect(dp + r.x, data);
}

// Direct Page,X — 16-bit
void Processor65816::instructionDirectIndexedModify16(Alu16 op) {
    uint8_t dp = fetch();
    idle2();
    idle();
    uint8_t dl = readDirect(dp + r.x + 0);
    uint8_t dh = readDirect(dp + r.x + 1);
    idle();
    uint16_t data = (this->*op)(makeWord(dl, dh));
    writeDirect(dp + r.x + 1, hi(data));
    lastCycle();
    writeDirect(dp + r.x + 0, lo(data));
}

//  Instruction implementations — Program counter control
//  (following bsnes processor/wdc65816/instructions-pc.cpp)

// Branch (conditional)
void Processor65816::instructionBranch(bool take) {
    if (!take) {
        lastCycle();
        fetch();
        return;
    }
    uint8_t offset = fetch();
    uint16_t target = static_cast<uint16_t>(r.pc + static_cast<int8_t>(offset));
    idle6(target);
    lastCycle();
    idle();
    r.pc = target;
}

// BRL — Branch Long (always, 16-bit offset)
void Processor65816::instructionBranchLong() {
    uint8_t ol = fetch();
    uint8_t oh = fetch();
    lastCycle();
    idle();
    r.pc = static_cast<uint16_t>(r.pc + static_cast<int16_t>(makeWord(ol, oh)));
}

// JMP Absolute
void Processor65816::instructionJumpShort() {
    uint8_t al = fetch();
    lastCycle();
    uint8_t ah = fetch();
    r.pc = makeWord(al, ah);
}

// JMP Long (24-bit — JML)
void Processor65816::instructionJumpLong() {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    lastCycle();
    uint8_t ab = fetch();
    r.pb = ab;
    r.pc = makeWord(al, ah);
}

// JMP (Indirect)
void Processor65816::instructionJumpIndirect() {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    uint8_t tl = read(addr);
    lastCycle();
    uint8_t th = read(static_cast<uint16_t>(addr + 1));
    r.pc = makeWord(tl, th);
}

// JMP (Absolute,X) — Indexed Indirect
void Processor65816::instructionJumpIndexedIndirect() {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    idle();
    uint16_t addr = static_cast<uint16_t>(makeWord(al, ah) + r.x);
    uint32_t fullAddr = (static_cast<uint32_t>(r.pb) << 16) | addr;
    uint8_t tl = read(fullAddr);
    lastCycle();
    uint8_t th = read((static_cast<uint32_t>(r.pb) << 16) | static_cast<uint16_t>(addr + 1));
    r.pc = makeWord(tl, th);
}

// JMP [Indirect Long] — JML
void Processor65816::instructionJumpIndirectLong() {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    uint16_t addr = makeWord(al, ah);
    uint8_t tl = read(addr);
    uint8_t th = read(static_cast<uint16_t>(addr + 1));
    lastCycle();
    uint8_t tb = read(static_cast<uint16_t>(addr + 2));
    r.pc = makeWord(tl, th);
    r.pb = tb;
}

// JSR Absolute
void Processor65816::instructionCallShort() {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    idle();
    uint16_t returnAddr = static_cast<uint16_t>(r.pc - 1);
    push(hi(returnAddr));
    lastCycle();
    push(lo(returnAddr));
    r.pc = makeWord(al, ah);
}

// JSL — JSR Long (24-bit)
void Processor65816::instructionCallLong() {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    pushN(r.pb);
    idle();
    uint8_t ab = fetch();
    uint16_t returnAddr = static_cast<uint16_t>(r.pc - 1);
    pushN(hi(returnAddr));
    lastCycle();
    pushN(lo(returnAddr));
    r.pb = ab;
    r.pc = makeWord(al, ah);
    if (r.e) r.s = static_cast<uint16_t>(0x0100 | lo(r.s));
}

// JSR (Absolute,X) — Call Indexed Indirect
void Processor65816::instructionCallIndexedIndirect() {
    uint8_t al = fetch();
    uint16_t returnAddr = static_cast<uint16_t>(r.pc);
    pushN(hi(returnAddr));
    pushN(lo(returnAddr));
    uint8_t ah = fetch();
    idle();
    uint16_t addr = static_cast<uint16_t>(makeWord(al, ah) + r.x);
    uint32_t fullAddr = (static_cast<uint32_t>(r.pb) << 16) | addr;
    uint8_t tl = read(fullAddr);
    lastCycle();
    uint8_t th = read((static_cast<uint32_t>(r.pb) << 16) | static_cast<uint16_t>(addr + 1));
    r.pc = makeWord(tl, th);
    if (r.e) r.s = static_cast<uint16_t>(0x0100 | lo(r.s));
}

// RTI — Return from Interrupt
void Processor65816::instructionReturnInterrupt() {
    idle();
    idle();
    r.p = pull();
    if (r.e) r.p |= (FlagM | FlagX);
    if (xf()) { r.x &= 0xFF; r.y &= 0xFF; }
    uint8_t pcl = pull();
    if (r.e) {
        lastCycle();
        uint8_t pch = pull();
        r.pc = makeWord(pcl, pch);
    } else {
        uint8_t pch = pull();
        lastCycle();
        r.pb = pull();
        r.pc = makeWord(pcl, pch);
    }
}

// RTS — Return from Subroutine
void Processor65816::instructionReturnShort() {
    idle();
    idle();
    uint8_t pcl = pull();
    uint8_t pch = pull();
    lastCycle();
    idle();
    r.pc = static_cast<uint16_t>(makeWord(pcl, pch) + 1);
}

// RTL — Return from Subroutine Long
void Processor65816::instructionReturnLong() {
    idle();
    idle();
    uint8_t pcl = pullN();
    uint8_t pch = pullN();
    lastCycle();
    r.pb = pullN();
    r.pc = static_cast<uint16_t>(makeWord(pcl, pch) + 1);
    if (r.e) r.s = static_cast<uint16_t>(0x0100 | lo(r.s));
}

//  Instruction implementations — Miscellaneous
//  (following bsnes processor/wdc65816/instructions-other.cpp)

// BIT #imm — 8-bit (only sets Z, doesn't touch N/V)
void Processor65816::instructionBitImmediate8() {
    lastCycle();
    uint8_t data = fetch();
    setFlag(FlagZ, ((r.a & 0xFF) & data) == 0);
}

// BIT #imm — 16-bit
void Processor65816::instructionBitImmediate16() {
    uint8_t dl = fetch();
    lastCycle();
    uint8_t dh = fetch();
    setFlag(FlagZ, (r.a & makeWord(dl, dh)) == 0);
}

// NOP
void Processor65816::instructionNoOperation() {
    lastCycle();
    idleIRQ();
}

// WDM — 2-byte NOP prefix
void Processor65816::instructionPrefix() {
    lastCycle();
    fetch();
}

// XBA — Exchange B and A (high/low bytes of A)
void Processor65816::instructionExchangeBA() {
    idle();
    lastCycle();
    idle();
    r.a = static_cast<uint16_t>((r.a >> 8) | (r.a << 8));
    setNZ8(lo(r.a));
}

// MVN/MVP — Block Move (8-bit A: 1-byte transfer count)
void Processor65816::instructionBlockMove8(int adjust) {
    uint8_t dstBank = fetch();
    uint8_t srcBank = fetch();
    r.db = dstBank;
    uint8_t data = read((static_cast<uint32_t>(srcBank) << 16) | r.x);
    write((static_cast<uint32_t>(dstBank) << 16) | r.y, data);
    idle();
    r.x = static_cast<uint16_t>((r.x & 0xFF00) | ((r.x + adjust) & 0xFF));
    r.y = static_cast<uint16_t>((r.y & 0xFF00) | ((r.y + adjust) & 0xFF));
    lastCycle();
    idle();
    if (r.a-- != 0) r.pc -= 3;
}

// Block Move — 16-bit
void Processor65816::instructionBlockMove16(int adjust) {
    uint8_t dstBank = fetch();
    uint8_t srcBank = fetch();
    r.db = dstBank;
    uint8_t data = read((static_cast<uint32_t>(srcBank) << 16) | r.x);
    write((static_cast<uint32_t>(dstBank) << 16) | r.y, data);
    idle();
    r.x = static_cast<uint16_t>(r.x + adjust);
    r.y = static_cast<uint16_t>(r.y + adjust);
    lastCycle();
    idle();
    if (r.a-- != 0) r.pc -= 3;
}

// BRK / COP — Software Interrupt
void Processor65816::instructionInterrupt(uint16_t vector) {
    fetch();
    if (!r.e) pushN(r.pb);
    push(hi(r.pc));
    push(lo(r.pc));
    push(r.p);
    r.p |= FlagI;  // Set interrupt disable
    r.p &= ~FlagD; // Clear decimal mode
    r.pb = 0x00;
    uint8_t pcl = read(vector + 0);
    lastCycle();
    uint8_t pch = read(vector + 1);
    r.pc = makeWord(pcl, pch);
}

// STP — Stop Processor
void Processor65816::instructionStop() {
    r.stp = true;
    // Return to the system scheduler while the CPU remains stopped.
    idle();
    idle();
}

// WAI — Wait for Interrupt
void Processor65816::instructionWait() {
    r.wai = true;
    while (r.wai) {
        lastCycle();
        idle();
        // In a real system, the subclass's lastCycle() would clear wai when
        // NMI or IRQ fires. In our single-threaded model, we break out after
        // one idle to let the outer loop poll interrupts.
        break;
    }
    idle();
}

// XCE — Exchange Carry and Emulation
void Processor65816::instructionExchangeCE() {
    lastCycle();
    idleIRQ();
    bool oldC = flagC();
    bool oldE = r.e;
    setFlag(FlagC, oldE);
    setE(oldC);
}

// SEC/SEI/SED — Set individual flag
void Processor65816::instructionSetFlag(uint8_t flag) {
    lastCycle();
    idleIRQ();
    setFlag(flag, true);
}

// CLC/CLI/CLD/CLV — Clear individual flag
void Processor65816::instructionClearFlag(uint8_t flag) {
    lastCycle();
    idleIRQ();
    setFlag(flag, false);
}

// REP — Reset Processor Status Bits
void Processor65816::instructionResetP() {
    uint8_t data = fetch();
    lastCycle();
    idle();
    r.p &= ~data;
    if (r.e) r.p |= (FlagM | FlagX);
    if (xf()) { r.x &= 0xFF; r.y &= 0xFF; }
}

// SEP — Set Processor Status Bits
void Processor65816::instructionSetP() {
    uint8_t data = fetch();
    lastCycle();
    idle();
    r.p |= data;
    if (r.e) r.p |= (FlagM | FlagX);
    if (xf()) { r.x &= 0xFF; r.y &= 0xFF; }
}

// Transfer — 8-bit (TAX, TXA, TAY, TYA, TXY, TYX)
void Processor65816::instructionTransfer8(uint16_t src, uint16_t& dst) {
    lastCycle();
    idleIRQ();
    dst = (dst & 0xFF00) | (src & 0xFF);
    setNZ8(static_cast<uint8_t>(dst));
}

// Transfer — 16-bit
void Processor65816::instructionTransfer16(uint16_t src, uint16_t& dst) {
    lastCycle();
    idleIRQ();
    dst = src;
    setNZ16(dst);
}

// TCS — Transfer C (accumulator 16-bit) to S
void Processor65816::instructionTransferCS() {
    lastCycle();
    idleIRQ();
    r.s = r.a;
    if (r.e) r.s = static_cast<uint16_t>(0x0100 | (r.s & 0xFF));
}

// TSX — Transfer S to X (8-bit index)
void Processor65816::instructionTransferSX8() {
    lastCycle();
    idleIRQ();
    r.x = r.s & 0xFF;
    setNZ8(static_cast<uint8_t>(r.x));
}

// TSX — Transfer S to X (16-bit index)
void Processor65816::instructionTransferSX16() {
    lastCycle();
    idleIRQ();
    r.x = r.s;
    setNZ16(r.x);
}

// TXS — Transfer X to S
void Processor65816::instructionTransferXS() {
    lastCycle();
    idleIRQ();
    if (r.e) {
        r.s = static_cast<uint16_t>(0x0100 | (r.x & 0xFF));
    } else {
        r.s = r.x;
    }
}

// Push 8-bit  (PHA/PHX/PHY/PHB/PHK)
void Processor65816::instructionPush8(uint16_t reg) {
    idle();
    lastCycle();
    push(static_cast<uint8_t>(reg));
}

// Push 16-bit (PHA 16 / PHX 16 / PHY 16)
void Processor65816::instructionPush16(uint16_t reg) {
    idle();
    push(hi(reg));
    lastCycle();
    push(lo(reg));
}

// PHD — Push D (always 16-bit)
void Processor65816::instructionPushD() {
    idle();
    pushN(hi(r.d));
    lastCycle();
    pushN(lo(r.d));
    if (r.e) r.s = static_cast<uint16_t>(0x0100 | lo(r.s));
}

// Pull 8-bit  (PLA/PLX/PLY)
void Processor65816::instructionPull8(uint16_t& reg) {
    idle();
    idle();
    lastCycle();
    uint8_t data = pull();
    reg = (reg & 0xFF00) | data;
    setNZ8(data);
}

// Pull 16-bit
void Processor65816::instructionPull16(uint16_t& reg) {
    idle();
    idle();
    uint8_t dl = pull();
    lastCycle();
    uint8_t dh = pull();
    reg = makeWord(dl, dh);
    setNZ16(reg);
}

// PLD — Pull D (always 16-bit)
void Processor65816::instructionPullD() {
    idle();
    idle();
    uint8_t dl = pullN();
    lastCycle();
    uint8_t dh = pullN();
    r.d = makeWord(dl, dh);
    setNZ16(r.d);
    if (r.e) r.s = static_cast<uint16_t>(0x0100 | lo(r.s));
}

// PLB — Pull Data Bank
void Processor65816::instructionPullB() {
    idle();
    idle();
    lastCycle();
    r.db = pull();
    setNZ8(r.db);
}

// PLP — Pull Processor Status
void Processor65816::instructionPullP() {
    idle();
    idle();
    lastCycle();
    r.p = pull();
    if (r.e) r.p |= (FlagM | FlagX);
    if (xf()) { r.x &= 0xFF; r.y &= 0xFF; }
}

// PEA — Push Effective Absolute Address
void Processor65816::instructionPushEffectiveAddress() {
    uint8_t al = fetch();
    uint8_t ah = fetch();
    pushN(ah);
    lastCycle();
    pushN(al);
    if (r.e) r.s = static_cast<uint16_t>(0x0100 | lo(r.s));
}

// PEI — Push Effective Indirect Address
void Processor65816::instructionPushEffectiveIndirectAddress() {
    uint8_t dp = fetch();
    idle2();
    uint8_t al = readDirectN(dp + 0);
    uint8_t ah = readDirectN(dp + 1);
    pushN(ah);
    lastCycle();
    pushN(al);
    if (r.e) r.s = static_cast<uint16_t>(0x0100 | lo(r.s));
}

// PER — Push Effective Relative Address
void Processor65816::instructionPushEffectiveRelativeAddress() {
    uint8_t ol = fetch();
    uint8_t oh = fetch();
    idle();
    uint16_t addr = static_cast<uint16_t>(r.pc + static_cast<int16_t>(makeWord(ol, oh)));
    pushN(hi(addr));
    lastCycle();
    pushN(lo(addr));
    if (r.e) r.s = static_cast<uint16_t>(0x0100 | lo(r.s));
}

//  Interrupt entry

void Processor65816::enterInterrupt(Interrupt type) {
    r.wai = false;
    // Hardware interrupt entry: dummy read + idle cycle, matching bsnes interrupt()
    read(static_cast<uint32_t>(r.pb) << 16 | r.pc);
    idle();

    if (type != Interrupt::Reset) {
        if (!r.e) pushN(r.pb);
        push(hi(r.pc));
        push(lo(r.pc));
        push(r.e ? (r.p & ~0x10) : r.p);  // Clear B flag in emulation mode
    }

    setFlag(FlagI, true);
    setFlag(FlagD, false);
    r.pb = 0x00;

    uint16_t vec = vectorAddress(type);
    uint8_t pcl = read(vec + 0);
    lastCycle();
    uint8_t pch = read(vec + 1);
    r.pc = makeWord(pcl, pch);
}

uint16_t Processor65816::vectorAddress(Interrupt type) const {
    if (r.e) {
        switch (type) {
        case Interrupt::Cop:   return 0xFFF4;
        case Interrupt::Abort: return 0xFFF8;
        case Interrupt::Nmi:   return 0xFFFA;
        case Interrupt::Reset: return 0xFFFC;
        case Interrupt::Irq:
        case Interrupt::Brk:
        default:               return 0xFFFE;
        }
    }
    switch (type) {
    case Interrupt::Cop:   return 0xFFE4;
    case Interrupt::Brk:   return 0xFFE6;
    case Interrupt::Abort: return 0xFFE8;
    case Interrupt::Nmi:   return 0xFFEA;
    case Interrupt::Irq:   return 0xFFEE;
    case Interrupt::Reset:
    default:               return 0xFFFC;
    }
}

//  256-opcode dispatch table
//  (following bsnes processor/wdc65816/instruction.hpp + instruction.cpp)
//
//  For each opcode we call the correct instruction function, parameterized
//  by the current M/X flag state.  Uses function pointer aliases:
//    Alu8/Alu16 for ALU operations
//    Register references for transfers/pushes/pulls

void Processor65816::instruction() {
    uint8_t opcode = fetch();

    // Shorthand macros for selecting 8/16-bit ALU variants
    #define M8  mf()
    #define X8  xf()

    switch (opcode) {

    // ADC
    case 0x69: M8 ? instructionImmediateRead8(&Processor65816::algorithmADC8)
                   : instructionImmediateRead16(&Processor65816::algorithmADC16); break;
    case 0x6D: M8 ? instructionBankRead8(&Processor65816::algorithmADC8)
                   : instructionBankRead16(&Processor65816::algorithmADC16); break;
    case 0x7D: M8 ? instructionBankRead8(&Processor65816::algorithmADC8, r.x)
                   : instructionBankRead16(&Processor65816::algorithmADC16, r.x); break;
    case 0x79: M8 ? instructionBankRead8(&Processor65816::algorithmADC8, r.y)
                   : instructionBankRead16(&Processor65816::algorithmADC16, r.y); break;
    case 0x6F: M8 ? instructionLongRead8(&Processor65816::algorithmADC8)
                   : instructionLongRead16(&Processor65816::algorithmADC16); break;
    case 0x7F: M8 ? instructionLongRead8(&Processor65816::algorithmADC8, r.x)
                   : instructionLongRead16(&Processor65816::algorithmADC16, r.x); break;
    case 0x65: M8 ? instructionDirectRead8(&Processor65816::algorithmADC8)
                   : instructionDirectRead16(&Processor65816::algorithmADC16); break;
    case 0x75: M8 ? instructionDirectRead8(&Processor65816::algorithmADC8, r.x)
                   : instructionDirectRead16(&Processor65816::algorithmADC16, r.x); break;
    case 0x72: M8 ? instructionIndirectRead8(&Processor65816::algorithmADC8)
                   : instructionIndirectRead16(&Processor65816::algorithmADC16); break;
    case 0x61: M8 ? instructionIndexedIndirectRead8(&Processor65816::algorithmADC8)
                   : instructionIndexedIndirectRead16(&Processor65816::algorithmADC16); break;
    case 0x71: M8 ? instructionIndirectIndexedRead8(&Processor65816::algorithmADC8)
                   : instructionIndirectIndexedRead16(&Processor65816::algorithmADC16); break;
    case 0x67: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmADC8)
                   : instructionIndirectLongRead16(&Processor65816::algorithmADC16); break;
    case 0x77: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmADC8, r.y)
                   : instructionIndirectLongRead16(&Processor65816::algorithmADC16, r.y); break;
    case 0x63: M8 ? instructionStackRead8(&Processor65816::algorithmADC8)
                   : instructionStackRead16(&Processor65816::algorithmADC16); break;
    case 0x73: M8 ? instructionIndirectStackRead8(&Processor65816::algorithmADC8)
                   : instructionIndirectStackRead16(&Processor65816::algorithmADC16); break;

    // SBC
    case 0xE9: M8 ? instructionImmediateRead8(&Processor65816::algorithmSBC8)
                   : instructionImmediateRead16(&Processor65816::algorithmSBC16); break;
    case 0xED: M8 ? instructionBankRead8(&Processor65816::algorithmSBC8)
                   : instructionBankRead16(&Processor65816::algorithmSBC16); break;
    case 0xFD: M8 ? instructionBankRead8(&Processor65816::algorithmSBC8, r.x)
                   : instructionBankRead16(&Processor65816::algorithmSBC16, r.x); break;
    case 0xF9: M8 ? instructionBankRead8(&Processor65816::algorithmSBC8, r.y)
                   : instructionBankRead16(&Processor65816::algorithmSBC16, r.y); break;
    case 0xEF: M8 ? instructionLongRead8(&Processor65816::algorithmSBC8)
                   : instructionLongRead16(&Processor65816::algorithmSBC16); break;
    case 0xFF: M8 ? instructionLongRead8(&Processor65816::algorithmSBC8, r.x)
                   : instructionLongRead16(&Processor65816::algorithmSBC16, r.x); break;
    case 0xE5: M8 ? instructionDirectRead8(&Processor65816::algorithmSBC8)
                   : instructionDirectRead16(&Processor65816::algorithmSBC16); break;
    case 0xF5: M8 ? instructionDirectRead8(&Processor65816::algorithmSBC8, r.x)
                   : instructionDirectRead16(&Processor65816::algorithmSBC16, r.x); break;
    case 0xF2: M8 ? instructionIndirectRead8(&Processor65816::algorithmSBC8)
                   : instructionIndirectRead16(&Processor65816::algorithmSBC16); break;
    case 0xE1: M8 ? instructionIndexedIndirectRead8(&Processor65816::algorithmSBC8)
                   : instructionIndexedIndirectRead16(&Processor65816::algorithmSBC16); break;
    case 0xF1: M8 ? instructionIndirectIndexedRead8(&Processor65816::algorithmSBC8)
                   : instructionIndirectIndexedRead16(&Processor65816::algorithmSBC16); break;
    case 0xE7: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmSBC8)
                   : instructionIndirectLongRead16(&Processor65816::algorithmSBC16); break;
    case 0xF7: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmSBC8, r.y)
                   : instructionIndirectLongRead16(&Processor65816::algorithmSBC16, r.y); break;
    case 0xE3: M8 ? instructionStackRead8(&Processor65816::algorithmSBC8)
                   : instructionStackRead16(&Processor65816::algorithmSBC16); break;
    case 0xF3: M8 ? instructionIndirectStackRead8(&Processor65816::algorithmSBC8)
                   : instructionIndirectStackRead16(&Processor65816::algorithmSBC16); break;

    // AND
    case 0x29: M8 ? instructionImmediateRead8(&Processor65816::algorithmAND8)
                   : instructionImmediateRead16(&Processor65816::algorithmAND16); break;
    case 0x2D: M8 ? instructionBankRead8(&Processor65816::algorithmAND8)
                   : instructionBankRead16(&Processor65816::algorithmAND16); break;
    case 0x3D: M8 ? instructionBankRead8(&Processor65816::algorithmAND8, r.x)
                   : instructionBankRead16(&Processor65816::algorithmAND16, r.x); break;
    case 0x39: M8 ? instructionBankRead8(&Processor65816::algorithmAND8, r.y)
                   : instructionBankRead16(&Processor65816::algorithmAND16, r.y); break;
    case 0x2F: M8 ? instructionLongRead8(&Processor65816::algorithmAND8)
                   : instructionLongRead16(&Processor65816::algorithmAND16); break;
    case 0x3F: M8 ? instructionLongRead8(&Processor65816::algorithmAND8, r.x)
                   : instructionLongRead16(&Processor65816::algorithmAND16, r.x); break;
    case 0x25: M8 ? instructionDirectRead8(&Processor65816::algorithmAND8)
                   : instructionDirectRead16(&Processor65816::algorithmAND16); break;
    case 0x35: M8 ? instructionDirectRead8(&Processor65816::algorithmAND8, r.x)
                   : instructionDirectRead16(&Processor65816::algorithmAND16, r.x); break;
    case 0x32: M8 ? instructionIndirectRead8(&Processor65816::algorithmAND8)
                   : instructionIndirectRead16(&Processor65816::algorithmAND16); break;
    case 0x21: M8 ? instructionIndexedIndirectRead8(&Processor65816::algorithmAND8)
                   : instructionIndexedIndirectRead16(&Processor65816::algorithmAND16); break;
    case 0x31: M8 ? instructionIndirectIndexedRead8(&Processor65816::algorithmAND8)
                   : instructionIndirectIndexedRead16(&Processor65816::algorithmAND16); break;
    case 0x27: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmAND8)
                   : instructionIndirectLongRead16(&Processor65816::algorithmAND16); break;
    case 0x37: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmAND8, r.y)
                   : instructionIndirectLongRead16(&Processor65816::algorithmAND16, r.y); break;
    case 0x23: M8 ? instructionStackRead8(&Processor65816::algorithmAND8)
                   : instructionStackRead16(&Processor65816::algorithmAND16); break;
    case 0x33: M8 ? instructionIndirectStackRead8(&Processor65816::algorithmAND8)
                   : instructionIndirectStackRead16(&Processor65816::algorithmAND16); break;

    // ORA
    case 0x09: M8 ? instructionImmediateRead8(&Processor65816::algorithmORA8)
                   : instructionImmediateRead16(&Processor65816::algorithmORA16); break;
    case 0x0D: M8 ? instructionBankRead8(&Processor65816::algorithmORA8)
                   : instructionBankRead16(&Processor65816::algorithmORA16); break;
    case 0x1D: M8 ? instructionBankRead8(&Processor65816::algorithmORA8, r.x)
                   : instructionBankRead16(&Processor65816::algorithmORA16, r.x); break;
    case 0x19: M8 ? instructionBankRead8(&Processor65816::algorithmORA8, r.y)
                   : instructionBankRead16(&Processor65816::algorithmORA16, r.y); break;
    case 0x0F: M8 ? instructionLongRead8(&Processor65816::algorithmORA8)
                   : instructionLongRead16(&Processor65816::algorithmORA16); break;
    case 0x1F: M8 ? instructionLongRead8(&Processor65816::algorithmORA8, r.x)
                   : instructionLongRead16(&Processor65816::algorithmORA16, r.x); break;
    case 0x05: M8 ? instructionDirectRead8(&Processor65816::algorithmORA8)
                   : instructionDirectRead16(&Processor65816::algorithmORA16); break;
    case 0x15: M8 ? instructionDirectRead8(&Processor65816::algorithmORA8, r.x)
                   : instructionDirectRead16(&Processor65816::algorithmORA16, r.x); break;
    case 0x12: M8 ? instructionIndirectRead8(&Processor65816::algorithmORA8)
                   : instructionIndirectRead16(&Processor65816::algorithmORA16); break;
    case 0x01: M8 ? instructionIndexedIndirectRead8(&Processor65816::algorithmORA8)
                   : instructionIndexedIndirectRead16(&Processor65816::algorithmORA16); break;
    case 0x11: M8 ? instructionIndirectIndexedRead8(&Processor65816::algorithmORA8)
                   : instructionIndirectIndexedRead16(&Processor65816::algorithmORA16); break;
    case 0x07: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmORA8)
                   : instructionIndirectLongRead16(&Processor65816::algorithmORA16); break;
    case 0x17: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmORA8, r.y)
                   : instructionIndirectLongRead16(&Processor65816::algorithmORA16, r.y); break;
    case 0x03: M8 ? instructionStackRead8(&Processor65816::algorithmORA8)
                   : instructionStackRead16(&Processor65816::algorithmORA16); break;
    case 0x13: M8 ? instructionIndirectStackRead8(&Processor65816::algorithmORA8)
                   : instructionIndirectStackRead16(&Processor65816::algorithmORA16); break;

    // EOR
    case 0x49: M8 ? instructionImmediateRead8(&Processor65816::algorithmEOR8)
                   : instructionImmediateRead16(&Processor65816::algorithmEOR16); break;
    case 0x4D: M8 ? instructionBankRead8(&Processor65816::algorithmEOR8)
                   : instructionBankRead16(&Processor65816::algorithmEOR16); break;
    case 0x5D: M8 ? instructionBankRead8(&Processor65816::algorithmEOR8, r.x)
                   : instructionBankRead16(&Processor65816::algorithmEOR16, r.x); break;
    case 0x59: M8 ? instructionBankRead8(&Processor65816::algorithmEOR8, r.y)
                   : instructionBankRead16(&Processor65816::algorithmEOR16, r.y); break;
    case 0x4F: M8 ? instructionLongRead8(&Processor65816::algorithmEOR8)
                   : instructionLongRead16(&Processor65816::algorithmEOR16); break;
    case 0x5F: M8 ? instructionLongRead8(&Processor65816::algorithmEOR8, r.x)
                   : instructionLongRead16(&Processor65816::algorithmEOR16, r.x); break;
    case 0x45: M8 ? instructionDirectRead8(&Processor65816::algorithmEOR8)
                   : instructionDirectRead16(&Processor65816::algorithmEOR16); break;
    case 0x55: M8 ? instructionDirectRead8(&Processor65816::algorithmEOR8, r.x)
                   : instructionDirectRead16(&Processor65816::algorithmEOR16, r.x); break;
    case 0x52: M8 ? instructionIndirectRead8(&Processor65816::algorithmEOR8)
                   : instructionIndirectRead16(&Processor65816::algorithmEOR16); break;
    case 0x41: M8 ? instructionIndexedIndirectRead8(&Processor65816::algorithmEOR8)
                   : instructionIndexedIndirectRead16(&Processor65816::algorithmEOR16); break;
    case 0x51: M8 ? instructionIndirectIndexedRead8(&Processor65816::algorithmEOR8)
                   : instructionIndirectIndexedRead16(&Processor65816::algorithmEOR16); break;
    case 0x47: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmEOR8)
                   : instructionIndirectLongRead16(&Processor65816::algorithmEOR16); break;
    case 0x57: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmEOR8, r.y)
                   : instructionIndirectLongRead16(&Processor65816::algorithmEOR16, r.y); break;
    case 0x43: M8 ? instructionStackRead8(&Processor65816::algorithmEOR8)
                   : instructionStackRead16(&Processor65816::algorithmEOR16); break;
    case 0x53: M8 ? instructionIndirectStackRead8(&Processor65816::algorithmEOR8)
                   : instructionIndirectStackRead16(&Processor65816::algorithmEOR16); break;

    // CMP
    case 0xC9: M8 ? instructionImmediateRead8(&Processor65816::algorithmCMP8)
                   : instructionImmediateRead16(&Processor65816::algorithmCMP16); break;
    case 0xCD: M8 ? instructionBankRead8(&Processor65816::algorithmCMP8)
                   : instructionBankRead16(&Processor65816::algorithmCMP16); break;
    case 0xDD: M8 ? instructionBankRead8(&Processor65816::algorithmCMP8, r.x)
                   : instructionBankRead16(&Processor65816::algorithmCMP16, r.x); break;
    case 0xD9: M8 ? instructionBankRead8(&Processor65816::algorithmCMP8, r.y)
                   : instructionBankRead16(&Processor65816::algorithmCMP16, r.y); break;
    case 0xCF: M8 ? instructionLongRead8(&Processor65816::algorithmCMP8)
                   : instructionLongRead16(&Processor65816::algorithmCMP16); break;
    case 0xDF: M8 ? instructionLongRead8(&Processor65816::algorithmCMP8, r.x)
                   : instructionLongRead16(&Processor65816::algorithmCMP16, r.x); break;
    case 0xC5: M8 ? instructionDirectRead8(&Processor65816::algorithmCMP8)
                   : instructionDirectRead16(&Processor65816::algorithmCMP16); break;
    case 0xD5: M8 ? instructionDirectRead8(&Processor65816::algorithmCMP8, r.x)
                   : instructionDirectRead16(&Processor65816::algorithmCMP16, r.x); break;
    case 0xD2: M8 ? instructionIndirectRead8(&Processor65816::algorithmCMP8)
                   : instructionIndirectRead16(&Processor65816::algorithmCMP16); break;
    case 0xC1: M8 ? instructionIndexedIndirectRead8(&Processor65816::algorithmCMP8)
                   : instructionIndexedIndirectRead16(&Processor65816::algorithmCMP16); break;
    case 0xD1: M8 ? instructionIndirectIndexedRead8(&Processor65816::algorithmCMP8)
                   : instructionIndirectIndexedRead16(&Processor65816::algorithmCMP16); break;
    case 0xC7: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmCMP8)
                   : instructionIndirectLongRead16(&Processor65816::algorithmCMP16); break;
    case 0xD7: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmCMP8, r.y)
                   : instructionIndirectLongRead16(&Processor65816::algorithmCMP16, r.y); break;
    case 0xC3: M8 ? instructionStackRead8(&Processor65816::algorithmCMP8)
                   : instructionStackRead16(&Processor65816::algorithmCMP16); break;
    case 0xD3: M8 ? instructionIndirectStackRead8(&Processor65816::algorithmCMP8)
                   : instructionIndirectStackRead16(&Processor65816::algorithmCMP16); break;

    // CPX
    case 0xE0: X8 ? instructionImmediateRead8(&Processor65816::algorithmCPX8)
                   : instructionImmediateRead16(&Processor65816::algorithmCPX16); break;
    case 0xEC: X8 ? instructionBankRead8(&Processor65816::algorithmCPX8)
                   : instructionBankRead16(&Processor65816::algorithmCPX16); break;
    case 0xE4: X8 ? instructionDirectRead8(&Processor65816::algorithmCPX8)
                   : instructionDirectRead16(&Processor65816::algorithmCPX16); break;

    // CPY
    case 0xC0: X8 ? instructionImmediateRead8(&Processor65816::algorithmCPY8)
                   : instructionImmediateRead16(&Processor65816::algorithmCPY16); break;
    case 0xCC: X8 ? instructionBankRead8(&Processor65816::algorithmCPY8)
                   : instructionBankRead16(&Processor65816::algorithmCPY16); break;
    case 0xC4: X8 ? instructionDirectRead8(&Processor65816::algorithmCPY8)
                   : instructionDirectRead16(&Processor65816::algorithmCPY16); break;

    // LDA
    case 0xA9: M8 ? instructionImmediateRead8(&Processor65816::algorithmLDA8)
                   : instructionImmediateRead16(&Processor65816::algorithmLDA16); break;
    case 0xAD: M8 ? instructionBankRead8(&Processor65816::algorithmLDA8)
                   : instructionBankRead16(&Processor65816::algorithmLDA16); break;
    case 0xBD: M8 ? instructionBankRead8(&Processor65816::algorithmLDA8, r.x)
                   : instructionBankRead16(&Processor65816::algorithmLDA16, r.x); break;
    case 0xB9: M8 ? instructionBankRead8(&Processor65816::algorithmLDA8, r.y)
                   : instructionBankRead16(&Processor65816::algorithmLDA16, r.y); break;
    case 0xAF: M8 ? instructionLongRead8(&Processor65816::algorithmLDA8)
                   : instructionLongRead16(&Processor65816::algorithmLDA16); break;
    case 0xBF: M8 ? instructionLongRead8(&Processor65816::algorithmLDA8, r.x)
                   : instructionLongRead16(&Processor65816::algorithmLDA16, r.x); break;
    case 0xA5: M8 ? instructionDirectRead8(&Processor65816::algorithmLDA8)
                   : instructionDirectRead16(&Processor65816::algorithmLDA16); break;
    case 0xB5: M8 ? instructionDirectRead8(&Processor65816::algorithmLDA8, r.x)
                   : instructionDirectRead16(&Processor65816::algorithmLDA16, r.x); break;
    case 0xB2: M8 ? instructionIndirectRead8(&Processor65816::algorithmLDA8)
                   : instructionIndirectRead16(&Processor65816::algorithmLDA16); break;
    case 0xA1: M8 ? instructionIndexedIndirectRead8(&Processor65816::algorithmLDA8)
                   : instructionIndexedIndirectRead16(&Processor65816::algorithmLDA16); break;
    case 0xB1: M8 ? instructionIndirectIndexedRead8(&Processor65816::algorithmLDA8)
                   : instructionIndirectIndexedRead16(&Processor65816::algorithmLDA16); break;
    case 0xA7: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmLDA8)
                   : instructionIndirectLongRead16(&Processor65816::algorithmLDA16); break;
    case 0xB7: M8 ? instructionIndirectLongRead8(&Processor65816::algorithmLDA8, r.y)
                   : instructionIndirectLongRead16(&Processor65816::algorithmLDA16, r.y); break;
    case 0xA3: M8 ? instructionStackRead8(&Processor65816::algorithmLDA8)
                   : instructionStackRead16(&Processor65816::algorithmLDA16); break;
    case 0xB3: M8 ? instructionIndirectStackRead8(&Processor65816::algorithmLDA8)
                   : instructionIndirectStackRead16(&Processor65816::algorithmLDA16); break;

    // LDX
    case 0xA2: X8 ? instructionImmediateRead8(&Processor65816::algorithmLDX8)
                   : instructionImmediateRead16(&Processor65816::algorithmLDX16); break;
    case 0xAE: X8 ? instructionBankRead8(&Processor65816::algorithmLDX8)
                   : instructionBankRead16(&Processor65816::algorithmLDX16); break;
    case 0xBE: X8 ? instructionBankRead8(&Processor65816::algorithmLDX8, r.y)
                   : instructionBankRead16(&Processor65816::algorithmLDX16, r.y); break;
    case 0xA6: X8 ? instructionDirectRead8(&Processor65816::algorithmLDX8)
                   : instructionDirectRead16(&Processor65816::algorithmLDX16); break;
    case 0xB6: X8 ? instructionDirectRead8(&Processor65816::algorithmLDX8, r.y)
                   : instructionDirectRead16(&Processor65816::algorithmLDX16, r.y); break;

    // LDY
    case 0xA0: X8 ? instructionImmediateRead8(&Processor65816::algorithmLDY8)
                   : instructionImmediateRead16(&Processor65816::algorithmLDY16); break;
    case 0xAC: X8 ? instructionBankRead8(&Processor65816::algorithmLDY8)
                   : instructionBankRead16(&Processor65816::algorithmLDY16); break;
    case 0xBC: X8 ? instructionBankRead8(&Processor65816::algorithmLDY8, r.x)
                   : instructionBankRead16(&Processor65816::algorithmLDY16, r.x); break;
    case 0xA4: X8 ? instructionDirectRead8(&Processor65816::algorithmLDY8)
                   : instructionDirectRead16(&Processor65816::algorithmLDY16); break;
    case 0xB4: X8 ? instructionDirectRead8(&Processor65816::algorithmLDY8, r.x)
                   : instructionDirectRead16(&Processor65816::algorithmLDY16, r.x); break;

    // BIT (non-immediate)
    case 0x2C: M8 ? instructionBankRead8(&Processor65816::algorithmBIT8)
                   : instructionBankRead16(&Processor65816::algorithmBIT16); break;
    case 0x3C: M8 ? instructionBankRead8(&Processor65816::algorithmBIT8, r.x)
                   : instructionBankRead16(&Processor65816::algorithmBIT16, r.x); break;
    case 0x24: M8 ? instructionDirectRead8(&Processor65816::algorithmBIT8)
                   : instructionDirectRead16(&Processor65816::algorithmBIT16); break;
    case 0x34: M8 ? instructionDirectRead8(&Processor65816::algorithmBIT8, r.x)
                   : instructionDirectRead16(&Processor65816::algorithmBIT16, r.x); break;

    // BIT #immediate
    case 0x89: M8 ? instructionBitImmediate8() : instructionBitImmediate16(); break;

    // STA
    case 0x8D: M8 ? instructionBankWrite8(r.a) : instructionBankWrite16(r.a); break;
    case 0x9D: M8 ? instructionBankWrite8(r.a, r.x) : instructionBankWrite16(r.a, r.x); break;
    case 0x99: M8 ? instructionBankWrite8(r.a, r.y) : instructionBankWrite16(r.a, r.y); break;
    case 0x8F: M8 ? instructionLongWrite8() : instructionLongWrite16(); break;
    case 0x9F: M8 ? instructionLongWrite8(r.x) : instructionLongWrite16(r.x); break;
    case 0x85: M8 ? instructionDirectWrite8(r.a) : instructionDirectWrite16(r.a); break;
    case 0x95: M8 ? instructionDirectWrite8(r.a, r.x) : instructionDirectWrite16(r.a, r.x); break;
    case 0x92: M8 ? instructionIndirectWrite8() : instructionIndirectWrite16(); break;
    case 0x81: M8 ? instructionIndexedIndirectWrite8() : instructionIndexedIndirectWrite16(); break;
    case 0x91: M8 ? instructionIndirectIndexedWrite8() : instructionIndirectIndexedWrite16(); break;
    case 0x87: M8 ? instructionIndirectLongWrite8() : instructionIndirectLongWrite16(); break;
    case 0x97: M8 ? instructionIndirectLongWrite8(r.y) : instructionIndirectLongWrite16(r.y); break;
    case 0x83: M8 ? instructionStackWrite8() : instructionStackWrite16(); break;
    case 0x93: M8 ? instructionIndirectStackWrite8() : instructionIndirectStackWrite16(); break;

    // STX
    case 0x8E: X8 ? instructionBankWrite8(r.x) : instructionBankWrite16(r.x); break;
    case 0x86: X8 ? instructionDirectWrite8(r.x) : instructionDirectWrite16(r.x); break;
    case 0x96: X8 ? instructionDirectWrite8(r.x, r.y) : instructionDirectWrite16(r.x, r.y); break;

    // STY
    case 0x8C: X8 ? instructionBankWrite8(r.y) : instructionBankWrite16(r.y); break;
    case 0x84: X8 ? instructionDirectWrite8(r.y) : instructionDirectWrite16(r.y); break;
    case 0x94: X8 ? instructionDirectWrite8(r.y, r.x) : instructionDirectWrite16(r.y, r.x); break;

    // STZ
    case 0x9C: M8 ? instructionBankWrite8(0) : instructionBankWrite16(0); break;
    case 0x9E: M8 ? instructionBankWrite8(0, r.x) : instructionBankWrite16(0, r.x); break;
    case 0x64: M8 ? instructionDirectWrite8(0) : instructionDirectWrite16(0); break;
    case 0x74: M8 ? instructionDirectWrite8(0, r.x) : instructionDirectWrite16(0, r.x); break;

    // ASL
    case 0x0A: M8 ? instructionImpliedModify8(&Processor65816::algorithmASL8, r.a)
                   : instructionImpliedModify16(&Processor65816::algorithmASL16, r.a); break;
    case 0x0E: M8 ? instructionBankModify8(&Processor65816::algorithmASL8)
                   : instructionBankModify16(&Processor65816::algorithmASL16); break;
    case 0x1E: M8 ? instructionBankIndexedModify8(&Processor65816::algorithmASL8)
                   : instructionBankIndexedModify16(&Processor65816::algorithmASL16); break;
    case 0x06: M8 ? instructionDirectModify8(&Processor65816::algorithmASL8)
                   : instructionDirectModify16(&Processor65816::algorithmASL16); break;
    case 0x16: M8 ? instructionDirectIndexedModify8(&Processor65816::algorithmASL8)
                   : instructionDirectIndexedModify16(&Processor65816::algorithmASL16); break;

    // LSR
    case 0x4A: M8 ? instructionImpliedModify8(&Processor65816::algorithmLSR8, r.a)
                   : instructionImpliedModify16(&Processor65816::algorithmLSR16, r.a); break;
    case 0x4E: M8 ? instructionBankModify8(&Processor65816::algorithmLSR8)
                   : instructionBankModify16(&Processor65816::algorithmLSR16); break;
    case 0x5E: M8 ? instructionBankIndexedModify8(&Processor65816::algorithmLSR8)
                   : instructionBankIndexedModify16(&Processor65816::algorithmLSR16); break;
    case 0x46: M8 ? instructionDirectModify8(&Processor65816::algorithmLSR8)
                   : instructionDirectModify16(&Processor65816::algorithmLSR16); break;
    case 0x56: M8 ? instructionDirectIndexedModify8(&Processor65816::algorithmLSR8)
                   : instructionDirectIndexedModify16(&Processor65816::algorithmLSR16); break;

    // ROL
    case 0x2A: M8 ? instructionImpliedModify8(&Processor65816::algorithmROL8, r.a)
                   : instructionImpliedModify16(&Processor65816::algorithmROL16, r.a); break;
    case 0x2E: M8 ? instructionBankModify8(&Processor65816::algorithmROL8)
                   : instructionBankModify16(&Processor65816::algorithmROL16); break;
    case 0x3E: M8 ? instructionBankIndexedModify8(&Processor65816::algorithmROL8)
                   : instructionBankIndexedModify16(&Processor65816::algorithmROL16); break;
    case 0x26: M8 ? instructionDirectModify8(&Processor65816::algorithmROL8)
                   : instructionDirectModify16(&Processor65816::algorithmROL16); break;
    case 0x36: M8 ? instructionDirectIndexedModify8(&Processor65816::algorithmROL8)
                   : instructionDirectIndexedModify16(&Processor65816::algorithmROL16); break;

    // ROR
    case 0x6A: M8 ? instructionImpliedModify8(&Processor65816::algorithmROR8, r.a)
                   : instructionImpliedModify16(&Processor65816::algorithmROR16, r.a); break;
    case 0x6E: M8 ? instructionBankModify8(&Processor65816::algorithmROR8)
                   : instructionBankModify16(&Processor65816::algorithmROR16); break;
    case 0x7E: M8 ? instructionBankIndexedModify8(&Processor65816::algorithmROR8)
                   : instructionBankIndexedModify16(&Processor65816::algorithmROR16); break;
    case 0x66: M8 ? instructionDirectModify8(&Processor65816::algorithmROR8)
                   : instructionDirectModify16(&Processor65816::algorithmROR16); break;
    case 0x76: M8 ? instructionDirectIndexedModify8(&Processor65816::algorithmROR8)
                   : instructionDirectIndexedModify16(&Processor65816::algorithmROR16); break;

    // INC
    case 0x1A: M8 ? instructionImpliedModify8(&Processor65816::algorithmINC8, r.a)
                   : instructionImpliedModify16(&Processor65816::algorithmINC16, r.a); break;
    case 0xEE: M8 ? instructionBankModify8(&Processor65816::algorithmINC8)
                   : instructionBankModify16(&Processor65816::algorithmINC16); break;
    case 0xFE: M8 ? instructionBankIndexedModify8(&Processor65816::algorithmINC8)
                   : instructionBankIndexedModify16(&Processor65816::algorithmINC16); break;
    case 0xE6: M8 ? instructionDirectModify8(&Processor65816::algorithmINC8)
                   : instructionDirectModify16(&Processor65816::algorithmINC16); break;
    case 0xF6: M8 ? instructionDirectIndexedModify8(&Processor65816::algorithmINC8)
                   : instructionDirectIndexedModify16(&Processor65816::algorithmINC16); break;

    // DEC
    case 0x3A: M8 ? instructionImpliedModify8(&Processor65816::algorithmDEC8, r.a)
                   : instructionImpliedModify16(&Processor65816::algorithmDEC16, r.a); break;
    case 0xCE: M8 ? instructionBankModify8(&Processor65816::algorithmDEC8)
                   : instructionBankModify16(&Processor65816::algorithmDEC16); break;
    case 0xDE: M8 ? instructionBankIndexedModify8(&Processor65816::algorithmDEC8)
                   : instructionBankIndexedModify16(&Processor65816::algorithmDEC16); break;
    case 0xC6: M8 ? instructionDirectModify8(&Processor65816::algorithmDEC8)
                   : instructionDirectModify16(&Processor65816::algorithmDEC16); break;
    case 0xD6: M8 ? instructionDirectIndexedModify8(&Processor65816::algorithmDEC8)
                   : instructionDirectIndexedModify16(&Processor65816::algorithmDEC16); break;

    // INX / DEX / INY / DEY
    case 0xE8: X8 ? instructionImpliedModify8(&Processor65816::algorithmINC8, r.x)
                   : instructionImpliedModify16(&Processor65816::algorithmINC16, r.x); break;
    case 0xCA: X8 ? instructionImpliedModify8(&Processor65816::algorithmDEC8, r.x)
                   : instructionImpliedModify16(&Processor65816::algorithmDEC16, r.x); break;
    case 0xC8: X8 ? instructionImpliedModify8(&Processor65816::algorithmINC8, r.y)
                   : instructionImpliedModify16(&Processor65816::algorithmINC16, r.y); break;
    case 0x88: X8 ? instructionImpliedModify8(&Processor65816::algorithmDEC8, r.y)
                   : instructionImpliedModify16(&Processor65816::algorithmDEC16, r.y); break;

    // TSB / TRB
    case 0x0C: M8 ? instructionBankModify8(&Processor65816::algorithmTSB8)
                   : instructionBankModify16(&Processor65816::algorithmTSB16); break;
    case 0x04: M8 ? instructionDirectModify8(&Processor65816::algorithmTSB8)
                   : instructionDirectModify16(&Processor65816::algorithmTSB16); break;
    case 0x1C: M8 ? instructionBankModify8(&Processor65816::algorithmTRB8)
                   : instructionBankModify16(&Processor65816::algorithmTRB16); break;
    case 0x14: M8 ? instructionDirectModify8(&Processor65816::algorithmTRB8)
                   : instructionDirectModify16(&Processor65816::algorithmTRB16); break;

    // Branches
    case 0x90: instructionBranch(!flagC()); break;  // BCC
    case 0xB0: instructionBranch( flagC()); break;  // BCS
    case 0xD0: instructionBranch(!flagZ()); break;  // BNE
    case 0xF0: instructionBranch( flagZ()); break;  // BEQ
    case 0x10: instructionBranch(!flagN()); break;  // BPL
    case 0x30: instructionBranch( flagN()); break;  // BMI
    case 0x50: instructionBranch(!flagV()); break;  // BVC
    case 0x70: instructionBranch( flagV()); break;  // BVS
    case 0x80: instructionBranch(true);     break;  // BRA
    case 0x82: instructionBranchLong();     break;  // BRL

    // Jumps
    case 0x4C: instructionJumpShort();          break;  // JMP abs
    case 0x5C: instructionJumpLong();           break;  // JML
    case 0x6C: instructionJumpIndirect();       break;  // JMP (abs)
    case 0x7C: instructionJumpIndexedIndirect(); break;  // JMP (abs,X)
    case 0xDC: instructionJumpIndirectLong();   break;  // JML [abs]

    // Calls
    case 0x20: instructionCallShort();          break;  // JSR abs
    case 0x22: instructionCallLong();           break;  // JSL
    case 0xFC: instructionCallIndexedIndirect(); break;  // JSR (abs,X)

    // Returns
    case 0x40: instructionReturnInterrupt(); break;  // RTI
    case 0x60: instructionReturnShort();     break;  // RTS
    case 0x6B: instructionReturnLong();      break;  // RTL

    // Flag operations
    case 0x18: instructionClearFlag(FlagC); break;  // CLC
    case 0x38: instructionSetFlag(FlagC);   break;  // SEC
    case 0x58: instructionClearFlag(FlagI); break;  // CLI
    case 0x78: instructionSetFlag(FlagI);   break;  // SEI
    case 0xD8: instructionClearFlag(FlagD); break;  // CLD
    case 0xF8: instructionSetFlag(FlagD);   break;  // SED
    case 0xB8: instructionClearFlag(FlagV); break;  // CLV
    case 0xC2: instructionResetP();         break;  // REP
    case 0xE2: instructionSetP();           break;  // SEP
    case 0xFB: instructionExchangeCE();     break;  // XCE

    // Transfers
    case 0xAA: X8 ? instructionTransfer8(r.a, r.x) : instructionTransfer16(r.a, r.x); break;  // TAX
    case 0x8A: M8 ? instructionTransfer8(r.x, r.a) : instructionTransfer16(r.x, r.a); break;  // TXA
    case 0xA8: X8 ? instructionTransfer8(r.a, r.y) : instructionTransfer16(r.a, r.y); break;  // TAY
    case 0x98: M8 ? instructionTransfer8(r.y, r.a) : instructionTransfer16(r.y, r.a); break;  // TYA
    case 0x9B: X8 ? instructionTransfer8(r.x, r.y) : instructionTransfer16(r.x, r.y); break;  // TXY
    case 0xBB: X8 ? instructionTransfer8(r.y, r.x) : instructionTransfer16(r.y, r.x); break;  // TYX
    case 0x5B: instructionTransfer16(r.a, r.d); break;       // TCD (always 16-bit)
    case 0x7B: instructionTransfer16(r.d, r.a); break;       // TDC (always 16-bit)
    case 0x1B: instructionTransferCS();    break;             // TCS
    case 0x3B: instructionTransfer16(r.s, r.a); break;        // TSC (always 16-bit)
    case 0x9A: instructionTransferXS();    break;             // TXS
    case 0xBA: X8 ? instructionTransferSX8() : instructionTransferSX16(); break;  // TSX

    // Push
    case 0x48: M8 ? instructionPush8(r.a)  : instructionPush16(r.a);  break;  // PHA
    case 0xDA: X8 ? instructionPush8(r.x)  : instructionPush16(r.x);  break;  // PHX
    case 0x5A: X8 ? instructionPush8(r.y)  : instructionPush16(r.y);  break;  // PHY
    case 0x08: instructionPush8(r.p);  break;                                   // PHP
    case 0x8B: instructionPush8(r.db); break;                                   // PHB
    case 0x4B: instructionPush8(r.pb); break;                                   // PHK
    case 0x0B: instructionPushD();     break;                                   // PHD

    // Pull
    case 0x68: M8 ? instructionPull8(r.a)  : instructionPull16(r.a);  break;  // PLA
    case 0xFA: X8 ? instructionPull8(r.x)  : instructionPull16(r.x);  break;  // PLX
    case 0x7A: X8 ? instructionPull8(r.y)  : instructionPull16(r.y);  break;  // PLY
    case 0x28: instructionPullP();   break;                                     // PLP
    case 0xAB: instructionPullB();   break;                                     // PLB
    case 0x2B: instructionPullD();   break;                                     // PLD

    // Push effective address
    case 0xF4: instructionPushEffectiveAddress();          break;  // PEA
    case 0xD4: instructionPushEffectiveIndirectAddress();   break;  // PEI
    case 0x62: instructionPushEffectiveRelativeAddress();   break;  // PER

    // Block moves
    case 0x54: xf() ? instructionBlockMove8(+1)  : instructionBlockMove16(+1);  break;  // MVN
    case 0x44: xf() ? instructionBlockMove8(-1)  : instructionBlockMove16(-1);  break;  // MVP

    // Interrupts
    case 0x00: instructionInterrupt(r.e ? 0xFFFE : 0xFFE6); break;  // BRK
    case 0x02: instructionInterrupt(r.e ? 0xFFF4 : 0xFFE4); break;  // COP

    // Misc
    case 0xEA: instructionNoOperation();  break;  // NOP
    case 0x42: instructionPrefix();       break;  // WDM
    case 0xEB: instructionExchangeBA();   break;  // XBA
    case 0xDB: instructionStop();         break;  // STP
    case 0xCB: instructionWait();         break;  // WAI

    default:
        // Unknown opcode — treat as 1-byte NOP (should never happen with correct ROM)
        lastCycle();
        idleIRQ();
        break;
    }

    #undef M8
    #undef X8
}

} // namespace snes::core
