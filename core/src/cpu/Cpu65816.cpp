// snes emulator
// core/src/cpu/Cpu65816.cpp
// The 65816 instruction and bus timing implementation.

#include "snes/core/Cpu65816.hpp"

namespace snes::core {

namespace {

uint8_t DecimalAdd8(uint8_t a, uint8_t b, bool carryIn, bool* carryOut) {
    int low = (a & 0x0F) + (b & 0x0F) + (carryIn ? 1 : 0);
    int high = (a >> 4) + (b >> 4);

    if (low > 9) {
        low += 6;
        ++high;
    }

    if (high > 9) {
        high += 6;
    }

    if (carryOut != nullptr) {
        *carryOut = high > 0x0F;
    }

    return static_cast<uint8_t>((high << 4) | (low & 0x0F));
}

uint8_t DecimalSub8(uint8_t a, uint8_t b, bool carryIn, bool* carryOut) {
    int low = (a & 0x0F) - (b & 0x0F) - (carryIn ? 0 : 1);
    int high = (a >> 4) - (b >> 4);

    if (low < 0) {
        low -= 6;
        --high;
    }

    if (high < 0) {
        high -= 6;
    }

    if (carryOut != nullptr) {
        *carryOut = high >= 0;
    }

    return static_cast<uint8_t>((high << 4) | (low & 0x0F));
}

} // namespace

Cpu65816::Cpu65816(ICpuBus& bus)
    : bus_(bus) {
    Reset();
}

void Cpu65816::Reset() {
    r_ = Registers{};
    r_.s = 0x01FF;
    r_.p = FlagM | FlagX | FlagI;
    r_.e = true;
    r_.pb = 0;
    r_.db = 0;
    r_.d = 0;

    pendingNmi_ = false;
    pendingIrq_ = false;
    pendingAbort_ = false;

    EnterInterrupt(Interrupt::Reset);
}

void Cpu65816::RequestNmi() {
    pendingNmi_ = true;
}

void Cpu65816::RequestIrq() {
    pendingIrq_ = true;
}

void Cpu65816::RequestAbort() {
    pendingAbort_ = true;
}

const Cpu65816::Registers& Cpu65816::GetRegisters() const noexcept {
    return r_;
}

uint64_t Cpu65816::Cycles() const noexcept {
    return cycles_;
}

uint32_t Cpu65816::Step() {
    const auto before = cycles_;

    if (pendingAbort_) {
        pendingAbort_ = false;
        EnterInterrupt(Interrupt::Abort);
        cycles_ += 8;
        return static_cast<uint32_t>(cycles_ - before);
    }

    if (pendingNmi_) {
        pendingNmi_ = false;
        EnterInterrupt(Interrupt::Nmi);
        cycles_ += 7;
        return static_cast<uint32_t>(cycles_ - before);
    }

    if (pendingIrq_ && !GetFlag(FlagI)) {
        pendingIrq_ = false;
        EnterInterrupt(Interrupt::Irq);
        cycles_ += 7;
        return static_cast<uint32_t>(cycles_ - before);
    }

    const auto opcode = Fetch8();

    switch (opcode) {
    case 0x00: // BRK
        Fetch8();
        EnterInterrupt(Interrupt::Brk);
        cycles_ += 7;
        break;

    case 0x02: // COP
        Fetch8();
        EnterInterrupt(Interrupt::Cop);
        cycles_ += 7;
        break;

    case 0x18: // CLC
        SetFlag(FlagC, false);
        cycles_ += 2;
        break;

    case 0x38: // SEC
        SetFlag(FlagC, true);
        cycles_ += 2;
        break;

    case 0x58: // CLI
        SetFlag(FlagI, false);
        cycles_ += 2;
        break;

    case 0x78: // SEI
        SetFlag(FlagI, true);
        cycles_ += 2;
        break;

    case 0xD8: // CLD
        SetFlag(FlagD, false);
        cycles_ += 2;
        break;

    case 0xF8: // SED
        SetFlag(FlagD, true);
        cycles_ += 2;
        break;

    case 0xC2: // REP
        OpRep();
        cycles_ += 3;
        break;

    case 0xE2: // SEP
        OpSep();
        cycles_ += 3;
        break;

    case 0xFB: // XCE
    {
        const bool oldE = r_.e;
        const bool oldC = GetFlag(FlagC);
        SetE(oldC);
        SetFlag(FlagC, oldE);
        cycles_ += 2;
        break;
    }

    case 0xEA: // NOP
        cycles_ += 2;
        break;

    case 0xA9: { // LDA #imm
        const auto value = ImmA();
        if (A8()) {
            r_.a = (r_.a & 0xFF00) | (value & 0x00FF);
            SetNZ8(static_cast<uint8_t>(r_.a));
            cycles_ += 2;
        } else {
            r_.a = value;
            SetNZ16(r_.a);
            cycles_ += 3;
        }
        break;
    }

    case 0xA2: { // LDX #imm
        const auto value = ImmIndex();
        if (Index8()) {
            r_.x = value & 0x00FF;
            SetNZ8(static_cast<uint8_t>(r_.x));
            cycles_ += 2;
        } else {
            r_.x = value;
            SetNZ16(r_.x);
            cycles_ += 3;
        }
        break;
    }

    case 0xA0: { // LDY #imm
        const auto value = ImmIndex();
        if (Index8()) {
            r_.y = value & 0x00FF;
            SetNZ8(static_cast<uint8_t>(r_.y));
            cycles_ += 2;
        } else {
            r_.y = value;
            SetNZ16(r_.y);
            cycles_ += 3;
        }
        break;
    }

    case 0x8D: { // STA abs
        const auto addr = Fetch16();
        WriteA(AbsAddress(addr), r_.a);
        cycles_ += A8() ? 4 : 5;
        break;
    }

    case 0x8E: { // STX abs
        const auto addr = Fetch16();
        if (Index8()) {
            bus_.Write(AbsAddress(addr), static_cast<uint8_t>(r_.x));
            cycles_ += 4;
        } else {
            bus_.Write(AbsAddress(addr), static_cast<uint8_t>(r_.x & 0xFF));
            bus_.Write(AbsAddress(static_cast<uint16_t>(addr + 1)), static_cast<uint8_t>(r_.x >> 8));
            cycles_ += 5;
        }
        break;
    }

    case 0x8C: { // STY abs
        const auto addr = Fetch16();
        if (Index8()) {
            bus_.Write(AbsAddress(addr), static_cast<uint8_t>(r_.y));
            cycles_ += 4;
        } else {
            bus_.Write(AbsAddress(addr), static_cast<uint8_t>(r_.y & 0xFF));
            bus_.Write(AbsAddress(static_cast<uint16_t>(addr + 1)), static_cast<uint8_t>(r_.y >> 8));
            cycles_ += 5;
        }
        break;
    }

    case 0x69: { // ADC #imm
        const auto value = ImmA();
        r_.a = AdcValue(r_.a, value, A8());
        cycles_ += A8() ? 2 : 3;
        break;
    }

    case 0xE9: { // SBC #imm
        const auto value = ImmA();
        r_.a = SbcValue(r_.a, value, A8());
        cycles_ += A8() ? 2 : 3;
        break;
    }

    case 0xC9: { // CMP #imm
        const auto value = ImmA();
        if (A8()) {
            const uint8_t lhs = static_cast<uint8_t>(r_.a);
            const uint8_t rhs = static_cast<uint8_t>(value);
            const uint8_t result = static_cast<uint8_t>(lhs - rhs);
            SetFlag(FlagC, lhs >= rhs);
            SetNZ8(result);
            cycles_ += 2;
        } else {
            const uint16_t lhs = r_.a;
            const uint16_t rhs = value;
            const uint16_t result = static_cast<uint16_t>(lhs - rhs);
            SetFlag(FlagC, lhs >= rhs);
            SetNZ16(result);
            cycles_ += 3;
        }
        break;
    }

    case 0xE0: { // CPX #imm
        const auto value = ImmIndex();
        if (Index8()) {
            const uint8_t lhs = static_cast<uint8_t>(r_.x);
            const uint8_t rhs = static_cast<uint8_t>(value);
            const uint8_t result = static_cast<uint8_t>(lhs - rhs);
            SetFlag(FlagC, lhs >= rhs);
            SetNZ8(result);
            cycles_ += 2;
        } else {
            const uint16_t lhs = r_.x;
            const uint16_t rhs = value;
            const uint16_t result = static_cast<uint16_t>(lhs - rhs);
            SetFlag(FlagC, lhs >= rhs);
            SetNZ16(result);
            cycles_ += 3;
        }
        break;
    }

    case 0xC0: { // CPY #imm
        const auto value = ImmIndex();
        if (Index8()) {
            const uint8_t lhs = static_cast<uint8_t>(r_.y);
            const uint8_t rhs = static_cast<uint8_t>(value);
            const uint8_t result = static_cast<uint8_t>(lhs - rhs);
            SetFlag(FlagC, lhs >= rhs);
            SetNZ8(result);
            cycles_ += 2;
        } else {
            const uint16_t lhs = r_.y;
            const uint16_t rhs = value;
            const uint16_t result = static_cast<uint16_t>(lhs - rhs);
            SetFlag(FlagC, lhs >= rhs);
            SetNZ16(result);
            cycles_ += 3;
        }
        break;
    }

    case 0x80: { // BRA
        const auto rel = static_cast<int8_t>(Fetch8());
        r_.pc = static_cast<uint16_t>(r_.pc + rel);
        cycles_ += 3;
        break;
    }

    case 0xF0: // BEQ
    case 0xD0: // BNE
    case 0x30: // BMI
    case 0x10: // BPL
    case 0xB0: // BCS
    case 0x90: { // BCC
        const auto rel = static_cast<int8_t>(Fetch8());

        bool condition = false;
        switch (opcode) {
        case 0xF0: condition = GetFlag(FlagZ); break;
        case 0xD0: condition = !GetFlag(FlagZ); break;
        case 0x30: condition = GetFlag(FlagN); break;
        case 0x10: condition = !GetFlag(FlagN); break;
        case 0xB0: condition = GetFlag(FlagC); break;
        case 0x90: condition = !GetFlag(FlagC); break;
        }

        cycles_ += 2;
        if (condition) {
            const auto oldPc = r_.pc;
            r_.pc = static_cast<uint16_t>(r_.pc + rel);
            cycles_ += 1;
            if (r_.e && ((oldPc & 0xFF00) != (r_.pc & 0xFF00))) {
                cycles_ += 1;
            }
        }
        break;
    }

    case 0x4C: { // JMP abs
        r_.pc = Fetch16();
        cycles_ += 3;
        break;
    }

    case 0x20: { // JSR abs
        const auto target = Fetch16();
        Push16(static_cast<uint16_t>(r_.pc - 1));
        r_.pc = target;
        cycles_ += 6;
        break;
    }

    case 0x60: // RTS
        r_.pc = static_cast<uint16_t>(Pop16() + 1);
        cycles_ += 6;
        break;

    case 0x40: { // RTI
        r_.p = Pop8();
        if (r_.e) {
            r_.p |= FlagM | FlagX;
        }
        r_.pc = Pop16();
        if (!r_.e) {
            r_.pb = Pop8();
        }
        cycles_ += r_.e ? 6 : 7;
        break;
    }

    case 0x9A: // TXS
        if (r_.e) {
            r_.s = static_cast<uint16_t>(0x0100 | (r_.x & 0x00FF));
        } else {
            r_.s = r_.x;
        }
        cycles_ += 2;
        break;

    case 0xBA: // TSX
        if (Index8()) {
            r_.x = r_.s & 0x00FF;
            SetNZ8(static_cast<uint8_t>(r_.x));
        } else {
            r_.x = r_.s;
            SetNZ16(r_.x);
        }
        cycles_ += 2;
        break;

    case 0xAA: // TAX
        if (Index8()) {
            r_.x = r_.a & 0x00FF;
            SetNZ8(static_cast<uint8_t>(r_.x));
        } else {
            r_.x = r_.a;
            SetNZ16(r_.x);
        }
        cycles_ += 2;
        break;

    case 0x8A: // TXA
        if (A8()) {
            r_.a = (r_.a & 0xFF00) | (r_.x & 0x00FF);
            SetNZ8(static_cast<uint8_t>(r_.a));
        } else {
            r_.a = r_.x;
            SetNZ16(r_.a);
        }
        cycles_ += 2;
        break;

    case 0xA8: // TAY
        if (Index8()) {
            r_.y = r_.a & 0x00FF;
            SetNZ8(static_cast<uint8_t>(r_.y));
        } else {
            r_.y = r_.a;
            SetNZ16(r_.y);
        }
        cycles_ += 2;
        break;

    case 0x98: // TYA
        if (A8()) {
            r_.a = (r_.a & 0xFF00) | (r_.y & 0x00FF);
            SetNZ8(static_cast<uint8_t>(r_.a));
        } else {
            r_.a = r_.y;
            SetNZ16(r_.a);
        }
        cycles_ += 2;
        break;

    case 0x48: // PHA
        if (A8()) {
            Push8(static_cast<uint8_t>(r_.a));
            cycles_ += 3;
        } else {
            Push16(r_.a);
            cycles_ += 4;
        }
        break;

    case 0x68: // PLA
        if (A8()) {
            r_.a = (r_.a & 0xFF00) | Pop8();
            SetNZ8(static_cast<uint8_t>(r_.a));
            cycles_ += 4;
        } else {
            r_.a = Pop16();
            SetNZ16(r_.a);
            cycles_ += 5;
        }
        break;

    case 0x08: // PHP
        Push8(static_cast<uint8_t>(r_.p | 0x10));
        cycles_ += 3;
        break;

    case 0x28: // PLP
        r_.p = Pop8();
        if (r_.e) {
            r_.p |= FlagM | FlagX;
        }
        cycles_ += 4;
        break;

    case 0xCA: // DEX
        if (Index8()) {
            r_.x = static_cast<uint8_t>(r_.x - 1);
            SetNZ8(static_cast<uint8_t>(r_.x));
        } else {
            r_.x = static_cast<uint16_t>(r_.x - 1);
            SetNZ16(r_.x);
        }
        cycles_ += 2;
        break;

    case 0xE8: // INX
        if (Index8()) {
            r_.x = static_cast<uint8_t>(r_.x + 1);
            SetNZ8(static_cast<uint8_t>(r_.x));
        } else {
            r_.x = static_cast<uint16_t>(r_.x + 1);
            SetNZ16(r_.x);
        }
        cycles_ += 2;
        break;

    case 0x88: // DEY
        if (Index8()) {
            r_.y = static_cast<uint8_t>(r_.y - 1);
            SetNZ8(static_cast<uint8_t>(r_.y));
        } else {
            r_.y = static_cast<uint16_t>(r_.y - 1);
            SetNZ16(r_.y);
        }
        cycles_ += 2;
        break;

    case 0xC8: // INY
        if (Index8()) {
            r_.y = static_cast<uint8_t>(r_.y + 1);
            SetNZ8(static_cast<uint8_t>(r_.y));
        } else {
            r_.y = static_cast<uint16_t>(r_.y + 1);
            SetNZ16(r_.y);
        }
        cycles_ += 2;
        break;

    default:
        // Reserved/not-yet-implemented opcode: emulate as NOP with conservative timing.
        cycles_ += 2;
        break;
    }

    return static_cast<uint32_t>(cycles_ - before);
}

uint8_t Cpu65816::Fetch8() {
    const uint32_t address = (static_cast<uint32_t>(r_.pb) << 16) | r_.pc;
    const auto value = bus_.Read(address);
    r_.pc = static_cast<uint16_t>(r_.pc + 1);
    return value;
}

uint16_t Cpu65816::Fetch16() {
    const auto lo = Fetch8();
    const auto hi = Fetch8();
    return static_cast<uint16_t>(lo | (hi << 8));
}

uint16_t Cpu65816::Read16(uint32_t address) {
    const auto lo = bus_.Read(address);
    const auto hi = bus_.Read((address & 0xFF0000) | ((address + 1) & 0xFFFF));
    return static_cast<uint16_t>(lo | (hi << 8));
}

uint32_t Cpu65816::AbsAddress(uint16_t address) const {
    return (static_cast<uint32_t>(r_.db) << 16) | address;
}

uint16_t Cpu65816::ImmA() {
    if (A8()) {
        return Fetch8();
    }
    return Fetch16();
}

uint16_t Cpu65816::ImmIndex() {
    if (Index8()) {
        return Fetch8();
    }
    return Fetch16();
}

uint16_t Cpu65816::ReadA(uint32_t address) {
    if (A8()) {
        return bus_.Read(address);
    }

    const auto lo = bus_.Read(address);
    const auto hi = bus_.Read((address & 0xFF0000) | ((address + 1) & 0xFFFF));
    return static_cast<uint16_t>(lo | (hi << 8));
}

void Cpu65816::WriteA(uint32_t address, uint16_t value) {
    if (A8()) {
        bus_.Write(address, static_cast<uint8_t>(value));
        return;
    }

    bus_.Write(address, static_cast<uint8_t>(value & 0x00FF));
    bus_.Write((address & 0xFF0000) | ((address + 1) & 0xFFFF), static_cast<uint8_t>(value >> 8));
}

void Cpu65816::Push8(uint8_t value) {
    if (r_.e) {
        bus_.Write(0x0100 | (r_.s & 0x00FF), value);
        r_.s = static_cast<uint16_t>(0x0100 | ((r_.s - 1) & 0x00FF));
        return;
    }

    bus_.Write(r_.s, value);
    r_.s = static_cast<uint16_t>(r_.s - 1);
}

void Cpu65816::Push16(uint16_t value) {
    Push8(static_cast<uint8_t>(value >> 8));
    Push8(static_cast<uint8_t>(value & 0x00FF));
}

uint8_t Cpu65816::Pop8() {
    if (r_.e) {
        r_.s = static_cast<uint16_t>(0x0100 | ((r_.s + 1) & 0x00FF));
        return bus_.Read(0x0100 | (r_.s & 0x00FF));
    }

    r_.s = static_cast<uint16_t>(r_.s + 1);
    return bus_.Read(r_.s);
}

uint16_t Cpu65816::Pop16() {
    const auto lo = Pop8();
    const auto hi = Pop8();
    return static_cast<uint16_t>(lo | (hi << 8));
}

void Cpu65816::SetFlag(uint8_t flag, bool set) {
    if (set) {
        r_.p |= flag;
    } else {
        r_.p &= static_cast<uint8_t>(~flag);
    }

    if (r_.e) {
        r_.p |= FlagM | FlagX;
    }
}

bool Cpu65816::GetFlag(uint8_t flag) const {
    return (r_.p & flag) != 0;
}

void Cpu65816::SetNZ8(uint8_t value) {
    SetFlag(FlagZ, value == 0);
    SetFlag(FlagN, (value & 0x80) != 0);
}

void Cpu65816::SetNZ16(uint16_t value) {
    SetFlag(FlagZ, value == 0);
    SetFlag(FlagN, (value & 0x8000) != 0);
}

void Cpu65816::SetE(bool enabled) {
    const bool oldE = r_.e;
    r_.e = enabled;

    if (enabled) {
        r_.p |= FlagM | FlagX;
        r_.x &= 0x00FF;
        r_.y &= 0x00FF;
        r_.s = static_cast<uint16_t>(0x0100 | (r_.s & 0x00FF));
    } else if (oldE && !enabled) {
        r_.s &= 0x01FF;
    }
}

void Cpu65816::EnterInterrupt(Interrupt type) {
    if (type != Interrupt::Reset) {
        if (!r_.e) {
            Push8(r_.pb);
        }

        const uint16_t pcToPush = (type == Interrupt::Brk || type == Interrupt::Cop) ? static_cast<uint16_t>(r_.pc + 1) : r_.pc;
        Push16(pcToPush);

        uint8_t pushedP = r_.p;
        if (type == Interrupt::Brk || type == Interrupt::Cop) {
            pushedP |= 0x10;
        } else {
            pushedP &= static_cast<uint8_t>(~0x10);
        }
        Push8(pushedP);
    }

    SetFlag(FlagI, true);
    SetFlag(FlagD, false);

    r_.pb = 0;
    r_.pc = Read16(VectorAddress(type));
}

uint16_t Cpu65816::VectorAddress(Interrupt type) const {
    if (r_.e) {
        switch (type) {
        case Interrupt::Cop: return 0xFFF4;
        case Interrupt::Abort: return 0xFFF8;
        case Interrupt::Nmi: return 0xFFFA;
        case Interrupt::Reset: return 0xFFFC;
        case Interrupt::Irq:
        case Interrupt::Brk:
        default:
            return 0xFFFE;
        }
    }

    switch (type) {
    case Interrupt::Cop: return 0xFFE4;
    case Interrupt::Brk: return 0xFFE6;
    case Interrupt::Abort: return 0xFFE8;
    case Interrupt::Nmi: return 0xFFEA;
    case Interrupt::Irq: return 0xFFEE;
    case Interrupt::Reset:
    default:
        return 0xFFFC;
    }
}

uint16_t Cpu65816::AdcValue(uint16_t lhs, uint16_t rhs, bool is8Bit) {
    const bool carryIn = GetFlag(FlagC);
    const bool decimal = GetFlag(FlagD);

    if (is8Bit) {
        const auto a = static_cast<uint8_t>(lhs);
        const auto b = static_cast<uint8_t>(rhs);

        const uint16_t binary = static_cast<uint16_t>(a) + static_cast<uint16_t>(b) + (carryIn ? 1u : 0u);
        const uint8_t result = decimal
            ? DecimalAdd8(a, b, carryIn, nullptr)
            : static_cast<uint8_t>(binary);

        SetFlag(FlagC, decimal ? (binary > 0x99) : (binary > 0xFF));
        SetFlag(FlagV, ((~(a ^ b) & (a ^ result)) & 0x80) != 0);
        SetNZ8(result);

        return static_cast<uint16_t>((lhs & 0xFF00) | result);
    }

    const uint16_t a = lhs;
    const uint16_t b = rhs;
    const uint32_t binary = static_cast<uint32_t>(a) + static_cast<uint32_t>(b) + (carryIn ? 1u : 0u);

    uint16_t result = static_cast<uint16_t>(binary);

    if (decimal) {
        bool c0 = carryIn;
        const uint8_t lo = DecimalAdd8(static_cast<uint8_t>(a & 0x00FF), static_cast<uint8_t>(b & 0x00FF), c0, &c0);
        const uint8_t hi = DecimalAdd8(static_cast<uint8_t>(a >> 8), static_cast<uint8_t>(b >> 8), c0, &c0);
        result = static_cast<uint16_t>(lo | (static_cast<uint16_t>(hi) << 8));
        SetFlag(FlagC, c0);
    } else {
        SetFlag(FlagC, binary > 0xFFFF);
    }

    SetFlag(FlagV, ((~(a ^ b) & (a ^ result)) & 0x8000) != 0);
    SetNZ16(result);
    return result;
}

uint16_t Cpu65816::SbcValue(uint16_t lhs, uint16_t rhs, bool is8Bit) {
    const bool carryIn = GetFlag(FlagC);
    const bool decimal = GetFlag(FlagD);

    if (is8Bit) {
        const auto a = static_cast<uint8_t>(lhs);
        const auto b = static_cast<uint8_t>(rhs);
        const int16_t binary = static_cast<int16_t>(a) - static_cast<int16_t>(b) - (carryIn ? 0 : 1);

        const uint8_t result = decimal
            ? DecimalSub8(a, b, carryIn, nullptr)
            : static_cast<uint8_t>(binary);

        SetFlag(FlagC, binary >= 0);
        SetFlag(FlagV, (((a ^ b) & (a ^ result)) & 0x80) != 0);
        SetNZ8(result);

        return static_cast<uint16_t>((lhs & 0xFF00) | result);
    }

    const uint16_t a = lhs;
    const uint16_t b = rhs;
    const int32_t binary = static_cast<int32_t>(a) - static_cast<int32_t>(b) - (carryIn ? 0 : 1);

    uint16_t result = static_cast<uint16_t>(binary);

    if (decimal) {
        bool c0 = carryIn;
        const uint8_t lo = DecimalSub8(static_cast<uint8_t>(a & 0x00FF), static_cast<uint8_t>(b & 0x00FF), c0, &c0);
        const uint8_t hi = DecimalSub8(static_cast<uint8_t>(a >> 8), static_cast<uint8_t>(b >> 8), c0, &c0);
        result = static_cast<uint16_t>(lo | (static_cast<uint16_t>(hi) << 8));
        SetFlag(FlagC, c0);
    } else {
        SetFlag(FlagC, binary >= 0);
    }

    SetFlag(FlagV, (((a ^ b) & (a ^ result)) & 0x8000) != 0);
    SetNZ16(result);
    return result;
}

void Cpu65816::OpRep() {
    const auto mask = Fetch8();
    r_.p = static_cast<uint8_t>(r_.p & ~mask);
    if (r_.e) {
        r_.p |= FlagM | FlagX;
    }

    if (Index8()) {
        r_.x &= 0x00FF;
        r_.y &= 0x00FF;
    }
}

void Cpu65816::OpSep() {
    const auto mask = Fetch8();
    r_.p = static_cast<uint8_t>(r_.p | mask);
    if (r_.e) {
        r_.p |= FlagM | FlagX;
    }

    if (Index8()) {
        r_.x &= 0x00FF;
        r_.y &= 0x00FF;
    }
}

bool Cpu65816::A8() const {
    return r_.e || GetFlag(FlagM);
}

bool Cpu65816::Index8() const {
    return r_.e || GetFlag(FlagX);
}

} // namespace snes::core
