// snes emulator
// core/include/snes/core/Cpu65816.hpp
// 65816 CPU bus and instruction interface.

#pragma once

#include <cstdint>

namespace snes::core {

class ICpuBus {
public:
    virtual ~ICpuBus() = default;
    virtual uint8_t Read(uint32_t address) = 0;
    virtual void Write(uint32_t address, uint8_t value) = 0;

    // Speed lookup: returns the number of master clocks for this address.
    // Default implementation returns 6 (fast ROM).  Override for real mapping.
    virtual uint8_t Speed(uint32_t /*address*/) const { return 6; }
};

class Cpu65816 {
public:
    struct Registers {
        uint16_t a = 0;
        uint16_t x = 0;
        uint16_t y = 0;
        uint16_t s = 0x01FF;
        uint16_t d = 0;
        uint16_t pc = 0;
        uint8_t db = 0;
        uint8_t pb = 0;
        uint8_t p = 0x34;
        bool e = true;
    };

    enum class Interrupt {
        Reset,
        Nmi,
        Irq,
        Brk,
        Cop,
        Abort
    };

    explicit Cpu65816(ICpuBus& bus);

    void Reset();
    void RequestNmi();
    void RequestIrq();
    void RequestAbort();

    uint32_t Step();

    const Registers& GetRegisters() const noexcept;
    uint64_t Cycles() const noexcept;

private:
    static constexpr uint8_t FlagN = 0x80;
    static constexpr uint8_t FlagV = 0x40;
    static constexpr uint8_t FlagM = 0x20;
    static constexpr uint8_t FlagX = 0x10;
    static constexpr uint8_t FlagD = 0x08;
    static constexpr uint8_t FlagI = 0x04;
    static constexpr uint8_t FlagZ = 0x02;
    static constexpr uint8_t FlagC = 0x01;

    uint8_t Fetch8();
    uint16_t Fetch16();
    uint16_t Read16(uint32_t address);
    uint32_t AbsAddress(uint16_t address) const;

    uint16_t ImmA();
    uint16_t ImmIndex();

    uint16_t ReadA(uint32_t address);
    void WriteA(uint32_t address, uint16_t value);

    void Push8(uint8_t value);
    void Push16(uint16_t value);
    uint8_t Pop8();
    uint16_t Pop16();

    void SetFlag(uint8_t flag, bool set);
    bool GetFlag(uint8_t flag) const;
    void SetNZ8(uint8_t value);
    void SetNZ16(uint16_t value);

    void SetE(bool enabled);
    void EnterInterrupt(Interrupt type);
    uint16_t VectorAddress(Interrupt type) const;

    uint16_t AdcValue(uint16_t lhs, uint16_t rhs, bool is8Bit);
    uint16_t SbcValue(uint16_t lhs, uint16_t rhs, bool is8Bit);

    void OpRep();
    void OpSep();

    bool A8() const;
    bool Index8() const;

    ICpuBus& bus_;
    Registers r_{};
    uint64_t cycles_ = 0;

    bool pendingNmi_ = false;
    bool pendingIrq_ = false;
    bool pendingAbort_ = false;
};

} // namespace snes::core
