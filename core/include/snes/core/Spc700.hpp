// snes emulator
// core/include/snes/core/Spc700.hpp
// SPC700 processor interface.

// Spc700.hpp — Sony SPC700 Audio Processor Core
//
// Pure processor implementation with virtual bus interface.
// The SNES SMP wrapper (future) will inherit this and provide the bus.
//
// Reference: bsnes processor/spc700/spc700.hpp

#pragma once

#include <array>
#include <cstddef>
#include <coroutine>
#include <cstdint>
#include <exception>
#include <memory_resource>
#include <utility>

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
    void StepCycle();         // Execute one bus tick; device waits may retain a cycle
    uint64_t CycleCount() const { return cycles_; }
    [[nodiscard]] bool InstructionInProgress() const noexcept;

protected:
    uint64_t cycles_ = 0;    // Total cycles consumed

    void* AllocateCoroutineFrame(std::size_t size);
    static void ReleaseCoroutineFrame(void* frame) noexcept;

    class Routine {
    public:
        struct promise_type;
        using Handle = std::coroutine_handle<promise_type>;

        struct promise_type {
            std::coroutine_handle<> continuation{};
            std::exception_ptr exception{};

            template <typename... Args>
            static void* operator new(std::size_t size, Spc700& cpu, Args&&...) {
                return cpu.AllocateCoroutineFrame(size);
            }

            static void operator delete(void* ptr, std::size_t size) noexcept {
                (void)size;
                ReleaseCoroutineFrame(ptr);
            }

            Routine get_return_object() noexcept { return Routine(Handle::from_promise(*this)); }
            std::suspend_always initial_suspend() const noexcept { return {}; }

            struct FinalAwaiter {
                bool await_ready() const noexcept { return false; }
                std::coroutine_handle<> await_suspend(Handle handle) const noexcept {
                    auto continuation = handle.promise().continuation;
                    return continuation ? continuation : std::noop_coroutine();
                }
                void await_resume() const noexcept {}
            };

            FinalAwaiter final_suspend() const noexcept { return {}; }
            void return_void() const noexcept {}
            void unhandled_exception() noexcept { exception = std::current_exception(); }
        };

        Routine() = default;
        explicit Routine(Handle handle) noexcept : handle_(handle) {}
        Routine(const Routine&) = delete;
        Routine& operator=(const Routine&) = delete;
        Routine(Routine&& other) noexcept : handle_(std::exchange(other.handle_, {})) {}
        Routine& operator=(Routine&& other) noexcept {
            if (this == &other) return *this;
            Reset();
            handle_ = std::exchange(other.handle_, {});
            return *this;
        }
        ~Routine() { Reset(); }

        struct Awaiter {
            Handle handle;
            bool await_ready() const noexcept { return !handle || handle.done(); }
            std::coroutine_handle<> await_suspend(std::coroutine_handle<> continuation) const noexcept {
                handle.promise().continuation = continuation;
                return handle;
            }
            void await_resume() const {
                if (handle && handle.promise().exception) std::rethrow_exception(handle.promise().exception);
            }
        };

        Awaiter operator co_await() const noexcept { return Awaiter{handle_}; }
        [[nodiscard]] Handle GetHandle() const noexcept { return handle_; }
        [[nodiscard]] bool Done() const noexcept { return handle_ && handle_.done(); }
        explicit operator bool() const noexcept { return static_cast<bool>(handle_); }
        void RethrowIfFailed() const {
            if (handle_ && handle_.promise().exception) std::rethrow_exception(handle_.promise().exception);
        }
        void Reset() noexcept {
            if (handle_) handle_.destroy();
            handle_ = {};
        }

    private:
        Handle handle_{};
    };

    enum class PendingCycleKind : uint8_t { None, Idle, Read, Write };
    struct PendingCycle {
        PendingCycleKind kind = PendingCycleKind::None;
        uint16_t address = 0;
        uint8_t data = 0;
        uint8_t* readResult = nullptr;
    };

    // Return false while the device is stretching this bus operation. The
    // coroutine resumes only after the read, write or idle operation completes.
    virtual bool ExecuteBusCycle(const PendingCycle& cycle);

    struct IdleCycle {
        Spc700* cpu;
        bool await_ready() const noexcept { return false; }
        void await_suspend(std::coroutine_handle<> handle) const noexcept;
        void await_resume() const noexcept {}
    };

    struct ReadCycle {
        Spc700* cpu;
        uint16_t address;
        uint8_t result = 0;
        bool await_ready() const noexcept { return false; }
        void await_suspend(std::coroutine_handle<> handle) noexcept;
        uint8_t await_resume() const noexcept { return result; }
    };

    struct WriteCycle {
        Spc700* cpu;
        uint16_t address;
        uint8_t data;
        bool await_ready() const noexcept { return false; }
        void await_suspend(std::coroutine_handle<> handle) const noexcept;
        void await_resume() const noexcept {}
    };

    struct InstructionBoundary {
        Spc700* cpu;
        bool await_ready() const noexcept { return false; }
        void await_suspend(std::coroutine_handle<> handle) const noexcept;
        void await_resume() const noexcept {}
    };

    struct alignas(std::max_align_t) CoroutineFrameHeader {
        std::pmr::memory_resource* resource = nullptr;
        std::size_t allocationSize = 0;
    };

    // The pool belongs to this CPU. It is deliberately declared before the
    // executor so reverse-order destruction releases every live frame first.
    std::pmr::unsynchronized_pool_resource framePool_{
        std::pmr::pool_options{64, 8192}};
    Routine executor_{};
    std::coroutine_handle<> activeCoroutine_{};
    PendingCycle pendingCycle_{};
    bool atInstructionBoundary_ = true;

    void BeginInstruction();
    void ResumeToCycleBoundary();
    void CheckExecutor();
    Routine ExecuteInstructions();

    // Memory access helpers
    IdleCycle  WaitCycle() noexcept { return IdleCycle{this}; }
    ReadCycle  Fetch();                              // read(PC++)
    ReadCycle  Load(uint8_t addr);                   // read(dp | addr)
    WriteCycle Store(uint8_t addr, uint8_t data);    // write(dp | addr, data)
    ReadCycle  Pull();                               // read(0x100 | ++S)
    WriteCycle Push(uint8_t data);                    // write(0x100 | S--, data)
    ReadCycle  ReadCycleAt(uint16_t address) noexcept { return ReadCycle{this, address}; }
    WriteCycle WriteCycleAt(uint16_t address, uint8_t data) noexcept { return WriteCycle{this, address, data}; }

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
    Routine InstrImmediateRead(AlgOp op, uint8_t& target);
    Routine InstrDirectRead(AlgOp op, uint8_t& target);
    Routine InstrDirectModify(ModOp op);
    Routine InstrDirectWrite(uint8_t data);
    Routine InstrDirectIndexedRead(AlgOp op, uint8_t& target, uint8_t index);
    Routine InstrDirectIndexedModify(ModOp op, uint8_t index);
    Routine InstrDirectIndexedWrite(uint8_t data, uint8_t index);
    Routine InstrAbsoluteRead(AlgOp op, uint8_t& target);
    Routine InstrAbsoluteModify(ModOp op);
    Routine InstrAbsoluteWrite(uint8_t data);
    Routine InstrAbsoluteIndexedRead(AlgOp op, uint8_t index);
    Routine InstrAbsoluteIndexedWrite(uint8_t index);
    Routine InstrIndexedIndirectRead(AlgOp op, uint8_t index);
    Routine InstrIndexedIndirectWrite(uint8_t data, uint8_t index);
    Routine InstrIndirectIndexedRead(AlgOp op, uint8_t index);
    Routine InstrIndirectIndexedWrite(uint8_t data, uint8_t index);
    Routine InstrIndirectXRead(AlgOp op);
    Routine InstrIndirectXWrite(uint8_t data);
    Routine InstrIndirectXIncrementRead(uint8_t& target);
    Routine InstrIndirectXIncrementWrite(uint8_t data);
    Routine InstrIndirectXCompareIndirectY(AlgOp op);
    Routine InstrIndirectXWriteIndirectY(AlgOp op);

    // Direct-Direct, Direct-Immediate
    Routine InstrDirectDirectCompare(AlgOp op);
    Routine InstrDirectDirectModify(AlgOp op);
    Routine InstrDirectDirectWrite();
    Routine InstrDirectImmediateCompare(AlgOp op);
    Routine InstrDirectImmediateModify(AlgOp op);
    Routine InstrDirectImmediateWrite();

    // 16-bit word operations
    Routine InstrDirectCompareWord(AlgOp16 op);
    Routine InstrDirectReadWord(AlgOp16 op);
    Routine InstrDirectModifyWord(int16_t adjust);
    Routine InstrDirectWriteWord();

    // Bit operations
    Routine InstrAbsoluteBitModify(uint8_t mode);
    Routine InstrAbsoluteBitSet(uint8_t bit, bool value);
    Routine InstrTestSetBitsAbsolute(bool set);

    // Branches
    Routine InstrBranch(bool take);
    Routine InstrBranchBit(uint8_t bit, bool match);
    Routine InstrBranchNotDirect();
    Routine InstrBranchNotDirectIndexed(uint8_t index);
    Routine InstrBranchNotDirectDecrement();
    Routine InstrBranchNotYDecrement();

    // Flow control
    Routine InstrCallAbsolute();
    Routine InstrCallPage();
    Routine InstrCallTable(uint8_t vector);
    Routine InstrJumpAbsolute();
    Routine InstrJumpIndirectX();
    Routine InstrReturnSubroutine();
    Routine InstrReturnInterrupt();
    Routine InstrBreak();

    // Register transfer
    Routine InstrTransfer(uint8_t from, uint8_t& to);

    // Push/Pull
    Routine InstrPush(uint8_t data);
    Routine InstrPull(uint8_t& target);
    Routine InstrPushP();
    Routine InstrPullP();

    // Flag manipulation
    Routine InstrFlagSet(bool& flag, bool value);
    Routine InstrOverflowClear();
    Routine InstrComplementCarry();

    // Implied register ops
    Routine InstrImpliedModify(ModOp op, uint8_t& target);

    // Special instructions
    Routine InstrMultiply();
    Routine InstrDivide();
    Routine InstrDecimalAdjustAdd();
    Routine InstrDecimalAdjustSub();
    Routine InstrExchangeNibble();
    Routine InstrNoOperation();
    Routine InstrSleep();
    Routine InstrStop();
};

} // namespace snes::core
