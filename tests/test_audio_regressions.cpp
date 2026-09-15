// snes emulator
// tests/test_audio_regressions.cpp
// Regression coverage for DSP and SMP audio behavior.

#include "snes/core/Dsp.hpp"
#include "snes/core/Spc700.hpp"
#include "snes/core/Smp.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

using snes::core::Dsp;
using snes::core::Spc700;
using snes::core::Smp;

namespace {
int failures = 0;
void Expect(bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

struct BusEvent {
    enum class Kind : uint8_t { Idle, Read, Write };
    Kind kind{};
    uint16_t address = 0;
    uint8_t data = 0;

    bool operator==(const BusEvent&) const = default;
};

class TraceSpc final : public Spc700 {
public:
    std::array<uint8_t, 65536> memory{};
    std::vector<BusEvent> events;

    void Idle() override {
        ++cycles_;
        events.push_back({BusEvent::Kind::Idle, 0, 0});
    }

    uint8_t Read(uint16_t address) override {
        ++cycles_;
        const uint8_t value = memory[address];
        events.push_back({BusEvent::Kind::Read, address, value});
        return value;
    }

    void Write(uint16_t address, uint8_t data) override {
        ++cycles_;
        memory[address] = data;
        events.push_back({BusEvent::Kind::Write, address, data});
    }

    void Prepare(uint8_t opcode, bool alternatePath) {
        Power();
        cycles_ = 0;
        events.clear();
        for (size_t i = 0; i < memory.size(); ++i) {
            memory[i] = static_cast<uint8_t>((i * 37u + (alternatePath ? 0x51u : 0x13u)) & 0xffu);
        }

        r.pc = 0x0200;
        r.a = 0x5a;
        r.x = 0x07; // Keep DIV well-defined and exercise indexed modes.
        r.y = 0x39;
        r.s = 0xd0;
        r.p.Unpack(alternatePath ? 0xe3 : 0x00);
        r.wait = false;
        r.stop = false;

        memory[0x0200] = opcode;
        memory[0x0201] = 0x40;
        memory[0x0202] = 0x20;
        memory[0x0203] = 0xfe;

        ConfigureBranchPath(opcode, alternatePath);
    }

private:
    uint16_t DirectAddress(uint8_t address) const {
        return static_cast<uint16_t>(r.p.p ? 0x0100 : 0x0000) | address;
    }

    void SetDirect(uint8_t address, uint8_t value) {
        memory[DirectAddress(address)] = value;
    }

    void ConfigureBranchPath(uint8_t opcode, bool take) {
        switch (opcode) {
        case 0x10: r.p.n = !take; return; // BPL
        case 0x30: r.p.n = take;  return; // BMI
        case 0x50: r.p.v = !take; return; // BVC
        case 0x70: r.p.v = take;  return; // BVS
        case 0x90: r.p.c = !take; return; // BCC
        case 0xb0: r.p.c = take;  return; // BCS
        case 0xd0: r.p.z = !take; return; // BNE
        case 0xf0: r.p.z = take;  return; // BEQ
        case 0x2e: // CBNE dp,rel
            SetDirect(0x40, take ? static_cast<uint8_t>(r.a ^ 0xff) : r.a);
            return;
        case 0xde: // CBNE dp+X,rel
            SetDirect(static_cast<uint8_t>(0x40 + r.x),
                      take ? static_cast<uint8_t>(r.a ^ 0xff) : r.a);
            return;
        case 0x6e: // DBNZ dp,rel: branch after decrement.
            SetDirect(0x40, take ? 2 : 1);
            return;
        case 0xfe: // DBNZ Y,rel: branch after decrement.
            r.y = take ? 2 : 1;
            return;
        default:
            break;
        }

        // BBS/BBC: x3 opcodes use the high opcode bits as the direct-page bit.
        if ((opcode & 0x0f) == 0x03) {
            const uint8_t bit = static_cast<uint8_t>(opcode >> 5);
            const bool branchOnSet = (opcode & 0x10) == 0;
            const bool bitSet = take ? branchOnSet : !branchOnSet;
            SetDirect(0x40, bitSet ? static_cast<uint8_t>(1u << bit) : 0);
        }
    }
};

bool SameRegisters(const Spc700::Registers& lhs, const Spc700::Registers& rhs) {
    return lhs.pc == rhs.pc && lhs.a == rhs.a && lhs.y == rhs.y && lhs.x == rhs.x &&
           lhs.s == rhs.s && lhs.p.Pack() == rhs.p.Pack() &&
           lhs.wait == rhs.wait && lhs.stop == rhs.stop;
}

void ExpectOpcode(bool condition, unsigned opcode, bool alternatePath, const char* what) {
    if (condition) return;
    std::fprintf(stderr, "FAIL: opcode %02X path %u %s\n",
                 opcode, alternatePath ? 1u : 0u, what);
    ++failures;
}

void SegmentedInstructionEquivalence() {
    for (unsigned opcode = 0; opcode < 256; ++opcode) {
        for (bool alternatePath : {false, true}) {
            TraceSpc full;
            TraceSpc segmented;
            full.Prepare(static_cast<uint8_t>(opcode), alternatePath);
            segmented.Prepare(static_cast<uint8_t>(opcode), alternatePath);

            full.Step();

            unsigned cycles = 0;
            do {
                segmented.StepCycle();
                ++cycles;
            } while (segmented.InstructionInProgress() && cycles < 64);

            ExpectOpcode(cycles < 64, opcode, alternatePath, "did not finish within 64 cycles");
            ExpectOpcode(full.CycleCount() == segmented.CycleCount(), opcode, alternatePath,
                         "changed cycle count when segmented");
            ExpectOpcode(full.events == segmented.events, opcode, alternatePath,
                         "changed bus ordering when segmented");
            ExpectOpcode(SameRegisters(full.r, segmented.r), opcode, alternatePath,
                         "changed register result when segmented");
            ExpectOpcode(full.memory == segmented.memory, opcode, alternatePath,
                         "changed memory result when segmented");
        }
    }
}

void PrepareSmpInstruction(Smp& smp, std::initializer_list<uint8_t> bytes) {
    smp.Power();
    smp.IoState().iplRomEnable = false;
    smp.r.pc = 0x0200;
    uint16_t address = 0x0200;
    for (uint8_t byte : bytes) smp.Ram()[address++] = byte;
}

void DeadlineSensitiveIo() {
    // MOV A,$F4: the external CPU may change its input latch after the operand
    // fetch. The SPC read must observe the value present on the actual read cycle.
    {
        Smp smp;
        PrepareSmpInstruction(smp, {0xe4, 0xf4});
        smp.PortWrite(0, 0x11);
        smp.RunUntil(2);
        Expect(smp.CycleCount() == 2, "RunUntil stops exactly before a port read");
        Expect(smp.r.a == 0x00, "port read has not happened before its cycle");
        smp.PortWrite(0, 0x77);
        smp.RunUntil(3);
        Expect(smp.r.a == 0x77, "port read samples the latch on its bus cycle");
    }

    // MOV $F4,A performs a dummy read before the write. CPU-visible output must
    // stay unchanged until the final write cycle.
    {
        Smp smp;
        PrepareSmpInstruction(smp, {0xc4, 0xf4});
        smp.r.a = 0x5a;
        smp.RunUntil(3);
        Expect(smp.CycleCount() == 3, "RunUntil stops exactly before a port write");
        Expect(smp.PortRead(0) == 0x00, "port write is deferred until its bus cycle");
        smp.RunUntil(4);
        Expect(smp.PortRead(0) == 0x5a, "port write becomes visible on its bus cycle");
    }

    // MOV $F1,#$10 clears the CPU-to-SMP input pair only on its store cycle.
    {
        Smp smp;
        PrepareSmpInstruction(smp, {0x8f, 0x10, 0xf1});
        smp.PortWrite(0, 0xaa);
        smp.PortWrite(1, 0xbb);
        smp.RunUntil(4);
        Expect(smp.ApuInput(0) == 0xaa && smp.ApuInput(1) == 0xbb,
               "CONTROL port clear is deferred until the store cycle");
        smp.RunUntil(5);
        Expect(smp.ApuInput(0) == 0x00 && smp.ApuInput(1) == 0x00,
               "CONTROL port clear occurs on the store cycle");
    }

    // MOV A,$FD reads and clears timer 0 output atomically on its read cycle.
    {
        Smp smp;
        PrepareSmpInstruction(smp, {0xe4, 0xfd});
        smp.GetTimer0().enable = false;
        smp.GetTimer0().stage3 = 7;
        smp.RunUntil(2);
        Expect(smp.GetTimer0().stage3 == 7, "timer output is not cleared before its read cycle");
        smp.RunUntil(3);
        Expect(smp.r.a == 7 && smp.GetTimer0().stage3 == 0,
               "timer read returns then clears output on the read cycle");
    }

    // A pending bus operation belongs to the old processor state and must vanish
    // if a reset occurs between cycles.
    {
        Smp smp;
        PrepareSmpInstruction(smp, {0xc4, 0xf4});
        smp.r.a = 0x99;
        smp.RunUntil(3);
        Expect(smp.InstructionInProgress(), "port store is pending before reset");
        smp.Power();
        Expect(!smp.InstructionInProgress() && smp.CycleCount() == 0,
               "power reset cancels a partially executed instruction");
        smp.IoState().iplRomEnable = false;
        smp.r.pc = 0x0300;
        smp.Ram()[0x0300] = 0x00;
        smp.RunUntil(2);
        Expect(smp.PortRead(0) == 0x00, "reset instruction cannot resume an old pending port write");
    }
}

void ExactDeadlineClocking() {
    for (uint8_t opcode : {uint8_t{0xef}, uint8_t{0xff}}) {
        Smp smp;
        PrepareSmpInstruction(smp, {opcode});
        smp.RunUntil(3);
        Expect(smp.CycleCount() == 3, "sleep/stop instruction ends on its exact third cycle");
        Expect(opcode == 0xef ? smp.r.wait : smp.r.stop,
               "sleep/stop state is entered at instruction completion");
        const uint16_t haltedPc = smp.r.pc;
        smp.RunUntil(67);
        Expect(smp.CycleCount() == 67, "halted sound CPU advances exactly to scheduler deadline");
        Expect(smp.r.pc == haltedPc, "halted sound CPU does not fetch another opcode");
    }

    Smp smp;
    Dsp dsp;
    smp.SetDsp(dsp);
    dsp.SetRam(smp.Ram());
    PrepareSmpInstruction(smp, {0x2f, 0xfe});
    smp.SetDsp(dsp);
    dsp.SetRam(smp.Ram());
    dsp.Power();
    smp.RunUntil(31);
    Expect(smp.CycleCount() == 31 && dsp.Phase() == 31,
           "DSP receives exactly one phase per SMP cycle before deadline");
    smp.RunUntil(32);
    Expect(smp.CycleCount() == 32 && dsp.Phase() == 0,
           "DSP phase wraps only when the deadline includes that cycle");
}

void CrossThreadCoroutineHandoff() {
    auto smp = std::make_unique<Smp>();
    PrepareSmpInstruction(*smp, {0xc4, 0xf4});
    smp->r.a = 0x6d;
    smp->RunUntil(3);
    Expect(smp->InstructionInProgress(),
           "cross-thread test suspends with the port store still pending");

    bool resumedCorrectly = false;
    std::thread secondThread(
        [cpu = std::move(smp), &resumedCorrectly]() mutable {
            cpu->RunUntil(4);
            resumedCorrectly = cpu->CycleCount() == 4 && cpu->PortRead(0) == 0x6d &&
                               !cpu->InstructionInProgress();
            // Destruction on this thread also returns the persistent executor
            // frame to the pool owned by this CPU instance.
            cpu.reset();
        });
    secondThread.join();

    Expect(resumedCorrectly,
           "suspended SPC execution can resume and be destroyed on another thread");
}

void HaltedSoundCpu() {
    for (uint8_t opcode : {0xef, 0xff}) {
        Smp smp;
        Dsp dsp;
        smp.SetDsp(dsp);
        dsp.SetRam(smp.Ram());
        std::array<int16_t, 128> output{};
        dsp.SetOutput(output.data(), 64);
        smp.r.pc = 0x200;
        smp.Ram()[0x200] = opcode;
        smp.Write(0xfc, 1);
        smp.Write(0xf1, 4);
        smp.RunUntil(128);
        Expect(dsp.SamplesWritten() == 4, "DSP keeps producing samples while the sound CPU is halted");
        Expect(smp.GetTimer2().stage3 == 8, "timer 2 keeps ticking while the sound CPU is halted");
        const auto pc = smp.r.pc;
        smp.RunUntil(1024);
        Expect(dsp.SamplesWritten() == 32, "halted audio remains continuous across scheduler calls");
        Expect(smp.r.pc == pc, "a halted sound CPU does not execute more instructions");
    }
}

void PowerClearsKeyOn() {
    Dsp reused;
    std::array<uint8_t, 65536> ram{};
    reused.SetRam(ram.data());
    reused.Write(Dsp::kKon, 1);
    reused.RunSample();
    reused.RunSample(); // Latch KON on the second sample.
    reused.Power();
    Dsp fresh;
    fresh.SetRam(ram.data());
    for (Dsp* dsp : {&reused, &fresh}) {
        dsp->Write(Dsp::kFlg, 0x20);
        dsp->Write(Dsp::kKon, 1);
    }
    for (int i = 0; i < 8; ++i) {
        reused.RunSample();
        fresh.RunSample();
        Expect(reused.GetVoice(0).konDelay == fresh.GetVoice(0).konDelay,
               "power reset accepts the same key-on as a fresh DSP");
    }
}
}

int main() {
    SegmentedInstructionEquivalence();
    DeadlineSensitiveIo();
    ExactDeadlineClocking();
    CrossThreadCoroutineHandoff();
    HaltedSoundCpu();
    PowerClearsKeyOn();
    return failures ? 1 : 0;
}
