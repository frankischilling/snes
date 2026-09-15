#include "snes/core/Dsp.hpp"
#include "snes/core/Smp.hpp"

#include <array>
#include <cstdio>
#include <stdexcept>

using namespace snes::core;

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void Program(Smp& smp, std::initializer_list<uint8_t> bytes) {
    smp.Power();
    smp.IoState().iplRomEnable = false;
    smp.r.pc = 0x200;
    unsigned address = 0x200;
    for (uint8_t value : bytes) smp.Ram()[address++] = value;
}

unsigned TimerPhase(const Smp& smp) {
    return smp.GetTimer0().stage0 + 128u * smp.GetTimer0().stage1;
}

void AddressClasses() {
    constexpr std::array<unsigned, 4> duration{1, 2, 5, 10};
    constexpr std::array<unsigned, 4> ticks{2, 4, 8, 16};
    for (unsigned internal = 0; internal < 4; ++internal) {
        for (unsigned external = 0; external < 4; ++external) {
            Smp smp;
            Dsp dsp;
            dsp.SetRam(smp.Ram());
            smp.SetDsp(dsp);
            smp.Write(0xf0, uint8_t(0x0a | (external << 4) | (internal << 6)));
            unsigned clocks = 1;
            unsigned timerTicks = 2;
            auto check = [&](unsigned setting) {
                clocks += duration[setting];
                timerTicks += ticks[setting];
                Check(smp.CycleCount() == clocks, "TEST follows the selected deterministic access model");
                Check(TimerPhase(smp) == timerTicks % 256,
                      "Timer divider follows TEST timer ticks independently of stretched CPU clocks");
                Check(dsp.Phase() == clocks % 32, "DSP phases follow elapsed clocks at every TEST setting");
            };
            smp.Idle(); check(internal);
            smp.Read(0xf2); check(internal);
            smp.Write(0xf8, 0x55); check(internal);
            Check(smp.Ram()[0xf8] == 0x55, "Stretched I/O writes still reach backing RAM");
            Check(smp.Read(0xffc0) == 0xcd, "Enabled IPL overlay reads its boot opcode"); check(internal);
            smp.Write(0xffc0, 0x42); check(internal);
            Check(smp.Ram()[0xffc0] == 0x42, "Enabled IPL overlay permits underlying RAM writes");
            smp.Read(0x400); check(external);
            smp.Write(0x400, 0x71); check(external);
            Check(smp.Ram()[0x400] == 0x71, "External RAM writes survive stretched accesses");
            smp.Write(0xf1, 0); check(internal);
            Check(smp.Read(0xffc0) == 0x42, "Disabled IPL overlay exposes underlying RAM"); check(external);
            smp.Write(0xffc0, 0x93); check(external);
            smp.r.p.p = true;
            smp.Write(0xf0, 0); check(internal);
            smp.Idle(); check(internal);
            Check(smp.IoState().internalWaitStates == internal &&
                  smp.IoState().externalWaitStates == external,
                  "The P flag blocks TEST changes without changing current access timing");
        }
    }
}

void PortReadHold() {
    constexpr std::array<unsigned, 4> duration{1, 2, 5, 10};
    for (unsigned setting = 1; setting < 4; ++setting) {
        for (uint8_t port = 0; port < 4; ++port) {
            Smp smp;
            Program(smp, {0xe4, uint8_t(0xf4 + port)}); // MOV A,port
            smp.IoState().internalWaitStates = uint8_t(setting);
            smp.PortWrite(port, 0x11);
            const auto sampledAt = 2 + (duration[setting] + 1) / 2;
            const auto finishedAt = 2 + duration[setting];
            smp.RunUntil(sampledAt);
            Check(smp.CycleCount() == sampledAt && smp.r.a == 0,
                  "A slow port read samples its input before completing the instruction");
            smp.PortWrite(port, 0x77);
            smp.RunUntil(finishedAt);
            Check(smp.r.a == 0x11 && !smp.InstructionInProgress(),
                  "A communication read holds its sampled value through the remaining wait");
            Check(smp.ApuInput(port) == 0x77, "Held read does not erase a later console write");
        }
    }
}

void StoreDeadlines() {
    constexpr std::array<unsigned, 4> duration{1, 2, 5, 10};
    for (unsigned internal = 0; internal < 4; ++internal) {
        for (unsigned external = 0; external < 4; ++external) {
            Smp smp;
            Program(smp, {0x8f, 0xa5, 0xf4}); // MOV port,#$A5
            smp.IoState().internalWaitStates = uint8_t(internal);
            smp.IoState().externalWaitStates = uint8_t(external);
            Dsp dsp;
            dsp.SetRam(smp.Ram()); smp.SetDsp(dsp);
            const unsigned finishedAt = 3 * duration[external] + 2 * duration[internal];
            for (unsigned clock = 1; clock <= finishedAt; ++clock) {
                smp.RunUntil(clock);
                Check(smp.CycleCount() == clock && dsp.Phase() == clock % 32,
                      "A stretched store never overshoots a scheduler deadline or loses DSP clocks");
                Check(smp.PortRead(0) == (clock == finishedAt ? 0xa5 : 0),
                      "A slow port store becomes visible only on its final clock");
            }
            Check(!smp.InstructionInProgress(), "Stretched instruction ends exactly on its final clock");
            Smp whole;
            Program(whole, {0x8f, 0xa5, 0xf4});
            whole.IoState().internalWaitStates = uint8_t(internal);
            whole.IoState().externalWaitStates = uint8_t(external);
            whole.Step();
            Check(whole.CycleCount() == finishedAt && whole.PortRead(0) == 0xa5 &&
                  TimerPhase(whole) == TimerPhase(smp),
                  "Whole-instruction stepping preserves the same waits and timer phase");
        }
    }
}

void RamAndTestSampling() {
    Smp smp;
    Program(smp, {0xe5, 0x00, 0x04}); // MOV A,$0400
    smp.IoState().externalWaitStates = 2;
    smp.Ram()[0x400] = 0x11;
    smp.RunUntil(19);
    smp.Ram()[0x400] = 0x77;
    smp.RunUntil(20);
    Check(smp.r.a == 0x77, "Ordinary RAM reads sample at the end of the wait, without port hold behavior");

    Program(smp, {0x8f, 0x0a, 0xf0, 0x00}); // MOV TEST,#$0A; NOP
    smp.IoState().internalWaitStates = 2;
    smp.RunUntil(12);
    Check(smp.IoState().internalWaitStates == 2,
          "A TEST write retains the old timing until its store completes");
    smp.RunUntil(13);
    Check(smp.IoState().internalWaitStates == 0 && !smp.InstructionInProgress(),
          "A TEST write applies the new divider after finishing under the old one");
    smp.Step();
    Check(smp.CycleCount() == 15, "The following instruction uses the new TEST divider");
}

void ResetDuringWait() {
    Smp smp;
    Program(smp, {0x8f, 0x99, 0xf4});
    smp.IoState().internalWaitStates = 3;
    smp.RunUntil(22);
    Check(smp.InstructionInProgress() && smp.PortRead(0) == 0,
          "Reset fixture is paused inside the last port-store wait");
    Program(smp, {0x00});
    smp.RunUntil(2);
    Check(smp.CycleCount() == 2 && !smp.InstructionInProgress() && smp.PortRead(0) == 0,
          "Power cancels a pending stretched store and restores default timing");
}

void HaltedClocking() {
    for (uint8_t opcode : {uint8_t(0xef), uint8_t(0xff)}) {
        Smp smp;
        Program(smp, {opcode});
        smp.IoState().internalWaitStates = 1;
        smp.GetTimer2().enable = true;
        smp.GetTimer2().target = 1;
        Dsp dsp;
        dsp.SetRam(smp.Ram()); smp.SetDsp(dsp);
        smp.RunUntil(65);
        Check((opcode == 0xef ? smp.r.wait : smp.r.stop) && smp.r.pc == 0x201,
              "SLEEP and STOP retain halted execution with stretched idle cycles");
        Check(smp.CycleCount() == 65 && dsp.Phase() == 1 && smp.GetTimer2().stage3 == 4,
              "DSP and timers continue through stretched halted idle cycles");
    }
}
}

int main() {
    try {
        AddressClasses(); PortReadHold(); StoreDeadlines(); RamAndTestSampling();
        ResetDuringWait(); HaltedClocking();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
    std::puts("SMP wait-state classes, clock ratios, port holds and exact deadlines passed");
}
