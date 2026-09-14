// snes emulator
// tests/test_cpu_regressions.cpp
// Regression coverage for CPU instructions, interrupts, and stops.

#include "snes/core/SnesCpu.hpp"
#include <array>
#include <cstdio>
#include <string_view>
#include <vector>

using namespace snes::core;
namespace {
int failures = 0;
int checks = 0;

void Check(const char* label, unsigned actual, unsigned expected) {
    ++checks;
    if (actual != expected) {
        std::fprintf(stderr, "FAIL %s: actual=$%X expected=$%X\n", label, actual, expected);
        ++failures;
    }
}

struct Bus : ICpuBus {
    std::array<uint8_t, 65536> ram{};
    std::vector<unsigned> writes;
    unsigned reads = 0;
    Bus() {
        ram.fill(0xea);
        ram[0xfffc] = 0x00;
        ram[0xfffd] = 0x80;
        for (unsigned vector : {0xfff4u, 0xfffau, 0xfffeu, 0xffe4u, 0xffe6u,
                                0xffeau, 0xffeeu}) {
            ram[vector] = 0x00;
            ram[vector + 1] = 0x90;
        }
    }
    uint8_t Read(uint32_t address) override { ++reads; return ram[address & 0xffff]; }
    void Write(uint32_t address, uint8_t data) override {
        writes.push_back(address);
        ram[address & 0xffff] = data;
    }
};

void InterruptStack() {
    // Exercise every byte of an interrupt frame across the page boundary.
    for (bool emulation : {false, true}) {
        for (unsigned stack : {0x100u, 0x101u, 0x102u, 0x1ffu}) {
            for (unsigned kind = 0; kind < 4; ++kind) {
                Bus bus;
                SnesCpu cpu(bus);
                cpu.Reset();
                auto& r = cpu.regs();
                r.e = emulation;
                r.s = static_cast<uint16_t>(stack);
                r.p = 0x39; // Decimal and carry set, IRQ unmasked.
                r.pb = emulation ? 0 : 0x12;
                bus.ram[0x8000] = kind == 0 ? 0x00 : kind == 1 ? 0x02 : 0xea;
                bus.ram[0x9000] = 0x40; // RTI
                if (kind == 2) cpu.RequestNmi();
                if (kind == 3) cpu.SetIrqLevel(true);
                cpu.Step();
                if (kind >= 2) {
                    cpu.SetIrqLevel(false);
                    cpu.Step();
                }
                Check("Interrupt vector", r.pc, 0x9000);
                Check("Interrupt clears decimal and masks IRQ", r.p & 0x0c, 0x04);
                const unsigned frameSize = emulation ? 3 : 4;
                Check("Interrupt frame size", unsigned(bus.writes.size()), frameSize);
                for (unsigned i = 0; i < bus.writes.size(); ++i) {
                    const unsigned address = emulation ? 0x100 | ((stack - i) & 0xff)
                                                       : (stack - i) & 0xffff;
                    Check("Interrupt stack write address", bus.writes[i], address);
                }
                const unsigned expectedStack = emulation ? 0x100 | ((stack - frameSize) & 0xff)
                                                         : (stack - frameSize) & 0xffff;
                Check("Interrupt stack pointer", r.s, expectedStack);
                const unsigned statusAddress = emulation ? 0x100 | ((stack - 2) & 0xff)
                                                         : (stack - 3) & 0xffff;
                Check("Saved interrupt status", bus.ram[statusAddress],
                      emulation && kind >= 2 ? 0x29 : 0x39);
                cpu.Step();
                Check("RTI restores PC", r.pc, kind < 2 ? 0x8002 : 0x8001);
                Check("RTI restores stack", r.s, stack);
                Check("RTI restores bank", r.pb, emulation ? 0 : 0x12);
                Check("RTI restores flags", r.p, 0x39);
            }
        }
    }
}

void ExtendedStackInstructions() {
    for (bool emulation : {false, true}) {
        for (unsigned opcode : {0x0bu, 0x2bu, 0xf4u, 0xd4u, 0x62u, 0x22u, 0x6bu, 0xfcu}) {
            Bus bus;
            SnesCpu cpu(bus);
            cpu.Reset();
            auto& r = cpu.regs();
            r.e = emulation;
            const bool pull = opcode == 0x2b || opcode == 0x6b;
            const unsigned bytes = opcode == 0x22 || opcode == 0x6b ? 3 : 2;
            r.s = pull ? 0x1ff : 0x100;
            r.d = opcode == 0xd4 ? 0x3400 : 0x5678;
            bus.ram[0x8000] = static_cast<uint8_t>(opcode);
            bus.ram[0x8001] = opcode == 0xd4 ? 0x10 : 0x78;
            bus.ram[0x8002] = 0x56;
            bus.ram[0x8003] = 0x12;
            bus.ram[0x3410] = 0x78;
            bus.ram[0x3411] = 0x56;
            bus.ram[0x5678] = 0x00;
            bus.ram[0x5679] = 0x90;
            bus.ram[0x200] = 0x78;
            bus.ram[0x201] = 0x56;
            bus.ram[0x202] = 0x12;
            cpu.Step();
            const unsigned fullStack = pull ? 0x1ff + bytes : 0x100 - bytes;
            const unsigned finalStack = emulation ? 0x100 | (fullStack & 0xff) : fullStack;
            Check("Extended instruction restores stack page", r.s, finalStack);
            if (pull) {
                Check("Extended pull crosses stack page", opcode == 0x2b ? r.d : r.pc,
                      opcode == 0x2b ? 0x5678 : 0x5679);
                if (opcode == 0x6b) Check("RTL restores bank", r.pb, 0x12);
            } else {
                Check("Extended push byte count", unsigned(bus.writes.size()), bytes);
                for (unsigned i = 0; i < bus.writes.size(); ++i)
                    Check("Extended push crosses stack page", bus.writes[i], 0x100 - i);
            }
            // A following ordinary push must use the restored page.
            bus.ram[r.pc] = 0x48; // PHA
            cpu.Step();
            Check("Following push uses restored stack", bus.writes.back(), finalStack);
        }
    }
}

void WaitInterrupt() {
    for (bool nmi : {false, true}) {
        for (bool beforeWait : {false, true}) {
            Bus bus;
            SnesCpu cpu(bus);
            cpu.Reset();
            cpu.regs().p &= ~Processor65816::FlagI;
            bus.ram[0x8000] = 0xcb; // WAI
            bus.ram[0x9000] = 0xa9; // LDA #$57
            bus.ram[0x9001] = 0x57;
            bus.ram[0x9002] = 0x40; // RTI
            auto signal = [&] { if (nmi) cpu.RequestNmi(); else cpu.SetIrqLevel(true); };
            if (beforeWait) signal();
            cpu.Step();
            if (!beforeWait) {
                signal();
                cpu.Step(); // Wake and schedule the interrupt.
            }
            cpu.SetIrqLevel(false);
            cpu.Step();
            Check("WAI interrupt vector", cpu.regs().pc, 0x9000);
            Check("Interrupt releases WAI", cpu.regs().wai, false);
            cpu.Step();
            Check("WAI handler executes", cpu.regs().a, 0x57);
            cpu.Step();
            Check("WAI RTI resumes next instruction", cpu.regs().pc, 0x8001);
        }
    }
    Bus bus;
    SnesCpu cpu(bus);
    cpu.Reset(); // IRQ remains masked.
    bus.ram[0x8000] = 0xcb;
    bus.ram[0x8001] = 0xa9;
    bus.ram[0x8002] = 0x63;
    cpu.Step();
    cpu.SetIrqLevel(true);
    cpu.Step();
    Check("Masked IRQ releases WAI", cpu.regs().wai, false);
    cpu.Step();
    Check("Masked IRQ resumes without vectoring", cpu.regs().a, 0x63);
    Check("Masked IRQ leaves stack alone", unsigned(bus.writes.size()), 0);
}

void Stop() {
    for (bool pending : {false, true}) {
        Bus bus;
        SnesCpu cpu(bus);
        cpu.Reset();
        cpu.regs().p &= ~Processor65816::FlagI;
        bus.ram[0x8000] = 0xdb; // STP
        if (pending) { cpu.RequestNmi(); cpu.SetIrqLevel(true); }
        Check("STP instruction clocks", cpu.Step(), 18);
        Check("STP sets stopped state", cpu.regs().stp, true);
        cpu.RequestNmi();
        cpu.SetIrqLevel(true);
        const unsigned reads = bus.reads;
        for (unsigned i = 0; i < 4; ++i) Check("Stopped step clocks", cpu.Step(), 6);
        Check("STP ignores interrupt vectoring", cpu.regs().pc, 0x8001);
        Check("STP does not read bus", bus.reads, reads);
        Check("STP does not push interrupt frames", unsigned(bus.writes.size()), 0);
        bus.ram[0x8000] = 0xea;
        cpu.Reset();
        Check("Reset clears stopped state", cpu.regs().stp, false);
        cpu.Step();
        Check("Reset resumes execution", cpu.regs().pc, 0x8001);
    }
}
} // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string_view(argv[1]) == "stop") Stop();
    else { InterruptStack(); ExtendedStackInstructions(); WaitInterrupt(); }
    std::printf("%d CPU checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
