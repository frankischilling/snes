#include "snes/core/CpuIoRegisters.hpp"
#include "snes/core/Emulator.hpp"
#include "snes/core/SnesCpu.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace snes::core;

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void Advance(CpuIoRegisters& io, unsigned cycles) {
    while (cycles--) io.AluStep();
}

uint16_t Result(CpuIoRegisters& io, uint16_t address) {
    return uint16_t(io.Read(address, 0) | (io.Read(address + 1, 0) << 8));
}

void MultiplicationResults() {
    CpuIoRegisters io;
    for (unsigned a = 0; a < 256; ++a) {
        for (unsigned b = 0; b < 256; ++b) {
            io.Write(0x4202, uint8_t(a));
            io.Write(0x4203, uint8_t(b));
            Check(io.rdmpy() == 0 && io.rddiv() == (b * 256 + a),
                  "Multiply trigger exposes the operands before its first edge");
            Advance(io, 8);
            Check(Result(io, 0x4216) == a * b && Result(io, 0x4214) == b,
                  "All 8-bit products and shifted operand results match arithmetic");
            Advance(io, 3);
            Check(io.rdmpy() == a * b && io.rddiv() == b,
                  "Completed multiply registers remain stable");
        }
    }
}

void DivisionResults() {
    CpuIoRegisters io;
    auto divide = [&](unsigned a, unsigned b) {
        io.Write(0x4204, uint8_t(a));
        io.Write(0x4205, uint8_t(a >> 8));
        io.Write(0x4206, uint8_t(b));
        Check(io.rdmpy() == a, "Division starts with the dividend in the remainder");
        Advance(io, 16);
        const auto quotient = b ? a / b : 0xffff;
        const auto remainder = b ? a % b : a;
        Check(Result(io, 0x4214) == quotient && Result(io, 0x4216) == remainder,
              "Division quotient and remainder match arithmetic including zero divisor");
    };
    for (unsigned a = 0; a < 65536; ++a) {
        for (unsigned b : {0u, 1u, 2u, 3u, 7u, 16u, 127u, 128u, 254u, 255u}) divide(a, b);
    }
    for (unsigned b = 0; b < 256; ++b) {
        for (unsigned a : {0u, 1u, 255u, 256u, 1000u, 32767u, 32768u, 65534u, 65535u}) divide(a, b);
    }
}

void IntermediateResultsAndBusyWrites() {
    CpuIoRegisters io;
    io.Write(0x4202, 12);
    io.Write(0x4203, 13);
    constexpr std::array<uint16_t, 8> products{0, 0, 52, 156, 156, 156, 156, 156};
    for (unsigned edge = 0; edge < products.size(); ++edge) {
        io.AluStep();
        Check(io.rdmpy() == products[edge], "Multiply exposes the expected partial sum at each edge");
        Check(io.rddiv() == (0x0d0c >> (edge + 1)), "Multiply shifts the visible operand register");
    }

    io.Reset();
    io.Write(0x4204, 0xe8);
    io.Write(0x4205, 3);
    io.Write(0x4206, 7);
    constexpr std::array<uint16_t, 16> quotients{0,0,0,0,0,0,0,0,1,2,4,8,17,35,71,142};
    constexpr std::array<uint16_t, 16> remainders{1000,1000,1000,1000,1000,1000,1000,1000,
                                                104,104,104,104,48,20,6,6};
    for (unsigned edge = 0; edge < quotients.size(); ++edge) {
        io.AluStep();
        Check(io.rddiv() == quotients[edge] && io.rdmpy() == remainders[edge],
              "Division exposes the expected intermediate quotient and remainder");
    }

    io.Write(0x4202, 0xff);
    io.Write(0x4203, 3);
    Advance(io, 4);
    Check(io.rdmpy() == 45, "Busy multiply fixture reaches the fourth partial sum");
    io.Write(0x4202, 0);
    io.Write(0x4203, 9);
    Check(io.rdmpy() == 0, "A busy multiply trigger clears only the accumulator");
    Advance(io, 4);
    Check(io.rdmpy() == 720 && io.rddiv() == 3,
          "Busy trigger and operand writes cannot replace or extend the active multiply");
    io.Write(0x4203, 5);
    Advance(io, 8);
    Check(io.rdmpy() == 0 && io.rddiv() == 5, "The following multiply uses the newly written operand");

    io.Write(0x4202, 0xff);
    io.Write(0x4203, 3);
    Advance(io, 4);
    io.Write(0x4204, 100);
    io.Write(0x4205, 0);
    io.Write(0x4206, 2);
    Check(io.rdmpy() == 100, "A busy divide trigger loads the remainder without changing the operation");
    Advance(io, 4);
    Check(io.rdmpy() == 820 && io.rddiv() == 3, "A rejected divide still leaves the multiply running");

    io.Write(0x4206, 7);
    Advance(io, 5);
    io.Reset();
    Advance(io, 32);
    Check(io.rdmpy() == 0 && io.rddiv() == 0, "Reset cancels unfinished math");
}

struct MathBus : ICpuBus {
    CpuIoRegisters io;
    uint8_t speed = 6;
    uint8_t Read(uint32_t address) override { return io.Read(address, 0); }
    void Write(uint32_t address, uint8_t data) override { io.Write(address, data); }
    uint8_t Speed(uint32_t) const override { return speed; }
};

void CpuEdges() {
    for (uint8_t speed : {6, 8, 12}) {
        MathBus bus;
        bus.speed = speed;
        SnesCpu cpu(bus);
        cpu.SetAluStepCallback([&](bool writeCycle) { bus.io.AluStep(writeCycle); });
        cpu.write(0x804202, 0xff);
        cpu.write(0x804203, 1);
        Check(bus.io.rdmpy() == 0, "The trigger store is not the first multiply edge");
        Check(cpu.read(0x804216) == 0 && bus.io.rdmpy() == 1,
              "CPU reads sample the result before advancing the ALU");
        cpu.idle();
        Check(bus.io.rdmpy() == 3, "CPU idle cycles advance the ALU");
        cpu.write(0x804202, 0);
        Check(bus.io.rdmpy() == 7, "CPU writes advance the ALU before changing operands");
        for (unsigned edge = 3; edge < 8; ++edge) cpu.read(0x008000);
        Check(bus.io.rdmpy() == 255 && bus.io.rddiv() == 1,
              "Fast, slow and extra-slow accesses each advance one math edge");
    }
}

void FinalCycleTriggers() {
    for (uint8_t speed : {6, 8, 12}) {
        for (unsigned delay : {7u, 8u, 9u}) {
            MathBus bus;
            bus.speed = speed;
            SnesCpu cpu(bus);
            cpu.SetAluStepCallback([&](bool writeCycle) { bus.io.AluStep(writeCycle); });
            cpu.write(0x4202, 0xff);
            cpu.write(0x4203, 3);
            for (unsigned cycle = 0; cycle < delay; ++cycle) cpu.idle();
            cpu.write(0x4203, 5);
            for (unsigned cycle = 0; cycle < 8; ++cycle) cpu.idle();
            Check(bus.io.rdmpy() == (delay == 7 ? 0 : 1275) &&
                  bus.io.rddiv() == (delay == 7 ? 3 : 5),
                  "A multiply trigger on the final busy cycle is rejected; the following cycle accepts it");
        }
        for (unsigned delay : {15u, 16u, 17u}) {
            MathBus bus;
            bus.speed = speed;
            SnesCpu cpu(bus);
            cpu.SetAluStepCallback([&](bool writeCycle) { bus.io.AluStep(writeCycle); });
            cpu.write(0x4204, 0xe8);
            cpu.write(0x4205, 3);
            cpu.write(0x4206, 7);
            for (unsigned cycle = 0; cycle < delay; ++cycle) cpu.idle();
            cpu.write(0x4206, 5);
            for (unsigned cycle = 0; cycle < 16; ++cycle) cpu.idle();
            Check(bus.io.rddiv() == (delay == 15 ? 142 : 200) &&
                  bus.io.rdmpy() == (delay == 15 ? 1000 : 0),
                  "A divide trigger on the final busy cycle is rejected; the following cycle accepts it");
        }
    }
}

std::unique_ptr<Emulator> Machine() {
    std::vector<uint8_t> rom(0x8000, 0xea);
    std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
    rom[0x7fc0] = 'T';
    rom[0x7fd5] = 0x20; rom[0x7fd6] = 0; rom[0x7fd7] = 5;
    rom[0x7fd8] = 0; rom[0x7fd9] = 1;
    rom[0x7fdc] = 0xff; rom[0x7fdd] = 0xff;
    rom[0x7fde] = 0; rom[0x7fdf] = 0;
    rom[0x7ffc] = 0; rom[0x7ffd] = 0x80;
    auto machine = std::make_unique<Emulator>();
    std::string error;
    if (!machine->LoadCartridge(rom, &error)) throw std::runtime_error(error);
    return machine;
}

void RefreshAndDma() {
    auto machine = Machine();
    auto& cpu = *machine->GetCpu();
    auto& io = machine->GetCpuIo();
    while (machine->GetTiming().HCounter() < 516) cpu.idle();
    cpu.write(0x004202, 0xff);
    cpu.write(0x004203, 1);
    cpu.idle();
    Check(io.rdmpy() == 1, "Math starts immediately before refresh");
    cpu.idle();
    Check(io.rdmpy() == 127, "Refresh contributes five math edges alongside the CPU idle edge");
    cpu.idle();
    Check(io.rdmpy() == 255, "The remaining math edge completes after refresh");

    machine = Machine();
    auto& dma = machine->GetDma();
    auto& channel = dma.Channel(0);
    channel.writeControl(0); channel.sourceBank = 0x7e; channel.sourceAddress = 0;
    channel.targetAddress = 0x10; channel.transferSize = 8;
    machine->GetBus().Write(0x004202, 0xff);
    machine->GetBus().Write(0x004203, 1);
    dma.Write(0x4300, 0);
    machine->GetBus().Write(0x00420b, 1);
    machine->GetCpu()->idle();
    Check(machine->GetCpuIo().rdmpy() == 1, "The CPU cycle before DMA advances math once");
    machine->GetCpu()->idle();
    Check(!dma.AnyDmaEnabled() && machine->GetCpuIo().rdmpy() == 3,
          "Ordinary DMA transfers do not add CPU math edges");
}

void WriteHeldAcrossDmaRefresh() {
    for (bool division : {false, true}) {
        for (bool unrelatedWrite : {false, true}) {
            auto machine = Machine();
            auto& cpu = *machine->GetCpu();
            auto& io = machine->GetCpuIo();
            auto& dma = machine->GetDma();
            while (machine->GetTiming().HCounter() < 480) cpu.idle();
            auto& channel = dma.Channel(0);
            channel.writeControl(0);
            channel.sourceBank = 0x7e;
            channel.sourceAddress = 0x1000;
            channel.targetAddress = 0x10;
            channel.transferSize = 16;
            const uint16_t trigger = division ? 0x4206 : 0x4203;
            if (division) {
                io.Write(0x4204, 0xe8);
                io.Write(0x4205, 3);
                io.Write(trigger, 7);
                Advance(io, 14);
            } else {
                io.Write(0x4202, 0xff);
                io.Write(trigger, 3);
                Advance(io, 6);
            }
            machine->GetBus().Write(0x420b, 1);
            cpu.idle(); // Arm DMA and leave one arithmetic edge outstanding.
            Check(machine->GetTiming().HCounter() < 538 && dma.AnyDmaEnabled(),
                  "The held-write fixture arms DMA before the refresh boundary");
            cpu.write(unrelatedWrite ? 0x7e0000 : trigger, 5);
            Check(machine->GetTiming().HCounter() > 578 && !dma.AnyDmaEnabled(),
                  "The pending CPU write resumes after DMA has crossed refresh");
            if (unrelatedWrite) {
                cpu.write(trigger, 5);
                for (unsigned cycle = 0; cycle < 16; ++cycle) cpu.idle();
                Check(io.rddiv() == (division ? 200 : 5) && io.rdmpy() == (division ? 0 : 1275),
                      "A busy decision for a RAM write cannot leak into the following math trigger");
            } else {
                Check(io.rddiv() == (division ? 142 : 3) && io.rdmpy() == (division ? 1000 : 0),
                      "Refresh during DMA preserves the final-busy-cycle trigger decision");
                for (unsigned cycle = 0; cycle < 16; ++cycle) cpu.idle();
                Check(io.rddiv() == (division ? 142 : 3) && io.rdmpy() == (division ? 1000 : 0),
                      "A DMA-delayed rejected trigger cannot leave a new operation running");
            }
        }
    }
}
}

int main() {
    try {
        MultiplicationResults(); DivisionResults(); IntermediateResultsAndBusyWrites();
        CpuEdges(); FinalCycleTriggers(); RefreshAndDma(); WriteHeldAcrossDmaRefresh();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
    std::puts("CPU multiply/divide results, intermediate states and bus timing passed");
}
