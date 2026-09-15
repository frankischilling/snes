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

std::unique_ptr<Emulator> Machine(std::initializer_list<uint8_t> code) {
    std::vector<uint8_t> rom(0x8000, 0xea);
    std::copy(code.begin(), code.end(), rom.begin());
    std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
    rom[0x7fc0] = 'T';
    rom[0x7fd5] = 0x20; rom[0x7fd6] = 0; rom[0x7fd7] = 5;
    rom[0x7fd8] = 0; rom[0x7fd9] = 1;
    rom[0x7fdc] = 0xff; rom[0x7fdd] = 0xff;
    rom[0x7fde] = 0; rom[0x7fdf] = 0;
    rom[0x7ffc] = 0; rom[0x7ffd] = 0x80;
    rom[0x7ffa] = 0; rom[0x7ffb] = 0x90;
    rom[0x7fea] = 0; rom[0x7feb] = 0x90;
    auto machine = std::make_unique<Emulator>();
    std::string error;
    if (!machine->LoadCartridge(rom, &error)) throw std::runtime_error(error);
    return machine;
}

void WordAccessesAdvanceHardware() {
    for (bool write : {false, true}) {
        auto machine = Machine({uint8_t(write ? 0x8d : 0xad), 0x10, 0x21});
        auto& cpu = *machine->GetCpu();
        cpu.regs().e = false;
        cpu.regs().p = 0;
        cpu.regs().a = 0x1234;
        std::vector<uint64_t> moments;
        std::vector<uint8_t> bytes;
        machine->GetBus().Map(0, 0, 0x2110, 0x2111,
            [&](uint32_t address, uint8_t) {
                moments.push_back(machine->GetTiming().MasterClocksElapsed());
                return uint8_t((address & 1) ? 0x12 : 0x34);
            },
            [&](uint32_t, uint8_t value) {
                moments.push_back(machine->GetTiming().MasterClocksElapsed());
                bytes.push_back(value);
            });
        Check(cpu.Step() == 36, "Word I/O instruction consumes three ROM cycles and two I/O cycles");
        Check(moments == std::vector<uint64_t>{24, 30}, "Hardware advances between the two bytes of word I/O");
        Check(machine->GetTiming().MasterClocksElapsed() == 36, "CPU clocks reach the scheduler exactly once");
        Check(machine->GetSmp().CycleCount() > 0, "SMP advances during CPU bus cycles");
        if (write) Check(bytes == std::vector<uint8_t>{0x34, 0x12}, "Word writes preserve low then high ordering");
        else Check(cpu.regs().a == 0x1234, "Word reads preserve low and high bytes");
    }
}

void HBlankWithinInstruction() {
    auto machine = Machine({0xad, 0x12, 0x42}); // LDA HVBJOY
    auto& cpu = *machine->GetCpu();
    // Reach H=1072 through CPU cycles, including the refresh halt.
    while (machine->GetTiming().HCounter() < 1072) cpu.idle();
    Check(machine->GetTiming().HCounter() == 1072, "Test begins before HBlank");
    cpu.Step();
    Check((cpu.regs().a & 0x40) != 0, "HVBJOY sees HBlank reached during operand fetches");
}

void SoundPortDeadlines() {
    for (Region region : {Region::NTSC, Region::PAL}) {
        for (uint8_t port = 0; port < 4; ++port) {
            for (bool mirror : {false, true}) {
                const uint8_t cpuPort = uint8_t((mirror ? 0x7c : 0x40) + port);
                auto reader = Machine({0xad, cpuPort, 0x21, 0xad, cpuPort, 0x21,
                    0xad, cpuPort, 0x21, 0xad, cpuPort, 0x21, 0xad, cpuPort, 0x21});
                reader->GetTiming().SetRegion(region);
                auto& sound = reader->GetSmp();
                sound.Power();
                sound.IoState().iplRomEnable = false;
                sound.r.pc = 0x200;
                sound.Ram()[0x200] = 0x8f; // MOV port,#$A5: store on cycle five.
                sound.Ram()[0x201] = 0xa5;
                sound.Ram()[0x202] = uint8_t(0xf4 + port);
                for (unsigned instruction = 0; instruction < 5; ++instruction) {
                    reader->GetCpu()->Step();
                    Check(reader->GetCpu()->regs().a == (instruction < 3 ? 0 : 0xa5),
                          "Console reads cannot observe a sound-port store before its cycle");
                    const auto target = reader->GetTiming().MasterClocksElapsed() * 1024000 /
                                        reader->GetTiming().MasterClockHz();
                    Check(sound.CycleCount() == target, "SMP follows the region clock ratio without overshoot");
                }

                auto writer = Machine({0x8d, cpuPort, 0x21});
                writer->GetTiming().SetRegion(region);
                auto& receiver = writer->GetSmp();
                receiver.Power();
                receiver.IoState().iplRomEnable = false;
                receiver.r.pc = 0x200;
                receiver.Ram()[0x200] = 0xe4; // MOV A,port: read on cycle three.
                receiver.Ram()[0x201] = uint8_t(0xf4 + port);
                writer->GetCpu()->regs().a = 0x6b;
                writer->GetCpu()->Step();
                Check(receiver.r.a == 0, "SMP port read remains pending after the console store");
                while (receiver.CycleCount() < 3) writer->GetCpu()->idle();
                Check(receiver.r.a == 0x6b,
                      "Sound read observes a console write arriving between opcode and data cycles");
            }
        }
    }
}

struct CpuBus : ICpuBus {
    std::array<uint8_t, 65536> bytes{};
    CpuBus() {
        bytes.fill(0xea);
        bytes[0xfffc] = 0; bytes[0xfffd] = 0x80;
        bytes[0xfffe] = 0; bytes[0xffff] = 0x90;
        bytes[0xfffa] = 0; bytes[0xfffb] = 0xa0;
    }
    uint8_t Read(uint32_t address) override { return bytes[address & 0xffff]; }
    void Write(uint32_t address, uint8_t value) override { bytes[address & 0xffff] = value; }
};

void InterruptMaskSampling() {
    for (uint8_t opcode : {uint8_t(0x58), uint8_t(0xc2), uint8_t(0x28)}) {
        CpuBus bus;
        SnesCpu cpu(bus);
        cpu.Reset();
        bus.bytes[0x8000] = opcode; // CLI, REP #I, PLP
        if (opcode == 0xc2) bus.bytes[0x8001] = 4;
        if (opcode == 0x28) bus.bytes[0x100] = 0x30;
        cpu.SetIrqLevel(true);
        cpu.Step();
        const auto resumed = cpu.regs().pc;
        Check(!cpu.flagI(), "Instruction unmasks IRQ");
        cpu.Step();
        Check(cpu.regs().pc == resumed + 1, "IRQ waits through the instruction following a late unmask");
        cpu.Step();
        Check(cpu.regs().pc == 0x9000, "IRQ enters after the next sampling point");
    }
    for (uint8_t opcode : {uint8_t(0x78), uint8_t(0xe2), uint8_t(0x28)}) {
        CpuBus bus;
        SnesCpu cpu(bus);
        cpu.Reset();
        cpu.regs().p &= ~Processor65816::FlagI;
        bus.bytes[0x8000] = opcode; // SEI, SEP #I, PLP
        if (opcode == 0xe2) bus.bytes[0x8001] = 4;
        if (opcode == 0x28) bus.bytes[0x100] = 0x34;
        cpu.SetIrqLevel(true);
        cpu.Step();
        Check(cpu.flagI(), "Instruction masks IRQ");
        cpu.SetIrqLevel(false);
        cpu.Step();
        Check(cpu.regs().pc == 0x9000, "A sampled IRQ survives the final-cycle mask change");
        Check((bus.bytes[(cpu.regs().s + 1) & 0x1ff] & 4) != 0, "Interrupt frame contains the new I flag");
    }
}

void NmiInsideInstruction() {
    CpuBus bus;
    SnesCpu cpu(bus);
    cpu.Reset();
    bus.bytes[0x8000] = 0xad; bus.bytes[0x8001] = 0; bus.bytes[0x8002] = 0x20;
    unsigned elapsed = 0;
    cpu.SetClockCallback([&](uint32_t clocks) {
        elapsed += clocks;
        if (elapsed == 12) cpu.RequestNmi();
    });
    cpu.Step();
    Check(cpu.regs().pc == 0x8003, "An instruction finishes before NMI entry");
    cpu.Step();
    Check(cpu.regs().pc == 0xa000, "NMI raised during an operand fetch reaches the same instruction's sample");
}

void EnableNmiDuringVblank() {
    for (bool word : {false, true}) {
        auto machine = Machine({0x8d, 0x00, 0x42, 0xea, 0xea, 0xdb});
        auto& cpu = *machine->GetCpu();
        while (machine->GetTiming().VCounter() < 225 || machine->GetTiming().HCounter() < 16) cpu.idle();
        if (word) {
            cpu.regs().e = false;
            cpu.regs().p = Processor65816::FlagX | Processor65816::FlagI;
        }
        cpu.regs().a = 0x80;
        cpu.Step();
        Check(cpu.regs().pc == 0x8003, "Enabling NMI finishes its register-write instruction");
        cpu.Step();
        Check(cpu.regs().pc == 0x8004, "A word store cannot accept NMI between its low and high register writes");
        cpu.Step();
        Check(cpu.regs().pc == 0x9000, "The register lock expires before the following access without an extra instruction delay");
    }
}

void DmaStartupCycle() {
    auto machine = Machine({0x8d, 0x0b, 0x42, 0xea, 0xdb}); // STA MDMAEN; NOP; STP
    auto& cpu = *machine->GetCpu();
    auto& dma = machine->GetDma();
    auto& channel = dma.Channel(0);
    channel.writeControl(0); channel.sourceBank = 0x7e; channel.sourceAddress = 0;
    channel.targetAddress = 0x10; channel.transferSize = 1;
    machine->GetBus().Write(0x7e0000, 0x5a);
    uint64_t writtenAt = 0;
    uint16_t pcAtWrite = 0;
    machine->GetBus().Map(0, 0, 0x2110, 0x2110,
        [](uint32_t, uint8_t value) { return value; },
        [&](uint32_t, uint8_t value) {
            Check(value == 0x5a, "DMA startup transfers the source byte");
            writtenAt = machine->GetTiming().MasterClocksElapsed();
            pcAtWrite = cpu.regs().pc;
        });
    cpu.regs().a = 1;
    cpu.Step();
    Check(dma.AnyDmaEnabled() && writtenAt == 0, "MDMAEN queues a transfer without running it inside the write");
    cpu.Step();
    Check(pcAtWrite == 0x8004, "One opcode fetch completes before DMA takes the bus");
    Check(writtenAt == 64 && machine->GetTiming().MasterClocksElapsed() == 74,
          "DMA startup and CPU resumption preserve bus phase");
    Check(!dma.AnyDmaEnabled() && channel.transferSize == 0, "DMA completes before the interrupted CPU cycle resumes");
}
}

int main() {
    try {
        WordAccessesAdvanceHardware(); HBlankWithinInstruction(); SoundPortDeadlines();
        InterruptMaskSampling(); NmiInsideInstruction(); EnableNmiDuringVblank(); DmaStartupCycle();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
    std::puts("CPU bus scheduling and interrupt sampling checks passed");
}
