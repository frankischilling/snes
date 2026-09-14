#include "snes/core/Emulator.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>

using namespace snes::core;

namespace {
unsigned checks = 0;
void Check(bool condition, const char* label) {
    ++checks;
    if (!condition) throw std::runtime_error(label);
}

struct Input final : IInputProvider {
    std::array<InputState, 8> pads{};
    std::array<MouseState, 2> mice{};
    InputState Poll(uint64_t) override { return pads[0]; }
    InputState PollController(int player, uint64_t) override { return pads.at(player); }
    MouseState PollMouse(int port, uint64_t) override {
        const auto state = mice.at(port);
        mice.at(port).dx = mice.at(port).dy = 0;
        return state;
    }
};

std::unique_ptr<Emulator> Machine(Input& input) {
    std::vector<uint8_t> rom(0x8000, 0xea);
    std::fill(rom.begin() + 0x7fc0, rom.end(), 0);
    rom[0x7fc0] = 'T';
    rom[0x7fd5] = 0x20;
    rom[0x7fd7] = 5;
    rom[0x7fdc] = rom[0x7fdd] = 0xff;
    rom[0x7ffd] = 0x80;
    auto emu = std::make_unique<Emulator>();
    emu->AttachInputProvider(&input);
    Check(emu->LoadCartridge(rom), "Generated controller test ROM loads");
    return emu;
}

void Latch(MemoryBus& bus) { bus.Write(0x4016, 1); bus.Write(0x4016, 0); }
uint32_t Packet(MemoryBus& bus, unsigned port, unsigned count, unsigned wire = 0) {
    uint32_t result = 0;
    for (unsigned i = 0; i < count; ++i)
        result = (result << 1) | ((bus.Read(0x4016 + port) >> wire) & 1);
    return result;
}

void Poll(Emulator& emu) {
    auto& poller = emu.GetAutoJoypad();
    // Complete a poll through the same timing callback used by the frame loop.
    poller.Reset();
    emu.GetBus().Write(0x4200, 1);
    emu.GetTiming().onJoypadPoll(256, 225, 225);
    Check(emu.GetCpuIo().Read(0x4212, 0) & 1, "Automatic polling sets busy");
    for (unsigned i = 1; i <= 33; ++i)
        emu.GetTiming().onJoypadPoll(512, 226, 225);
    Check(!(emu.GetCpuIo().Read(0x4212, 0) & 1), "Automatic polling clears busy");
}

void PadsAndPolling() {
    Input input;
    input.pads[0].b = input.pads[0].start = input.pads[0].r = true;
    input.pads[1].y = input.pads[1].a = true;
    auto emu = Machine(input);
    auto& bus = emu->GetBus();
    Latch(bus);
    Check(Packet(bus, 0, 16) == 0x9010, "Port one serial button order and signature");
    Check(Packet(bus, 1, 16) == 0x4080, "Port two has independent input");
    for (unsigned i = 0; i < 512; ++i) Check((bus.Read(0x4016) & 3) == 1, "Exhausted pad stays high");
    bus.Write(0x4016, 1);
    for (unsigned i = 0; i < 20; ++i) Check(bus.Read(0x4016) & 1, "Latch high holds first bit");
    bus.Write(0x4016, 0);
    Check(Packet(bus, 0, 16) == 0x9010, "Latch-high reads do not consume pad bits");
    Latch(bus);
    Check(Packet(bus, 0, 2) == 2, "Read two manual bits before polling");
    Poll(*emu);
    Check(emu->GetAutoJoypad().Joy1() == 0x9010 && emu->GetAutoJoypad().Joy2() == 0x4080,
          "Auto poll independently latches both physical ports");
    Check((bus.Read(0x4016) & 3) == 1 && (bus.Read(0x4017) & 3) == 1,
          "Automatic polling consumes the manual shift registers");
    Check(bus.Read(0x4218) == 0x10 && bus.Read(0x4219) == 0x90, "Auto result reaches CPU I/O");

    auto& poller = emu->GetAutoJoypad();
    poller.Reset();
    bus.Write(0x4200, 1);
    poller.Tick128(256, 225, 225);
    bus.Write(0x4016, 1);
    bus.Write(0x4200, 0);
    poller.Tick128(512, 226, 225);
    for (unsigned i = 0; i < 20; ++i) Check(bus.Read(0x4016) & 1, "CPU latch survives automatic release");
    bus.Write(0x4016, 0);
    Check(Packet(bus, 0, 16) == 0x9010, "Disabling auto polling during latch releases it");

    struct LegacyInput final : IInputProvider {
        InputState Poll(uint64_t) override { InputState state; state.b = true; return state; }
    } legacy;
    emu->AttachInputProvider(&legacy);
    Latch(bus);
    Check(Packet(bus, 0, 16) == 0x8000 && Packet(bus, 1, 16) == 0,
          "Legacy single-player providers do not duplicate input on port two");
}

void Mouse() {
    Input input;
    auto emu = Machine(input);
    auto& bus = emu->GetBus();
    auto& ports = emu->GetAutoJoypad().Ports();
    ports.Configure(0, ControllerDevice::Mouse);
    input.mice[0] = {300, -260, true, false};
    Latch(bus);
    Check(Packet(bus, 0, 32) == 0x0041ff7f, "Mouse buttons and signed movement packet");
    for (unsigned i = 0; i < 512; ++i) Check((bus.Read(0x4016) & 3) == 1, "Mouse exhausted reads stay high");
    Latch(bus);
    Check(Packet(bus, 0, 32) == 0x0041ff7f, "Mouse retains clipped movement");
    Latch(bus);
    Check(Packet(bus, 0, 32) == 0x0041862e, "Mouse emits final movement remainder");
    Latch(bus);
    Check(Packet(bus, 0, 32) == 0x00410000, "Mouse movement is consumed once");
    for (const auto sensitivity : {1u, 2u, 0u}) {
        bus.Write(0x4016, 1);
        Check(!(bus.Read(0x4016) & 1), "Mouse sensitivity command returns zero");
        bus.Write(0x4016, 0);
        Check(Packet(bus, 0, 32) == (0x00410000 | (sensitivity << 20)), "Mouse cycles three sensitivity settings");
    }
    input.mice[0] = {-1, 2, false, true};
    Poll(*emu);
    Check(emu->GetAutoJoypad().Joy1() == 0x81, "Auto poll reads mouse signature and buttons");
    Check(Packet(bus, 0, 16) == 0x0281, "Manual reads continue mouse movement after auto poll");
    ports.Configure(0, ControllerDevice::None);
    Latch(bus);
    Check(Packet(bus, 0, 32) == 0, "Disconnected port drives no serial data");
}

void Multitap() {
    Input input;
    input.pads[1].b = true;
    input.pads[2].y = true;
    input.pads[3].a = true;
    input.pads[4].r = true;
    auto emu = Machine(input);
    auto& bus = emu->GetBus();
    auto& ports = emu->GetAutoJoypad().Ports();
    ports.Configure(1, ControllerDevice::Multitap, {1, 2, 3, 4});
    bus.Write(0x4016, 1);
    Check((bus.Read(0x4017) & 3) == 2, "Multitap latch-high signature");
    bus.Write(0x4016, 0);
    Check((bus.Read(0x4017) & 3) == 1, "Multitap first pair B bits");
    bus.Write(0x4201, 0x7f);
    Check(Packet(bus, 1, 16) == 0x80, "WRIO bit seven selects second pair");
    bus.Write(0x4201, 0xff);
    Check((bus.Read(0x4017) & 3) == 2, "Multitap pair counters are independent");
    Latch(bus);
    Check(Packet(bus, 1, 16, 1) == 0x4000, "Second serial wire carries the other pad");
    Check((bus.Read(0x4017) & 3) == 3, "Multitap returns ones after both pads finish");
    Poll(*emu);
    Check(emu->GetAutoJoypad().Joy2() == 0x8000 && emu->GetAutoJoypad().Joy4() == 0x4000,
          "Multitap auto results populate JOY2 and JOY4");
    bus.Write(0x4201, 0x7f);
    Check(Packet(bus, 1, 16, 1) == 0x10, "Unpolled multitap pair remains available");
    ports.Configure(0, ControllerDevice::Multitap, {1, -1, 3, 4});
    Latch(bus);
    bus.Write(0x4201, 0xbf);
    Check(Packet(bus, 0, 16) == 0x80, "Port one multitap uses WRIO bit six");
    bus.Write(0x4201, 0xff);
    Check(Packet(bus, 0, 16, 1) == 0, "Unplugged multitap pad returns zero");
    Check((bus.Read(0x4016) & 3) == 1, "Unplugged pad stays zero after packet end");
    ports.Reset();
    Latch(bus);
    Check((bus.Read(0x4016) & 3) == 1, "Reset preserves device selection and clears pair counters");
}
}

int main() {
    try {
        PadsAndPolling();
        Mouse();
        Multitap();
        std::printf("%u controller checks passed\n", checks);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "Controller regression: %s\n", error.what());
        return 1;
    }
}
