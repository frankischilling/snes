#include "snes/core/Emulator.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>

using namespace snes::core;
namespace {
void Check(bool condition, const char* label) { if (!condition) throw std::runtime_error(label); }
void Latch(ControllerPorts& ports) { ports.SetLatch(true); ports.SetLatch(false); }
uint32_t Packet(ControllerPorts& ports, unsigned bits, unsigned port = 1) {
    uint32_t value = 0;
    while (bits--) value = (value << 1) | ports.Read(port);
    return value;
}

void ScopePackets() {
    ControllerPorts ports;
    LightGunState input;
    ports.SetGunCallback([&](int, int) { return input; });
    ports.Configure(1, ControllerDevice::SuperScope);
    Latch(ports);
    Check(Packet(ports, 16) == 0x00ff, "Scope idle signature and ones after packet");
    input.trigger = true; input.offscreen = false;
    ports.SetLatch(true);
    Check(ports.Read(1) == 1 && ports.Read(1) == 1, "Strobe holds first Scope bit");
    ports.SetLatch(false);
    Check(Packet(ports, 16) == 0x80ff, "Scope trigger packet");
    Latch(ports);
    Check(Packet(ports, 8) == 0, "Normal trigger is reported once per press");
    input.turbo = true;
    Latch(ports);
    Check(Packet(ports, 8) == 0xa0, "Turbo starts firing a held trigger");
    Latch(ports);
    Check(Packet(ports, 8) == 0xa0, "Turbo repeats through strobes");
    input.turbo = false;
    Latch(ports);
    Check(Packet(ports, 8) == 0x20, "Stopping turbo clears fire but retains reported turbo flag");
    input.trigger = false; Latch(ports);
    input.trigger = true; input.offscreen = true; Latch(ports);
    Check(Packet(ports, 8) == 0x82, "Next trigger refreshes turbo and offscreen flags");
    input.trigger = false; input.offscreen = false; Latch(ports);
    Check(Packet(ports, 8) == 2, "Offscreen flag persists without a shot");
    input.cursor = true; input.pause = true; Latch(ports);
    Check(Packet(ports, 8) == 0x50, "Cursor and pause bits");
    Latch(ports);
    Check(Packet(ports, 8) == 0, "Normal cursor and pause are one-shot");
    for (unsigned i = 0; i < 1000; ++i) Check(ports.Read(1) == 1, "Scope reads saturate");
    ports.Reset(); input = {}; Latch(ports);
    Check(Packet(ports, 8) == 0, "Reset retains device but clears packet state");
}

void JustifierPackets() {
    for (bool paired : {false, true}) {
        ControllerPorts ports;
        std::array<LightGunState, 2> input;
        input[0].trigger = true; input[1].trigger = input[1].start = true;
        ports.Configure(1, paired ? ControllerDevice::Justifiers : ControllerDevice::Justifier);
        ports.SetGunCallback([&](int, int gun) { return input[gun]; });
        ports.SetLatch(true);
        Check(ports.Read(1) == 0 && ports.Read(1) == 0, "Justifier strobe reads zero");
        ports.SetLatch(false);
        Check(Packet(ports, 32) == (paired ? 0x000e55d8u : 0x000e5588u), "Justifier signature, buttons, and select bit");
        for (unsigned i = 0; i < 1000; ++i) Check(ports.Read(1) == 1, "Justifier reads saturate");
        Latch(ports);
        Check(Packet(ports, 32) == (paired ? 0x000e55d0u : 0x000e5580u), "Gun selection toggles with every strobe");
        input[0].trigger = false; input[0].start = true;
        Latch(ports);
        Check(Packet(ports, 32) == (paired ? 0x000e5578u : 0x000e5528u), "Independent start and trigger bits");
    }
}

void RiflePackets() {
    ControllerPorts ports;
    LightGunState input;
    ports.Configure(1, ControllerDevice::MacsRifle);
    ports.SetGunCallback([&](int, int) { return input; });
    for (bool strobe : {false, true}) {
        ports.SetLatch(strobe);
        input.trigger = false;
        Check(Packet(ports, 32) == 0, "Rifle trigger low");
        input.trigger = true;
        Check(Packet(ports, 32) == 0xffffffff, "Rifle exposes live trigger without latching");
    }
}

struct Input : IInputProvider {
    std::array<LightGunState, 2> guns;
    InputState Poll(uint64_t) override { return {}; }
    LightGunState PollLightGun(int, int gun, uint64_t) override { return guns[gun]; }
};
std::unique_ptr<Emulator> Machine(Input& input) {
    std::vector<uint8_t> rom(0x8000, 0);
    std::fill_n(rom.begin() + 0x7fc0, 21, ' ');
    rom[0] = 0xdb; rom[0x7fc0] = 'T'; rom[0x7fd5] = 0x20;
    rom[0x7fd7] = 5; rom[0x7fdc] = rom[0x7fdd] = 0xff; rom[0x7ffd] = 0x80;
    auto emu = std::make_unique<Emulator>();
    emu->AttachInputProvider(&input);
    Check(emu->LoadCartridge(rom), "Fixture loads");
    return emu;
}
unsigned Counter(MemoryBus& bus, unsigned address) {
    const unsigned lo = bus.Read(address);
    return lo | ((bus.Read(address) & 1) << 8);
}

void BeamLatches() {
    for (auto device : {ControllerDevice::SuperScope, ControllerDevice::Justifier,
                        ControllerDevice::Justifiers, ControllerDevice::MacsRifle}) {
        Input input;
        input.guns[0].x = 32; input.guns[0].y = 10;
        input.guns[0].offscreen = false; input.guns[0].trigger = true;
        input.guns[1] = input.guns[0]; input.guns[1].x = 200; input.guns[1].y = 20;
        auto emu = Machine(input);
        auto& ports = emu->GetAutoJoypad().Ports();
        auto& bus = emu->GetBus();
        ports.Configure(1, device);
        ports.PollLightGuns();
        const unsigned h = device == ControllerDevice::MacsRifle ? 108 : 72;
        const unsigned v = device == ControllerDevice::MacsRifle ? 52 : 11;
        emu->GetTiming().Tick(v * 1364 + (h - 1) * 4);
        Check((bus.Read(0x213f) & 0x40) == 0, "Counter does not latch before beam arrival");
        emu->GetTiming().Tick(6);
        Check((bus.Read(0x4213) & 0x80) == 0, "Gun pulls I/O pin low at beam hit");
        Check((bus.Read(0x213f) & 0x40) != 0, "Gun sets PPU latch flag");
        Check(Counter(bus, 0x213c) == h && Counter(bus, 0x213d) == v, "Gun beam coordinates");
        emu->GetTiming().Tick(4);
        Check((bus.Read(0x4213) & 0x80) != 0, "Gun pulse releases I/O pin");
        Check((bus.Read(0x213f) & 0x40) == 0, "Latch flag clears after status read");

        ports.FrameBegin(); bus.Write(0x4201, 0x7f); bus.Read(0x213f);
        emu->GetPpu().LatchCounters(17, 23);
        ports.PollLightGuns(); ports.BeamPosition(h, v, 224);
        // WRIO low keeps STAT78 bit 6 high regardless of gun pulses; inspect coordinates instead.
        Check(Counter(bus, 0x213c) == 17 && Counter(bus, 0x213d) == 23,
              "WRIO-low beam leaves stored coordinates intact");
        bus.Write(0x4201, 0xff); bus.Read(0x213f);
        ports.FrameBegin(); input.guns[0].offscreen = true;
        ports.PollLightGuns(); ports.BeamPosition(h, v, 224);
        Check((bus.Read(0x213f) & 0x40) == 0, "Offscreen gun does not latch");
    }
}

void GunSelectionAndPorts() {
    ControllerPorts ports;
    std::array<LightGunState, 2> input;
    input[0].offscreen = input[1].offscreen = false;
    input[0].x = 10; input[1].x = 50;
    input[0].y = input[1].y = 5;
    ports.SetGunCallback([&](int, int gun) { return input[gun]; });
    unsigned h = 0, hits = 0;
    ports.SetGunLatchCallback([&](uint16_t x, uint16_t) { h = x; ++hits; });
    ports.Configure(1, ControllerDevice::Justifiers);
    Latch(ports); ports.BeamPosition(90, 6, 224);
    Check(hits == 1 && h == 90, "Second gun selected after first strobe");
    ports.FrameBegin(); Latch(ports); ports.BeamPosition(50, 6, 224);
    Check(hits == 2 && h == 50, "First gun selected after second strobe");
    ports.Configure(1, ControllerDevice::Justifier);
    Latch(ports); ports.BeamPosition(90, 6, 224);
    Check(hits == 2, "Missing second gun does not latch");
    ports.Configure(1, ControllerDevice::None); ports.Configure(0, ControllerDevice::Justifier);
    Latch(ports); ports.BeamPosition(90, 6, 224);
    Check(hits == 2 && Packet(ports, 32, 0) == 0x000e5508, "Port 1 has serial data but no PPU latch wire");
}

void AutomaticPolling() {
    Input input;
    input.guns[0].trigger = true; input.guns[0].offscreen = false;
    auto emu = Machine(input);
    emu->GetAutoJoypad().Ports().Configure(1, ControllerDevice::SuperScope);
    auto& bus = emu->GetBus();
    bus.Write(0x4200, 1);
    emu->StepFrame();
    Check(bus.Read(0x421a) == 0xff && bus.Read(0x421b) == 0x80, "Automatic polling reads Scope's eight bits then ones");
    Check((bus.Read(0x4017) & 1) == 1, "Manual reader continues after automatic clocks");
}

void ScopeBeamAfterRead() {
    ControllerPorts ports;
    LightGunState input;
    input.x = 50; input.y = 20; input.offscreen = false; input.trigger = true;
    ports.Configure(1, ControllerDevice::SuperScope);
    ports.SetGunCallback([&](int, int) { return input; });
    unsigned hits = 0;
    ports.SetGunLatchCallback([&](uint16_t, uint16_t) { ++hits; });
    Latch(ports);
    Check(Packet(ports, 8) == 0x80, "Read shot before the beam arrives");
    ports.BeamPosition(89, 21, 224);
    Check(hits == 0, "Beam waits for the aimed dot");
    ports.BeamPosition(90, 21, 224);
    Check(hits == 1, "Consuming the shot packet retains its beam latch");
    ports.BeamPosition(90, 21, 224);
    Check(hits == 1, "One optical pulse per field");
    ports.FrameBegin(); ports.PollLightGuns(); ports.BeamPosition(90, 21, 224);
    Check(hits == 1, "Held normal trigger does not rearm next field");
    input.turbo = true;
    ports.PollLightGuns(); ports.BeamPosition(91, 21, 224);
    Check(hits == 1, "A shot armed after its dot cannot latch retroactively");
    ports.FrameBegin(); ports.PollLightGuns(); ports.BeamPosition(90, 21, 224);
    Check(hits == 2, "Turbo rearms on a following field");
    for (auto position : {std::pair{-1, 20}, std::pair{256, 20}, std::pair{50, -1}, std::pair{50, 224}}) {
        input.x = position.first; input.y = position.second;
        ports.FrameBegin(); ports.PollLightGuns();
        ports.BeamPosition(uint16_t(input.x + 40), uint16_t(input.y + 1), 224);
        Check(hits == 2, "Out-of-range coordinates cannot latch");
    }
}
}
int main() {
    try { ScopePackets(); JustifierPackets(); RiflePackets(); BeamLatches(); GunSelectionAndPorts(); AutomaticPolling(); ScopeBeamAfterRead(); }
    catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
    std::puts("Light gun protocol, beam, and automatic polling checks passed");
}
