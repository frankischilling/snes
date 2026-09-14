// snes emulator
// tools/src/trace_dump.cpp
// Command-line CPU, PPU, and DMA trace export.

#include "snes/core/Emulator.hpp"

#include <iostream>

namespace {

class ConsoleTraceSink final : public snes::core::ITraceSink {
public:
    void OnTrace(const snes::core::TraceEvent& event) override {
        std::visit([this](const auto& e) { Print(e); }, event);
    }

private:
    void Print(const snes::core::CpuBusTraceEvent& event) {
        std::cout << "[CPU] cycle=" << event.cycle
                  << " addr=0x" << std::hex << event.address << std::dec
                  << " val=" << static_cast<unsigned>(event.value)
                  << " write=" << event.isWrite << '\n';
    }

    void Print(const snes::core::PpuTraceEvent& event) {
        std::cout << "[PPU] cycle=" << event.cycle
                  << " reg=0x" << std::hex << event.registerAddress << std::dec
                  << " val=" << static_cast<unsigned>(event.value)
                  << " write=" << event.isWrite << '\n';
    }

    void Print(const snes::core::DmaTraceEvent& event) {
        std::cout << "[DMA] cycle=" << event.cycle
                  << " ch=" << static_cast<unsigned>(event.channel)
                  << " src=0x" << std::hex << event.sourceAddress
                  << " dst=0x" << event.destinationAddress << std::dec
                  << " len=" << event.length << '\n';
    }
};

} // namespace

int main() {
    snes::core::Emulator emulator;

    ConsoleTraceSink sink;
    emulator.AddTraceSink(&sink);

    emulator.StepFrame();
    return 0;
}
