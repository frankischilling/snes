// snes emulator
// core/include/snes/core/Trace.hpp
// CPU, PPU, and DMA trace event types.

#pragma once

#include <cstdint>
#include <variant>

namespace snes::core {

struct CpuBusTraceEvent {
    uint64_t cycle = 0;
    uint32_t address = 0;
    uint8_t value = 0;
    bool isWrite = false;
};

struct PpuTraceEvent {
    uint64_t cycle = 0;
    uint16_t registerAddress = 0;
    uint8_t value = 0;
    bool isWrite = false;
};

struct DmaTraceEvent {
    uint64_t cycle = 0;
    uint8_t channel = 0;
    uint32_t sourceAddress = 0;
    uint16_t destinationAddress = 0;
    uint16_t length = 0;
};

using TraceEvent = std::variant<CpuBusTraceEvent, PpuTraceEvent, DmaTraceEvent>;

class ITraceSink {
public:
    virtual ~ITraceSink() = default;
    virtual void OnTrace(const TraceEvent& event) = 0;
};

} // namespace snes::core
