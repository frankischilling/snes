// BS-X BIOS cartridge controller and Satellaview register interface.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <span>

namespace snes::core {

struct BroadcastTime {
    uint16_t year = 2000;
    uint8_t month = 1, day = 1, weekday = 7; // Sunday=1 through Saturday=7.
    uint8_t hour = 0, minute = 0, second = 0;
};

class Bsx {
public:
    // BIOS, memory pack and battery RAM remain owned by the cartridge. Their
    // allocations must stay stable for this device's lifetime. PSRAM is owned here.
    Bsx(std::span<const uint8_t> bios, std::span<uint8_t> memoryPack, std::span<uint8_t> sram);
    ~Bsx();
    Bsx(Bsx&&) noexcept;
    Bsx& operator=(Bsx&&) noexcept;
    Bsx(const Bsx&) = delete;
    Bsx& operator=(const Bsx&) = delete;

    // Reset the controller/protocol without erasing any RAM or flash. Injected
    // broadcast assets and the time provider remain installed.
    void Reset();
    static bool Selects(uint32_t cpuAddress) noexcept;
    uint8_t ReadCpu(uint32_t cpuAddress, uint8_t openBus = 0xff);
    void WriteCpu(uint32_t cpuAddress, uint8_t value);
    bool CpuIrqPending() const noexcept;
    std::span<uint8_t> Psram() noexcept;

    // Both receivers independently select these logical channels. Payloads are
    // copied; each has ceil(size/22) packets. At the end of a sequence, a missing
    // following asset restarts sequence zero. An empty payload removes that asset.
    // Channel zero is reserved for the clock. False means invalid channel/size.
    bool LoadStream(uint16_t channel, uint8_t sequence, std::span<const uint8_t> payload);
    void ClearStreams();

    // No filesystem, network or wall clock is consulted by the device. With no
    // provider the clock stays at 2000-01-01 00:00:00. Each 23-byte clock record
    // captures one provider result, shared between the two receiver data ports.
    void SetTimeSource(std::function<BroadcastTime()> source);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace snes::core
