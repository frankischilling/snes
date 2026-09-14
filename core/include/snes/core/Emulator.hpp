#pragma once
// Emulator.hpp — Top-level SNES system integration
//
// Owns every hardware subsystem and wires them together:
//   CPU (65816)  ↔  MemoryBus  ↔  PPU / APU / DMA / Cartridge
//   Timing → DRAM refresh, HDMA, NMI/IRQ polling, auto-joypad
//   IrqController → NMI / IRQ dispatch to CPU
//   AutoJoypad → controller polling fed into CpuIoRegisters
//   Smp + Dsp → APU audio generation
//
// StepFrame() runs one complete video frame (262 scanlines NTSC) by
// executing CPU instructions, ticking the timing subsystem, synchronising
// the SMP, and dispatching NMI/IRQ.

#include "snes/core/Platform.hpp"
#include "snes/core/Cartridge.hpp"
#include "snes/core/Trace.hpp"
#include "snes/core/Dsp.hpp"
#include "snes/core/Smp.hpp"
#include "snes/core/Ppu.hpp"
#include "snes/core/CpuIoRegisters.hpp"
#include "snes/core/Dma.hpp"
#include "snes/core/IrqController.hpp"
#include "snes/core/AutoJoypad.hpp"
#include "snes/core/Timing.hpp"
#include "snes/core/MemoryBus.hpp"
#include "snes/core/Dsp1.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace snes::core {

class SnesCpu;  // forward — constructed after MemoryBus

struct FrameStepOptions {
    bool emitCpuBusTrace = true;
    bool emitPpuTrace = true;
    bool emitDmaTrace = true;
};

struct FrameStepResult {
    uint64_t frameIndex = 0;
    uint64_t masterCycles = 0;
    InputState latchedInput{};
};

class Emulator {
public:
    struct Config {
        uint32_t frameRate = 60;
        uint32_t visibleWidth = 256;
        uint32_t visibleHeight = 224;
        uint64_t masterCyclesPerFrame = 357368;
    };

    explicit Emulator();
    explicit Emulator(Config config);
    ~Emulator();

    // Non-copyable, non-movable (owns large subsystems by value)
    Emulator(const Emulator&) = delete;
    Emulator& operator=(const Emulator&) = delete;

    void AttachVideoOutput(IVideoOutput* output);
    void AttachAudioOutput(IAudioOutput* output);
    void AttachInputProvider(IInputProvider* input);
    void AddTraceSink(ITraceSink* sink);

    bool LoadCartridge(std::span<const uint8_t> romData, std::string* error = nullptr);
    bool LoadCartridgeFromFile(const std::string& path, std::string* error = nullptr);

    const Cartridge* LoadedCartridge() const noexcept;
    void LoadSram(std::span<const uint8_t> data) { if (cartridge_) cartridge_->LoadSram(data); }

    FrameStepResult StepFrame(const FrameStepOptions& options = {});

    uint64_t CurrentFrame() const noexcept;
    uint64_t CurrentMasterCycles() const noexcept;

    // Subsystem accessors (for testing / debugging)
    Ppu&            GetPpu()      noexcept { return ppu_; }
    Smp&            GetSmp()      noexcept { return smp_; }
    Dsp&            GetDsp()      noexcept { return dsp_; }
    Timing&         GetTiming()   noexcept { return timing_; }
    IrqController&  GetIrq()     noexcept { return irq_; }
    AutoJoypad&     GetAutoJoypad() noexcept { return autoJoypad_; }
    CpuIoRegisters& GetCpuIo()   noexcept { return cpuIo_; }
    DmaController&  GetDma()     noexcept { return dma_; }
    MemoryBus&      GetBus()     noexcept { return bus_; }
    SnesCpu*        GetCpu()     noexcept { return cpu_.get(); }

    bool Initialized() const noexcept { return initialized_; }

private:
    /// Wire all subsystem callbacks and map the bus.
    /// Called from LoadCartridge() once the cartridge is ready.
    void InitSubsystems();

    void EmitTrace(const TraceEvent& event) const;

    Config config_;
    uint64_t frameIndex_ = 0;
    uint64_t masterCycles_ = 0;
    bool initialized_ = false;

    IVideoOutput* videoOutput_ = nullptr;
    IAudioOutput* audioOutput_ = nullptr;
    IInputProvider* inputProvider_ = nullptr;
    std::vector<ITraceSink*> traceSinks_;

    CartridgeDatabase cartridgeDatabase_{};
    std::optional<Cartridge> cartridge_{};

    // Hardware subsystems (construction order matters — bus_ before cpu_)
    Dsp             dsp_;
    Smp             smp_;
    Ppu             ppu_;
    CpuIoRegisters  cpuIo_;
    DmaController   dma_;
    IrqController   irq_;
    AutoJoypad      autoJoypad_;
    Timing          timing_;
    MemoryBus       bus_;
    std::unique_ptr<SnesCpu> cpu_;   // created in InitSubsystems()
    std::unique_ptr<Dsp1> dsp1_;       // created if cartridge uses DSP-1

    // Audio output buffer (stereo interleaved int16_t, enough for 1+ frames)
    static constexpr int kAudioBufSamples = 2048;
    std::array<int16_t, kAudioBufSamples * 2> audioBuf_{};

    // Clock accumulator for DRAM-refresh / DMA / HDMA penalties.
    // Set inside Timing callbacks; drained after each CPU Step().
    uint32_t pendingExtraClocks_ = 0;

    // Serial joypad state (manual $4016/$4017 reads)
    // Mirrors bsnes's Gamepad struct: latch flag, shift counter, latched bits
    struct SerialJoypad {
        bool     latched  = false;   ///< Current latch line state
        uint32_t counter  = 0;       ///< Bit position (0-15, then 16+ = return 1)
        // Latched button states (snapshotted on latch 1→0 transition)
        bool b = false, y = false, select = false, start = false;
        bool up = false, down = false, left = false, right = false;
        bool a = false, x = false, l = false, r = false;
    };
    SerialJoypad serialPad_[2];      ///< Port 0 and Port 1
    bool autoJoypadLatch_ = false;   ///< Auto-joypad latch signal (ORed with CPU latch)
};

} // namespace snes::core
