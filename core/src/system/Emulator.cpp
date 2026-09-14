// snes emulator
// core/src/system/Emulator.cpp
// Top-level subsystem wiring, cartridge loading, and frame execution.

// Emulator.cpp — SNES system integration and frame loop

#include "snes/core/Emulator.hpp"
#include "snes/core/SnesCpu.hpp"
#include "snes/core/Logging.hpp"

#include <fstream>
#include <iterator>
#include <optional>
#include <utility>

namespace snes::core {

namespace {
std::optional<std::vector<uint8_t>> ReadImage(const std::string& path, std::string* error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        if (error) *error = "Unable to open ROM file: " + path;
        return std::nullopt;
    }
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (file.bad() || data.empty()) {
        if (error) *error = "ROM file is empty or unreadable: " + path;
        return std::nullopt;
    }
    return data;
}

std::optional<const char*> UnsupportedChipName(EnhancementChip chip) {
    switch (chip) {
    case EnhancementChip::None:
    case EnhancementChip::Dsp1:
    case EnhancementChip::Dsp2:
    case EnhancementChip::Obc1:
    case EnhancementChip::Srtc:
        return std::nullopt;
    default:
        return ChipName(chip);
    }
}
}

// Construction / destruction

Emulator::Emulator() : config_(Config{}) {
    Logger::Instance().Write(LogLevel::Info, "Emulator initialized");
}

Emulator::Emulator(Config config)
    : config_(std::move(config)) {
    Logger::Instance().Write(LogLevel::Info, "Emulator initialized");
}

Emulator::~Emulator() = default;

// Platform attachment

void Emulator::AttachVideoOutput(IVideoOutput* output) {
    videoOutput_ = output;
}

void Emulator::AttachAudioOutput(IAudioOutput* output) {
    audioOutput_ = output;
}

void Emulator::AttachInputProvider(IInputProvider* input) {
    inputProvider_ = input;
}

void Emulator::AddTraceSink(ITraceSink* sink) {
    if (sink == nullptr) {
        return;
    }
    traceSinks_.push_back(sink);
}

// Cartridge loading

bool Emulator::LoadCartridge(std::span<const uint8_t> romData, std::string* error) {
    RomNormalizationInfo normalization;
    auto cartridge = Cartridge::FromRomImage(romData, &normalization, &cartridgeDatabase_, error);
    if (!cartridge.has_value()) {
        Logger::Instance().Write(LogLevel::Error, "Failed to load cartridge image");
        return false;
    }

    if (auto unsupported = UnsupportedChipName(cartridge->Header().chip);
        unsupported.has_value()) {
        if (error != nullptr) {
            *error = std::string("Unsupported enhancement chip: ") + *unsupported
                   + " (cartridge type $"
                   + "0123456789ABCDEF"[(cartridge->Header().cartridgeType >> 4) & 0x0F]
                   + "0123456789ABCDEF"[(cartridge->Header().cartridgeType >> 0) & 0x0F]
                   + ")";
        }
        Logger::Instance().Write(LogLevel::Error,
                                 "Unsupported cartridge enhancement chip detected");
        return false;
    }

    cartridge_ = std::move(*cartridge);
    Logger::Instance().Write(
        LogLevel::Info,
        "Cartridge loaded: " + cartridge_->Header().title +
            (normalization.hadCopierHeader ? " [copier-header-normalized]" : "") +
            (normalization.hadInterleave ? " [deinterleaved]" : ""));

    // Wire all subsystems now that we have a cartridge
    InitSubsystems();

    return true;
}

bool Emulator::LoadSufamiTurbo(std::span<const uint8_t> bios, std::span<const uint8_t> slotA,
                             std::span<const uint8_t> slotB, std::string* error) {
    auto cartridge = Cartridge::FromSufamiTurbo(bios, slotA, slotB, error);
    if (!cartridge) return false;
    cartridge_ = std::move(*cartridge);
    InitSubsystems();
    return true;
}

bool Emulator::LoadCartridgeFromFile(const std::string& path, std::string* error) {
    const auto data = ReadImage(path, error);
    return data && LoadCartridge(*data, error);
}

bool Emulator::LoadSufamiTurboFromFiles(const std::string& biosPath, const std::string& slotAPath,
                                      const std::string& slotBPath, std::string* error) {
    const auto bios = ReadImage(biosPath, error);
    if (!bios) return false;
    std::array<std::vector<uint8_t>, 2> slots;
    const std::array paths{slotAPath, slotBPath};
    for (unsigned i = 0; i < 2; ++i) {
        if (paths[i].empty() || paths[i] == "-") continue;
        auto image = ReadImage(paths[i], error);
        if (!image) return false;
        slots[i] = std::move(*image);
    }
    return LoadSufamiTurbo(*bios, slots[0], slots[1], error);
}

const Cartridge* Emulator::LoadedCartridge() const noexcept {
    return cartridge_ ? &*cartridge_ : nullptr;
}

// InitSubsystems — wire callbacks, map bus, reset everything

void Emulator::InitSubsystems() {
    // 1. Inter-subsystem connections
    dsp_.SetRam(smp_.Ram());
    smp_.SetDsp(dsp_);
    dma_.SetBus(&bus_);

    // 2. Map the 24-bit address space
    bus_.Reset();
    bus_.MapWram();
    bus_.MapPpu(ppu_);
    bus_.MapApu(smp_);
    bus_.MapCpuIo(cpuIo_);
    bus_.MapDma(dma_);
    bus_.MapCartridge(*cartridge_);

    // Map the DSP-1 data and status windows for the cartridge board.
    dsp1_.reset();
    dsp2_.reset();
    obc1_.reset();
    srtc_.reset();
    const auto chip = cartridge_->Header().chip;
    if (chip == EnhancementChip::Dsp1) {
        dsp1_ = std::make_unique<Dsp1>();
        auto* dsp1ptr = dsp1_.get();
        const bool loRom = cartridge_->Header().mapping == MappingType::LoRom;
        const bool largeLoRom = loRom && cartridge_->RomData().size() > 0x100000;
        const uint8_t bankLo = loRom ? (largeLoRom ? 0x60 : 0x20) : 0x00;
        const uint8_t bankHi = loRom ? (largeLoRom ? 0x6F : 0x3F) : 0x1F;
        const uint16_t addrLo = loRom ? (largeLoRom ? 0x0000 : 0x8000) : 0x6000;
        const uint16_t addrHi = loRom && !largeLoRom ? 0xFFFF : 0x7FFF;
        const uint16_t statusStart = loRom ? (largeLoRom ? 0x4000 : 0xC000) : 0x7000;
        auto dsp1Read = [dsp1ptr, statusStart](uint32_t addr, uint8_t /*openBus*/) -> uint8_t {
            return (addr & 0xFFFF) >= statusStart ? dsp1ptr->GetSr() : dsp1ptr->GetDr();
        };
        auto dsp1Write = [dsp1ptr, statusStart](uint32_t addr, uint8_t data) {
            if ((addr & 0xFFFF) < statusStart)
                dsp1ptr->SetDr(data);
        };
        uint8_t slot = bus_.RegisterHandler(dsp1Read, dsp1Write);
        bus_.MapRange(bankLo, bankHi, addrLo, addrHi, slot);
        bus_.MapRange(bankLo | 0x80, bankHi | 0x80, addrLo, addrHi, slot);
    }

    if (chip == EnhancementChip::Dsp2) {
        dsp2_ = std::make_unique<Dsp2>();
        const auto slot = bus_.RegisterHandler(
            [this](uint32_t, uint8_t) { return dsp2_->Read(); },
            [this](uint32_t, uint8_t data) { dsp2_->Write(data); });
        for (uint8_t bank : {0x20, 0xa0}) {
            bus_.MapRange(bank, bank + 0x1f, 0x6000, 0x6fff, slot);
            bus_.MapRange(bank, bank + 0x1f, 0x8000, 0xbfff, slot);
        }
    }
    if (chip == EnhancementChip::Obc1) {
        obc1_ = std::make_unique<Obc1>();
        const auto slot = bus_.RegisterHandler(
            [this](uint32_t address, uint8_t) { return obc1_->Read(static_cast<uint16_t>(address)); },
            [this](uint32_t address, uint8_t data) { obc1_->Write(static_cast<uint16_t>(address), data); });
        bus_.MapRange(0x00, 0x3f, 0x6000, 0x7fff, slot);
        bus_.MapRange(0x80, 0xbf, 0x6000, 0x7fff, slot);
    }

    if (chip == EnhancementChip::Srtc) {
        srtc_ = std::make_unique<Srtc>();
        const auto slot = bus_.RegisterHandler(
            [this](uint32_t address, uint8_t openBus) {
                return (address & 1) ? openBus : srtc_->Read();
            },
            [this](uint32_t address, uint8_t data) { if (address & 1) srtc_->Write(data); });
        bus_.MapRange(0x00, 0x3f, 0x2800, 0x2801, slot);
        bus_.MapRange(0x80, 0xbf, 0x2800, 0x2801, slot);
    }

    // 3. Create the 65816 CPU (needs bus_ reference)
    cpu_ = std::make_unique<SnesCpu>(bus_);

    // 4. Wire CpuIoRegisters → subsystem callbacks

    // $4200 NMITIMEN → IrqController + AutoJoypad
    cpuIo_.SetNmitimenCallback([this](uint8_t data) {
        irq_.NmitimenUpdate(data);
        autoJoypad_.SetAutoJoypadPoll((data & 0x01) != 0);
    });

    // $420D MEMSEL → MemoryBus fast-ROM + Cartridge access timing
    cpuIo_.SetMemselCallback([this](bool fast) {
        bus_.SetFastRom(fast);
        if (cartridge_) {
            cartridge_->SetMemselFast(fast);
        }
    });

    // $420B MDMAEN → DMA controller (run immediately, accumulate clocks)
    cpuIo_.SetDmaEnableCallback([this](uint8_t channels) {
        dma_.EnableDma(channels);
        pendingExtraClocks_ += dma_.RunDma();
    });

    // $420C HDMAEN → DMA controller
    cpuIo_.SetHdmaEnableCallback([this](uint8_t channels) {
        dma_.EnableHdma(channels);
    });

    // PPU counter latch ($4201 WRIO bit-7 falling edge)
    cpuIo_.SetPpuLatchCallback([this]() {
        ppu_.LatchCounters(timing_.HDot(), timing_.VCounter());
    });
    cpuIo_.SetPioCallback([this](uint8_t pio) {
        ppu_.SetCpuPio(pio);
    });

    // PPU counter latch ($2137 SLHV read when PIO bit 7 is high)
    ppu_.SetCounterLatchCallback([this]() {
        ppu_.LatchCounters(timing_.HDot(), timing_.VCounter());
    });

    // SETINI changes the VBlank boundary used by rendering, DMA and CPU I/O.
    ppu_.SetVDispCallback([this](uint16_t vdisp) {
        timing_.SetVDisp(vdisp);
    });

    // HVBJOY ($4212) timing query
    cpuIo_.SetTimingQueryCallback([this]() -> TimingQuery {
        return TimingQuery{
            timing_.HCounter(),
            timing_.VCounter(),
            timing_.VDisp(),
        };
    });

    // $4210 RDNMI → IrqController
    cpuIo_.SetRdnmiCallback([this]() -> bool {
        return irq_.Rdnmi();
    });

    // $4211 TIMEUP → IrqController
    cpuIo_.SetTimeupCallback([this]() -> bool {
        return irq_.Timeup();
    });

    // $4207-$420A H/V timer target writes → IrqController
    cpuIo_.SetHVTimeChangeCallback([this](uint16_t htime, uint16_t vtime) {
        irq_.SetHTime(htime);
        irq_.SetVTime(vtime);
    });

    // 5. Wire Timing → subsystem callbacks

    // V=0, H=0 — start of frame
    timing_.onFrameBegin = [this]() {
        ppu_.SetCurrentLine(0);
        ppu_.SetCurrentDot(0);
        ppu_.FrameBegin();
        autoJoypad_.FrameBegin();
        dma_.HdmaReset();
        if (dma_.AnyHdmaEnabled()) {
            pendingExtraClocks_ += dma_.HdmaSetup();
        }
    };

    // New scanline
    timing_.onScanline = [this](uint16_t v) {
        ppu_.SetCurrentLine(v);
        ppu_.SetCurrentDot(0);
    };

    // Cache visible scanlines at the render sampling point rather than H=0.
    timing_.onRenderCycle = [this](uint16_t v) {
        ppu_.ScanlineBegin(v);
    };

    // VBlank start (V = VDisp)
    timing_.onVBlankBegin = [this]() {
        ppu_.VBlankBegin();
    };

    // NMI assertion point — handled indirectly by IrqController::Poll
    timing_.onNmiPoint = nullptr;

    // DRAM refresh — steal 40 master clocks from the CPU
    timing_.onDramRefresh = [this]() {
        cpu_->ApplyDramRefreshPenalty();
        pendingExtraClocks_ += Timing::kDramRefreshClocks;
    };

    // HDMA per visible scanline
    timing_.onHdmaTransfer = [this](uint16_t /*v*/) {
        if (dma_.AnyHdmaActive()) {
            pendingExtraClocks_ += dma_.HdmaRun();
        }
    };

    // IRQ/NMI condition polling (every ~4 master clocks)
    timing_.onIrqPoll = [this](uint16_t h, uint16_t v, uint16_t vdisp, uint16_t hperiod) {
        irq_.Poll(h, v, vdisp, hperiod);
    };

    // Auto-joypad polling (every 128 master clocks)
    timing_.onJoypadPoll = [this](uint16_t h, uint16_t v, uint16_t vdisp) {
        autoJoypad_.Tick128(h, v, vdisp);
        // Synchronize joy registers into CpuIoRegisters
        cpuIo_.setJoy1(autoJoypad_.Joy1());
        cpuIo_.setJoy2(autoJoypad_.Joy2());
        cpuIo_.setJoy3(autoJoypad_.Joy3());
        cpuIo_.setJoy4(autoJoypad_.Joy4());
        cpuIo_.setAutoJoypadCounter(autoJoypad_.Counter());
    };

    // HBlank callback (not currently needed, but available)
    timing_.onHBlank = nullptr;

    // 6. CPU ALU step (multiply/divide hardware pipelining during DRAM refresh)
    cpu_->SetAluStepCallback([this]() {
        cpuIo_.AluStep();
    });

    // 7. AutoJoypad input source
    autoJoypad_.SetInputCallback([this](int /*port*/) -> InputState {
        if (inputProvider_) {
            return inputProvider_->Poll(frameIndex_);
        }
        return {};
    });

    // 8. Serial joypad ($4016/$4017) — manual strobe/read support
    //
    // Many games (DKC, etc.) read controllers via $4016/$4017 serial
    // protocol in addition to or instead of auto-joypad ($4218-$421F).
    // Implements bsnes Gamepad::latch() + Gamepad::data() behavior.
    cpuIo_.SetJoypadLatchCallback([this](bool latch) {
        // bsnes: latch signal is CPU latch OR auto-joypad latch.
        // The auto-joypad state machine sets autoJoypadLatch_ separately.
        // Here we handle the CPU-side write to $4016 bit 0.
        for (int port = 0; port < 2; port++) {
            bool effectiveLatch = latch | autoJoypadLatch_;
            auto& pad = serialPad_[port];
            if (pad.latched == effectiveLatch) continue;
            pad.latched = effectiveLatch;
            pad.counter = 0;

            if (!effectiveLatch) {
                // Latch released (1→0): snapshot button states
                InputState input;
                if (inputProvider_) {
                    input = inputProvider_->Poll(frameIndex_);
                }
                pad.b      = input.b;
                pad.y      = input.y;
                pad.select = input.select;
                pad.start  = input.start;
                pad.up     = input.up   && !input.down;
                pad.down   = input.down && !input.up;
                pad.left   = input.left && !input.right;
                pad.right  = input.right && !input.left;
                pad.a      = input.a;
                pad.x      = input.x;
                pad.l      = input.l;
                pad.r      = input.r;
            }
        }
    });

    cpuIo_.SetJoypadDataCallback([this](int port) -> uint8_t {
        auto& pad = serialPad_[port & 1];
        if (pad.counter >= 16) return 1;
        if (pad.latched) {
            // While latched, always return current state of first button (B)
            InputState input;
            if (inputProvider_) {
                input = inputProvider_->Poll(frameIndex_);
            }
            return input.b ? 1 : 0;
        }

        bool bit = false;
        switch (pad.counter++) {
        case  0: bit = pad.b;      break;
        case  1: bit = pad.y;      break;
        case  2: bit = pad.select; break;
        case  3: bit = pad.start;  break;
        case  4: bit = pad.up;     break;
        case  5: bit = pad.down;   break;
        case  6: bit = pad.left;   break;
        case  7: bit = pad.right;  break;
        case  8: bit = pad.a;      break;
        case  9: bit = pad.x;      break;
        case 10: bit = pad.l;      break;
        case 11: bit = pad.r;      break;
        default: bit = false;      break;  // Bits 12-15: signature (0)
        }
        return bit ? 1 : 0;
    });

    // 9. Reset all subsystems to power-on state
    dsp_.Power();
    smp_.Power();
    ppu_.Reset();
    cpuIo_.Reset();
    dma_.Reset();
    irq_.Reset();
    autoJoypad_.Reset();
    timing_.Reset();

    const auto country = cartridge_->Header().country;
    const bool pal = (country >= 2 && country <= 12) || country == 18;
    timing_.SetRegion(pal ? Region::PAL : Region::NTSC);
    ppu_.SetPal(pal);
    frameIndex_ = 0;
    masterCycles_ = 0;
    pendingExtraClocks_ = 0;

    // Reset serial joypad state
    for (auto& pad : serialPad_) {
        pad = SerialJoypad{};
    }
    autoJoypadLatch_ = false;

    // Set up DSP audio output buffer
    dsp_.SetOutput(audioBuf_.data(), kAudioBufSamples);

    // CPU reset — reads the reset vector from the cartridge
    cpu_->Reset();

    initialized_ = true;

    Logger::Instance().Write(LogLevel::Info, "Subsystems initialized");
}

// StepFrame — run one complete video frame

FrameStepResult Emulator::StepFrame(const FrameStepOptions& options) {
    // Stub path — no cartridge loaded yet
    if (!initialized_) {
        const auto input = inputProvider_ ? inputProvider_->Poll(frameIndex_) : InputState{};

        if (options.emitCpuBusTrace) {
            EmitTrace(CpuBusTraceEvent{
                .cycle = masterCycles_,
                .address = 0x00FFFC,
                .value = 0,
                .isWrite = false,
            });
        }
        if (options.emitPpuTrace) {
            EmitTrace(PpuTraceEvent{
                .cycle = masterCycles_,
                .registerAddress = 0x2100,
                .value = 0,
                .isWrite = false,
            });
        }
        if (options.emitDmaTrace) {
            EmitTrace(DmaTraceEvent{
                .cycle = masterCycles_,
                .channel = 0,
                .sourceAddress = 0x7E0000,
                .destinationAddress = 0x2118,
                .length = 0,
            });
        }

        if (videoOutput_) {
            VideoFrame frame;
            frame.width = config_.visibleWidth;
            frame.height = config_.visibleHeight;
            frame.pixels.resize(
                static_cast<size_t>(frame.width) * frame.height, 0xFF000000);
            videoOutput_->Present(frame);
        }
        if (audioOutput_) {
            AudioBuffer audio;
            audio.sampleRate = 32000;
            audioOutput_->Submit(audio);
        }

        masterCycles_ += config_.masterCyclesPerFrame;
        ++frameIndex_;

        return FrameStepResult{
            .frameIndex = frameIndex_,
            .masterCycles = masterCycles_,
            .latchedInput = input,
        };
    }

    // Real frame loop
    const auto input = inputProvider_ ? inputProvider_->Poll(frameIndex_) : InputState{};

    // Prepare DSP audio buffer for this frame
    dsp_.ResetSamplesWritten();
    dsp_.SetOutput(audioBuf_.data(), kAudioBufSamples);

    // Frame timing bookkeeping
    const uint64_t frameStart  = timing_.MasterClocksElapsed();
    const uint32_t frameTarget = timing_.MasterClocksThisFrame();
    pendingExtraClocks_ = 0;

    // Main loop: execute CPU → tick timing → sync SMP → NMI/IRQ

    while (timing_.MasterClocksElapsed() - frameStart < frameTarget) {
        // 1. Execute one 65816 instruction
        const uint32_t cpuClocks = cpu_->Step();


        // 2. Advance timing by the instruction's master clocks.
        //    This may fire onDramRefresh / onHdmaTransfer / onIrqPoll etc.
        //    which set pendingExtraClocks_.
        timing_.Tick(cpuClocks);

        // 3. Drain accumulated DRAM / DMA / HDMA penalty clocks through timing
        //    so that H/V counters stay consistent with real elapsed time.
        if (pendingExtraClocks_ > 0) {
            const uint32_t extra = pendingExtraClocks_;
            pendingExtraClocks_ = 0;
            timing_.Tick(extra);
        }

        // 4. Synchronize the SMP to the current master-clock elapsed time.
        //    SMP target = elapsed × (1,024,000 / 21,477,272)
        const uint64_t elapsed   = timing_.MasterClocksElapsed();
        const uint64_t smpTarget = elapsed * Smp::kClockFrequency
                                   / timing_.MasterClockHz();
        smp_.RunUntil(smpTarget);

        // 5. NMI / IRQ dispatch (matches bsnes lastCycle() logic)
        if (!irq_.IrqLocked()) {
            if (irq_.NmiTest()) {
                cpu_->RequestNmi();
            }
            cpu_->SetIrqLevel(irq_.IrqTest(!cpu_->flagI()));
        } else {
            irq_.SetIrqLock(false);
        }
    }


    // End-of-frame processing

    // RenderFrame is now called from VBlankBegin() — no separate call here.
    // The previous duplicate call consumed V=1 from the next frame, causing
    // alternating 1-line / 223-line renders and incorrect scroll values.

    // Submit audio (convert int16_t → float)
    if (audioOutput_) {
        AudioBuffer audio;
        audio.sampleRate = 32000;
        const int samples = dsp_.SamplesWritten();
        if (samples > 0) {
            audio.interleavedStereo.resize(static_cast<size_t>(samples) * 2);
            for (int i = 0; i < samples * 2; ++i) {
                audio.interleavedStereo[i] =
                    static_cast<float>(audioBuf_[i]) / 32768.0f;
            }
        }
        audioOutput_->Submit(audio);
    }

    // Present video — extract only the visible region from the PPU's
    // internal buffer. Visible rows are written starting at output row 0,
    // so the frontend can copy them directly without preserving the hidden
    // 240-line border area.
    if (videoOutput_) {
        VideoFrame frame;
        const bool overscan = ppu_.FrameOverscan();
        const int visH   = overscan ? 239 : 224;
        const int startY = 0;
        frame.width  = 256;
        frame.height = static_cast<uint32_t>(visH);
        const uint32_t* src = ppu_.OutputData();
        if (src) {
            frame.pixels.resize(static_cast<size_t>(256) * visH);
            for (int y = 0; y < visH; y++) {
                const uint32_t* row = src + static_cast<size_t>(startY + y) * 512;
                std::copy(row, row + 256, frame.pixels.data() + y * 256);
            }
        } else {
            frame.pixels.resize(static_cast<size_t>(256) * visH, 0xFF000000);
        }
        videoOutput_->Present(frame);
    }

    // Update counters
    masterCycles_ = timing_.MasterClocksElapsed();
    ++frameIndex_;

    return FrameStepResult{
        .frameIndex   = frameIndex_,
        .masterCycles = masterCycles_,
        .latchedInput = input,
    };
}

// Accessors

uint64_t Emulator::CurrentFrame() const noexcept {
    return frameIndex_;
}

uint64_t Emulator::CurrentMasterCycles() const noexcept {
    return masterCycles_;
}

void Emulator::EmitTrace(const TraceEvent& event) const {
    for (auto* sink : traceSinks_) {
        sink->OnTrace(event);
    }
}

} // namespace snes::core
