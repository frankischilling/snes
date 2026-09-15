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
    case EnhancementChip::Sdd1:
    case EnhancementChip::St010:
    case EnhancementChip::Spc7110:
    case EnhancementChip::Spc7110Rtc:
    case EnhancementChip::Sa1:
    case EnhancementChip::SuperFx:
    case EnhancementChip::Cx4:
    case EnhancementChip::St011:
    case EnhancementChip::St018:
    case EnhancementChip::Dsp3:
    case EnhancementChip::Dsp4:
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

bool Emulator::LoadBroadcastCartridge(std::span<const uint8_t> base, std::span<const uint8_t> pack,
                                      std::string* error) {
    auto cartridge = Cartridge::FromBroadcastCartridge(base, pack, error);
    if (!cartridge) return false;
    cartridge_ = std::move(*cartridge);
    InitSubsystems();
    return true;
}

bool Emulator::LoadBroadcastCartridgeFromFiles(const std::string& basePath, const std::string& packPath,
                                               std::string* error) {
    const auto base = ReadImage(basePath, error);
    if (!base) return false;
    if (packPath.empty() || packPath == "-") return LoadBroadcastCartridge(*base, {}, error);
    const auto pack = ReadImage(packPath, error);
    return pack && LoadBroadcastCartridge(*base, *pack, error);
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
    dma_.SetCartridge(&*cartridge_);

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
    cpu_->SetInterruptPollCallback([this]() {
        if (irq_.IrqLocked()) return false;
        if (irq_.NmiTest()) cpu_->RequestNmi();
        // Keep the physical IRQ request visible while I is set so WAI wakes.
        const bool timerIrq = irq_.IrqTest(true);
        cpu_->SetIrqLevel(timerIrq || cartridge_->CpuIrqPending());
        return true;
    });

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

    // Start DMA after the current CPU instruction has completed.
    cpuIo_.SetDmaEnableCallback([this](uint8_t channels) {
        dma_.EnableDma(channels);
    });
    dma_.SetClockCallback([this](uint32_t clocks) { AdvanceClocks(clocks); });
    dma_.SetClockQuery([this]() { return timing_.MasterClocksElapsed(); });
    dma_.SetBoundaryCallback([this]() { ServicePendingHdma(); });

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
        autoJoypad_.Ports().SetPio(pio);
    });
    cpuIo_.SetPioInputCallback([this]() { return autoJoypad_.Ports().IoBits(); });
    autoJoypad_.Ports().SetGunLatchCallback([this](uint16_t h, uint16_t v) {
        ppu_.LatchCounters(h, v);
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
        ppu_.SetCurrentHClock(0);
        ppu_.FrameBegin();
        autoJoypad_.FrameBegin();
        autoJoypad_.Ports().FrameBegin();
    };
    timing_.onHdmaSetup = [this]() {
        dma_.HdmaReset();
        pendingHdmaSetup_ = dma_.AnyHdmaEnabled();
    };

    // New scanline
    timing_.onScanline = [this](uint16_t v) {
        ppu_.SetCurrentLine(v);
        ppu_.SetCurrentHClock(0);
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
        pendingHdmaRun_ = dma_.AnyHdmaActive();
    };

    // IRQ/NMI condition polling (every ~4 master clocks)
    timing_.onIrqPoll = [this](uint16_t h, uint16_t v, uint16_t vdisp, uint16_t hperiod) {
        irq_.Poll(h, v, vdisp, hperiod);
        autoJoypad_.Ports().BeamPosition(timing_.HDot(), v, vdisp - 1);
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

    // 6. Multiply/divide clocking during CPU cycles and DRAM refresh.
    cpu_->SetAluStepCallback([this](bool writeCycle) {
        cpuIo_.AluStep(writeCycle);
    });

    // 7. AutoJoypad input source
    autoJoypad_.SetInputCallback([this](int player) -> InputState {
        if (inputProvider_) {
            return inputProvider_->PollController(player, frameIndex_);
        }
        return {};
    });

    autoJoypad_.Ports().SetMouseCallback([this](int port) {
        return inputProvider_ ? inputProvider_->PollMouse(port, frameIndex_) : MouseState{};
    });
    autoJoypad_.Ports().SetGunCallback([this](int port, int gun) {
        return inputProvider_ ? inputProvider_->PollLightGun(port, gun, frameIndex_) : LightGunState{};
    });

    // Manual reads and automatic polling share the physical latch and clocks.
    cpuIo_.SetJoypadLatchCallback([this](bool latch) {
        autoJoypad_.SetManualLatch(latch);
    });

    cpuIo_.SetJoypadDataCallback([this](int port) -> uint8_t {
        return autoJoypad_.Ports().Read(port);
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
    cartridge_->SetPal(pal);
    ppu_.SetPal(pal);
    frameIndex_ = 0;
    masterCycles_ = 0;
    pendingExtraClocks_ = 0;
    pendingHdmaSetup_ = false;
    pendingHdmaRun_ = false;
    servicingHdma_ = false;
    dmaArmed_ = false;
    audioOverflow_.clear();
    collectingAudio_ = false;

    // Set up DSP audio output buffer
    dsp_.SetOutput(audioBuf_.data(), kAudioBufSamples);

    // CPU reset — reads the reset vector from the cartridge
    cpu_->Reset();
    cpu_->SetClockCallback([this](uint32_t clocks) { AdvanceClocks(clocks); });
    cpu_->SetBeforeCycleCallback([this](uint32_t clocks) { BeginCpuCycle(clocks); });

    initialized_ = true;

    Logger::Instance().Write(LogLevel::Info, "Subsystems initialized");
}

// StepFrame — run one complete video frame

FrameStepResult Emulator::StepFrame(const FrameStepOptions& options) {
    // Stub path — no cartridge loaded yet
    if (!initialized_) {
        const auto input = inputProvider_ ? inputProvider_->PollController(0, frameIndex_) : InputState{};

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
    autoJoypad_.Ports().PollLightGuns();
    const auto input = inputProvider_ ? inputProvider_->PollController(0, frameIndex_) : InputState{};

    // Prepare DSP audio buffer for this frame
    audioOverflow_.clear();
    collectingAudio_ = audioOutput_ != nullptr;
    dsp_.ResetSamplesWritten();
    dsp_.SetOutput(audioBuf_.data(), kAudioBufSamples);

    // Frame timing bookkeeping
    const uint64_t frameTarget = timing_.FrameCount() + 1;

    // Each CPU bus access and DMA interval advances the connected hardware.

    while (timing_.FrameCount() < frameTarget) {
        cpu_->Step();
    }
    collectingAudio_ = false;
    // End-of-frame processing

    // RenderFrame is now called from VBlankBegin() — no separate call here.
    // The previous duplicate call consumed V=1 from the next frame, causing
    // alternating 1-line / 223-line renders and incorrect scroll values.

    // Submit audio (convert int16_t → float)
    if (audioOutput_) {
        AudioBuffer audio;
        audio.sampleRate = 32000;
        const int samples = dsp_.SamplesWritten();
        audio.interleavedStereo.reserve(audioOverflow_.size() + static_cast<size_t>(samples) * 2);
        for (int16_t value : audioOverflow_)
            audio.interleavedStereo.push_back(static_cast<float>(value) / 32768.0f);
        for (int i = 0; i < samples * 2; ++i)
            audio.interleavedStereo.push_back(static_cast<float>(audioBuf_[i]) / 32768.0f);
        audioOutput_->Submit(audio);
    }

    // Present video — extract only the visible region from the PPU's
    // internal buffer. Visible rows are written starting at output row 0,
    // so the frontend can copy them directly without preserving the hidden
    // 240-line border area.
    if (videoOutput_) {
        VideoFrame frame;
        frame.width = ppu_.FrameWidth();
        frame.height = ppu_.FrameHeight();
        const uint32_t* src = ppu_.OutputData();
        if (src) {
            frame.pixels.resize(static_cast<size_t>(frame.width) * frame.height);
            for (uint32_t y = 0; y < frame.height; y++) {
                const uint32_t* row = src + static_cast<size_t>(y) * Ppu::OutputWidth;
                std::copy_n(row, frame.width, frame.pixels.data() + y * frame.width);
            }
        } else {
            frame.pixels.resize(static_cast<size_t>(frame.width) * frame.height, 0xFF000000);
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

void Emulator::AdvanceClocks(uint32_t clocks) {
    const auto before = timing_.MasterClocksElapsed();
    timing_.SetInterlace(ppu_.Interlace());
    timing_.Tick(clocks);
    // A penalty can trigger another refresh or HDMA event. Drain all of them
    // before the CPU resumes, including events near the frame boundary.
    while (pendingExtraClocks_ != 0) {
        const auto extra = pendingExtraClocks_;
        pendingExtraClocks_ = 0;
        timing_.Tick(extra);
    }
    ppu_.SetCurrentHClock(timing_.HCounter());
    const auto elapsed = timing_.MasterClocksElapsed();
    cartridge_->AdvanceHardware(static_cast<uint32_t>(elapsed - before));
    smp_.RunUntil(elapsed * Smp::kClockFrequency / timing_.MasterClockHz());
    // A single DMA operation can cross several fields before StepFrame returns.
    // Drain complete blocks during its bus phases so the DSP never discards the
    // rest of that operation's audio when the normal frame buffer fills.
    if (collectingAudio_ && dsp_.SamplesWritten() == kAudioBufSamples) {
        audioOverflow_.insert(audioOverflow_.end(), audioBuf_.begin(), audioBuf_.end());
        dsp_.ResetSamplesWritten();
    }
}

void Emulator::BeginCpuCycle(uint32_t clocks) {
    dma_.SetCpuCycleClocks(clocks);
    // A request first arms the controller. One complete CPU cycle runs before
    // the controller takes the bus at the following cycle boundary.
    if (dmaArmed_) {
        ServicePendingHdma();
        if (dma_.AnyDmaEnabled()) {
            dma_.RunDma();
            irq_.SetIrqLock(true);
        }
        dmaArmed_ = false;
    }
    if (pendingHdmaSetup_ || pendingHdmaRun_ || dma_.AnyDmaEnabled()) dmaArmed_ = true;
    // Release an earlier bus operation's lock before the new access. A write
    // to $4200 can set a fresh lock which must survive until the following
    // access, including the last-cycle sample inside a sixteen-bit store.
    irq_.SetIrqLock(false);
}

void Emulator::ServicePendingHdma() {
    if (servicingHdma_ || dma_.InHdma()) return;
    servicingHdma_ = true;
    while (pendingHdmaSetup_ || pendingHdmaRun_) {
        if (pendingHdmaSetup_) {
            pendingHdmaSetup_ = false;
            if (dma_.AnyHdmaEnabled()) {
                dma_.HdmaSetup();
                irq_.SetIrqLock(true);
            }
        } else {
            pendingHdmaRun_ = false;
            if (dma_.AnyHdmaActive()) {
                dma_.HdmaRun();
                irq_.SetIrqLock(true);
            }
        }
    }
    servicingHdma_ = false;
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
