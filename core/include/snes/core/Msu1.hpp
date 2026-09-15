// snes emulator
// File-backed MSU-1 data registers and stereo PCM playback.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>

namespace snes::core {

class Msu1 {
public:
    using StereoSample = std::array<int16_t, 2>;
    static constexpr uint32_t SampleRate = 44100;

    // A ROM named game.sfc uses game.msu and game-<track>.pcm beside it.
    // An empty data file is valid. Failure closes any previously loaded pack.
    bool Open(const std::filesystem::path& romPath, std::string* error = nullptr);
    void Close();
    void Reset(); // Keep the data file attached, but reset all device registers.
    bool IsOpen() const noexcept { return data_.IsOpen(); }

    static constexpr bool Selects(uint32_t address) noexcept {
        return address <= 0xffffff && (address & 0x400000) == 0 &&
               (address & 0xfff8) == 0x2000;
    }
    uint8_t Read(uint32_t address, uint8_t openBus = 0xff);
    void Write(uint32_t address, uint8_t value);

    // Advance one output frame, even when the host is not collecting audio.
    // Samples are averaged over its exact 44,100/outputRate PCM interval.
    // Fractional frames persist across calls; zero rate does not advance time.
    // Mix the returned sample with the DSP at the same emulated instant.
    StereoSample NextSample(uint32_t outputRate = 32000);

private:
    // Each file has a fixed cache. Neither file size nor seek distance changes
    // memory consumption, including data seeks past the signed 32-bit range.
    class Stream {
    public:
        bool Open(const std::filesystem::path& path);
        void Close();
        bool Read(uint64_t offset, std::span<uint8_t> destination);
        bool IsOpen() const noexcept { return file_.is_open(); }
        uint64_t Size() const noexcept { return size_; }
    private:
        std::ifstream file_;
        std::array<uint8_t, 8192> cache_{};
        uint64_t size_ = 0;
        uint64_t cacheOffset_ = 0;
        size_t cacheSize_ = 0;
    };

    struct ResumePoint {
        bool valid = false;
        uint16_t track = 0;
        uint64_t nextFrame = 0;
        StereoSample sample{};
        uint32_t ticksLeft = 0;
        uint32_t outputRate = 0;
    };

    void SelectTrack();
    bool ReadFrame();

    static constexpr uint8_t AudioError = 0x08;
    static constexpr uint8_t Playing = 0x10;
    static constexpr uint8_t Repeating = 0x20;

    Stream data_;
    Stream pcm_;
    std::filesystem::path basePath_;
    uint32_t dataSeek_ = 0;
    uint64_t dataOffset_ = 0;
    uint16_t trackSeek_ = 0;
    uint16_t track_ = 0;
    uint8_t status_ = 0;
    uint8_t volume_ = 0;
    uint64_t frameCount_ = 0;
    uint64_t nextFrame_ = 0;
    uint64_t loopFrame_ = 0;
    StereoSample sample_{};
    uint32_t ticksLeft_ = 0;
    uint32_t outputRate_ = 0;
    ResumePoint resume_{};
};

} // namespace snes::core
