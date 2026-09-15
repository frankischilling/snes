#include "snes/core/Msu1.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winioctl.h>
#endif

using snes::core::Msu1;
using Sample = Msu1::StereoSample;

namespace {

void Check(bool condition, const char* description) {
    if (!condition) throw std::runtime_error(description);
}

// All fixtures live under the test's working directory. Only paths created
// by this fixture are removed, and streams in each test die before it does.
class Fixture {
public:
    explicit Fixture(const std::filesystem::path& name = "game.sfc") {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        for (unsigned attempt = 0; attempt < 64; ++attempt) {
            auto candidate = std::filesystem::current_path() /
                ("msu1-fixture-" + std::to_string(stamp) + "-" + std::to_string(attempt));
            std::error_code error;
            if (std::filesystem::create_directory(candidate, error)) {
                directory_ = std::move(candidate);
                break;
            }
            if (error) throw std::filesystem::filesystem_error("Create MSU-1 fixture", candidate, error);
        }
        Check(!directory_.empty(), "A unique fixture directory can be created");
        rom = directory_ / name;
    }

    ~Fixture() {
        std::error_code error;
        for (const auto& path : files_) std::filesystem::remove(path, error);
        std::filesystem::remove(directory_, error);
    }

    std::filesystem::path DataPath() const {
        auto result = rom;
        result.replace_extension(".msu");
        return result;
    }

    std::filesystem::path TrackPath(uint16_t track) const {
        auto result = rom;
        result.replace_extension();
        result += "-" + std::to_string(track) + ".pcm";
        return result;
    }

    void Data(std::span<const uint8_t> bytes = {}) { Write(DataPath(), bytes); }

    void RawTrack(uint16_t track, std::span<const uint8_t> bytes) {
        Write(TrackPath(track), bytes);
    }

    void Pcm(uint16_t track, std::span<const Sample> frames, uint32_t loop = 0) {
        std::vector<uint8_t> bytes(8 + frames.size() * 4);
        bytes[0] = 'M'; bytes[1] = 'S'; bytes[2] = 'U'; bytes[3] = '1';
        for (unsigned byte = 0; byte < 4; ++byte)
            bytes[4 + byte] = static_cast<uint8_t>(loop >> (byte * 8));
        for (size_t frame = 0; frame < frames.size(); ++frame) {
            for (unsigned channel = 0; channel < 2; ++channel) {
                const auto bits = static_cast<uint16_t>(frames[frame][channel]);
                bytes[8 + frame * 4 + channel * 2] = static_cast<uint8_t>(bits);
                bytes[9 + frame * 4 + channel * 2] = static_cast<uint8_t>(bits >> 8);
            }
        }
        RawTrack(track, bytes);
    }

    void SparseData() {
        static_assert(sizeof(std::streamoff) >= 8);
        const std::array<uint8_t, 1> first{0x19};
        Data(first);
#ifdef _WIN32
        // Mark the file sparse before extending it: the test must never
        // allocate four gigabytes of physical disk space on Windows.
        const auto path = DataPath();
        const HANDLE handle = CreateFileW(path.c_str(), GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL, nullptr);
        Check(handle != INVALID_HANDLE_VALUE, "Open sparse fixture handle");
        DWORD returned = 0;
        const bool sparse = DeviceIoControl(handle, FSCTL_SET_SPARSE, nullptr, 0,
            nullptr, 0, &returned, nullptr) != 0;
        CloseHandle(handle);
        Check(sparse, "Mark large fixture sparse before extending it");
#endif
        std::fstream file(DataPath(), std::ios::binary | std::ios::in | std::ios::out);
        file.seekp(static_cast<std::streamoff>(0x80000003ull));
        file.put(static_cast<char>(0x5a));
        file.seekp(static_cast<std::streamoff>(0xffffffffull));
        file.put(static_cast<char>(0xa5));
        file.close();
        Check(!file.fail(), "Write sparse data sentinels");
        Check(std::filesystem::file_size(DataPath()) == 0x100000000ull,
              "Sparse fixture reaches the end of the 32-bit seek space");
    }

    std::filesystem::path rom;

private:
    void Write(const std::filesystem::path& path, std::span<const uint8_t> bytes) {
        if (std::find(files_.begin(), files_.end(), path) == files_.end()) files_.push_back(path);
        std::ofstream file(path, std::ios::binary | std::ios::trunc);
        Check(file.is_open(), "Open generated fixture");
        if (!bytes.empty())
            file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        file.close();
        Check(!file.fail(), "Write generated fixture");
    }

    std::filesystem::path directory_;
    std::vector<std::filesystem::path> files_;
};

void Track(Msu1& chip, uint16_t track) {
    chip.Write(0x2004, static_cast<uint8_t>(track));
    chip.Write(0x2005, static_cast<uint8_t>(track >> 8));
}

void Seek(Msu1& chip, uint32_t offset) {
    for (unsigned byte = 0; byte < 4; ++byte)
        chip.Write(0x2000 + byte, static_cast<uint8_t>(offset >> (byte * 8)));
}

void Play(Msu1& chip, uint16_t track, uint8_t control = 1, uint8_t volume = 255) {
    Track(chip, track);
    chip.Write(0x2006, volume);
    chip.Write(0x2007, control);
}

void DiscoveryMappingAndLifecycle() {
    Fixture fixture;
    Msu1 chip;
    Check(!chip.IsOpen() && chip.Read(0x2002, 0x73) == 0x73, "Absent device leaves open bus unchanged");
    chip.Write(0x2007, 3);
    Check(chip.NextSample() == Sample{}, "Absent device produces silence");
    std::string error;
    Check(!chip.Open(fixture.rom, &error) && !error.empty(), "Missing data file reports discovery failure");
    fixture.Data();
    const std::array frames{Sample{1200, -2400}, Sample{3600, -4800}};
    fixture.Pcm(0, frames);
    Check(chip.Open(fixture.rom, &error) && error.empty(), "Empty data file enables an audio-only pack");
    Check(chip.Read(0x2000) == 0x02 && chip.Read(0x2001) == 0,
          "Revision two is ready immediately and empty data reads zero");
    const std::string signature = "S-MSU1";
    for (uint32_t bank = 0; bank < 256; ++bank) {
        const bool mapped = (bank & 0x40) == 0;
        for (uint32_t port = 0; port < 8; ++port) {
            const auto address = (bank << 16) | 0x2000 | port;
            Check(Msu1::Selects(address) == mapped, "All and only the documented banks select MSU-1");
            if (port >= 2)
                Check(chip.Read(address, 0x6c) == (mapped ? signature[port - 2] : 0x6c),
                      "Identification signature is mirrored through both I/O bank ranges");
        }
        Check(!Msu1::Selects((bank << 16) | 0x1fff) &&
              !Msu1::Selects((bank << 16) | 0x2008), "Register window has exact boundaries");
    }
    Check(!Msu1::Selects(0x1002002) && chip.Read(0x1002002, 0x6c) == 0x6c,
          "Addresses beyond the CPU address space do not alias the device");
    chip.Write(0x402004, 99);
    chip.Write(0x402005, 99);
    Check(chip.Read(0x2000) == 2, "Unmapped writes do not select tracks");

    Play(chip, 0);
    Check(chip.NextSample(44100) == frames[0], "Track zero starts at PCM data instead of its header");
    chip.Write(0x2007, 4);
    chip.Reset();
    Check(chip.IsOpen() && chip.Read(0x2000) == 2, "Reset retains attachment and clears all status flags");
    Track(chip, 0);
    chip.Write(0x2007, 1);
    Check(chip.NextSample(44100) == Sample{}, "Reset clears volume and continues silent playback");
    Play(chip, 0);
    Check(chip.NextSample(44100) == frames[0], "Reset invalidates the saved resume point");
    chip.Close();
    Check(!chip.IsOpen() && chip.Read(0x2002, 0x91) == 0x91 && chip.NextSample() == Sample{},
          "Close removes both data and audio state");
    Check(chip.Open(fixture.rom), "Pack can reopen after close");
    Play(chip, 0);
    Check(!chip.Open(fixture.rom.parent_path() / "missing.sfc") && !chip.IsOpen(),
          "Failed replacement closes the preceding pack");
    Check(!chip.Open({}) && !chip.IsOpen(), "An empty ROM path cannot discover an unrelated pack");
}

void DataLatchesAndCacheBoundaries() {
    Fixture fixture;
    std::vector<uint8_t> bytes(20013);
    for (size_t index = 0; index < bytes.size(); ++index)
        bytes[index] = static_cast<uint8_t>(index * 29 + 7);
    fixture.Data(bytes);
    Msu1 chip;
    Check(chip.Open(fixture.rom), "Open generated data stream");
    Check(chip.Read(0x2001) == bytes[0], "Data starts at offset zero");
    chip.Write(0x2000, 0x34);
    chip.Write(0x2001, 0x12);
    chip.Write(0x2002, 0);
    Check(chip.Read(0x2001) == bytes[1], "Low seek bytes only update the latch");
    chip.Write(0x802003, 0);
    Check(chip.Read(0x3f2001) == bytes[0x1234], "High seek byte commits the shared 32-bit address");
    Seek(chip, 0);
    for (const auto byte : bytes)
        Check(chip.Read(0x2001) == byte, "Sequential reads cross multiple fixed cache boundaries");
    Check(chip.Read(0x2001) == 0 && chip.Read(0x2001) == 0, "Repeated reads at EOF return zero");
    Seek(chip, 0xffffffff);
    Check(chip.Read(0x2001) == 0 && chip.Read(0x2000) == 2, "Seek past EOF completes without busy state");
    Seek(chip, 8191);
    for (size_t index = 8191; index < 8205; ++index)
        Check(chip.Read(0x2001) == bytes[index], "Seeking recovers after EOF and across cache boundaries");
    chip.Reset();
    Check(chip.Read(0x2001) == bytes[0], "Reset rewinds the data position");
}

void SparseFourGigabyteData() {
    Fixture fixture;
    fixture.SparseData();
    Msu1 chip;
    Check(chip.Open(fixture.rom), "Open a sparse four-gigabyte data file");
    Seek(chip, 0x80000003);
    Check(chip.Read(0x2001) == 0x5a, "Data seek above two gigabytes is unsigned and 64-bit safe");
    Check(chip.Read(0x2001) == 0, "Sparse holes read as data without allocating their span");
    Seek(chip, 0xffffffff);
    Check(chip.Read(0x2001) == 0xa5, "All four seek bytes address the final byte of four gigabytes");
    Check(chip.Read(0x2001) == 0 && chip.Read(0x2001) == 0,
          "Sequential EOF after the maximum seek does not wrap to byte zero");
    Seek(chip, 0);
    Check(chip.Read(0x2001) == 0x19, "A small seek works after the largest possible seek");
}

void TrackValidationAndSelection() {
    Fixture fixture(std::filesystem::path(u8"game.part \u03c0.sfc"));
    fixture.Data();
    const std::array first{Sample{111, -222}, Sample{333, -444}};
    const std::array second{Sample{-555, 666}, Sample{-777, 888}};
    fixture.Pcm(1, first);
    fixture.Pcm(260, second);
    fixture.RawTrack(2, std::array<uint8_t, 12>{'B', 'A', 'D', '1'});
    fixture.RawTrack(3, std::array<uint8_t, 4>{'M', 'S', 'U', '1'});
    fixture.RawTrack(4, std::array<uint8_t, 7>{'M', 'S', 'U', '1'});
    fixture.Pcm(5, {});
    fixture.RawTrack(6, std::array<uint8_t, 10>{'M', 'S', 'U', '1'});
    fixture.RawTrack(7, std::array<uint8_t, 13>{'M', 'S', 'U', '1'});
    fixture.RawTrack(8, {});
    Msu1 chip;
    Check(chip.Open(fixture.rom), "Dotted Unicode file stem discovers its sibling data file");
    for (uint16_t track : {2, 3, 4, 5, 6, 7, 8, 999}) {
        Track(chip, track);
        Check(chip.Read(0x2000) == 0x0a, "Missing or malformed PCM sets the audio error flag");
        chip.Write(0x2007, 3);
        for (unsigned sample = 0; sample < 3; ++sample)
            Check(chip.NextSample(32000) == Sample{}, "Invalid repeating PCM cannot loop or leak old samples");
        Check(chip.Read(0x2000) == 0x0a, "Control writes cannot start an errored track");
    }
    Play(chip, 1);
    Check(chip.Read(0x2000) == 0x12 && chip.NextSample(44100) == first[0],
          "A successful selection clears the preceding track error");
    chip.Write(0x2004, 4);
    Check(chip.Read(0x2000) == 0x12 && chip.NextSample(44100) == first[1],
          "Track low-byte write leaves the current track running");
    chip.Write(0x2005, 1);
    Check(chip.Read(0x2000) == 2, "Track high-byte write selects and stops the complete 16-bit track");
    chip.Write(0x2007, 1);
    Check(chip.NextSample(44100) == second[0], "Track names use the full decimal track number");
}

void SamplesVolumePauseAndEnd() {
    Fixture fixture;
    fixture.Data();
    const std::array frames{Sample{-32768, 32767}, Sample{-255, 255}, Sample{12345, -23456}};
    fixture.Pcm(1, frames);
    Msu1 chip;
    Check(chip.Open(fixture.rom), "Open volume fixture");
    Play(chip, 1);
    for (const auto sample : frames)
        Check(chip.NextSample(44100) == sample, "PCM decodes little-endian signed stereo with both full-scale endpoints");
    Check(chip.Read(0x2000) == 0x12, "The final complete source frame is emitted before EOF is examined");
    Check(chip.NextSample(44100) == Sample{} && chip.Read(0x2000) == 2,
          "Nonrepeating EOF produces silence and clears play/repeat");
    chip.Write(0x2007, 0xf9);
    Check(chip.NextSample(44100) == frames[0], "Play restarts an ended track and ignores reserved control bits");
    chip.Write(0x2007, 0);
    for (unsigned sample = 0; sample < 4; ++sample)
        Check(chip.NextSample(44100) == Sample{}, "Paused audio is silent");
    chip.Write(0x2007, 1);
    Check(chip.NextSample(44100) == frames[1], "Plain pause preserves the next unread sample");
    chip.Write(0x2006, 128);
    const Sample scaled{static_cast<int16_t>(int32_t{frames[2][0]} * 128 / 255),
                        static_cast<int16_t>(int32_t{frames[2][1]} * 128 / 255)};
    Check(chip.NextSample(44100) == scaled, "Volume uses the complete 0-255 range with signed arithmetic");
    Play(chip, 1, 1, 0);
    Check(chip.NextSample(44100) == Sample{}, "Zero volume mutes the source");
    chip.Write(0x2006, 255);
    Check(chip.NextSample(44100) == frames[1], "Muted playback still advances the PCM position");
    chip.Write(0x2007, 2);
    Check(chip.Read(0x2000) == 0x22, "Repeat and play control bits are independent at write time");
    Check(chip.NextSample(44100) == Sample{} && chip.Read(0x2000) == 2,
          "An idle audio interval clears repeat without playing");

    const std::array<Sample, 1> one{Sample{30000, -30000}};
    fixture.Pcm(2, one);
    Play(chip, 2);
    const Sample tail{static_cast<int16_t>(30000ll * 32000 / 44100),
                      static_cast<int16_t>(-30000ll * 32000 / 44100)};
    Check(chip.NextSample(32000) == tail && chip.Read(0x2000) == 2,
          "A partial final output interval retains the audible tail and pads the rest with silence");
}

void RepeatAndLoopFallback() {
    Fixture fixture;
    fixture.Data();
    const std::array frames{Sample{100, -100}, Sample{200, -200}, Sample{300, -300}};
    fixture.Pcm(1, frames, 1);
    fixture.Pcm(2, frames, 3);
    fixture.Pcm(3, frames, std::numeric_limits<uint32_t>::max());
    fixture.Pcm(4, std::span<const Sample>(frames).first(1), 0);
    Msu1 chip;
    Check(chip.Open(fixture.rom), "Open loop fixture");
    Play(chip, 1, 3);
    for (unsigned sample = 0; sample < 15; ++sample) {
        const auto expected = sample < 3 ? sample : 1 + (sample - 3) % 2;
        Check(chip.NextSample(44100) == frames[expected], "Loop point is measured in stereo frames after the header");
        Check(chip.Read(0x2000) == 0x32, "Looping retains play and repeat flags");
    }
    for (uint16_t track : {2, 3}) {
        Play(chip, track, 3);
        for (unsigned sample = 0; sample < 12; ++sample)
            Check(chip.NextSample(44100) == frames[sample % 3],
                  "Out-of-range loop indices safely fall back to the first PCM frame");
    }
    Play(chip, 4, 3);
    for (unsigned sample = 0; sample < 20; ++sample)
        Check(chip.NextSample(32000) == frames[0], "Single-frame repeat has no silent gaps at a different output rate");
    Check(chip.NextSample(std::numeric_limits<uint32_t>::max()) == frames[0] &&
          chip.NextSample(1) == frames[0], "Rate changes and extreme valid rates remain bounded");
}

void ResumeSlotsAndTrackZero() {
    Fixture fixture;
    fixture.Data();
    const std::array first{Sample{111, -222}, Sample{333, -444}, Sample{555, -666}};
    const std::array second{Sample{-777, 888}, Sample{-999, 1111}, Sample{-1222, 1333}};
    for (uint16_t track : {0, 1, 65535}) fixture.Pcm(track, first);
    fixture.Pcm(2, second);
    Msu1 chip;
    Check(chip.Open(fixture.rom), "Open resume fixture");
    for (uint16_t track : {0, 1, 65535}) {
        Play(chip, track);
        Check(chip.NextSample(44100) == first[0], "Every track number starts from its first PCM frame");
        chip.Write(0x2007, 4);
        Check(chip.NextSample(44100) == Sample{}, "Resume capture stops playback");
        Play(chip, 2);
        Check(chip.NextSample(44100) == second[0], "Another track can play while a resume slot is retained");
        Play(chip, track);
        Check(chip.NextSample(44100) == first[1], "Track reselection consumes the matching saved position");
        Play(chip, track);
        Check(chip.NextSample(44100) == first[0], "The saved position is consumed only once");
    }
    chip.Write(0x2007, 4);
    Track(chip, 999);
    Play(chip, 65535);
    Check(chip.NextSample(44100) == first[1], "Failed track opens do not erase an existing resume slot");
    Play(chip, 1);
    chip.NextSample(44100);
    chip.Write(0x2007, 4);
    Play(chip, 2);
    chip.NextSample(44100);
    chip.Write(0x2007, 4);
    Play(chip, 1);
    Check(chip.NextSample(44100) == first[0], "Capturing a second track replaces the single resume slot");
    Play(chip, 2);
    Check(chip.NextSample(44100) == second[1], "An unmatched selection leaves the saved track available");
    chip.Reset();
    Play(chip, 1, 5);
    chip.NextSample(44100);
    Play(chip, 2);
    chip.NextSample(44100);
    Play(chip, 1);
    Check(chip.NextSample(44100) == first[0], "Resume bit combined with play does not capture a stopped position");
}

void FractionalPauseResumeAndVolume() {
    Fixture fixture;
    fixture.Data();
    const std::array frames{Sample{28000, -20000}, Sample{-23000, 17000}, Sample{19000, -31000}};
    fixture.Pcm(1, frames, 0);
    fixture.Pcm(2, std::array<Sample, 1>{Sample{1234, -2345}}, 0);
    Msu1 continuous;
    Msu1 interrupted;
    Check(continuous.Open(fixture.rom) && interrupted.Open(fixture.rom), "Open independent fractional-playback streams");
    Play(continuous, 1, 3);
    Play(interrupted, 1, 3);
    Check(interrupted.NextSample(48000) == continuous.NextSample(48000), "Both renderers begin at the same PCM phase");
    interrupted.Write(0x2007, 0);
    for (unsigned frame = 0; frame < 13; ++frame)
        Check(interrupted.NextSample(48000) == Sample{}, "Fractional pause emits silence without consuming the held frame");
    interrupted.Write(0x2007, 3);
    Check(interrupted.NextSample(48000) == continuous.NextSample(48000), "Plain pause preserves the fractional frame");
    interrupted.Write(0x2007, 4);
    Play(interrupted, 2, 3);
    for (unsigned frame = 0; frame < 11; ++frame) interrupted.NextSample(48000);
    Play(interrupted, 1, 3);
    for (unsigned frame = 0; frame < 30; ++frame)
        Check(interrupted.NextSample(48000) == continuous.NextSample(48000),
              "Track resume restores fractional duration without dropping or duplicating a frame");
    interrupted.Write(0x2006, 0);
    Check(interrupted.NextSample(48000) == Sample{}, "Volume changes immediately affect a partially held PCM frame");
    continuous.NextSample(48000);
    interrupted.Write(0x2006, 255);
    Check(interrupted.NextSample(48000) == continuous.NextSample(48000), "Unmuting retains exact fractional progression");
}

void RationalRatesAgainstIntegralOracle() {
    Fixture fixture;
    fixture.Data();
    std::vector<Sample> frames(44100);
    std::vector<std::array<int64_t, 2>> prefix(frames.size() + 1);
    for (size_t index = 0; index < frames.size(); ++index) {
        frames[index] = {
            static_cast<int16_t>(static_cast<int32_t>((index * 137) % 65536) - 32768),
            static_cast<int16_t>(static_cast<int32_t>((index * 911 + 77) % 65536) - 32768)};
        for (unsigned channel = 0; channel < 2; ++channel)
            prefix[index + 1][channel] = prefix[index][channel] + frames[index][channel];
    }
    fixture.Pcm(1, frames);
    Msu1 chip;
    Check(chip.Open(fixture.rom), "Open generated one-second PCM waveform");
    constexpr uint8_t volume = 187;
    for (uint32_t rate : {1u, 32000u, 44100u, 48000u, 96000u}) {
        Play(chip, 1, 1, volume);
        Check(chip.NextSample(0) == Sample{} && chip.Read(0x2000) == 0x12,
              "A zero output rate neither divides by zero nor advances playback");
        const auto integral = [&](uint64_t time, unsigned channel) {
            const auto index = static_cast<size_t>(time / rate);
            const auto fraction = time % rate;
            return prefix[index][channel] * rate +
                (index < frames.size() ? int64_t{frames[index][channel]} * static_cast<int64_t>(fraction) : 0);
        };
        for (uint32_t frame = 0; frame < rate; ++frame) {
            Sample expected{};
            for (unsigned channel = 0; channel < 2; ++channel) {
                const auto sum = integral((uint64_t{frame} + 1) * 44100, channel) -
                                 integral(uint64_t{frame} * 44100, channel);
                expected[channel] = static_cast<int16_t>(sum * volume / (44100ll * 255));
            }
            Check(chip.NextSample(rate) == expected,
                  "Every resampled frame matches an independent cumulative-integral oracle");
        }
        Check(chip.Read(0x2000) == 0x12, "One output second consumes exactly 44,100 PCM frames without lookahead");
        Check(chip.NextSample(rate) == Sample{} && chip.Read(0x2000) == 2,
              "PCM EOF occurs at the same rational time for every output rate");
    }
}

} // namespace

int main() {
    try {
        DiscoveryMappingAndLifecycle();
        DataLatchesAndCacheBoundaries();
        SparseFourGigabyteData();
        TrackValidationAndSelection();
        SamplesVolumePauseAndEnd();
        RepeatAndLoopFallback();
        ResumeSlotsAndTrackZero();
        FractionalPauseResumeAndVolume();
        RationalRatesAgainstIntegralOracle();
        std::puts("MSU-1 tests passed (9 groups)");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "MSU-1 test failure: %s\n", error.what());
        return 1;
    }
}
