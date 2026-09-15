// snes emulator
// Streaming MSU-1 register and PCM implementation.

#include "snes/core/Msu1.hpp"

#include <algorithm>
#include <system_error>

namespace snes::core {
namespace {

uint32_t Little32(const uint8_t* bytes) {
    return uint32_t{bytes[0]} | (uint32_t{bytes[1]} << 8) |
           (uint32_t{bytes[2]} << 16) | (uint32_t{bytes[3]} << 24);
}

int16_t SignedLittle16(const uint8_t* bytes) {
    const int32_t value = int32_t{bytes[0]} | (int32_t{bytes[1]} << 8);
    return static_cast<int16_t>(value < 0x8000 ? value : value - 0x10000);
}

} // namespace

bool Msu1::Stream::Open(const std::filesystem::path& path) {
    Close();
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error)) return false;
    file_.open(path, std::ios::binary | std::ios::ate);
    const auto end = file_.tellg();
    if (!file_ || end < std::streampos{0}) {
        Close();
        return false;
    }
    size_ = static_cast<uint64_t>(end);
    return true;
}

void Msu1::Stream::Close() {
    if (file_.is_open()) file_.close();
    file_.clear();
    size_ = 0;
    cacheOffset_ = 0;
    cacheSize_ = 0;
}

bool Msu1::Stream::Read(uint64_t offset, std::span<uint8_t> destination) {
    if (!IsOpen() || offset > size_ || destination.size() > size_ - offset ||
        destination.size() > cache_.size()) return false;
    if (destination.empty()) return true;

    const bool cached = offset >= cacheOffset_ && offset - cacheOffset_ <= cacheSize_ &&
        destination.size() <= cacheSize_ - static_cast<size_t>(offset - cacheOffset_);
    if (!cached) {
        cacheSize_ = 0;
        cacheOffset_ = offset;
        file_.clear();
        file_.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
        if (!file_) return false;
        const auto count = static_cast<std::streamsize>(
            std::min<uint64_t>(cache_.size(), size_ - offset));
        file_.read(reinterpret_cast<char*>(cache_.data()), count);
        cacheSize_ = static_cast<size_t>(file_.gcount());
        if (cacheSize_ < destination.size()) return false;
    }
    const auto begin = static_cast<size_t>(offset - cacheOffset_);
    std::copy_n(cache_.data() + begin, destination.size(), destination.data());
    return true;
}

bool Msu1::Open(const std::filesystem::path& romPath, std::string* error) {
    Close();
    if (error) error->clear();
    if (romPath.empty()) {
        if (error) *error = "MSU-1 ROM path is empty";
        return false;
    }
    basePath_ = romPath;
    basePath_.replace_extension();
    auto dataPath = basePath_;
    dataPath += ".msu";
    if (!data_.Open(dataPath)) {
        if (error) *error = "Cannot open MSU-1 data file: " + dataPath.string();
        basePath_.clear();
        return false;
    }
    return true;
}

void Msu1::Close() {
    Reset();
    data_.Close();
    basePath_.clear();
}

void Msu1::Reset() {
    pcm_.Close();
    dataSeek_ = 0;
    dataOffset_ = 0;
    trackSeek_ = 0;
    track_ = 0;
    status_ = 0;
    volume_ = 0;
    frameCount_ = 0;
    nextFrame_ = 0;
    loopFrame_ = 0;
    sample_ = {};
    ticksLeft_ = 0;
    outputRate_ = 0;
    resume_ = {};
}

uint8_t Msu1::Read(uint32_t address, uint8_t openBus) {
    if (!IsOpen() || !Selects(address)) return openBus;
    const auto port = address & 7;
    if (port == 0) return status_ | 0x02;
    if (port == 1) {
        std::array<uint8_t, 1> value{};
        if (!data_.Read(dataOffset_, value)) return 0;
        ++dataOffset_;
        return value[0];
    }
    static constexpr std::array<uint8_t, 6> signature{'S', '-', 'M', 'S', 'U', '1'};
    return signature[port - 2];
}

void Msu1::Write(uint32_t address, uint8_t value) {
    if (!IsOpen() || !Selects(address)) return;
    const auto port = address & 7;
    if (port <= 3) {
        const auto shift = port * 8;
        dataSeek_ = (dataSeek_ & ~(uint32_t{0xff} << shift)) | (uint32_t{value} << shift);
        if (port == 3) dataOffset_ = dataSeek_;
        return;
    }
    if (port == 4) {
        trackSeek_ = static_cast<uint16_t>((trackSeek_ & 0xff00) | value);
    } else if (port == 5) {
        trackSeek_ = static_cast<uint16_t>((trackSeek_ & 0x00ff) | (uint16_t{value} << 8));
        SelectTrack();
    } else if (port == 6) {
        volume_ = value;
    } else if ((status_ & AudioError) == 0) {
        status_ = static_cast<uint8_t>((status_ & ~(Playing | Repeating)) | ((value & 3) << 4));
        if ((value & 5) == 4 && pcm_.IsOpen()) {
            resume_ = {true, track_, nextFrame_, sample_, ticksLeft_, outputRate_};
        }
    }
}

void Msu1::SelectTrack() {
    track_ = trackSeek_;
    status_ = static_cast<uint8_t>((status_ & ~(Playing | Repeating)) | AudioError);
    pcm_.Close();
    frameCount_ = 0;
    nextFrame_ = 0;
    loopFrame_ = 0;
    sample_ = {};
    ticksLeft_ = 0;

    auto path = basePath_;
    path += "-" + std::to_string(track_) + ".pcm";
    if (!pcm_.Open(path)) return;
    std::array<uint8_t, 8> header{};
    if (!pcm_.Read(0, header) || header[0] != 'M' || header[1] != 'S' ||
        header[2] != 'U' || header[3] != '1' || pcm_.Size() < 12 ||
        (pcm_.Size() - 8) % 4 != 0) {
        pcm_.Close();
        return;
    }
    frameCount_ = (pcm_.Size() - 8) / 4;
    loopFrame_ = Little32(header.data() + 4);
    if (loopFrame_ >= frameCount_) loopFrame_ = 0;
    status_ &= static_cast<uint8_t>(~AudioError);

    if (resume_.valid && resume_.track == track_) {
        if (resume_.nextFrame <= frameCount_) {
            nextFrame_ = resume_.nextFrame;
            sample_ = resume_.sample;
            ticksLeft_ = resume_.ticksLeft;
            outputRate_ = resume_.outputRate;
        }
        resume_.valid = false;
    }
}

bool Msu1::ReadFrame() {
    if (nextFrame_ >= frameCount_) {
        if ((status_ & Repeating) != 0) {
            nextFrame_ = loopFrame_;
        } else {
            status_ &= static_cast<uint8_t>(~(Playing | Repeating));
            nextFrame_ = 0;
            sample_ = {};
            return false;
        }
    }
    std::array<uint8_t, 4> bytes{};
    if (!pcm_.Read(8 + nextFrame_ * 4, bytes)) {
        status_ = static_cast<uint8_t>((status_ & ~(Playing | Repeating)) | AudioError);
        sample_ = {};
        pcm_.Close();
        return false;
    }
    sample_ = {SignedLittle16(bytes.data()), SignedLittle16(bytes.data() + 2)};
    ++nextFrame_;
    return true;
}

Msu1::StereoSample Msu1::NextSample(uint32_t outputRate) {
    if (outputRate == 0 || !IsOpen()) return {};
    if ((status_ & Playing) == 0 || !pcm_.IsOpen()) {
        status_ &= static_cast<uint8_t>(~(Playing | Repeating));
        return {};
    }
    if (outputRate_ != outputRate) {
        if (ticksLeft_ != 0 && outputRate_ != 0) {
            ticksLeft_ = static_cast<uint32_t>(
                (uint64_t{ticksLeft_} * outputRate + outputRate_ - 1) / outputRate_);
        }
        outputRate_ = outputRate;
    }

    // A PCM frame lasts outputRate integer ticks, and each output frame lasts
    // 44,100 ticks. Integrate their overlaps without reading a frame ahead.
    // Even at an output rate of one Hz, this loop takes at most 44,101 steps.
    uint32_t remaining = SampleRate;
    std::array<int64_t, 2> sum{};
    while (remaining != 0) {
        if (ticksLeft_ == 0) {
            if (!ReadFrame()) break;
            ticksLeft_ = outputRate;
        }
        const auto covered = std::min(remaining, ticksLeft_);
        sum[0] += int64_t{sample_[0]} * covered;
        sum[1] += int64_t{sample_[1]} * covered;
        ticksLeft_ -= covered;
        remaining -= covered;
    }
    constexpr int64_t divisor = int64_t{SampleRate} * 255;
    return {static_cast<int16_t>(sum[0] * volume_ / divisor),
            static_cast<int16_t>(sum[1] * volume_ / divisor)};
}

} // namespace snes::core
