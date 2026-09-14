// SdlAudioOutput.cpp — SDL3-based audio output implementation

#include "SdlAudioOutput.hpp"

#include <stdexcept>
#include <limits>
#include <string>

namespace snes::frontend {

// Construction / Destruction

SdlAudioOutput::SdlAudioOutput() : SdlAudioOutput(Config{}) {}

SdlAudioOutput::SdlAudioOutput(const Config& config) {
    if (config.sampleRate <= 0 || config.sampleRate > 192000 || config.channels != 2) {
        throw std::invalid_argument("Audio requires stereo input at 1 to 192000 Hz");
    }
    sampleRate_ = config.sampleRate;
    targetQueuedBytes_ = ((sampleRate_ * 40 + 999) / 1000) * 2 * sizeof(float);
    // Describe the format we will push: stereo float at the SNES sample rate.
    SDL_AudioSpec spec{};
    spec.format   = SDL_AUDIO_F32;
    spec.channels = config.channels;
    spec.freq     = config.sampleRate;

    // SDL_OpenAudioDeviceStream opens a device, creates a stream, and binds
    // them together.  Passing nullptr for the callback means we will push
    // data ourselves via SDL_PutAudioStreamData (queue-based model).
    // The stream stays paused until Resume() is requested and input is ready.
    stream_ = SDL_OpenAudioDeviceStream(
        SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK,
        &spec,
        nullptr,   // no callback — push model
        nullptr);  // no userdata

    if (!stream_) {
        throw std::runtime_error(
            std::string("SDL_OpenAudioDeviceStream failed: ") + SDL_GetError());
    }
}

SdlAudioOutput::~SdlAudioOutput() {
    if (stream_) {
        // Destroying the stream also closes the associated audio device.
        SDL_DestroyAudioStream(stream_);
    }
}

// IAudioOutput::Submit

void SdlAudioOutput::Submit(const snes::core::AudioBuffer& buffer) {
    if (!stream_) return;
    if (buffer.interleavedStereo.empty()) return;

    if (buffer.sampleRate != sampleRate_ || buffer.interleavedStereo.size() % 2 != 0) {
        throw std::invalid_argument("Audio buffer does not match the stream's stereo input format");
    }
    if (buffer.interleavedStereo.size() > std::numeric_limits<int>::max() / sizeof(float)) {
        throw std::length_error("Audio buffer is too large");
    }
    // After a stall, collect a fresh buffer before restarting the device.
    const int available = SDL_GetAudioStreamAvailable(stream_);
    if (available < 0) {
        throw std::runtime_error(std::string("Audio availability query failed: ") + SDL_GetError());
    }
    // SDL may retain a few input frames for the resampler even after output drains.
    if (playing_ && available == 0) {
        if (!SDL_PauseAudioStreamDevice(stream_)) {
            throw std::runtime_error(std::string("Audio pause failed: ") + SDL_GetError());
        }
        playing_ = false;
    }

    // Push interleaved float samples into SDL's internal queue.
    // SDL will resample automatically if the hardware rate differs from 32 kHz.
    const int bytes = static_cast<int>(
        buffer.interleavedStereo.size() * sizeof(float));
    if (!SDL_PutAudioStreamData(stream_, buffer.interleavedStereo.data(), bytes)) {
        throw std::runtime_error(std::string("Audio submit failed: ") + SDL_GetError());
    }
    StartIfReady();
}

// Playback control

void SdlAudioOutput::Resume() {
    playbackRequested_ = true;
    StartIfReady();
}

void SdlAudioOutput::Pause() {
    if (stream_ && !SDL_PauseAudioStreamDevice(stream_)) {
        throw std::runtime_error(std::string("Audio pause failed: ") + SDL_GetError());
    }
    playbackRequested_ = false;
    playing_ = false;
}

int SdlAudioOutput::QueuedBytes() const {
    const int bytes = stream_ ? SDL_GetAudioStreamQueued(stream_) : 0;
    if (bytes < 0) {
        throw std::runtime_error(std::string("Audio queue query failed: ") + SDL_GetError());
    }
    return bytes;
}

bool SdlAudioOutput::NeedsSamples() const {
    return QueuedBytes() < targetQueuedBytes_;
}

void SdlAudioOutput::StartIfReady() {
    if (stream_ && playbackRequested_ && !playing_ && !NeedsSamples()) {
        if (!SDL_ResumeAudioStreamDevice(stream_)) {
            throw std::runtime_error(std::string("Audio resume failed: ") + SDL_GetError());
        }
        playing_ = true;
    }
}

} // namespace snes::frontend
