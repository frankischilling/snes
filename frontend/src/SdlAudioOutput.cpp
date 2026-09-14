// ============================================================================
// SdlAudioOutput.cpp — SDL3-based audio output implementation
// ============================================================================

#include "SdlAudioOutput.hpp"

#include <stdexcept>

namespace snes::frontend {

// ============================================================================
// Construction / Destruction
// ============================================================================

SdlAudioOutput::SdlAudioOutput() : SdlAudioOutput(Config{}) {}

SdlAudioOutput::SdlAudioOutput(const Config& config) {
    // Describe the format we will push: stereo float at the SNES sample rate.
    SDL_AudioSpec spec{};
    spec.format   = SDL_AUDIO_F32;
    spec.channels = config.channels;
    spec.freq     = config.sampleRate;

    // SDL_OpenAudioDeviceStream opens a device, creates a stream, and binds
    // them together.  Passing nullptr for the callback means we will push
    // data ourselves via SDL_PutAudioStreamData (queue-based model).
    // The stream starts **paused** — call Resume() to begin playback.
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

// ============================================================================
// IAudioOutput::Submit
// ============================================================================

void SdlAudioOutput::Submit(const snes::core::AudioBuffer& buffer) {
    if (!stream_) return;
    if (buffer.interleavedStereo.empty()) return;

    // Keep latency bounded without destructively trimming queued samples.
    // If we're already too far ahead, skip this frame's submit and let the
    // device drain naturally to avoid audible discontinuities.
    constexpr int kBytesPerFrame = static_cast<int>(2 * sizeof(float));
    constexpr int kMaxQueueMs = 80;
    const int maxQueuedBytes =
        (buffer.sampleRate * kBytesPerFrame * kMaxQueueMs) / 1000;

    const int queued = SDL_GetAudioStreamAvailable(stream_);
    if (queued > maxQueuedBytes) {
        return;
    }

    // Push interleaved float samples into SDL's internal queue.
    // SDL will resample automatically if the hardware rate differs from 32 kHz.
    const int bytes = static_cast<int>(
        buffer.interleavedStereo.size() * sizeof(float));
    SDL_PutAudioStreamData(stream_, buffer.interleavedStereo.data(), bytes);
}

// ============================================================================
// Playback control
// ============================================================================

void SdlAudioOutput::Resume() {
    if (stream_) SDL_ResumeAudioStreamDevice(stream_);
}

void SdlAudioOutput::Pause() {
    if (stream_) SDL_PauseAudioStreamDevice(stream_);
}

int SdlAudioOutput::QueuedBytes() const {
    return stream_ ? SDL_GetAudioStreamAvailable(stream_) : 0;
}

} // namespace snes::frontend
