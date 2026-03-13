#pragma once
// ============================================================================
// SdlAudioOutput.hpp — SDL3-based audio output for the SNES emulator
//
// Opens an SDL3 audio stream at 32 kHz stereo float and implements
// IAudioOutput::Submit() by pushing interleaved samples into SDL's
// internal queue.  SDL3's audio stream handles buffering, resampling
// (if the hardware rate differs), and draining automatically.
//
// Usage:
//     SdlAudioOutput audio;            // opens default playback device
//     audio.Resume();                   // un-pause
//     audio.Submit(buffer);             // push samples each frame
//     // ... on shutdown, destructor cleans up
// ============================================================================

#include "snes/core/Platform.hpp"

#include <SDL3/SDL.h>
#include <cstdint>

namespace snes::frontend {

class SdlAudioOutput final : public snes::core::IAudioOutput {
public:
    struct Config {
        int sampleRate = 32000;       // SNES native rate
        int channels   = 2;           // stereo
    };

    explicit SdlAudioOutput(const Config& config = {});
    ~SdlAudioOutput() override;

    // Non-copyable / non-movable (owns SDL resources)
    SdlAudioOutput(const SdlAudioOutput&) = delete;
    SdlAudioOutput& operator=(const SdlAudioOutput&) = delete;

    // IAudioOutput
    void Submit(const snes::core::AudioBuffer& buffer) override;

    // -----------------------------------------------------------------------
    // Playback control
    // -----------------------------------------------------------------------

    /// Un-pause the audio device (SDL3 streams start paused).
    void Resume();

    /// Pause the audio device.
    void Pause();

    /// Returns true if the audio stream was created successfully.
    bool IsValid() const noexcept { return stream_ != nullptr; }

    /// Returns the approximate number of bytes queued in the stream.
    int QueuedBytes() const;

private:
    SDL_AudioStream* stream_ = nullptr;
};

} // namespace snes::frontend
