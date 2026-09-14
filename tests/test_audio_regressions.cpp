#include "snes/core/Dsp.hpp"
#include "snes/core/Smp.hpp"

#include <array>
#include <cstdio>

using snes::core::Dsp;
using snes::core::Smp;

namespace {
int failures = 0;
void Expect(bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

void HaltedSoundCpu() {
    for (uint8_t opcode : {0xef, 0xff}) {
        Smp smp;
        Dsp dsp;
        smp.SetDsp(dsp);
        dsp.SetRam(smp.Ram());
        std::array<int16_t, 128> output{};
        dsp.SetOutput(output.data(), 64);
        smp.r.pc = 0x200;
        smp.Ram()[0x200] = opcode;
        smp.Write(0xfc, 1);
        smp.Write(0xf1, 4);
        smp.RunUntil(128);
        Expect(dsp.SamplesWritten() == 4, "DSP keeps producing samples while the sound CPU is halted");
        Expect(smp.GetTimer2().stage3 == 8, "timer 2 keeps ticking while the sound CPU is halted");
        const auto pc = smp.r.pc;
        smp.RunUntil(1024);
        Expect(dsp.SamplesWritten() == 32, "halted audio remains continuous across scheduler calls");
        Expect(smp.r.pc == pc, "a halted sound CPU does not execute more instructions");
    }
}

void PowerClearsKeyOn() {
    Dsp reused;
    std::array<uint8_t, 65536> ram{};
    reused.SetRam(ram.data());
    reused.Write(Dsp::kKon, 1);
    reused.RunSample();
    reused.RunSample(); // Latch KON on the second sample.
    reused.Power();
    Dsp fresh;
    fresh.SetRam(ram.data());
    for (Dsp* dsp : {&reused, &fresh}) {
        dsp->Write(Dsp::kFlg, 0x20);
        dsp->Write(Dsp::kKon, 1);
    }
    for (int i = 0; i < 8; ++i) {
        reused.RunSample();
        fresh.RunSample();
        Expect(reused.GetVoice(0).konDelay == fresh.GetVoice(0).konDelay,
               "power reset accepts the same key-on as a fresh DSP");
    }
}
}

int main() {
    HaltedSoundCpu();
    PowerClearsKeyOn();
    return failures ? 1 : 0;
}
