// Clock-boundary regression tests for DSP latches and the shared SMP RAM bus.
#include "snes/core/Dsp.hpp"
#include "snes/core/Smp.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <vector>

using snes::core::Dsp;
using snes::core::Smp;

namespace {
void Check(bool condition, const char* description) {
    if (!condition) throw std::runtime_error(description);
}

struct Fixture {
    std::vector<uint8_t> ram = std::vector<uint8_t>(65536);
    std::array<int16_t, 256> output{};
    Dsp dsp;
    Fixture() {
        dsp.SetRam(ram.data());
        dsp.SetOutput(output.data(), int(output.size() / 2));
        dsp.Write(Dsp::kFlg, 0x20);
    }
    void Tick(unsigned count) { while (count--) dsp.Tick(); }
    void To(unsigned phase) {
        Check(phase < 32, "Valid requested DSP phase");
        while (dsp.Phase() != phase) dsp.Tick();
    }
    void Word(uint16_t address, uint16_t value) {
        ram[address] = uint8_t(value);
        ram[uint16_t(address + 1)] = uint8_t(value >> 8);
    }
    uint16_t Word(uint16_t address) const {
        return uint16_t(ram[address] | (uint16_t(ram[uint16_t(address + 1)]) << 8));
    }
    void WarmEcho() {
        dsp.Write(Dsp::kEsa, 0x80);
        dsp.Write(Dsp::kEdl, 0);
        dsp.RunSample();
        dsp.ResetSamplesWritten();
    }
};

void OutputAndCounterPhases() {
    Fixture f;
    f.dsp.Write(Dsp::kFlg, 0x5f);
    f.Tick(27);
    Check(f.dsp.Phase() == 27 && f.dsp.SamplesWritten() == 0 && f.dsp.Counter() == 0,
          "Output and the envelope counter do not advance before their phases");
    f.Tick(1);
    Check(f.dsp.SamplesWritten() == 1 && f.output[0] == 0 && f.output[1] == 0,
          "Muted stereo output is published at phase 27");
    f.Tick(2);
    Check(f.dsp.Noise() == 0x4000 && f.dsp.Counter() == 0, "Noise and the rate counter retain their values through phase 29");
    f.Tick(1);
    Check(f.dsp.Noise() == 0x2000 && f.dsp.Counter() == 30719, "Noise and the rate counter advance together at phase 30");
    f.Tick(1);
    Check(f.dsp.Phase() == 0, "The DSP schedule wraps after 32 clocks");
    f.Tick(11);
    const int count = f.dsp.SamplesWritten();
    f.dsp.RunSample();
    Check(f.dsp.Phase() == 11 && f.dsp.SamplesWritten() == count + 1,
          "RunSample advances one full cycle without discarding the current phase");

    std::array<int16_t, 4> guarded{1234, 1234, 1234, 1234};
    f.dsp.SetOutput(guarded.data(), 1);
    for (unsigned i = 0; i < 4; ++i) f.dsp.RunSample();
    Check(f.dsp.SamplesWritten() == 1 && guarded[2] == 1234 && guarded[3] == 1234,
          "Output capacity limits writes without stopping DSP phase execution");
}

void PitchAndEnvelopeLatches() {
    Fixture f;
    f.Tick(21);
    f.dsp.Write(Dsp::kPitchL, 0x34);
    f.dsp.Write(Dsp::kPitchH, 0x12);
    f.Tick(1);
    f.dsp.Write(Dsp::kPitchL, 0x56);
    f.dsp.Write(Dsp::kPitchH, 0x23);
    f.Tick(1);
    f.dsp.Write(Dsp::kPitchH, 0x3f);
    f.Tick(8);
    Check(f.dsp.GetVoice(0).interpPos == 0, "Voice zero pitch is not applied before phase 31");
    f.Tick(1);
    Check(f.dsp.GetVoice(0).interpPos == 0x2334,
          "Pitch uses the low byte from phase 21 and high byte from phase 22");

    Fixture adsr;
    auto& voice = adsr.dsp.GetVoice(0);
    voice.envMode = Dsp::Attack;
    adsr.dsp.Write(Dsp::kAdsr0, 0x8f);
    adsr.dsp.Write(Dsp::kGain, 0x7f);
    adsr.Tick(22);
    adsr.dsp.Write(Dsp::kAdsr0, 0);
    adsr.Tick(9);
    Check(voice.env == 0x400, "ADSR0 is latched before a later register write can select GAIN mode");

    Fixture gain;
    gain.dsp.GetVoice(0).envMode = Dsp::Sustain;
    gain.dsp.Write(Dsp::kGain, 1);
    gain.Tick(30);
    gain.dsp.Write(Dsp::kGain, 0x7f);
    gain.Tick(1);
    Check(gain.dsp.GetVoice(0).env == 0x7f0, "GAIN is sampled when the envelope actually runs at phase 30");
}

void KeyOnAndGlobalLatches() {
    Fixture f;
    f.dsp.Write(Dsp::kKon, 3);
    f.Tick(62);
    Check(f.dsp.GetVoice(0).konDelay == 0 && f.dsp.GetVoice(1).konDelay == 0,
          "Key-on waits for the every-other-sample latch");
    f.Tick(1);
    Check(f.dsp.GetVoice(0).konDelay == 5 && f.dsp.GetVoice(1).konDelay == 0,
          "Voice zero accepts KON at phase 30 before voice one reaches its render phase");
    f.dsp.Write(Dsp::kKon, 0);
    f.Tick(3);
    Check(f.dsp.GetVoice(1).konDelay == 5, "Clearing KON after capture does not cancel another voice's pending key-on");

    Fixture noise;
    noise.dsp.Write(Dsp::kNon, 2);
    noise.dsp.RunSample();
    noise.dsp.Write(Dsp::kNon, 0);
    noise.dsp.GetVoice(1).env = 0x7ff;
    noise.dsp.GetVoice(1).envMode = Dsp::Sustain;
    noise.dsp.Write(0x17, 0x7f);
    noise.Tick(2);
    Check(noise.dsp.GetVoice(1).output == -32752, "Noise-enable remains latched for voice one after NON is cleared");
    noise.dsp.RunSample();
    Check(noise.dsp.GetVoice(1).output == 0, "The next phase-28 NON capture restores the decoded voice sample");

    Fixture directory;
    directory.dsp.Write(Dsp::kDir, 2);
    directory.dsp.RunSample();
    directory.dsp.Write(Dsp::kSrcn, 3);
    directory.Word(0x20c, 0x4000);
    directory.Word(0x210, 0x5000);
    directory.Tick(18);
    directory.dsp.Write(Dsp::kSrcn, 4);
    directory.Tick(3);
    directory.dsp.GetVoice(0).konDelay = 5;
    directory.Tick(1);
    directory.Word(0x20c, 0x6000);
    directory.dsp.Write(Dsp::kDir, 3);
    directory.Tick(9);
    Check(directory.dsp.GetVoice(0).brrAddr == 0x4000,
          "Source number and directory pointer remain captured across later register and RAM writes");
}

void RegisterPublication() {
    Fixture f;
    f.dsp.GetVoice(0).env = 0x400;
    f.dsp.GetVoice(0).envMode = Dsp::Sustain;
    f.dsp.Write(Dsp::kGain, 0x40);
    f.dsp.RunSample();
    f.Tick(2);
    f.dsp.Write(0x79, 0xa5);
    f.Tick(1);
    f.dsp.Write(0x78, 0xb6);
    f.Tick(1);
    Check(f.dsp.Read(Dsp::kOutX) == 0xa5,
          "An OUTX write for voice seven overrides the shared pending voice-zero publication");
    f.Tick(1);
    Check(f.dsp.Read(Dsp::kEnvX) == 0xb6,
          "An ENVX write for voice seven overrides the shared pending voice-zero publication");
    f.dsp.RunSample();
    Check(f.dsp.Read(Dsp::kEnvX) == 0x40, "The next voice envelope capture replaces the CPU-injected value");

    for (bool clear : {false, true}) {
        Fixture end;
        auto& v = end.dsp.GetVoice(1);
        v.brrAddr = 0x2000; v.brrOffset = 7; v.interpPos = 0x4000;
        end.ram[0x2000] = 3;
        end.Word(2, 0x3000);
        end.Tick(4);
        Check(v.brrAddr == 0x3000 && end.dsp.Read(Dsp::kEndx) == 0,
              "BRR loop completion precedes the delayed ENDX publication");
        if (clear) end.dsp.Write(Dsp::kEndx, 0xff);
        end.Tick(1);
        Check(end.dsp.Read(Dsp::kEndx) == 0, "ENDX remains hidden through the intervening output-capture phase");
        end.Tick(1);
        Check(end.dsp.Read(Dsp::kEndx) == (clear ? 0 : 2),
              "ENDX publishes at phase 5 unless a CPU write cleared its pending latch");
    }
}

void BrrReadPhases() {
    Fixture f;
    auto& v = f.dsp.GetVoice(0);
    v.brrAddr = 0x2000; v.brrOffset = 1; v.interpPos = 0x4000;
    f.ram[0x2000] = 0x20; f.ram[0x2001] = 0x12; f.ram[0x2002] = 0x34;
    f.Tick(26);
    f.ram[0x2000] = 0xd0;
    f.ram[0x2001] = 0xfe;
    f.ram[0x2002] = 0x56;
    f.Tick(5);
    Check(v.bufPos == 0 && v.brrOffset == 1, "BRR decoding waits until phase 31");
    f.Tick(1);
    const std::array<int, 4> expected{4, 8, 20, 24};
    Check(std::equal(expected.begin(), expected.end(), v.buf) &&
          std::equal(expected.begin(), expected.end(), v.buf + Dsp::BrrBufSize),
          "BRR uses the header and first byte from phase 25 but the second byte from phase 31");
    Check(v.brrOffset == 3 && v.bufPos == 4, "BRR advances exactly one group of four decoded samples");

    for (unsigned shift = 13; shift <= 15; ++shift) {
        Fixture invalid;
        auto& voice = invalid.dsp.GetVoice(1);
        voice.brrAddr = 0xffff; voice.brrOffset = 1; voice.interpPos = 0x4000;
        invalid.ram[0xffff] = uint8_t(shift << 4);
        invalid.ram[0] = 0x7f; invalid.ram[1] = 0x80;
        invalid.Tick(3);
        Check(voice.buf[0] == 0 && voice.buf[1] == -4096 && voice.buf[2] == -4096 && voice.buf[3] == 0,
              "Invalid BRR shifts retain only sign, and data reads wrap through address zero");
    }
}

void EchoReadAndMixPhases() {
    Fixture f;
    f.WarmEcho();
    f.Word(0x8000, 0x4000); f.Word(0x8002, 0x2000);
    f.dsp.Write(0x7f, 64);
    f.dsp.Write(Dsp::kEVolL, 127); f.dsp.Write(Dsp::kEVolR, 127);
    f.Tick(23);
    f.Word(0x8000, 0);
    f.Word(0x8002, 0x6000);
    f.Tick(1);
    f.Word(0x8002, 0);
    f.Tick(3);
    Check(f.dsp.SamplesWritten() == 0, "The stereo frame is not published with only the left mix complete");
    f.dsp.Write(Dsp::kEVolL, 0); f.dsp.Write(Dsp::kEVolR, 64);
    f.Tick(1);
    Check(f.dsp.SamplesWritten() == 1 && f.output[0] == 8128 && f.output[1] == 6144,
          "Echo reads L/R at phases 22/23 and samples their final volumes separately at phases 26/27");

    Fixture lateTap;
    lateTap.WarmEcho();
    lateTap.Word(0x8000, 0x4000);
    lateTap.dsp.Write(Dsp::kEVolL, 64);
    lateTap.Tick(25);
    lateTap.dsp.Write(0x7f, 64);
    lateTap.Tick(1);
    lateTap.dsp.Write(0x7f, 0);
    lateTap.Tick(2);
    Check(lateTap.output[0] == 4096, "The newest FIR tap uses its coefficient at phase 25 and preserves the resulting sum");
}

void EchoWriteAndAddressPhases() {
    Fixture f;
    f.WarmEcho();
    f.Word(0x8000, 0x1111); f.Word(0x8002, 0x2222);
    f.Tick(28);
    f.dsp.Write(Dsp::kFlg, 0);
    f.Tick(1);
    Check(f.Word(0x8000) == 0x1111 && f.Word(0x8002) == 0x2222, "Echo write-enable capture does not write RAM early");
    f.dsp.Write(Dsp::kFlg, 0x20);
    f.Tick(1);
    Check(f.Word(0x8000) == 0 && f.Word(0x8002) == 0x2222,
          "Left echo writes at phase 29 using the previous phase's enable bit");
    f.Tick(1);
    Check(f.Word(0x8002) == 0x2222, "Right echo uses the newly captured disable bit at phase 30");
    f.Word(0x8000, 0x3333);
    f.To(28);
    f.Tick(1);
    f.dsp.Write(Dsp::kFlg, 0);
    f.Tick(2);
    Check(f.Word(0x8000) == 0x3333 && f.Word(0x8002) == 0,
          "Independent FLG captures also allow right-only echo writes");

    Fixture address;
    address.dsp.Write(Dsp::kFlg, 0);
    address.dsp.Write(Dsp::kEsa, 0x80);
    address.dsp.Write(Dsp::kEdl, 1);
    address.Word(0, 0x1234); address.Word(0x8000, 0x5678); address.Word(0x8004, 0x6789);
    address.dsp.RunSample();
    Check(address.Word(0) == 0 && address.Word(0x8000) == 0x5678,
          "New ESA is captured after the current echo address has already been used");
    address.dsp.RunSample();
    Check(address.Word(0x8004) == 0 && address.Word(0x8000) == 0x5678,
          "The next echo frame combines the new ESA with the advancing byte offset");

    Fixture delay;
    delay.dsp.Write(Dsp::kFlg, 0);
    delay.dsp.Write(Dsp::kEdl, 1);
    delay.dsp.RunSample();
    delay.dsp.Write(Dsp::kEdl, 0);
    delay.Word(8, 0x1111);
    delay.dsp.RunSample(); delay.dsp.RunSample();
    Check(delay.Word(8) == 0, "Changing EDL mid-buffer does not shorten the already latched echo length");
}

void SoundCpuSharedClock() {
    Smp smp;
    Dsp dsp;
    smp.SetDsp(dsp); dsp.SetRam(smp.Ram());
    dsp.Write(Dsp::kFlg, 0);
    for (unsigned i = 0; i < 29; ++i) smp.Idle();
    smp.Ram()[0] = 0xff; smp.Ram()[1] = 0xee;
    smp.Ram()[2] = 0xdd; smp.Ram()[3] = 0xcc;
    smp.Write(0, 0xa5);
    Check(smp.Ram()[0] == 0xa5 && smp.Ram()[1] == 0,
          "DSP echo writes precede an SMP write to the same shared-RAM word on that cycle");
    Check(smp.Read(2) == 0 && smp.Ram()[3] == 0,
          "An SMP read observes the echo word written on its own DSP clock");
    Check(smp.CycleCount() == 31 && dsp.Phase() == 31, "Each SMP idle/read/write advances exactly one DSP phase");
    const auto before = smp.CycleCount();
    smp.PortWrite(0, 0x63);
    Check(smp.CycleCount() == before, "CPU port-latch changes do not double-advance SMP scheduling");
    smp.Write(0xf4, 0x29);
    Check(smp.Read(0xf4) == 0x63 && smp.PortRead(0) == 0x29, "CPU and SMP ports retain independent directional latches");
    smp.Write(0xf1, 0x10);
    Check(smp.ApuInput(0) == 0 && smp.PortRead(0) == 0x29, "CONTROL clears the input pair without disturbing CPU-visible output");
    Check(dsp.Phase() == (smp.CycleCount() & 31), "Port I/O remains on the same shared DSP clock");

    Smp halted;
    Dsp haltedDsp;
    std::array<int16_t, 8> frames{};
    halted.SetDsp(haltedDsp); haltedDsp.SetRam(halted.Ram());
    haltedDsp.SetOutput(frames.data(), 4);
    halted.r.stop = true;
    halted.RunUntil(64);
    Check(halted.CycleCount() == 64 && haltedDsp.Phase() == 0 && haltedDsp.SamplesWritten() == 2,
          "A halted sound CPU advances all DSP phases through scheduler calls");
    halted.RunUntil(64);
    Check(haltedDsp.SamplesWritten() == 2 && haltedDsp.Phase() == 0,
          "Repeated synchronization at the same timestamp does not run the DSP twice");
}

void BatchedTimerEdges() {
    Smp::Timer<16> batched, singles;
    batched.enable = singles.enable = true;
    batched.target = singles.target = 0;
    batched.Step(8192, true, false);
    for (unsigned i = 0; i < 8192; ++i) singles.Step(1, true, false);
    Check(batched.stage3 == 1 && batched.stage2 == 0 && batched.stage0 == 0,
          "Timer target zero counts 256 falling edges even in a large batched step");
    Check(batched.stage0 == singles.stage0 && batched.stage1 == singles.stage1 && batched.stage2 == singles.stage2 &&
          batched.stage3 == singles.stage3 && batched.line == singles.line,
          "Batched timer stepping preserves every divider and gate edge");
    Smp::Timer<128> slow;
    slow.enable = true; slow.target = 3;
    slow.Step(13997, true, false);
    Check(slow.stage0 == 45 && slow.stage1 == 1 && slow.stage2 == 0 && slow.stage3 == 2 && slow.line,
          "Timer output wraps at four bits while retaining fractional divider state");
}
}

int main() {
    try {
        OutputAndCounterPhases(); PitchAndEnvelopeLatches(); KeyOnAndGlobalLatches();
        RegisterPublication(); BrrReadPhases(); EchoReadAndMixPhases();
        EchoWriteAndAddressPhases(); SoundCpuSharedClock(); BatchedTimerEdges();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "DSP phase regression: %s\n", error.what());
        return 1;
    }
    std::puts("DSP phase, latch, BRR/echo RAM, output, timer and SMP synchronization checks passed");
}
