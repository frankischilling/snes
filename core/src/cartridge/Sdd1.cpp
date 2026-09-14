#include "snes/core/Sdd1.hpp"

#include <array>

namespace snes::core {
namespace {

// S-DD1 probability states: Golomb order, next MPS state, next LPS state.
// Hardware algorithm: https://wiki.superfamicom.org/s-dd1
struct State { unsigned order, predicted, unexpected; };
constexpr std::array<State, 33> states{{
    {0,25,25}, {0,2,1}, {0,3,1}, {0,4,2}, {0,5,3},
    {1,6,4}, {1,7,5}, {1,8,6}, {1,9,7},
    {2,10,8}, {2,11,9}, {2,12,10}, {2,13,11},
    {3,14,12}, {3,15,13}, {3,16,14}, {3,17,15},
    {4,18,16}, {4,19,17}, {5,20,18}, {5,21,19},
    {6,22,20}, {6,23,21}, {7,24,22}, {7,24,23},
    {0,26,1}, {1,27,2}, {2,28,4}, {3,29,8},
    {4,30,12}, {5,31,16}, {6,32,18}, {7,24,22}
}};

class Decoder {
public:
    explicit Decoder(const std::function<uint8_t(uint32_t)>& reader)
        : reader_(reader), input_(reader(0)), header_(input_) {}

    std::vector<uint8_t> Decode(size_t size) {
        std::vector<uint8_t> result(size);
        const unsigned layout = header_ >> 6;
        for (size_t offset = 0; offset < size;) {
            if (layout == 3) {
                uint8_t pixel = 0;
                for (unsigned plane = 0; plane < 8; ++plane)
                    pixel |= PlaneBit(plane) << plane;
                result[offset++] = pixel;
            } else {
                // Planar tiles store two interleaved planes, eight rows each.
                const unsigned plane = layout == 0 ? 0 :
                    unsigned((offset / 16) * 2) & (layout == 1 ? 7 : 3);
                uint8_t even = 0, odd = 0;
                for (unsigned x = 0; x < 8; ++x) {
                    even = uint8_t((even << 1) | PlaneBit(plane));
                    odd = uint8_t((odd << 1) | PlaneBit(plane + 1));
                }
                result[offset++] = even;
                if (offset < size) result[offset++] = odd;
            }
        }
        return result;
    }

private:
    unsigned InputBit() {
        if (bitsLeft_ == 0) {
            input_ = reader_(++inputOffset_);
            bitsLeft_ = 8;
        }
        return (input_ >> --bitsLeft_) & 1;
    }

    unsigned PlaneBit(unsigned plane) {
        constexpr unsigned masks[] = {0x1c0, 0x180, 0xc0, 0x180};
        const unsigned model = (header_ >> 4) & 3;
        const unsigned context = ((plane & 1) << 4) |
            ((history_[plane] & masks[model]) >> 5) |
            (history_[plane] & (model == 3 ? 3 : 1));
        auto& prediction = predictions_[context];
        const auto state = states[prediction.state];
        auto& run = runs_[state.order];
        if (run.zeros == 0 && !run.one) {
            run.one = InputBit() != 0;
            run.zeros = 1u << state.order;
            if (run.one) {
                unsigned remainder = 0;
                for (unsigned bit = 0; bit < state.order; ++bit)
                    remainder |= InputBit() << bit;
                run.zeros -= remainder + 1;
            }
        }
        const bool unexpected = run.zeros == 0;
        if (unexpected) run.one = false;
        else --run.zeros;

        const unsigned value = prediction.bit ^ unsigned(unexpected);
        if (run.zeros == 0 && !run.one) {
            if (unexpected && prediction.state < 2) prediction.bit ^= 1;
            prediction.state = unexpected ? state.unexpected : state.predicted;
        }
        history_[plane] = uint16_t((history_[plane] << 1) | value);
        return value;
    }

    const std::function<uint8_t(uint32_t)>& reader_;
    uint32_t inputOffset_ = 0;
    uint8_t input_;
    uint8_t header_;
    unsigned bitsLeft_ = 4;
    struct Prediction { unsigned state = 0, bit = 0; };
    struct Run { unsigned zeros = 0; bool one = false; };
    std::array<Prediction, 32> predictions_{};
    std::array<Run, 8> runs_{};
    std::array<uint16_t, 8> history_{};
};
}

std::vector<uint8_t> DecompressSdd1(
    const std::function<uint8_t(uint32_t)>& reader, size_t outputSize) {
    return Decoder(reader).Decode(outputSize);
}

} // namespace snes::core
