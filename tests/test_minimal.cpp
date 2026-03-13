#include <cstdio>
#include "snes/core/Ppu.hpp"
int main() {
    std::printf("Before Ppu\n");
    snes::core::Ppu ppu;
    std::printf("After Ppu\n");
    return 0;
}
