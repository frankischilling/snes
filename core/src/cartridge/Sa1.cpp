// snes emulator
// SA-1 bus, register file, and processor scheduling.
#include "snes/core/Sa1.hpp"

#include <bit>

namespace snes::core {

Sa1::Sa1(std::span<const uint8_t> rom, std::span<uint8_t> bwram)
    : rom_(rom), bwram_(bwram) {
    Reset();
}

void Sa1::Reset(std::span<const uint8_t> rom, std::span<uint8_t> bwram) {
    rom_ = rom;
    bwram_ = bwram;
    expansion_ = {};
    broadcastBoard_ = false;
    Reset();
}

void Sa1::Reset() {
    power();
    iram_.fill(0);
    registers_.fill(0);
    registers_[0x00] = 0x20;
    for (uint8_t bank = 0; bank < 4; ++bank) registers_[0x20 + bank] = bank;
    registers_[0x28] = 0x0f;
    cpuFlags_ = sa1Flags_ = 0;
    nmiServiced_ = sampledIrq_ = hardwareVector_ = false;
    hardwareVectorAddress_ = hardwareVectorValue_ = 0;
    requestedClocks_ = executedClocks_ = 0;
    hClocks_ = vCounter_ = latchedH_ = latchedV_ = 0;
    timerMatch_ = false;
    arithmeticResult_ = arithmeticNext_ = 0;
    arithmeticClocks_ = 0;
    arithmeticOverflow_ = arithmeticNextOverflow_ = false;
    bitAddress_ = bitResult_ = bitOffset_ = 0;
    dma_ = {};
    conversionActive_ = false;
    conversionSource_ = conversionDestination_ = 0;
    conversionTile_ = UINT32_MAX;
    conversionRows_ = conversionBuffer_ = 0;
}

void Sa1::SetPal(bool enabled) noexcept {
    videoLines_ = enabled ? 312 : 262;
    if (!(registers_[0x10] & 0x80)) vCounter_ %= videoLines_;
}

uint8_t Sa1::ReadCpu(uint32_t address, uint8_t openBus) {
    return ReadMemory(address & 0xffffff, openBus, Side::Cpu);
}

void Sa1::WriteCpu(uint32_t address, uint8_t value) {
    WriteMemory(address & 0xffffff, value, Side::Cpu);
}

uint8_t Sa1::ReadSa1(uint32_t address, uint8_t openBus) {
    return ReadMemory(address & 0xffffff, openBus, Side::Sa1);
}

void Sa1::WriteSa1(uint32_t address, uint8_t value) {
    WriteMemory(address & 0xffffff, value, Side::Sa1);
}

bool Sa1::CpuIrqPending() const noexcept {
    return (cpuFlags_ & registers_[0x01] & 0xa0) != 0;
}

bool Sa1::Sa1IrqPending() const noexcept {
    return (sa1Flags_ & registers_[0x0a] & 0xe0) != 0;
}

bool Sa1::Sa1NmiPending() const noexcept {
    return (sa1Flags_ & registers_[0x0a] & 0x10) && !nmiServiced_;
}

void Sa1::Advance(uint32_t masterClocks) {
    requestedClocks_ += masterClocks;
    // Processor65816 executes whole instructions. Carry any overshoot into the
    // next call so small scheduler slices cannot make the chip run too fast.
    while (executedClocks_ < requestedClocks_) {
        if (dma_.active) {
            StepDma();
        } else if ((registers_[0x00] & 0x60) || regs().stp) {
            Clock(static_cast<uint32_t>(requestedClocks_ - executedClocks_));
        } else {
            StepProcessor();
        }
    }
}

void Sa1::StepProcessor() {
    if (Sa1NmiPending()) {
        nmiServiced_ = true;
        InterruptProcessor(Interrupt::Nmi);
        return;
    }
    const bool irq = Sa1IrqPending();
    if (sampledIrq_ || (regs().wai && irq && !flagI())) {
        sampledIrq_ = false;
        InterruptProcessor(Interrupt::Irq);
        return;
    }
    if (regs().wai) {
        if (irq) regs().wai = false;
        idle();
        sampledIrq_ = Sa1IrqPending() && !flagI();
        return;
    }
    instruction();
}

void Sa1::ResetProcessor() {
    power();
    regs().pc = RegisterWord(0x03);
    sampledIrq_ = false;
    nmiServiced_ = false;
    hardwareVector_ = false;
}

void Sa1::InterruptProcessor(Interrupt type) {
    hardwareVector_ = true;
    hardwareVectorAddress_ = vectorAddress(type);
    hardwareVectorValue_ = RegisterWord(type == Interrupt::Nmi ? 0x05 : 0x07);
    enterInterrupt(type);
    hardwareVector_ = false;
}

uint8_t Sa1::AccessClocks(uint32_t address) const noexcept {
    const uint8_t bank = static_cast<uint8_t>(address >> 16);
    if ((bank >= 0x40 && bank <= (broadcastBoard_ ? 0x7d : 0x4f)) || (bank >= 0x60 && bank <= 0x6f) ||
        ((address & 0x40e000) == 0x006000)) return 4;
    return 2;
}

void Sa1::idle() { Clock(2); }

uint8_t Sa1::read(uint32_t address) {
    address &= 0xffffff;
    Clock(AccessClocks(address));
    uint8_t value;
    if (hardwareVector_ && (address == hardwareVectorAddress_ || address == hardwareVectorAddress_ + 1u)) {
        value = static_cast<uint8_t>(hardwareVectorValue_ >> ((address & 1) * 8));
    } else {
        value = ReadMemory(address, regs().mdr, Side::Sa1);
    }
    regs().mar = address;
    regs().mdr = value;
    return value;
}

void Sa1::write(uint32_t address, uint8_t value) {
    address &= 0xffffff;
    Clock(AccessClocks(address));
    WriteMemory(address, value, Side::Sa1);
    regs().mar = address;
    regs().mdr = value;
}

void Sa1::lastCycle() {
    regs().irq = Sa1IrqPending();
    sampledIrq_ = regs().irq && !flagI();
    if (regs().wai && (regs().irq || Sa1NmiPending())) regs().wai = false;
}

bool Sa1::interruptPending() const {
    return Sa1NmiPending() || (Sa1IrqPending() && !flagI());
}

void Sa1::Clock(uint32_t masterClocks) {
    executedClocks_ += masterClocks;
    AdvanceTimer(masterClocks);
    if (arithmeticClocks_) {
        if (masterClocks >= arithmeticClocks_) {
            arithmeticClocks_ = 0;
            arithmeticResult_ = arithmeticNext_;
            arithmeticOverflow_ = arithmeticNextOverflow_;
        } else {
            arithmeticClocks_ -= static_cast<uint8_t>(masterClocks);
        }
    }
}

uint16_t Sa1::RegisterWord(uint8_t offset) const noexcept {
    return static_cast<uint16_t>(registers_[offset] | (registers_[offset + 1] << 8));
}

uint32_t Sa1::RegisterAddress(uint8_t offset) const noexcept {
    return RegisterWord(offset) | (uint32_t(registers_[offset + 2]) << 16);
}

void Sa1::StoreAddress(uint8_t offset, uint32_t address) noexcept {
    for (unsigned byte = 0; byte < 3; ++byte)
        registers_[offset + byte] = static_cast<uint8_t>(address >> (byte * 8));
}

size_t Sa1::Mirror(size_t address, size_t size) noexcept {
    if (!size) return 0;
    size_t base = 0;
    while (address >= size) {
        const size_t bit = std::bit_floor(address);
        address -= bit;
        if (size > bit) {
            size -= bit;
            base += bit;
        }
    }
    return base + address;
}

bool Sa1::RomOffset(uint32_t address, uint32_t& offset) const noexcept {
    const uint8_t bank = static_cast<uint8_t>(address >> 16);
    if (bank >= 0xc0) {
        const unsigned block = (bank - 0xc0) >> 4;
        offset = uint32_t(registers_[0x20 + block] & 7) * 0x100000 + (address & 0xfffff);
        return true;
    }
    if (!(bank & 0x40) && (address & 0x8000)) {
        const unsigned block = ((bank & 0x80) >> 6) | ((bank & 0x20) >> 5);
        const uint8_t mapping = registers_[0x20 + block];
        unsigned page = (mapping & 0x80) ? (mapping & 7) : block;
        if (broadcastBoard_ && (mapping & 7) >= 4 && !(mapping & 0x80)) page += 4;
        offset = page * 0x100000 + uint32_t(bank & 0x1f) * 0x8000 + (address & 0x7fff);
        return true;
    }
    return false;
}

uint8_t Sa1::ReadRom(uint32_t offset, uint8_t openBus) const {
    if (broadcastBoard_ && offset >= 0x400000)
        return expansion_.empty() ? openBus : expansion_[Mirror(offset - 0x400000, expansion_.size())];
    return rom_.empty() ? openBus : rom_[Mirror(offset, rom_.size())];
}

uint8_t Sa1::ReadBwram(uint32_t offset, uint8_t openBus) const {
    return bwram_.empty() ? openBus : bwram_[Mirror(offset & 0x3ffff, bwram_.size())];
}

bool Sa1::BwramWritable(uint32_t offset, Side) const noexcept {
    if (bwram_.empty()) return false;
    offset &= 0x3ffff;
    // Both enable registers feed the same external RAM write gate. Enabling
    // either port permits writes from both processors to the protected area.
    return ((registers_[0x26] | registers_[0x27]) & 0x80) ||
           offset >= (uint32_t{0x100} << (registers_[0x28] & 15));
}

void Sa1::WriteBwram(uint32_t offset, uint8_t value, Side side, bool dma) {
    if (!bwram_.empty() && (dma || BwramWritable(offset, side)))
        bwram_[Mirror(offset & 0x3ffff, bwram_.size())] = value;
}

void Sa1::WriteIram(uint32_t offset, uint8_t value, Side side) {
    offset &= 0x7ff;
    if (registers_[side == Side::Cpu ? 0x29 : 0x2a] & (1u << (offset >> 8)))
        iram_[offset] = value;
}

uint8_t Sa1::BitmapDepth() const noexcept { return (registers_[0x3f] & 0x80) ? 2 : 4; }

uint8_t Sa1::ReadBitmap(uint32_t pixel, uint8_t openBus) const {
    if (bwram_.empty()) return openBus;
    const unsigned depth = BitmapDepth();
    const unsigned perByte = 8 / depth;
    return static_cast<uint8_t>((ReadBwram(pixel / perByte) >> ((pixel % perByte) * depth)) & ((1u << depth) - 1));
}

void Sa1::WriteBitmap(uint32_t pixel, uint8_t value) {
    const unsigned depth = BitmapDepth();
    const unsigned perByte = 8 / depth;
    const unsigned shift = (pixel % perByte) * depth;
    const unsigned mask = ((1u << depth) - 1) << shift;
    const uint32_t offset = pixel / perByte;
    // The packed-pixel write port bypasses the linear BW-RAM protection gate.
    WriteBwram(offset, static_cast<uint8_t>((ReadBwram(offset) & ~mask) | ((value << shift) & mask)), Side::Sa1, true);
}

uint8_t Sa1::ReadMemory(uint32_t address, uint8_t openBus, Side side, bool io) {
    const uint8_t bank = static_cast<uint8_t>(address >> 16);
    const uint16_t low = static_cast<uint16_t>(address);
    if (!(bank & 0x40)) {
        if (low < 0x800 && side == Side::Sa1) return iram_[low];
        if (low >= 0x2200 && low <= 0x23ff)
            return io ? ReadRegister(low, openBus, side) : openBus;
        if (low >= 0x3000 && low <= 0x37ff) return iram_[low & 0x7ff];
        if (low >= 0x6000 && low <= 0x7fff) {
            const uint8_t mapping = registers_[side == Side::Cpu ? 0x24 : 0x25];
            if (side == Side::Sa1 && (mapping & 0x80))
                return ReadBitmap(uint32_t(mapping & 0x7f) * 0x2000 + (low & 0x1fff), openBus);
            const uint32_t offset = uint32_t(mapping & 0x1f) * 0x2000 + (low & 0x1fff);
            if (side == Side::Cpu && conversionActive_) return ReadConverted(offset);
            return ReadBwram(offset, openBus);
        }
    }
    if (side == Side::Sa1 && bank >= 0x60 && bank <= 0x6f)
        return ReadBitmap(address & 0xfffff, openBus);
    if (bank >= 0x40 && bank <= (broadcastBoard_ ? 0x7d : 0x4f)) {
        if (broadcastBoard_) address &= 0x1ffff;
        if (side == Side::Cpu && conversionActive_) return ReadConverted(address & 0x3ffff);
        return ReadBwram(address & 0x3ffff, openBus);
    }
    uint32_t romOffset;
    if (RomOffset(address, romOffset)) {
        // Vector substitution affects the bank-zero vector table, including
        // its native and emulation entries, without modifying the ROM image.
        if (side == Side::Cpu && bank == 0) {
            if ((registers_[0x09] & 0x10) && ((low & 0xffee) == 0xffea))
                return registers_[0x0c + (low & 1)];
            if ((registers_[0x09] & 0x40) && ((low & 0xffee) == 0xffee))
                return registers_[0x0e + (low & 1)];
        }
        return ReadRom(romOffset, openBus);
    }
    return openBus;
}

void Sa1::WriteMemory(uint32_t address, uint8_t value, Side side) {
    const uint8_t bank = static_cast<uint8_t>(address >> 16);
    const uint16_t low = static_cast<uint16_t>(address);
    if (!(bank & 0x40)) {
        if (low < 0x800 && side == Side::Sa1) { WriteIram(low, value, side); return; }
        if (low >= 0x2200 && low <= 0x23ff) { WriteRegister(low, value, side); return; }
        if (low >= 0x3000 && low <= 0x37ff) { WriteIram(low, value, side); return; }
        if (low >= 0x6000 && low <= 0x7fff) {
            const uint8_t mapping = registers_[side == Side::Cpu ? 0x24 : 0x25];
            if (side == Side::Sa1 && (mapping & 0x80))
                WriteBitmap(uint32_t(mapping & 0x7f) * 0x2000 + (low & 0x1fff), value);
            else
                WriteBwram(uint32_t(mapping & 0x1f) * 0x2000 + (low & 0x1fff), value, side);
            return;
        }
    }
    if (side == Side::Sa1 && bank >= 0x60 && bank <= 0x6f) WriteBitmap(address & 0xfffff, value);
    else if (bank >= 0x40 && bank <= (broadcastBoard_ ? 0x7d : 0x4f))
        WriteBwram(address & (broadcastBoard_ ? 0x1ffff : 0x3ffff), value, side);
}

bool Sa1::WritableRegister(uint16_t address, Side side) noexcept {
    if (address >= 0x2231 && address <= 0x2237) return true;
    if (side == Side::Cpu)
        return (address >= 0x2200 && address <= 0x2208) ||
               (address >= 0x2220 && address <= 0x2224) ||
               address == 0x2226 || address == 0x2228 || address == 0x2229;
    return (address >= 0x2209 && address <= 0x220f) ||
           (address >= 0x2210 && address <= 0x2215) || address == 0x2225 ||
           address == 0x2227 || address == 0x222a || address == 0x2230 ||
           address == 0x2238 || address == 0x2239 ||
           (address >= 0x223f && address <= 0x2254) ||
           (address >= 0x2258 && address <= 0x225b);
}

uint8_t Sa1::ReadRegister(uint16_t address, uint8_t openBus, Side side) {
    if (side == Side::Cpu) {
        if (address == 0x2300) return (registers_[0x09] & 0x5f) | (cpuFlags_ & 0xa0);
        if (address == 0x230e) return 0x23;
        return openBus;
    }
    if (address == 0x2301) return (registers_[0x00] & 15) | (sa1Flags_ & 0xf0);
    if (address == 0x2302) {
        latchedH_ = hClocks_ / 4;
        latchedV_ = vCounter_;
        return static_cast<uint8_t>(latchedH_);
    }
    if (address == 0x2303) return static_cast<uint8_t>(latchedH_ >> 8);
    if (address == 0x2304) return static_cast<uint8_t>(latchedV_);
    if (address == 0x2305) return static_cast<uint8_t>(latchedV_ >> 8);
    if (address >= 0x2306 && address <= 0x230a)
        return static_cast<uint8_t>(arithmeticResult_ >> ((address - 0x2306) * 8));
    if (address == 0x230b) return arithmeticOverflow_ ? 0x80 : 0;
    if (address == 0x230c) return static_cast<uint8_t>(bitResult_);
    if (address == 0x230d) {
        const uint8_t value = static_cast<uint8_t>(bitResult_ >> 8);
        if (registers_[0x58] & 0x80) AdvanceBits();
        return value;
    }
    return openBus;
}

void Sa1::WriteRegister(uint16_t address, uint8_t value, Side side) {
    if (!WritableRegister(address, side)) return;
    const uint8_t offset = static_cast<uint8_t>(address);
    const uint8_t previous = registers_[offset];
    registers_[offset] = value;
    switch (offset) {
    case 0x00:
        if ((previous & 0x20) && !(value & 0x20)) ResetProcessor();
        if (value & 0x80) sa1Flags_ |= 0x80;
        if (value & 0x10) {
            sa1Flags_ |= 0x10;
            nmiServiced_ = false;
        }
        break;
    case 0x02: cpuFlags_ &= static_cast<uint8_t>(~(value & 0xa0)); break;
    case 0x09: if (value & 0x80) cpuFlags_ |= 0x80; break;
    case 0x0a:
        if ((value & 0x10) && !(previous & 0x10)) nmiServiced_ = false;
        break;
    case 0x0b:
        sa1Flags_ &= static_cast<uint8_t>(~(value & 0xf0));
        if (value & 0x10) nmiServiced_ = false;
        break;
    case 0x10:
        hClocks_ %= (value & 0x80) ? 2048 : 1364;
        vCounter_ %= (value & 0x80) ? 512 : videoLines_;
        timerMatch_ = false;
        break;
    case 0x11: hClocks_ = vCounter_ = 0; timerMatch_ = false; break;
    case 0x12: case 0x13: case 0x14: case 0x15: timerMatch_ = false; break;
    case 0x30:
        if ((value & 0xb0) != 0xb0) conversionActive_ = false;
        if (!(value & 0x80)) {
            dma_.active = conversionActive_ = false;
            conversionRows_ = conversionBuffer_ = 0;
        } else if (!(previous & 0x80) || ((value ^ previous) & 0x30)) {
            conversionRows_ = conversionBuffer_ = 0;
        }
        break;
    case 0x31:
        if (value & 0x80) conversionActive_ = false;
        conversionTile_ = UINT32_MAX;
        break;
    case 0x36:
        if ((registers_[0x30] & 0xa4) == 0x80) StartDma();
        else if ((registers_[0x30] & 0xb0) == 0xb0) StartConversion();
        break;
    case 0x37: if ((registers_[0x30] & 0xa4) == 0x84) StartDma(); break;
    case 0x47: case 0x4f:
        if ((registers_[0x30] & 0xb0) == 0xa0) PushConversionRows();
        break;
    case 0x50:
        if (value & 2) {
            arithmeticResult_ = arithmeticNext_ = 0;
            arithmeticClocks_ = 0;
            arithmeticOverflow_ = arithmeticNextOverflow_ = false;
        }
        break;
    case 0x54: StartArithmetic(); break;
    case 0x58: AdvanceBits(); break;
    case 0x5b:
        bitAddress_ = RegisterAddress(0x59);
        bitOffset_ = 0;
        RefreshBitResult();
        break;
    default: break;
    }
}

} // namespace snes::core
