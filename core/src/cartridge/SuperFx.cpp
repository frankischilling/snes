// snes emulator
// Clocked GSU execution, cartridge arbitration, and graphics writeback.
#include "snes/core/SuperFx.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <coroutine>
#include <exception>
#include <utility>

namespace snes::core {
namespace {
constexpr uint16_t Zero = 0x0002, Carry = 0x0004, Sign = 0x0008, Overflow = 0x0010;
constexpr uint16_t Go = 0x0020, RomBusy = 0x0040, Alt1 = 0x0100, Alt2 = 0x0200;
constexpr uint16_t With = 0x1000, Irq = 0x8000, StatusMask = 0x9f7e;

size_t Mirror(size_t address, size_t size) {
    if (!size) return 0;
    if (address < size) return address;
    const auto line = std::bit_floor(address);
    return size > line ? line + Mirror(address - line, size - line)
                       : Mirror(address - line, size);
}

bool IoBank(uint32_t address) { return (address & 0x400000) == 0; }
}

struct SuperFx::Impl {
    enum class Port { None, Rom, Ram };

    // A single suspended execution frame retains the instruction and its operands
    // across CPU clock slices, including slices ending in a cache fill or bus stall.
    struct Execution {
        struct promise_type {
            Execution get_return_object() {
                return Execution{std::coroutine_handle<promise_type>::from_promise(*this)};
            }
            std::suspend_always initial_suspend() noexcept { return {}; }
            std::suspend_always final_suspend() noexcept { return {}; }
            void return_void() noexcept {}
            void unhandled_exception() { std::terminate(); }
        };
        std::coroutine_handle<promise_type> handle{};
        Execution() = default;
        explicit Execution(std::coroutine_handle<promise_type> value) : handle(value) {}
        Execution(Execution&& other) noexcept : handle(std::exchange(other.handle, {})) {}
        Execution& operator=(Execution&& other) noexcept {
            if (handle) handle.destroy();
            handle = std::exchange(other.handle, {});
            return *this;
        }
        ~Execution() { if (handle) handle.destroy(); }
    };

    struct PixelRow {
        uint16_t position = 0xffff;
        uint8_t pending = 0;
        std::array<uint8_t, 8> colors{};
    };

    std::span<const uint8_t> rom;
    std::span<uint8_t> ram;
    std::array<uint16_t, 16> r{};
    std::array<uint8_t, 512> cache{};
    uint32_t validLines = 0;
    std::array<PixelRow, 2> pixels{};
    uint16_t status = 0, cacheBase = 0, lastRam = 0;
    uint8_t programBank = 0, romBank = 0, ramBank = 0;
    uint8_t config = 0, screenBase = 0, screenMode = 0, clockSelect = 0;
    uint8_t backupEnable = 0, color = 0, plotOptions = 0, pipeline = 1;
    unsigned source = 0, destination = 0;
    bool pcWritten = false, r14Written = false;
    uint32_t romRemaining = 0, romAddress = 0;
    uint8_t romBuffer = 0;
    uint32_t ramRemaining = 0, ramAddress = 0;
    uint8_t ramBuffer = 0;
    uint32_t waitRemaining = 0;
    Port waitPort = Port::None;
    Execution execution;

    Impl(std::span<const uint8_t> image, std::span<uint8_t> memory) : rom(image), ram(memory) {
        execution = Run();
    }

    bool Running() const { return (status & Go) != 0; }
    unsigned CoreCost() const { return clockSelect ? 1u : 2u; }
    unsigned MemoryCost() const { return clockSelect ? 5u : 6u; }
    bool Granted(Port port) const {
        return port == Port::None || (screenMode & (port == Port::Rom ? 0x10 : 0x08));
    }
    static Port BankPort(uint8_t bank) {
        if (bank < 0x60) return Port::Rom;
        if (bank >= 0x70 && bank <= 0x73) return Port::Ram;
        return Port::None;
    }
    uint8_t RamByte(uint32_t address) const {
        return ram.empty() ? 0 : ram[Mirror(address & 0x3ffff, ram.size())];
    }
    void StoreRam(uint32_t address, uint8_t value) {
        if (!ram.empty()) ram[Mirror(address & 0x3ffff, ram.size())] = value;
    }
    uint8_t MemoryByte(uint32_t address) const {
        const auto bank = uint8_t(address >> 16);
        if (BankPort(bank) == Port::Ram) return RamByte(address);
        if (BankPort(bank) != Port::Rom || rom.empty()) return 0;
        const size_t offset = bank < 0x40 ? (size_t(bank) << 15) | (address & 0x7fff)
                                          : address & 0x1fffff;
        return rom[Mirror(offset, std::min(rom.size(), size_t(0x200000)))];
    }
    void Flag(uint16_t mask, bool set) {
        status = uint16_t((status & ~mask) | (set ? mask : 0));
    }
    void ResultFlags(uint16_t value, bool byteSign = false) {
        Flag(Zero, value == 0);
        Flag(Sign, (value & (byteSign ? 0x80 : 0x8000)) != 0);
    }
    void SetRegister(unsigned index, uint16_t value) {
        r[index] = value;
        if (index == 14) r14Written = true;
        if (index == 15) pcWritten = true;
    }
    void ClearPrefix() {
        status &= uint16_t(~(Alt1 | Alt2 | With));
        source = destination = 0;
    }
    void StartRomRead() {
        romAddress = (uint32_t(romBank) << 16) | r[14];
        romRemaining = MemoryCost();
        status |= RomBusy;
    }
    void SetColor(uint8_t value) {
        if (plotOptions & 4) color = uint8_t((color & 0xf0) | (value >> 4));
        else if (plotOptions & 8) color = uint8_t((color & 0xf0) | (value & 15));
        else color = value;
    }
    unsigned Depth() const {
        constexpr unsigned depths[]{2, 4, 4, 8};
        return depths[screenMode & 3];
    }
    uint32_t PixelAddress(uint8_t x, uint8_t y) const {
        const unsigned heightMode = (plotOptions & 16) ? 3u :
                                    ((screenMode >> 2) & 1u) | ((screenMode >> 4) & 2u);
        const unsigned tx = x / 8, ty = y / 8;
        const unsigned tile = heightMode == 3 ? (tx % 16) + (ty % 16) * 16 +
                                               (tx / 16) * 256 + (ty / 16) * 512
                                             : tx * (16 + heightMode * 4) + ty;
        return (uint32_t(screenBase) << 10) + tile * Depth() * 8 + (y % 8) * 2;
    }
    static unsigned PlaneOffset(unsigned plane) { return (plane / 2) * 16 + plane % 2; }

    struct Wait {
        Impl& chip;
        uint32_t clocks;
        Port port = Port::None;
        bool await_ready() const noexcept { return clocks == 0 && chip.Granted(port); }
        void await_suspend(std::coroutine_handle<>) const noexcept {
            chip.waitRemaining = clocks;
            chip.waitPort = port;
        }
        void await_resume() const noexcept {}
    };

    struct Fetch {
        Impl& chip;
        uint16_t address;
        uint8_t bank;
        uint16_t offset;
        bool cached, hit;
        Fetch(Impl& value, uint16_t pc) : chip(value), address(pc), bank(value.programBank),
            offset(uint16_t(pc - value.cacheBase)), cached(offset < 512),
            hit(cached && (value.validLines & (uint32_t(1) << (offset / 16)))) {}
        bool await_ready() const noexcept { return false; }
        void await_suspend(std::coroutine_handle<>) const noexcept {
            const auto port = hit ? Port::None : BankPort(bank);
            const unsigned conflict = hit ? 0 : port == Port::Rom ? chip.romRemaining :
                                              port == Port::Ram ? chip.ramRemaining : 0;
            chip.waitRemaining = conflict + (hit ? chip.CoreCost() : chip.MemoryCost() * (cached ? 16 : 1));
            chip.waitPort = port;
        }
        uint8_t await_resume() const noexcept {
            if (!cached) return chip.MemoryByte((uint32_t(bank) << 16) | address);
            if (!hit) {
                const unsigned line = offset & 0x1f0;
                const uint32_t base = (uint32_t(bank) << 16) | (address & 0xfff0);
                for (unsigned i = 0; i < 16; ++i) chip.cache[line + i] = chip.MemoryByte(base + i);
                chip.validLines |= uint32_t(1) << (offset / 16);
            }
            return chip.cache[offset];
        }
    };

    struct RamRead {
        Impl& chip;
        uint32_t address;
        bool await_ready() const noexcept { return false; }
        void await_suspend(std::coroutine_handle<>) const noexcept {
            chip.waitRemaining = chip.ramRemaining + chip.MemoryCost();
            chip.waitPort = Port::Ram;
        }
        uint8_t await_resume() const noexcept { return chip.RamByte(address); }
    };

    struct RamWrite {
        Impl& chip;
        uint32_t address;
        uint8_t value;
        bool await_ready() const noexcept { return chip.ramRemaining == 0 && chip.Granted(Port::Ram); }
        void await_suspend(std::coroutine_handle<>) const noexcept {
            chip.waitRemaining = chip.ramRemaining;
            chip.waitPort = Port::Ram;
        }
        void await_resume() const noexcept {
            chip.ramAddress = address;
            chip.ramBuffer = value;
            chip.ramRemaining = chip.MemoryCost();
        }
    };

    struct RomRead {
        Impl& chip;
        bool await_ready() const noexcept { return chip.romRemaining == 0; }
        void await_suspend(std::coroutine_handle<>) const noexcept {
            chip.waitRemaining = chip.romRemaining;
            chip.waitPort = BankPort(uint8_t(chip.romAddress >> 16));
        }
        uint8_t await_resume() const noexcept { return chip.romBuffer; }
    };

    struct FlushPixels {
        Impl& chip;
        unsigned index;
        uint32_t address;
        unsigned depth;
        FlushPixels(Impl& value, unsigned row) : chip(value), index(row),
            address(value.PixelAddress(uint8_t(value.pixels[row].position * 8),
                                       uint8_t(value.pixels[row].position >> 5))), depth(value.Depth()) {}
        bool await_ready() const noexcept { return chip.pixels[index].pending == 0; }
        void await_suspend(std::coroutine_handle<>) const noexcept {
            chip.waitRemaining = chip.ramRemaining + depth * chip.MemoryCost() *
                                 (chip.pixels[index].pending == 0xff ? 1 : 2);
            chip.waitPort = Port::Ram;
        }
        void await_resume() const noexcept {
            auto& row = chip.pixels[index];
            if (!row.pending) return;
            for (unsigned plane = 0; plane < depth; ++plane) {
                uint8_t packed = 0;
                for (unsigned lane = 0; lane < 8; ++lane)
                    packed |= uint8_t(((row.colors[lane] >> plane) & 1) << (7 - lane));
                const auto target = address + PlaneOffset(plane);
                chip.StoreRam(target, uint8_t((packed & row.pending) | (chip.RamByte(target) & ~row.pending)));
            }
            row.pending = 0;
        }
    };

    void TickBuffers(uint32_t clocks) {
        if (romRemaining && Granted(BankPort(uint8_t(romAddress >> 16)))) {
            romRemaining -= std::min(clocks, romRemaining);
            if (!romRemaining) { romBuffer = MemoryByte(romAddress); status &= uint16_t(~RomBusy); }
        }
        if (ramRemaining && Granted(Port::Ram)) {
            ramRemaining -= std::min(clocks, ramRemaining);
            if (!ramRemaining) StoreRam(ramAddress, ramBuffer);
        }
    }

    void Advance(uint64_t clocks) {
        if (!clocks) return;
        for (;;) {
            // Resume at the exact completion boundary, even when this slice ends
            // there. The next instruction will suspend before consuming more time.
            while (Running() && waitRemaining == 0 && Granted(waitPort)) {
                waitPort = Port::None;
                execution.handle.resume();
            }
            if (!clocks) return;
            uint64_t elapsed = std::min(clocks, uint64_t(0xffffffff));
            const bool advancing = Running() && Granted(waitPort);
            if (advancing && waitRemaining) elapsed = std::min(elapsed, uint64_t(waitRemaining));
            if (romRemaining && Granted(BankPort(uint8_t(romAddress >> 16))))
                elapsed = std::min(elapsed, uint64_t(romRemaining));
            if (ramRemaining && Granted(Port::Ram)) elapsed = std::min(elapsed, uint64_t(ramRemaining));
            TickBuffers(uint32_t(elapsed));
            if (advancing) waitRemaining -= uint32_t(elapsed);
            clocks -= elapsed;
        }
    }

    Execution Run() {
        for (;;) {
            if (!Running()) co_await std::suspend_always{};
            const uint8_t opcode = pipeline;
            pipeline = co_await Fetch{*this, r[15]};
            pcWritten = r14Written = false;
            const unsigned mode = (status >> 8) & 3;
            const unsigned n = opcode & 15;
            const uint16_t input = r[source];
            // Store-immediate instructions sample their register before consuming
            // the address bytes; this also defines the otherwise unusual R15 case.
            const uint16_t operandRegister = r[n];
            uint16_t immediate = 0;
            if ((opcode >= 0x05 && opcode <= 0x0f) || (opcode & 0xf0) == 0xa0 || opcode >= 0xf0) {
                immediate = pipeline;
                ++r[15];
                pipeline = co_await Fetch{*this, r[15]};
                if (opcode >= 0xf0) {
                    immediate |= uint16_t(pipeline) << 8;
                    ++r[15];
                    pipeline = co_await Fetch{*this, r[15]};
                }
            }
            bool clearPrefix = true;
            if (opcode >= 0x05 && opcode <= 0x0f) {
                const bool s = (status & Sign) != 0, v = (status & Overflow) != 0;
                bool take = false;
                switch (opcode) {
                case 0x05: take = true; break;
                case 0x06: take = s == v; break;
                case 0x07: take = s != v; break;
                case 0x08: take = !(status & Zero); break;
                case 0x09: take = (status & Zero) != 0; break;
                case 0x0a: take = !s; break;
                case 0x0b: take = s; break;
                case 0x0c: take = !(status & Carry); break;
                case 0x0d: take = (status & Carry) != 0; break;
                case 0x0e: take = !v; break;
                case 0x0f: take = v; break;
                }
                if (take) SetRegister(15, uint16_t(r[15] + int8_t(immediate)));
                clearPrefix = false;
            } else if (opcode >= 0x10 && opcode <= 0x1f) {
                if (status & With) SetRegister(n, input);
                else { destination = n; clearPrefix = false; }
            } else if (opcode >= 0x20 && opcode <= 0x2f) {
                source = destination = n;
                status |= With;
                clearPrefix = false;
            } else if (opcode >= 0x30 && opcode <= 0x3b) {
                lastRam = r[n];
                const uint32_t base = uint32_t(ramBank) << 16;
                co_await RamWrite{*this, base | lastRam, uint8_t(input)};
                if (!(mode & 1)) co_await RamWrite{*this, base | (lastRam ^ 1u), uint8_t(input >> 8)};
            } else if (opcode >= 0x40 && opcode <= 0x4b) {
                lastRam = r[n];
                const uint32_t base = uint32_t(ramBank) << 16;
                uint16_t value = co_await RamRead{*this, base | lastRam};
                if (!(mode & 1)) value |= uint16_t(co_await RamRead{*this, base | (lastRam ^ 1u)}) << 8;
                SetRegister(destination, value);
            } else if (opcode >= 0x50 && opcode <= 0x6f) {
                const bool subtract = opcode >= 0x60;
                const uint16_t right = (mode & 2) && !(subtract && mode == 3) ? uint16_t(n) : r[n];
                const int borrow = subtract && mode == 1 && !(status & Carry);
                const int carry = !subtract && (mode & 1) && (status & Carry);
                const int32_t value = subtract ? int32_t(input) - right - borrow : int32_t(input) + right + carry;
                Flag(Carry, subtract ? value >= 0 : value > 0xffff);
                Flag(Overflow, ((subtract ? (input ^ right) : ~(input ^ right)) & (input ^ value) & 0x8000) != 0);
                ResultFlags(uint16_t(value));
                if (!(subtract && mode == 3)) SetRegister(destination, uint16_t(value));
            } else if ((opcode >= 0x71 && opcode <= 0x7f) || (opcode >= 0xc1 && opcode <= 0xcf)) {
                const uint16_t right = (mode & 2) ? uint16_t(n) : r[n];
                const auto value = uint16_t(opcode < 0x80 ? input & ((mode & 1) ? uint16_t(~right) : right)
                                                          : (mode & 1) ? input ^ right : input | right);
                SetRegister(destination, value);
                ResultFlags(value);
            } else if (opcode >= 0x80 && opcode <= 0x8f) {
                const uint8_t right = uint8_t((mode & 2) ? n : r[n]);
                const auto value = uint16_t((mode & 1) ? uint8_t(input) * right : int8_t(input) * int8_t(right));
                if (!(config & 0x20)) co_await Wait{*this, CoreCost()};
                SetRegister(destination, value);
                ResultFlags(value);
            } else if (opcode >= 0x91 && opcode <= 0x94) {
                r[11] = uint16_t(r[15] + n);
            } else if (opcode >= 0x98 && opcode <= 0x9d) {
                if (mode & 1) {
                    programBank = uint8_t(r[n] & 0x7f);
                    SetRegister(15, input);
                    cacheBase = r[15] & 0xfff0;
                    validLines = 0;
                } else SetRegister(15, r[n]);
            } else if ((opcode & 0xf0) == 0xa0 || opcode >= 0xf0) {
                if (!mode) SetRegister(n, opcode < 0xf0 ? uint16_t(int16_t(int8_t(immediate))) : immediate);
                else {
                    lastRam = opcode < 0xf0 ? uint16_t(immediate * 2) : immediate;
                    const uint32_t base = uint32_t(ramBank) << 16;
                    if (mode & 1) {
                        const uint8_t low = co_await RamRead{*this, base | lastRam};
                        const uint8_t high = co_await RamRead{*this, base | (lastRam ^ 1u)};
                        SetRegister(n, uint16_t(low | (uint16_t(high) << 8)));
                    } else {
                        co_await RamWrite{*this, base | lastRam, uint8_t(operandRegister)};
                        co_await RamWrite{*this, base | (lastRam ^ 1u), uint8_t(operandRegister >> 8)};
                    }
                }
            } else if (opcode >= 0xb0 && opcode <= 0xbf) {
                if (status & With) {
                    const uint16_t value = r[n];
                    SetRegister(destination, value);
                    ResultFlags(value);
                    Flag(Overflow, (value & 0x80) != 0);
                } else { source = n; clearPrefix = false; }
            } else if ((opcode >= 0xd0 && opcode <= 0xde) || (opcode >= 0xe0 && opcode <= 0xee)) {
                SetRegister(n, uint16_t(r[n] + (opcode < 0xe0 ? 1 : -1)));
                ResultFlags(r[n]);
            } else {
                switch (opcode) {
                case 0x00:
                    status &= uint16_t(~Go);
                    if (!(config & 0x80)) status |= Irq;
                    pipeline = 1;
                    break;
                case 0x01: break;
                case 0x02: {
                    const uint16_t base = r[15] & 0xfff0;
                    if (base != cacheBase) { cacheBase = base; validLines = 0; }
                    break;
                }
                case 0x03: case 0x04: case 0x96: case 0x97: {
                    uint16_t value;
                    const bool carry = (status & Carry) != 0;
                    if (opcode == 0x04) value = uint16_t((input << 1) | carry);
                    else if (opcode == 0x97) value = uint16_t((input >> 1) | (carry ? 0x8000 : 0));
                    else if (opcode == 0x96) value = (mode & 1) && input == 0xffff ? 0 : uint16_t(int16_t(input) >> 1);
                    else value = input >> 1;
                    Flag(Carry, (input & (opcode == 0x04 ? 0x8000 : 1)) != 0);
                    SetRegister(destination, value);
                    ResultFlags(value);
                    break;
                }
                case 0x3c:
                    --r[12];
                    ResultFlags(r[12]);
                    if (r[12]) SetRegister(15, r[13]);
                    break;
                case 0x3d: case 0x3e: case 0x3f:
                    status = uint16_t((status & ~With) | ((opcode - 0x3c) << 8));
                    clearPrefix = false;
                    break;
                case 0x4c:
                    if (mode & 1) {
                        co_await FlushPixels{*this, 1};
                        co_await FlushPixels{*this, 0};
                        uint16_t value = 0;
                        const uint32_t address = PixelAddress(uint8_t(r[1]), uint8_t(r[2]));
                        for (unsigned plane = 0; plane < Depth(); ++plane) {
                            const uint8_t packed = co_await RamRead{*this, address + PlaneOffset(plane)};
                            value |= uint16_t(((packed >> (7 - (r[1] & 7))) & 1) << plane);
                        }
                        SetRegister(destination, value);
                        ResultFlags(value);
                    } else {
                        const uint8_t x = uint8_t(r[1]), y = uint8_t(r[2]);
                        ++r[1];
                        const bool transparent = !(plotOptions & 1) &&
                            (((screenMode & 3) != 3 || (plotOptions & 8)) ? !(color & 15) : !color);
                        if (!transparent) {
                            const auto row = uint16_t(unsigned(y) * 32 + x / 8);
                            if (pixels[0].position != row) {
                                co_await FlushPixels{*this, 1};
                                pixels[1] = pixels[0];
                                pixels[0].position = row;
                                pixels[0].pending = 0;
                            }
                            uint8_t value = color;
                            if ((plotOptions & 2) && (screenMode & 3) != 3)
                                value = uint8_t((((x ^ y) & 1) ? value >> 4 : value) & 15);
                            pixels[0].colors[x & 7] = value;
                            pixels[0].pending |= uint8_t(0x80 >> (x & 7));
                            if (pixels[0].pending == 0xff) {
                                co_await FlushPixels{*this, 1};
                                pixels[1] = pixels[0];
                                pixels[0].pending = 0;
                            }
                        }
                    }
                    break;
                case 0x4d: case 0x4f: case 0x95: case 0x9e: case 0xc0: {
                    const uint16_t value = opcode == 0x4d ? uint16_t((input << 8) | (input >> 8)) :
                        opcode == 0x4f ? uint16_t(~input) : opcode == 0x95 ? uint16_t(int16_t(int8_t(input))) :
                        opcode == 0x9e ? uint16_t(input & 0xff) : uint16_t(input >> 8);
                    SetRegister(destination, value);
                    ResultFlags(value, opcode == 0x9e || opcode == 0xc0);
                    break;
                }
                case 0x4e:
                    if (mode & 1) plotOptions = uint8_t(input & 31);
                    else SetColor(uint8_t(input));
                    break;
                case 0x70: {
                    const uint16_t value = uint16_t((r[7] & 0xff00) | (r[8] >> 8));
                    SetRegister(destination, value);
                    Flag(Overflow, (value & 0xc0c0) != 0);
                    Flag(Sign, (value & 0x8080) != 0);
                    Flag(Carry, (value & 0xe0e0) != 0);
                    Flag(Zero, (value & 0xf0f0) != 0);
                    break;
                }
                case 0x90: {
                    const uint32_t base = uint32_t(ramBank) << 16;
                    co_await RamWrite{*this, base | lastRam, uint8_t(input)};
                    co_await RamWrite{*this, base | (lastRam ^ 1u), uint8_t(input >> 8)};
                    break;
                }
                case 0x9f: {
                    const auto product = uint32_t(int32_t(int16_t(input)) * int32_t(int16_t(r[6])));
                    co_await Wait{*this, ((config & 0x20) ? 3u : 7u) * CoreCost()};
                    if (mode & 1) r[4] = uint16_t(product);
                    SetRegister(destination, uint16_t(product >> 16));
                    ResultFlags(uint16_t(product >> 16));
                    Flag(Carry, (product & 0x8000) != 0);
                    break;
                }
                case 0xdf:
                    if (!(mode & 2)) SetColor(co_await RomRead{*this});
                    else if (mode == 2) {
                        co_await Wait{*this, ramRemaining, ramRemaining ? Port::Ram : Port::None};
                        ramBank = uint8_t(input & 3);
                    } else {
                        co_await RomRead{*this};
                        romBank = uint8_t(input & 0x7f);
                    }
                    break;
                case 0xef: {
                    const uint8_t byte = co_await RomRead{*this};
                    const uint16_t value = mode == 0 ? byte : mode == 1 ? uint16_t((input & 0xff) | (uint16_t(byte) << 8)) :
                        mode == 2 ? uint16_t((input & 0xff00) | byte) : uint16_t(int16_t(int8_t(byte)));
                    SetRegister(destination, value);
                    break;
                }
                }
            }
            if (clearPrefix) ClearPrefix();
            if (r14Written) StartRomRead();
            if (!pcWritten) ++r[15];
        }
    }

    uint8_t ReadIo(uint16_t address) {
        if (address >= 0x3100) return cache[(address - 0x3100 + cacheBase) & 511];
        if (address < 0x3020) return uint8_t(r[(address >> 1) & 15] >> ((address & 1) * 8));
        switch (address) {
        case 0x3030: return uint8_t(status & StatusMask);
        case 0x3031: {
            const auto value = uint8_t((status & StatusMask) >> 8);
            status &= uint16_t(~Irq);
            return value;
        }
        case 0x3034: return programBank;
        case 0x3036: return romBank;
        case 0x303b: return 0x04;
        case 0x303c: return ramBank;
        case 0x303e: return uint8_t(cacheBase);
        case 0x303f: return uint8_t(cacheBase >> 8);
        default: return 0;
        }
    }

    void WriteIo(uint16_t address, uint8_t value) {
        if (address >= 0x3100) {
            const unsigned offset = (address - 0x3100 + cacheBase) & 511;
            cache[offset] = value;
            if ((offset & 15) == 15) validLines |= uint32_t(1) << (offset / 16);
            return;
        }
        if (address < 0x3020) {
            const unsigned n = (address >> 1) & 15;
            r[n] = (address & 1) ? uint16_t((r[n] & 0xff) | (uint16_t(value) << 8))
                                 : uint16_t((r[n] & 0xff00) | value);
            if (n == 14) StartRomRead();
            if (address == 0x301f) status |= Go;
            return;
        }
        switch (address) {
        case 0x3030: {
            const bool wasRunning = Running();
            status = uint16_t((status & 0xff00) | (value & StatusMask));
            if (wasRunning && !Running()) { cacheBase = 0; validLines = 0; }
            break;
        }
        case 0x3031: status = uint16_t((status & 0xff) | ((uint16_t(value) << 8) & StatusMask)); break;
        case 0x3033: backupEnable = value & 1; break;
        case 0x3034: programBank = value & 0x7f; validLines = 0; break;
        case 0x3037: config = value & 0xa0; break;
        case 0x3038: screenBase = value; break;
        case 0x3039: clockSelect = value & 1; break;
        case 0x303a: screenMode = value & 0x3f; break;
        default: break;
        }
    }

    bool CpuRamAddress(uint32_t address, uint32_t& offset) const {
        const unsigned bank = (address >> 16) & 0xff;
        const unsigned low = address & 0xffff;
        if (IoBank(address) && low >= 0x6000 && low < 0x8000) { offset = low & 0x1fff; return true; }
        // Four-megabyte boards connect the entire upper linear window to ROM.
        // Smaller boards expose only F0/F1 as aliases of the first two RAM banks.
        if (bank >= 0xf2 || (rom.size() > 0x200000 && bank >= 0xc0)) return false;
        const unsigned mirrored = bank & 0x7f;
        if (mirrored >= 0x70 && mirrored <= 0x73 && (mirrored <= 0x71 || ram.size() > 0x20000)) {
            offset = ((mirrored - 0x70) << 16) | low;
            return true;
        }
        return false;
    }
};

SuperFx::SuperFx(std::span<const uint8_t> rom, std::span<uint8_t> ram) : impl_(std::make_unique<Impl>(rom, ram)) {}
SuperFx::~SuperFx() = default;
SuperFx::SuperFx(SuperFx&&) noexcept = default;
SuperFx& SuperFx::operator=(SuperFx&&) noexcept = default;
void SuperFx::Reset() { Reset(impl_->rom, impl_->ram); }
void SuperFx::Reset(std::span<const uint8_t> rom, std::span<uint8_t> ram) { impl_ = std::make_unique<Impl>(rom, ram); }
void SuperFx::Advance(uint64_t clocks) { impl_->Advance(clocks); }
bool SuperFx::CpuIrqPending() const noexcept { return (impl_->status & Irq) != 0; }

bool SuperFx::Selects(uint32_t address) noexcept {
    address &= 0xffffff;
    const unsigned low = address & 0xffff, bank = (address >> 16) & 0x7f;
    return (IoBank(address) && ((low >= 0x3000 && low <= 0x32ff) || low >= 0x6000)) ||
           (bank >= 0x40 && bank <= 0x5f) || (bank >= 0x70 && bank <= 0x73) || address >= 0xc00000;
}

uint8_t SuperFx::ReadCpu(uint32_t address, uint8_t openBus) {
    address &= 0xffffff;
    const auto low = uint16_t(address);
    if (IoBank(address) && low >= 0x3000 && low <= 0x32ff) return impl_->ReadIo(low);
    uint32_t offset = 0;
    if (impl_->CpuRamAddress(address, offset)) {
        if (impl_->ram.empty() || (impl_->Running() && (impl_->screenMode & 8))) return openBus;
        return impl_->RamByte(offset);
    }
    const unsigned bank = (address >> 16) & 0x7f;
    const bool extended = impl_->rom.size() > 0x200000 && address >= 0xc00000;
    const bool mapped = extended || (bank < 0x40 && low >= 0x8000) || (bank >= 0x40 && bank <= 0x5f);
    if (!mapped || impl_->rom.empty()) return openBus;
    if (impl_->Running() && (impl_->screenMode & 0x10)) {
        constexpr std::array<uint8_t, 16> driven{0, 1, 0, 1, 4, 1, 0, 1, 0, 1, 8, 1, 0, 1, 12, 1};
        return driven[low & 15];
    }
    if (extended) return impl_->rom[Mirror(address & 0x3fffff, std::min(impl_->rom.size(), size_t(0x400000)))];
    return impl_->MemoryByte((bank << 16) | low);
}

void SuperFx::WriteCpu(uint32_t address, uint8_t value) {
    address &= 0xffffff;
    const auto low = uint16_t(address);
    if (IoBank(address) && low >= 0x3000 && low <= 0x32ff) { impl_->WriteIo(low, value); return; }
    uint32_t offset = 0;
    if (impl_->CpuRamAddress(address, offset) && !(impl_->Running() && (impl_->screenMode & 8)))
        impl_->StoreRam(offset, value);
}

} // namespace snes::core
