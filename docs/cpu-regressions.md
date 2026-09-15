# CPU regression checks

The CPU suites use small programs in synthetic memory to check interrupt entry, stack boundaries, and stopped or waiting states. Checks remain active in Release builds.

## Stop and wait

`STP` stops instruction execution until reset. Each subsequent CPU step returns control to the system scheduler without fetching instructions or servicing interrupts. The stop test has a five-second timeout so an instruction that hangs cannot stall the test run.

`WAI` resumes when an interrupt arrives. An unmasked IRQ or NMI enters its handler with the wait state cleared. A masked IRQ releases the wait state and continues at the next instruction without entering a handler. Tests cover interrupts arriving before the wait instruction and while the CPU is waiting, including handler execution and return through `RTI`.

## Stack boundaries

In emulation mode, interrupt entry wraps each stack write within $0100-$01FF. Tests exercise BRK, COP, IRQ, and NMI with the stack pointer at $0100, $0101, $0102, and $01FF. They check write addresses, saved status, interrupt vectors, and restoration of the program counter, bank, flags, and stack pointer through `RTI`. Native-mode cases check the full 16-bit stack behavior.

PHD, PLD, PEA, PEI, PER, JSL, RTL, and indexed indirect JSR can cross the stack-page boundary during an instruction. In emulation mode, they restore the stack pointer's high byte to $01 before the next instruction. Tests verify the crossing, final pointer, and address used by a following PHA. Native-mode cases retain the full pointer.

## Bus and interrupt sampling

Every CPU read, write, and internal cycle advances the connected hardware. A word I/O instruction can observe different horizontal positions for its two bytes. Tests place an HBlank edge between operand fetches and a register read, and an NMI edge inside an instruction.

Instruction handlers sample IRQ before their final cycle. CLI, SEI, REP, SEP, and PLP can change the interrupt mask after that sample. A latched IRQ keeps the sampled decision instead of testing the new mask again at entry. The regression cases check both delayed unmasking and an interrupt that was already accepted before the instruction masks it.

An earlier bus operation's interrupt lock clears before the following access; a new `$4200` write retains its lock until then. Regressions enable NMI during VBlank through both byte and word stores. They check that the sample between a word store's bytes stays suppressed while the following instruction accepts the NMI without an extra delay. `snes_bus_scheduling_tests` also checks DMA startup and resumption at CPU cycle boundaries.

## Run the checks

```powershell
cmake --build --preset build-release
ctest --test-dir out/build/release -R 'snes_(cpu|bus|dma)' --output-on-failure
```

These tests cover the cases above, not every opcode or interrupt-timing interaction.
