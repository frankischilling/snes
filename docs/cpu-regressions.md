# CPU regression checks

The CPU suites use small programs in synthetic memory to check interrupt entry, stack boundaries, and stopped or waiting states. Checks remain active in Release builds.

## Stop and wait

`STP` stops instruction execution until reset. Each subsequent CPU step returns control to the system scheduler without fetching instructions or servicing interrupts. The stop test has a five-second timeout so an instruction that hangs cannot stall the test run.

`WAI` resumes when an interrupt arrives. An unmasked IRQ or NMI enters its handler with the wait state cleared. A masked IRQ releases the wait state and continues at the next instruction without entering a handler. Tests cover interrupts arriving before the wait instruction and while the CPU is waiting, including handler execution and return through `RTI`.

## Stack boundaries

In emulation mode, interrupt entry wraps each stack write within $0100-$01FF. Tests exercise BRK, COP, IRQ, and NMI with the stack pointer at $0100, $0101, $0102, and $01FF. They check write addresses, saved status, interrupt vectors, and restoration of the program counter, bank, flags, and stack pointer through `RTI`. Native-mode cases check the full 16-bit stack behavior.

PHD, PLD, PEA, PEI, PER, JSL, RTL, and indexed indirect JSR can cross the stack-page boundary during an instruction. In emulation mode, they restore the stack pointer's high byte to $01 before the next instruction. Tests verify the crossing, final pointer, and address used by a following PHA. Native-mode cases retain the full pointer.

## Run the checks

```powershell
cmake --build --preset build-release
ctest --test-dir out/build/release -R snes_cpu_ --output-on-failure
```

These tests cover the cases above, not every opcode or interrupt-timing interaction.
