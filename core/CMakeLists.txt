add_library(snes_core
  src/Cartridge.cpp
  src/Cpu65816.cpp
  src/CpuIoRegisters.cpp
  src/Dma.cpp
  src/Dsp.cpp
  src/Dsp1.cpp
  src/Ppu.cpp
  src/PpuRender.cpp
  src/Processor65816.cpp
  src/SnesCpu.cpp
  src/MemoryBus.cpp
  src/Emulator.cpp
  src/Logging.cpp
  src/Spc700.cpp
  src/Smp.cpp
  src/Timing.cpp
  src/IrqController.cpp
  src/AutoJoypad.cpp
)

target_include_directories(snes_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
  target_compile_options(snes_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(snes_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
