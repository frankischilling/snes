add_library(snes_core
  src/cartridge/Cartridge.cpp
  src/cartridge/BroadcastCartridge.cpp
  src/cartridge/Sdd1.cpp
  src/cartridge/St010.cpp
  src/cpu/Cpu65816.cpp
  src/cpu/Processor65816.cpp
  src/cpu/SnesCpu.cpp
  src/memory/MemoryBus.cpp
  src/ppu/Ppu.cpp
  src/ppu/PpuRender.cpp
  src/audio/Dsp.cpp
  src/audio/Dsp1.cpp
  src/audio/Dsp2.cpp
  src/audio/Spc700.cpp
  src/audio/Smp.cpp
  src/audio/Srtc.cpp
  src/io/AutoJoypad.cpp
  src/io/ControllerPorts.cpp
  src/io/CpuIoRegisters.cpp
  src/io/IrqController.cpp
  src/dma/Dma.cpp
  src/timing/Timing.cpp
  src/system/Emulator.cpp
  src/system/Logging.cpp
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
