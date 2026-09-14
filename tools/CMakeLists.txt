add_executable(snes_trace_dump
  src/trace_dump.cpp
)

target_link_libraries(snes_trace_dump
  PRIVATE
    snes_core
)

add_executable(snes_rom_smoke src/rom_smoke.cpp)
target_link_libraries(snes_rom_smoke PRIVATE snes_core)
