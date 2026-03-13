add_executable(snes_trace_dump
  src/trace_dump.cpp
)

target_link_libraries(snes_trace_dump
  PRIVATE
    snes_core
)
