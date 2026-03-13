add_executable(snes_core_tests
  test_main.cpp
)

target_link_libraries(snes_core_tests
  PRIVATE
    snes_core
)

# Increase stack size — debug-mode MSVC allocates stack for all locals in main()
# simultaneously, and the test creates multiple MemoryBus (128 KB WRAM each),
# Smp (64 KB RAM each), plus Ppu objects in separate scopes, exceeding the
# default 1 MB stack.
if(MSVC)
  target_link_options(snes_core_tests PRIVATE /STACK:8388608)
endif()

add_test(NAME snes_core_tests COMMAND snes_core_tests)
