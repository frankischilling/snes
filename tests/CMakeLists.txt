add_executable(snes_core_tests
  test_main.cpp
)

target_link_libraries(snes_core_tests
  PRIVATE
    snes_core
)

# Keep the assert-based checks active in Release builds too.
if(MSVC)
  target_compile_options(snes_core_tests PRIVATE /UNDEBUG)
else()
  target_compile_options(snes_core_tests PRIVATE -UNDEBUG)
endif()

# Increase stack size — debug-mode MSVC allocates stack for all locals in main()
# simultaneously, and the test creates multiple MemoryBus (128 KB WRAM each),
# Smp (64 KB RAM each), plus Ppu objects in separate scopes, exceeding the
# default 1 MB stack.
if(MSVC)
  target_link_options(snes_core_tests PRIVATE /STACK:8388608)
endif()

add_test(NAME snes_core_tests COMMAND snes_core_tests)

add_executable(snes_ppu_regression_tests test_ppu_regressions.cpp)
target_link_libraries(snes_ppu_regression_tests PRIVATE snes_core)
add_test(NAME snes_ppu_regression_tests COMMAND snes_ppu_regression_tests)

add_executable(snes_hardware_regression_tests test_hardware_regressions.cpp)
target_link_libraries(snes_hardware_regression_tests PRIVATE snes_core)
add_test(NAME snes_hardware_regression_tests COMMAND snes_hardware_regression_tests)

add_executable(snes_audio_regression_tests test_audio_regressions.cpp)
target_link_libraries(snes_audio_regression_tests PRIVATE snes_core)
add_test(NAME snes_audio_regression_tests COMMAND snes_audio_regression_tests)

add_executable(snes_sdl_audio_tests test_sdl_audio.cpp ../frontend/src/SdlAudioOutput.cpp)
target_include_directories(snes_sdl_audio_tests PRIVATE ../frontend/src)
target_link_libraries(snes_sdl_audio_tests PRIVATE snes_core SDL3::SDL3)
foreach(rate 32000 44100 48000)
  add_test(NAME snes_sdl_audio_${rate} COMMAND snes_sdl_audio_tests ${rate})
endforeach()
