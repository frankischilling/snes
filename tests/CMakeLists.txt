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

# Debug and sanitizer builds can keep the legacy main() fixtures on the stack
# simultaneously. Reserve space for them with both MSVC and Clang on Windows.
if(MINGW)
  target_link_options(snes_core_tests PRIVATE "LINKER:--stack,8388608")
elseif(WIN32)
  target_link_options(snes_core_tests PRIVATE "LINKER:/STACK:8388608")
endif()

add_test(NAME snes_core_tests COMMAND snes_core_tests)

add_executable(snes_ppu_regression_tests test_ppu_regressions.cpp)
target_link_libraries(snes_ppu_regression_tests PRIVATE snes_core)
add_test(NAME snes_ppu_regression_tests COMMAND snes_ppu_regression_tests)

add_executable(snes_hardware_regression_tests test_hardware_regressions.cpp)
target_link_libraries(snes_hardware_regression_tests PRIVATE snes_core)
add_test(NAME snes_hardware_regression_tests COMMAND snes_hardware_regression_tests)

add_executable(snes_cartridge_device_tests test_cartridge_devices.cpp)
target_link_libraries(snes_cartridge_device_tests PRIVATE snes_core)
add_test(NAME snes_cartridge_device_tests COMMAND snes_cartridge_device_tests)

add_executable(snes_sdd1_tests test_sdd1.cpp)
target_link_libraries(snes_sdd1_tests PRIVATE snes_core)
add_test(NAME snes_sdd1_tests COMMAND snes_sdd1_tests)

add_executable(snes_st010_tests test_st010.cpp)
target_link_libraries(snes_st010_tests PRIVATE snes_core)
add_test(NAME snes_st010_tests COMMAND snes_st010_tests)

add_executable(snes_spc7110_tests test_spc7110.cpp)
target_link_libraries(snes_spc7110_tests PRIVATE snes_core)
add_test(NAME snes_spc7110_tests COMMAND snes_spc7110_tests)

add_executable(snes_dma_timing_tests test_dma_timing.cpp)
target_link_libraries(snes_dma_timing_tests PRIVATE snes_core)
add_test(NAME snes_dma_timing_tests COMMAND snes_dma_timing_tests)

add_executable(snes_controller_tests test_controller_ports.cpp)
target_link_libraries(snes_controller_tests PRIVATE snes_core)
add_test(NAME snes_controller_tests COMMAND snes_controller_tests)

add_executable(snes_light_gun_tests test_light_guns.cpp)
target_link_libraries(snes_light_gun_tests PRIVATE snes_core)
add_test(NAME snes_light_gun_tests COMMAND snes_light_gun_tests)

add_executable(snes_sdl_input_tests test_sdl_input.cpp ../frontend/src/SdlInputProvider.cpp)
target_include_directories(snes_sdl_input_tests PRIVATE ../frontend/src)
target_link_libraries(snes_sdl_input_tests PRIVATE snes_core SDL3::SDL3)
add_test(NAME snes_sdl_input_tests COMMAND snes_sdl_input_tests)

add_executable(snes_cpu_regression_tests test_cpu_regressions.cpp)
target_link_libraries(snes_cpu_regression_tests PRIVATE snes_core)
add_test(NAME snes_cpu_regression_tests COMMAND snes_cpu_regression_tests)
add_executable(snes_bus_scheduling_tests test_bus_scheduling.cpp)
target_link_libraries(snes_bus_scheduling_tests PRIVATE snes_core)
add_test(NAME snes_bus_scheduling_tests COMMAND snes_bus_scheduling_tests)
add_executable(snes_cpu_math_tests test_cpu_math.cpp)
target_link_libraries(snes_cpu_math_tests PRIVATE snes_core)
add_test(NAME snes_cpu_math_tests COMMAND snes_cpu_math_tests)
add_executable(snes_smp_wait_tests test_smp_wait.cpp)
target_link_libraries(snes_smp_wait_tests PRIVATE snes_core)
add_test(NAME snes_smp_wait_tests COMMAND snes_smp_wait_tests)
add_executable(snes_sa1_tests test_sa1.cpp)
target_link_libraries(snes_sa1_tests PRIVATE snes_core)
add_test(NAME snes_sa1_tests COMMAND snes_sa1_tests)
add_executable(snes_superfx_tests test_superfx.cpp)
target_link_libraries(snes_superfx_tests PRIVATE snes_core)
add_test(NAME snes_superfx_tests COMMAND snes_superfx_tests)
add_executable(snes_cartridge_processor_tests test_cartridge_processors.cpp)
target_link_libraries(snes_cartridge_processor_tests PRIVATE snes_core)
add_test(NAME snes_cartridge_processor_tests COMMAND snes_cartridge_processor_tests)
add_executable(snes_cx4_tests test_cx4.cpp)
target_link_libraries(snes_cx4_tests PRIVATE snes_core)
add_test(NAME snes_cx4_tests COMMAND snes_cx4_tests)
add_executable(snes_seta_protocol_tests test_seta_protocol.cpp)
target_link_libraries(snes_seta_protocol_tests PRIVATE snes_core)
add_test(NAME snes_seta_protocol_tests COMMAND snes_seta_protocol_tests)
add_executable(snes_bsx_tests test_bsx.cpp)
target_link_libraries(snes_bsx_tests PRIVATE snes_core)
add_test(NAME snes_bsx_tests COMMAND snes_bsx_tests)
add_executable(snes_cartridge_command_tests test_cartridge_commands.cpp)
target_link_libraries(snes_cartridge_command_tests PRIVATE snes_core)
add_test(NAME snes_cartridge_command_tests COMMAND snes_cartridge_command_tests)
add_executable(snes_dsp3_tests test_dsp3.cpp)
target_link_libraries(snes_dsp3_tests PRIVATE snes_core)
add_test(NAME snes_dsp3_tests COMMAND snes_dsp3_tests)
add_executable(snes_dsp4_tests test_dsp4.cpp)
target_link_libraries(snes_dsp4_tests PRIVATE snes_core)
add_test(NAME snes_dsp4_tests COMMAND snes_dsp4_tests)
add_test(NAME snes_cpu_stop_tests COMMAND snes_cpu_regression_tests stop)
set_tests_properties(snes_cpu_stop_tests PROPERTIES TIMEOUT 5)

add_executable(snes_audio_regression_tests test_audio_regressions.cpp)
find_package(Threads REQUIRED)
target_link_libraries(snes_audio_regression_tests PRIVATE snes_core Threads::Threads)
add_test(NAME snes_audio_regression_tests COMMAND snes_audio_regression_tests)
add_executable(snes_dsp_phase_tests test_dsp_phases.cpp)
target_link_libraries(snes_dsp_phase_tests PRIVATE snes_core)
add_test(NAME snes_dsp_phase_tests COMMAND snes_dsp_phase_tests)

add_executable(snes_sdl_audio_tests test_sdl_audio.cpp ../frontend/src/SdlAudioOutput.cpp)

add_executable(snes_save_ram_tests test_save_ram.cpp ../frontend/src/SaveRamFile.cpp)
target_include_directories(snes_save_ram_tests PRIVATE ../frontend/src)
target_link_libraries(snes_save_ram_tests PRIVATE snes_core SDL3::SDL3)
add_test(NAME snes_save_ram_tests COMMAND snes_save_ram_tests)
target_include_directories(snes_sdl_audio_tests PRIVATE ../frontend/src)
target_link_libraries(snes_sdl_audio_tests PRIVATE snes_core SDL3::SDL3)
foreach(rate 32000 44100 48000)
  add_test(NAME snes_sdl_audio_${rate} COMMAND snes_sdl_audio_tests ${rate})
endforeach()

# Opt in with an existing diagnostic checkout; ROMs are never fetched by a build.
set(SNES_DIAGNOSTIC_ROM_DIR "" CACHE PATH "Root of an external SnesTests checkout")
if(SNES_DIAGNOSTIC_ROM_DIR)
  set(dma_irq_rom "${SNES_DIAGNOSTIC_ROM_DIR}/dma_irq_test/dma_irq_test.sfc")
  if(NOT EXISTS "${dma_irq_rom}")
    message(FATAL_ERROR "SNES_DIAGNOSTIC_ROM_DIR does not contain dma_irq_test/dma_irq_test.sfc")
  endif()
  add_test(NAME snes_external_dma_irq
    COMMAND snes_rom_diagnostic "${dma_irq_rom}" 300
      --expect "${CMAKE_CURRENT_SOURCE_DIR}/expected/dma_irq.txt")
  set_tests_properties(snes_external_dma_irq PROPERTIES TIMEOUT 120 LABELS "external")
endif()

set(SNES_PROCESSOR_DIAGNOSTIC_DIR "" CACHE PATH "Root of the external CPU/SPC diagnostic ROM release")
if(SNES_PROCESSOR_DIAGNOSTIC_DIR)
  foreach(processor cpu spc)
    if(processor STREQUAL "cpu")
      set(processor_rom "${SNES_PROCESSOR_DIAGNOSTIC_DIR}/cputest/cputest-full.sfc")
    else()
      set(processor_rom "${SNES_PROCESSOR_DIAGNOSTIC_DIR}/spctest/spctest.sfc")
    endif()
    if(NOT EXISTS "${processor_rom}")
      message(FATAL_ERROR "Missing processor diagnostic ROM: ${processor_rom}")
    endif()
    add_test(NAME snes_external_${processor}
      COMMAND snes_rom_diagnostic "${processor_rom}" 220 0
        --expect "${CMAKE_CURRENT_SOURCE_DIR}/expected/processor.txt")
    set_tests_properties(snes_external_${processor} PROPERTIES TIMEOUT 120 LABELS "external")
  endforeach()
endif()
