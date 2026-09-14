add_executable(snes_frontend
  src/main.cpp
  src/SdlVideoOutput.cpp
  src/SdlAudioOutput.cpp
  src/SdlInputProvider.cpp
  src/SaveRamFile.cpp
)

target_link_libraries(snes_frontend
  PRIVATE
    snes_core
    SDL3::SDL3
)
