cmake_minimum_required(VERSION 3.21)

project(snes_emulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SNES_ENABLE_TESTS "Build tests" ON)

add_compile_definitions(__PRFCHWINTRIN_H)

# ============================================================================
# SDL3 — fetched from GitHub if not already installed
# ============================================================================
include(FetchContent)

# Try a local/system install first; fall back to FetchContent.
find_package(SDL3 QUIET)
if(NOT SDL3_FOUND)
  message(STATUS "SDL3 not found locally — fetching via FetchContent")
  FetchContent_Declare(
    SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG        release-3.2.8
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
  )
  # Build SDL3 as a static library so the frontend is self-contained
  set(SDL_SHARED OFF CACHE BOOL "" FORCE)
  set(SDL_STATIC ON  CACHE BOOL "" FORCE)
  set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
  set(SDL_DISABLE_INSTALL ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(SDL3)
endif()

add_subdirectory(core)
add_subdirectory(frontend)
add_subdirectory(tools)

if(SNES_ENABLE_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
