cmake_minimum_required(VERSION 3.16)
project("gameboy_emulator")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set (SOURCES "main.cc" "src/cpu.cc" "src/cartridge.cc" "src/mmu.cc" "src/timer.cc" "src/ppu.cc")
set(CMAKE_BUILD_TYPE Debug)
add_executable(gameboy_emulator ${SOURCES})
