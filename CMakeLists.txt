project("gameboy_emulator")
set (SOURCES "main.cc" "src/cpu.cc" "src/cartridge.cc" "src/mmu.cc")
set(CMAKE_BUILD_TYPE Debug)
add_executable(gameboy_emulator ${SOURCES})
