project("gameboy_emulator")
set (SOURCES "main.cc" "src/cpu.cc" "src/cartridge.cc" "src/mmu.cc")
add_executable(gameboy_emulator ${SOURCES})
