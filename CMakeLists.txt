project("gameboy_emulator")
set (SOURCES "main.cc" "src/cpu.cc")
add_executable(gameboy_emulator ${SOURCES})
