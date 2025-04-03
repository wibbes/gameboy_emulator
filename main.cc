#include "src/cartridge.h"
#include "src/cpu.h"
#include "SDL3/SDL_main.h"

int main(int argc, char *argv[]) {
  
  CPU cpu(std::make_shared<Cartridge>());
  // PPU ppu;
  // ppu.Initialize();
  std::cout << "Running CPU..." << std::endl;
  for (;;) {
    cpu.Run();
    // if (ppu.Run()) break;
  }
  return 0;
}
