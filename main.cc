#include "src/cartridge.h"
#include "src/cpu.h"

int main(int argc, char *argv[]) {
  Cartridge cartridge;
  CPU cpu(&cartridge.data);
  for(;;) {
    cpu.Run();
  }
  return 0;
}
