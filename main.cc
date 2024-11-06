#include "src/cartridge.h"
#include "src/cpu.h"

int main(int argc, char *argv[]) {
  CPU cpu(std::make_shared<Cartridge>());
  std::cout << "Running CPU..." << std::endl;
  for (;;) {
    cpu.Run();
  }
  return 0;
}
