#include "src/cartridge.h"
#include "src/cpu.h"
#include <cstdio>

int main(int argc, char *argv[]) {
  Cartridge cartridge;
  CPU cpu(&cartridge.data);
  std::cout << "Running CPU..." << '\n';
  FILE *log = std::fopen("debug.txt", "w");
  for (;;) {
    std::fprintf(
        log,
        "A:%02X F:%02X B:%02X C:%02X D:%02X E:%02X H:%02X L:%02X SP:%04X "
        "PC:%04X PCMEM:%02X,%02X,%02X,%02X\n",
        cpu.reg_a_, cpu.reg_f_, cpu.reg_b_, cpu.reg_c_, cpu.reg_d_, cpu.reg_e_,
        cpu.reg_h_, cpu.reg_l_, cpu.reg_sp_, cpu.reg_pc_,
        cpu.mmu_->memory_[cpu.reg_pc_], cpu.mmu_->memory_[cpu.reg_pc_ + 1],
        cpu.mmu_->memory_[cpu.reg_pc_ + 2], cpu.mmu_->memory_[cpu.reg_pc_ + 3]
    );
    cpu.Run();
  }
  std::fclose(log);
  return 0;
}
