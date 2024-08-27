#include "src/cartridge.h"
#include "src/cpu.h"
#include <cstdio>
#include <fstream>

int main(int argc, char *argv[]) {
  Cartridge cartridge;
  CPU cpu(&cartridge.data);
  std::cout << "Running CPU..." << std::endl;
  std::ofstream log("debug.txt");
  for (;;) {
    if (log.is_open()) {
      log << std::setfill('0') << std::hex << std::uppercase
          << "A:" << std::setw(2) << +cpu.reg_a_ << " F:" << std::setw(2)
          << +cpu.reg_f_ << " B:" << std::setw(2) << +cpu.reg_b_
          << " C:" << std::setw(2) << +cpu.reg_c_ << " D:" << std::setw(2)
          << +cpu.reg_d_ << " E:" << std::setw(2) << +cpu.reg_e_
          << " H:" << std::setw(2) << +cpu.reg_h_ << " L:" << std::setw(2)
          << +cpu.reg_l_ << " SP:" << std::setw(4) << +cpu.reg_sp_
          << " PC:" << std::setw(4) << +cpu.reg_pc_ << " PCMEM:" << std::setw(2)
          << +cpu.mmu_->memory_[cpu.reg_pc_] << "," << std::setw(2)
          << +cpu.mmu_->memory_[cpu.reg_pc_ + 1] << "," << std::setw(2)
          << +cpu.mmu_->memory_[cpu.reg_pc_ + 2] << "," << std::setw(2)
          << +cpu.mmu_->memory_[cpu.reg_pc_ + 3] << std::endl;
      cpu.Run();
    }
  }
  log.close();
  return 0;
}
