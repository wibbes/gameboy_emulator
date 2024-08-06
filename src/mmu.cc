#include "mmu.h"

void MMU::WriteMemory(uint16_t address, uint8_t value) {
  if (address == 0xFF01) {
    std::cout << std::hex << +value;
  }
  memory_[address] = value;
}

uint8_t MMU::ReadMemory(uint16_t address) { return memory_[address]; }
