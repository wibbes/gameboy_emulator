#include "mmu.h"

void MMU::WriteMemory(uint16_t address, uint8_t value) {
  switch (address) {
  case 0xFF01:
    std::cout << std::hex << value;
    break;
  default:
    memory_[address] = value;
    break;
  }
}

uint8_t MMU::ReadMemory(uint16_t address) {
  if (address == 0xFF44)
    return 0x90;
  return memory_[address];
}
