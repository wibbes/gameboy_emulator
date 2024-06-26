#include "mmu.h"

void MMU::set(uint16_t address, uint8_t value) {
  memory_[address] = value;
}

uint8_t MMU::get(uint16_t address) {
  return memory_[address];
} 
