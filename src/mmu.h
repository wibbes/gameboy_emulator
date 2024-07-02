#ifndef GAMEBOY_EMULATOR_SRC_MMU_H_
#define GAMEBOY_EMULATOR_SRC_MMU_H_

#include <cstdint>
#include <vector>
#include <iostream>

class MMU {
public:
  std::vector<uint8_t> memory_;
  MMU(std::vector<uint8_t>* cart) : memory_(*cart){};
  ~MMU() = default;

  void WriteMemory(uint16_t address, uint8_t value);
  uint8_t ReadMemory(uint16_t address);
};

#endif
