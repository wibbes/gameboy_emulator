#ifndef GAMEBOY_EMULATOR_SRC_MMU_H_
#define GAMEBOY_EMULATOR_SRC_MMU_H_

#include <cstdint>
#include <vector>

class MMU {
public:
  std::vector<uint8_t> memory_;
  MMU(std::vector<uint8_t> cart) : memory_(cart){};
  ~MMU() = default;
};

#endif
