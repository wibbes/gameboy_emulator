#ifndef GAMEBOY_EMULATOR_SRC_MMU_H_
#define GAMEBOY_EMULATOR_SRC_MMU_H_

#include <vector>
#include <cstdint>

class MMU {
  std::vector<uint8_t> memory_;
};

#endif
