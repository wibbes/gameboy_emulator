#ifndef GAMEBOY_EMULATOR_SRC_CPU_H_
#define GAMEBOY_EMULATOR_SRC_CPU_H_

#include <cstdint>
#include <iostream>
#include <utility>
#include <memory>

#include "mmu.h"

class CPU {
public:
  uint8_t reg_a_, reg_b_, reg_c_, reg_d_, reg_e_, reg_f_, reg_h_, reg_l_;
  std::pair<uint8_t *, uint8_t *> reg_af_, reg_bc_, reg_de_, reg_hl_;
  std::unique_ptr<MMU> mmu;
  CPU(std::vector<uint8_t> *cartridge)
      : reg_a_(0x10), reg_b_(0x0), reg_c_(0x0), reg_d_(0x0), reg_e_(0x0),
        reg_f_(0x0), reg_h_(0x0), reg_l_(0x0),
        reg_af_(std::make_pair(&reg_a_, &reg_f_)), 
        reg_bc_(std::make_pair(&reg_b_, &reg_c_)), 
        reg_de_(std::make_pair(&reg_d_, &reg_e_)),
        reg_hl_(std::make_pair(&reg_h_, &reg_l_)), mmu(std::make_unique<MMU>(cartridge)) {}
  ~CPU() = default;

private:
};

#endif
