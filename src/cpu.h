#ifndef GAMEBOY_EMULATOR_SRC_CPU_H_
#define GAMEBOY_EMULATOR_SRC_CPU_H_

#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <utility>

#include "instructions.h"
#include "mmu.h"

class Register16 {
public:
  uint8_t *low;
  uint8_t *high;
  Register16(uint8_t *hh, uint8_t *ll) : high(hh), low(ll){};
  ~Register16() = default;
  uint16_t GetRegister();
  void SetRegister(uint16_t value);

private:
};

class CPU {
public:
  uint8_t reg_a_, reg_b_, reg_c_, reg_d_, reg_e_, reg_f_, reg_h_, reg_l_;
  uint16_t reg_sp_, reg_pc_;
  std::unique_ptr<Register16> reg_af_, reg_bc_, reg_de_, reg_hl_;
  std::unique_ptr<MMU> mmu;
  CPU(std::vector<uint8_t> *cartridge)
      : reg_a_(0x10), reg_b_(0x0), reg_c_(0x0), reg_d_(0x0), reg_e_(0x0),
        reg_f_(0x0), reg_h_(0x0), reg_l_(0x0), reg_sp_(0x0000), reg_pc_(0x0100),
        reg_af_(std::make_unique<Register16>(&reg_a_, &reg_f_)),
        reg_bc_(std::make_unique<Register16>(&reg_b_, &reg_c_)),
        reg_de_(std::make_unique<Register16>(&reg_d_, &reg_e_)),
        reg_hl_(std::make_unique<Register16>(&reg_h_, &reg_l_)),
        mmu(std::make_unique<MMU>(cartridge)) {}
  ~CPU() = default;
  void Run();
  void Fetch();
  void Decode(uint8_t opcode);
  void Execute(Instruction instruction);

  void LD(Register16 reg, uint16_t value);

private:
};

#endif
