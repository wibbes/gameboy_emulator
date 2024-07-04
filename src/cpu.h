#ifndef GAMEBOY_EMULATOR_SRC_CPU_H_
#define GAMEBOY_EMULATOR_SRC_CPU_H_

#include <cstdint>
#include <iostream>
#include <memory>

#include "instructions.h"
#include "mmu.h"

class Register16 {
public:
  uint8_t *low_;
  uint8_t *high_;
  Register16(uint8_t *hh, uint8_t *ll) : high_(hh), low_(ll){};
  ~Register16() = default;
  uint16_t GetRegister();
  void SetRegister(uint16_t value);

private:
};

class CPU {
public:
  const uint8_t flag_z_, flag_n_, flag_h_, flag_c_;
  const std::vector<uint8_t> rst_jump_vectors;
  uint8_t reg_a_, reg_b_, reg_c_, reg_d_, reg_e_, reg_f_, reg_h_, reg_l_;
  uint16_t reg_sp_, reg_pc_;
  std::unique_ptr<Register16> reg_af_, reg_bc_, reg_de_, reg_hl_;
  std::unique_ptr<MMU> mmu_;
  CPU(std::vector<uint8_t> *cartridge)
      : flag_z_(0x07), flag_n_(0x06), flag_h_(0x05), flag_c_(0x04),
        rst_jump_vectors({0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38}),
        reg_a_(0x10), reg_b_(0x0), reg_c_(0x0), reg_d_(0x0), reg_e_(0x0),
        reg_f_(0x0), reg_h_(0x0), reg_l_(0x0), reg_sp_(0xFFFE), reg_pc_(0x0100),
        reg_af_(std::make_unique<Register16>(&reg_a_, &reg_f_)),
        reg_bc_(std::make_unique<Register16>(&reg_b_, &reg_c_)),
        reg_de_(std::make_unique<Register16>(&reg_d_, &reg_e_)),
        reg_hl_(std::make_unique<Register16>(&reg_h_, &reg_l_)),
        mmu_(std::make_unique<MMU>(cartridge)) {}
  ~CPU() = default;
  bool GetFlag(uint8_t flag);
  void Run();
  void Fetch();
  void Decode(uint8_t opcode);
  void Execute(Instruction instruction);

  void LD(Register16 reg, uint16_t value);
  void LD(uint8_t *reg, uint8_t value);

  void PUSH(uint16_t value);
  void POP(Register16 reg);

  void RST(uint8_t jmp_vector);
  void JP();
  void JP(uint8_t condition);

private:
};

#endif
