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
  bool ime, halted;
  CPU(std::vector<uint8_t> *cartridge)
      : flag_z_(0x07), flag_n_(0x06), flag_h_(0x05), flag_c_(0x04),
        rst_jump_vectors({0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38}),
        reg_a_(0x01), reg_b_(0x0), reg_c_(0x13), reg_d_(0x0), reg_e_(0xD8),
        reg_f_(0xB0), reg_h_(0x01), reg_l_(0x4D), reg_sp_(0xFFFE), reg_pc_(0x0100),
        reg_af_(std::make_unique<Register16>(&reg_a_, &reg_f_)),
        reg_bc_(std::make_unique<Register16>(&reg_b_, &reg_c_)),
        reg_de_(std::make_unique<Register16>(&reg_d_, &reg_e_)),
        reg_hl_(std::make_unique<Register16>(&reg_h_, &reg_l_)),
        mmu_(std::make_unique<MMU>(cartridge)),ime(false), halted(false) {}
  ~CPU() = default;

  bool GetFlag(uint8_t flag);
  void SetFlag(uint8_t flag);
  void ClearFlag(uint8_t flag);

  uint8_t GetBit(uint8_t reg, uint8_t bit);
  void SetBit(uint8_t &reg, uint8_t bit);
  void ClearBit(uint8_t &reg, uint8_t bit);

  void Run();
  void Fetch();
  void Decode(uint8_t opcode);
  void Execute(Instruction instruction);
  void ExecuteExtended(Instruction instruction);

  void LD(Register16 reg, uint16_t value);
  void LD(uint8_t *reg, uint8_t value);

  void PUSH(uint16_t value);
  void POP(Register16 reg);

  void RST(uint8_t jmp_vector);
  void JP();
  void JP_HL();
  void JP(uint8_t condition);
  void JR();
  void RET();
  void CALL();

  void BIT(uint8_t reg, uint8_t bit);
  void RES(uint8_t reg, uint8_t bit);
  void SET(uint8_t reg, uint8_t bit);

  void INC(Register16 reg);
  void INC(uint8_t &reg);
  void INC_HL();
  void DEC(Register16 reg);
  void DEC(uint8_t &reg);
  void DEC_HL();

  void AND(uint8_t &reg);
  void OR(uint8_t &reg);
  void XOR(uint8_t &reg);
  void CP(uint8_t &reg);

  void RL(uint8_t &reg);
  void RLC(uint8_t &reg);
  void RR(uint8_t &reg);
  void RRC(uint8_t &reg);
  void SLA(uint8_t &reg);
  void SRA(uint8_t &reg);
  void SRL(uint8_t &reg);
  void SWAP(uint8_t &reg);

  void ADD(uint8_t &reg);
  void ADD_HL(uint16_t value);
  void ADD_SP();
  void ADC(uint8_t &reg);
  void SUB(uint8_t &reg);
  void SBC(uint8_t &reg);

  void DAA();
  void CPL();
  void NOP();
  void CCF();
  void SCF();
  void DI();
  void EI();
  void HALT();
  void STOP();

private:
};

#endif
