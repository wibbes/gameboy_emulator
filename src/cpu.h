#ifndef GAMEBOY_EMULATOR_SRC_CPU_H_
#define GAMEBOY_EMULATOR_SRC_CPU_H_

#include "cartridge.h"
#include "instructions.h"
#include "mmu.h"
#include "timer.h"
#include <cstdint>
#include <iostream>
#include <memory>

class Register16 {
public:
  uint8_t *low_;
  uint8_t *high_;
  explicit Register16(uint8_t *hh, uint8_t *ll) : high_(hh), low_(ll) {};
  ~Register16() = default;
  uint16_t GetRegister();
  void SetRegister(uint16_t value);

private:
};

class CPU {
public:
  explicit CPU(std::shared_ptr<Cartridge> cartridge)
      : reg_a_(0x01), reg_b_(0x0), reg_c_(0x13), reg_d_(0x0), reg_e_(0xD8),
        reg_f_(0xB0), reg_h_(0x01), reg_l_(0x4D), conditional_m_cycles_(0x0),
        reg_sp_(0xFFFE), reg_pc_(0x0100), reg_af_(Register16(&reg_a_, &reg_f_)),
        reg_bc_(Register16(&reg_b_, &reg_c_)),
        reg_de_(Register16(&reg_d_, &reg_e_)),
        reg_hl_(Register16(&reg_h_, &reg_l_)),
        mmu_(std::make_shared<MMU>(cartridge)),
        timer_(std::make_unique<Timer>(mmu_)), ime_(false), halted_(false),
        ime_enable_pending_(false) {}
  ~CPU() = default;

  void Run();

private:
  uint8_t reg_a_, reg_b_, reg_c_, reg_d_, reg_e_, reg_f_, reg_h_, reg_l_,
      conditional_m_cycles_;
  uint16_t reg_sp_, reg_pc_;
  bool ime_, halted_, ime_enable_pending_;

  std::shared_ptr<MMU> mmu_;
  std::unique_ptr<Timer> timer_;
  Register16 reg_af_, reg_bc_, reg_de_, reg_hl_;

  static constexpr uint8_t kFlagZ = 0x07;
  static constexpr uint8_t kFlagN = 0x06;
  static constexpr uint8_t kFlagH = 0x05;
  static constexpr uint8_t kFlagC = 0x04;

  static constexpr std::array<uint8_t, 8> rst_jump_vectors_ = {
      0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38};
  static constexpr std::array<uint8_t, 5> interrupt_vectors_ = {
      0x40, 0x48, 0x50, 0x58, 0x60};

  void Fetch();
  void Decode(uint8_t opcode);
  void Execute(Instruction instruction);
  void ExecuteExtended(Instruction instruction);
  void HandleInterrupt(uint8_t interrupt);

  bool GetFlag(uint8_t flag);
  void SetFlag(uint8_t flag);
  void ClearFlag(uint8_t flag);

  uint8_t GetBit(uint8_t reg, uint8_t bit);
  void SetBit(uint8_t &reg, uint8_t bit);
  void ClearBit(uint8_t &reg, uint8_t bit);

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
  void RES(uint8_t &reg, uint8_t bit);
  void SET(uint8_t &reg, uint8_t bit);

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
};

#endif
