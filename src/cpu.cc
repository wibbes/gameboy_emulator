#include "cpu.h"
#include "instructions.h"

uint16_t Register16::GetRegister() { return *high_ << 8 | (*low_ & 0x00FF); }

void Register16::SetRegister(uint16_t value) {
  *high_ = value >> 8;
  *low_ = static_cast<uint8_t>(value & 0x00FF);
}

const uint16_t MakeWord(uint8_t high, uint8_t low) {
  return static_cast<uint16_t>(high << 8 | low & 0x00FF);
}

bool CPU::GetFlag(uint8_t flag) {
  return static_cast<bool>((reg_f_ >> flag) & 0x01);
}

uint8_t CPU::GetBit(uint8_t reg, uint8_t bit) {
  return (reg & (1 << bit)) >> bit;
}

void CPU::SetBit(uint8_t &reg, uint8_t bit) { reg |= (0x01 << bit); }

void CPU::ClearBit(uint8_t &reg, uint8_t bit) { reg &= ~(0x01 << bit); }

void CPU::SetFlag(uint8_t flag) { *reg_af_.low_ |= (0x01 << flag); }

void CPU::ClearFlag(uint8_t flag) { *reg_af_.low_ &= ~(0x01 << flag); }

void CPU::LD(Register16 reg, uint16_t value) { reg.SetRegister(value); }

void CPU::LD(uint8_t *reg, uint8_t value) { *reg = value; }

void CPU::PUSH(uint16_t value) {
  WriteMMU(reg_sp_ - 1, static_cast<uint8_t>((value >> 8) & 0xFF));
  WriteMMU(reg_sp_ - 2, static_cast<uint8_t>(value & 0xFF));
  reg_sp_ -= 2;
}

void CPU::POP(Register16 reg) {
  reg.SetRegister(MakeWord(ReadMMU(reg_sp_ + 1), ReadMMU(reg_sp_)));
  reg_sp_ += 2;
}

void CPU::RST(uint8_t jmp_vector) {
  PUSH(reg_pc_ + 1);
  reg_pc_ = static_cast<uint16_t>(jmp_vector);
  reg_pc_--;
}

void CPU::JP() {
  /*
   * M1: opcode, M2: lo, M3: hi, M4: set reg_pc_
   * */
  reg_pc_ = MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1));
  TickMCycle();
  reg_pc_ -= 3;
}

void CPU::JP(uint8_t condition) {
  bool flag = condition & 0x10 ? GetFlag(kFlagC) : GetFlag(kFlagZ);
  if (flag & condition) {
    JP();
  }
}

void CPU::JP_HL() { reg_pc_ = reg_hl_.GetRegister() - 1; }

void CPU::JR() { reg_pc_ += static_cast<int8_t>(ReadMMU(reg_pc_ + 1)); }

void CPU::RET() {
  reg_pc_ = MakeWord(ReadMMU(reg_sp_ + 1), ReadMMU(reg_sp_)) - 1;
  reg_sp_ += 2;
}

void CPU::CALL() {
  PUSH(reg_pc_ + 3);
  reg_pc_ = MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1)) - 3;
}

void CPU::BIT(uint8_t reg, uint8_t bit) {
  GetBit(reg, bit) == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  SetFlag(kFlagH);
}

void CPU::SET(uint8_t &reg, uint8_t bit) { reg |= 1UL << bit; }

void CPU::RES(uint8_t &reg, uint8_t bit) { reg &= ~(1UL << bit); }

void CPU::INC(Register16 reg) {
  // Does not change registers
  reg.SetRegister(reg.GetRegister() + 1);
}

void CPU::INC(uint8_t &reg) {
  ++reg;
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  (reg & 0x0F) == 0 ? SetFlag(kFlagH) : ClearFlag(kFlagH);
}

void CPU::INC_HL() {
  uint8_t value = ReadMMU(reg_hl_.GetRegister()) + 1;
  value == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  (value & 0x0F) == 0 ? SetFlag(kFlagH) : ClearFlag(kFlagH);
  WriteMMU(reg_hl_.GetRegister(), value);
}

void CPU::DEC(Register16 reg) {
  // Does not change registers
  reg.SetRegister(reg.GetRegister() - 1);
}

void CPU::DEC(uint8_t &reg) {
  --reg;
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  SetFlag(kFlagN);
  (reg & 0x0F) == 0x0F ? SetFlag(kFlagH) : ClearFlag(kFlagH);
}

void CPU::DEC_HL() {
  uint8_t new_value = ReadMMU(reg_hl_.GetRegister()) - 1;
  WriteMMU(reg_hl_.GetRegister(), new_value);
  new_value == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  SetFlag(kFlagN);
  (new_value & 0x0F) == 0x0F ? SetFlag(kFlagH) : ClearFlag(kFlagH);
}

void CPU::AND(uint8_t &reg) {
  reg_a_ &= reg;
  reg_a_ == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  SetFlag(kFlagH);
  ClearFlag(kFlagC);
}

void CPU::OR(uint8_t &reg) {
  reg_a_ |= reg;
  reg_a_ == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagH);
  ClearFlag(kFlagN);
  ClearFlag(kFlagC);
}

void CPU::XOR(uint8_t &reg) {
  reg_a_ ^= reg;
  reg_a_ == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagH);
  ClearFlag(kFlagN);
  ClearFlag(kFlagC);
}

void CPU::CP(uint8_t &reg) {
  reg_a_ == reg ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  SetFlag(kFlagN);
  ((reg_a_ - reg) & 0x0F) > (reg_a_ & 0x0F) ? SetFlag(kFlagH)
                                            : ClearFlag(kFlagH);
  reg_a_ < reg ? SetFlag(kFlagC) : ClearFlag(kFlagC);
}

void CPU::RL(uint8_t &reg) {
  uint8_t carry = GetFlag(kFlagC);
  (reg & 0x80) != 0 ? SetFlag(kFlagC) : ClearFlag(kFlagC);
  reg <<= 1;
  reg |= carry;
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
}

void CPU::RR(uint8_t &reg) {
  uint8_t carry = GetFlag(kFlagC);
  (reg & 0x01) != 0 ? SetFlag(kFlagC) : ClearFlag(kFlagC);
  reg >>= 1;
  carry ? reg |= 0x80 : reg |= 0;
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
}

void CPU::RLC(uint8_t &reg) {
  uint8_t msb = (reg & 0x80) >> 7;
  reg = (reg << 1) | msb;
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
  msb ? SetFlag(kFlagC) : ClearFlag(kFlagC);
}

void CPU::RRC(uint8_t &reg) {
  uint8_t lsb = reg & 0x01;
  reg = (reg >> 1) | (lsb << 7);
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
  lsb ? SetFlag(kFlagC) : ClearFlag(kFlagC);
}

void CPU::SLA(uint8_t &reg) {
  (reg & 0x80) != 0 ? SetFlag(kFlagC) : ClearFlag(kFlagC);
  reg <<= 1;
  ClearBit(reg, 0);
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
}

void CPU::SRA(uint8_t &reg) {
  (reg & 0x01) != 0 ? SetFlag(kFlagC) : ClearFlag(kFlagC);
  if ((reg & 0x80) != 0) {
    reg >>= 1;
    reg |= 0x80;
  } else {
    reg >>= 1;
  }
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
}

void CPU::SRL(uint8_t &reg) {
  (reg & 0x01) != 0 ? SetFlag(kFlagC) : ClearFlag(kFlagC);
  reg >>= 1;
  ClearBit(reg, 7);
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
}

void CPU::SWAP(uint8_t &reg) {
  reg = ((reg & 0x0F) << 4) | ((reg & 0xF0) >> 4);
  reg == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
  ClearFlag(kFlagC);
}

void CPU::ADD(uint8_t &reg) {
  int eval = reg_a_ + reg;
  int16_t test_carries = static_cast<int16_t>(reg_a_ ^ reg ^ eval);
  reg_a_ = eval;
  reg_a_ == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ((test_carries & 0x10) != 0) ? SetFlag(kFlagH) : ClearFlag(kFlagH);
  ((test_carries & 0x100) != 0) ? SetFlag(kFlagC) : ClearFlag(kFlagC);
}

void CPU::ADD_HL(uint16_t value) {
  int eval = reg_hl_.GetRegister() + value;
  int test_carries = reg_hl_.GetRegister() ^ value ^ eval;
  ClearFlag(kFlagN);
  ((test_carries & 0x1000) != 0) ? SetFlag(kFlagH) : ClearFlag(kFlagH);
  ((test_carries & 0x10000) != 0) ? SetFlag(kFlagC) : ClearFlag(kFlagC);
  reg_hl_.SetRegister(eval);
}

void CPU::ADD_SP() {
  int immediate = static_cast<char>(ReadMMU(reg_pc_ + 1));
  int eval = reg_sp_ + immediate;
  int test_carries = reg_sp_ ^ immediate ^ eval;
  reg_sp_ = eval;
  ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  (test_carries & 0x10) != 0 ? SetFlag(kFlagH) : ClearFlag(kFlagH);
  (test_carries & 0x100) != 0 ? SetFlag(kFlagC) : ClearFlag(kFlagC);
}

void CPU::ADC(uint8_t &reg) {
  uint8_t carry = GetFlag(kFlagC);
  int eval = reg_a_ + (reg + carry);
  int test_carries = reg_a_ ^ reg ^ eval;
  reg_a_ = eval;
  reg_a_ == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  ClearFlag(kFlagN);
  ((test_carries & 0x10) != 0) ? SetFlag(kFlagH) : ClearFlag(kFlagH);
  ((test_carries & 0x100) != 0) ? SetFlag(kFlagC) : ClearFlag(kFlagC);
}

void CPU::SUB(uint8_t &reg) {
  int eval = reg_a_ - reg;
  int16_t test_carries = static_cast<int16_t>(reg_a_ ^ reg ^ eval);
  reg_a_ = eval;
  reg_a_ == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  SetFlag(kFlagN);
  ((test_carries & 0x10) != 0) ? SetFlag(kFlagH) : ClearFlag(kFlagH);
  ((test_carries & 0x100) != 0) ? SetFlag(kFlagC) : ClearFlag(kFlagC);
}

void CPU::SBC(uint8_t &reg) {
  uint8_t carry = GetFlag(kFlagC) ? 1 : 0;
  int eval = reg_a_ - (reg + carry);
  int16_t test_carries = static_cast<int16_t>(reg_a_ ^ reg ^ eval);
  reg_a_ = eval;
  reg_a_ == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
  SetFlag(kFlagN);
  ((test_carries & 0x10) != 0) ? SetFlag(kFlagH) : ClearFlag(kFlagH);
  ((test_carries & 0x100) != 0) ? SetFlag(kFlagC) : ClearFlag(kFlagC);
}

void CPU::DAA() {
  if (!GetFlag(kFlagN)) {
    if (GetFlag(kFlagC) || reg_a_ > 0x99) {
      reg_a_ += 0x60;
      SetFlag(kFlagC);
    }
    if (GetFlag(kFlagH) || (reg_a_ & 0x0F) > 0x09) {
      reg_a_ += 0x06;
    }
  } else {
    if (GetFlag(kFlagC))
      reg_a_ -= 0x60;
    if (GetFlag(kFlagH))
      reg_a_ -= 0x06;
  }
  ClearFlag(kFlagH);
  reg_a_ == 0 ? SetFlag(kFlagZ) : ClearFlag(kFlagZ);
}

void CPU::CPL() {
  reg_a_ = ~reg_a_;
  SetFlag(kFlagN);
  SetFlag(kFlagH);
}

void CPU::NOP() { return; }

void CPU::CCF() {
  reg_f_ ^= (0x01 << kFlagC);
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
}

void CPU::SCF() {
  ClearFlag(kFlagN);
  ClearFlag(kFlagH);
  SetFlag(kFlagC);
}

void CPU::DI() { ime_ = false; }

void CPU::EI() { ime_ = true; }

void CPU::HALT() {
  uint8_t pending = mmu_->ie_->GetState() & mmu_->if_->GetState();

  if (!ime_ && pending) {
    halt_bug_ = true; // HALT bug
  } else {
    halted_ = true; // Normal HALT
  }
  // halted_ = true;
}

void CPU::STOP() {
  // Before calling this...
  //  - reset interrupt enable
  //  - reset I/O
}

void CPU::CheckInterrupts() {
  if (ime_ &&
      (mmu_->ie_->GetState() & mmu_->if_->GetState())) { // Interrupts pending
    ime_ = false;
    // Loop through each interrupt
    for (auto i = 0ull; i < 5; ++i) {
      // If pending interrupt is found, handle it
      if (mmu_->ie_->GetInterrupt(i) && mmu_->if_->GetInterrupt(i)) {
        HandleInterrupt(i);
        halted_ = false;
      }
    }
  }
}

void CPU::WriteMMU(uint16_t address, uint8_t value) {
  TickMCycle();
  mmu_->WriteMemory(address, value);
}

uint8_t CPU::ReadMMU(uint16_t address) {
  TickMCycle();
  return mmu_->ReadMemory(address);
}

void CPU::TickMCycle() {
  Tick();
  Tick();
  Tick();
  Tick();
  ++cycles_elapsed_;
}

bool CPU::CheckHalt() {
  bool interrupt_pending = mmu_->ie_->GetState() & mmu_->if_->GetState();
  if (interrupt_pending) {
    halted_ = false;
    return false;
  }
  if (halted_) {
    TickMCycle();
    return true;
  } else {
    return false;
  }
}

void CPU::CheckEI(uint8_t opcode) {
  if (opcode != 0xFB && ime_enable_pending_) {
    EI();
    ime_enable_pending_ = false;
  }
}

bool CPU::CheckCycles(uint8_t opcode, uint8_t cycles_for_instruction) {
  if (cycles_for_instruction == cycles_elapsed_) {
    return true;
    cycles_elapsed_ = 0;
    return true;
  } else {
    std::cout << "WRONG CYCLES FOR 0x" << std::hex << +opcode << " "
              << instructions.at(opcode).mnemonic_ << "\nEXPECTED "
              << +cycles_for_instruction << ", RAN " << +cycles_elapsed_
              << '\n';
    cycles_elapsed_ = 0;
    return false;
  }
  return true;
}

void CPU::Run() { Step(); }

void CPU::Step() {
  cycles_elapsed_ = 0;
  CheckInterrupts();
  if (CheckHalt())
    return;
  uint8_t opcode = mmu_->ReadMemory(reg_pc_);
  TickMCycle();
  Execute(instructions.at(opcode));
  CheckEI(opcode);
  if (!CheckCycles(opcode, instructions.at(opcode).cycles_))
    exit(1);
}

void CPU::Tick() { timer_->Tick(); }

void CPU::HandleInterrupt(uint8_t interrupt) {
  mmu_->if_->ResetInterrupt(interrupt);
  PUSH(reg_pc_);
  reg_pc_ = interrupt_vectors_[interrupt];
  // Run the ISR until RETI is found
  while (true) {
    uint8_t opcode = ReadMMU(reg_pc_);
    Execute(instructions.at(opcode));
    // PC increments within Execute, so this checks next PC value for
    // RET or RETI opcodes then runs it.
    opcode = ReadMMU(reg_pc_);
    if (opcode == 0xC9 || opcode == 0xD9) {
      Execute(instructions.at(opcode));
      break;
    }
  }
  // TickMCycle();
  // TickMCycle();
  // TickMCycle();
}

void CPU::Execute(Instruction instruction) {
  switch (instruction.opcode_) {
  case 0x00:
    NOP();
    break;
  case 0x01:
    LD(reg_bc_, MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1)));
    break;
  case 0x02: // LD (BC), A
    WriteMMU(reg_bc_.GetRegister(), reg_a_);
    break;
  case 0x03:
    INC(reg_bc_);
    break;
  case 0x04:
    INC(reg_b_);
    break;
  case 0x05:
    DEC(reg_b_);
    break;
  case 0x06: // LD B, d8
    LD(&reg_b_, ReadMMU(reg_pc_ + 1));
    break;
  case 0x07:
    RLC(reg_a_);
    ClearFlag(kFlagZ);
    break;
  case 0x08: // LD (a16), SP
    WriteMMU(MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1)),
             reg_sp_ & 0xFF);
    WriteMMU(MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1)) + 1,
             (reg_sp_ & 0xFF00) >> 8);
    break;
  case 0x09:
    ADD_HL(reg_bc_.GetRegister());
    break;
  case 0x0A:
    LD(&reg_a_, ReadMMU(reg_bc_.GetRegister()));
    break;
  case 0x0B:
    DEC(reg_bc_);
    break;
  case 0x0C:
    INC(reg_c_);
    break;
  case 0x0D:
    DEC(reg_c_);
    break;
  case 0x0E:
    LD(&reg_c_, ReadMMU(reg_pc_ + 1));
    break;
  case 0x0F:
    RRC(reg_a_);
    ClearFlag(kFlagZ);
    break;
  case 0x10:
    STOP();
    break;
  case 0x11:
    LD(reg_de_, MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1)));
    break;
  case 0x12:
    WriteMMU(reg_de_.GetRegister(), reg_a_);
    break;
  case 0x13:
    INC(reg_de_);
    break;
  case 0x14:
    INC(reg_d_);
    break;
  case 0x15:
    DEC(reg_d_);
    break;
  case 0x16:
    LD(&reg_d_, ReadMMU(reg_pc_ + 1));
    break;
  case 0x17:
    RL(reg_a_);
    ClearFlag(kFlagZ);
    break;
  case 0x18:
    JR();
    break;
  case 0x19:
    ADD_HL(reg_de_.GetRegister());
    break;
  case 0x1A:
    LD(&reg_a_, ReadMMU(reg_de_.GetRegister()));
    break;
  case 0x1B:
    DEC(reg_de_);
    break;
  case 0x1C:
    INC(reg_e_);
    break;
  case 0x1D:
    DEC(reg_e_);
    break;
  case 0x1E:
    LD(&reg_e_, ReadMMU(reg_pc_ + 1));
    break;
  case 0x1F:
    RR(reg_a_);
    // RR, A and RRA are two different opcodes. 0x1F resets Z.
    ClearFlag(kFlagZ);
    break;
  case 0x20:
    if (!GetFlag(kFlagZ)) {
      conditional_m_cycles_ = 1;
      JR();
    } else {
      TickMCycle();
    }
    break;
  case 0x21:
    LD(reg_hl_, MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1)));
    break;
  case 0x22:
    WriteMMU(reg_hl_.GetRegister(), reg_a_);
    reg_hl_.SetRegister(reg_hl_.GetRegister() + 1);
    break;
  case 0x23:
    INC(reg_hl_);
    break;
  case 0x24:
    INC(reg_h_);
    break;
  case 0x25:
    DEC(reg_h_);
    break;
  case 0x26:
    LD(&reg_h_, ReadMMU(reg_pc_ + 1));
    break;
  case 0x27:
    DAA();
    break;
  case 0x28:
    if (GetFlag(kFlagZ)) {
      conditional_m_cycles_ = 1;
      JR();
    }
    break;
  case 0x29:
    ADD_HL(reg_hl_.GetRegister());
    break;
  case 0x2A:
    LD(&reg_a_, ReadMMU(reg_hl_.GetRegister()));
    reg_hl_.SetRegister(reg_hl_.GetRegister() + 1);
    break;
  case 0x2B:
    DEC(reg_hl_);
    break;
  case 0x2C:
    INC(reg_l_);
    break;
  case 0x2D:
    DEC(reg_l_);
    break;
  case 0x2E:
    LD(&reg_l_, ReadMMU(reg_pc_ + 1));
    break;
  case 0x2F:
    CPL();
    break;
  case 0x30:
    if (!GetFlag(kFlagC)) {
      conditional_m_cycles_ = 1;
      JR();
    }
    break;
  case 0x31:
    reg_sp_ = MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1));
    break;
  case 0x32:
    WriteMMU(reg_hl_.GetRegister(), reg_a_);
    reg_hl_.SetRegister(reg_hl_.GetRegister() - 1);
    break;
  case 0x33:
    ++reg_sp_;
    break;
  case 0x34:
    INC_HL();
    break;
  case 0x35:
    DEC_HL();
    break;
  case 0x36:
    WriteMMU(reg_hl_.GetRegister(), ReadMMU(reg_pc_ + 1));
    break;
  case 0x37:
    SCF();
    break;
  case 0x38:
    if (GetFlag(kFlagC)) {
      conditional_m_cycles_ = 1;
      JR();
    }
    break;
  case 0x39:
    ADD_HL(reg_sp_);
    break;
  case 0x3A:
    LD(&reg_a_, ReadMMU(reg_hl_.GetRegister()));
    reg_hl_.SetRegister(reg_hl_.GetRegister() - 1);
    break;
  case 0x3B:
    --reg_sp_;
    break;
  case 0x3C:
    INC(reg_a_);
    break;
  case 0x3D:
    DEC(reg_a_);
    break;
  case 0x3E:
    LD(&reg_a_, ReadMMU(reg_pc_ + 1));
    break;
  case 0x3F:
    CCF();
    break;
  case 0x40: // LD B, B
    LD(&reg_b_, reg_b_);
    break;
  case 0x41: // LD B, C
    LD(&reg_b_, reg_c_);
    break;
  case 0x42: // LD B, D
    LD(&reg_b_, reg_d_);
    break;
  case 0x43: // LD B, E
    LD(&reg_b_, reg_e_);
    break;
  case 0x44: // LD B, H
    LD(&reg_b_, reg_h_);
    break;
  case 0x45: // LD B, L
    LD(&reg_b_, reg_l_);
    break;
  case 0x46: // LD B, (HL)
    LD(&reg_b_, ReadMMU(reg_hl_.GetRegister()));
    break;
  case 0x47: // LD B, A
    LD(&reg_b_, reg_a_);
    break;
  case 0x48: // LD C, B
    LD(&reg_c_, reg_b_);
    break;
  case 0x49: // LD C, C
    LD(&reg_c_, reg_c_);
    break;
  case 0x4A: // LD C, D
    LD(&reg_c_, reg_d_);
    break;
  case 0x4B: // LD C, E
    LD(&reg_c_, reg_e_);
    break;
  case 0x4C: // LD C, H
    LD(&reg_c_, reg_h_);
    break;
  case 0x4D: // LD C, L
    LD(&reg_c_, reg_l_);
    break;
  case 0x4E: // LD C, (HL)
    LD(&reg_c_, ReadMMU(reg_hl_.GetRegister()));
    break;
  case 0x4F: // LD C, A
    LD(&reg_c_, reg_a_);
    break;
  case 0x50: // LD D, B
    LD(&reg_d_, reg_b_);
    break;
  case 0x51: // LD D, C
    LD(&reg_d_, reg_c_);
    break;
  case 0x52: // LD D, D
    LD(&reg_d_, reg_d_);
    break;
  case 0x53: // LD D, E
    LD(&reg_d_, reg_e_);
    break;
  case 0x54: // LD D, H
    LD(&reg_d_, reg_h_);
    break;
  case 0x55: // LD D, L
    LD(&reg_d_, reg_l_);
    break;
  case 0x56: // LD D, (HL)
    LD(&reg_d_, ReadMMU(reg_hl_.GetRegister()));
    break;
  case 0x57: // LD D, A
    LD(&reg_d_, reg_a_);
    break;
  case 0x58: // LD E, B
    LD(&reg_e_, reg_b_);
    break;
  case 0x59: // LD E, C
    LD(&reg_e_, reg_c_);
    break;
  case 0x5A: // LD E, D
    LD(&reg_e_, reg_d_);
    break;
  case 0x5B: // LD E, E
    LD(&reg_e_, reg_e_);
    break;
  case 0x5C: // LD E, H
    LD(&reg_e_, reg_h_);
    break;
  case 0x5D: // LD E, L
    LD(&reg_e_, reg_l_);
    break;
  case 0x5E: // LD E, (HL)
    LD(&reg_e_, ReadMMU(reg_hl_.GetRegister()));
    break;
  case 0x5F: // LD E, A
    LD(&reg_e_, reg_a_);
    break;
  case 0x60: // LD H, B
    LD(&reg_h_, reg_b_);
    break;
  case 0x61: // LD H, C
    LD(&reg_h_, reg_c_);
    break;
  case 0x62: // LD H, D
    LD(&reg_h_, reg_d_);
    break;
  case 0x63: // LD H, E
    LD(&reg_h_, reg_e_);
    break;
  case 0x64: // LD H, H
    LD(&reg_h_, reg_h_);
    break;
  case 0x65: // LD H, L
    LD(&reg_h_, reg_l_);
    break;
  case 0x66: // LD H, (HL)
    LD(&reg_h_, ReadMMU(reg_hl_.GetRegister()));
    break;
  case 0x67: // LD H, A
    LD(&reg_h_, reg_a_);
    break;
  case 0x68: // LD L, B
    LD(&reg_l_, reg_b_);
    break;
  case 0x69: // LD L, C
    LD(&reg_l_, reg_c_);
    break;
  case 0x6A: // LD L, D
    LD(&reg_l_, reg_d_);
    break;
  case 0x6B: // LD L, E
    LD(&reg_l_, reg_e_);
    break;
  case 0x6C: // LD L, H
    LD(&reg_l_, reg_h_);
    break;
  case 0x6D: // LD L, L
    LD(&reg_l_, reg_l_);
    break;
  case 0x6E: // LD L, (HL)
    LD(&reg_l_, ReadMMU(reg_hl_.GetRegister()));
    break;
  case 0x6F: // LD L, A
    LD(&reg_l_, reg_a_);
    break;
  case 0x70: // LD (HL), B
    WriteMMU(reg_hl_.GetRegister(), reg_b_);
    break;
  case 0x71: // LD (HL), C
    WriteMMU(reg_hl_.GetRegister(), reg_c_);
    break;
  case 0x72: // LD (HL), D
    WriteMMU(reg_hl_.GetRegister(), reg_d_);
    break;
  case 0x73: // LD (HL), E
    WriteMMU(reg_hl_.GetRegister(), reg_e_);
    break;
  case 0x74: // LD (HL), H
    WriteMMU(reg_hl_.GetRegister(), reg_h_);
    break;
  case 0x75: // LD (HL), L
    WriteMMU(reg_hl_.GetRegister(), reg_l_);
    break;
  case 0x76:
    HALT();
    break;
  case 0x77: // LD (HL), A
    WriteMMU(reg_hl_.GetRegister(), reg_a_);
    break;
  case 0x78: // LD A, B
    LD(&reg_a_, reg_b_);
    break;
  case 0x79: // LD A, C
    LD(&reg_a_, reg_c_);
    break;
  case 0x7A: // LD A, D
    LD(&reg_a_, reg_d_);
    break;
  case 0x7B: // LD A, E
    LD(&reg_a_, reg_e_);
    break;
  case 0x7C: // LD A, H
    LD(&reg_a_, reg_h_);
    break;
  case 0x7D: // LD A, L
    LD(&reg_a_, reg_l_);
    break;
  case 0x7E: // LD A, (HL)
    LD(&reg_a_, ReadMMU(reg_hl_.GetRegister()));
    break;
  case 0x7F: // LD A, A
    LD(&reg_a_, reg_a_);
    break;
  case 0x80:
    ADD(reg_b_);
    break;
  case 0x81:
    ADD(reg_c_);
    break;
  case 0x82:
    ADD(reg_d_);
    break;
  case 0x83:
    ADD(reg_e_);
    break;
  case 0x84:
    ADD(reg_h_);
    break;
  case 0x85:
    ADD(reg_l_);
    break;
  case 0x86: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    ADD(byte);
    break;
  }
  case 0x87:
    ADD(reg_a_);
    break;
  case 0x88:
    ADC(reg_b_);
    break;
  case 0x89:
    ADC(reg_c_);
    break;
  case 0x8A:
    ADC(reg_d_);
    break;
  case 0x8B:
    ADC(reg_e_);
    break;
  case 0x8C:
    ADC(reg_h_);
    break;
  case 0x8D:
    ADC(reg_l_);
    break;
  case 0x8E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    ADC(byte);
    break;
  }
  case 0x8F:
    ADC(reg_a_);
    break;
  case 0x90:
    SUB(reg_b_);
    break;
  case 0x91:
    SUB(reg_c_);
    break;
  case 0x92:
    SUB(reg_d_);
    break;
  case 0x93:
    SUB(reg_e_);
    break;
  case 0x94:
    SUB(reg_h_);
    break;
  case 0x95:
    SUB(reg_l_);
    break;
  case 0x96: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SUB(byte);
    break;
  }
  case 0x97:
    SUB(reg_a_);
    break;
  case 0x98:
    SBC(reg_b_);
    break;
  case 0x99:
    SBC(reg_c_);
    break;
  case 0x9A:
    SBC(reg_d_);
    break;
  case 0x9B:
    SBC(reg_e_);
    break;
  case 0x9C:
    SBC(reg_h_);
    break;
  case 0x9D:
    SBC(reg_l_);
    break;
  case 0x9E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SBC(byte);
    break;
  }
  case 0x9F:
    SBC(reg_a_);
    break;
  case 0xA0:
    AND(reg_b_);
    break;
  case 0xA1:
    AND(reg_c_);
    break;
  case 0xA2:
    AND(reg_d_);
    break;
  case 0xA3:
    AND(reg_e_);
    break;
  case 0xA4:
    AND(reg_h_);
    break;
  case 0xA5:
    AND(reg_l_);
    break;
  case 0xA6: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    AND(byte);
    break;
  }
  case 0xA7:
    AND(reg_a_);
    break;
  case 0xA8:
    XOR(reg_b_);
    break;
  case 0xA9:
    XOR(reg_c_);
    break;
  case 0xAA:
    XOR(reg_d_);
    break;
  case 0xAB:
    XOR(reg_e_);
    break;
  case 0xAC:
    XOR(reg_h_);
    break;
  case 0xAD:
    XOR(reg_l_);
    break;
  case 0xAE: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    XOR(byte);
    break;
  }
  case 0xAF:
    XOR(reg_a_);
    break;
  case 0xB0:
    OR(reg_b_);
    break;
  case 0xB1:
    OR(reg_c_);
    break;
  case 0xB2:
    OR(reg_d_);
    break;
  case 0xB3:
    OR(reg_e_);
    break;
  case 0xB4:
    OR(reg_h_);
    break;
  case 0xB5:
    OR(reg_l_);
    break;
  case 0xB6: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    OR(byte);
    break;
  }
  case 0xB7:
    OR(reg_a_);
    break;
  case 0xB8:
    CP(reg_b_);
    break;
  case 0xB9:
    CP(reg_c_);
    break;
  case 0xBA:
    CP(reg_d_);
    break;
  case 0xBB:
    CP(reg_e_);
    break;
  case 0xBC:
    CP(reg_h_);
    break;
  case 0xBD:
    CP(reg_l_);
    break;
  case 0xBE: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    CP(byte);
    break;
  }
  case 0xBF:
    CP(reg_a_);
    break;
  case 0xC0:
    if (!GetFlag(kFlagZ)) {
      conditional_m_cycles_ = 3;
      RET();
    }
    break;
  case 0xC1:
    POP(reg_bc_);
    break;
  case 0xC2:
    if (!GetFlag(kFlagZ)) {
      conditional_m_cycles_ = 1;
      JP();
    }
    break;
  case 0xC3:
    JP();
    break;
  case 0xC4:
    if (!GetFlag(kFlagZ)) {
      conditional_m_cycles_ = 3;
      CALL();
    }
    break;
  case 0xC5:
    PUSH(reg_bc_.GetRegister());
    break;
  case 0xC6: {
    uint8_t byte = ReadMMU(reg_pc_ + 1);
    ADD(byte);
    break;
  }
  case 0xC7:
    RST(rst_jump_vectors_[0]);
    break;
  case 0xC8:
    if (GetFlag(kFlagZ)) {
      RET();
    }
    break;
  case 0xC9:
    RET();
    break;
  case 0xCA:
    if (GetFlag(kFlagZ)) {
      conditional_m_cycles_ = 1;
      JP();
    }
    break;
  case 0xCB:
    ExecuteExtended(extended_instructions.at(ReadMMU(reg_pc_ + 1)));
    break;
  case 0xCC:
    if (GetFlag(kFlagZ)) {
      conditional_m_cycles_ = 3;
      CALL();
    }
    break;
  case 0xCD:
    CALL();
    break;
  case 0xCE: {
    uint8_t byte = ReadMMU(reg_pc_ + 1);
    ADC(byte);
    break;
  }
  case 0xCF:
    RST(rst_jump_vectors_[1]);
    break;
  case 0xD0:
    if (!GetFlag(kFlagC)) {
      conditional_m_cycles_ = 3;
      RET();
    }
    break;
  case 0xD1:
    POP(reg_de_);
    break;
  case 0xD2:
    if (!GetFlag(kFlagC)) {
      conditional_m_cycles_ = 1;
      JP();
    }
    break;
  case 0xD3:
    break;
  case 0xD4:
    if (!GetFlag(kFlagC)) {
      conditional_m_cycles_ = 3;
      CALL();
    }
    break;
  case 0xD5:
    PUSH(reg_de_.GetRegister());
    break;
  case 0xD6: {
    uint8_t byte = ReadMMU(reg_pc_ + 1);
    SUB(byte);
    break;
  }
  case 0xD7:
    RST(rst_jump_vectors_[2]);
    break;
  case 0xD8:
    if (GetFlag(kFlagC)) {
      conditional_m_cycles_ = 3;
      RET();
    }
    break;
  case 0xD9:
    RET();
    ime_ = true;
    break;
  case 0xDA:
    if (GetFlag(kFlagC)) {
      conditional_m_cycles_ = 1;
      JP();
    }
    break;
  case 0xDB:
    break;
  case 0xDC:
    if (GetFlag(kFlagC)) {
      conditional_m_cycles_ = 3;
      CALL();
    }
    break;
  case 0xDD:
    break;
  case 0xDE: {
    uint8_t byte = ReadMMU(reg_pc_ + 1);
    SBC(byte);
    break;
  } break;
  case 0xDF:
    RST(rst_jump_vectors_[3]);
    break;
  case 0xE0: {
    uint16_t word = static_cast<uint16_t>(0xFF00 + ReadMMU(reg_pc_ + 1));
    WriteMMU(word, reg_a_);
    break;
  }
  case 0xE1:
    POP(reg_hl_);
    break;
  case 0xE2:
    // MSB == 0xFF, so the possible range is 0xFF00 - 0xFFFF
    WriteMMU(MakeWord(0xFF, reg_c_), reg_a_);
    break;
  case 0xE3:
    break;
  case 0xE4:
    break;
  case 0xE5:
    PUSH(reg_hl_.GetRegister());
    break;
  case 0xE6: {
    uint8_t byte = ReadMMU(reg_pc_ + 1);
    AND(byte);
    break;
  }
  case 0xE7:
    RST(rst_jump_vectors_[4]);
    break;
  case 0xE8:
    ADD_SP();
    break;
  case 0xE9:
    JP_HL();
    break;
  case 0xEA: {
    uint16_t word = MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1));
    WriteMMU(word, reg_a_);
    break;
  }
  case 0xEB:
    break;
  case 0xEC:
    break;
  case 0xED:
    break;
  case 0xEE: {
    uint8_t byte = ReadMMU(reg_pc_ + 1);
    XOR(byte);
    break;
  }
  case 0xEF:
    RST(rst_jump_vectors_[5]);
    break;
  case 0xF0:
    if (ReadMMU(reg_pc_ + 1) == 0x44) {
      reg_a_ = 0x90;
    } else {
      uint16_t word = static_cast<uint16_t>(0xFF00 + ReadMMU(reg_pc_ + 1));
      LD(&reg_a_, ReadMMU(word));
    }
    break;
  case 0xF1:
    POP(reg_af_);
    *reg_af_.low_ &= 0xF0; // don't modify flags register
    break;
  case 0xF2:
    LD(&reg_a_, ReadMMU(MakeWord(0xFF, reg_c_)));
    break;
  case 0xF3:
    DI();
    break;
  case 0xF4:
    break;
  case 0xF5:
    PUSH(reg_af_.GetRegister());
    break;
  case 0xF6: {
    uint8_t byte = ReadMMU(reg_pc_ + 1);
    OR(byte);
    break;
  }
  case 0xF7:
    RST(rst_jump_vectors_[6]);
    break;
  case 0xF8: {
    int immediate = static_cast<char>(ReadMMU(reg_pc_ + 1));
    int eval = reg_sp_ + immediate;
    int test_carries = reg_sp_ ^ immediate ^ eval;
    reg_hl_.SetRegister(immediate + reg_sp_);
    ClearFlag(kFlagZ);
    ClearFlag(kFlagN);
    ((test_carries & 0x10) != 0) ? SetFlag(kFlagH) : ClearFlag(kFlagH);
    ((test_carries & 0x100) != 0) ? SetFlag(kFlagC) : ClearFlag(kFlagC);
    break;
  }
  case 0xF9:
    reg_sp_ = reg_hl_.GetRegister();
    break;
  case 0xFA: {
    uint8_t byte =
        ReadMMU(MakeWord(ReadMMU(reg_pc_ + 2), ReadMMU(reg_pc_ + 1)));
    LD(&reg_a_, byte);
    break;
  }
  case 0xFB:
    ime_enable_pending_ = true;
    break;
  case 0xFC:
    break;
  case 0xFD:
    break;
  case 0xFE: {
    uint8_t byte = ReadMMU(reg_pc_ + 1);
    CP(byte);
    break;
  }
  case 0xFF:
    RST(rst_jump_vectors_[7]);
    break;
  default:
    break;
  }
  reg_pc_ += instruction.length_;
}

void CPU::ExecuteExtended(Instruction instruction) {
  switch (instruction.opcode_) {
  case 0x00:
    RLC(reg_b_);
    break;
  case 0x01:
    RLC(reg_c_);
    break;
  case 0x02:
    RLC(reg_d_);
    break;
  case 0x03:
    RLC(reg_e_);
    break;
  case 0x04:
    RLC(reg_h_);
    break;
  case 0x05:
    RLC(reg_l_);
    break;
  case 0x06: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RLC(byte);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x07:
    RLC(reg_a_);
    break;
  case 0x08:
    RRC(reg_b_);
    break;
  case 0x09:
    RRC(reg_c_);
    break;
  case 0x0A:
    RRC(reg_d_);
    break;
  case 0x0B:
    RRC(reg_e_);
    break;
  case 0x0C:
    RRC(reg_h_);
    break;
  case 0x0D:
    RRC(reg_l_);
    break;
  case 0x0E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RRC(byte);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x0F:
    RRC(reg_a_);
    break;
  case 0x10:
    RL(reg_b_);
    break;
  case 0x11:
    RL(reg_c_);
    break;
  case 0x12:
    RL(reg_d_);
    break;
  case 0x13:
    RL(reg_e_);
    break;
  case 0x14:
    RL(reg_h_);
    break;
  case 0x15:
    RL(reg_l_);
    break;
  case 0x16: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RL(byte);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x17:
    RL(reg_a_);
    break;
  case 0x18:
    RR(reg_b_);
    break;
  case 0x19:
    RR(reg_c_);
    break;
  case 0x1A:
    RR(reg_d_);
    break;
  case 0x1B:
    RR(reg_e_);
    break;
  case 0x1C:
    RR(reg_h_);
    break;
  case 0x1D:
    RR(reg_l_);
    break;
  case 0x1E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RR(byte);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x1F:
    RR(reg_a_);
    break;
  case 0x20:
    SLA(reg_b_);
    break;
  case 0x21:
    SLA(reg_c_);
    break;
  case 0x22:
    SLA(reg_d_);
    break;
  case 0x23:
    SLA(reg_e_);
    break;
  case 0x24:
    SLA(reg_h_);
    break;
  case 0x25:
    SLA(reg_l_);
    break;
  case 0x26: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SLA(byte);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x27:
    SLA(reg_a_);
    break;
  case 0x28:
    SRA(reg_b_);
    break;
  case 0x29:
    SRA(reg_c_);
    break;
  case 0x2A:
    SRA(reg_d_);
    break;
  case 0x2B:
    SRA(reg_e_);
    break;
  case 0x2C:
    SRA(reg_h_);
    break;
  case 0x2D:
    SRA(reg_l_);
    break;
  case 0x2E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SRA(byte);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x2F:
    SRA(reg_a_);
    break;
  case 0x30:
    SWAP(reg_b_);
    break;
  case 0x31:
    SWAP(reg_c_);
    break;
  case 0x32:
    SWAP(reg_d_);
    break;
  case 0x33:
    SWAP(reg_e_);
    break;
  case 0x34:
    SWAP(reg_h_);
    break;
  case 0x35:
    SWAP(reg_l_);
    break;
  case 0x36: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SWAP(byte);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x37:
    SWAP(reg_a_);
    break;
  case 0x38:
    SRL(reg_b_);
    break;
  case 0x39:
    SRL(reg_c_);
    break;
  case 0x3A:
    SRL(reg_d_);
    break;
  case 0x3B:
    SRL(reg_e_);
    break;
  case 0x3C:
    SRL(reg_h_);
    break;
  case 0x3D:
    SRL(reg_l_);
    break;
  case 0x3E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SRL(byte);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x3F:
    SRL(reg_a_);
    break;
  case 0x40:
    BIT(reg_b_, 0);
    break;
  case 0x41:
    BIT(reg_c_, 0);
    break;
  case 0x42:
    BIT(reg_d_, 0);
    break;
  case 0x43:
    BIT(reg_e_, 0);
    break;
  case 0x44:
    BIT(reg_h_, 0);
    break;
  case 0x45:
    BIT(reg_l_, 0);
    break;
  case 0x46: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    BIT(byte, 0);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x47:
    BIT(reg_a_, 0);
    break;
  case 0x48:
    BIT(reg_b_, 1);
    break;
  case 0x49:
    BIT(reg_c_, 1);
    break;
  case 0x4A:
    BIT(reg_d_, 1);
    break;
  case 0x4B:
    BIT(reg_e_, 1);
    break;
  case 0x4C:
    BIT(reg_h_, 1);
    break;
  case 0x4D:
    BIT(reg_l_, 1);
    break;
  case 0x4E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    BIT(byte, 1);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x4F:
    BIT(reg_a_, 1);
    break;
  case 0x50:
    BIT(reg_b_, 2);
    break;
  case 0x51:
    BIT(reg_c_, 2);
    break;
  case 0x52:
    BIT(reg_d_, 2);
    break;
  case 0x53:
    BIT(reg_e_, 2);
    break;
  case 0x54:
    BIT(reg_h_, 2);
    break;
  case 0x55:
    BIT(reg_l_, 2);
    break;
  case 0x56: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    BIT(byte, 2);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x57:
    BIT(reg_a_, 2);
    break;
  case 0x58:
    BIT(reg_b_, 3);
    break;
  case 0x59:
    BIT(reg_c_, 3);
    break;
  case 0x5A:
    BIT(reg_d_, 3);
    break;
  case 0x5B:
    BIT(reg_e_, 3);
    break;
  case 0x5C:
    BIT(reg_h_, 3);
    break;
  case 0x5D:
    BIT(reg_l_, 3);
    break;
  case 0x5E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    BIT(byte, 3);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x5F:
    BIT(reg_a_, 3);
    break;
  case 0x60:
    BIT(reg_b_, 4);
    break;
  case 0x61:
    BIT(reg_c_, 4);
    break;
  case 0x62:
    BIT(reg_d_, 4);
    break;
  case 0x63:
    BIT(reg_e_, 4);
    break;
  case 0x64:
    BIT(reg_h_, 4);
    break;
  case 0x65:
    BIT(reg_l_, 4);
    break;
  case 0x66: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    BIT(byte, 4);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x67:
    BIT(reg_a_, 4);
    break;
  case 0x68:
    BIT(reg_b_, 5);
    break;
  case 0x69:
    BIT(reg_c_, 5);
    break;
  case 0x6A:
    BIT(reg_d_, 5);
    break;
  case 0x6B:
    BIT(reg_e_, 5);
    break;
  case 0x6C:
    BIT(reg_h_, 5);
    break;
  case 0x6D:
    BIT(reg_l_, 5);
    break;
  case 0x6E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    BIT(byte, 5);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x6F:
    BIT(reg_a_, 5);
    break;
  case 0x70:
    BIT(reg_b_, 6);
    break;
  case 0x71:
    BIT(reg_c_, 6);
    break;
  case 0x72:
    BIT(reg_d_, 6);
    break;
  case 0x73:
    BIT(reg_e_, 6);
    break;
  case 0x74:
    BIT(reg_h_, 6);
    break;
  case 0x75:
    BIT(reg_l_, 6);
    break;
  case 0x76: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    BIT(byte, 6);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x77:
    BIT(reg_a_, 6);
    break;
  case 0x78:
    BIT(reg_b_, 7);
    break;
  case 0x79:
    BIT(reg_c_, 7);
    break;
  case 0x7A:
    BIT(reg_d_, 7);
    break;
  case 0x7B:
    BIT(reg_e_, 7);
    break;
  case 0x7C:
    BIT(reg_h_, 7);
    break;
  case 0x7D:
    BIT(reg_l_, 7);
    break;
  case 0x7E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    BIT(byte, 7);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x7F:
    BIT(reg_a_, 7);
    break;
  case 0x80:
    RES(reg_b_, 0);
    break;
  case 0x81:
    RES(reg_c_, 0);
    break;
  case 0x82:
    RES(reg_d_, 0);
    break;
  case 0x83:
    RES(reg_e_, 0);
    break;
  case 0x84:
    RES(reg_h_, 0);
    break;
  case 0x85:
    RES(reg_l_, 0);
    break;
  case 0x86: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RES(byte, 0);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x87:
    RES(reg_a_, 0);
    break;
  case 0x88:
    RES(reg_b_, 1);
    break;
  case 0x89:
    RES(reg_c_, 1);
    break;
  case 0x8A:
    RES(reg_d_, 1);
    break;
  case 0x8B:
    RES(reg_e_, 1);
    break;
  case 0x8C:
    RES(reg_h_, 1);
    break;
  case 0x8D:
    RES(reg_l_, 1);
    break;
  case 0x8E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RES(byte, 1);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x8F:
    RES(reg_a_, 1);
    break;
  case 0x90:
    RES(reg_b_, 2);
    break;
  case 0x91:
    RES(reg_c_, 2);
    break;
  case 0x92:
    RES(reg_d_, 2);
    break;
  case 0x93:
    RES(reg_e_, 2);
    break;
  case 0x94:
    RES(reg_h_, 2);
    break;
  case 0x95:
    RES(reg_l_, 2);
    break;
  case 0x96: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RES(byte, 2);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x97:
    RES(reg_a_, 2);
    break;
  case 0x98:
    RES(reg_b_, 3);
    break;
  case 0x99:
    RES(reg_c_, 3);
    break;
  case 0x9A:
    RES(reg_d_, 3);
    break;
  case 0x9B:
    RES(reg_e_, 3);
    break;
  case 0x9C:
    RES(reg_h_, 3);
    break;
  case 0x9D:
    RES(reg_l_, 3);
    break;
  case 0x9E: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RES(byte, 3);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0x9F:
    RES(reg_a_, 3);
    break;
  case 0xA0:
    RES(reg_b_, 4);
    break;
  case 0xA1:
    RES(reg_c_, 4);
    break;
  case 0xA2:
    RES(reg_d_, 4);
    break;
  case 0xA3:
    RES(reg_e_, 4);
    break;
  case 0xA4:
    RES(reg_h_, 4);
    break;
  case 0xA5:
    RES(reg_l_, 4);
    break;
  case 0xA6: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RES(byte, 4);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xA7:
    RES(reg_a_, 4);
    break;
  case 0xA8:
    RES(reg_b_, 5);
    break;
  case 0xA9:
    RES(reg_c_, 5);
    break;
  case 0xAA:
    RES(reg_d_, 5);
    break;
  case 0xAB:
    RES(reg_e_, 5);
    break;
  case 0xAC:
    RES(reg_h_, 5);
    break;
  case 0xAD:
    RES(reg_l_, 5);
    break;
  case 0xAE: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RES(byte, 5);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xAF:
    RES(reg_a_, 5);
    break;
  case 0xB0:
    RES(reg_b_, 6);
    break;
  case 0xB1:
    RES(reg_c_, 6);
    break;
  case 0xB2:
    RES(reg_d_, 6);
    break;
  case 0xB3:
    RES(reg_e_, 6);
    break;
  case 0xB4:
    RES(reg_h_, 6);
    break;
  case 0xB5:
    RES(reg_l_, 6);
    break;
  case 0xB6: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RES(byte, 6);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xB7:
    RES(reg_a_, 6);
    break;
  case 0xB8:
    RES(reg_b_, 7);
    break;
  case 0xB9:
    RES(reg_c_, 7);
    break;
  case 0xBA:
    RES(reg_d_, 7);
    break;
  case 0xBB:
    RES(reg_e_, 7);
    break;
  case 0xBC:
    RES(reg_h_, 7);
    break;
  case 0xBD:
    RES(reg_l_, 7);
    break;
  case 0xBE: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    RES(byte, 7);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xBF:
    RES(reg_a_, 7);
    break;
  case 0xC0:
    SET(reg_b_, 0);
    break;
  case 0xC1:
    SET(reg_c_, 0);
    break;
  case 0xC2:
    SET(reg_d_, 0);
    break;
  case 0xC3:
    SET(reg_e_, 0);
    break;
  case 0xC4:
    SET(reg_h_, 0);
    break;
  case 0xC5:
    SET(reg_l_, 0);
    break;
  case 0xC6: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SET(byte, 0);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xC7:
    SET(reg_a_, 0);
    break;
  case 0xC8:
    SET(reg_b_, 1);
    break;
  case 0xC9:
    SET(reg_c_, 1);
    break;
  case 0xCA:
    SET(reg_d_, 1);
    break;
  case 0xCB:
    SET(reg_e_, 1);
    break;
  case 0xCC:
    SET(reg_h_, 1);
    break;
  case 0xCD:
    SET(reg_l_, 1);
    break;
  case 0xCE: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SET(byte, 1);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xCF:
    SET(reg_a_, 1);
    break;
  case 0xD0:
    SET(reg_b_, 2);
    break;
  case 0xD1:
    SET(reg_c_, 2);
    break;
  case 0xD2:
    SET(reg_d_, 2);
    break;
  case 0xD3:
    SET(reg_e_, 2);
    break;
  case 0xD4:
    SET(reg_h_, 2);
    break;
  case 0xD5:
    SET(reg_l_, 2);
    break;
  case 0xD6: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SET(byte, 2);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xD7:
    SET(reg_a_, 2);
    break;
  case 0xD8:
    SET(reg_b_, 3);
    break;
  case 0xD9:
    SET(reg_c_, 3);
    break;
  case 0xDA:
    SET(reg_d_, 3);
    break;
  case 0xDB:
    SET(reg_e_, 3);
    break;
  case 0xDC:
    SET(reg_h_, 3);
    break;
  case 0xDD:
    SET(reg_l_, 3);
    break;
  case 0xDE: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SET(byte, 3);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xDF:
    SET(reg_a_, 3);
    break;
  case 0xE0:
    SET(reg_b_, 4);
    break;
  case 0xE1:
    SET(reg_c_, 4);
    break;
  case 0xE2:
    SET(reg_d_, 4);
    break;
  case 0xE3:
    SET(reg_e_, 4);
    break;
  case 0xE4:
    SET(reg_h_, 4);
    break;
  case 0xE5:
    SET(reg_l_, 4);
    break;
  case 0xE6: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SET(byte, 4);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xE7:
    SET(reg_a_, 4);
    break;
  case 0xE8:
    SET(reg_b_, 5);
    break;
  case 0xE9:
    SET(reg_c_, 5);
    break;
  case 0xEA:
    SET(reg_d_, 5);
    break;
  case 0xEB:
    SET(reg_e_, 5);
    break;
  case 0xEC:
    SET(reg_h_, 5);
    break;
  case 0xED:
    SET(reg_l_, 5);
    break;
  case 0xEE: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SET(byte, 5);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xEF:
    SET(reg_a_, 5);
    break;
  case 0xF0:
    SET(reg_b_, 6);
    break;
  case 0xF1:
    SET(reg_c_, 6);
    break;
  case 0xF2:
    SET(reg_d_, 6);
    break;
  case 0xF3:
    SET(reg_e_, 6);
    break;
  case 0xF4:
    SET(reg_h_, 6);
    break;
  case 0xF5:
    SET(reg_l_, 6);
    break;
  case 0xF6: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SET(byte, 6);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xF7:
    SET(reg_a_, 6);
    break;
  case 0xF8:
    SET(reg_b_, 7);
    break;
  case 0xF9:
    SET(reg_c_, 7);
    break;
  case 0xFA:
    SET(reg_d_, 7);
    break;
  case 0xFB:
    SET(reg_e_, 7);
    break;
  case 0xFC:
    SET(reg_h_, 7);
    break;
  case 0xFD:
    SET(reg_l_, 7);
    break;
  case 0xFE: {
    uint8_t byte = ReadMMU(reg_hl_.GetRegister());
    SET(byte, 7);
    WriteMMU(reg_hl_.GetRegister(), byte);
    break;
  }
  case 0xFF:
    SET(reg_a_, 7);
    break;
  }
  reg_pc_ += instruction.length_;
}
