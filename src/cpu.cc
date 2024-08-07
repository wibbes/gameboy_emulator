#include "cpu.h"

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

void CPU::SetFlag(uint8_t flag) { *reg_af_->low_ |= (0x01 << flag); }

void CPU::ClearFlag(uint8_t flag) { *reg_af_->low_ &= ~(0x01 << flag); }

void CPU::LD(Register16 reg, uint16_t value) { reg.SetRegister(value); }

void CPU::LD(uint8_t *reg, uint8_t value) { *reg = value; }

void CPU::PUSH(uint16_t value) {
  mmu_->WriteMemory(reg_sp_ - 1, static_cast<uint8_t>((value >> 8) & 0xFF));
  mmu_->WriteMemory(reg_sp_ - 2, static_cast<uint8_t>(value & 0xFF));
  reg_sp_ -= 2;
}

void CPU::POP(Register16 reg) {
  reg.SetRegister(
      MakeWord(mmu_->ReadMemory(reg_sp_ + 1), mmu_->ReadMemory(reg_sp_)));
  reg_sp_ += 2;
}

void CPU::RST(uint8_t jmp_vector) {
  PUSH(reg_pc_ + 1);
  reg_pc_ = static_cast<uint16_t>(rst_jump_vectors[jmp_vector]);
}

void CPU::JP() {
  reg_pc_ =
      MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1));
  reg_pc_ -= 3;
}

void CPU::JP(uint8_t condition) {
  bool flag = condition & 0x10 ? GetFlag(flag_c_) : GetFlag(flag_z_);
  if (flag & condition) {
    JP();
  } else { // If its a zero flag
    ++reg_pc_;
  }
}

void CPU::JP_HL() { reg_pc_ = reg_hl_->GetRegister(); }

void CPU::JR() {
  reg_pc_ += static_cast<int8_t>(mmu_->ReadMemory(reg_pc_ + 1));
}

void CPU::RET() {
  reg_pc_ = MakeWord(mmu_->ReadMemory(reg_sp_ + 1), mmu_->ReadMemory(reg_sp_));
  reg_sp_ += 2;
}

void CPU::CALL() {
  reg_pc_ += 2;
  mmu_->WriteMemory(reg_sp_ - 1, static_cast<uint8_t>((reg_pc_ >> 8) & 0xFF));
  mmu_->WriteMemory(reg_sp_ - 2, static_cast<uint8_t>(reg_pc_ & 0xFF));
  reg_pc_ =
      MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)) -
      1;
  reg_sp_ -= 2;
}

void CPU::BIT(uint8_t reg, uint8_t bit) {
  GetBit(reg, bit) == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  SetFlag(flag_c_);
}

void CPU::INC(Register16 reg) {
  reg.SetRegister(reg.GetRegister() + 1);
  // instruction does not change registers
}

void CPU::INC(uint8_t &reg) {
  ++reg;
  reg == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  (reg & 0x0F) == 0 ? SetFlag(flag_h_) : ClearFlag(flag_h_);
}

void CPU::INC_HL() {
  uint8_t eval = reg_hl_->GetRegister() + 1;
  eval == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  (eval & 0x0F) == 0 ? SetFlag(flag_h_) : ClearFlag(flag_h_);
  reg_hl_->SetRegister(eval);
}

void CPU::DEC(Register16 reg) {
  reg.SetRegister(reg.GetRegister() + 1);
  // instruction does not change registers
}

void CPU::DEC(uint8_t &reg) {
  --reg;
  reg == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  SetFlag(flag_n_);
  (reg & 0x0F) == 0x0F ? SetFlag(flag_h_) : ClearFlag(flag_h_);
}

void CPU::DEC_HL() {
  uint8_t eval = mmu_->ReadMemory(reg_hl_->GetRegister() + 1);
  eval == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  SetFlag(flag_n_);
  (eval & 0x0F) == 0x0F ? SetFlag(flag_h_) : ClearFlag(flag_h_);
  reg_hl_->SetRegister(eval);
}

void CPU::AND(uint8_t &reg) {
  reg_a_ &= reg;
  reg_a_ == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_h_);
  SetFlag(flag_n_);
  ClearFlag(flag_c_);
}
void CPU::OR(uint8_t &reg) {
  reg_a_ |= reg;
  reg_a_ == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_h_);
  SetFlag(flag_n_);
  ClearFlag(flag_c_);
}
void CPU::XOR(uint8_t &reg) {
  reg_a_ ^= reg;
  reg_a_ == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_h_);
  SetFlag(flag_n_);
  ClearFlag(flag_c_);
}
void CPU::CP(uint8_t &reg) {
  reg_a_ == reg ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  SetFlag(flag_n_);
  ((reg_a_ - reg) & 0x0F) > (reg_a_ & 0x0F) ? SetFlag(flag_h_)
                                            : ClearFlag(flag_h_);
  reg_a_ < reg ? SetFlag(flag_c_) : ClearFlag(flag_c_);
}

void CPU::RL(uint8_t &reg, bool extended) {
  uint8_t carry = GetFlag(flag_c_);
  (reg & 0x80) != 0 ? SetFlag(flag_c_) : ClearFlag(flag_c_);
  reg <<= 1;
  reg |= carry;
  ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ClearFlag(flag_h_);
  if (extended && reg == 0) {
    SetFlag(flag_z_);
  }
}

void CPU::RR(uint8_t &reg, bool extended) {
  uint8_t carry = GetFlag(flag_c_);
  (reg & 0x01) != 0 ? SetFlag(flag_c_) : ClearFlag(flag_c_);
  reg >>= 1;
  carry ? reg |= 0x80 : reg |= 0;
  ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ClearFlag(flag_h_);
  if (extended && reg == 0) {
    SetFlag(flag_z_);
  }
}

void CPU::RLC(uint8_t &reg, bool extended) {
  (reg & 0x80) != 0 ? SetFlag(flag_c_) : ClearFlag(flag_c_);
  ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ClearFlag(flag_h_);
  reg <<= 1;
  if (GetFlag(flag_c_)) {
    reg |= 0x01;
  }
  if (extended && reg == 0) {
    SetFlag(flag_z_);
  }
}

void CPU::RRC(uint8_t &reg, bool extended) {
  (reg & 0x80) != 0 ? SetFlag(flag_c_) : ClearFlag(flag_c_);
  ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ClearFlag(flag_h_);
  reg >>= 1;
  if (GetFlag(flag_c_)) {
    reg |= 0x80;
  }
  if (extended && reg == 0) {
    SetFlag(flag_z_);
  }
}

void CPU::SLA(uint8_t &reg) {
  (reg & 0x80) != 0 ? SetFlag(flag_c_) : ClearFlag(flag_c_);
  reg <<= 1;
  ClearBit(reg, 0);
  reg == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ClearFlag(flag_h_);
}

void CPU::SRA(uint8_t &reg) {
  (reg & 0x01) != 0 ? SetFlag(flag_c_) : ClearFlag(flag_c_);
  if ((reg & 0x80) != 0) {
    reg >>= 1;
    reg |= 0x80;
  } else {
    reg >>= 1;
  }
  reg == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ClearFlag(flag_h_);
}

void CPU::SRL(uint8_t &reg) {
  (reg & 0x01) != 0 ? SetFlag(flag_c_) : ClearFlag(flag_c_);
  reg >>= 1;
  ClearBit(reg, 7);
  reg == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ClearFlag(flag_h_);
}

void CPU::SWAP(uint8_t &reg) {
  reg = ((reg & 0x0F) << 4) | ((reg & 0xF0) >> 4);
  reg == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ClearFlag(flag_h_);
  ClearFlag(flag_c_);
}

void CPU::ADD(uint8_t &reg) {
  uint8_t carry = GetFlag(flag_c_);
  int8_t eval = reg_a_ + reg;
  int16_t test_carries = static_cast<int16_t>(reg_a_ ^ reg ^ eval);
  reg_a_ = eval;
  reg_a_ == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ((test_carries & 0x10) != 0) ? SetFlag(flag_h_) : ClearFlag(flag_h_);
  ((test_carries & 0x100) != 0) ? SetFlag(flag_c_) : ClearFlag(flag_c_);
}

void CPU::ADD_HL(uint16_t value) {
  int16_t eval = reg_hl_->GetRegister() + value;
  int test_carries =
      static_cast<int16_t>(reg_hl_->GetRegister() ^ value ^ eval);
  ClearFlag(flag_n_);
  ((test_carries & 0x1000) != 0) ? SetFlag(flag_h_) : ClearFlag(flag_h_);
  ((test_carries & 0x10000) != 0) ? SetFlag(flag_c_) : ClearFlag(flag_c_);
  reg_hl_->SetRegister(eval);
}

void CPU::ADD_SP() {
  int immediate = static_cast<char>(mmu_->ReadMemory(reg_pc_ + 1));
  int eval = reg_sp_ + immediate;
  int test_carries = reg_sp_ ^ immediate ^ eval;
  reg_sp_ = eval;
  ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  (test_carries & 0x10) != 0 ? SetFlag(flag_h_) : ClearFlag(flag_h_);
  (test_carries & 0x100) != 0 ? SetFlag(flag_c_) : ClearFlag(flag_c_);
}

void CPU::ADC(uint8_t &reg) {
  uint8_t carry = GetFlag(flag_c_);
  int8_t eval = reg_a_ + (reg + carry);
  int16_t test_carries = static_cast<int16_t>(reg_a_ ^ reg ^ eval);
  reg_a_ = eval;
  reg_a_ == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  ClearFlag(flag_n_);
  ((test_carries & 0x10) != 0) ? SetFlag(flag_h_) : ClearFlag(flag_h_);
  ((test_carries & 0x100) != 0) ? SetFlag(flag_c_) : ClearFlag(flag_c_);
}

void CPU::SUB(uint8_t &reg) {
  int8_t eval = reg_a_ - reg;
  int16_t test_carries = static_cast<int16_t>(reg_a_ ^ reg ^ eval);
  reg_a_ = eval;
  reg_a_ == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  SetFlag(flag_n_);
  ((test_carries & 0x10) != 0) ? SetFlag(flag_h_) : ClearFlag(flag_h_);
  ((test_carries & 0x100) != 0) ? SetFlag(flag_c_) : ClearFlag(flag_c_);
}

void CPU::SBC(uint8_t &reg) {
  uint8_t carry = GetFlag(flag_c_);
  int8_t eval = reg_a_ - (reg + carry);
  int16_t test_carries = static_cast<int16_t>(reg_a_ ^ reg ^ eval);
  reg_a_ = eval;
  reg_a_ == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
  SetFlag(flag_n_);
  ((test_carries & 0x10) != 0) ? SetFlag(flag_h_) : ClearFlag(flag_h_);
  ((test_carries & 0x100) != 0) ? SetFlag(flag_c_) : ClearFlag(flag_c_);
}

void CPU::DAA() {
  if (!GetFlag(flag_n_)) {
    if (GetFlag(flag_c_) || reg_a_ > 0x99) {
      reg_a_ += 0x60;
      SetFlag(flag_c_);
    }
    if (GetFlag(flag_h_) || (reg_a_ & 0x0F) > 0x09) {
      reg_a_ += 0x06;
    }
  } else {
    if (GetFlag(flag_c_))
      reg_a_ -= 0x60;
    if (GetFlag(flag_h_))
      reg_a_ -= 0x06;
  }
  ClearFlag(flag_h_);
  reg_a_ == 0 ? SetFlag(flag_z_) : ClearFlag(flag_z_);
}

void CPU::CPL() {
  reg_a_ = ~reg_a_;
  SetFlag(flag_n_);
  SetFlag(flag_h_);
}

void CPU::NOP() { return; }

void CPU::CCF() {
  reg_a_ ^= (0x01 << flag_c_);
  ClearFlag(flag_n_);
  ClearFlag(flag_h_);
}

void CPU::SCF() { SetFlag(flag_c_); }

void CPU::DI() { ime = false; }

void CPU::EI() { ime = true; }

void CPU::HALT() { halted = true; }

void CPU::STOP() {
  // Before calling this...
  //  - reset interrupt enable
  //  - reset I/O
}

void CPU::Run() { Fetch(); }

void CPU::Fetch() { Decode(mmu_->ReadMemory(reg_pc_)); }

void CPU::Decode(uint8_t opcode) { Execute(instructions.at(opcode)); }

void CPU::Execute(Instruction instruction) {
  std::cout << std::hex << std::uppercase
            << instruction.mnemonic_
            << " Opcode: " << +instruction.opcode_ << " "
            << +mmu_->memory_[reg_pc_ + 1] << " " << +mmu_->memory_[reg_pc_ + 2]
            << " PC: " << +reg_pc_ << " Length: " << +instruction.length_
            << '\n';
  switch (instruction.opcode_) {
  case 0x00:
    NOP();
    break;
  case 0x01:
    LD(*reg_bc_,
       MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)));
    break;
  case 0x02: // LD (BC), A
    mmu_->WriteMemory(reg_bc_->GetRegister(), reg_a_);
    break;
  case 0x03:
    INC(*reg_bc_);
    break;
  case 0x04:
    INC(reg_b_);
    break;
  case 0x05:
    DEC(reg_b_);
    break;
  case 0x06: // LD B, d8
    LD(&reg_b_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x07:
    RLC(reg_a_);
    break;
  case 0x08: // LD (a16), SP
    mmu_->WriteMemory(
        MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)),
        reg_sp_ & 0xFF);
    mmu_->WriteMemory(
        MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)) +
            1,
        (reg_sp_ & 0xFF00) >> 8);
    break;
  case 0x09:
    mmu_->WriteMemory(mmu_->ReadMemory(MakeWord(mmu_->ReadMemory(reg_pc_ + 2),
                                                mmu_->ReadMemory(reg_pc_ + 1))),
                      reg_sp_);
    break;
  case 0x0A:
    LD(&reg_a_, mmu_->ReadMemory(reg_bc_->GetRegister()));
    break;
  case 0x0B:
    DEC(*reg_bc_);
    break;
  case 0x0C:
    INC(reg_c_);
    break;
  case 0x0D:
    DEC(reg_c_);
    break;
  case 0x0E:
    LD(&reg_c_, mmu_->ReadMemory(reg_pc_ + 1));
    break;
  case 0x0F:
    RRC(reg_a_);
    break;
  case 0x10:
    STOP();
    break;
  case 0x11:
    LD(*reg_de_,
       MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)));
    break;
  case 0x12:
    mmu_->WriteMemory(reg_de_->GetRegister(), reg_a_);
    break;
  case 0x13:
    INC(*reg_de_);
    break;
  case 0x14:
    INC(reg_d_);
    break;
  case 0x15:
    DEC(reg_d_);
    break;
  case 0x16:
    LD(&reg_d_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x17:
    RL(reg_a_);
    break;
  case 0x18:
    JR();
    break;
  case 0x19:
    ADD_HL(reg_de_->GetRegister());
    break;
  case 0x1A:
    LD(&reg_a_, mmu_->ReadMemory(reg_de_->GetRegister()));
    break;
  case 0x1B:
    DEC(*reg_de_);
    break;
  case 0x1C:
    INC(reg_e_);
    break;
  case 0x1D:
    DEC(reg_e_);
    break;
  case 0x1E:
    LD(&reg_e_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x1F:
    RR(reg_a_);
    break;
  case 0x20:
    if (!GetFlag(flag_z_)) {
      JR();
    }
    break;
  case 0x21:
    LD(*reg_hl_,
       MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)));

    break;
  case 0x22:
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_a_);
    reg_hl_->SetRegister(reg_hl_->GetRegister() + 1);
    break;
  case 0x23:
    INC(*reg_hl_);
    break;
  case 0x24:
    INC(reg_h_);
    break;
  case 0x25:
    DEC(reg_h_);
    break;
  case 0x26:
    LD(&reg_h_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x27:
    DAA();
    break;
  case 0x28:
    if (GetFlag(flag_z_)) {
      JR();
    }
    break;
  case 0x29:
    ADD_HL(reg_hl_->GetRegister());
    break;
  case 0x2A:
    LD(&reg_a_, mmu_->ReadMemory(reg_hl_->GetRegister()));
    reg_hl_->SetRegister(reg_hl_->GetRegister() + 1);
    break;
  case 0x2B:
    DEC(*reg_hl_);
    break;
  case 0x2C:
    INC(reg_l_);
    break;
  case 0x2D:
    DEC(reg_l_);
    break;
  case 0x2E:
    LD(&reg_l_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x2F:
    CPL();
    break;
  case 0x30:
    if (!GetFlag(flag_c_)) {
      JR();
    }
    break;
  case 0x31:
    reg_sp_ =
        MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1));
    break;
  case 0x32:
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_a_);
    reg_hl_->SetRegister(reg_hl_->GetRegister() - 1);
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
    mmu_->WriteMemory(reg_hl_->GetRegister(), mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x37:
    SCF();
    break;
  case 0x38:
    if (GetFlag(flag_c_)) {
      JR();
    }
    break;
  case 0x39:
    ADD_HL(reg_sp_);
    break;
  case 0x3A:
    LD(&reg_a_, mmu_->ReadMemory(reg_hl_->GetRegister()));
    reg_hl_->SetRegister(reg_hl_->GetRegister() - 1);
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
    LD(&reg_a_, mmu_->ReadMemory(++reg_pc_));
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
    LD(&reg_b_, mmu_->ReadMemory(reg_hl_->GetRegister()));
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
  case 0x4D: // LD C, L
    LD(&reg_c_, reg_l_);
    break;
  case 0x4E: // LD C, (HL)
    LD(&reg_c_, mmu_->ReadMemory(reg_hl_->GetRegister()));
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
    LD(&reg_d_, mmu_->ReadMemory(reg_hl_->GetRegister()));
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
    LD(&reg_e_, mmu_->ReadMemory(reg_hl_->GetRegister()));
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
    LD(&reg_h_, mmu_->ReadMemory(reg_hl_->GetRegister()));
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
    LD(&reg_l_, mmu_->ReadMemory(reg_hl_->GetRegister()));
    break;
  case 0x6F: // LD L, A
    LD(&reg_l_, reg_a_);
    break;
  case 0x70: // LD (HL), B
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_b_);
    break;
  case 0x71: // LD (HL), C
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_c_);
    break;
  case 0x72: // LD (HL), D
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_d_);
    break;
  case 0x73: // LD (HL), E
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_e_);
    break;
  case 0x74: // LD (HL), H
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_h_);
    break;
  case 0x75: // LD (HL), L
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_l_);
    break;
  case 0x76:
    HALT();
    break;
  case 0x77: // LD (HL), A
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_a_);
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
    LD(&reg_a_, mmu_->ReadMemory(reg_hl_->GetRegister()));
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
    uint8_t byte = mmu_->ReadMemory(reg_hl_->GetRegister());
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
    uint8_t byte = mmu_->ReadMemory(reg_hl_->GetRegister());
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
    uint8_t byte = mmu_->ReadMemory(reg_hl_->GetRegister());
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
    uint8_t byte = mmu_->ReadMemory(reg_hl_->GetRegister());
    SBC(byte);
    break;
  }
  case 0x9F:
    SBC(reg_b_);
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
    uint8_t byte = mmu_->ReadMemory(reg_hl_->GetRegister());
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
    uint8_t byte = mmu_->ReadMemory(reg_hl_->GetRegister());
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
    uint8_t byte = mmu_->ReadMemory(reg_hl_->GetRegister());
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
    uint8_t byte = mmu_->ReadMemory(reg_hl_->GetRegister());
    CP(byte);
    break;
  }
  case 0xBF:
    CP(reg_a_);
    break;
  case 0xC0:
    if (!GetFlag(flag_z_)) {
      RET();
    }
    break;
  case 0xC1:
    POP(*reg_bc_);
    break;
  case 0xC2:
    if (!GetFlag(flag_z_)) {
      JP();
    }
    break;
  case 0xC3:
    JP();
    break;
  case 0xC4:
    if (!GetFlag(flag_z_)) {
      CALL();
    }
    break;
  case 0xC5:
    PUSH(reg_bc_->GetRegister());
    break;
  case 0xC6: {
    uint8_t byte = mmu_->ReadMemory(reg_pc_ + 1);
    ADD(byte);
    break;
  }
  case 0xC7:
    RST(rst_jump_vectors[0]);
    break;
  case 0xC8:
    if (GetFlag(flag_z_)) {
      RET();
    }
    break;
  case 0xC9:
    RET();
    break;
  case 0xCA:
    if (GetFlag(flag_z_)) {
      JP();
    }
    break;
  case 0xCB:
    // Extended instruciton prefix
    break;
  case 0xCC:
    if (GetFlag(flag_z_)) {
      CALL();
    }
    break;
  case 0xCD:
    CALL();
    break;
  case 0xCE: {
    uint8_t byte = mmu_->ReadMemory(reg_pc_ + 1);
    ADC(byte);
    break;
  }
  case 0xCF:
    RST(rst_jump_vectors[1]);
    break;
  case 0xD0:
    if (!GetFlag(flag_c_)) {
      RET();
    }
    break;
  case 0xD1:
    POP(*reg_de_);
    break;
  case 0xD2:
    if (!GetFlag(flag_c_)) {
      JP();
    }
    break;
  case 0xD3:
    break;
  case 0xD4:
    if (!GetFlag(flag_c_)) {
      CALL();
    }
    break;
  case 0xD5:
    PUSH(reg_de_->GetRegister());
    break;
  case 0xD6: {
    uint8_t byte = mmu_->ReadMemory(reg_pc_ + 1);
    SUB(byte);
    break;
  }
  case 0xD7:
    RST(rst_jump_vectors[3]);
    break;
  case 0xD8:
    if (GetFlag(flag_c_)) {
      RET();
    }
    break;
  case 0xD9:
    RET();
    ime = false;
    break;
  case 0xDA:
    if (GetFlag(flag_c_)) {
      JP();
    }
    break;
  case 0xDB:
    break;
  case 0xDC:
    if (GetFlag(flag_c_)) {
      CALL();
    }
    break;
  case 0xDD:
    break;
  case 0xDE: {
    uint8_t byte = mmu_->ReadMemory(reg_pc_ + 1);
    SBC(byte);
    break;
  } break;
  case 0xDF:
    RST(rst_jump_vectors[3]);
    break;
  case 0xE0: {
    uint16_t word =
        static_cast<uint16_t>(0xFF00 + mmu_->ReadMemory(reg_pc_ + 1));
    mmu_->WriteMemory(word, reg_a_);
    break;
  }
  case 0xE1:
    POP(*reg_hl_);
    break;
  case 0xE2:
    // MSB == 0xFF, so the possible range is 0xFF00 - 0xFFFF
    mmu_->WriteMemory(MakeWord(0xFF, reg_c_), reg_a_);
    break;
  case 0xE3:
    break;
  case 0xE4:
    break;
  case 0xE5:
    PUSH(reg_hl_->GetRegister());
    break;
  case 0xE6: {
    uint8_t byte = mmu_->ReadMemory(reg_pc_ + 1);
    AND(byte);
    break;
  }
  case 0xE7:
    RST(rst_jump_vectors[4]);
    break;
  case 0xE8:
    ADD_SP();
    break;
  case 0xE9:
    JP_HL();
    break;
  case 0xEA:
    mmu_->WriteMemory(MakeWord(mmu_->ReadMemory(reg_pc_ + 2), reg_pc_ + 1),
                      reg_a_);
    break;
  case 0xEB:
    break;
  case 0xEC:
    break;
  case 0xED:
    break;
  case 0xEE: {
    uint8_t byte = mmu_->ReadMemory(reg_pc_ + 1);
    XOR(byte);
    break;
  }
  case 0xEF:
    RST(rst_jump_vectors[5]);
    break;
  case 0xF0:
    if (mmu_->ReadMemory(reg_pc_ + 1) == 0x44) {
      reg_a_ = 0x90;
    } else {
      uint16_t word =
          static_cast<uint16_t>(0xFF00 + mmu_->ReadMemory(reg_pc_ + 1));
      LD(&reg_a_, mmu_->ReadMemory(word));
    }
    break;
  case 0xF1:
    POP(*reg_af_);
    *reg_af_->low_ &= 0xF0; // don't modify flags register
    break;
  case 0xF2:
    LD(&reg_a_, mmu_->ReadMemory(MakeWord(0xFF, reg_c_)));
    break;
  case 0xF3:
    DI();
    break;
  case 0xF4:
    break;
  case 0xF5:
    PUSH(reg_af_->GetRegister());
    break;
  case 0xF6: {
    uint8_t byte = mmu_->ReadMemory(reg_pc_ + 1);
    OR(byte);
    break;
  }
  case 0xF7:
    RST(rst_jump_vectors[6]);
    break;
  case 0xF8: {
    int immediate = static_cast<char>(mmu_->ReadMemory(reg_pc_ + 1));
    int eval = reg_sp_ + immediate;
    int test_carries = reg_sp_ ^ immediate ^ eval;
    reg_hl_->SetRegister(reg_sp_ + immediate);
    ClearFlag(flag_z_);
    ClearFlag(flag_n_);
    (test_carries & 0x10) != 0 ? SetFlag(flag_h_) : ClearFlag(flag_h_);
    (test_carries & 0x100) != 0 ? SetFlag(flag_c_) : ClearFlag(flag_c_);
  }
  case 0xF9:
    reg_sp_ = reg_hl_->GetRegister();
    break;
  case 0xFA:
    LD(&reg_a_, mmu_->ReadMemory(MakeWord(mmu_->ReadMemory(reg_pc_ + 2),
                                          mmu_->ReadMemory(reg_pc_ + 1))));
    break;
  case 0xFB:
    EI();
    break;
  case 0xFC:
    break;
  case 0xFD:
    break;
  case 0xFE: {
    uint8_t byte = mmu_->ReadMemory(reg_pc_ + 1);
    CP(byte);
    break;
  }
  case 0xFF:
    RST(rst_jump_vectors[7]);
    break;
  default:
    break;
  }
  reg_pc_ += instruction.length_;
}
