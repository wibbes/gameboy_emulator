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
      MakeWord(mmu_->ReadMemory(reg_sp_ + 1), mmu_->ReadMemory(reg_sp_ + 2)));
  reg_sp_ += 2;
}

void CPU::RST(uint8_t jmp_vector) {
  PUSH(reg_pc_ + 1);
  reg_pc_ = static_cast<uint16_t>(rst_jump_vectors[jmp_vector]);
}

void CPU::JP() {
  reg_pc_ =
      MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1));
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
}

void CPU::CALL() {
  reg_pc_ += 2;
  mmu_->WriteMemory(reg_sp_ - 1, static_cast<uint8_t>((reg_pc_ >> 8) & 0xFF));
  mmu_->WriteMemory(reg_sp_ - 2, static_cast<uint8_t>(reg_pc_ & 0xFF));
  reg_pc_ =
      MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1));
  reg_sp_ -= 2;
}

void CPU::BIT(uint8_t bit, uint8_t reg) {
}

void CPU::Run() { Fetch(); }

void CPU::Fetch() { Decode(mmu_->ReadMemory(reg_pc_++)); }

void CPU::Decode(uint8_t opcode) {
  if (opcode != 0x00)
    std::cout << instructions.at(opcode).mnemonic_ << '\n';
  Execute(instructions.at(opcode));
}

void CPU::Execute(Instruction instruction) {
  switch (instruction.opcode_) {
  case 0x01: // LD BC, d16
    LD(*reg_bc_,
       MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)));
    reg_pc_ += 2;
    break;
  case 0x02: // LD (BC), A
    mmu_->WriteMemory(reg_bc_->GetRegister(), reg_a_);
    break;
  case 0x06: // LD B, d8
    LD(&reg_b_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x08: // LD (a16), SP
    mmu_->WriteMemory(
        MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)),
        reg_sp_ & 0xFF);
    mmu_->WriteMemory(
        MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)) +
            1,
        (reg_sp_ & 0xFF00) >> 8);
    reg_pc_ += 2;
    break;
  case 0x0A: // LD A, (BC)
    LD(&reg_a_, mmu_->ReadMemory(reg_bc_->GetRegister()));
    break;
  case 0x0E: // LD C, d8
    LD(&reg_e_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x11: // LD DE, d16
    LD(*reg_de_,
       MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)));
    reg_pc_ += 2;
    break;
  case 0x12: // LD (DE), A
    mmu_->WriteMemory(reg_de_->GetRegister(), reg_a_);
    break;
  case 0x16: // LD D, d8
    LD(&reg_d_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x1A: // LD A, (DE)
    LD(&reg_a_, mmu_->ReadMemory(reg_de_->GetRegister()));
    break;
  case 0x1E: // LD E, d8
    LD(&reg_e_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x21: // LD HL, d16
    LD(*reg_hl_,
       MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1)));
    reg_pc_ += 2;
    break;
  case 0x22: // LD (HL+), A
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_a_);
    reg_hl_->SetRegister(reg_hl_->GetRegister() + 1);
    break;
  case 0x26: // LD H, d8
    LD(&reg_h_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x2A: // LD A, (HL+)
    LD(&reg_a_, mmu_->ReadMemory(reg_hl_->GetRegister()));
    reg_hl_->SetRegister(reg_hl_->GetRegister() + 1);
    break;
  case 0x2E: // LD L, d8
    LD(&reg_l_, mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x31: // LD SP, d16
    reg_sp_ =
        MakeWord(mmu_->ReadMemory(reg_pc_ + 2), mmu_->ReadMemory(reg_pc_ + 1));
    reg_pc_ += 2;
    break;
  case 0x32: // LD (HL-), A
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_a_);
    reg_hl_->SetRegister(reg_hl_->GetRegister() - 1);
    break;
  case 0x36: // LD (HL), d8
    mmu_->WriteMemory(reg_hl_->GetRegister(), mmu_->ReadMemory(++reg_pc_));
    break;
  case 0x3A: // LD A, (HL-)
    LD(&reg_a_, mmu_->ReadMemory(reg_hl_->GetRegister()));
    reg_hl_->SetRegister(reg_hl_->GetRegister() - 1);
    break;
  case 0x3E: // LD A, d8
    LD(&reg_a_, mmu_->ReadMemory(++reg_pc_));
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
    break;
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
    reg_pc_ += 2;
    break;
  case 0x71: // LD (HL), C
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_c_);
    reg_pc_ += 2;
    break;
  case 0x72: // LD (HL), D
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_d_);
    reg_pc_ += 2;
    break;
  case 0x73: // LD (HL), E
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_e_);
    reg_pc_ += 2;
    break;
  case 0x74: // LD (HL), H
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_h_);
    reg_pc_ += 2;
    break;
  case 0x75: // LD (HL), L
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_l_);
    reg_pc_ += 2;
    break;
  case 0x77: // LD (HL), A
    mmu_->WriteMemory(reg_hl_->GetRegister(), reg_a_);
    reg_pc_ += 2;
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
  case 0xE2: // LD (C), A
    // MSB == 0xFF, so the possible range is 0xFF00 - 0xFFFF
    mmu_->WriteMemory(MakeWord(0xFF, reg_c_), reg_a_);
    break;
  case 0xEA: // LD (a16), A
    mmu_->WriteMemory(MakeWord(mmu_->ReadMemory(reg_pc_ + 2), reg_pc_ + 1),
                      reg_a_);
    reg_pc_ += 2;
    break;
  case 0xF2: // LD A, (C)
    LD(&reg_a_, mmu_->ReadMemory(MakeWord(0xFF, reg_c_)));
    break;
  case 0xFA: // LD A, (a16)
    LD(&reg_a_, mmu_->ReadMemory(MakeWord(mmu_->ReadMemory(reg_pc_ + 2),
                                          mmu_->ReadMemory(reg_pc_ + 1))));
    break;
  default:
    break;
  }
}
