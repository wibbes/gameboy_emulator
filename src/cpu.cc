#include "cpu.h"

uint16_t Register16::GetRegister() { return *high << 8 | (*low & 0x00FF); }

void Register16::SetRegister(uint16_t value) {
  *high = value >> 8;
  *low = static_cast<uint8_t>(value & 0x00FF);
}

void CPU::Run() { Fetch(); }

void CPU::Fetch() { Decode(mmu->ReadMemory(reg_pc_++)); }

void CPU::Decode(uint8_t opcode) { Execute(instructions.at(opcode)); }

void CPU::Execute(Instruction instruction) {
  std::cout << instruction.mnemonic_ << " " << std::hex << +reg_pc_ << '\n';
  switch (instruction.opcode_) {
  case 0x00:
    break;
  default:
    break;
  }
}
