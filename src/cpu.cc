#include "cpu.h"

void CPU::Run() { Fetch(); }

void CPU::Fetch() { Decode(mmu->ReadMemory(reg_pc_++)); }

void CPU::Decode(uint8_t opcode) { Execute(instructions.at(0x00)); }

void CPU::Execute(Instruction instruction) {
  switch (instruction.opcode_) {
  case 0x00:
    std::cout << "Opcode 0x00: " << instruction.mnemonic_ << '\n';
    break;
  default:
    break;
  }
}
