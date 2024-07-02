#include "cpu.h"

void CPU::Run() { Fetch(); }

void CPU::Fetch() { Decode(mmu->ReadMemory(reg_pc_++)); }

void CPU::Decode(uint8_t opcode) { Execute(instructions.at(opcode)); }

void CPU::Execute(Instruction instruction) {
  std::cout << instruction.mnemonic_ << " " << std::hex << +reg_pc_ << '\n';
}
