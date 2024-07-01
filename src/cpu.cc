#include "cpu.h"

void CPU::Run() { Fetch(); }

void CPU::Fetch() { Decode(mmu->ReadMemory(reg_pc_++)); }

void CPU::Decode(uint8_t opcode) { Execute(instructions_[opcode]); }

void CPU::Execute(Instruction instruction) {}
