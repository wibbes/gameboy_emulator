#include "mmu.h"

void InterruptRegister::SetState(uint8_t state) {
  // ints are implicitly converted so this is generally type-safe to do
  state_ = (state & 0b00011111);
}

void InterruptRegister::SetInterrupt(uint8_t interrupt) {
  state_.set(interrupt);
}

void InterruptRegister::ResetInterrupt(uint8_t interrupt) {
  state_.reset(interrupt);
}

uint8_t InterruptRegister::GetState() {
  if (state_.none())
    return 0;
  return static_cast<uint8_t>(state_.to_ulong());
}

uint8_t InterruptRegister::GetInterrupt(uint8_t interrupt) {
  return state_.test(interrupt);
}

void MMU::WriteMemory(uint16_t address, uint8_t value) {
  switch (address) {
  case 0xFF01:
    std::cout << std::hex << value;
    break;
  case 0xFF0F: // interrupt flags
    std::cout << "Wrote " << +value << " to IF" << '\n';
    if_->SetState(value);
    memory_[0xFF0F] = value;
    break;
  case 0xFFFF: // Interrupt enable
    std::cout << "Wrote " << +value << " to IE" << '\n';
    ie_->SetState(value);
    memory_[0xFFFF] = value;
    break;
  default:
    memory_[address] = value;
    break;
  }
}

uint8_t MMU::ReadMemory(uint16_t address) {
  if (address == 0xFF44)
    return 0x90;
  return memory_[address];
}
