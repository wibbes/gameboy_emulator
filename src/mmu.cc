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
  case 0xFF01: // serial
    std::cout << std::hex << value;
    break;
  case 0xFF04: // DIV
    memory_[0xFF04] = 0x00;
    timer_->Write(0xFF04, 0x00);
    break;
  case 0xFF05: // TIMA
    memory_[0xFF05] = value;
    timer_->Write(0xFF05, value);
    break;
  case 0xFF06: // TMA
    memory_[0xFF06] = value;
    timer_->Write(0xFF06, value);
    break;
  case 0xFF07: // TAC
    memory_[0xFF07] = value;
    std::cout << memory_[0xFf07] << '\n';
    timer_->Write(0xFF07, value);
    break;
  case 0xFF0F: // interrupt flags
    if_->SetState(value);
    memory_[0xFF0F] = value;
    break;
  case 0xFFFF: // interrupt flags
    ie_->SetState(value);
    memory_[0xFFFF] = value;
    break;
  default:
    memory_[address] = value;
    break;
  }
}

uint8_t MMU::ReadMemory(uint16_t address) {
  switch (address) {
  case 0xFF04: // DIV 
    return timer_->Read(0xFF04);
    break;
  case 0xFF05: // TIMA   
    return timer_->Read(0xFF05);
    break;
  case 0xFF06: // TMA
    return timer_->Read(0xFF06);
    break;
  case 0xFF07: // TAC
    return timer_->Read(0xFF07);
    break;
  case 0xFF0F: // interrupt flags
    return if_->GetState();
  case 0xFFFF: // interrupt enable
    return ie_->GetState();
  case 0xFF44:
    return 0x90;
  default:
    return memory_[address];
  }
}
