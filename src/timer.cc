#include "timer.h"

void Timer::UpdateTimer(uint8_t cycles) noexcept {
  tac_ = mmu_->ReadMemory(0xFF07);
  UpdateDIV(cycles);
  // Is the timer enabled?
  if(tac_ & (1 << 2)) {
    UpdateTIMA(cycles);
  }
}

void Timer::UpdateDIV(uint8_t cycles) noexcept {
  div_clock_ += cycles;
  while(div_clock_ >= 256) {
    ++div_;
    div_clock_ -= 256;
  }
}

void Timer::UpdateTIMA(uint8_t cycles) noexcept {
  uint16_t clock_rate_ = frequencies_[tac_ & 0x03];
  tima_clock_ += cycles;
  if(tima_reset_pending_) {
    tima_ = tma_;
    tima_reset_pending_ = false;
    mmu_->WriteMemory(0xFF05, tima_);
  }
  while(tima_clock_ >= clock_rate_) {
    tima_clock_ -= clock_rate_;
    if(tima_ == 0xFF) {
      mmu_->if_->SetInterrupt(2);
      tima_ = 0x00;
      tima_reset_pending_ = true;
    } else {
      ++tima_;
    }
    mmu_->WriteMemory(0xFF05, tima_);
  }
}
