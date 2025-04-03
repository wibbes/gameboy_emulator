#include "timer.h"

uint8_t Timer::Read(uint16_t address) {
  switch (address) {
  case 0xFF04:
    return div_ >> 8;
  case 0xFF05:
    return tima_;
  case 0xFF06:
    return tma_;
  case 0xFF07:
    return tac_ | kTACMask;
  default:
    break;
  }
  return 0;
};

void Timer::Write(uint16_t address, uint8_t value) {
  switch (address) {
  case 0xFF04:
    div_ = 0;
    break;
  case 0xFF05:
    tima_ = value;
    break;
  case 0xFF06:
    tma_ = value;
    break;
  case 0xFF07:
    tac_ = value & 0x07;
    timer_enabled_ = (tac_ & 0x04);
    current_frequency_ = frequencies_[tac_ & 0x03];
    break;
  default:
    break;
  }

  // switch (address) {
  // case 0xFF04: // reset DIV anytime it's written to directly
  //   div_ = 0;
  //   break;
  // case 0xFF05:
  //   if (tima_overflow_ticks_ != 5) {
  //     tima_ = value;
  //     tima_overflowed_ = false;
  //     tima_overflow_ticks_ = 0;
  //   }
  //   break;
  // case 0xFF06:
  //   tma_ = value;
  //   if (tima_overflow_ticks_ == 5)
  //     tima_ = value;
  //   break;
  // case 0xFF07: {
  //   bool old_timer_enabled = timer_enabled_;
  //   uint16_t old_frequency = current_frequency_;
  //   tac_ = value & 0b111;
  //   current_frequency_ = 1 << frequencies_[tac_ & 0b11];
  //   timer_enabled_ = (tac_ & (1 << 2)) != 0;
  //   TIMABug(old_timer_enabled, old_frequency);
  // } break;
  // default:
  //   break;
  // }
  // return;
}

void Timer::Tick() {
  bool curr_bit = (div_ >> current_frequency_) & 1;

  div_ = (div_ + 1) & 0xFFFF;

  bool next_bit = (div_ >> current_frequency_) & 1;

  if (timer_enabled_ && curr_bit && !next_bit) {
    if (++tima_ == 0x00) {
      tima_ = tma_;
      if_ptr_->SetInterrupt(2); // Bit 2 = Timer
    }
  }

  // ++div_;
  // bool freq = (div_ & current_frequency_) != 0;
  // freq &= timer_enabled_;
  //
  // if (last_freq_ && !freq) {
  //   if (++tima_ == 0x00) {
  //     tima_overflow_ticks_ = 0;
  //   }
  // }
  //
  // last_freq_ = freq;
  //
  // if (tima_overflowed_) {
  //   ++tima_overflow_ticks_;
  //   if (tima_overflow_ticks_ == 4)
  //     if_ptr_->SetInterrupt(2);
  //   else if (tima_overflow_ticks_ == 5) {
  //     tima_ = tma_;
  //   } else if (tima_overflow_ticks_ == 6) {
  //     tima_overflowed_ = false;
  //     tima_overflow_ticks_ = 0;
  //   }
  // }
}
//
// // https://gbdev.gg8.se/wiki/articles/Timer_Obscure_Behaviour
// //
// // When writing to TAC, if the previously selected multiplexer input was '1'
// and
// // the new input is '0', TIMA will increase too. This doesn't happen when the
// // timer is disabled, but it also happens when disabling the timer (the same
// // effect as writing to DIV). The following code explains the behaviour in
// DMG
// // and MGB.
void Timer::TIMABug(bool old_enable, uint16_t old_freq) {
  if (old_enable)
    return;
  if (div_ & old_freq) {
    if (timer_enabled_ || !(div_ & current_frequency_)) {
      if (++tima_ == 0x00) {
        tima_ = tma_;
        if_ptr_->SetInterrupt(2);
        // mmu_->WriteMemory(0xFF05, tima_);
      }

      last_freq_ = false;
    }
  }
}
//
// // #include "timer.h"
// //
// // void Timer::UpdateTimer(uint8_t cycles) noexcept {
// //   tac_ = ReadMMU(0xFF07);
// //   UpdateDIV(cycles);
// //   // Is the timer enabled?
// //   if(tac_ & (1 << 2)) {
// //     UpdateTIMA(cycles);
// //   }
// // }
// //
// // void Timer::UpdateDIV(uint8_t cycles) noexcept {
// //   div_clock_ += cycles;
// //   while(div_clock_ >= 256) {
// //     ++div_;
// //     div_clock_ -= 256;
// //   }
// // }
// //
// // void Timer::UpdateTIMA(uint8_t cycles) noexcept {
// //   uint16_t clock_rate_ = frequencies_[tac_ & 0x03];
// //   tima_clock_ += cycles;
// //   if(tima_reset_pending_) {
// //     tima_ = tma_;
// //     tima_reset_pending_ = false;
// //     WriteMMU(0xFF05, tima_);
// //   }
// //   while(tima_clock_ >= clock_rate_) {
// //     tima_clock_ -= clock_rate_;
// //     if(tima_ == 0xFF) {
// //       mmu_->if_->SetInterrupt(2);
// //       tima_ = 0x00;
// //       tima_reset_pending_ = true;
// //     } else {
// //       ++tima_;
// //     }
// //     WriteMMU(0xFF05, tima_);
// //   }
// // }
