#ifndef GAMEBOY_EMULATOR_SRC_TIMER_H_
#define GAMEBOY_EMULATOR_SRC_TIMER_H_
#include <cstdint>
#include <vector>

#include "interruptregister.h"
#include <memory>

class Timer {
public:
  explicit Timer() noexcept
      : tac_(0x00), div_(0x00), tima_(0x00), tma_(0x00),
        tima_overflow_ticks_(0x00), tima_overflowed_(false),
        timer_enabled_(false), current_frequency_(1 << 9), last_freq_(false),
        if_ptr_(nullptr) {};
  void Tick();
  uint8_t Read(uint16_t address);
  void Write(uint16_t address, uint8_t value);
  void TIMABug(bool old_enable, uint16_t old_freq);
  void LinkIF(InterruptRegister *if_ptr) { if_ptr_ = if_ptr; };

private:
  uint8_t tac_;
  uint8_t div_;
  uint8_t tima_;
  uint8_t tma_;
  uint8_t tima_overflow_ticks_;
  uint16_t current_frequency_;

  bool tima_overflowed_;
  bool timer_enabled_;
  bool last_freq_;

  InterruptRegister *if_ptr_;

  std::vector<uint16_t> frequencies_ = {9, 3, 5, 7};

  static constexpr uint8_t kTACMask = 0xF8; // zero last 3 bits
};

#endif

//
// #include "mmu.h"
// #include <array>
// #include <cstdint>
//
// class Timer {
// public:
//   explicit Timer(std::shared_ptr<MMU> m) noexcept
//       : div_(0x00), tma_(0x00), tima_(0x00), tac_(0x00), div_clock_(0x0000),
//         tima_clock_(0x0000), frequencies_({kClockRate1024, kClockRate16,
//                                            kClockRate64, kClockRate256}),
//         tima_reset_pending_(false), mmu_(m){};
//   ~Timer() = default;
//
//   void UpdateTimer(uint8_t cycles) noexcept;
//   void UpdateDIV(uint8_t cycles) noexcept;
//   void UpdateTIMA(uint8_t cycles) noexcept;
//
// private:
//   static constexpr uint16_t kClockRate1024 = 1024;
//   static constexpr uint16_t kClockRate16 = 16;
//   static constexpr uint16_t kClockRate64 = 64;
//   static constexpr uint16_t kClockRate256 = 256;
//
//   uint8_t div_, tma_, tima_, tac_;
//   uint16_t div_clock_, tima_clock_;
//   const std::array<uint16_t, 4> frequencies_;
//   bool tima_reset_pending_;
//   std::shared_ptr<MMU> mmu_;
// };
//
//
