#ifndef GAMEBOY_EMULATOR_SRC_TIMER_H_
#define GAMEBOY_EMULATOR_SRC_TIMER_H_

#include "mmu.h"
#include <array>
#include <cstdint>

class Timer {
public:
  explicit Timer(std::shared_ptr<MMU> m) noexcept
      : div_(0x00), tma_(0x00), tima_(0x00), tac_(0x00), div_clock_(0x0000),
        tima_clock_(0x0000), frequencies_({kClockRate1024, kClockRate16,
                                           kClockRate64, kClockRate256}),
        tima_reset_pending_(false), mmu_(m){};
  ~Timer() = default;

  void UpdateTimer(uint8_t cycles) noexcept;
  void UpdateDIV(uint8_t cycles) noexcept;
  void UpdateTIMA(uint8_t cycles) noexcept;

private:
  static constexpr uint16_t kClockRate1024 = 1024;
  static constexpr uint16_t kClockRate16 = 16;
  static constexpr uint16_t kClockRate64 = 64;
  static constexpr uint16_t kClockRate256 = 256;

  uint8_t div_, tma_, tima_, tac_;
  uint16_t div_clock_, tima_clock_;
  const std::array<uint16_t, 4> frequencies_;
  bool tima_reset_pending_;
  std::shared_ptr<MMU> mmu_;
};
#endif
