#ifndef GAMEBOY_EMULATOR_SRC_TIMER_H_
#define GAMEBOY_EMULATOR_SRC_TIMER_H_

#include "mmu.h"
#include <cstdint>
#include <vector>

class Timer {
public:
  uint8_t div_, tma_, tima_, tac_;
  uint16_t div_clock_, tima_clock_;
  const std::vector<uint16_t> frequencies_;
  bool tima_reset_pending_;
  std::shared_ptr<MMU> mmu_;
  Timer(std::shared_ptr<MMU> m)
      : div_(0x00), tma_(0x00), tima_(0x00), tac_(0x00), div_clock_(0x0000),
        tima_clock_(0x0000), frequencies_({1024, 16, 64, 256}),
        tima_reset_pending_(false), mmu_(m) {}
  ~Timer() = default;

  void UpdateTimer(uint8_t cycles);
  void UpdateDIV(uint8_t cycles);
  void UpdateTIMA(uint8_t cycles);
  void UpdateTAC(uint8_t value);

private:
};
#endif
