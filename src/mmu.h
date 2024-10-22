#ifndef GAMEBOY_EMULATOR_SRC_MMU_H_
#define GAMEBOY_EMULATOR_SRC_MMU_H_

#include <bitset>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

class Timer {
public:
  uint8_t div_;
  uint32_t div_internal_counter;
  uint8_t tima_;
  uint32_t tima_internal_counter;
  uint8_t tma_;
  std::bitset<8> tac_;
  const std::vector<uint16_t> frequencies;
  bool tima_reset_pending_;
  Timer()
      : div_(0x0), div_internal_counter(0x0), tima_(0x0),
        tima_internal_counter(0x0), tma_(0x0), tac_(0x00),
        frequencies({1024, 16, 64, 256}), tima_reset_pending_(false) {};
  ~Timer() = default;

private:
};

class InterruptRegister {
public:
  InterruptRegister() : state_(0x0b11111) {};
  ~InterruptRegister() = default;
  std::bitset<5> state_;

  void SetState(uint8_t state);
  void SetInterrupt(uint8_t interrupt);
  void ResetInterrupt(uint8_t interrupt);
  uint8_t GetState();
  uint8_t GetInterrupt(uint8_t interrupt);

private:
};

class MMU {
public:
  std::vector<uint8_t> memory_;
  std::unique_ptr<InterruptRegister> ie_, if_;
  std::unique_ptr<Timer> timer_;
  MMU(std::vector<uint8_t> *cart)
      : memory_(*cart), ie_(std::make_unique<InterruptRegister>()),
        if_(std::make_unique<InterruptRegister>()),
        timer_(std::make_unique<Timer>()) {}
  ~MMU() = default;

  void WriteMemory(uint16_t address, uint8_t value);
  uint8_t ReadMemory(uint16_t address);
};

#endif
