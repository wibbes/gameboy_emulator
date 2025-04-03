#ifndef GAMEBOY_EMULATOR_SRC_INTERRUPTREGISTER_H_
#define GAMEBOY_EMULATOR_SRC_INTERRUPTREGISTER_H_

#include <bitset>
#include <cstdint>

class InterruptRegister {
public:
  InterruptRegister() : state_(0x0b00000) {};
  ~InterruptRegister() = default;

  void SetState(uint8_t state);
  void SetInterrupt(uint8_t interrupt);
  void ResetInterrupt(uint8_t interrupt);
  uint8_t GetState();
  uint8_t GetInterrupt(uint8_t interrupt);

private:
  std::bitset<5> state_;
};

#endif
