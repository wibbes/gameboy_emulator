#ifndef GAMEBOY_EMULATOR_SRC_MMU_H_
#define GAMEBOY_EMULATOR_SRC_MMU_H_

#include <bitset>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "cartridge.h"

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

class MMU {
public:
  std::unique_ptr<InterruptRegister> ie_, if_;
  MMU(std::shared_ptr<Cartridge> cart)
      : memory_(cart->data), ie_(std::make_unique<InterruptRegister>()),
        if_(std::make_unique<InterruptRegister>()) {}
  ~MMU() = default;
  
  void WriteMemory(uint16_t address, uint8_t value);
  uint8_t ReadMemory(uint16_t address);
private:
  std::vector<uint8_t> memory_;
};

#endif
