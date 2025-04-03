#ifndef GAMEBOY_EMULATOR_SRC_MMU_H_
#define GAMEBOY_EMULATOR_SRC_MMU_H_

#include <bitset>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "cartridge.h"
#include "timer.h"
#include "interruptregister.h"

class MMU {
public:
  std::unique_ptr<InterruptRegister> ie_, if_;
  MMU(std::shared_ptr<Cartridge> cart, std::shared_ptr<Timer> timer)
      : memory_(cart->data), timer_(timer), ie_(std::make_unique<InterruptRegister>()),
        if_(std::make_unique<InterruptRegister>()) {
          timer_->LinkIF(if_.get());
        }
  ~MMU() = default;

  void WriteMemory(uint16_t address, uint8_t value);
  uint8_t ReadMemory(uint16_t address);

private:
  std::vector<uint8_t> memory_;
  std::shared_ptr<Timer> timer_;
};

#endif
