#ifndef GAMEBOY_EMULATOR_SRC_CARTRIDGE_H_
#define GAMEBOY_EMULATOR_SRC_CARTRIDGE_H_

#include <fstream>
#include <vector>
#include <filesystem>

class Cartridge {
public:
  std::vector<uint8_t> data;

  Cartridge() : data(LoadCartridge("../roms/cpu_instrs/individual/02-interrupts.gb")){};
  ~Cartridge() = default;
  std::vector<uint8_t> LoadCartridge(const std::string &name);
};

#endif
