#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
class Cartridge {
public:
  std::vector<uint8_t> data;

  Cartridge() : data(LoadCartridge("../roms/ld.gb")){};
  ~Cartridge() = default;
  std::vector<uint8_t> LoadCartridge(const std::string &name);
};
