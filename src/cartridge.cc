#include "cartridge.h"

std::vector<uint8_t> Cartridge::LoadCartridge(const std::string &name) {
  std::filesystem::path file_path{name};
  auto size = std::filesystem::file_size(file_path);
  if (size == 0)
    return {};
  std::vector<uint8_t> cart(size);
  std::ifstream file(name, std::ios_base::binary);
  file.read(reinterpret_cast<char *>(cart.data()), size);
  file.close();
  return cart;
}
