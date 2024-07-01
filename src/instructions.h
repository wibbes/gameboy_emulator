#include <array>
#include <cstdint>
#include <string>
#include <vector>

class Instruction {
public:
  uint8_t opcode_, length_, cycles_;
  std::string mnemonic_;
  Instruction(uint8_t opcode, std::string mnemonic, uint8_t length,
              uint8_t cycles)
      : opcode_(opcode), mnemonic_(mnemonic), length_(length), cycles_(cycles) {
  }
  ~Instruction() = default;
};

const static std::vector<Instruction> instructions{
    Instruction(0x00, "NOP", 0x00, 0x00)};
