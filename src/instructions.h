#ifndef GAMEBOY_EMULATOR_SRC_INSTRUCTION_H_
#define GAMEBOY_EMULATOR_SRC_INSTRUCTION_H_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

class Instruction {
public:
  uint8_t opcode_, length_, cycles_;
  std::string mnemonic_;
  Instruction(uint8_t opcode, std::string &&mnemonic, uint8_t length,
              uint8_t cycles)
      : opcode_(opcode), mnemonic_(std::move(mnemonic)), length_(length),
        cycles_(cycles) {}
  ~Instruction() = default;
};

// Cycles are in M-states
const static std::vector<Instruction> instructions{
    Instruction(0x00, "NOP", 1, 1),
    Instruction(0x01, "LD BC,d16", 3, 3),
    Instruction(0x02, "LD (BC),A", 1, 2),
    Instruction(0x03, "INC BC", 1, 2),
    Instruction(0x04, "INC B", 1, 1),
    Instruction(0x05, "DEC B", 1, 1),
    Instruction(0x06, "LD B,d8", 2, 2),
    Instruction(0x07, "RLCA", 1, 1),
    Instruction(0x08, "LD (a16),SP", 3, 5),
    Instruction(0x09, "ADD HL,BC", 1, 2),
    Instruction(0x0A, "LD A,(BC)", 1, 2),
    Instruction(0x0B, "DEC BC", 1, 2),
    Instruction(0x0C, "INC C", 1, 1),
    Instruction(0x0D, "DEC C", 1, 1),
    Instruction(0x0E, "LD C,d8", 2, 2),
    Instruction(0x0F, "RRCA", 1, 1),
    Instruction(0x10, "STOP 0", 2, 1),
    Instruction(0x11, "LD DE,d16", 3, 3),
    Instruction(0x12, "LD (DE),A", 1, 2),
    Instruction(0x13, "INC DE", 1, 2),
    Instruction(0x14, "INC D", 1, 1),
    Instruction(0x15, "DEC D", 1, 1),
    Instruction(0x16, "LD D,d8", 2, 2),
    Instruction(0x17, "RLA", 1, 1),
    Instruction(0x18, "JR r8", 2, 2),
    Instruction(0x19, "ADD HL,DE", 1, 2),
    Instruction(0x1A, "LD A,(DE)", 1, 2),
    Instruction(0x1B, "DEC DE", 1, 2),
    Instruction(0x1C, "INC E", 1, 1),
    Instruction(0x1D, "DEC E", 1, 1),
    Instruction(0x1E, "LD E,d8", 2, 2),
    Instruction(0x1F, "RRA", 1, 1),
    Instruction(0x20, "JR NZ,r8", 2, 2),
    Instruction(0x21, "LD HL,d16", 3, 3),
    Instruction(0x22, "LD (HL+),A", 1, 2),
    Instruction(0x23, "INC HL", 1, 2),
    Instruction(0x24, "INC H", 1, 1),
    Instruction(0x25, "DEC H", 1, 1),
    Instruction(0x26, "LD H,d8", 2, 2),
    Instruction(0x27, "DAA", 1, 1),
    Instruction(0x28, "JR Z,r8", 2, 2),
    Instruction(0x29, "ADD HL,HL", 1, 2),
    Instruction(0x2A, "LD A,(HL+)", 1, 2),
    Instruction(0x2B, "DEC HL", 1, 2),
    Instruction(0x2C, "INC L", 1, 1),
    Instruction(0x2D, "DEC L", 1, 1),
    Instruction(0x2E, "LD L,d8", 2, 2),
    Instruction(0x2F, "CPL", 1, 1),
    Instruction(0x30, "JR NC,r8", 2, 2),
    Instruction(0x31, "LD SP,d16", 3, 3),
    Instruction(0x32, "LD (HL-),A", 1, 2),
    Instruction(0x33, "INC SP", 1, 2),
    Instruction(0x34, "INC (HL)", 1, 3),
    Instruction(0x35, "DEC (HL)", 1, 3),
    Instruction(0x36, "LD (HL),d8", 2, 3),
    Instruction(0x37, "SCF", 1, 1),
    Instruction(0x38, "JR C,r8", 2, 2),
    Instruction(0x39, "ADD HL,SP", 1, 2),
    Instruction(0x3A, "LD A,(HL-)", 1, 2),
    Instruction(0x3B, "DEC SP", 1, 2),
    Instruction(0x3C, "INC A", 1, 1),
    Instruction(0x3D, "DEC A", 1, 1),
    Instruction(0x3E, "LD A,d8", 2, 2),
    Instruction(0x3F, "CCF", 1, 1),
    Instruction(0x40, "LD B,B", 1, 1),
    Instruction(0x41, "LD B,C", 1, 1),
    Instruction(0x42, "LD B,D", 1, 1),
    Instruction(0x43, "LD B,E", 1, 1),
    Instruction(0x44, "LD B,H", 1, 1),
    Instruction(0x45, "LD B,L", 1, 1),
    Instruction(0x46, "LD B,(HL)", 1, 2),
    Instruction(0x47, "LD B,A", 1, 1),
    Instruction(0x48, "LD C,B", 1, 1),
    Instruction(0x49, "LD C,C", 1, 1),
    Instruction(0x4A, "LD C,D", 1, 1),
    Instruction(0x4B, "LD C,E", 1, 1),
    Instruction(0x4C, "LD C,H", 1, 1),
    Instruction(0x4D, "LD C,L", 1, 1),
    Instruction(0x4E, "LD C,(HL)", 1, 2),
    Instruction(0x4F, "LD C,A", 1, 1),
    Instruction(0x50, "LD D,B", 1, 1),
    Instruction(0x51, "LD D,C", 1, 1),
    Instruction(0x52, "LD D,D", 1, 1),
    Instruction(0x53, "LD D,E", 1, 1),
    Instruction(0x54, "LD D,H", 1, 1),
    Instruction(0x55, "LD D,L", 1, 1),
    Instruction(0x56, "LD D,(HL)", 1, 2),
    Instruction(0x57, "LD D,A", 1, 1),
    Instruction(0x58, "LD E,B", 1, 1),
    Instruction(0x59, "LD E,C", 1, 1),
    Instruction(0x5A, "LD E,D", 1, 1),
    Instruction(0x5B, "LD E,E", 1, 1),
    Instruction(0x5C, "LD E,H", 1, 1),
    Instruction(0x5D, "LD E,L", 1, 1),
    Instruction(0x5E, "LD E,(HL)", 1, 2),
    Instruction(0x5F, "LD E,A", 1, 1),
    Instruction(0x60, "LD H,B", 1, 1),
    Instruction(0x61, "LD H,C", 1, 1),
    Instruction(0x62, "LD H,D", 1, 1),
    Instruction(0x63, "LD H,E", 1, 1),
    Instruction(0x64, "LD H,H", 1, 1),
    Instruction(0x65, "LD H,L", 1, 1),
    Instruction(0x66, "LD H,(HL)", 1, 2),
    Instruction(0x67, "LD H,A", 1, 1),
    Instruction(0x68, "LD L,B", 1, 1),
    Instruction(0x69, "LD L,C", 1, 1),
    Instruction(0x6A, "LD L,D", 1, 1),
    Instruction(0x6B, "LD L,E", 1, 1),
    Instruction(0x6C, "LD L,H", 1, 1),
    Instruction(0x6D, "LD L,L", 1, 1),
    Instruction(0x6E, "LD L,(HL)", 1, 2),
    Instruction(0x6F, "LD L,A", 1, 1),
    Instruction(0x70, "LD (HL),B", 1, 2),
    Instruction(0x71, "LD (HL),C", 1, 2),
    Instruction(0x72, "LD (HL),D", 1, 2),
    Instruction(0x73, "LD (HL),E", 1, 2),
    Instruction(0x74, "LD (HL),H", 1, 2),
    Instruction(0x75, "LD (HL),L", 1, 2),
    Instruction(0x76, "HALT", 1, 1),
    Instruction(0x77, "LD (HL),A", 1, 2),
    Instruction(0x78, "LD A,B", 1, 1),
    Instruction(0x79, "LD A,C", 1, 1),
    Instruction(0x7A, "LD A,D", 1, 1),
    Instruction(0x7B, "LD A,E", 1, 1),
    Instruction(0x7C, "LD A,H", 1, 1),
    Instruction(0x7D, "LD A,L", 1, 1),
    Instruction(0x7E, "LD A,(HL)", 1, 2),
    Instruction(0x7F, "LD A,A", 1, 1),
    Instruction(0x80, "ADD A,B", 1, 1),
    Instruction(0x81, "ADD A,C", 1, 1),
    Instruction(0x82, "ADD A,D", 1, 1),
    Instruction(0x83, "ADD A,E", 1, 1),
    Instruction(0x84, "ADD A,H", 1, 1),
    Instruction(0x85, "ADD A,L", 1, 1),
    Instruction(0x86, "ADD A,(HL)", 1, 2),
    Instruction(0x87, "ADD A,A", 1, 1),
    Instruction(0x88, "ADC A,B", 1, 1),
    Instruction(0x89, "ADC A,C", 1, 1),
    Instruction(0x8A, "ADC A,D", 1, 1),
    Instruction(0x8B, "ADC A,E", 1, 1),
    Instruction(0x8C, "ADC A,H", 1, 1),
    Instruction(0x8D, "ADC A,L", 1, 1),
    Instruction(0x8E, "ADC A,(HL)", 1, 2),
    Instruction(0x8F, "ADC A,A", 1, 1),
    Instruction(0x90, "SUB B", 1, 1),
    Instruction(0x91, "SUB C", 1, 1),
    Instruction(0x92, "SUB D", 1, 1),
    Instruction(0x93, "SUB E", 1, 1),
    Instruction(0x94, "SUB H", 1, 1),
    Instruction(0x95, "SUB L", 1, 1),
    Instruction(0x96, "SUB (HL)", 1, 2),
    Instruction(0x97, "SUB A", 1, 1),
    Instruction(0x98, "SBC A,B", 1, 1),
    Instruction(0x99, "SBC A,C", 1, 1),
    Instruction(0x9A, "SBC A,D", 1, 1),
    Instruction(0x9B, "SBC A,E", 1, 1),
    Instruction(0x9C, "SBC A,H", 1, 1),
    Instruction(0x9D, "SBC A,L", 1, 1),
    Instruction(0x9E, "SBC A,(HL)", 1, 2),
    Instruction(0x9F, "SBC A,A", 1, 1),
    Instruction(0xA0, "AND B", 1, 1),
    Instruction(0xA1, "AND C", 1, 1),
    Instruction(0xA2, "AND D", 1, 1),
    Instruction(0xA3, "AND E", 1, 1),
    Instruction(0xA4, "AND H", 1, 1),
    Instruction(0xA5, "AND L", 1, 1),
    Instruction(0xA6, "AND (HL)", 1, 2),
    Instruction(0xA7, "AND A", 1, 1),
    Instruction(0xA8, "XOR B", 1, 1),
    Instruction(0xA9, "XOR C", 1, 1),
    Instruction(0xAA, "XOR D", 1, 1),
    Instruction(0xAB, "XOR E", 1, 1),
    Instruction(0xAC, "XOR H", 1, 1),
    Instruction(0xAD, "XOR L", 1, 1),
    Instruction(0xAE, "XOR (HL)", 1, 2),
    Instruction(0xAF, "XOR A", 1, 1),
    Instruction(0xB0, "OR B", 1, 1),
    Instruction(0xB1, "OR C", 1, 1),
    Instruction(0xB2, "OR D", 1, 1),
    Instruction(0xB3, "OR E", 1, 1),
    Instruction(0xB4, "OR H", 1, 1),
    Instruction(0xB5, "OR L", 1, 1),
    Instruction(0xB6, "OR (HL)", 1, 2),
    Instruction(0xB7, "OR A", 1, 1),
    Instruction(0xB8, "CP B", 1, 1),
    Instruction(0xB9, "CP C", 1, 1),
    Instruction(0xBA, "CP D", 1, 1),
    Instruction(0xBB, "CP E", 1, 1),
    Instruction(0xBC, "CP H", 1, 1),
    Instruction(0xBD, "CP L", 1, 1),
    Instruction(0xBE, "CP (HL)", 1, 2),
    Instruction(0xBF, "CP A", 1, 1),
    Instruction(0xC0, "RET NZ", 1, 5),
    Instruction(0xC1, "POP BC", 1, 3),
    Instruction(0xC2, "JP NZ,a16", 3, 4),
    Instruction(0xC3, "JP a16", 3, 4),
    Instruction(0xC4, "CALL NZ,a16", 3, 6),
    Instruction(0xC5, "PUSH BC", 1, 4),
    Instruction(0xC6, "ADD A,d8", 2, 2),
    Instruction(0xC7, "RST 00H", 1, 4),
    Instruction(0xC8, "RET Z", 1, 5),
    Instruction(0xC9, "RET", 1, 4),
    Instruction(0xCA, "JP Z,a16", 3, 4),
    Instruction(0xCB, "PREFIX CB", 2, 2),
    Instruction(0xCC, "CALL Z,a16", 3, 6),
    Instruction(0xCD, "CALL a16", 3, 6),
    Instruction(0xCE, "ADC A,d8", 2, 2),
    Instruction(0xCF, "RST 08H", 1, 4),
    Instruction(0xD0, "RET NC", 1, 5),
    Instruction(0xD1, "POP DE", 1, 3),
    Instruction(0xD2, "JP NC,a16", 3, 4),
    Instruction(0xD3, "OUT (C),A", 1, 2),
    Instruction(0xD4, "CALL NC,a16", 3, 6),
    Instruction(0xD5, "PUSH DE", 1, 4),
    Instruction(0xD6, "SUB d8", 2, 2),
    Instruction(0xD7, "RST 10H", 1, 4),
    Instruction(0xD8, "RET C", 1, 5),
    Instruction(0xD9, "RETI", 1, 4),
    Instruction(0xDA, "JP C,a16", 3, 4),
    Instruction(0xDB, "IN A,(C)", 1, 2),
    Instruction(0xDC, "CALL C,a16", 3, 6),
    Instruction(0xDD, "PREFIX DD", 2, 2),
    Instruction(0xDE, "SBC A,d8", 2, 2),
    Instruction(0xDF, "RST 18H", 1, 4),
    Instruction(0xE0, "LD (a16),A", 3, 5),
    Instruction(0xE1, "POP HL", 1, 3),
    Instruction(0xE2, "LD (C),A", 1, 2),
    Instruction(0xE3, "EX (SP),HL", 1, 2),
    Instruction(0xE4, "LD (a16),A", 3, 5),
    Instruction(0xE5, "PUSH HL", 1, 4),
    Instruction(0xE6, "AND d8", 2, 2),
    Instruction(0xE7, "RST 20H", 1, 4),
    Instruction(0xE8, "ADD SP,d8", 2, 4),
    Instruction(0xE9, "JP (HL)", 1, 1),
    Instruction(0xEA, "LD (a16),A", 3, 5),
    Instruction(0xEB, "NOP", 1, 1),
    Instruction(0xEC, "LD (a16),A", 3, 5),
    Instruction(0xED, "PREFIX ED", 2, 2),
    Instruction(0xEE, "XOR d8", 2, 2),
    Instruction(0xEF, "RST 28H", 1, 4),
    Instruction(0xF0, "LD A,(a16)", 3, 5),
    Instruction(0xF1, "POP AF", 1, 3),
    Instruction(0xF2, "LD A,(C)", 1, 2),
    Instruction(0xF3, "DI", 1, 1),
    Instruction(0xF4, "LD A,(a16)", 3, 5),
    Instruction(0xF5, "PUSH AF", 1, 4),
    Instruction(0xF6, "OR d8", 2, 2),
    Instruction(0xF7, "RST 30H", 1, 4),
    Instruction(0xF8, "LD HL,SP+d8", 2, 4),
    Instruction(0xF9, "LD SP,HL", 1, 2),
    Instruction(0xFA, "LD A,(a16)", 3, 5),
    Instruction(0xFB, "EI", 1, 1),
    Instruction(0xFC, "LD A,(a16)", 3, 5),
    Instruction(0xFD, "PREFIX FD", 2, 2),
    Instruction(0xFE, "CP d8", 2, 2),
    Instruction(0xFF, "RST 38H", 1, 4)

    /*Instruction(0x00, "NOP", 1, 1),*/
    /*Instruction(0x01, "LD BC,d16", 3, 3),*/
    /*Instruction(0x02, "LD (BC),A", 1, 2),*/
    /*Instruction(0x03, "INC BC", 1, 2),*/
    /*Instruction(0x04, "INC B", 1, 1),*/
    /*Instruction(0x05, "DEC B", 1, 1),*/
    /*Instruction(0x06, "LD B,d8", 2, 2),*/
    /*Instruction(0x07, "RLCA", 1, 1),*/
    /*Instruction(0x08, "LD (a16),SP", 3, 5),*/
    /*Instruction(0x09, "ADD HL,BC", 1, 2),*/
    /*Instruction(0x0A, "LD A,(BC)", 1, 2),*/
    /*Instruction(0x0B, "DEC BC", 1, 2),*/
    /*Instruction(0x0C, "INC C", 1, 1),*/
    /*Instruction(0x0D, "DEC C", 1, 1),*/
    /*Instruction(0x0E, "LD C,d8", 2, 2),*/
    /*Instruction(0x0F, "RRCA", 1, 1),*/
    /*Instruction(0x10, "STOP 0", 2, 1),*/
    /*Instruction(0x11, "LD DE,d16", 3, 1),*/
    /*Instruction(0x12, "LD (DE),A", 1, 1),*/
    /*Instruction(0x13, "INC DE", 1, 1),*/
    /*Instruction(0x14, "INC D", 1, 1),*/
    /*Instruction(0x15, "DEC D", 1, 1),*/
    /*Instruction(0x16, "LD D,d8", 2, 1),*/
    /*Instruction(0x17, "RLA", 1, 1),*/
    /*Instruction(0x18, "JR r8", 2, 1),*/
    /*Instruction(0x19, "ADD HL,DE", 1, 1),*/
    /*Instruction(0x1A, "LD A,(DE)", 1, 1),*/
    /*Instruction(0x1B, "DEC DE", 1, 1),*/
    /*Instruction(0x1C, "INC E", 1, 1),*/
    /*Instruction(0x1D, "DEC E", 1, 1),*/
    /*Instruction(0x1E, "LD E,d8", 2, 1),*/
    /*Instruction(0x1F, "RRA", 1, 1),*/
    /*Instruction(0x20, "JR NZ,r8", 2, 1),*/
    /*Instruction(0x21, "LD HL,d16", 3, 1),*/
    /*Instruction(0x22, "LD (HL+),A", 1, 1),*/
    /*Instruction(0x23, "INC HL", 1, 1),*/
    /*Instruction(0x24, "INC H", 1, 1),*/
    /*Instruction(0x25, "DEC H", 1, 1),*/
    /*Instruction(0x26, "LD H,d8", 2, 1),*/
    /*Instruction(0x27, "DAA", 1, 1),*/
    /*Instruction(0x28, "JR Z,r8", 2, 1),*/
    /*Instruction(0x29, "ADD HL,HL", 1, 1),*/
    /*Instruction(0x2A, "LD A,(HL+)", 1, 1),*/
    /*Instruction(0x2B, "DEC HL", 1, 1),*/
    /*Instruction(0x2C, "INC L", 1, 1),*/
    /*Instruction(0x2D, "DEC L", 1, 1),*/
    /*Instruction(0x2E, "LD L,d8", 2, 1),*/
    /*Instruction(0x2F, "CPL", 1, 1),*/
    /*Instruction(0x30, "JR NC,r8", 2, 1),*/
    /*Instruction(0x31, "LD SP,d16", 3, 1),*/
    /*Instruction(0x32, "LD (HL-),A", 1, 1),*/
    /*Instruction(0x33, "INC SP", 1, 1),*/
    /*Instruction(0x34, "INC (HL)", 1, 1),*/
    /*Instruction(0x35, "DEC (HL)", 1, 1),*/
    /*Instruction(0x36, "LD (HL),d8", 2, 1),*/
    /*Instruction(0x37, "SCF", 1, 1),*/
    /*Instruction(0x38, "JR C,r8", 2, 1),*/
    /*Instruction(0x39, "ADD HL,SP", 1, 1),*/
    /*Instruction(0x3A, "LD A,(HL-)", 1, 1),*/
    /*Instruction(0x3B, "DEC SP", 1, 1),*/
    /*Instruction(0x3C, "INC A", 1, 1),*/
    /*Instruction(0x3D, "DEC A", 1, 1),*/
    /*Instruction(0x3E, "LD A,d8", 2, 1),*/
    /*Instruction(0x3F, "CCF", 1, 1),*/
    /*Instruction(0x40, "LD B,B", 1, 1),*/
    /*Instruction(0x41, "LD B,C", 1, 1),*/
    /*Instruction(0x42, "LD B,D", 1, 1),*/
    /*Instruction(0x43, "LD B,E", 1, 1),*/
    /*Instruction(0x44, "LD B,H", 1, 1),*/
    /*Instruction(0x45, "LD B,L", 1, 1),*/
    /*Instruction(0x46, "LD B,(HL)", 1, 1),*/
    /*Instruction(0x47, "LD B,A", 1, 1),*/
    /*Instruction(0x48, "LD C,B", 1, 1),*/
    /*Instruction(0x49, "LD C,C", 1, 1),*/
    /*Instruction(0x4A, "LD C,D", 1, 1),*/
    /*Instruction(0x4B, "LD C,E", 1, 1),*/
    /*Instruction(0x4C, "LD C,H", 1, 1),*/
    /*Instruction(0x4D, "LD C,L", 1, 1),*/
    /*Instruction(0x4E, "LD C,(HL)", 1, 1),*/
    /*Instruction(0x4F, "LD C,A", 1, 1),*/
    /*Instruction(0x50, "LD D,B", 1, 1),*/
    /*Instruction(0x51, "LD D,C", 1, 1),*/
    /*Instruction(0x52, "LD D,D", 1, 1),*/
    /*Instruction(0x53, "LD D,E", 1, 1),*/
    /*Instruction(0x54, "LD D,H", 1, 1),*/
    /*Instruction(0x55, "LD D,L", 1, 1),*/
    /*Instruction(0x56, "LD D,(HL)", 1, 1),*/
    /*Instruction(0x57, "LD D,A", 1, 1),*/
    /*Instruction(0x58, "LD E,B", 1, 1),*/
    /*Instruction(0x59, "LD E,C", 1, 1),*/
    /*Instruction(0x5A, "LD E,D", 1, 1),*/
    /*Instruction(0x5B, "LD E,E", 1, 1),*/
    /*Instruction(0x5C, "LD E,H", 1, 1),*/
    /*Instruction(0x5D, "LD E,L", 1, 1),*/
    /*Instruction(0x5E, "LD E,(HL)", 1, 1),*/
    /*Instruction(0x5F, "LD E,A", 1, 1),*/
    /*Instruction(0x60, "LD H,B", 1, 1),*/
    /*Instruction(0x61, "LD H,C", 1, 1),*/
    /*Instruction(0x62, "LD H,D", 1, 1),*/
    /*Instruction(0x63, "LD H,E", 1, 1),*/
    /*Instruction(0x64, "LD H,H", 1, 1),*/
    /*Instruction(0x65, "LD H,L", 1, 1),*/
    /*Instruction(0x66, "LD H,(HL)", 1, 1),*/
    /*Instruction(0x67, "LD H,A", 1, 1),*/
    /*Instruction(0x68, "LD L,B", 1, 1),*/
    /*Instruction(0x69, "LD L,C", 1, 1),*/
    /*Instruction(0x6A, "LD L,D", 1, 1),*/
    /*Instruction(0x6B, "LD L,E", 1, 1),*/
    /*Instruction(0x6C, "LD L,H", 1, 1),*/
    /*Instruction(0x6D, "LD L,L", 1, 1),*/
    /*Instruction(0x6E, "LD L,(HL)", 1, 1),*/
    /*Instruction(0x6F, "LD L,A", 1, 1),*/
    /*Instruction(0x70, "LD (HL),B", 1, 1),*/
    /*Instruction(0x71, "LD (HL),C", 1, 1),*/
    /*Instruction(0x72, "LD (HL),D", 1, 1),*/
    /*Instruction(0x73, "LD (HL),E", 1, 1),*/
    /*Instruction(0x74, "LD (HL),H", 1, 1),*/
    /*Instruction(0x75, "LD (HL),L", 1, 1),*/
    /*Instruction(0x76, "HALT", 1, 1),*/
    /*Instruction(0x77, "LD (HL),A", 1, 1),*/
    /*Instruction(0x78, "LD A,B", 1, 1),*/
    /*Instruction(0x79, "LD A,C", 1, 1),*/
    /*Instruction(0x7A, "LD A,D", 1, 1),*/
    /*Instruction(0x7B, "LD A,E", 1, 1),*/
    /*Instruction(0x7C, "LD A,H", 1, 1),*/
    /*Instruction(0x7D, "LD A,L", 1, 1),*/
    /*Instruction(0x7E, "LD A,(HL)", 1, 1),*/
    /*Instruction(0x7F, "LD A,A", 1, 1),*/
    /*Instruction(0x80, "ADD A,B", 1, 1),*/
    /*Instruction(0x81, "ADD A,C", 1, 1),*/
    /*Instruction(0x82, "ADD A,D", 1, 1),*/
    /*Instruction(0x83, "ADD A,E", 1, 1),*/
    /*Instruction(0x84, "ADD A,H", 1, 1),*/
    /*Instruction(0x85, "ADD A,L", 1, 1),*/
    /*Instruction(0x86, "ADD A,(HL)", 1, 1),*/
    /*Instruction(0x87, "ADD A,A", 1, 1),*/
    /*Instruction(0x88, "ADC A,B", 1, 1),*/
    /*Instruction(0x89, "ADC A,C", 1, 1),*/
    /*Instruction(0x8A, "ADC A,D", 1, 1),*/
    /*Instruction(0x8B, "ADC A,E", 1, 1),*/
    /*Instruction(0x8C, "ADC A,H", 1, 1),*/
    /*Instruction(0x8D, "ADC A,L", 1, 1),*/
    /*Instruction(0x8E, "ADC A,(HL)", 1, 1),*/
    /*Instruction(0x8F, "ADC A,A", 1, 1),*/
    /*Instruction(0x90, "SUB B", 1, 1),*/
    /*Instruction(0x91, "SUB C", 1, 1),*/
    /*Instruction(0x92, "SUB D", 1, 1),*/
    /*Instruction(0x93, "SUB E", 1, 1),*/
    /*Instruction(0x94, "SUB H", 1, 1),*/
    /*Instruction(0x95, "SUB L", 1, 1),*/
    /*Instruction(0x96, "SUB (HL)", 1, 1),*/
    /*Instruction(0x97, "SUB A", 1, 1),*/
    /*Instruction(0x98, "SBC A,B", 1, 1),*/
    /*Instruction(0x99, "SBC A,C", 1, 1),*/
    /*Instruction(0x9A, "SBC A,D", 1, 1),*/
    /*Instruction(0x9B, "SBC A,E", 1, 1),*/
    /*Instruction(0x9C, "SBC A,H", 1, 1),*/
    /*Instruction(0x9D, "SBC A,L", 1, 1),*/
    /*Instruction(0x9E, "SBC A,(HL)", 1, 1),*/
    /*Instruction(0x9F, "SBC A,A", 1, 1),*/
    /*Instruction(0xA0, "AND B", 1, 1),*/
    /*Instruction(0xA1, "AND C", 1, 1),*/
    /*Instruction(0xA2, "AND D", 1, 1),*/
    /*Instruction(0xA3, "AND E", 1, 1),*/
    /*Instruction(0xA4, "AND H", 1, 1),*/
    /*Instruction(0xA5, "AND L", 1, 1),*/
    /*Instruction(0xA6, "AND (HL)", 1, 1),*/
    /*Instruction(0xA7, "AND A", 1, 1),*/
    /*Instruction(0xA8, "XOR B", 1, 1),*/
    /*Instruction(0xA9, "XOR C", 1, 1),*/
    /*Instruction(0xAA, "XOR D", 1, 1),*/
    /*Instruction(0xAB, "XOR E", 1, 1),*/
    /*Instruction(0xAC, "XOR H", 1, 1),*/
    /*Instruction(0xAD, "XOR L", 1, 1),*/
    /*Instruction(0xAE, "XOR (HL)", 1, 1),*/
    /*Instruction(0xAF, "XOR A", 1, 1),*/
    /*Instruction(0xB0, "OR B", 1, 1),*/
    /*Instruction(0xB1, "OR C", 1, 1),*/
    /*Instruction(0xB2, "OR D", 1, 1),*/
    /*Instruction(0xB3, "OR E", 1, 1),*/
    /*Instruction(0xB4, "OR H", 1, 1),*/
    /*Instruction(0xB5, "OR L", 1, 1),*/
    /*Instruction(0xB6, "OR (HL)", 1, 1),*/
    /*Instruction(0xB7, "OR A", 1, 1),*/
    /*Instruction(0xB8, "CP B", 1, 1),*/
    /*Instruction(0xB9, "CP C", 1, 1),*/
    /*Instruction(0xBA, "CP D", 1, 1),*/
    /*Instruction(0xBB, "CP E", 1, 1),*/
    /*Instruction(0xBC, "CP H", 1, 1),*/
    /*Instruction(0xBD, "CP L", 1, 1),*/
    /*Instruction(0xBE, "CP (HL)", 1, 1),*/
    /*Instruction(0xBF, "CP A", 1, 1),*/
    /*Instruction(0xC0, "RET NZ", 1, 1),*/
    /*Instruction(0xC1, "POP BC", 1, 1),*/
    /*Instruction(0xC2, "JP NZ,a16", 3, 1),*/
    /*Instruction(0xC3, "JP a16", 3, 1),*/
    /*Instruction(0xC4, "CALL NZ,a16", 3, 1),*/
    /*Instruction(0xC5, "PUSH BC", 1, 1),*/
    /*Instruction(0xC6, "ADD A,d8", 2, 1),*/
    /*Instruction(0xC7, "RST 00H", 1, 1),*/
    /*Instruction(0xC8, "RET Z", 1, 1),*/
    /*Instruction(0xC9, "RET", 1, 1),*/
    /*Instruction(0xCA, "JP Z,a16", 3, 1),*/
    /*Instruction(0xCB, "NONE", 1, 1),*/
    /*Instruction(0xCC, "CALL Z,a16", 3, 1),*/
    /*Instruction(0xCD, "CALL a16", 3, 1),*/
    /*Instruction(0xCE, "ADC A,d8", 2, 1),*/
    /*Instruction(0xCF, "RST 08H", 1, 1),*/
    /*Instruction(0xD0, "RET NC", 1, 1),*/
    /*Instruction(0xD1, "POP DE", 1, 1),*/
    /*Instruction(0xD2, "JP NC,a16", 3, 1),*/
    /*Instruction(0xD3, "NONE", 1, 1),*/
    /*Instruction(0xD4, "CALL NC,a16", 3, 1),*/
    /*Instruction(0xD5, "PUSH DE", 1, 1),*/
    /*Instruction(0xD6, "SUB d8", 2, 1),*/
    /*Instruction(0xD7, "RST 10H", 1, 1),*/
    /*Instruction(0xD8, "RET C", 1, 1),*/
    /*Instruction(0xD9, "RETI", 1, 1),*/
    /*Instruction(0xDA, "JP C,a16", 3, 1),*/
    /*Instruction(0xDB, "NONE", 1, 1),*/
    /*Instruction(0xDC, "CALL C,a16", 3, 1),*/
    /*Instruction(0xDD, "NONE", 1, 1),*/
    /*Instruction(0xDE, "SBC A,d8", 2, 1),*/
    /*Instruction(0xDF, "RST 18H", 1, 1),*/
    /*Instruction(0xE0, "LDH (a8),A", 2, 1),*/
    /*Instruction(0xE1, "POP HL", 1, 1),*/
    /*Instruction(0xE2, "LD (C),A", 1, 1),*/
    /*Instruction(0xE3, "NONE", 1, 1),*/
    /*Instruction(0xE4, "NONE", 1, 1),*/
    /*Instruction(0xE5, "PUSH HL", 1, 1),*/
    /*Instruction(0xE6, "AND d8", 2, 1),*/
    /*Instruction(0xE7, "RST 20H", 1, 1),*/
    /*Instruction(0xE8, "ADD SP,r8", 2, 1),*/
    /*Instruction(0xE9, "JP (HL)", 1, 1),*/
    /*Instruction(0xEA, "LD (a16),A", 3, 1),*/
    /*Instruction(0xEB, "NONE", 1, 1),*/
    /*Instruction(0xEC, "NONE", 1, 1),*/
    /*Instruction(0xED, "NONE", 1, 1),*/
    /*Instruction(0xEE, "XOR d8", 2, 1),*/
    /*Instruction(0xEF, "RST 28H", 1, 1),*/
    /*Instruction(0xF0, "LDH A,(a8)", 2, 1),*/
    /*Instruction(0xF1, "POP AF", 1, 1),*/
    /*Instruction(0xF2, "LD A,(C)", 1, 1),*/
    /*Instruction(0xF3, "DI", 1, 1),*/
    /*Instruction(0xF4, "NONE", 1, 1),*/
    /*Instruction(0xF5, "PUSH AF", 1, 1),*/
    /*Instruction(0xF6, "OR d8", 2, 1),*/
    /*Instruction(0xF7, "RST 30H", 1, 1),*/
    /*Instruction(0xF8, "LD HL,SP+r8", 2, 1),*/
    /*Instruction(0xF9, "LD SP,HL", 1, 1),*/
    /*Instruction(0xFA, "LD A,(a16)", 3, 1),*/
    /*Instruction(0xFB, "EI", 1, 1),*/
    /*Instruction(0xFC, "NONE", 1, 1),*/
    /*Instruction(0xFD, "NONE", 1, 1),*/
    /*Instruction(0xFE, "CP d8", 2, 1),*/
    /*Instruction(0xFF, "RST 38H", 1, 1)*/
};

#endif
