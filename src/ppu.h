#ifndef GAMEBOY_EMULATOR_SRC_PPU_H_
#define GAMEBOY_EMULATOR_SRC_PPU_H_

#include "SDL3/SDL.h"
#include <iostream>

enum PPU_MODE { OAM_SCAN, DRAW, HBLANK, VBLANK };

class PPU {
public:
  explicit PPU() {}
  ~PPU() = default;
  void Initialize();
  void Clean();
  int Run();

private:
  SDL_Window* window;
  SDL_Renderer* renderer;

};

#endif
