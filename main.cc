#include "src/cartridge.h"
#include "src/cpu.h"
#include "SDL3/SDL.h"
#include "SDL3/SDL_main.h"
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_render.h>

int main(int argc, char *argv[]) {
  /*
   * SDL initialization code
   */
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window* window = SDL_CreateWindow("GameBoy Emulator", 160, 144, 0);
  if(window == nullptr) {
    std::cerr << "Couldn't create window: " << SDL_GetError() << '\n';
    SDL_Quit();
    return 1;
  }
  SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
  if(renderer == nullptr) {
    std::cerr << "Couldn't create renderer: " << SDL_GetError() << '\n';
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  SDL_Event event;


  CPU cpu(std::make_shared<Cartridge>());
  std::cout << "Running CPU..." << std::endl;
  for (;;) {
    cpu.Run();
    SDL_PollEvent(&event);
    if(event.type == SDL_EVENT_QUIT) {
      break;
    }
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);
  }
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
