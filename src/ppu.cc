#include "ppu.h"
#include <cstdlib>
/*
 * -- GAMEBOY PPU --
 *
 *  160 x 144 Pixel LCD
 *  Tiles
 *      -> 8x8 pixel sets
 *      -> Stored in VRAM @ 0x8000 -> 0x9FFF with TWO addressing modes
 *          -> 8000 as a base pointer, then add tile_number  * 16
            -> 9000 as a base pointer, then add signed_tile_number * 16
                -> tile one = 0x9000, tile two = 0x9010
                -> tile 0xFF = 0x8FF0, 0xFE = 0x8FE0 etc.
            -> Read bit 4 of LCDC to figure the addressing mode
 *      -> 1 row in tile takes 2 bytes to store colour data,
 *           16 bytes needed to store a single 8x8 tile and are read
 sequentially
 *      -> PPU operates on a per-pixel, not per-tile, basis when drawing to the
 LCD
 *  Display Layers
 *      -> Background
 *          -> 32x32 grid
 *          -> Two VRAM sections @ 0x9800-0x8BFF, 0x9C00-0x9FFF -> 'background
 maps'
 *              -> First byte in background map is the tile at the very top
 left. Across then down
 *          -> GameBoy only shows 20x18 (160x144) section at a single time ->
 'Viewport'
 *              -> SCX, SCY registers designate the offset from left and top
 respectively
 *                  -> Wrap around if exceeding the border of the background
 *      -> Window
 *          -> 32x32 grid
 *          -> Overlay-type layer, position determined with WX, WY registers
 *
 *      -> Sprites
 *          -> 8x8 pixel tiles which act irrespective of BG / Window layers
 *          -> Stored in OAM memory region which holds data for up to 40 sprites
 *  PPU Modes
 *      -> OAM Scan (MODE 2)
 *          -> Every scanline starts here except VBLANK
 *          -> PPU searches OAM memory for sprites to render to the current
 scanline
 *          -> Sprite is ONLY added to the buffer if the following is true:
 *              -> Sprite x position > 0
 *              -> LY + 16 >= Sprite y position
 *              -> LY + 16 < Sprite y position + Sprite height (8 in normal, 16
 in tall sprite mode)
 *              -> Total number of sprite already stored in OAM buffer < 10
 *      -> Draw (MODE 3)
 *          -> PPU transfers pixels to the LCD
 *          -> Variable duration based on BG scrolling, amount of sprites on the
 scanline etc.
 *      -> HBLANK (MODE 0)
 *          -> Happens AFTER draw mode
 *          -> Essentially pauses the PPU until it reaches a fixed 456 t-cycles
 *      -> VBLANK (MODE 1)
 *          -> Same as HBLANK, except it happens at the end of every frame
 instead of every scanline
 *          -> There are 154 scanlines, but 10 at the end aren't shown to allow
 the PPU to enter this VBLANK state
 *          -> VBLANK scanlines the same as HBLANK = 456 t-cycles. VBLANK for
 one frame = 4560 t-cycles total
 *
 *    Pixel FIFO
 *        -> Pixels are pushed to the screen one by one using two 8-bit shift
 registers which holds data of up to 8 pixels
 *        -> Each pixel stored in the FIFO tracks:
 *            -> Colour - value from the tile data, NOT the palette
 *            -> Palette - OBP0 or OBP1
 *            -> Sprite Priority - only for Colour GameBoy
 *            -> BG Priority - only for sprites. bit 7 of OAM for the sprite
 */

void PPU::Initialize() {

  SDL_Init(SDL_INIT_VIDEO);
  window = SDL_CreateWindow("GameBoy Emulator", 160, 144, 0);
  if (window == nullptr) {
    std::cerr << "Couldn't create window: " << SDL_GetError() << '\n';
    SDL_Quit();
  }
  renderer = SDL_CreateRenderer(window, nullptr);
  if (renderer == nullptr) {
    std::cerr << "Couldn't create renderer: " << SDL_GetError() << '\n';
    SDL_DestroyWindow(window);
    SDL_Quit();
  }
}

void PPU::Clean() {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

int PPU::Run() {
  SDL_Event event;
  SDL_PollEvent(&event);
  if (event.type == SDL_EVENT_QUIT) {
    // temporary
    std::cerr << "Quitting SDL...\n";
    Clean();
    return 1;
  }

  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  return 0;
}
