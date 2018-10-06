#ifndef mcp23017_H
#define mcp23017_H

#define MCP23017_ADDR      (0x20U << 1)
uint8_t init_mcp23017();
void backlight_on();
void backlight_off();
#endif
