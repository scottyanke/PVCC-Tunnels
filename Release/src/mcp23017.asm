;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.7.0 #10231 (Linux)
;--------------------------------------------------------
	.module mcp23017
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _i2c_read
	.globl _i2c_write_addr
	.globl _i2c_write
	.globl _i2c_stop
	.globl _i2c_start
	.globl _i2c_clear
	.globl _init_mcp23017
	.globl _backlight_on
	.globl _backlight_off
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	../src/mcp23017.c: 12: uint8_t init_mcp23017() {
;	-----------------------------------------
;	 function init_mcp23017
;	-----------------------------------------
_init_mcp23017:
;	../src/mcp23017.c: 14: i2c_clear();
	call	_i2c_clear
;	../src/mcp23017.c: 15: if (i2c_start())
	call	_i2c_start
	tnz	a
	jrne	00110$
	jp	00102$
00110$:
;	../src/mcp23017.c: 17: i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	push	#0x40
	call	_i2c_write_addr
	pop	a
;	../src/mcp23017.c: 18: i2c_write(0x00);  // gpioA as output
	push	#0x00
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 19: i2c_write(0x00);
	push	#0x00
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 20: i2c_stop();
	call	_i2c_stop
;	../src/mcp23017.c: 21: i2c_start();
	call	_i2c_start
;	../src/mcp23017.c: 22: i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	push	#0x40
	call	_i2c_write_addr
	pop	a
;	../src/mcp23017.c: 23: i2c_write(0x01);  // gpioB as input
	push	#0x01
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 24: i2c_write(0xff);
	push	#0xff
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 25: i2c_stop();
	call	_i2c_stop
;	../src/mcp23017.c: 26: i2c_start();
	call	_i2c_start
;	../src/mcp23017.c: 27: i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	push	#0x40
	call	_i2c_write_addr
	pop	a
;	../src/mcp23017.c: 28: i2c_write(0x0d);  // gpioB has internal pull-ups
	push	#0x0d
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 29: i2c_write(0xff);
	push	#0xff
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 30: i2c_stop();
	call	_i2c_stop
;	../src/mcp23017.c: 31: i2c_start();
	call	_i2c_start
;	../src/mcp23017.c: 32: i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	push	#0x40
	call	_i2c_write_addr
	pop	a
;	../src/mcp23017.c: 33: i2c_write(0x03);  // gpioB has reversed polarity
	push	#0x03
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 34: i2c_write(0xff);
	push	#0xff
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 35: i2c_stop();
	call	_i2c_stop
;	../src/mcp23017.c: 36: i2c_start();
	call	_i2c_start
;	../src/mcp23017.c: 37: i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	push	#0x40
	call	_i2c_write_addr
	pop	a
;	../src/mcp23017.c: 38: i2c_write(0x13);  // looking at getting from GPIOB
	push	#0x13
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 39: i2c_stop();
	call	_i2c_stop
;	../src/mcp23017.c: 40: i2c_start();
	call	_i2c_start
;	../src/mcp23017.c: 41: i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	push	#0x40
	call	_i2c_write_addr
	pop	a
;	../src/mcp23017.c: 42: i2c_stop();
	call	_i2c_stop
;	../src/mcp23017.c: 43: i2c_start();
	call	_i2c_start
;	../src/mcp23017.c: 44: i2c_write_addr(MCP23017_ADDR + I2C_READ);
	push	#0x41
	call	_i2c_write_addr
	pop	a
;	../src/mcp23017.c: 45: rb = i2c_read();
;	../src/mcp23017.c: 46: return rb;
	jp	_i2c_read
00102$:
;	../src/mcp23017.c: 49: return 0xff;
	ld	a, #0xff
;	../src/mcp23017.c: 51: }
	ret
;	../src/mcp23017.c: 52: void backlight_on()		// was part of the original design that had an LCD
;	-----------------------------------------
;	 function backlight_on
;	-----------------------------------------
_backlight_on:
;	../src/mcp23017.c: 54: i2c_start();
	call	_i2c_start
;	../src/mcp23017.c: 55: i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	push	#0x40
	call	_i2c_write_addr
	pop	a
;	../src/mcp23017.c: 56: i2c_write(0x12);  // gpioA register
	push	#0x12
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 57: i2c_write(0x80);   // set the backlight pin on
	push	#0x80
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 58: i2c_stop();
;	../src/mcp23017.c: 59: }
	jp	_i2c_stop
;	../src/mcp23017.c: 60: void backlight_off()
;	-----------------------------------------
;	 function backlight_off
;	-----------------------------------------
_backlight_off:
;	../src/mcp23017.c: 62: i2c_start();
	call	_i2c_start
;	../src/mcp23017.c: 63: i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	push	#0x40
	call	_i2c_write_addr
	pop	a
;	../src/mcp23017.c: 64: i2c_write(0x12);  // gpioA register
	push	#0x12
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 65: i2c_write(0x00);   // set the backlight pin off
	push	#0x00
	call	_i2c_write
	pop	a
;	../src/mcp23017.c: 66: i2c_stop();
;	../src/mcp23017.c: 67: }
	jp	_i2c_stop
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
