;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.7.0 #10231 (Linux)
;--------------------------------------------------------
	.module i2c
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _i2c_init
	.globl _i2c_clear
	.globl _i2c_start
	.globl _i2c_stop
	.globl _i2c_write
	.globl _i2c_write_addr
	.globl _i2c_read
	.globl _i2c_read_arr
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
;	../src/i2c.c: 6: void i2c_init() {
;	-----------------------------------------
;	 function i2c_init
;	-----------------------------------------
_i2c_init:
;	../src/i2c.c: 7: I2C->FREQR = 0x16;
	mov	0x5212+0, #0x16
;	../src/i2c.c: 8: I2C->CCRL = 0xa0; // 100kHz
	mov	0x521b+0, #0xa0
;	../src/i2c.c: 9: I2C->CCRH &= ~(0x80); // standard mode
	bres	21020, #7
;	../src/i2c.c: 10: I2C->OARH = 0x40;  // I2C_OARH_ADDMODE; // 7-bit addressing
	mov	0x5214+0, #0x40
;	../src/i2c.c: 11: I2C->CR1 = I2C_CR1_PE;
	mov	0x5210+0, #0x01
;	../src/i2c.c: 12: }
	ret
;	../src/i2c.c: 14: void i2c_clear() {
;	-----------------------------------------
;	 function i2c_clear
;	-----------------------------------------
_i2c_clear:
;	../src/i2c.c: 15: (void) I2C->SR1;
	ldw	x, #0x5217
	ld	a, (x)
;	../src/i2c.c: 16: I2C->DR = 0x00;
	mov	0x5216+0, #0x00
;	../src/i2c.c: 17: }
	ret
;	../src/i2c.c: 19: uint8_t i2c_start() {
;	-----------------------------------------
;	 function i2c_start
;	-----------------------------------------
_i2c_start:
;	../src/i2c.c: 21: I2C->CR2 |= I2C_CR2_START;
	bset	21009, #0
;	../src/i2c.c: 22: i = 0;	// this is done to set a limit on how long to wait
	clrw	x
;	../src/i2c.c: 23: while (!(I2C->SR1 & I2C_SR1_SB) && i < 800)
00102$:
	ld	a, 0x5217
	srl	a
	jrc	00104$
	cpw	x, #0x0320
	jrnc	00104$
;	../src/i2c.c: 24: i++;
	incw	x
	jra	00102$
00104$:
;	../src/i2c.c: 25: if (i > 799)
	cpw	x, #0x031f
	jrule	00106$
;	../src/i2c.c: 26: return 0;
	clr	a
	ret
00106$:
;	../src/i2c.c: 28: return 1;
	ld	a, #0x01
;	../src/i2c.c: 29: }
	ret
;	../src/i2c.c: 31: uint8_t i2c_stop() {
;	-----------------------------------------
;	 function i2c_stop
;	-----------------------------------------
_i2c_stop:
;	../src/i2c.c: 33: I2C->CR2 |= I2C_CR2_STOP;
	bset	21009, #1
;	../src/i2c.c: 34: i = 0;
	clrw	x
;	../src/i2c.c: 35: while ((I2C->SR3 & I2C_SR3_MSL) && i < 800)
00102$:
	ld	a, 0x5219
	srl	a
	jrnc	00104$
	cpw	x, #0x0320
	jrnc	00104$
;	../src/i2c.c: 36: i++;
	incw	x
	jra	00102$
00104$:
;	../src/i2c.c: 37: if (i > 799)
	cpw	x, #0x031f
	jrule	00106$
;	../src/i2c.c: 38: return 0;
	clr	a
	ret
00106$:
;	../src/i2c.c: 40: return 1;
	ld	a, #0x01
;	../src/i2c.c: 41: }
	ret
;	../src/i2c.c: 43: void i2c_write(uint8_t data) {
;	-----------------------------------------
;	 function i2c_write
;	-----------------------------------------
_i2c_write:
;	../src/i2c.c: 45: I2C->DR = data;
	ldw	x, #0x5216
	ld	a, (0x03, sp)
	ld	(x), a
;	../src/i2c.c: 46: i = 0;
	clrw	x
;	../src/i2c.c: 47: while (!(I2C->SR1 & I2C_SR1_TXE) && i < 800)
00102$:
	ld	a, 0x5217
	tnz	a
	jrpl	00119$
	ret
00119$:
	cpw	x, #0x0320
	jrc	00120$
	ret
00120$:
;	../src/i2c.c: 48: i++;
	incw	x
	jra	00102$
;	../src/i2c.c: 49: }
	ret
;	../src/i2c.c: 51: void i2c_write_addr(uint8_t addr) {
;	-----------------------------------------
;	 function i2c_write_addr
;	-----------------------------------------
_i2c_write_addr:
;	../src/i2c.c: 53: I2C->DR = addr;
	ldw	x, #0x5216
	ld	a, (0x03, sp)
	ld	(x), a
;	../src/i2c.c: 54: i = 0;
	clrw	x
;	../src/i2c.c: 55: while (!(I2C->SR1 & I2C_SR1_ADDR) && i < 800)
00102$:
	ld	a, 0x5217
	bcp	a, #0x02
	jrne	00104$
	cpw	x, #0x0320
	jrnc	00104$
;	../src/i2c.c: 56: i++;;
	incw	x
	jra	00102$
00104$:
;	../src/i2c.c: 57: (void) I2C->SR3; // check BUS_BUSY
	ldw	x, #0x5219
	ld	a, (x)
;	../src/i2c.c: 58: I2C->CR2 |= (I2C_CR2_ACK);
	bset	21009, #2
;	../src/i2c.c: 59: }
	ret
;	../src/i2c.c: 61: uint8_t i2c_read() {
;	-----------------------------------------
;	 function i2c_read
;	-----------------------------------------
_i2c_read:
;	../src/i2c.c: 62: I2C->CR2 &= ~(I2C_CR2_ACK);
	bres	21009, #2
;	../src/i2c.c: 63: i2c_stop();
	call	_i2c_stop
;	../src/i2c.c: 64: while (!(I2C->SR1 & I2C_SR1_RXNE)) ;
00101$:
	ld	a, 0x5217
	bcp	a, #0x40
	jreq	00101$
;	../src/i2c.c: 65: return I2C->DR;
	ld	a, 0x5216
;	../src/i2c.c: 66: }
	ret
;	../src/i2c.c: 68: void i2c_read_arr(uint8_t *buf, uint8_t len) {
;	-----------------------------------------
;	 function i2c_read_arr
;	-----------------------------------------
_i2c_read_arr:
;	../src/i2c.c: 69: while (len-- > 1) {
00104$:
	ld	a, (0x05, sp)
	push	a
	ld	a, (1, sp)
	dec	a
	ld	(0x06, sp), a
	pop	a
;	../src/i2c.c: 72: *(buf++) = I2C->DR;
	ldw	x, (0x03, sp)
;	../src/i2c.c: 69: while (len-- > 1) {
	cp	a, #0x01
	jrule	00106$
;	../src/i2c.c: 70: I2C->CR2 |= (1 << I2C_CR2_ACK);
	bset	21009, #4
;	../src/i2c.c: 71: while (!(I2C->SR1 & I2C_SR1_RXNE)) ;
00101$:
	ld	a, 0x5217
	bcp	a, #0x40
	jreq	00101$
;	../src/i2c.c: 72: *(buf++) = I2C->DR;
	ld	a, 0x5216
	ld	(x), a
	incw	x
	ldw	(0x03, sp), x
	jra	00104$
00106$:
;	../src/i2c.c: 74: *buf = i2c_read();
	pushw	x
	call	_i2c_read
	popw	x
	ld	(x), a
;	../src/i2c.c: 75: }
	ret
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
