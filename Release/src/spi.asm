;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.7.0 #10231 (Linux)
;--------------------------------------------------------
	.module spi
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _SPI_init
	.globl _SPI_read
	.globl _SPI_write
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
;	../src/spi.c: 4: void SPI_init() {
;	-----------------------------------------
;	 function SPI_init
;	-----------------------------------------
_SPI_init:
;	../src/spi.c: 5: SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_SPE | (5 << 4);
	mov	0x5200+0, #0x54
;	../src/spi.c: 6: SPI->CR2 = SPI_CR2_SSM | SPI_CR2_SSI | SPI_CR2_BDM | SPI_CR2_BDOE;
	mov	0x5201+0, #0xc3
;	../src/spi.c: 7: }
	ret
;	../src/spi.c: 9: uint8_t SPI_read() {
;	-----------------------------------------
;	 function SPI_read
;	-----------------------------------------
_SPI_read:
;	../src/spi.c: 10: SPI_write(0xFF);
	push	#0xff
	call	_SPI_write
	pop	a
;	../src/spi.c: 11: while (!(SPI->SR & SPI_SR_RXNE));
00101$:
	ld	a, 0x5203
	srl	a
	jrnc	00101$
;	../src/spi.c: 12: return SPI->DR;
	ld	a, 0x5204
;	../src/spi.c: 13: }
	ret
;	../src/spi.c: 15: void SPI_write(uint8_t data) {
;	-----------------------------------------
;	 function SPI_write
;	-----------------------------------------
_SPI_write:
;	../src/spi.c: 16: SPI->DR = data;
	ldw	x, #0x5204
	ld	a, (0x03, sp)
	ld	(x), a
;	../src/spi.c: 17: while (!(SPI->SR & SPI_SR_TXE));
00101$:
	ld	a, 0x5203
	bcp	a, #0x02
	jreq	00101$
;	../src/spi.c: 18: }
	ret
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
