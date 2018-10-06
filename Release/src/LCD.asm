;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.7.0 #10231 (Linux)
;--------------------------------------------------------
	.module LCD
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _font6x8
	.globl _SPI_write
	.globl _strlen
	.globl _LCD_init
	.globl _LCD_cmd
	.globl _LCD_write
	.globl _LCD_clear
	.globl _LCD_goto
	.globl _LCD_putc
	.globl _LCD_printf
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
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 30: static inline void LCD_gpio_init() {
;	-----------------------------------------
;	 function LCD_gpio_init
;	-----------------------------------------
_LCD_gpio_init:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 31: GPIOD->DDR |= (1 << LCD_CE_PIN) | (1 << LCD_RST_PIN);
	ld	a, 0x5011
	or	a, #0x06
	ld	0x5011, a
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 32: GPIOD->CR1 |= (1 << LCD_CE_PIN) | (1 << LCD_RST_PIN);
	ld	a, 0x5012
	or	a, #0x06
	ld	0x5012, a
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 33: GPIOC->DDR |= (1 << LCD_DC_PIN);
	bset	20492, #4
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 34: GPIOC->CR1 |= (1 << LCD_DC_PIN);
	bset	20493, #4
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 35: }
	ret
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 37: static inline void LCD_DC_set() {
;	-----------------------------------------
;	 function LCD_DC_set
;	-----------------------------------------
_LCD_DC_set:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 38: GPIOC->ODR |= (1 << LCD_DC_PIN);
	bset	20490, #4
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 39: }
	ret
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 41: static inline void LCD_DC_clear() {
;	-----------------------------------------
;	 function LCD_DC_clear
;	-----------------------------------------
_LCD_DC_clear:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 42: GPIOC->ODR &= ~(1 << LCD_DC_PIN);
	bres	20490, #4
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 43: }
	ret
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 45: static inline void LCD_CE_set() {
;	-----------------------------------------
;	 function LCD_CE_set
;	-----------------------------------------
_LCD_CE_set:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 46: while ((SPI->SR & SPI_SR_BSY));
00101$:
	ld	a, 0x5203
	tnz	a
	jrmi	00101$
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 47: GPIOD->ODR |= (1 << LCD_CE_PIN);
	bset	20495, #1
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 48: }
	ret
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 50: static inline void LCD_CE_clear() {
;	-----------------------------------------
;	 function LCD_CE_clear
;	-----------------------------------------
_LCD_CE_clear:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 51: GPIOD->ODR &= ~(1 << LCD_CE_PIN);
	bres	20495, #1
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 52: }
	ret
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 54: static inline void LCD_RST_set() {
;	-----------------------------------------
;	 function LCD_RST_set
;	-----------------------------------------
_LCD_RST_set:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 55: GPIOD->ODR |= (1 << LCD_RST_PIN);
	bset	20495, #2
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 56: }
	ret
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 58: static inline void LCD_RST_clear() {
;	-----------------------------------------
;	 function LCD_RST_clear
;	-----------------------------------------
_LCD_RST_clear:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 59: GPIOD->ODR &= ~(1 << LCD_RST_PIN);
	bres	20495, #2
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 60: }
	ret
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 62: static inline void LCD_SPI_write(uint8_t word) {
;	-----------------------------------------
;	 function LCD_SPI_write
;	-----------------------------------------
_LCD_SPI_write:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 63: SPI_write(word);
	ld	a, (0x03, sp)
	push	a
	call	_SPI_write
	pop	a
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 64: }
	ret
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 66: static inline void LCD_delay_ms(int ms) {
;	-----------------------------------------
;	 function LCD_delay_ms
;	-----------------------------------------
_LCD_delay_ms:
	sub	sp, #8
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 67: delay_ms(ms);
	ldw	y, (0x0b, sp)
	clrw	x
	tnzw	y
	jrpl	00116$
	decw	x
00116$:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clr	(0x08, sp)
	clr	(0x07, sp)
	clr	(0x06, sp)
	clr	(0x05, sp)
	pushw	y
	pushw	x
	push	#0x78
	push	#0x03
	clrw	x
	pushw	x
	call	__mullong
	addw	sp, #8
	ldw	(0x03, sp), x
	ldw	(0x01, sp), y
00104$:
	ldw	x, (0x07, sp)
	cpw	x, (0x03, sp)
	ld	a, (0x06, sp)
	sbc	a, (0x02, sp)
	ld	a, (0x05, sp)
	sbc	a, (0x01, sp)
	jrnc	00106$
;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x07, sp)
	addw	y, #0x0001
	ld	a, (0x06, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x05, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x07, sp), y
	ldw	(0x05, sp), x
	jra	00104$
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 67: delay_ms(ms);
00106$:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 68: }
	addw	sp, #8
	ret
;	../src/LCD.c: 7: void LCD_init() {
;	-----------------------------------------
;	 function LCD_init
;	-----------------------------------------
_LCD_init:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 31: GPIOD->DDR |= (1 << LCD_CE_PIN) | (1 << LCD_RST_PIN);
	ld	a, 0x5011
	or	a, #0x06
	ld	0x5011, a
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 32: GPIOD->CR1 |= (1 << LCD_CE_PIN) | (1 << LCD_RST_PIN);
	ld	a, 0x5012
	or	a, #0x06
	ld	0x5012, a
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 33: GPIOC->DDR |= (1 << LCD_DC_PIN);
	bset	20492, #4
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 34: GPIOC->CR1 |= (1 << LCD_DC_PIN);
	bset	20493, #4
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 51: GPIOD->ODR &= ~(1 << LCD_CE_PIN);
	bres	20495, #1
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 59: GPIOD->ODR &= ~(1 << LCD_RST_PIN);
	ld	a, 0x500f
	and	a, #0xfb
	ld	0x500f, a
;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	y
	clrw	x
00113$:
	cpw	y, #0xad70
	ld	a, xl
	sbc	a, #0x00
	ld	a, xh
	sbc	a, #0x00
	jrnc	00105$
;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	incw	y
	jrne	00113$
	incw	x
	jra	00113$
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 67: delay_ms(ms);
00105$:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 55: GPIOD->ODR |= (1 << LCD_RST_PIN);
	bset	20495, #2
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 46: while ((SPI->SR & SPI_SR_BSY));
00108$:
	ld	a, 0x5203
	tnz	a
	jrmi	00108$
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 47: GPIOD->ODR |= (1 << LCD_CE_PIN);
	bset	20495, #1
;	../src/LCD.c: 14: LCD_cmd(0x21); // extended commands
	push	#0x21
	call	_LCD_cmd
	pop	a
;	../src/LCD.c: 15: LCD_cmd(0xc1); // contrast Vop=6.4V
	push	#0xc1
	call	_LCD_cmd
	pop	a
;	../src/LCD.c: 16: LCD_cmd(0x04); // temperature coefficient
	push	#0x04
	call	_LCD_cmd
	pop	a
;	../src/LCD.c: 17: LCD_cmd(0x13); // bias = 1:48
	push	#0x13
	call	_LCD_cmd
	pop	a
;	../src/LCD.c: 19: LCD_cmd(0x20); // standard commands
	push	#0x20
	call	_LCD_cmd
	pop	a
;	../src/LCD.c: 20: LCD_cmd(0x0C); // normal mode
	push	#0x0c
	call	_LCD_cmd
	pop	a
;	../src/LCD.c: 22: LCD_clear();
;	../src/LCD.c: 23: }
	jp	_LCD_clear
;	../src/LCD.c: 25: void LCD_cmd(uint8_t cmd) {
;	-----------------------------------------
;	 function LCD_cmd
;	-----------------------------------------
_LCD_cmd:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 51: GPIOD->ODR &= ~(1 << LCD_CE_PIN);
	bres	20495, #1
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 42: GPIOC->ODR &= ~(1 << LCD_DC_PIN);
	bres	20490, #4
;	../src/LCD.c: 28: LCD_SPI_write(cmd);
	ld	a, (0x03, sp)
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 63: SPI_write(word);
	push	a
	call	_SPI_write
	pop	a
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 46: while ((SPI->SR & SPI_SR_BSY));
00104$:
	ld	a, 0x5203
	tnz	a
	jrmi	00104$
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 47: GPIOD->ODR |= (1 << LCD_CE_PIN);
	bset	20495, #1
;	../src/LCD.c: 29: LCD_CE_set();
;	../src/LCD.c: 30: }
	ret
;	../src/LCD.c: 32: void LCD_write(uint8_t data) {
;	-----------------------------------------
;	 function LCD_write
;	-----------------------------------------
_LCD_write:
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 51: GPIOD->ODR &= ~(1 << LCD_CE_PIN);
	bres	20495, #1
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 38: GPIOC->ODR |= (1 << LCD_DC_PIN);
	bset	20490, #4
;	../src/LCD.c: 35: LCD_SPI_write(data);
	ld	a, (0x03, sp)
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 63: SPI_write(word);
	push	a
	call	_SPI_write
	pop	a
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 46: while ((SPI->SR & SPI_SR_BSY));
00104$:
	ld	a, 0x5203
	tnz	a
	jrmi	00104$
;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 47: GPIOD->ODR |= (1 << LCD_CE_PIN);
	bset	20495, #1
;	../src/LCD.c: 36: LCD_CE_set();
;	../src/LCD.c: 37: }
	ret
;	../src/LCD.c: 39: void LCD_clear() {
;	-----------------------------------------
;	 function LCD_clear
;	-----------------------------------------
_LCD_clear:
	sub	sp, #2
;	../src/LCD.c: 40: uint16_t i = 84 * 6;
	ldw	x, #0x01f8
;	../src/LCD.c: 41: LCD_goto(0, 0);
	pushw	x
	push	#0x00
	push	#0x00
	call	_LCD_goto
	addw	sp, #2
	popw	x
;	../src/LCD.c: 42: while (i-- > 0)
00101$:
	ldw	(0x01, sp), x
	decw	x
	ldw	y, (0x01, sp)
	jreq	00104$
;	../src/LCD.c: 43: LCD_write(0);
	pushw	x
	push	#0x00
	call	_LCD_write
	pop	a
	popw	x
	jra	00101$
00104$:
;	../src/LCD.c: 44: }
	addw	sp, #2
	ret
;	../src/LCD.c: 46: void LCD_goto(uint8_t col, uint8_t row) {
;	-----------------------------------------
;	 function LCD_goto
;	-----------------------------------------
_LCD_goto:
;	../src/LCD.c: 47: LCD_cmd(0x80 | col);
	ld	a, (0x03, sp)
	exg	a, xl
	clr	a
	exg	a, xl
	or	a, #0x80
	push	a
	call	_LCD_cmd
	pop	a
;	../src/LCD.c: 48: LCD_cmd(0x40 | row);
	ld	a, (0x04, sp)
	exg	a, xl
	clr	a
	exg	a, xl
	or	a, #0x40
	push	a
	call	_LCD_cmd
	pop	a
;	../src/LCD.c: 49: }
	ret
;	../src/LCD.c: 51: void LCD_putc(char c) {
;	-----------------------------------------
;	 function LCD_putc
;	-----------------------------------------
_LCD_putc:
	sub	sp, #5
;	../src/LCD.c: 53: if (c == ' ') {
	ld	a, (0x08, sp)
	cp	a, #0x20
	jrne	00103$
;	../src/LCD.c: 54: LCD_write(0);
	push	#0x00
	call	_LCD_write
	pop	a
;	../src/LCD.c: 55: LCD_write(0);
	push	#0x00
	call	_LCD_write
	pop	a
	jra	00107$
00103$:
;	../src/LCD.c: 57: c -= 32;  // just printable characters
	ld	a, (0x08, sp)
	sub	a, #0x20
	ld	(0x08, sp), a
;	../src/LCD.c: 59: for (i=0; i<6; i++)
	clr	(0x05, sp)
	ldw	x, #_font6x8+0
	ldw	(0x03, sp), x
	ld	a, (0x08, sp)
	ld	xl, a
	ld	a, #0x06
	mul	x, a
	addw	x, (0x03, sp)
	ldw	(0x01, sp), x
00105$:
;	../src/LCD.c: 60: LCD_write(font6x8[c][i]);
	clrw	x
	ld	a, (0x05, sp)
	ld	xl, a
	addw	x, (0x01, sp)
	ld	a, (x)
	push	a
	call	_LCD_write
	pop	a
;	../src/LCD.c: 59: for (i=0; i<6; i++)
	inc	(0x05, sp)
	ld	a, (0x05, sp)
	cp	a, #0x06
	jrc	00105$
;	../src/LCD.c: 61: LCD_write(0);
	push	#0x00
	call	_LCD_write
	pop	a
00107$:
;	../src/LCD.c: 64: }
	addw	sp, #5
	ret
;	../src/LCD.c: 66: void LCD_printf(char *s) {
;	-----------------------------------------
;	 function LCD_printf
;	-----------------------------------------
_LCD_printf:
	sub	sp, #2
;	../src/LCD.c: 69: len = strlen(s);
	ldw	x, (0x05, sp)
	pushw	x
	call	_strlen
	addw	sp, #2
	ld	a, xl
	ld	(0x01, sp), a
;	../src/LCD.c: 70: for (i = 0; i < len; i++)
	clr	(0x02, sp)
00103$:
	ld	a, (0x02, sp)
	cp	a, (0x01, sp)
	jrnc	00105$
;	../src/LCD.c: 72: c = s[i];
	clrw	x
	ld	a, (0x02, sp)
	ld	xl, a
	addw	x, (0x05, sp)
	ld	a, (x)
;	../src/LCD.c: 73: LCD_putc(c);
	push	a
	call	_LCD_putc
	pop	a
;	../src/LCD.c: 70: for (i = 0; i < len; i++)
	inc	(0x02, sp)
	jra	00103$
00105$:
;	../src/LCD.c: 75: }
	addw	sp, #2
	ret
	.area CODE
_font6x8:
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x2f	; 47
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x02	; 2
	.db #0x05	; 5
	.db #0x02	; 2
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x14	; 20
	.db #0x7f	; 127
	.db #0x14	; 20
	.db #0x7f	; 127
	.db #0x14	; 20
	.db #0x00	; 0
	.db #0x24	; 36
	.db #0x2a	; 42
	.db #0x7f	; 127
	.db #0x2a	; 42
	.db #0x12	; 18
	.db #0x00	; 0
	.db #0x62	; 98	'b'
	.db #0x64	; 100	'd'
	.db #0x08	; 8
	.db #0x13	; 19
	.db #0x23	; 35
	.db #0x00	; 0
	.db #0x36	; 54	'6'
	.db #0x49	; 73	'I'
	.db #0x55	; 85	'U'
	.db #0x22	; 34
	.db #0x50	; 80	'P'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x1c	; 28
	.db #0x22	; 34
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x1c	; 28
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x14	; 20
	.db #0x08	; 8
	.db #0x3e	; 62
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x3e	; 62
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0xa0	; 160
	.db #0x60	; 96
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x60	; 96
	.db #0x60	; 96
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x10	; 16
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x02	; 2
	.db #0x00	; 0
	.db #0x3e	; 62
	.db #0x51	; 81	'Q'
	.db #0x49	; 73	'I'
	.db #0x45	; 69	'E'
	.db #0x3e	; 62
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x42	; 66	'B'
	.db #0x7f	; 127
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x42	; 66	'B'
	.db #0x61	; 97	'a'
	.db #0x51	; 81	'Q'
	.db #0x49	; 73	'I'
	.db #0x46	; 70	'F'
	.db #0x00	; 0
	.db #0x21	; 33
	.db #0x41	; 65	'A'
	.db #0x45	; 69	'E'
	.db #0x4b	; 75	'K'
	.db #0x31	; 49	'1'
	.db #0x00	; 0
	.db #0x18	; 24
	.db #0x14	; 20
	.db #0x12	; 18
	.db #0x7f	; 127
	.db #0x10	; 16
	.db #0x00	; 0
	.db #0x27	; 39
	.db #0x45	; 69	'E'
	.db #0x45	; 69	'E'
	.db #0x45	; 69	'E'
	.db #0x39	; 57	'9'
	.db #0x00	; 0
	.db #0x3c	; 60
	.db #0x4a	; 74	'J'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x30	; 48	'0'
	.db #0x00	; 0
	.db #0x01	; 1
	.db #0x71	; 113	'q'
	.db #0x09	; 9
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x00	; 0
	.db #0x36	; 54	'6'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x36	; 54	'6'
	.db #0x00	; 0
	.db #0x06	; 6
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x29	; 41
	.db #0x1e	; 30
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x36	; 54	'6'
	.db #0x36	; 54	'6'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x56	; 86	'V'
	.db #0x36	; 54	'6'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x22	; 34
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x14	; 20
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x02	; 2
	.db #0x01	; 1
	.db #0x51	; 81	'Q'
	.db #0x09	; 9
	.db #0x06	; 6
	.db #0x00	; 0
	.db #0x32	; 50	'2'
	.db #0x49	; 73	'I'
	.db #0x59	; 89	'Y'
	.db #0x51	; 81	'Q'
	.db #0x3e	; 62
	.db #0x00	; 0
	.db #0x7c	; 124
	.db #0x12	; 18
	.db #0x11	; 17
	.db #0x12	; 18
	.db #0x7c	; 124
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x36	; 54	'6'
	.db #0x00	; 0
	.db #0x3e	; 62
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x1c	; 28
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x01	; 1
	.db #0x00	; 0
	.db #0x3e	; 62
	.db #0x41	; 65	'A'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x7a	; 122	'z'
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x7f	; 127
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x7f	; 127
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x41	; 65	'A'
	.db #0x3f	; 63
	.db #0x01	; 1
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x22	; 34
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x02	; 2
	.db #0x0c	; 12
	.db #0x02	; 2
	.db #0x7f	; 127
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x04	; 4
	.db #0x08	; 8
	.db #0x10	; 16
	.db #0x7f	; 127
	.db #0x00	; 0
	.db #0x3e	; 62
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x3e	; 62
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x06	; 6
	.db #0x00	; 0
	.db #0x3e	; 62
	.db #0x41	; 65	'A'
	.db #0x51	; 81	'Q'
	.db #0x21	; 33
	.db #0x5e	; 94
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x09	; 9
	.db #0x19	; 25
	.db #0x29	; 41
	.db #0x46	; 70	'F'
	.db #0x00	; 0
	.db #0x46	; 70	'F'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x31	; 49	'1'
	.db #0x00	; 0
	.db #0x01	; 1
	.db #0x01	; 1
	.db #0x7f	; 127
	.db #0x01	; 1
	.db #0x01	; 1
	.db #0x00	; 0
	.db #0x3f	; 63
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x3f	; 63
	.db #0x00	; 0
	.db #0x1f	; 31
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x1f	; 31
	.db #0x00	; 0
	.db #0x3f	; 63
	.db #0x40	; 64
	.db #0x38	; 56	'8'
	.db #0x40	; 64
	.db #0x3f	; 63
	.db #0x00	; 0
	.db #0x63	; 99	'c'
	.db #0x14	; 20
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x63	; 99	'c'
	.db #0x00	; 0
	.db #0x07	; 7
	.db #0x08	; 8
	.db #0x70	; 112	'p'
	.db #0x08	; 8
	.db #0x07	; 7
	.db #0x00	; 0
	.db #0x61	; 97	'a'
	.db #0x51	; 81	'Q'
	.db #0x49	; 73	'I'
	.db #0x45	; 69	'E'
	.db #0x43	; 67	'C'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x55	; 85	'U'
	.db #0x2a	; 42
	.db #0x55	; 85	'U'
	.db #0x2a	; 42
	.db #0x55	; 85	'U'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x7f	; 127
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x04	; 4
	.db #0x02	; 2
	.db #0x01	; 1
	.db #0x02	; 2
	.db #0x04	; 4
	.db #0x00	; 0
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x01	; 1
	.db #0x02	; 2
	.db #0x04	; 4
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x78	; 120	'x'
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x48	; 72	'H'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x38	; 56	'8'
	.db #0x00	; 0
	.db #0x38	; 56	'8'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x20	; 32
	.db #0x00	; 0
	.db #0x38	; 56	'8'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x48	; 72	'H'
	.db #0x7f	; 127
	.db #0x00	; 0
	.db #0x38	; 56	'8'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x18	; 24
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x7e	; 126
	.db #0x09	; 9
	.db #0x01	; 1
	.db #0x02	; 2
	.db #0x00	; 0
	.db #0x18	; 24
	.db #0xa4	; 164
	.db #0xa4	; 164
	.db #0xa4	; 164
	.db #0x7c	; 124
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x04	; 4
	.db #0x78	; 120	'x'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x44	; 68	'D'
	.db #0x7d	; 125
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x40	; 64
	.db #0x80	; 128
	.db #0x84	; 132
	.db #0x7d	; 125
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x7f	; 127
	.db #0x10	; 16
	.db #0x28	; 40
	.db #0x44	; 68	'D'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x7f	; 127
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x7c	; 124
	.db #0x04	; 4
	.db #0x18	; 24
	.db #0x04	; 4
	.db #0x78	; 120	'x'
	.db #0x00	; 0
	.db #0x7c	; 124
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x04	; 4
	.db #0x78	; 120	'x'
	.db #0x00	; 0
	.db #0x38	; 56	'8'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x38	; 56	'8'
	.db #0x00	; 0
	.db #0xfc	; 252
	.db #0x24	; 36
	.db #0x24	; 36
	.db #0x24	; 36
	.db #0x18	; 24
	.db #0x00	; 0
	.db #0x18	; 24
	.db #0x24	; 36
	.db #0x24	; 36
	.db #0x18	; 24
	.db #0xfc	; 252
	.db #0x00	; 0
	.db #0x7c	; 124
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x04	; 4
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x48	; 72	'H'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x20	; 32
	.db #0x00	; 0
	.db #0x04	; 4
	.db #0x3f	; 63
	.db #0x44	; 68	'D'
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x00	; 0
	.db #0x3c	; 60
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x7c	; 124
	.db #0x00	; 0
	.db #0x1c	; 28
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x1c	; 28
	.db #0x00	; 0
	.db #0x3c	; 60
	.db #0x40	; 64
	.db #0x30	; 48	'0'
	.db #0x40	; 64
	.db #0x3c	; 60
	.db #0x00	; 0
	.db #0x44	; 68	'D'
	.db #0x28	; 40
	.db #0x10	; 16
	.db #0x28	; 40
	.db #0x44	; 68	'D'
	.db #0x00	; 0
	.db #0x1c	; 28
	.db #0xa0	; 160
	.db #0xa0	; 160
	.db #0xa0	; 160
	.db #0x7c	; 124
	.db #0x00	; 0
	.db #0x44	; 68	'D'
	.db #0x64	; 100	'd'
	.db #0x54	; 84	'T'
	.db #0x4c	; 76	'L'
	.db #0x44	; 68	'D'
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.area INITIALIZER
	.area CABS (ABS)
