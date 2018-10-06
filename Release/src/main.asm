;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.7.0 #10231 (Linux)
;--------------------------------------------------------
	.module main
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _measure
	.globl _InitialiseIWDG
	.globl _InitialiseSystemClock
	.globl _init_mcp23017
	.globl _i2c_read_arr
	.globl _i2c_write_addr
	.globl _i2c_write
	.globl _i2c_stop
	.globl _i2c_start
	.globl _i2c_init
	.globl _UART_read_byte
	.globl _uart_write
	.globl _uart_init
	.globl _sprintf
	.globl _printf
	.globl _last_measure
	.globl _version
	.globl _address
	.globl _esc
	.globl _buf
	.globl _light_time
	.globl _Global_time
	.globl _putchar
	.globl _GPIO_Init
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
_Global_time::
	.ds 4
_light_time::
	.ds 4
_buf::
	.ds 20
_esc::
	.ds 1
_address::
	.ds 1
_version::
	.ds 12
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
_last_measure::
	.ds 4
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ; reset
	int 0x0000 ; trap
	int _TLI_IRQHandler ; int0
	int _AWU_IRQHandler ; int1
	int _CLK_IRQHandler ; int2
	int _EXTI_PORTA_IRQHandler ; int3
	int _EXTI_PORTB_IRQHandler ; int4
	int _EXTI_PORTC_IRQHandler ; int5
	int _EXTI_PORTD_IRQHandler ; int6
	int _EXTI_PORTE_IRQHandler ; int7
	int 0x0000 ; int8
	int 0x0000 ; int9
	int _SPI_IRQHandler ; int10
	int _TIM1_UPD_OVF_TRG_BRK_IRQHandler ; int11
	int _TIM1_CAP_COM_IRQHandler ; int12
	int _TIM2_UPD_OVF_BRK_IRQHandler ; int13
	int _TIM2_CAP_COM_IRQHandler ; int14
	int 0x0000 ; int15
	int 0x0000 ; int16
	int _UART1_TX_IRQHandler ; int17
	int _UART1_RX_IRQHandler ; int18
	int _I2C_IRQHandler ; int19
	int 0x0000 ; int20
	int 0x0000 ; int21
	int _ADC1_IRQHandler ; int22
	int _TIM4_UPD_OVF_IRQHandler ; int23
	int _EEPROM_EEC_IRQHandler ; int24
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	../src/main.c: 40: int putchar(int c) {
;	-----------------------------------------
;	 function putchar
;	-----------------------------------------
_putchar:
;	../src/main.c: 41: uart_write(c);
	ld	a, (0x04, sp)
	push	a
	call	_uart_write
	pop	a
;	../src/main.c: 42: return c;
	ldw	x, (0x03, sp)
;	../src/main.c: 43: }
	ret
;	../src/main.c: 49: void InitialiseSystemClock()
;	-----------------------------------------
;	 function InitialiseSystemClock
;	-----------------------------------------
_InitialiseSystemClock:
;	../src/main.c: 51: CLK->ICKR = 0;                       //  Reset the Internal Clock Register.
	mov	0x50c0+0, #0x00
;	../src/main.c: 52: CLK->ICKR = CLK_ICKR_HSIEN;          //  Enable the HSI.
	mov	0x50c0+0, #0x01
;	../src/main.c: 53: CLK->ECKR = 0;                       //  Disable the external clock.
	mov	0x50c1+0, #0x00
;	../src/main.c: 54: while (!(CLK->ICKR & CLK_ICKR_HSIRDY)); //  Wait for the HSI to be ready for use.
00101$:
	ld	a, 0x50c0
	bcp	a, #0x02
	jreq	00101$
;	../src/main.c: 55: CLK->CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
	mov	0x50c6+0, #0x00
;	../src/main.c: 56: CLK->PCKENR1 = 0xff; //CLK_PCKENR1_TIM4 | CLK_PCKENR1_UART1 | CLK_PCKENR1_SPI | CLK_PCKENR1_I2C ;  //  Enable select peripheral clocks.
	mov	0x50c7+0, #0xff
;	../src/main.c: 57: CLK->PCKENR2 = 0xff; //CLK_PCKENR2_AWU;      //  Only enable the AWU watchdog service
	mov	0x50ca+0, #0xff
;	../src/main.c: 58: CLK->CCOR = 0;                       //  Turn off CCO.
	mov	0x50c9+0, #0x00
;	../src/main.c: 59: CLK->HSITRIMR = 0;                   //  Turn off any HSIU trimming.
	mov	0x50cc+0, #0x00
;	../src/main.c: 60: CLK->SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
	mov	0x50cd+0, #0x00
;	../src/main.c: 61: CLK->SWR = 0xe1;                     //  Use HSI as the clock source.
	mov	0x50c4+0, #0xe1
;	../src/main.c: 62: CLK->SWCR = 0;                       //  Reset the clock switch control register.
	mov	0x50c5+0, #0x00
;	../src/main.c: 63: CLK->SWCR |= CLK_SWCR_SWEN;          //  Enable switching.
	bset	20677, #1
;	../src/main.c: 64: while (CLK->SWCR & CLK_SWCR_SWBSY);  //  Pause while the clock switch is busy.
00104$:
	ld	a, 0x50c5
	srl	a
	jrc	00104$
;	../src/main.c: 65: }
	ret
;	../src/main.c: 70: void InitialiseIWDG()
;	-----------------------------------------
;	 function InitialiseIWDG
;	-----------------------------------------
_InitialiseIWDG:
;	../src/main.c: 72: IWDG->KR = 0xcc;         //  Start the independent watchdog.
	mov	0x50e0+0, #0xcc
;	../src/main.c: 73: IWDG->KR = 0x55;         //  Allow the IWDG registers to be programmed.
	mov	0x50e0+0, #0x55
;	../src/main.c: 74: IWDG->PR = 0x06;         //  Prescaler is 6 => each count is 1.02 second with RLR = 0xff
	mov	0x50e1+0, #0x06
;	../src/main.c: 75: IWDG->RLR = 0xff;        //  Reload counter.  T = 2 x TLSI x PR x R LR
	mov	0x50e2+0, #0xff
;	../src/main.c: 76: IWDG->KR = 0xaa;         //  Reset the counter.
	mov	0x50e0+0, #0xaa
;	../src/main.c: 77: }
	ret
;	../src/main.c: 79: void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, GPIO_Mode_TypeDef GPIO_Mode)
;	-----------------------------------------
;	 function GPIO_Init
;	-----------------------------------------
_GPIO_Init:
	sub	sp, #5
;	../src/main.c: 82: GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
	ldw	y, (0x08, sp)
	ldw	x, y
	addw	x, #0x0004
	ldw	(0x04, sp), x
	ld	a, (x)
	push	a
	ld	a, (0x0b, sp)
	cpl	a
	ld	(0x02, sp), a
	pop	a
	and	a, (0x01, sp)
	ldw	x, (0x04, sp)
	ld	(x), a
;	../src/main.c: 93: GPIOx->DDR |= (uint8_t)GPIO_Pin;
	ldw	x, y
	incw	x
	incw	x
	ldw	(0x02, sp), x
;	../src/main.c: 86: if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x80) != (uint8_t)0x00) /* Output mode */
	tnz	(0x0b, sp)
	jrpl	00105$
;	../src/main.c: 89: GPIOx->ODR |= (uint8_t)GPIO_Pin;
	ld	a, (y)
;	../src/main.c: 88: if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x10) != (uint8_t)0x00) /* High level */
	push	a
	ld	a, (0x0c, sp)
	bcp	a, #0x10
	pop	a
	jreq	00102$
;	../src/main.c: 89: GPIOx->ODR |= (uint8_t)GPIO_Pin;
	or	a, (0x0a, sp)
	ld	(y), a
	jra	00103$
00102$:
;	../src/main.c: 91: GPIOx->ODR &= (uint8_t)(~(GPIO_Pin));
	and	a, (0x01, sp)
	ld	(y), a
00103$:
;	../src/main.c: 93: GPIOx->DDR |= (uint8_t)GPIO_Pin;
	ldw	x, (0x02, sp)
	ld	a, (x)
	or	a, (0x0a, sp)
	ldw	x, (0x02, sp)
	ld	(x), a
	jra	00106$
00105$:
;	../src/main.c: 96: GPIOx->DDR &= (uint8_t)(~(GPIO_Pin));
	ldw	x, (0x02, sp)
	ld	a, (x)
	and	a, (0x01, sp)
	ldw	x, (0x02, sp)
	ld	(x), a
00106$:
;	../src/main.c: 101: GPIOx->CR1 |= (uint8_t)GPIO_Pin;
	ldw	x, y
	addw	x, #0x0003
	ld	a, (x)
;	../src/main.c: 100: if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x40) != (uint8_t)0x00) /* Pull-Up or Push-Pull */
	push	a
	ld	a, (0x0c, sp)
	bcp	a, #0x40
	pop	a
	jreq	00108$
;	../src/main.c: 101: GPIOx->CR1 |= (uint8_t)GPIO_Pin;
	or	a, (0x0a, sp)
	ld	(x), a
	jra	00109$
00108$:
;	../src/main.c: 103: GPIOx->CR1 &= (uint8_t)(~(GPIO_Pin));
	and	a, (0x01, sp)
	ld	(x), a
00109$:
;	../src/main.c: 82: GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
	ldw	x, (0x04, sp)
	ld	a, (x)
;	../src/main.c: 107: if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x20) != (uint8_t)0x00) /* Interrupt or Slow slope */
	push	a
	ld	a, (0x0c, sp)
	bcp	a, #0x20
	pop	a
	jreq	00111$
;	../src/main.c: 108: GPIOx->CR2 |= (uint8_t)GPIO_Pin;
	or	a, (0x0a, sp)
	ldw	x, (0x04, sp)
	ld	(x), a
	jra	00113$
00111$:
;	../src/main.c: 110: GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
	and	a, (0x01, sp)
	ldw	x, (0x04, sp)
	ld	(x), a
00113$:
;	../src/main.c: 111: }
	addw	sp, #5
	ret
;	../src/main.c: 113: void measure(uint8_t tell)	// the measure() function talks to the AM2320 via i2c
;	-----------------------------------------
;	 function measure
;	-----------------------------------------
_measure:
	sub	sp, #34
;	../src/main.c: 118: if (i2c_start())	// have to check to see if the start was successful
	call	_i2c_start
	tnz	a
	jrne	00160$
	jp	00106$
00160$:
;	../src/main.c: 120: i2c_write_addr(AM2320_ADDR | I2C_WRITE);  // this is just to wake the AM2320 up
	push	#0xb8
	call	_i2c_write_addr
	pop	a
;	../src/main.c: 121: i2c_stop();
	call	_i2c_stop
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	y
	clr	a
	ld	xl, a
	clr	a
00114$:
	push	a
	cpw	y, #0x3408
	ld	a, xl
	sbc	a, #0x00
	ld	a, (1, sp)
	sbc	a, #0x00
	pop	a
	jrnc	00108$
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	addw	y, #0x0001
	push	a
	ld	a, xl
	adc	a, #0x00
	ld	xl, a
	pop	a
	adc	a, #0x00
	jra	00114$
;	../src/main.c: 123: delay_ms(15);	// the AM2320 needs this time to initialize itselt
00108$:
;	../src/main.c: 125: i2c_start();								// now we ask for a reading
	call	_i2c_start
;	../src/main.c: 126: i2c_write_addr(AM2320_ADDR | I2C_WRITE);
	push	#0xb8
	call	_i2c_write_addr
	pop	a
;	../src/main.c: 127: i2c_write(0x03);	// the the AM2320 we want 4 bytes from address 0
	push	#0x03
	call	_i2c_write
	pop	a
;	../src/main.c: 128: i2c_write(0x00);
	push	#0x00
	call	_i2c_write
	pop	a
;	../src/main.c: 129: i2c_write(0x04);
	push	#0x04
	call	_i2c_write
	pop	a
;	../src/main.c: 130: i2c_stop();
	call	_i2c_stop
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	y
	clr	a
	ld	xl, a
	clr	a
00117$:
	push	a
	cpw	y, #0x06f0
	ld	a, xl
	sbc	a, #0x00
	ld	a, (1, sp)
	sbc	a, #0x00
	pop	a
	jrnc	00110$
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	addw	y, #0x0001
	push	a
	ld	a, xl
	adc	a, #0x00
	ld	xl, a
	pop	a
	adc	a, #0x00
	jra	00117$
;	../src/main.c: 131: delay_ms(2);
00110$:
;	../src/main.c: 134: i2c_start();
	call	_i2c_start
;	../src/main.c: 135: i2c_write_addr(AM2320_ADDR | I2C_READ);
	push	#0xb9
	call	_i2c_write_addr
	pop	a
;	../src/main.c: 136: i2c_read_arr(buf, 6);	// the first two bytes are useless
	ldw	x, #_buf+0
	ldw	(0x11, sp), x
	push	#0x06
	pushw	x
	call	_i2c_read_arr
	addw	sp, #3
;	../src/main.c: 137: humidity = (buf[2] << 8) + buf[3];	// get the 16-bit humidity
	ldw	x, (0x11, sp)
	ld	a, (0x2, x)
	ld	xh, a
	clr	(0x10, sp)
	ldw	y, (0x11, sp)
	ld	a, (0x3, y)
	clr	(0x05, sp)
	add	a, (0x10, sp)
	rlwa	x
	adc	a, (0x05, sp)
	ld	xh, a
	ldw	(0x21, sp), x
;	../src/main.c: 138: temp = (buf[4] << 8) + buf[5];		// and the 16-bit temperature
	ldw	x, (0x11, sp)
	ld	a, (0x4, x)
	ld	(0x02, sp), a
	clr	(0x01, sp)
	ld	a, (0x02, sp)
	clr	(0x0a, sp)
	ldw	x, (0x11, sp)
	push	a
	ld	a, (0x5, x)
	ld	xl, a
	pop	a
	clr	(0x07, sp)
	push	a
	ld	a, xl
	add	a, (0x0b, sp)
	ld	xl, a
	pop	a
	adc	a, (0x07, sp)
	ld	xh, a
;	../src/main.c: 139: temp = temp * 1.8 + 320;	// convert temperature to fahrenheit
	pushw	x
	call	___uint2fs
	addw	sp, #2
	pushw	x
	pushw	y
	push	#0x66
	push	#0x66
	push	#0xe6
	push	#0x3f
	call	___fsmul
	addw	sp, #8
	push	#0x00
	push	#0x00
	push	#0xa0
	push	#0x43
	pushw	x
	pushw	y
	call	___fsadd
	addw	sp, #8
	pushw	x
	pushw	y
	call	___fs2uint
	addw	sp, #4
	ldw	(0x1f, sp), x
;	../src/main.c: 140: if (buf[4] & 0x80)  // is it negative?
	tnz	(0x02, sp)
	jrpl	00102$
;	../src/main.c: 141: temp *= -1;
	ldw	x, (0x1f, sp)
	pushw	x
	push	#0xff
	push	#0xff
	call	__mulint
	addw	sp, #4
	ldw	(0x1f, sp), x
00102$:
;	../src/main.c: 142: if (tell)	// if tell is set, transmit via rs485
	tnz	(0x25, sp)
	jrne	00164$
	jp	00104$
00164$:
;	../src/main.c: 144: rs485xmit_on();	// turn the RS485 chips transmitter on
	ld	a, 0x500f
	or	a, #0x10
	ld	0x500f, a
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	clr	(0x1c, sp)
	clr	(0x1b, sp)
00120$:
	cpw	x, #0x6810
	ld	a, (0x1c, sp)
	sbc	a, #0x00
	ld	a, (0x1b, sp)
	sbc	a, #0x00
	jrnc	00112$
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	addw	x, #0x0001
	ld	a, (0x1c, sp)
	adc	a, #0x00
	ld	yl, a
	ld	a, (0x1b, sp)
	adc	a, #0x00
	ld	yh, a
	ldw	(0x1b, sp), y
	jra	00120$
;	../src/main.c: 145: delay_ms(30);	// wait for everything to be ready
00112$:
;	../src/main.c: 146: printf("%c:%2d\.%1d%%:%3d\.%1d\r\n",address,humidity / 10,humidity % 10, temp / 10, temp %10);
	ldw	x, (0x1f, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x0d, sp), y
	ldw	x, (0x1f, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x0b, sp), x
	ldw	x, (0x21, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x03, sp), y
	ldw	x, (0x21, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x19, sp), x
	ld	a, _address+0
	ld	(0x18, sp), a
	clr	(0x17, sp)
	ldw	x, #___str_0+0
	ldw	(0x15, sp), x
	ldw	y, x
	ldw	(0x13, sp), y
	ldw	x, (0x0d, sp)
	pushw	x
	ldw	x, (0x0d, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x1f, sp)
	pushw	x
	ldw	x, (0x1f, sp)
	pushw	x
	ldw	x, (0x1d, sp)
	pushw	x
	call	_printf
	addw	sp, #12
;	../src/main.c: 147: rs485xmit_off(); // turn the transmitter back off
	ld	a, 0x500f
	and	a, #0xef
	ld	0x500f, a
00104$:
;	../src/main.c: 149: reset_watchdog();  // reset the watchdog timer
	mov	0x50e0+0, #0xaa
00106$:
;	../src/main.c: 152: reset_watchdog();  // reset the watchdog timer
	mov	0x50e0+0, #0xaa
;	../src/main.c: 153: last_measure = Global_time;	// when was the last time we transmitted anything
	ldw	x, _Global_time+2
	ldw	y, _Global_time+0
	ldw	_last_measure+2, x
	ldw	_last_measure+0, y
;	../src/main.c: 154: }
	addw	sp, #34
	ret
;	../src/main.c: 156: void main() {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	sub	sp, #93
;	../src/main.c: 157: unsigned long esc_time = 0L;
	clrw	x
	ldw	(0x5c, sp), x
	ldw	(0x5a, sp), x
;	../src/main.c: 161: esc = 0;
	clr	_esc+0
;	../src/main.c: 163: sprintf(version,"%02d%02d%02d-%02d%02d", BUILD_YEAR - 2000, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN);
	ldw	x, #___str_3+0
	ldw	(0x12, sp), x
	ld	a, (x)
	ld	(0x0d, sp), a
	ld	a, (0x0d, sp)
	cp	a, #0x3f
	jrne	00384$
	ld	a, #0x01
	ld	(0x0c, sp), a
	jra	00385$
00384$:
	clr	(0x0c, sp)
00385$:
	tnz	(0x0c, sp)
	jreq	00139$
	ldw	x, #0x0063
	ldw	(0x10, sp), x
	jra	00140$
00139$:
	ldw	x, (0x12, sp)
	ld	a, (0x3, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	sllw	x
	sllw	x
	addw	x, (1, sp)
	sllw	x
	addw	sp, #2
	ldw	(0x0e, sp), x
	ldw	x, (0x12, sp)
	ld	a, (0x4, x)
	clrw	x
	ld	xl, a
	addw	x, (0x0e, sp)
	subw	x, #0x0030
	ldw	(0x10, sp), x
00140$:
	tnz	(0x0c, sp)
	jreq	00141$
	ldw	x, #0x0063
	ldw	(0x08, sp), x
	jra	00142$
00141$:
	clrw	x
	ld	a, (0x0d, sp)
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	sllw	x
	sllw	x
	addw	x, (1, sp)
	sllw	x
	addw	sp, #2
	ldw	(0x06, sp), x
	ldw	x, (0x12, sp)
	ld	a, (0x1, x)
	clrw	x
	ld	xl, a
	addw	x, (0x06, sp)
	subw	x, #0x0030
	ldw	(0x04, sp), x
	ldw	y, x
	ldw	(0x08, sp), y
00142$:
	ldw	x, #___str_2+0
	ldw	(0x02, sp), x
	ld	a, (x)
	ld	(0x0b, sp), a
	ld	a, (0x0b, sp)
	cp	a, #0x3f
	jrne	00389$
	ld	a, #0x01
	ld	(0x0a, sp), a
	jra	00390$
00389$:
	clr	(0x0a, sp)
00390$:
	tnz	(0x0a, sp)
	jreq	00143$
	ldw	x, #0x0063
	ldw	(0x15, sp), x
	jra	00144$
00143$:
	ldw	x, (0x02, sp)
	ld	a, (0x4, x)
	ld	(0x14, sp), a
	ld	a, (0x14, sp)
	cp	a, #0x30
	jrc	00145$
	clrw	x
	ld	a, (0x14, sp)
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	sllw	x
	sllw	x
	addw	x, (1, sp)
	sllw	x
	addw	sp, #2
	ldw	(0x19, sp), x
	jra	00146$
00145$:
	clrw	x
	ldw	(0x19, sp), x
00146$:
	ldw	x, (0x02, sp)
	ld	a, (0x5, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	addw	x, (0x19, sp)
	ldw	(0x15, sp), x
00144$:
	tnz	(0x0a, sp)
	jreq	00147$
	ldw	x, #0x0063
	ldw	(0x17, sp), x
	jp	00148$
00147$:
	ld	a, (0x0b, sp)
	cp	a, #0x4a
	jrne	00395$
	ld	a, #0x01
	ld	(0x1d, sp), a
	jra	00396$
00395$:
	clr	(0x1d, sp)
00396$:
	ldw	x, (0x02, sp)
	incw	x
	ldw	(0x1b, sp), x
	ldw	x, (0x02, sp)
	incw	x
	incw	x
	ldw	(0x20, sp), x
	tnz	(0x1d, sp)
	jreq	00149$
	ldw	x, (0x1b, sp)
	ld	a, (x)
	cp	a, #0x61
	jrne	00149$
	ldw	x, (0x20, sp)
	ld	a, (x)
	cp	a, #0x6e
	jrne	00149$
	clrw	x
	incw	x
	jp	00150$
00149$:
	ld	a, (0x0b, sp)
	cp	a, #0x46
	jrne	00157$
	ldw	x, #0x0002
	ldw	(0x1e, sp), x
	jp	00158$
00157$:
	ld	a, (0x0b, sp)
	cp	a, #0x4d
	jrne	00408$
	ld	a, #0x01
	ld	(0x23, sp), a
	jra	00409$
00408$:
	clr	(0x23, sp)
00409$:
	tnz	(0x23, sp)
	jreq	00159$
	ldw	x, (0x1b, sp)
	ld	a, (x)
	cp	a, #0x61
	jrne	00159$
	ldw	x, (0x20, sp)
	ld	a, (x)
	cp	a, #0x72
	jrne	00159$
	ldw	x, #0x0003
	jp	00160$
00159$:
	ld	a, (0x0b, sp)
	cp	a, #0x41
	jrne	00418$
	ld	a, #0x01
	ld	(0x22, sp), a
	jra	00419$
00418$:
	clr	(0x22, sp)
00419$:
	tnz	(0x22, sp)
	jreq	00167$
	ldw	x, (0x1b, sp)
	ld	a, (x)
	cp	a, #0x70
	jrne	00167$
	ldw	x, #0x0004
	ldw	(0x26, sp), x
	jp	00168$
00167$:
	tnz	(0x23, sp)
	jreq	00172$
	ldw	x, (0x1b, sp)
	ld	a, (x)
	cp	a, #0x61
	jrne	00172$
	ldw	x, (0x20, sp)
	ld	a, (x)
	cp	a, #0x79
	jrne	00172$
	ldw	x, #0x0005
	ldw	(0x24, sp), x
	jp	00173$
00172$:
	tnz	(0x1d, sp)
	jreq	00180$
	ldw	x, (0x1b, sp)
	ld	a, (x)
	cp	a, #0x75
	jrne	00180$
	ldw	x, (0x20, sp)
	ld	a, (x)
	cp	a, #0x6e
	jrne	00180$
	ldw	x, #0x0006
	jra	00181$
00180$:
	tnz	(0x1d, sp)
	jreq	00188$
	ldw	x, (0x1b, sp)
	ld	a, (x)
	cp	a, #0x75
	jrne	00188$
	ldw	x, (0x20, sp)
	ld	a, (x)
	cp	a, #0x6c
	jrne	00188$
	ldw	x, #0x0007
	ldw	(0x2a, sp), x
	jra	00189$
00188$:
	tnz	(0x22, sp)
	jreq	00196$
	ldw	x, (0x1b, sp)
	ld	a, (x)
	cp	a, #0x75
	jrne	00196$
	ldw	x, #0x0008
	jra	00197$
00196$:
	ld	a, (0x0b, sp)
	cp	a, #0x53
	jrne	00201$
	ldw	x, #0x0009
	ldw	(0x28, sp), x
	jra	00202$
00201$:
	ld	a, (0x0b, sp)
	cp	a, #0x4f
	jrne	00203$
	ldw	x, #0x000a
	jra	00204$
00203$:
	ld	a, (0x0b, sp)
	cp	a, #0x4e
	jrne	00205$
	ldw	x, #0x000b
	ldw	(0x2e, sp), x
	jra	00206$
00205$:
	ld	a, (0x0b, sp)
	cp	a, #0x44
	jrne	00207$
	ldw	x, #0x000c
	jra	00208$
00207$:
	ldw	x, #0x0063
00208$:
	ldw	(0x2e, sp), x
00206$:
	ldw	x, (0x2e, sp)
00204$:
	ldw	(0x28, sp), x
00202$:
	ldw	x, (0x28, sp)
00197$:
	ldw	(0x2a, sp), x
00189$:
	ldw	x, (0x2a, sp)
00181$:
	ldw	(0x24, sp), x
00173$:
	ldw	y, (0x24, sp)
	ldw	(0x26, sp), y
00168$:
	ldw	x, (0x26, sp)
00160$:
	ldw	(0x1e, sp), x
00158$:
	ldw	x, (0x1e, sp)
00150$:
	ldw	(0x17, sp), x
00148$:
	tnz	(0x0a, sp)
	jreq	00209$
	ldw	x, #0x0063
	jra	00210$
00209$:
	ldw	x, (0x02, sp)
	ld	a, (0x7, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	push	#0xe8
	push	#0x03
	call	__mulint
	addw	sp, #4
	ldw	(0x2c, sp), x
	ldw	x, (0x02, sp)
	ld	a, (0x8, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	push	#0x64
	push	#0x00
	call	__mulint
	addw	sp, #4
	addw	x, (0x2c, sp)
	ldw	(0x32, sp), x
	ldw	x, (0x02, sp)
	ld	a, (0x9, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	sllw	x
	sllw	x
	addw	x, (1, sp)
	sllw	x
	addw	sp, #2
	addw	x, (0x32, sp)
	ldw	(0x30, sp), x
	ldw	x, (0x02, sp)
	ld	a, (0xa, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	addw	x, (0x30, sp)
	ldw	(0x36, sp), x
00210$:
	subw	x, #0x07d0
	ldw	(0x34, sp), x
	ldw	x, #___str_1+0
	ldw	(0x3a, sp), x
	ldw	x, #_version+0
	ldw	(0x38, sp), x
	ldw	y, x
	ldw	x, (0x10, sp)
	pushw	x
	ldw	x, (0x0a, sp)
	pushw	x
	ldw	x, (0x19, sp)
	pushw	x
	ldw	x, (0x1d, sp)
	pushw	x
	ldw	x, (0x3c, sp)
	pushw	x
	ldw	x, (0x44, sp)
	pushw	x
	pushw	y
	call	_sprintf
	addw	sp, #14
;	../src/main.c: 165: disableInterrupts();
	sim
;	../src/main.c: 166: InitialiseSystemClock();
	call	_InitialiseSystemClock
;	../src/main.c: 167: InitialiseIWDG();		// not really necessary, but what the heck...
	call	_InitialiseIWDG
;	../src/main.c: 168: GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);  // rs485 xmit indicator
	push	#0xe0
	push	#0x10
	push	#0x0f
	push	#0x50
	call	_GPIO_Init
	addw	sp, #4
;	../src/main.c: 170: rs485xmit_off();	// turn off the D4 pin so we can receive rs485 data
	bres	20495, #4
;	../src/main.c: 175: CFG->GCR |= 1; // disable SWIM
	ld	a, 0x7f60
	clrw	x
	ld	xl, a
	srlw	x
	scf
	rlcw	x
	ld	a, xl
	ld	0x7f60, a
;	../src/main.c: 178: TIM4->PSCR = 7;   // prescaler
	mov	0x5347+0, #0x07
;	../src/main.c: 179: TIM4->ARR = 125;  // auto reload register
	mov	0x5348+0, #0x7d
;	../src/main.c: 181: TIM4->IER = TIM4_IER_UIE;
	mov	0x5343+0, #0x01
;	../src/main.c: 183: TIM4->CR1 = TIM4_CR1_ARPE | TIM4_CR1_URS | TIM4_CR1_CEN;
	mov	0x5340+0, #0x85
;	../src/main.c: 185: reset_watchdog();  // reset the watchdog timer
	mov	0x50e0+0, #0xaa
;	../src/main.c: 187: Global_time = 0L;	// used as an internal clock
	clrw	x
	ldw	_Global_time+2, x
	ldw	_Global_time+0, x
;	../src/main.c: 188: uart_init();		// setup for 9600 8-N-1
	call	_uart_init
;	../src/main.c: 189: i2c_init();			// talk to the AM2320 and the MCP23017 to read switches
	call	_i2c_init
;	../src/main.c: 190: reset_watchdog();
	mov	0x50e0+0, #0xaa
;	../src/main.c: 192: enableInterrupts();
	rim
;	../src/main.c: 193: reset_watchdog();  // reset the watchdog timer
	mov	0x50e0+0, #0xaa
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	ldw	(0x54, sp), x
	ldw	(0x52, sp), x
00126$:
	ldw	x, (0x54, sp)
	cpw	x, #0x6330
	ld	a, (0x53, sp)
	sbc	a, #0x03
	ld	a, (0x52, sp)
	sbc	a, #0x00
	jrnc	00118$
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x54, sp)
	addw	y, #0x0001
	ld	a, (0x53, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x52, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x54, sp), y
	ldw	(0x52, sp), x
	jra	00126$
;	../src/main.c: 195: delay_ms(250);
00118$:
;	../src/main.c: 196: rs485xmit_on();	// turn the RS485 chips transmitter on
	ld	a, 0x500f
	or	a, #0x10
	ld	0x500f, a
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	ldw	(0x58, sp), x
	ldw	(0x56, sp), x
00129$:
	ldw	x, (0x58, sp)
	cpw	x, #0x6810
	ld	a, (0x57, sp)
	sbc	a, #0x00
	ld	a, (0x56, sp)
	sbc	a, #0x00
	jrnc	00120$
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x58, sp)
	addw	y, #0x0001
	ld	a, (0x57, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x56, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x58, sp), y
	ldw	(0x56, sp), x
	jra	00129$
;	../src/main.c: 197: delay_ms(30);
00120$:
;	../src/main.c: 198: printf("Initializing...%s\r\n",version);
	ldw	y, (0x38, sp)
	ldw	x, #___str_4+0
	pushw	y
	pushw	x
	call	_printf
	addw	sp, #4
;	../src/main.c: 199: rs485xmit_off(); // turn the transmitter back off
	ld	a, 0x500f
	and	a, #0xef
	ld	0x500f, a
;	../src/main.c: 204: address = init_mcp23017();
	call	_init_mcp23017
	ld	_address+0, a
;	../src/main.c: 206: reset_watchdog();  // reset the watchdog timer
	mov	0x50e0+0, #0xaa
;	../src/main.c: 208: delay_ms(50 * (address - 0x30));	//  do this to stagger startup announcements
	clrw	x
	ld	a, _address+0
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	push	#0x32
	push	#0x00
	call	__mulint
	addw	sp, #4
	clrw	y
	tnzw	x
	jrpl	00464$
	decw	y
00464$:
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clr	(0x51, sp)
	clr	(0x50, sp)
	clr	(0x4f, sp)
	clr	(0x4e, sp)
	pushw	x
	pushw	y
	push	#0x78
	push	#0x03
	clrw	x
	pushw	x
	call	__mullong
	addw	sp, #8
	ldw	(0x40, sp), x
	ldw	(0x3e, sp), y
00132$:
	ldw	x, (0x50, sp)
	cpw	x, (0x40, sp)
	ld	a, (0x4f, sp)
	sbc	a, (0x3f, sp)
	ld	a, (0x4e, sp)
	sbc	a, (0x3e, sp)
	jrnc	00122$
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x50, sp)
	addw	y, #0x0001
	ld	a, (0x4f, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x4e, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x50, sp), y
	ldw	(0x4e, sp), x
	jra	00132$
;	../src/main.c: 208: delay_ms(50 * (address - 0x30));	//  do this to stagger startup announcements
00122$:
;	../src/main.c: 209: reset_watchdog();  // reset the watchdog timer
	mov	0x50e0+0, #0xaa
;	../src/main.c: 210: rs485xmit_on();	// turn the RS485 chips transmitter on
	ld	a, 0x500f
	or	a, #0x10
	ld	0x500f, a
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	ldw	(0x4c, sp), x
	ldw	(0x4a, sp), x
00135$:
	ldw	x, (0x4c, sp)
	cpw	x, #0x6810
	ld	a, (0x4b, sp)
	sbc	a, #0x00
	ld	a, (0x4a, sp)
	sbc	a, #0x00
	jrnc	00124$
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x4c, sp)
	addw	y, #0x0001
	ld	a, (0x4b, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x4a, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x4c, sp), y
	ldw	(0x4a, sp), x
	jra	00135$
;	../src/main.c: 211: delay_ms(30);
00124$:
;	../src/main.c: 212: printf("%c:Running:%s:%02x\r\n",address,version,address);
	clrw	x
	ld	a, _address+0
	ld	xl, a
	ldw	y, (0x38, sp)
	ldw	(0x3c, sp), y
	ldw	y, #___str_5+0
	pushw	x
	ld	a, (0x3f, sp)
	push	a
	ld	a, (0x3f, sp)
	push	a
	pushw	x
	pushw	y
	call	_printf
	addw	sp, #8
;	../src/main.c: 213: rs485xmit_off(); // turn the transmitter back off
	ld	a, 0x500f
	and	a, #0xef
	ld	0x500f, a
;	../src/main.c: 216: do{
00114$:
;	../src/main.c: 217: reset_watchdog();  // reset the watchdog timer
	mov	0x50e0+0, #0xaa
;	../src/main.c: 218: if(UART_read_byte(&rb)){ // buffer isn't empty
	ldw	x, sp
	incw	x
	pushw	x
	call	_UART_read_byte
	addw	sp, #2
	tnz	a
	jreq	00108$
;	../src/main.c: 219: switch(rb){
	ld	a, (0x01, sp)
	cp	a, #0x1b
	jrne	00102$
;	../src/main.c: 221: esc = 1;	// set the flag to show an escpage character was received
	mov	_esc+0, #0x01
;	../src/main.c: 222: esc_time = Global_time;	// only wait 2 seconds for the next character after the escape
	ldw	x, _Global_time+2
	ldw	(0x5c, sp), x
	ldw	x, _Global_time+0
	ldw	(0x5a, sp), x
;	../src/main.c: 223: break;
	jra	00108$
;	../src/main.c: 224: default:
00102$:
;	../src/main.c: 225: if (rb == address && esc)  // address must match the switches read by mcp23017
	ld	a, (0x01, sp)
	cp	a, _address+0
	jrne	00104$
	tnz	_esc+0
	jreq	00104$
;	../src/main.c: 227: Global_time = 0L;   // when was the last time we were called?
	clrw	x
	ldw	_Global_time+2, x
	ldw	_Global_time+0, x
;	../src/main.c: 228: measure(1);			// do a measurement, and send the results
	push	#0x01
	call	_measure
	pop	a
;	../src/main.c: 229: last_measure = Global_time;
	ldw	x, _Global_time+2
	ldw	y, _Global_time+0
	ldw	_last_measure+2, x
	ldw	_last_measure+0, y
00104$:
;	../src/main.c: 231: esc = 0;	// reset the escape character flag
	clr	_esc+0
;	../src/main.c: 232: }
00108$:
;	../src/main.c: 234: if (esc && (Global_time - esc_time > 2000))  // give it 2 seconds
	tnz	_esc+0
	jreq	00110$
	ldw	x, _Global_time+2
	subw	x, (0x5c, sp)
	ldw	(0x48, sp), x
	ld	a, _Global_time+1
	sbc	a, (0x5b, sp)
	ld	(0x47, sp), a
	ld	a, _Global_time+0
	sbc	a, (0x5a, sp)
	ld	(0x46, sp), a
	ldw	x, #0x07d0
	cpw	x, (0x48, sp)
	clr	a
	sbc	a, (0x47, sp)
	clr	a
	sbc	a, (0x46, sp)
	jrnc	00110$
;	../src/main.c: 235: esc = 0;  // reset the esc, since it should have been followed by the id right away
	clr	_esc+0
00110$:
;	../src/main.c: 236: if (Global_time - last_measure > 600000L)  // every 10 minutes take a silent measurement
	ldw	x, _Global_time+2
	subw	x, _last_measure+2
	ldw	(0x44, sp), x
	ld	a, _Global_time+1
	sbc	a, _last_measure+1
	ld	(0x43, sp), a
	ld	a, _Global_time+0
	sbc	a, _last_measure+0
	ld	(0x42, sp), a
	ldw	x, #0x27c0
	cpw	x, (0x44, sp)
	ld	a, #0x09
	sbc	a, (0x43, sp)
	clr	a
	sbc	a, (0x42, sp)
	jrc	00477$
	jp	00114$
00477$:
;	../src/main.c: 238: measure(0);
	push	#0x00
	call	_measure
	pop	a
;	../src/main.c: 239: last_measure = Global_time;
	ldw	x, _Global_time+2
	ldw	y, _Global_time+0
	ldw	_last_measure+2, x
	ldw	_last_measure+0, y
;	../src/main.c: 241: }while(1);
	jp	00114$
;	../src/main.c: 242: }
	addw	sp, #93
	ret
	.area CODE
___str_0:
	.ascii "%c:%2d.%1d%%:%3d.%1d"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_1:
	.ascii "%02d%02d%02d-%02d%02d"
	.db 0x00
___str_2:
	.ascii "Oct  5 2018"
	.db 0x00
___str_3:
	.ascii "23:14:42"
	.db 0x00
___str_4:
	.ascii "Initializing...%s"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_5:
	.ascii "%c:Running:%s:%02x"
	.db 0x0d
	.db 0x0a
	.db 0x00
	.area INITIALIZER
__xinit__last_measure:
	.byte #0x00,#0x00,#0x00,#0x00	; 0
	.area CABS (ABS)
