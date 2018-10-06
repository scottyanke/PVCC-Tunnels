                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.7.0 #10231 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module main
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _main
                                     12 	.globl _measure
                                     13 	.globl _InitialiseIWDG
                                     14 	.globl _InitialiseSystemClock
                                     15 	.globl _init_mcp23017
                                     16 	.globl _i2c_read_arr
                                     17 	.globl _i2c_write_addr
                                     18 	.globl _i2c_write
                                     19 	.globl _i2c_stop
                                     20 	.globl _i2c_start
                                     21 	.globl _i2c_init
                                     22 	.globl _UART_read_byte
                                     23 	.globl _uart_write
                                     24 	.globl _uart_init
                                     25 	.globl _sprintf
                                     26 	.globl _printf
                                     27 	.globl _last_measure
                                     28 	.globl _version
                                     29 	.globl _address
                                     30 	.globl _esc
                                     31 	.globl _buf
                                     32 	.globl _light_time
                                     33 	.globl _Global_time
                                     34 	.globl _putchar
                                     35 	.globl _GPIO_Init
                                     36 ;--------------------------------------------------------
                                     37 ; ram data
                                     38 ;--------------------------------------------------------
                                     39 	.area DATA
      000001                         40 _Global_time::
      000001                         41 	.ds 4
      000005                         42 _light_time::
      000005                         43 	.ds 4
      000009                         44 _buf::
      000009                         45 	.ds 20
      00001D                         46 _esc::
      00001D                         47 	.ds 1
      00001E                         48 _address::
      00001E                         49 	.ds 1
      00001F                         50 _version::
      00001F                         51 	.ds 12
                                     52 ;--------------------------------------------------------
                                     53 ; ram data
                                     54 ;--------------------------------------------------------
                                     55 	.area INITIALIZED
      000049                         56 _last_measure::
      000049                         57 	.ds 4
                                     58 ;--------------------------------------------------------
                                     59 ; Stack segment in internal ram 
                                     60 ;--------------------------------------------------------
                                     61 	.area	SSEG
      FFFFFF                         62 __start__stack:
      FFFFFF                         63 	.ds	1
                                     64 
                                     65 ;--------------------------------------------------------
                                     66 ; absolute external ram data
                                     67 ;--------------------------------------------------------
                                     68 	.area DABS (ABS)
                                     69 ;--------------------------------------------------------
                                     70 ; interrupt vector 
                                     71 ;--------------------------------------------------------
                                     72 	.area HOME
      008000                         73 __interrupt_vect:
      008000 82 00 80 6F             74 	int s_GSINIT ; reset
      008004 82 00 00 00             75 	int 0x0000 ; trap
      008008 82 00 81 57             76 	int _TLI_IRQHandler ; int0
      00800C 82 00 81 58             77 	int _AWU_IRQHandler ; int1
      008010 82 00 81 59             78 	int _CLK_IRQHandler ; int2
      008014 82 00 81 5A             79 	int _EXTI_PORTA_IRQHandler ; int3
      008018 82 00 81 5B             80 	int _EXTI_PORTB_IRQHandler ; int4
      00801C 82 00 81 5C             81 	int _EXTI_PORTC_IRQHandler ; int5
      008020 82 00 81 5D             82 	int _EXTI_PORTD_IRQHandler ; int6
      008024 82 00 81 5E             83 	int _EXTI_PORTE_IRQHandler ; int7
      008028 82 00 00 00             84 	int 0x0000 ; int8
      00802C 82 00 00 00             85 	int 0x0000 ; int9
      008030 82 00 81 5F             86 	int _SPI_IRQHandler ; int10
      008034 82 00 81 60             87 	int _TIM1_UPD_OVF_TRG_BRK_IRQHandler ; int11
      008038 82 00 81 61             88 	int _TIM1_CAP_COM_IRQHandler ; int12
      00803C 82 00 81 62             89 	int _TIM2_UPD_OVF_BRK_IRQHandler ; int13
      008040 82 00 81 63             90 	int _TIM2_CAP_COM_IRQHandler ; int14
      008044 82 00 00 00             91 	int 0x0000 ; int15
      008048 82 00 00 00             92 	int 0x0000 ; int16
      00804C 82 00 81 64             93 	int _UART1_TX_IRQHandler ; int17
      008050 82 00 81 65             94 	int _UART1_RX_IRQHandler ; int18
      008054 82 00 81 AB             95 	int _I2C_IRQHandler ; int19
      008058 82 00 00 00             96 	int 0x0000 ; int20
      00805C 82 00 00 00             97 	int 0x0000 ; int21
      008060 82 00 81 AC             98 	int _ADC1_IRQHandler ; int22
      008064 82 00 81 AD             99 	int _TIM4_UPD_OVF_IRQHandler ; int23
      008068 82 00 81 D3            100 	int _EEPROM_EEC_IRQHandler ; int24
                                    101 ;--------------------------------------------------------
                                    102 ; global & static initialisations
                                    103 ;--------------------------------------------------------
                                    104 	.area HOME
                                    105 	.area GSINIT
                                    106 	.area GSFINAL
                                    107 	.area GSINIT
      00806F                        108 __sdcc_gs_init_startup:
      00806F                        109 __sdcc_init_data:
                                    110 ; stm8_genXINIT() start
      00806F AE 00 48         [ 2]  111 	ldw x, #l_DATA
      008072 27 07            [ 1]  112 	jreq	00002$
      008074                        113 00001$:
      008074 72 4F 00 00      [ 1]  114 	clr (s_DATA - 1, x)
      008078 5A               [ 2]  115 	decw x
      008079 26 F9            [ 1]  116 	jrne	00001$
      00807B                        117 00002$:
      00807B AE 00 06         [ 2]  118 	ldw	x, #l_INITIALIZER
      00807E 27 09            [ 1]  119 	jreq	00004$
      008080                        120 00003$:
      008080 D6 99 EC         [ 1]  121 	ld	a, (s_INITIALIZER - 1, x)
      008083 D7 00 48         [ 1]  122 	ld	(s_INITIALIZED - 1, x), a
      008086 5A               [ 2]  123 	decw	x
      008087 26 F7            [ 1]  124 	jrne	00003$
      008089                        125 00004$:
                                    126 ; stm8_genXINIT() end
                                    127 	.area GSFINAL
      008089 CC 80 6C         [ 2]  128 	jp	__sdcc_program_startup
                                    129 ;--------------------------------------------------------
                                    130 ; Home
                                    131 ;--------------------------------------------------------
                                    132 	.area HOME
                                    133 	.area HOME
      00806C                        134 __sdcc_program_startup:
      00806C CC 84 4B         [ 2]  135 	jp	_main
                                    136 ;	return from main will return to caller
                                    137 ;--------------------------------------------------------
                                    138 ; code
                                    139 ;--------------------------------------------------------
                                    140 	.area CODE
                                    141 ;	../src/main.c: 40: int putchar(int c) {
                                    142 ;	-----------------------------------------
                                    143 ;	 function putchar
                                    144 ;	-----------------------------------------
      0081D4                        145 _putchar:
                                    146 ;	../src/main.c: 41: uart_write(c);
      0081D4 7B 04            [ 1]  147 	ld	a, (0x04, sp)
      0081D6 88               [ 1]  148 	push	a
      0081D7 CD 8A 61         [ 4]  149 	call	_uart_write
      0081DA 84               [ 1]  150 	pop	a
                                    151 ;	../src/main.c: 42: return c;
      0081DB 1E 03            [ 2]  152 	ldw	x, (0x03, sp)
                                    153 ;	../src/main.c: 43: }
      0081DD 81               [ 4]  154 	ret
                                    155 ;	../src/main.c: 49: void InitialiseSystemClock()
                                    156 ;	-----------------------------------------
                                    157 ;	 function InitialiseSystemClock
                                    158 ;	-----------------------------------------
      0081DE                        159 _InitialiseSystemClock:
                                    160 ;	../src/main.c: 51: CLK->ICKR = 0;                       //  Reset the Internal Clock Register.
      0081DE 35 00 50 C0      [ 1]  161 	mov	0x50c0+0, #0x00
                                    162 ;	../src/main.c: 52: CLK->ICKR = CLK_ICKR_HSIEN;          //  Enable the HSI.
      0081E2 35 01 50 C0      [ 1]  163 	mov	0x50c0+0, #0x01
                                    164 ;	../src/main.c: 53: CLK->ECKR = 0;                       //  Disable the external clock.
      0081E6 35 00 50 C1      [ 1]  165 	mov	0x50c1+0, #0x00
                                    166 ;	../src/main.c: 54: while (!(CLK->ICKR & CLK_ICKR_HSIRDY)); //  Wait for the HSI to be ready for use.
      0081EA                        167 00101$:
      0081EA C6 50 C0         [ 1]  168 	ld	a, 0x50c0
      0081ED A5 02            [ 1]  169 	bcp	a, #0x02
      0081EF 27 F9            [ 1]  170 	jreq	00101$
                                    171 ;	../src/main.c: 55: CLK->CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
      0081F1 35 00 50 C6      [ 1]  172 	mov	0x50c6+0, #0x00
                                    173 ;	../src/main.c: 56: CLK->PCKENR1 = 0xff; //CLK_PCKENR1_TIM4 | CLK_PCKENR1_UART1 | CLK_PCKENR1_SPI | CLK_PCKENR1_I2C ;  //  Enable select peripheral clocks.
      0081F5 35 FF 50 C7      [ 1]  174 	mov	0x50c7+0, #0xff
                                    175 ;	../src/main.c: 57: CLK->PCKENR2 = 0xff; //CLK_PCKENR2_AWU;      //  Only enable the AWU watchdog service
      0081F9 35 FF 50 CA      [ 1]  176 	mov	0x50ca+0, #0xff
                                    177 ;	../src/main.c: 58: CLK->CCOR = 0;                       //  Turn off CCO.
      0081FD 35 00 50 C9      [ 1]  178 	mov	0x50c9+0, #0x00
                                    179 ;	../src/main.c: 59: CLK->HSITRIMR = 0;                   //  Turn off any HSIU trimming.
      008201 35 00 50 CC      [ 1]  180 	mov	0x50cc+0, #0x00
                                    181 ;	../src/main.c: 60: CLK->SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
      008205 35 00 50 CD      [ 1]  182 	mov	0x50cd+0, #0x00
                                    183 ;	../src/main.c: 61: CLK->SWR = 0xe1;                     //  Use HSI as the clock source.
      008209 35 E1 50 C4      [ 1]  184 	mov	0x50c4+0, #0xe1
                                    185 ;	../src/main.c: 62: CLK->SWCR = 0;                       //  Reset the clock switch control register.
      00820D 35 00 50 C5      [ 1]  186 	mov	0x50c5+0, #0x00
                                    187 ;	../src/main.c: 63: CLK->SWCR |= CLK_SWCR_SWEN;          //  Enable switching.
      008211 72 12 50 C5      [ 1]  188 	bset	20677, #1
                                    189 ;	../src/main.c: 64: while (CLK->SWCR & CLK_SWCR_SWBSY);  //  Pause while the clock switch is busy.
      008215                        190 00104$:
      008215 C6 50 C5         [ 1]  191 	ld	a, 0x50c5
      008218 44               [ 1]  192 	srl	a
      008219 25 FA            [ 1]  193 	jrc	00104$
                                    194 ;	../src/main.c: 65: }
      00821B 81               [ 4]  195 	ret
                                    196 ;	../src/main.c: 70: void InitialiseIWDG()
                                    197 ;	-----------------------------------------
                                    198 ;	 function InitialiseIWDG
                                    199 ;	-----------------------------------------
      00821C                        200 _InitialiseIWDG:
                                    201 ;	../src/main.c: 72: IWDG->KR = 0xcc;         //  Start the independent watchdog.
      00821C 35 CC 50 E0      [ 1]  202 	mov	0x50e0+0, #0xcc
                                    203 ;	../src/main.c: 73: IWDG->KR = 0x55;         //  Allow the IWDG registers to be programmed.
      008220 35 55 50 E0      [ 1]  204 	mov	0x50e0+0, #0x55
                                    205 ;	../src/main.c: 74: IWDG->PR = 0x06;         //  Prescaler is 6 => each count is 1.02 second with RLR = 0xff
      008224 35 06 50 E1      [ 1]  206 	mov	0x50e1+0, #0x06
                                    207 ;	../src/main.c: 75: IWDG->RLR = 0xff;        //  Reload counter.  T = 2 x TLSI x PR x R LR
      008228 35 FF 50 E2      [ 1]  208 	mov	0x50e2+0, #0xff
                                    209 ;	../src/main.c: 76: IWDG->KR = 0xaa;         //  Reset the counter.
      00822C 35 AA 50 E0      [ 1]  210 	mov	0x50e0+0, #0xaa
                                    211 ;	../src/main.c: 77: }
      008230 81               [ 4]  212 	ret
                                    213 ;	../src/main.c: 79: void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, GPIO_Mode_TypeDef GPIO_Mode)
                                    214 ;	-----------------------------------------
                                    215 ;	 function GPIO_Init
                                    216 ;	-----------------------------------------
      008231                        217 _GPIO_Init:
      008231 52 05            [ 2]  218 	sub	sp, #5
                                    219 ;	../src/main.c: 82: GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
      008233 16 08            [ 2]  220 	ldw	y, (0x08, sp)
      008235 93               [ 1]  221 	ldw	x, y
      008236 1C 00 04         [ 2]  222 	addw	x, #0x0004
      008239 1F 04            [ 2]  223 	ldw	(0x04, sp), x
      00823B F6               [ 1]  224 	ld	a, (x)
      00823C 88               [ 1]  225 	push	a
      00823D 7B 0B            [ 1]  226 	ld	a, (0x0b, sp)
      00823F 43               [ 1]  227 	cpl	a
      008240 6B 02            [ 1]  228 	ld	(0x02, sp), a
      008242 84               [ 1]  229 	pop	a
      008243 14 01            [ 1]  230 	and	a, (0x01, sp)
      008245 1E 04            [ 2]  231 	ldw	x, (0x04, sp)
      008247 F7               [ 1]  232 	ld	(x), a
                                    233 ;	../src/main.c: 93: GPIOx->DDR |= (uint8_t)GPIO_Pin;
      008248 93               [ 1]  234 	ldw	x, y
      008249 5C               [ 1]  235 	incw	x
      00824A 5C               [ 1]  236 	incw	x
      00824B 1F 02            [ 2]  237 	ldw	(0x02, sp), x
                                    238 ;	../src/main.c: 86: if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x80) != (uint8_t)0x00) /* Output mode */
      00824D 0D 0B            [ 1]  239 	tnz	(0x0b, sp)
      00824F 2A 1E            [ 1]  240 	jrpl	00105$
                                    241 ;	../src/main.c: 89: GPIOx->ODR |= (uint8_t)GPIO_Pin;
      008251 90 F6            [ 1]  242 	ld	a, (y)
                                    243 ;	../src/main.c: 88: if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x10) != (uint8_t)0x00) /* High level */
      008253 88               [ 1]  244 	push	a
      008254 7B 0C            [ 1]  245 	ld	a, (0x0c, sp)
      008256 A5 10            [ 1]  246 	bcp	a, #0x10
      008258 84               [ 1]  247 	pop	a
      008259 27 06            [ 1]  248 	jreq	00102$
                                    249 ;	../src/main.c: 89: GPIOx->ODR |= (uint8_t)GPIO_Pin;
      00825B 1A 0A            [ 1]  250 	or	a, (0x0a, sp)
      00825D 90 F7            [ 1]  251 	ld	(y), a
      00825F 20 04            [ 2]  252 	jra	00103$
      008261                        253 00102$:
                                    254 ;	../src/main.c: 91: GPIOx->ODR &= (uint8_t)(~(GPIO_Pin));
      008261 14 01            [ 1]  255 	and	a, (0x01, sp)
      008263 90 F7            [ 1]  256 	ld	(y), a
      008265                        257 00103$:
                                    258 ;	../src/main.c: 93: GPIOx->DDR |= (uint8_t)GPIO_Pin;
      008265 1E 02            [ 2]  259 	ldw	x, (0x02, sp)
      008267 F6               [ 1]  260 	ld	a, (x)
      008268 1A 0A            [ 1]  261 	or	a, (0x0a, sp)
      00826A 1E 02            [ 2]  262 	ldw	x, (0x02, sp)
      00826C F7               [ 1]  263 	ld	(x), a
      00826D 20 08            [ 2]  264 	jra	00106$
      00826F                        265 00105$:
                                    266 ;	../src/main.c: 96: GPIOx->DDR &= (uint8_t)(~(GPIO_Pin));
      00826F 1E 02            [ 2]  267 	ldw	x, (0x02, sp)
      008271 F6               [ 1]  268 	ld	a, (x)
      008272 14 01            [ 1]  269 	and	a, (0x01, sp)
      008274 1E 02            [ 2]  270 	ldw	x, (0x02, sp)
      008276 F7               [ 1]  271 	ld	(x), a
      008277                        272 00106$:
                                    273 ;	../src/main.c: 101: GPIOx->CR1 |= (uint8_t)GPIO_Pin;
      008277 93               [ 1]  274 	ldw	x, y
      008278 1C 00 03         [ 2]  275 	addw	x, #0x0003
      00827B F6               [ 1]  276 	ld	a, (x)
                                    277 ;	../src/main.c: 100: if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x40) != (uint8_t)0x00) /* Pull-Up or Push-Pull */
      00827C 88               [ 1]  278 	push	a
      00827D 7B 0C            [ 1]  279 	ld	a, (0x0c, sp)
      00827F A5 40            [ 1]  280 	bcp	a, #0x40
      008281 84               [ 1]  281 	pop	a
      008282 27 05            [ 1]  282 	jreq	00108$
                                    283 ;	../src/main.c: 101: GPIOx->CR1 |= (uint8_t)GPIO_Pin;
      008284 1A 0A            [ 1]  284 	or	a, (0x0a, sp)
      008286 F7               [ 1]  285 	ld	(x), a
      008287 20 03            [ 2]  286 	jra	00109$
      008289                        287 00108$:
                                    288 ;	../src/main.c: 103: GPIOx->CR1 &= (uint8_t)(~(GPIO_Pin));
      008289 14 01            [ 1]  289 	and	a, (0x01, sp)
      00828B F7               [ 1]  290 	ld	(x), a
      00828C                        291 00109$:
                                    292 ;	../src/main.c: 82: GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
      00828C 1E 04            [ 2]  293 	ldw	x, (0x04, sp)
      00828E F6               [ 1]  294 	ld	a, (x)
                                    295 ;	../src/main.c: 107: if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x20) != (uint8_t)0x00) /* Interrupt or Slow slope */
      00828F 88               [ 1]  296 	push	a
      008290 7B 0C            [ 1]  297 	ld	a, (0x0c, sp)
      008292 A5 20            [ 1]  298 	bcp	a, #0x20
      008294 84               [ 1]  299 	pop	a
      008295 27 07            [ 1]  300 	jreq	00111$
                                    301 ;	../src/main.c: 108: GPIOx->CR2 |= (uint8_t)GPIO_Pin;
      008297 1A 0A            [ 1]  302 	or	a, (0x0a, sp)
      008299 1E 04            [ 2]  303 	ldw	x, (0x04, sp)
      00829B F7               [ 1]  304 	ld	(x), a
      00829C 20 05            [ 2]  305 	jra	00113$
      00829E                        306 00111$:
                                    307 ;	../src/main.c: 110: GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
      00829E 14 01            [ 1]  308 	and	a, (0x01, sp)
      0082A0 1E 04            [ 2]  309 	ldw	x, (0x04, sp)
      0082A2 F7               [ 1]  310 	ld	(x), a
      0082A3                        311 00113$:
                                    312 ;	../src/main.c: 111: }
      0082A3 5B 05            [ 2]  313 	addw	sp, #5
      0082A5 81               [ 4]  314 	ret
                                    315 ;	../src/main.c: 113: void measure(uint8_t tell)	// the measure() function talks to the AM2320 via i2c
                                    316 ;	-----------------------------------------
                                    317 ;	 function measure
                                    318 ;	-----------------------------------------
      0082A6                        319 _measure:
      0082A6 52 22            [ 2]  320 	sub	sp, #34
                                    321 ;	../src/main.c: 118: if (i2c_start())	// have to check to see if the start was successful
      0082A8 CD 80 AA         [ 4]  322 	call	_i2c_start
      0082AB 4D               [ 1]  323 	tnz	a
      0082AC 26 03            [ 1]  324 	jrne	00160$
      0082AE CC 84 36         [ 2]  325 	jp	00106$
      0082B1                        326 00160$:
                                    327 ;	../src/main.c: 120: i2c_write_addr(AM2320_ADDR | I2C_WRITE);  // this is just to wake the AM2320 up
      0082B1 4B B8            [ 1]  328 	push	#0xb8
      0082B3 CD 80 FC         [ 4]  329 	call	_i2c_write_addr
      0082B6 84               [ 1]  330 	pop	a
                                    331 ;	../src/main.c: 121: i2c_stop();
      0082B7 CD 80 C7         [ 4]  332 	call	_i2c_stop
                                    333 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0082BA 90 5F            [ 1]  334 	clrw	y
      0082BC 4F               [ 1]  335 	clr	a
      0082BD 97               [ 1]  336 	ld	xl, a
      0082BE 4F               [ 1]  337 	clr	a
      0082BF                        338 00114$:
      0082BF 88               [ 1]  339 	push	a
      0082C0 90 A3 34 08      [ 2]  340 	cpw	y, #0x3408
      0082C4 9F               [ 1]  341 	ld	a, xl
      0082C5 A2 00            [ 1]  342 	sbc	a, #0x00
      0082C7 7B 01            [ 1]  343 	ld	a, (1, sp)
      0082C9 A2 00            [ 1]  344 	sbc	a, #0x00
      0082CB 84               [ 1]  345 	pop	a
      0082CC 24 0F            [ 1]  346 	jrnc	00108$
                                    347 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
      0082CE 9D               [ 1]  348 	nop
                                    349 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0082CF 72 A9 00 01      [ 2]  350 	addw	y, #0x0001
      0082D3 88               [ 1]  351 	push	a
      0082D4 9F               [ 1]  352 	ld	a, xl
      0082D5 A9 00            [ 1]  353 	adc	a, #0x00
      0082D7 97               [ 1]  354 	ld	xl, a
      0082D8 84               [ 1]  355 	pop	a
      0082D9 A9 00            [ 1]  356 	adc	a, #0x00
      0082DB 20 E2            [ 2]  357 	jra	00114$
                                    358 ;	../src/main.c: 123: delay_ms(15);	// the AM2320 needs this time to initialize itselt
      0082DD                        359 00108$:
                                    360 ;	../src/main.c: 125: i2c_start();								// now we ask for a reading
      0082DD CD 80 AA         [ 4]  361 	call	_i2c_start
                                    362 ;	../src/main.c: 126: i2c_write_addr(AM2320_ADDR | I2C_WRITE);
      0082E0 4B B8            [ 1]  363 	push	#0xb8
      0082E2 CD 80 FC         [ 4]  364 	call	_i2c_write_addr
      0082E5 84               [ 1]  365 	pop	a
                                    366 ;	../src/main.c: 127: i2c_write(0x03);	// the the AM2320 we want 4 bytes from address 0
      0082E6 4B 03            [ 1]  367 	push	#0x03
      0082E8 CD 80 E4         [ 4]  368 	call	_i2c_write
      0082EB 84               [ 1]  369 	pop	a
                                    370 ;	../src/main.c: 128: i2c_write(0x00);
      0082EC 4B 00            [ 1]  371 	push	#0x00
      0082EE CD 80 E4         [ 4]  372 	call	_i2c_write
      0082F1 84               [ 1]  373 	pop	a
                                    374 ;	../src/main.c: 129: i2c_write(0x04);
      0082F2 4B 04            [ 1]  375 	push	#0x04
      0082F4 CD 80 E4         [ 4]  376 	call	_i2c_write
      0082F7 84               [ 1]  377 	pop	a
                                    378 ;	../src/main.c: 130: i2c_stop();
      0082F8 CD 80 C7         [ 4]  379 	call	_i2c_stop
                                    380 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0082FB 90 5F            [ 1]  381 	clrw	y
      0082FD 4F               [ 1]  382 	clr	a
      0082FE 97               [ 1]  383 	ld	xl, a
      0082FF 4F               [ 1]  384 	clr	a
      008300                        385 00117$:
      008300 88               [ 1]  386 	push	a
      008301 90 A3 06 F0      [ 2]  387 	cpw	y, #0x06f0
      008305 9F               [ 1]  388 	ld	a, xl
      008306 A2 00            [ 1]  389 	sbc	a, #0x00
      008308 7B 01            [ 1]  390 	ld	a, (1, sp)
      00830A A2 00            [ 1]  391 	sbc	a, #0x00
      00830C 84               [ 1]  392 	pop	a
      00830D 24 0F            [ 1]  393 	jrnc	00110$
                                    394 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
      00830F 9D               [ 1]  395 	nop
                                    396 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008310 72 A9 00 01      [ 2]  397 	addw	y, #0x0001
      008314 88               [ 1]  398 	push	a
      008315 9F               [ 1]  399 	ld	a, xl
      008316 A9 00            [ 1]  400 	adc	a, #0x00
      008318 97               [ 1]  401 	ld	xl, a
      008319 84               [ 1]  402 	pop	a
      00831A A9 00            [ 1]  403 	adc	a, #0x00
      00831C 20 E2            [ 2]  404 	jra	00117$
                                    405 ;	../src/main.c: 131: delay_ms(2);
      00831E                        406 00110$:
                                    407 ;	../src/main.c: 134: i2c_start();
      00831E CD 80 AA         [ 4]  408 	call	_i2c_start
                                    409 ;	../src/main.c: 135: i2c_write_addr(AM2320_ADDR | I2C_READ);
      008321 4B B9            [ 1]  410 	push	#0xb9
      008323 CD 80 FC         [ 4]  411 	call	_i2c_write_addr
      008326 84               [ 1]  412 	pop	a
                                    413 ;	../src/main.c: 136: i2c_read_arr(buf, 6);	// the first two bytes are useless
      008327 AE 00 09         [ 2]  414 	ldw	x, #_buf+0
      00832A 1F 11            [ 2]  415 	ldw	(0x11, sp), x
      00832C 4B 06            [ 1]  416 	push	#0x06
      00832E 89               [ 2]  417 	pushw	x
      00832F CD 81 2D         [ 4]  418 	call	_i2c_read_arr
      008332 5B 03            [ 2]  419 	addw	sp, #3
                                    420 ;	../src/main.c: 137: humidity = (buf[2] << 8) + buf[3];	// get the 16-bit humidity
      008334 1E 11            [ 2]  421 	ldw	x, (0x11, sp)
      008336 E6 02            [ 1]  422 	ld	a, (0x2, x)
      008338 95               [ 1]  423 	ld	xh, a
      008339 0F 10            [ 1]  424 	clr	(0x10, sp)
      00833B 16 11            [ 2]  425 	ldw	y, (0x11, sp)
      00833D 90 E6 03         [ 1]  426 	ld	a, (0x3, y)
      008340 0F 05            [ 1]  427 	clr	(0x05, sp)
      008342 1B 10            [ 1]  428 	add	a, (0x10, sp)
      008344 02               [ 1]  429 	rlwa	x
      008345 19 05            [ 1]  430 	adc	a, (0x05, sp)
      008347 95               [ 1]  431 	ld	xh, a
      008348 1F 21            [ 2]  432 	ldw	(0x21, sp), x
                                    433 ;	../src/main.c: 138: temp = (buf[4] << 8) + buf[5];		// and the 16-bit temperature
      00834A 1E 11            [ 2]  434 	ldw	x, (0x11, sp)
      00834C E6 04            [ 1]  435 	ld	a, (0x4, x)
      00834E 6B 02            [ 1]  436 	ld	(0x02, sp), a
      008350 0F 01            [ 1]  437 	clr	(0x01, sp)
      008352 7B 02            [ 1]  438 	ld	a, (0x02, sp)
      008354 0F 0A            [ 1]  439 	clr	(0x0a, sp)
      008356 1E 11            [ 2]  440 	ldw	x, (0x11, sp)
      008358 88               [ 1]  441 	push	a
      008359 E6 05            [ 1]  442 	ld	a, (0x5, x)
      00835B 97               [ 1]  443 	ld	xl, a
      00835C 84               [ 1]  444 	pop	a
      00835D 0F 07            [ 1]  445 	clr	(0x07, sp)
      00835F 88               [ 1]  446 	push	a
      008360 9F               [ 1]  447 	ld	a, xl
      008361 1B 0B            [ 1]  448 	add	a, (0x0b, sp)
      008363 97               [ 1]  449 	ld	xl, a
      008364 84               [ 1]  450 	pop	a
      008365 19 07            [ 1]  451 	adc	a, (0x07, sp)
      008367 95               [ 1]  452 	ld	xh, a
                                    453 ;	../src/main.c: 139: temp = temp * 1.8 + 320;	// convert temperature to fahrenheit
      008368 89               [ 2]  454 	pushw	x
      008369 CD 90 94         [ 4]  455 	call	___uint2fs
      00836C 5B 02            [ 2]  456 	addw	sp, #2
      00836E 89               [ 2]  457 	pushw	x
      00836F 90 89            [ 2]  458 	pushw	y
      008371 4B 66            [ 1]  459 	push	#0x66
      008373 4B 66            [ 1]  460 	push	#0x66
      008375 4B E6            [ 1]  461 	push	#0xe6
      008377 4B 3F            [ 1]  462 	push	#0x3f
      008379 CD 8A B2         [ 4]  463 	call	___fsmul
      00837C 5B 08            [ 2]  464 	addw	sp, #8
      00837E 4B 00            [ 1]  465 	push	#0x00
      008380 4B 00            [ 1]  466 	push	#0x00
      008382 4B A0            [ 1]  467 	push	#0xa0
      008384 4B 43            [ 1]  468 	push	#0x43
      008386 89               [ 2]  469 	pushw	x
      008387 90 89            [ 2]  470 	pushw	y
      008389 CD 8D 91         [ 4]  471 	call	___fsadd
      00838C 5B 08            [ 2]  472 	addw	sp, #8
      00838E 89               [ 2]  473 	pushw	x
      00838F 90 89            [ 2]  474 	pushw	y
      008391 CD 90 A0         [ 4]  475 	call	___fs2uint
      008394 5B 04            [ 2]  476 	addw	sp, #4
      008396 1F 1F            [ 2]  477 	ldw	(0x1f, sp), x
                                    478 ;	../src/main.c: 140: if (buf[4] & 0x80)  // is it negative?
      008398 0D 02            [ 1]  479 	tnz	(0x02, sp)
      00839A 2A 0E            [ 1]  480 	jrpl	00102$
                                    481 ;	../src/main.c: 141: temp *= -1;
      00839C 1E 1F            [ 2]  482 	ldw	x, (0x1f, sp)
      00839E 89               [ 2]  483 	pushw	x
      00839F 4B FF            [ 1]  484 	push	#0xff
      0083A1 4B FF            [ 1]  485 	push	#0xff
      0083A3 CD 8D 78         [ 4]  486 	call	__mulint
      0083A6 5B 04            [ 2]  487 	addw	sp, #4
      0083A8 1F 1F            [ 2]  488 	ldw	(0x1f, sp), x
      0083AA                        489 00102$:
                                    490 ;	../src/main.c: 142: if (tell)	// if tell is set, transmit via rs485
      0083AA 0D 25            [ 1]  491 	tnz	(0x25, sp)
      0083AC 26 03            [ 1]  492 	jrne	00164$
      0083AE CC 84 32         [ 2]  493 	jp	00104$
      0083B1                        494 00164$:
                                    495 ;	../src/main.c: 144: rs485xmit_on();	// turn the RS485 chips transmitter on
      0083B1 C6 50 0F         [ 1]  496 	ld	a, 0x500f
      0083B4 AA 10            [ 1]  497 	or	a, #0x10
      0083B6 C7 50 0F         [ 1]  498 	ld	0x500f, a
                                    499 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0083B9 5F               [ 1]  500 	clrw	x
      0083BA 0F 1C            [ 1]  501 	clr	(0x1c, sp)
      0083BC 0F 1B            [ 1]  502 	clr	(0x1b, sp)
      0083BE                        503 00120$:
      0083BE A3 68 10         [ 2]  504 	cpw	x, #0x6810
      0083C1 7B 1C            [ 1]  505 	ld	a, (0x1c, sp)
      0083C3 A2 00            [ 1]  506 	sbc	a, #0x00
      0083C5 7B 1B            [ 1]  507 	ld	a, (0x1b, sp)
      0083C7 A2 00            [ 1]  508 	sbc	a, #0x00
      0083C9 24 14            [ 1]  509 	jrnc	00112$
                                    510 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
      0083CB 9D               [ 1]  511 	nop
                                    512 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0083CC 1C 00 01         [ 2]  513 	addw	x, #0x0001
      0083CF 7B 1C            [ 1]  514 	ld	a, (0x1c, sp)
      0083D1 A9 00            [ 1]  515 	adc	a, #0x00
      0083D3 90 97            [ 1]  516 	ld	yl, a
      0083D5 7B 1B            [ 1]  517 	ld	a, (0x1b, sp)
      0083D7 A9 00            [ 1]  518 	adc	a, #0x00
      0083D9 90 95            [ 1]  519 	ld	yh, a
      0083DB 17 1B            [ 2]  520 	ldw	(0x1b, sp), y
      0083DD 20 DF            [ 2]  521 	jra	00120$
                                    522 ;	../src/main.c: 145: delay_ms(30);	// wait for everything to be ready
      0083DF                        523 00112$:
                                    524 ;	../src/main.c: 146: printf("%c:%2d\.%1d%%:%3d\.%1d\r\n",address,humidity / 10,humidity % 10, temp / 10, temp %10);
      0083DF 1E 1F            [ 2]  525 	ldw	x, (0x1f, sp)
      0083E1 90 AE 00 0A      [ 2]  526 	ldw	y, #0x000a
      0083E5 65               [ 2]  527 	divw	x, y
      0083E6 17 0D            [ 2]  528 	ldw	(0x0d, sp), y
      0083E8 1E 1F            [ 2]  529 	ldw	x, (0x1f, sp)
      0083EA 90 AE 00 0A      [ 2]  530 	ldw	y, #0x000a
      0083EE 65               [ 2]  531 	divw	x, y
      0083EF 1F 0B            [ 2]  532 	ldw	(0x0b, sp), x
      0083F1 1E 21            [ 2]  533 	ldw	x, (0x21, sp)
      0083F3 90 AE 00 0A      [ 2]  534 	ldw	y, #0x000a
      0083F7 65               [ 2]  535 	divw	x, y
      0083F8 17 03            [ 2]  536 	ldw	(0x03, sp), y
      0083FA 1E 21            [ 2]  537 	ldw	x, (0x21, sp)
      0083FC 90 AE 00 0A      [ 2]  538 	ldw	y, #0x000a
      008400 65               [ 2]  539 	divw	x, y
      008401 1F 19            [ 2]  540 	ldw	(0x19, sp), x
      008403 C6 00 1E         [ 1]  541 	ld	a, _address+0
      008406 6B 18            [ 1]  542 	ld	(0x18, sp), a
      008408 0F 17            [ 1]  543 	clr	(0x17, sp)
      00840A AE 89 17         [ 2]  544 	ldw	x, #___str_0+0
      00840D 1F 15            [ 2]  545 	ldw	(0x15, sp), x
      00840F 90 93            [ 1]  546 	ldw	y, x
      008411 17 13            [ 2]  547 	ldw	(0x13, sp), y
      008413 1E 0D            [ 2]  548 	ldw	x, (0x0d, sp)
      008415 89               [ 2]  549 	pushw	x
      008416 1E 0D            [ 2]  550 	ldw	x, (0x0d, sp)
      008418 89               [ 2]  551 	pushw	x
      008419 1E 07            [ 2]  552 	ldw	x, (0x07, sp)
      00841B 89               [ 2]  553 	pushw	x
      00841C 1E 1F            [ 2]  554 	ldw	x, (0x1f, sp)
      00841E 89               [ 2]  555 	pushw	x
      00841F 1E 1F            [ 2]  556 	ldw	x, (0x1f, sp)
      008421 89               [ 2]  557 	pushw	x
      008422 1E 1D            [ 2]  558 	ldw	x, (0x1d, sp)
      008424 89               [ 2]  559 	pushw	x
      008425 CD 90 DE         [ 4]  560 	call	_printf
      008428 5B 0C            [ 2]  561 	addw	sp, #12
                                    562 ;	../src/main.c: 147: rs485xmit_off(); // turn the transmitter back off
      00842A C6 50 0F         [ 1]  563 	ld	a, 0x500f
      00842D A4 EF            [ 1]  564 	and	a, #0xef
      00842F C7 50 0F         [ 1]  565 	ld	0x500f, a
      008432                        566 00104$:
                                    567 ;	../src/main.c: 149: reset_watchdog();  // reset the watchdog timer
      008432 35 AA 50 E0      [ 1]  568 	mov	0x50e0+0, #0xaa
      008436                        569 00106$:
                                    570 ;	../src/main.c: 152: reset_watchdog();  // reset the watchdog timer
      008436 35 AA 50 E0      [ 1]  571 	mov	0x50e0+0, #0xaa
                                    572 ;	../src/main.c: 153: last_measure = Global_time;	// when was the last time we transmitted anything
      00843A CE 00 03         [ 2]  573 	ldw	x, _Global_time+2
      00843D 90 CE 00 01      [ 2]  574 	ldw	y, _Global_time+0
      008441 CF 00 4B         [ 2]  575 	ldw	_last_measure+2, x
      008444 90 CF 00 49      [ 2]  576 	ldw	_last_measure+0, y
                                    577 ;	../src/main.c: 154: }
      008448 5B 22            [ 2]  578 	addw	sp, #34
      00844A 81               [ 4]  579 	ret
                                    580 ;	../src/main.c: 156: void main() {
                                    581 ;	-----------------------------------------
                                    582 ;	 function main
                                    583 ;	-----------------------------------------
      00844B                        584 _main:
      00844B 52 5D            [ 2]  585 	sub	sp, #93
                                    586 ;	../src/main.c: 157: unsigned long esc_time = 0L;
      00844D 5F               [ 1]  587 	clrw	x
      00844E 1F 5C            [ 2]  588 	ldw	(0x5c, sp), x
      008450 1F 5A            [ 2]  589 	ldw	(0x5a, sp), x
                                    590 ;	../src/main.c: 161: esc = 0;
      008452 72 5F 00 1D      [ 1]  591 	clr	_esc+0
                                    592 ;	../src/main.c: 163: sprintf(version,"%02d%02d%02d-%02d%02d", BUILD_YEAR - 2000, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN);
      008456 AE 89 50         [ 2]  593 	ldw	x, #___str_3+0
      008459 1F 12            [ 2]  594 	ldw	(0x12, sp), x
      00845B F6               [ 1]  595 	ld	a, (x)
      00845C 6B 0D            [ 1]  596 	ld	(0x0d, sp), a
      00845E 7B 0D            [ 1]  597 	ld	a, (0x0d, sp)
      008460 A1 3F            [ 1]  598 	cp	a, #0x3f
      008462 26 06            [ 1]  599 	jrne	00384$
      008464 A6 01            [ 1]  600 	ld	a, #0x01
      008466 6B 0C            [ 1]  601 	ld	(0x0c, sp), a
      008468 20 02            [ 2]  602 	jra	00385$
      00846A                        603 00384$:
      00846A 0F 0C            [ 1]  604 	clr	(0x0c, sp)
      00846C                        605 00385$:
      00846C 0D 0C            [ 1]  606 	tnz	(0x0c, sp)
      00846E 27 07            [ 1]  607 	jreq	00139$
      008470 AE 00 63         [ 2]  608 	ldw	x, #0x0063
      008473 1F 10            [ 2]  609 	ldw	(0x10, sp), x
      008475 20 22            [ 2]  610 	jra	00140$
      008477                        611 00139$:
      008477 1E 12            [ 2]  612 	ldw	x, (0x12, sp)
      008479 E6 03            [ 1]  613 	ld	a, (0x3, x)
      00847B 5F               [ 1]  614 	clrw	x
      00847C 97               [ 1]  615 	ld	xl, a
      00847D 1D 00 30         [ 2]  616 	subw	x, #0x0030
      008480 89               [ 2]  617 	pushw	x
      008481 58               [ 2]  618 	sllw	x
      008482 58               [ 2]  619 	sllw	x
      008483 72 FB 01         [ 2]  620 	addw	x, (1, sp)
      008486 58               [ 2]  621 	sllw	x
      008487 5B 02            [ 2]  622 	addw	sp, #2
      008489 1F 0E            [ 2]  623 	ldw	(0x0e, sp), x
      00848B 1E 12            [ 2]  624 	ldw	x, (0x12, sp)
      00848D E6 04            [ 1]  625 	ld	a, (0x4, x)
      00848F 5F               [ 1]  626 	clrw	x
      008490 97               [ 1]  627 	ld	xl, a
      008491 72 FB 0E         [ 2]  628 	addw	x, (0x0e, sp)
      008494 1D 00 30         [ 2]  629 	subw	x, #0x0030
      008497 1F 10            [ 2]  630 	ldw	(0x10, sp), x
      008499                        631 00140$:
      008499 0D 0C            [ 1]  632 	tnz	(0x0c, sp)
      00849B 27 07            [ 1]  633 	jreq	00141$
      00849D AE 00 63         [ 2]  634 	ldw	x, #0x0063
      0084A0 1F 08            [ 2]  635 	ldw	(0x08, sp), x
      0084A2 20 24            [ 2]  636 	jra	00142$
      0084A4                        637 00141$:
      0084A4 5F               [ 1]  638 	clrw	x
      0084A5 7B 0D            [ 1]  639 	ld	a, (0x0d, sp)
      0084A7 97               [ 1]  640 	ld	xl, a
      0084A8 1D 00 30         [ 2]  641 	subw	x, #0x0030
      0084AB 89               [ 2]  642 	pushw	x
      0084AC 58               [ 2]  643 	sllw	x
      0084AD 58               [ 2]  644 	sllw	x
      0084AE 72 FB 01         [ 2]  645 	addw	x, (1, sp)
      0084B1 58               [ 2]  646 	sllw	x
      0084B2 5B 02            [ 2]  647 	addw	sp, #2
      0084B4 1F 06            [ 2]  648 	ldw	(0x06, sp), x
      0084B6 1E 12            [ 2]  649 	ldw	x, (0x12, sp)
      0084B8 E6 01            [ 1]  650 	ld	a, (0x1, x)
      0084BA 5F               [ 1]  651 	clrw	x
      0084BB 97               [ 1]  652 	ld	xl, a
      0084BC 72 FB 06         [ 2]  653 	addw	x, (0x06, sp)
      0084BF 1D 00 30         [ 2]  654 	subw	x, #0x0030
      0084C2 1F 04            [ 2]  655 	ldw	(0x04, sp), x
      0084C4 90 93            [ 1]  656 	ldw	y, x
      0084C6 17 08            [ 2]  657 	ldw	(0x08, sp), y
      0084C8                        658 00142$:
      0084C8 AE 89 44         [ 2]  659 	ldw	x, #___str_2+0
      0084CB 1F 02            [ 2]  660 	ldw	(0x02, sp), x
      0084CD F6               [ 1]  661 	ld	a, (x)
      0084CE 6B 0B            [ 1]  662 	ld	(0x0b, sp), a
      0084D0 7B 0B            [ 1]  663 	ld	a, (0x0b, sp)
      0084D2 A1 3F            [ 1]  664 	cp	a, #0x3f
      0084D4 26 06            [ 1]  665 	jrne	00389$
      0084D6 A6 01            [ 1]  666 	ld	a, #0x01
      0084D8 6B 0A            [ 1]  667 	ld	(0x0a, sp), a
      0084DA 20 02            [ 2]  668 	jra	00390$
      0084DC                        669 00389$:
      0084DC 0F 0A            [ 1]  670 	clr	(0x0a, sp)
      0084DE                        671 00390$:
      0084DE 0D 0A            [ 1]  672 	tnz	(0x0a, sp)
      0084E0 27 07            [ 1]  673 	jreq	00143$
      0084E2 AE 00 63         [ 2]  674 	ldw	x, #0x0063
      0084E5 1F 15            [ 2]  675 	ldw	(0x15, sp), x
      0084E7 20 31            [ 2]  676 	jra	00144$
      0084E9                        677 00143$:
      0084E9 1E 02            [ 2]  678 	ldw	x, (0x02, sp)
      0084EB E6 04            [ 1]  679 	ld	a, (0x4, x)
      0084ED 6B 14            [ 1]  680 	ld	(0x14, sp), a
      0084EF 7B 14            [ 1]  681 	ld	a, (0x14, sp)
      0084F1 A1 30            [ 1]  682 	cp	a, #0x30
      0084F3 25 14            [ 1]  683 	jrc	00145$
      0084F5 5F               [ 1]  684 	clrw	x
      0084F6 7B 14            [ 1]  685 	ld	a, (0x14, sp)
      0084F8 97               [ 1]  686 	ld	xl, a
      0084F9 1D 00 30         [ 2]  687 	subw	x, #0x0030
      0084FC 89               [ 2]  688 	pushw	x
      0084FD 58               [ 2]  689 	sllw	x
      0084FE 58               [ 2]  690 	sllw	x
      0084FF 72 FB 01         [ 2]  691 	addw	x, (1, sp)
      008502 58               [ 2]  692 	sllw	x
      008503 5B 02            [ 2]  693 	addw	sp, #2
      008505 1F 19            [ 2]  694 	ldw	(0x19, sp), x
      008507 20 03            [ 2]  695 	jra	00146$
      008509                        696 00145$:
      008509 5F               [ 1]  697 	clrw	x
      00850A 1F 19            [ 2]  698 	ldw	(0x19, sp), x
      00850C                        699 00146$:
      00850C 1E 02            [ 2]  700 	ldw	x, (0x02, sp)
      00850E E6 05            [ 1]  701 	ld	a, (0x5, x)
      008510 5F               [ 1]  702 	clrw	x
      008511 97               [ 1]  703 	ld	xl, a
      008512 1D 00 30         [ 2]  704 	subw	x, #0x0030
      008515 72 FB 19         [ 2]  705 	addw	x, (0x19, sp)
      008518 1F 15            [ 2]  706 	ldw	(0x15, sp), x
      00851A                        707 00144$:
      00851A 0D 0A            [ 1]  708 	tnz	(0x0a, sp)
      00851C 27 08            [ 1]  709 	jreq	00147$
      00851E AE 00 63         [ 2]  710 	ldw	x, #0x0063
      008521 1F 17            [ 2]  711 	ldw	(0x17, sp), x
      008523 CC 86 52         [ 2]  712 	jp	00148$
      008526                        713 00147$:
      008526 7B 0B            [ 1]  714 	ld	a, (0x0b, sp)
      008528 A1 4A            [ 1]  715 	cp	a, #0x4a
      00852A 26 06            [ 1]  716 	jrne	00395$
      00852C A6 01            [ 1]  717 	ld	a, #0x01
      00852E 6B 1D            [ 1]  718 	ld	(0x1d, sp), a
      008530 20 02            [ 2]  719 	jra	00396$
      008532                        720 00395$:
      008532 0F 1D            [ 1]  721 	clr	(0x1d, sp)
      008534                        722 00396$:
      008534 1E 02            [ 2]  723 	ldw	x, (0x02, sp)
      008536 5C               [ 1]  724 	incw	x
      008537 1F 1B            [ 2]  725 	ldw	(0x1b, sp), x
      008539 1E 02            [ 2]  726 	ldw	x, (0x02, sp)
      00853B 5C               [ 1]  727 	incw	x
      00853C 5C               [ 1]  728 	incw	x
      00853D 1F 20            [ 2]  729 	ldw	(0x20, sp), x
      00853F 0D 1D            [ 1]  730 	tnz	(0x1d, sp)
      008541 27 13            [ 1]  731 	jreq	00149$
      008543 1E 1B            [ 2]  732 	ldw	x, (0x1b, sp)
      008545 F6               [ 1]  733 	ld	a, (x)
      008546 A1 61            [ 1]  734 	cp	a, #0x61
      008548 26 0C            [ 1]  735 	jrne	00149$
      00854A 1E 20            [ 2]  736 	ldw	x, (0x20, sp)
      00854C F6               [ 1]  737 	ld	a, (x)
      00854D A1 6E            [ 1]  738 	cp	a, #0x6e
      00854F 26 05            [ 1]  739 	jrne	00149$
      008551 5F               [ 1]  740 	clrw	x
      008552 5C               [ 1]  741 	incw	x
      008553 CC 86 50         [ 2]  742 	jp	00150$
      008556                        743 00149$:
      008556 7B 0B            [ 1]  744 	ld	a, (0x0b, sp)
      008558 A1 46            [ 1]  745 	cp	a, #0x46
      00855A 26 08            [ 1]  746 	jrne	00157$
      00855C AE 00 02         [ 2]  747 	ldw	x, #0x0002
      00855F 1F 1E            [ 2]  748 	ldw	(0x1e, sp), x
      008561 CC 86 4E         [ 2]  749 	jp	00158$
      008564                        750 00157$:
      008564 7B 0B            [ 1]  751 	ld	a, (0x0b, sp)
      008566 A1 4D            [ 1]  752 	cp	a, #0x4d
      008568 26 06            [ 1]  753 	jrne	00408$
      00856A A6 01            [ 1]  754 	ld	a, #0x01
      00856C 6B 23            [ 1]  755 	ld	(0x23, sp), a
      00856E 20 02            [ 2]  756 	jra	00409$
      008570                        757 00408$:
      008570 0F 23            [ 1]  758 	clr	(0x23, sp)
      008572                        759 00409$:
      008572 0D 23            [ 1]  760 	tnz	(0x23, sp)
      008574 27 14            [ 1]  761 	jreq	00159$
      008576 1E 1B            [ 2]  762 	ldw	x, (0x1b, sp)
      008578 F6               [ 1]  763 	ld	a, (x)
      008579 A1 61            [ 1]  764 	cp	a, #0x61
      00857B 26 0D            [ 1]  765 	jrne	00159$
      00857D 1E 20            [ 2]  766 	ldw	x, (0x20, sp)
      00857F F6               [ 1]  767 	ld	a, (x)
      008580 A1 72            [ 1]  768 	cp	a, #0x72
      008582 26 06            [ 1]  769 	jrne	00159$
      008584 AE 00 03         [ 2]  770 	ldw	x, #0x0003
      008587 CC 86 4C         [ 2]  771 	jp	00160$
      00858A                        772 00159$:
      00858A 7B 0B            [ 1]  773 	ld	a, (0x0b, sp)
      00858C A1 41            [ 1]  774 	cp	a, #0x41
      00858E 26 06            [ 1]  775 	jrne	00418$
      008590 A6 01            [ 1]  776 	ld	a, #0x01
      008592 6B 22            [ 1]  777 	ld	(0x22, sp), a
      008594 20 02            [ 2]  778 	jra	00419$
      008596                        779 00418$:
      008596 0F 22            [ 1]  780 	clr	(0x22, sp)
      008598                        781 00419$:
      008598 0D 22            [ 1]  782 	tnz	(0x22, sp)
      00859A 27 0F            [ 1]  783 	jreq	00167$
      00859C 1E 1B            [ 2]  784 	ldw	x, (0x1b, sp)
      00859E F6               [ 1]  785 	ld	a, (x)
      00859F A1 70            [ 1]  786 	cp	a, #0x70
      0085A1 26 08            [ 1]  787 	jrne	00167$
      0085A3 AE 00 04         [ 2]  788 	ldw	x, #0x0004
      0085A6 1F 26            [ 2]  789 	ldw	(0x26, sp), x
      0085A8 CC 86 4A         [ 2]  790 	jp	00168$
      0085AB                        791 00167$:
      0085AB 0D 23            [ 1]  792 	tnz	(0x23, sp)
      0085AD 27 16            [ 1]  793 	jreq	00172$
      0085AF 1E 1B            [ 2]  794 	ldw	x, (0x1b, sp)
      0085B1 F6               [ 1]  795 	ld	a, (x)
      0085B2 A1 61            [ 1]  796 	cp	a, #0x61
      0085B4 26 0F            [ 1]  797 	jrne	00172$
      0085B6 1E 20            [ 2]  798 	ldw	x, (0x20, sp)
      0085B8 F6               [ 1]  799 	ld	a, (x)
      0085B9 A1 79            [ 1]  800 	cp	a, #0x79
      0085BB 26 08            [ 1]  801 	jrne	00172$
      0085BD AE 00 05         [ 2]  802 	ldw	x, #0x0005
      0085C0 1F 24            [ 2]  803 	ldw	(0x24, sp), x
      0085C2 CC 86 46         [ 2]  804 	jp	00173$
      0085C5                        805 00172$:
      0085C5 0D 1D            [ 1]  806 	tnz	(0x1d, sp)
      0085C7 27 13            [ 1]  807 	jreq	00180$
      0085C9 1E 1B            [ 2]  808 	ldw	x, (0x1b, sp)
      0085CB F6               [ 1]  809 	ld	a, (x)
      0085CC A1 75            [ 1]  810 	cp	a, #0x75
      0085CE 26 0C            [ 1]  811 	jrne	00180$
      0085D0 1E 20            [ 2]  812 	ldw	x, (0x20, sp)
      0085D2 F6               [ 1]  813 	ld	a, (x)
      0085D3 A1 6E            [ 1]  814 	cp	a, #0x6e
      0085D5 26 05            [ 1]  815 	jrne	00180$
      0085D7 AE 00 06         [ 2]  816 	ldw	x, #0x0006
      0085DA 20 68            [ 2]  817 	jra	00181$
      0085DC                        818 00180$:
      0085DC 0D 1D            [ 1]  819 	tnz	(0x1d, sp)
      0085DE 27 15            [ 1]  820 	jreq	00188$
      0085E0 1E 1B            [ 2]  821 	ldw	x, (0x1b, sp)
      0085E2 F6               [ 1]  822 	ld	a, (x)
      0085E3 A1 75            [ 1]  823 	cp	a, #0x75
      0085E5 26 0E            [ 1]  824 	jrne	00188$
      0085E7 1E 20            [ 2]  825 	ldw	x, (0x20, sp)
      0085E9 F6               [ 1]  826 	ld	a, (x)
      0085EA A1 6C            [ 1]  827 	cp	a, #0x6c
      0085EC 26 07            [ 1]  828 	jrne	00188$
      0085EE AE 00 07         [ 2]  829 	ldw	x, #0x0007
      0085F1 1F 2A            [ 2]  830 	ldw	(0x2a, sp), x
      0085F3 20 4D            [ 2]  831 	jra	00189$
      0085F5                        832 00188$:
      0085F5 0D 22            [ 1]  833 	tnz	(0x22, sp)
      0085F7 27 0C            [ 1]  834 	jreq	00196$
      0085F9 1E 1B            [ 2]  835 	ldw	x, (0x1b, sp)
      0085FB F6               [ 1]  836 	ld	a, (x)
      0085FC A1 75            [ 1]  837 	cp	a, #0x75
      0085FE 26 05            [ 1]  838 	jrne	00196$
      008600 AE 00 08         [ 2]  839 	ldw	x, #0x0008
      008603 20 3B            [ 2]  840 	jra	00197$
      008605                        841 00196$:
      008605 7B 0B            [ 1]  842 	ld	a, (0x0b, sp)
      008607 A1 53            [ 1]  843 	cp	a, #0x53
      008609 26 07            [ 1]  844 	jrne	00201$
      00860B AE 00 09         [ 2]  845 	ldw	x, #0x0009
      00860E 1F 28            [ 2]  846 	ldw	(0x28, sp), x
      008610 20 2C            [ 2]  847 	jra	00202$
      008612                        848 00201$:
      008612 7B 0B            [ 1]  849 	ld	a, (0x0b, sp)
      008614 A1 4F            [ 1]  850 	cp	a, #0x4f
      008616 26 05            [ 1]  851 	jrne	00203$
      008618 AE 00 0A         [ 2]  852 	ldw	x, #0x000a
      00861B 20 1F            [ 2]  853 	jra	00204$
      00861D                        854 00203$:
      00861D 7B 0B            [ 1]  855 	ld	a, (0x0b, sp)
      00861F A1 4E            [ 1]  856 	cp	a, #0x4e
      008621 26 07            [ 1]  857 	jrne	00205$
      008623 AE 00 0B         [ 2]  858 	ldw	x, #0x000b
      008626 1F 2E            [ 2]  859 	ldw	(0x2e, sp), x
      008628 20 10            [ 2]  860 	jra	00206$
      00862A                        861 00205$:
      00862A 7B 0B            [ 1]  862 	ld	a, (0x0b, sp)
      00862C A1 44            [ 1]  863 	cp	a, #0x44
      00862E 26 05            [ 1]  864 	jrne	00207$
      008630 AE 00 0C         [ 2]  865 	ldw	x, #0x000c
      008633 20 03            [ 2]  866 	jra	00208$
      008635                        867 00207$:
      008635 AE 00 63         [ 2]  868 	ldw	x, #0x0063
      008638                        869 00208$:
      008638 1F 2E            [ 2]  870 	ldw	(0x2e, sp), x
      00863A                        871 00206$:
      00863A 1E 2E            [ 2]  872 	ldw	x, (0x2e, sp)
      00863C                        873 00204$:
      00863C 1F 28            [ 2]  874 	ldw	(0x28, sp), x
      00863E                        875 00202$:
      00863E 1E 28            [ 2]  876 	ldw	x, (0x28, sp)
      008640                        877 00197$:
      008640 1F 2A            [ 2]  878 	ldw	(0x2a, sp), x
      008642                        879 00189$:
      008642 1E 2A            [ 2]  880 	ldw	x, (0x2a, sp)
      008644                        881 00181$:
      008644 1F 24            [ 2]  882 	ldw	(0x24, sp), x
      008646                        883 00173$:
      008646 16 24            [ 2]  884 	ldw	y, (0x24, sp)
      008648 17 26            [ 2]  885 	ldw	(0x26, sp), y
      00864A                        886 00168$:
      00864A 1E 26            [ 2]  887 	ldw	x, (0x26, sp)
      00864C                        888 00160$:
      00864C 1F 1E            [ 2]  889 	ldw	(0x1e, sp), x
      00864E                        890 00158$:
      00864E 1E 1E            [ 2]  891 	ldw	x, (0x1e, sp)
      008650                        892 00150$:
      008650 1F 17            [ 2]  893 	ldw	(0x17, sp), x
      008652                        894 00148$:
      008652 0D 0A            [ 1]  895 	tnz	(0x0a, sp)
      008654 27 05            [ 1]  896 	jreq	00209$
      008656 AE 00 63         [ 2]  897 	ldw	x, #0x0063
      008659 20 52            [ 2]  898 	jra	00210$
      00865B                        899 00209$:
      00865B 1E 02            [ 2]  900 	ldw	x, (0x02, sp)
      00865D E6 07            [ 1]  901 	ld	a, (0x7, x)
      00865F 5F               [ 1]  902 	clrw	x
      008660 97               [ 1]  903 	ld	xl, a
      008661 1D 00 30         [ 2]  904 	subw	x, #0x0030
      008664 89               [ 2]  905 	pushw	x
      008665 4B E8            [ 1]  906 	push	#0xe8
      008667 4B 03            [ 1]  907 	push	#0x03
      008669 CD 8D 78         [ 4]  908 	call	__mulint
      00866C 5B 04            [ 2]  909 	addw	sp, #4
      00866E 1F 2C            [ 2]  910 	ldw	(0x2c, sp), x
      008670 1E 02            [ 2]  911 	ldw	x, (0x02, sp)
      008672 E6 08            [ 1]  912 	ld	a, (0x8, x)
      008674 5F               [ 1]  913 	clrw	x
      008675 97               [ 1]  914 	ld	xl, a
      008676 1D 00 30         [ 2]  915 	subw	x, #0x0030
      008679 89               [ 2]  916 	pushw	x
      00867A 4B 64            [ 1]  917 	push	#0x64
      00867C 4B 00            [ 1]  918 	push	#0x00
      00867E CD 8D 78         [ 4]  919 	call	__mulint
      008681 5B 04            [ 2]  920 	addw	sp, #4
      008683 72 FB 2C         [ 2]  921 	addw	x, (0x2c, sp)
      008686 1F 32            [ 2]  922 	ldw	(0x32, sp), x
      008688 1E 02            [ 2]  923 	ldw	x, (0x02, sp)
      00868A E6 09            [ 1]  924 	ld	a, (0x9, x)
      00868C 5F               [ 1]  925 	clrw	x
      00868D 97               [ 1]  926 	ld	xl, a
      00868E 1D 00 30         [ 2]  927 	subw	x, #0x0030
      008691 89               [ 2]  928 	pushw	x
      008692 58               [ 2]  929 	sllw	x
      008693 58               [ 2]  930 	sllw	x
      008694 72 FB 01         [ 2]  931 	addw	x, (1, sp)
      008697 58               [ 2]  932 	sllw	x
      008698 5B 02            [ 2]  933 	addw	sp, #2
      00869A 72 FB 32         [ 2]  934 	addw	x, (0x32, sp)
      00869D 1F 30            [ 2]  935 	ldw	(0x30, sp), x
      00869F 1E 02            [ 2]  936 	ldw	x, (0x02, sp)
      0086A1 E6 0A            [ 1]  937 	ld	a, (0xa, x)
      0086A3 5F               [ 1]  938 	clrw	x
      0086A4 97               [ 1]  939 	ld	xl, a
      0086A5 1D 00 30         [ 2]  940 	subw	x, #0x0030
      0086A8 72 FB 30         [ 2]  941 	addw	x, (0x30, sp)
      0086AB 1F 36            [ 2]  942 	ldw	(0x36, sp), x
      0086AD                        943 00210$:
      0086AD 1D 07 D0         [ 2]  944 	subw	x, #0x07d0
      0086B0 1F 34            [ 2]  945 	ldw	(0x34, sp), x
      0086B2 AE 89 2E         [ 2]  946 	ldw	x, #___str_1+0
      0086B5 1F 3A            [ 2]  947 	ldw	(0x3a, sp), x
      0086B7 AE 00 1F         [ 2]  948 	ldw	x, #_version+0
      0086BA 1F 38            [ 2]  949 	ldw	(0x38, sp), x
      0086BC 90 93            [ 1]  950 	ldw	y, x
      0086BE 1E 10            [ 2]  951 	ldw	x, (0x10, sp)
      0086C0 89               [ 2]  952 	pushw	x
      0086C1 1E 0A            [ 2]  953 	ldw	x, (0x0a, sp)
      0086C3 89               [ 2]  954 	pushw	x
      0086C4 1E 19            [ 2]  955 	ldw	x, (0x19, sp)
      0086C6 89               [ 2]  956 	pushw	x
      0086C7 1E 1D            [ 2]  957 	ldw	x, (0x1d, sp)
      0086C9 89               [ 2]  958 	pushw	x
      0086CA 1E 3C            [ 2]  959 	ldw	x, (0x3c, sp)
      0086CC 89               [ 2]  960 	pushw	x
      0086CD 1E 44            [ 2]  961 	ldw	x, (0x44, sp)
      0086CF 89               [ 2]  962 	pushw	x
      0086D0 90 89            [ 2]  963 	pushw	y
      0086D2 CD 90 74         [ 4]  964 	call	_sprintf
      0086D5 5B 0E            [ 2]  965 	addw	sp, #14
                                    966 ;	../src/main.c: 165: disableInterrupts();
      0086D7 9B               [ 1]  967 	sim
                                    968 ;	../src/main.c: 166: InitialiseSystemClock();
      0086D8 CD 81 DE         [ 4]  969 	call	_InitialiseSystemClock
                                    970 ;	../src/main.c: 167: InitialiseIWDG();		// not really necessary, but what the heck...
      0086DB CD 82 1C         [ 4]  971 	call	_InitialiseIWDG
                                    972 ;	../src/main.c: 168: GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);  // rs485 xmit indicator
      0086DE 4B E0            [ 1]  973 	push	#0xe0
      0086E0 4B 10            [ 1]  974 	push	#0x10
      0086E2 4B 0F            [ 1]  975 	push	#0x0f
      0086E4 4B 50            [ 1]  976 	push	#0x50
      0086E6 CD 82 31         [ 4]  977 	call	_GPIO_Init
      0086E9 5B 04            [ 2]  978 	addw	sp, #4
                                    979 ;	../src/main.c: 170: rs485xmit_off();	// turn off the D4 pin so we can receive rs485 data
      0086EB 72 19 50 0F      [ 1]  980 	bres	20495, #4
                                    981 ;	../src/main.c: 175: CFG->GCR |= 1; // disable SWIM
      0086EF C6 7F 60         [ 1]  982 	ld	a, 0x7f60
      0086F2 5F               [ 1]  983 	clrw	x
      0086F3 97               [ 1]  984 	ld	xl, a
      0086F4 54               [ 2]  985 	srlw	x
      0086F5 99               [ 1]  986 	scf
      0086F6 59               [ 2]  987 	rlcw	x
      0086F7 9F               [ 1]  988 	ld	a, xl
      0086F8 C7 7F 60         [ 1]  989 	ld	0x7f60, a
                                    990 ;	../src/main.c: 178: TIM4->PSCR = 7;   // prescaler
      0086FB 35 07 53 47      [ 1]  991 	mov	0x5347+0, #0x07
                                    992 ;	../src/main.c: 179: TIM4->ARR = 125;  // auto reload register
      0086FF 35 7D 53 48      [ 1]  993 	mov	0x5348+0, #0x7d
                                    994 ;	../src/main.c: 181: TIM4->IER = TIM4_IER_UIE;
      008703 35 01 53 43      [ 1]  995 	mov	0x5343+0, #0x01
                                    996 ;	../src/main.c: 183: TIM4->CR1 = TIM4_CR1_ARPE | TIM4_CR1_URS | TIM4_CR1_CEN;
      008707 35 85 53 40      [ 1]  997 	mov	0x5340+0, #0x85
                                    998 ;	../src/main.c: 185: reset_watchdog();  // reset the watchdog timer
      00870B 35 AA 50 E0      [ 1]  999 	mov	0x50e0+0, #0xaa
                                   1000 ;	../src/main.c: 187: Global_time = 0L;	// used as an internal clock
      00870F 5F               [ 1] 1001 	clrw	x
      008710 CF 00 03         [ 2] 1002 	ldw	_Global_time+2, x
      008713 CF 00 01         [ 2] 1003 	ldw	_Global_time+0, x
                                   1004 ;	../src/main.c: 188: uart_init();		// setup for 9600 8-N-1
      008716 CD 8A 48         [ 4] 1005 	call	_uart_init
                                   1006 ;	../src/main.c: 189: i2c_init();			// talk to the AM2320 and the MCP23017 to read switches
      008719 CD 80 8C         [ 4] 1007 	call	_i2c_init
                                   1008 ;	../src/main.c: 190: reset_watchdog();
      00871C 35 AA 50 E0      [ 1] 1009 	mov	0x50e0+0, #0xaa
                                   1010 ;	../src/main.c: 192: enableInterrupts();
      008720 9A               [ 1] 1011 	rim
                                   1012 ;	../src/main.c: 193: reset_watchdog();  // reset the watchdog timer
      008721 35 AA 50 E0      [ 1] 1013 	mov	0x50e0+0, #0xaa
                                   1014 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008725 5F               [ 1] 1015 	clrw	x
      008726 1F 54            [ 2] 1016 	ldw	(0x54, sp), x
      008728 1F 52            [ 2] 1017 	ldw	(0x52, sp), x
      00872A                       1018 00126$:
      00872A 1E 54            [ 2] 1019 	ldw	x, (0x54, sp)
      00872C A3 63 30         [ 2] 1020 	cpw	x, #0x6330
      00872F 7B 53            [ 1] 1021 	ld	a, (0x53, sp)
      008731 A2 03            [ 1] 1022 	sbc	a, #0x03
      008733 7B 52            [ 1] 1023 	ld	a, (0x52, sp)
      008735 A2 00            [ 1] 1024 	sbc	a, #0x00
      008737 24 17            [ 1] 1025 	jrnc	00118$
                                   1026 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
      008739 9D               [ 1] 1027 	nop
                                   1028 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      00873A 16 54            [ 2] 1029 	ldw	y, (0x54, sp)
      00873C 72 A9 00 01      [ 2] 1030 	addw	y, #0x0001
      008740 7B 53            [ 1] 1031 	ld	a, (0x53, sp)
      008742 A9 00            [ 1] 1032 	adc	a, #0x00
      008744 97               [ 1] 1033 	ld	xl, a
      008745 7B 52            [ 1] 1034 	ld	a, (0x52, sp)
      008747 A9 00            [ 1] 1035 	adc	a, #0x00
      008749 95               [ 1] 1036 	ld	xh, a
      00874A 17 54            [ 2] 1037 	ldw	(0x54, sp), y
      00874C 1F 52            [ 2] 1038 	ldw	(0x52, sp), x
      00874E 20 DA            [ 2] 1039 	jra	00126$
                                   1040 ;	../src/main.c: 195: delay_ms(250);
      008750                       1041 00118$:
                                   1042 ;	../src/main.c: 196: rs485xmit_on();	// turn the RS485 chips transmitter on
      008750 C6 50 0F         [ 1] 1043 	ld	a, 0x500f
      008753 AA 10            [ 1] 1044 	or	a, #0x10
      008755 C7 50 0F         [ 1] 1045 	ld	0x500f, a
                                   1046 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008758 5F               [ 1] 1047 	clrw	x
      008759 1F 58            [ 2] 1048 	ldw	(0x58, sp), x
      00875B 1F 56            [ 2] 1049 	ldw	(0x56, sp), x
      00875D                       1050 00129$:
      00875D 1E 58            [ 2] 1051 	ldw	x, (0x58, sp)
      00875F A3 68 10         [ 2] 1052 	cpw	x, #0x6810
      008762 7B 57            [ 1] 1053 	ld	a, (0x57, sp)
      008764 A2 00            [ 1] 1054 	sbc	a, #0x00
      008766 7B 56            [ 1] 1055 	ld	a, (0x56, sp)
      008768 A2 00            [ 1] 1056 	sbc	a, #0x00
      00876A 24 17            [ 1] 1057 	jrnc	00120$
                                   1058 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
      00876C 9D               [ 1] 1059 	nop
                                   1060 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      00876D 16 58            [ 2] 1061 	ldw	y, (0x58, sp)
      00876F 72 A9 00 01      [ 2] 1062 	addw	y, #0x0001
      008773 7B 57            [ 1] 1063 	ld	a, (0x57, sp)
      008775 A9 00            [ 1] 1064 	adc	a, #0x00
      008777 97               [ 1] 1065 	ld	xl, a
      008778 7B 56            [ 1] 1066 	ld	a, (0x56, sp)
      00877A A9 00            [ 1] 1067 	adc	a, #0x00
      00877C 95               [ 1] 1068 	ld	xh, a
      00877D 17 58            [ 2] 1069 	ldw	(0x58, sp), y
      00877F 1F 56            [ 2] 1070 	ldw	(0x56, sp), x
      008781 20 DA            [ 2] 1071 	jra	00129$
                                   1072 ;	../src/main.c: 197: delay_ms(30);
      008783                       1073 00120$:
                                   1074 ;	../src/main.c: 198: printf("Initializing...%s\r\n",version);
      008783 16 38            [ 2] 1075 	ldw	y, (0x38, sp)
      008785 AE 89 59         [ 2] 1076 	ldw	x, #___str_4+0
      008788 90 89            [ 2] 1077 	pushw	y
      00878A 89               [ 2] 1078 	pushw	x
      00878B CD 90 DE         [ 4] 1079 	call	_printf
      00878E 5B 04            [ 2] 1080 	addw	sp, #4
                                   1081 ;	../src/main.c: 199: rs485xmit_off(); // turn the transmitter back off
      008790 C6 50 0F         [ 1] 1082 	ld	a, 0x500f
      008793 A4 EF            [ 1] 1083 	and	a, #0xef
      008795 C7 50 0F         [ 1] 1084 	ld	0x500f, a
                                   1085 ;	../src/main.c: 204: address = init_mcp23017();
      008798 CD 89 82         [ 4] 1086 	call	_init_mcp23017
      00879B C7 00 1E         [ 1] 1087 	ld	_address+0, a
                                   1088 ;	../src/main.c: 206: reset_watchdog();  // reset the watchdog timer
      00879E 35 AA 50 E0      [ 1] 1089 	mov	0x50e0+0, #0xaa
                                   1090 ;	../src/main.c: 208: delay_ms(50 * (address - 0x30));	//  do this to stagger startup announcements
      0087A2 5F               [ 1] 1091 	clrw	x
      0087A3 C6 00 1E         [ 1] 1092 	ld	a, _address+0
      0087A6 97               [ 1] 1093 	ld	xl, a
      0087A7 1D 00 30         [ 2] 1094 	subw	x, #0x0030
      0087AA 89               [ 2] 1095 	pushw	x
      0087AB 4B 32            [ 1] 1096 	push	#0x32
      0087AD 4B 00            [ 1] 1097 	push	#0x00
      0087AF CD 8D 78         [ 4] 1098 	call	__mulint
      0087B2 5B 04            [ 2] 1099 	addw	sp, #4
      0087B4 90 5F            [ 1] 1100 	clrw	y
      0087B6 5D               [ 2] 1101 	tnzw	x
      0087B7 2A 02            [ 1] 1102 	jrpl	00464$
      0087B9 90 5A            [ 2] 1103 	decw	y
      0087BB                       1104 00464$:
                                   1105 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0087BB 0F 51            [ 1] 1106 	clr	(0x51, sp)
      0087BD 0F 50            [ 1] 1107 	clr	(0x50, sp)
      0087BF 0F 4F            [ 1] 1108 	clr	(0x4f, sp)
      0087C1 0F 4E            [ 1] 1109 	clr	(0x4e, sp)
      0087C3 89               [ 2] 1110 	pushw	x
      0087C4 90 89            [ 2] 1111 	pushw	y
      0087C6 4B 78            [ 1] 1112 	push	#0x78
      0087C8 4B 03            [ 1] 1113 	push	#0x03
      0087CA 5F               [ 1] 1114 	clrw	x
      0087CB 89               [ 2] 1115 	pushw	x
      0087CC CD 90 F4         [ 4] 1116 	call	__mullong
      0087CF 5B 08            [ 2] 1117 	addw	sp, #8
      0087D1 1F 40            [ 2] 1118 	ldw	(0x40, sp), x
      0087D3 17 3E            [ 2] 1119 	ldw	(0x3e, sp), y
      0087D5                       1120 00132$:
      0087D5 1E 50            [ 2] 1121 	ldw	x, (0x50, sp)
      0087D7 13 40            [ 2] 1122 	cpw	x, (0x40, sp)
      0087D9 7B 4F            [ 1] 1123 	ld	a, (0x4f, sp)
      0087DB 12 3F            [ 1] 1124 	sbc	a, (0x3f, sp)
      0087DD 7B 4E            [ 1] 1125 	ld	a, (0x4e, sp)
      0087DF 12 3E            [ 1] 1126 	sbc	a, (0x3e, sp)
      0087E1 24 17            [ 1] 1127 	jrnc	00122$
                                   1128 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
      0087E3 9D               [ 1] 1129 	nop
                                   1130 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0087E4 16 50            [ 2] 1131 	ldw	y, (0x50, sp)
      0087E6 72 A9 00 01      [ 2] 1132 	addw	y, #0x0001
      0087EA 7B 4F            [ 1] 1133 	ld	a, (0x4f, sp)
      0087EC A9 00            [ 1] 1134 	adc	a, #0x00
      0087EE 97               [ 1] 1135 	ld	xl, a
      0087EF 7B 4E            [ 1] 1136 	ld	a, (0x4e, sp)
      0087F1 A9 00            [ 1] 1137 	adc	a, #0x00
      0087F3 95               [ 1] 1138 	ld	xh, a
      0087F4 17 50            [ 2] 1139 	ldw	(0x50, sp), y
      0087F6 1F 4E            [ 2] 1140 	ldw	(0x4e, sp), x
      0087F8 20 DB            [ 2] 1141 	jra	00132$
                                   1142 ;	../src/main.c: 208: delay_ms(50 * (address - 0x30));	//  do this to stagger startup announcements
      0087FA                       1143 00122$:
                                   1144 ;	../src/main.c: 209: reset_watchdog();  // reset the watchdog timer
      0087FA 35 AA 50 E0      [ 1] 1145 	mov	0x50e0+0, #0xaa
                                   1146 ;	../src/main.c: 210: rs485xmit_on();	// turn the RS485 chips transmitter on
      0087FE C6 50 0F         [ 1] 1147 	ld	a, 0x500f
      008801 AA 10            [ 1] 1148 	or	a, #0x10
      008803 C7 50 0F         [ 1] 1149 	ld	0x500f, a
                                   1150 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008806 5F               [ 1] 1151 	clrw	x
      008807 1F 4C            [ 2] 1152 	ldw	(0x4c, sp), x
      008809 1F 4A            [ 2] 1153 	ldw	(0x4a, sp), x
      00880B                       1154 00135$:
      00880B 1E 4C            [ 2] 1155 	ldw	x, (0x4c, sp)
      00880D A3 68 10         [ 2] 1156 	cpw	x, #0x6810
      008810 7B 4B            [ 1] 1157 	ld	a, (0x4b, sp)
      008812 A2 00            [ 1] 1158 	sbc	a, #0x00
      008814 7B 4A            [ 1] 1159 	ld	a, (0x4a, sp)
      008816 A2 00            [ 1] 1160 	sbc	a, #0x00
      008818 24 17            [ 1] 1161 	jrnc	00124$
                                   1162 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 14: __asm__("nop");
      00881A 9D               [ 1] 1163 	nop
                                   1164 ;	/home/scott/projects-stm8/pvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      00881B 16 4C            [ 2] 1165 	ldw	y, (0x4c, sp)
      00881D 72 A9 00 01      [ 2] 1166 	addw	y, #0x0001
      008821 7B 4B            [ 1] 1167 	ld	a, (0x4b, sp)
      008823 A9 00            [ 1] 1168 	adc	a, #0x00
      008825 97               [ 1] 1169 	ld	xl, a
      008826 7B 4A            [ 1] 1170 	ld	a, (0x4a, sp)
      008828 A9 00            [ 1] 1171 	adc	a, #0x00
      00882A 95               [ 1] 1172 	ld	xh, a
      00882B 17 4C            [ 2] 1173 	ldw	(0x4c, sp), y
      00882D 1F 4A            [ 2] 1174 	ldw	(0x4a, sp), x
      00882F 20 DA            [ 2] 1175 	jra	00135$
                                   1176 ;	../src/main.c: 211: delay_ms(30);
      008831                       1177 00124$:
                                   1178 ;	../src/main.c: 212: printf("%c:Running:%s:%02x\r\n",address,version,address);
      008831 5F               [ 1] 1179 	clrw	x
      008832 C6 00 1E         [ 1] 1180 	ld	a, _address+0
      008835 97               [ 1] 1181 	ld	xl, a
      008836 16 38            [ 2] 1182 	ldw	y, (0x38, sp)
      008838 17 3C            [ 2] 1183 	ldw	(0x3c, sp), y
      00883A 90 AE 89 6D      [ 2] 1184 	ldw	y, #___str_5+0
      00883E 89               [ 2] 1185 	pushw	x
      00883F 7B 3F            [ 1] 1186 	ld	a, (0x3f, sp)
      008841 88               [ 1] 1187 	push	a
      008842 7B 3F            [ 1] 1188 	ld	a, (0x3f, sp)
      008844 88               [ 1] 1189 	push	a
      008845 89               [ 2] 1190 	pushw	x
      008846 90 89            [ 2] 1191 	pushw	y
      008848 CD 90 DE         [ 4] 1192 	call	_printf
      00884B 5B 08            [ 2] 1193 	addw	sp, #8
                                   1194 ;	../src/main.c: 213: rs485xmit_off(); // turn the transmitter back off
      00884D C6 50 0F         [ 1] 1195 	ld	a, 0x500f
      008850 A4 EF            [ 1] 1196 	and	a, #0xef
      008852 C7 50 0F         [ 1] 1197 	ld	0x500f, a
                                   1198 ;	../src/main.c: 216: do{
      008855                       1199 00114$:
                                   1200 ;	../src/main.c: 217: reset_watchdog();  // reset the watchdog timer
      008855 35 AA 50 E0      [ 1] 1201 	mov	0x50e0+0, #0xaa
                                   1202 ;	../src/main.c: 218: if(UART_read_byte(&rb)){ // buffer isn't empty
      008859 96               [ 1] 1203 	ldw	x, sp
      00885A 5C               [ 1] 1204 	incw	x
      00885B 89               [ 2] 1205 	pushw	x
      00885C CD 8A 7F         [ 4] 1206 	call	_UART_read_byte
      00885F 5B 02            [ 2] 1207 	addw	sp, #2
      008861 4D               [ 1] 1208 	tnz	a
      008862 27 42            [ 1] 1209 	jreq	00108$
                                   1210 ;	../src/main.c: 219: switch(rb){
      008864 7B 01            [ 1] 1211 	ld	a, (0x01, sp)
      008866 A1 1B            [ 1] 1212 	cp	a, #0x1b
      008868 26 10            [ 1] 1213 	jrne	00102$
                                   1214 ;	../src/main.c: 221: esc = 1;	// set the flag to show an escpage character was received
      00886A 35 01 00 1D      [ 1] 1215 	mov	_esc+0, #0x01
                                   1216 ;	../src/main.c: 222: esc_time = Global_time;	// only wait 2 seconds for the next character after the escape
      00886E CE 00 03         [ 2] 1217 	ldw	x, _Global_time+2
      008871 1F 5C            [ 2] 1218 	ldw	(0x5c, sp), x
      008873 CE 00 01         [ 2] 1219 	ldw	x, _Global_time+0
      008876 1F 5A            [ 2] 1220 	ldw	(0x5a, sp), x
                                   1221 ;	../src/main.c: 223: break;
      008878 20 2C            [ 2] 1222 	jra	00108$
                                   1223 ;	../src/main.c: 224: default:
      00887A                       1224 00102$:
                                   1225 ;	../src/main.c: 225: if (rb == address && esc)  // address must match the switches read by mcp23017
      00887A 7B 01            [ 1] 1226 	ld	a, (0x01, sp)
      00887C C1 00 1E         [ 1] 1227 	cp	a, _address+0
      00887F 26 21            [ 1] 1228 	jrne	00104$
      008881 72 5D 00 1D      [ 1] 1229 	tnz	_esc+0
      008885 27 1B            [ 1] 1230 	jreq	00104$
                                   1231 ;	../src/main.c: 227: Global_time = 0L;   // when was the last time we were called?
      008887 5F               [ 1] 1232 	clrw	x
      008888 CF 00 03         [ 2] 1233 	ldw	_Global_time+2, x
      00888B CF 00 01         [ 2] 1234 	ldw	_Global_time+0, x
                                   1235 ;	../src/main.c: 228: measure(1);			// do a measurement, and send the results
      00888E 4B 01            [ 1] 1236 	push	#0x01
      008890 CD 82 A6         [ 4] 1237 	call	_measure
      008893 84               [ 1] 1238 	pop	a
                                   1239 ;	../src/main.c: 229: last_measure = Global_time;
      008894 CE 00 03         [ 2] 1240 	ldw	x, _Global_time+2
      008897 90 CE 00 01      [ 2] 1241 	ldw	y, _Global_time+0
      00889B CF 00 4B         [ 2] 1242 	ldw	_last_measure+2, x
      00889E 90 CF 00 49      [ 2] 1243 	ldw	_last_measure+0, y
      0088A2                       1244 00104$:
                                   1245 ;	../src/main.c: 231: esc = 0;	// reset the escape character flag
      0088A2 72 5F 00 1D      [ 1] 1246 	clr	_esc+0
                                   1247 ;	../src/main.c: 232: }
      0088A6                       1248 00108$:
                                   1249 ;	../src/main.c: 234: if (esc && (Global_time - esc_time > 2000))  // give it 2 seconds
      0088A6 72 5D 00 1D      [ 1] 1250 	tnz	_esc+0
      0088AA 27 27            [ 1] 1251 	jreq	00110$
      0088AC CE 00 03         [ 2] 1252 	ldw	x, _Global_time+2
      0088AF 72 F0 5C         [ 2] 1253 	subw	x, (0x5c, sp)
      0088B2 1F 48            [ 2] 1254 	ldw	(0x48, sp), x
      0088B4 C6 00 02         [ 1] 1255 	ld	a, _Global_time+1
      0088B7 12 5B            [ 1] 1256 	sbc	a, (0x5b, sp)
      0088B9 6B 47            [ 1] 1257 	ld	(0x47, sp), a
      0088BB C6 00 01         [ 1] 1258 	ld	a, _Global_time+0
      0088BE 12 5A            [ 1] 1259 	sbc	a, (0x5a, sp)
      0088C0 6B 46            [ 1] 1260 	ld	(0x46, sp), a
      0088C2 AE 07 D0         [ 2] 1261 	ldw	x, #0x07d0
      0088C5 13 48            [ 2] 1262 	cpw	x, (0x48, sp)
      0088C7 4F               [ 1] 1263 	clr	a
      0088C8 12 47            [ 1] 1264 	sbc	a, (0x47, sp)
      0088CA 4F               [ 1] 1265 	clr	a
      0088CB 12 46            [ 1] 1266 	sbc	a, (0x46, sp)
      0088CD 24 04            [ 1] 1267 	jrnc	00110$
                                   1268 ;	../src/main.c: 235: esc = 0;  // reset the esc, since it should have been followed by the id right away
      0088CF 72 5F 00 1D      [ 1] 1269 	clr	_esc+0
      0088D3                       1270 00110$:
                                   1271 ;	../src/main.c: 236: if (Global_time - last_measure > 600000L)  // every 10 minutes take a silent measurement
      0088D3 CE 00 03         [ 2] 1272 	ldw	x, _Global_time+2
      0088D6 72 B0 00 4B      [ 2] 1273 	subw	x, _last_measure+2
      0088DA 1F 44            [ 2] 1274 	ldw	(0x44, sp), x
      0088DC C6 00 02         [ 1] 1275 	ld	a, _Global_time+1
      0088DF C2 00 4A         [ 1] 1276 	sbc	a, _last_measure+1
      0088E2 6B 43            [ 1] 1277 	ld	(0x43, sp), a
      0088E4 C6 00 01         [ 1] 1278 	ld	a, _Global_time+0
      0088E7 C2 00 49         [ 1] 1279 	sbc	a, _last_measure+0
      0088EA 6B 42            [ 1] 1280 	ld	(0x42, sp), a
      0088EC AE 27 C0         [ 2] 1281 	ldw	x, #0x27c0
      0088EF 13 44            [ 2] 1282 	cpw	x, (0x44, sp)
      0088F1 A6 09            [ 1] 1283 	ld	a, #0x09
      0088F3 12 43            [ 1] 1284 	sbc	a, (0x43, sp)
      0088F5 4F               [ 1] 1285 	clr	a
      0088F6 12 42            [ 1] 1286 	sbc	a, (0x42, sp)
      0088F8 25 03            [ 1] 1287 	jrc	00477$
      0088FA CC 88 55         [ 2] 1288 	jp	00114$
      0088FD                       1289 00477$:
                                   1290 ;	../src/main.c: 238: measure(0);
      0088FD 4B 00            [ 1] 1291 	push	#0x00
      0088FF CD 82 A6         [ 4] 1292 	call	_measure
      008902 84               [ 1] 1293 	pop	a
                                   1294 ;	../src/main.c: 239: last_measure = Global_time;
      008903 CE 00 03         [ 2] 1295 	ldw	x, _Global_time+2
      008906 90 CE 00 01      [ 2] 1296 	ldw	y, _Global_time+0
      00890A CF 00 4B         [ 2] 1297 	ldw	_last_measure+2, x
      00890D 90 CF 00 49      [ 2] 1298 	ldw	_last_measure+0, y
                                   1299 ;	../src/main.c: 241: }while(1);
      008911 CC 88 55         [ 2] 1300 	jp	00114$
                                   1301 ;	../src/main.c: 242: }
      008914 5B 5D            [ 2] 1302 	addw	sp, #93
      008916 81               [ 4] 1303 	ret
                                   1304 	.area CODE
      008917                       1305 ___str_0:
      008917 25 63 3A 25 32 64 2E  1306 	.ascii "%c:%2d.%1d%%:%3d.%1d"
             25 31 64 25 25 3A 25
             33 64 2E 25 31 64
      00892B 0D                    1307 	.db 0x0d
      00892C 0A                    1308 	.db 0x0a
      00892D 00                    1309 	.db 0x00
      00892E                       1310 ___str_1:
      00892E 25 30 32 64 25 30 32  1311 	.ascii "%02d%02d%02d-%02d%02d"
             64 25 30 32 64 2D 25
             30 32 64 25 30 32 64
      008943 00                    1312 	.db 0x00
      008944                       1313 ___str_2:
      008944 4F 63 74 20 20 35 20  1314 	.ascii "Oct  5 2018"
             32 30 31 38
      00894F 00                    1315 	.db 0x00
      008950                       1316 ___str_3:
      008950 32 33 3A 31 34 3A 34  1317 	.ascii "23:14:42"
             32
      008958 00                    1318 	.db 0x00
      008959                       1319 ___str_4:
      008959 49 6E 69 74 69 61 6C  1320 	.ascii "Initializing...%s"
             69 7A 69 6E 67 2E 2E
             2E 25 73
      00896A 0D                    1321 	.db 0x0d
      00896B 0A                    1322 	.db 0x0a
      00896C 00                    1323 	.db 0x00
      00896D                       1324 ___str_5:
      00896D 25 63 3A 52 75 6E 6E  1325 	.ascii "%c:Running:%s:%02x"
             69 6E 67 3A 25 73 3A
             25 30 32 78
      00897F 0D                    1326 	.db 0x0d
      008980 0A                    1327 	.db 0x0a
      008981 00                    1328 	.db 0x00
                                   1329 	.area INITIALIZER
      0099ED                       1330 __xinit__last_measure:
      0099ED 00 00 00 00           1331 	.byte #0x00,#0x00,#0x00,#0x00	; 0
                                   1332 	.area CABS (ABS)
