                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.7.0 #10231 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module LCD
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _font6x8
                                     12 	.globl _SPI_write
                                     13 	.globl _strlen
                                     14 	.globl _LCD_init
                                     15 	.globl _LCD_cmd
                                     16 	.globl _LCD_write
                                     17 	.globl _LCD_clear
                                     18 	.globl _LCD_goto
                                     19 	.globl _LCD_putc
                                     20 	.globl _LCD_printf
                                     21 ;--------------------------------------------------------
                                     22 ; ram data
                                     23 ;--------------------------------------------------------
                                     24 	.area DATA
                                     25 ;--------------------------------------------------------
                                     26 ; ram data
                                     27 ;--------------------------------------------------------
                                     28 	.area INITIALIZED
                                     29 ;--------------------------------------------------------
                                     30 ; absolute external ram data
                                     31 ;--------------------------------------------------------
                                     32 	.area DABS (ABS)
                                     33 ;--------------------------------------------------------
                                     34 ; global & static initialisations
                                     35 ;--------------------------------------------------------
                                     36 	.area HOME
                                     37 	.area GSINIT
                                     38 	.area GSFINAL
                                     39 	.area GSINIT
                                     40 ;--------------------------------------------------------
                                     41 ; Home
                                     42 ;--------------------------------------------------------
                                     43 	.area HOME
                                     44 	.area HOME
                                     45 ;--------------------------------------------------------
                                     46 ; code
                                     47 ;--------------------------------------------------------
                                     48 	.area CODE
                                     49 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 30: static inline void LCD_gpio_init() {
                                     50 ;	-----------------------------------------
                                     51 ;	 function LCD_gpio_init
                                     52 ;	-----------------------------------------
      00808C                         53 _LCD_gpio_init:
                                     54 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 31: GPIOD->DDR |= (1 << LCD_CE_PIN) | (1 << LCD_RST_PIN);
      00808C C6 50 11         [ 1]   55 	ld	a, 0x5011
      00808F AA 06            [ 1]   56 	or	a, #0x06
      008091 C7 50 11         [ 1]   57 	ld	0x5011, a
                                     58 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 32: GPIOD->CR1 |= (1 << LCD_CE_PIN) | (1 << LCD_RST_PIN);
      008094 C6 50 12         [ 1]   59 	ld	a, 0x5012
      008097 AA 06            [ 1]   60 	or	a, #0x06
      008099 C7 50 12         [ 1]   61 	ld	0x5012, a
                                     62 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 33: GPIOC->DDR |= (1 << LCD_DC_PIN);
      00809C 72 18 50 0C      [ 1]   63 	bset	20492, #4
                                     64 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 34: GPIOC->CR1 |= (1 << LCD_DC_PIN);
      0080A0 72 18 50 0D      [ 1]   65 	bset	20493, #4
                                     66 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 35: }
      0080A4 81               [ 4]   67 	ret
                                     68 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 37: static inline void LCD_DC_set() {
                                     69 ;	-----------------------------------------
                                     70 ;	 function LCD_DC_set
                                     71 ;	-----------------------------------------
      0080A5                         72 _LCD_DC_set:
                                     73 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 38: GPIOC->ODR |= (1 << LCD_DC_PIN);
      0080A5 72 18 50 0A      [ 1]   74 	bset	20490, #4
                                     75 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 39: }
      0080A9 81               [ 4]   76 	ret
                                     77 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 41: static inline void LCD_DC_clear() {
                                     78 ;	-----------------------------------------
                                     79 ;	 function LCD_DC_clear
                                     80 ;	-----------------------------------------
      0080AA                         81 _LCD_DC_clear:
                                     82 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 42: GPIOC->ODR &= ~(1 << LCD_DC_PIN);
      0080AA 72 19 50 0A      [ 1]   83 	bres	20490, #4
                                     84 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 43: }
      0080AE 81               [ 4]   85 	ret
                                     86 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 45: static inline void LCD_CE_set() {
                                     87 ;	-----------------------------------------
                                     88 ;	 function LCD_CE_set
                                     89 ;	-----------------------------------------
      0080AF                         90 _LCD_CE_set:
                                     91 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 46: while ((SPI->SR & SPI_SR_BSY));
      0080AF                         92 00101$:
      0080AF C6 52 03         [ 1]   93 	ld	a, 0x5203
      0080B2 4D               [ 1]   94 	tnz	a
      0080B3 2B FA            [ 1]   95 	jrmi	00101$
                                     96 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 47: GPIOD->ODR |= (1 << LCD_CE_PIN);
      0080B5 72 12 50 0F      [ 1]   97 	bset	20495, #1
                                     98 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 48: }
      0080B9 81               [ 4]   99 	ret
                                    100 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 50: static inline void LCD_CE_clear() {
                                    101 ;	-----------------------------------------
                                    102 ;	 function LCD_CE_clear
                                    103 ;	-----------------------------------------
      0080BA                        104 _LCD_CE_clear:
                                    105 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 51: GPIOD->ODR &= ~(1 << LCD_CE_PIN);
      0080BA 72 13 50 0F      [ 1]  106 	bres	20495, #1
                                    107 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 52: }
      0080BE 81               [ 4]  108 	ret
                                    109 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 54: static inline void LCD_RST_set() {
                                    110 ;	-----------------------------------------
                                    111 ;	 function LCD_RST_set
                                    112 ;	-----------------------------------------
      0080BF                        113 _LCD_RST_set:
                                    114 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 55: GPIOD->ODR |= (1 << LCD_RST_PIN);
      0080BF 72 14 50 0F      [ 1]  115 	bset	20495, #2
                                    116 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 56: }
      0080C3 81               [ 4]  117 	ret
                                    118 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 58: static inline void LCD_RST_clear() {
                                    119 ;	-----------------------------------------
                                    120 ;	 function LCD_RST_clear
                                    121 ;	-----------------------------------------
      0080C4                        122 _LCD_RST_clear:
                                    123 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 59: GPIOD->ODR &= ~(1 << LCD_RST_PIN);
      0080C4 72 15 50 0F      [ 1]  124 	bres	20495, #2
                                    125 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 60: }
      0080C8 81               [ 4]  126 	ret
                                    127 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 62: static inline void LCD_SPI_write(uint8_t word) {
                                    128 ;	-----------------------------------------
                                    129 ;	 function LCD_SPI_write
                                    130 ;	-----------------------------------------
      0080C9                        131 _LCD_SPI_write:
                                    132 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 63: SPI_write(word);
      0080C9 7B 03            [ 1]  133 	ld	a, (0x03, sp)
      0080CB 88               [ 1]  134 	push	a
      0080CC CD 90 02         [ 4]  135 	call	_SPI_write
      0080CF 84               [ 1]  136 	pop	a
                                    137 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 64: }
      0080D0 81               [ 4]  138 	ret
                                    139 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 66: static inline void LCD_delay_ms(int ms) {
                                    140 ;	-----------------------------------------
                                    141 ;	 function LCD_delay_ms
                                    142 ;	-----------------------------------------
      0080D1                        143 _LCD_delay_ms:
      0080D1 52 08            [ 2]  144 	sub	sp, #8
                                    145 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 67: delay_ms(ms);
      0080D3 16 0B            [ 2]  146 	ldw	y, (0x0b, sp)
      0080D5 5F               [ 1]  147 	clrw	x
      0080D6 90 5D            [ 2]  148 	tnzw	y
      0080D8 2A 01            [ 1]  149 	jrpl	00116$
      0080DA 5A               [ 2]  150 	decw	x
      0080DB                        151 00116$:
                                    152 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0080DB 0F 08            [ 1]  153 	clr	(0x08, sp)
      0080DD 0F 07            [ 1]  154 	clr	(0x07, sp)
      0080DF 0F 06            [ 1]  155 	clr	(0x06, sp)
      0080E1 0F 05            [ 1]  156 	clr	(0x05, sp)
      0080E3 90 89            [ 2]  157 	pushw	y
      0080E5 89               [ 2]  158 	pushw	x
      0080E6 4B 78            [ 1]  159 	push	#0x78
      0080E8 4B 03            [ 1]  160 	push	#0x03
      0080EA 5F               [ 1]  161 	clrw	x
      0080EB 89               [ 2]  162 	pushw	x
      0080EC CD 96 CB         [ 4]  163 	call	__mullong
      0080EF 5B 08            [ 2]  164 	addw	sp, #8
      0080F1 1F 03            [ 2]  165 	ldw	(0x03, sp), x
      0080F3 17 01            [ 2]  166 	ldw	(0x01, sp), y
      0080F5                        167 00104$:
      0080F5 1E 07            [ 2]  168 	ldw	x, (0x07, sp)
      0080F7 13 03            [ 2]  169 	cpw	x, (0x03, sp)
      0080F9 7B 06            [ 1]  170 	ld	a, (0x06, sp)
      0080FB 12 02            [ 1]  171 	sbc	a, (0x02, sp)
      0080FD 7B 05            [ 1]  172 	ld	a, (0x05, sp)
      0080FF 12 01            [ 1]  173 	sbc	a, (0x01, sp)
      008101 24 17            [ 1]  174 	jrnc	00106$
                                    175 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 14: __asm__("nop");
      008103 9D               [ 1]  176 	nop
                                    177 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008104 16 07            [ 2]  178 	ldw	y, (0x07, sp)
      008106 72 A9 00 01      [ 2]  179 	addw	y, #0x0001
      00810A 7B 06            [ 1]  180 	ld	a, (0x06, sp)
      00810C A9 00            [ 1]  181 	adc	a, #0x00
      00810E 97               [ 1]  182 	ld	xl, a
      00810F 7B 05            [ 1]  183 	ld	a, (0x05, sp)
      008111 A9 00            [ 1]  184 	adc	a, #0x00
      008113 95               [ 1]  185 	ld	xh, a
      008114 17 07            [ 2]  186 	ldw	(0x07, sp), y
      008116 1F 05            [ 2]  187 	ldw	(0x05, sp), x
      008118 20 DB            [ 2]  188 	jra	00104$
                                    189 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 67: delay_ms(ms);
      00811A                        190 00106$:
                                    191 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 68: }
      00811A 5B 08            [ 2]  192 	addw	sp, #8
      00811C 81               [ 4]  193 	ret
                                    194 ;	../src/LCD.c: 7: void LCD_init() {
                                    195 ;	-----------------------------------------
                                    196 ;	 function LCD_init
                                    197 ;	-----------------------------------------
      00811D                        198 _LCD_init:
                                    199 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 31: GPIOD->DDR |= (1 << LCD_CE_PIN) | (1 << LCD_RST_PIN);
      00811D C6 50 11         [ 1]  200 	ld	a, 0x5011
      008120 AA 06            [ 1]  201 	or	a, #0x06
      008122 C7 50 11         [ 1]  202 	ld	0x5011, a
                                    203 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 32: GPIOD->CR1 |= (1 << LCD_CE_PIN) | (1 << LCD_RST_PIN);
      008125 C6 50 12         [ 1]  204 	ld	a, 0x5012
      008128 AA 06            [ 1]  205 	or	a, #0x06
      00812A C7 50 12         [ 1]  206 	ld	0x5012, a
                                    207 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 33: GPIOC->DDR |= (1 << LCD_DC_PIN);
      00812D 72 18 50 0C      [ 1]  208 	bset	20492, #4
                                    209 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 34: GPIOC->CR1 |= (1 << LCD_DC_PIN);
      008131 72 18 50 0D      [ 1]  210 	bset	20493, #4
                                    211 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 51: GPIOD->ODR &= ~(1 << LCD_CE_PIN);
      008135 72 13 50 0F      [ 1]  212 	bres	20495, #1
                                    213 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 59: GPIOD->ODR &= ~(1 << LCD_RST_PIN);
      008139 C6 50 0F         [ 1]  214 	ld	a, 0x500f
      00813C A4 FB            [ 1]  215 	and	a, #0xfb
      00813E C7 50 0F         [ 1]  216 	ld	0x500f, a
                                    217 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008141 90 5F            [ 1]  218 	clrw	y
      008143 5F               [ 1]  219 	clrw	x
      008144                        220 00113$:
      008144 90 A3 AD 70      [ 2]  221 	cpw	y, #0xad70
      008148 9F               [ 1]  222 	ld	a, xl
      008149 A2 00            [ 1]  223 	sbc	a, #0x00
      00814B 9E               [ 1]  224 	ld	a, xh
      00814C A2 00            [ 1]  225 	sbc	a, #0x00
      00814E 24 08            [ 1]  226 	jrnc	00105$
                                    227 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 14: __asm__("nop");
      008150 9D               [ 1]  228 	nop
                                    229 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008151 90 5C            [ 1]  230 	incw	y
      008153 26 EF            [ 1]  231 	jrne	00113$
      008155 5C               [ 1]  232 	incw	x
      008156 20 EC            [ 2]  233 	jra	00113$
                                    234 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 67: delay_ms(ms);
      008158                        235 00105$:
                                    236 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 55: GPIOD->ODR |= (1 << LCD_RST_PIN);
      008158 72 14 50 0F      [ 1]  237 	bset	20495, #2
                                    238 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 46: while ((SPI->SR & SPI_SR_BSY));
      00815C                        239 00108$:
      00815C C6 52 03         [ 1]  240 	ld	a, 0x5203
      00815F 4D               [ 1]  241 	tnz	a
      008160 2B FA            [ 1]  242 	jrmi	00108$
                                    243 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 47: GPIOD->ODR |= (1 << LCD_CE_PIN);
      008162 72 12 50 0F      [ 1]  244 	bset	20495, #1
                                    245 ;	../src/LCD.c: 14: LCD_cmd(0x21); // extended commands
      008166 4B 21            [ 1]  246 	push	#0x21
      008168 CD 81 8D         [ 4]  247 	call	_LCD_cmd
      00816B 84               [ 1]  248 	pop	a
                                    249 ;	../src/LCD.c: 15: LCD_cmd(0xc1); // contrast Vop=6.4V
      00816C 4B C1            [ 1]  250 	push	#0xc1
      00816E CD 81 8D         [ 4]  251 	call	_LCD_cmd
      008171 84               [ 1]  252 	pop	a
                                    253 ;	../src/LCD.c: 16: LCD_cmd(0x04); // temperature coefficient
      008172 4B 04            [ 1]  254 	push	#0x04
      008174 CD 81 8D         [ 4]  255 	call	_LCD_cmd
      008177 84               [ 1]  256 	pop	a
                                    257 ;	../src/LCD.c: 17: LCD_cmd(0x13); // bias = 1:48
      008178 4B 13            [ 1]  258 	push	#0x13
      00817A CD 81 8D         [ 4]  259 	call	_LCD_cmd
      00817D 84               [ 1]  260 	pop	a
                                    261 ;	../src/LCD.c: 19: LCD_cmd(0x20); // standard commands
      00817E 4B 20            [ 1]  262 	push	#0x20
      008180 CD 81 8D         [ 4]  263 	call	_LCD_cmd
      008183 84               [ 1]  264 	pop	a
                                    265 ;	../src/LCD.c: 20: LCD_cmd(0x0C); // normal mode
      008184 4B 0C            [ 1]  266 	push	#0x0c
      008186 CD 81 8D         [ 4]  267 	call	_LCD_cmd
      008189 84               [ 1]  268 	pop	a
                                    269 ;	../src/LCD.c: 22: LCD_clear();
                                    270 ;	../src/LCD.c: 23: }
      00818A CC 81 C1         [ 2]  271 	jp	_LCD_clear
                                    272 ;	../src/LCD.c: 25: void LCD_cmd(uint8_t cmd) {
                                    273 ;	-----------------------------------------
                                    274 ;	 function LCD_cmd
                                    275 ;	-----------------------------------------
      00818D                        276 _LCD_cmd:
                                    277 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 51: GPIOD->ODR &= ~(1 << LCD_CE_PIN);
      00818D 72 13 50 0F      [ 1]  278 	bres	20495, #1
                                    279 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 42: GPIOC->ODR &= ~(1 << LCD_DC_PIN);
      008191 72 19 50 0A      [ 1]  280 	bres	20490, #4
                                    281 ;	../src/LCD.c: 28: LCD_SPI_write(cmd);
      008195 7B 03            [ 1]  282 	ld	a, (0x03, sp)
                                    283 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 63: SPI_write(word);
      008197 88               [ 1]  284 	push	a
      008198 CD 90 02         [ 4]  285 	call	_SPI_write
      00819B 84               [ 1]  286 	pop	a
                                    287 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 46: while ((SPI->SR & SPI_SR_BSY));
      00819C                        288 00104$:
      00819C C6 52 03         [ 1]  289 	ld	a, 0x5203
      00819F 4D               [ 1]  290 	tnz	a
      0081A0 2B FA            [ 1]  291 	jrmi	00104$
                                    292 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 47: GPIOD->ODR |= (1 << LCD_CE_PIN);
      0081A2 72 12 50 0F      [ 1]  293 	bset	20495, #1
                                    294 ;	../src/LCD.c: 29: LCD_CE_set();
                                    295 ;	../src/LCD.c: 30: }
      0081A6 81               [ 4]  296 	ret
                                    297 ;	../src/LCD.c: 32: void LCD_write(uint8_t data) {
                                    298 ;	-----------------------------------------
                                    299 ;	 function LCD_write
                                    300 ;	-----------------------------------------
      0081A7                        301 _LCD_write:
                                    302 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 51: GPIOD->ODR &= ~(1 << LCD_CE_PIN);
      0081A7 72 13 50 0F      [ 1]  303 	bres	20495, #1
                                    304 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 38: GPIOC->ODR |= (1 << LCD_DC_PIN);
      0081AB 72 18 50 0A      [ 1]  305 	bset	20490, #4
                                    306 ;	../src/LCD.c: 35: LCD_SPI_write(data);
      0081AF 7B 03            [ 1]  307 	ld	a, (0x03, sp)
                                    308 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 63: SPI_write(word);
      0081B1 88               [ 1]  309 	push	a
      0081B2 CD 90 02         [ 4]  310 	call	_SPI_write
      0081B5 84               [ 1]  311 	pop	a
                                    312 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 46: while ((SPI->SR & SPI_SR_BSY));
      0081B6                        313 00104$:
      0081B6 C6 52 03         [ 1]  314 	ld	a, 0x5203
      0081B9 4D               [ 1]  315 	tnz	a
      0081BA 2B FA            [ 1]  316 	jrmi	00104$
                                    317 ;	/home/scott/projects-stm8/lvcc-tunnels/inc/LCD_config.h: 47: GPIOD->ODR |= (1 << LCD_CE_PIN);
      0081BC 72 12 50 0F      [ 1]  318 	bset	20495, #1
                                    319 ;	../src/LCD.c: 36: LCD_CE_set();
                                    320 ;	../src/LCD.c: 37: }
      0081C0 81               [ 4]  321 	ret
                                    322 ;	../src/LCD.c: 39: void LCD_clear() {
                                    323 ;	-----------------------------------------
                                    324 ;	 function LCD_clear
                                    325 ;	-----------------------------------------
      0081C1                        326 _LCD_clear:
      0081C1 52 02            [ 2]  327 	sub	sp, #2
                                    328 ;	../src/LCD.c: 40: uint16_t i = 84 * 6;
      0081C3 AE 01 F8         [ 2]  329 	ldw	x, #0x01f8
                                    330 ;	../src/LCD.c: 41: LCD_goto(0, 0);
      0081C6 89               [ 2]  331 	pushw	x
      0081C7 4B 00            [ 1]  332 	push	#0x00
      0081C9 4B 00            [ 1]  333 	push	#0x00
      0081CB CD 81 E5         [ 4]  334 	call	_LCD_goto
      0081CE 5B 02            [ 2]  335 	addw	sp, #2
      0081D0 85               [ 2]  336 	popw	x
                                    337 ;	../src/LCD.c: 42: while (i-- > 0)
      0081D1                        338 00101$:
      0081D1 1F 01            [ 2]  339 	ldw	(0x01, sp), x
      0081D3 5A               [ 2]  340 	decw	x
      0081D4 16 01            [ 2]  341 	ldw	y, (0x01, sp)
      0081D6 27 0A            [ 1]  342 	jreq	00104$
                                    343 ;	../src/LCD.c: 43: LCD_write(0);
      0081D8 89               [ 2]  344 	pushw	x
      0081D9 4B 00            [ 1]  345 	push	#0x00
      0081DB CD 81 A7         [ 4]  346 	call	_LCD_write
      0081DE 84               [ 1]  347 	pop	a
      0081DF 85               [ 2]  348 	popw	x
      0081E0 20 EF            [ 2]  349 	jra	00101$
      0081E2                        350 00104$:
                                    351 ;	../src/LCD.c: 44: }
      0081E2 5B 02            [ 2]  352 	addw	sp, #2
      0081E4 81               [ 4]  353 	ret
                                    354 ;	../src/LCD.c: 46: void LCD_goto(uint8_t col, uint8_t row) {
                                    355 ;	-----------------------------------------
                                    356 ;	 function LCD_goto
                                    357 ;	-----------------------------------------
      0081E5                        358 _LCD_goto:
                                    359 ;	../src/LCD.c: 47: LCD_cmd(0x80 | col);
      0081E5 7B 03            [ 1]  360 	ld	a, (0x03, sp)
      0081E7 41               [ 1]  361 	exg	a, xl
      0081E8 4F               [ 1]  362 	clr	a
      0081E9 41               [ 1]  363 	exg	a, xl
      0081EA AA 80            [ 1]  364 	or	a, #0x80
      0081EC 88               [ 1]  365 	push	a
      0081ED CD 81 8D         [ 4]  366 	call	_LCD_cmd
      0081F0 84               [ 1]  367 	pop	a
                                    368 ;	../src/LCD.c: 48: LCD_cmd(0x40 | row);
      0081F1 7B 04            [ 1]  369 	ld	a, (0x04, sp)
      0081F3 41               [ 1]  370 	exg	a, xl
      0081F4 4F               [ 1]  371 	clr	a
      0081F5 41               [ 1]  372 	exg	a, xl
      0081F6 AA 40            [ 1]  373 	or	a, #0x40
      0081F8 88               [ 1]  374 	push	a
      0081F9 CD 81 8D         [ 4]  375 	call	_LCD_cmd
      0081FC 84               [ 1]  376 	pop	a
                                    377 ;	../src/LCD.c: 49: }
      0081FD 81               [ 4]  378 	ret
                                    379 ;	../src/LCD.c: 51: void LCD_putc(char c) {
                                    380 ;	-----------------------------------------
                                    381 ;	 function LCD_putc
                                    382 ;	-----------------------------------------
      0081FE                        383 _LCD_putc:
      0081FE 52 05            [ 2]  384 	sub	sp, #5
                                    385 ;	../src/LCD.c: 53: if (c == ' ') {
      008200 7B 08            [ 1]  386 	ld	a, (0x08, sp)
      008202 A1 20            [ 1]  387 	cp	a, #0x20
      008204 26 0E            [ 1]  388 	jrne	00103$
                                    389 ;	../src/LCD.c: 54: LCD_write(0);
      008206 4B 00            [ 1]  390 	push	#0x00
      008208 CD 81 A7         [ 4]  391 	call	_LCD_write
      00820B 84               [ 1]  392 	pop	a
                                    393 ;	../src/LCD.c: 55: LCD_write(0);
      00820C 4B 00            [ 1]  394 	push	#0x00
      00820E CD 81 A7         [ 4]  395 	call	_LCD_write
      008211 84               [ 1]  396 	pop	a
      008212 20 33            [ 2]  397 	jra	00107$
      008214                        398 00103$:
                                    399 ;	../src/LCD.c: 57: c -= 32;  // just printable characters
      008214 7B 08            [ 1]  400 	ld	a, (0x08, sp)
      008216 A0 20            [ 1]  401 	sub	a, #0x20
      008218 6B 08            [ 1]  402 	ld	(0x08, sp), a
                                    403 ;	../src/LCD.c: 59: for (i=0; i<6; i++)
      00821A 0F 05            [ 1]  404 	clr	(0x05, sp)
      00821C AE 82 73         [ 2]  405 	ldw	x, #_font6x8+0
      00821F 1F 03            [ 2]  406 	ldw	(0x03, sp), x
      008221 7B 08            [ 1]  407 	ld	a, (0x08, sp)
      008223 97               [ 1]  408 	ld	xl, a
      008224 A6 06            [ 1]  409 	ld	a, #0x06
      008226 42               [ 4]  410 	mul	x, a
      008227 72 FB 03         [ 2]  411 	addw	x, (0x03, sp)
      00822A 1F 01            [ 2]  412 	ldw	(0x01, sp), x
      00822C                        413 00105$:
                                    414 ;	../src/LCD.c: 60: LCD_write(font6x8[c][i]);
      00822C 5F               [ 1]  415 	clrw	x
      00822D 7B 05            [ 1]  416 	ld	a, (0x05, sp)
      00822F 97               [ 1]  417 	ld	xl, a
      008230 72 FB 01         [ 2]  418 	addw	x, (0x01, sp)
      008233 F6               [ 1]  419 	ld	a, (x)
      008234 88               [ 1]  420 	push	a
      008235 CD 81 A7         [ 4]  421 	call	_LCD_write
      008238 84               [ 1]  422 	pop	a
                                    423 ;	../src/LCD.c: 59: for (i=0; i<6; i++)
      008239 0C 05            [ 1]  424 	inc	(0x05, sp)
      00823B 7B 05            [ 1]  425 	ld	a, (0x05, sp)
      00823D A1 06            [ 1]  426 	cp	a, #0x06
      00823F 25 EB            [ 1]  427 	jrc	00105$
                                    428 ;	../src/LCD.c: 61: LCD_write(0);
      008241 4B 00            [ 1]  429 	push	#0x00
      008243 CD 81 A7         [ 4]  430 	call	_LCD_write
      008246 84               [ 1]  431 	pop	a
      008247                        432 00107$:
                                    433 ;	../src/LCD.c: 64: }
      008247 5B 05            [ 2]  434 	addw	sp, #5
      008249 81               [ 4]  435 	ret
                                    436 ;	../src/LCD.c: 66: void LCD_printf(char *s) {
                                    437 ;	-----------------------------------------
                                    438 ;	 function LCD_printf
                                    439 ;	-----------------------------------------
      00824A                        440 _LCD_printf:
      00824A 52 02            [ 2]  441 	sub	sp, #2
                                    442 ;	../src/LCD.c: 69: len = strlen(s);
      00824C 1E 05            [ 2]  443 	ldw	x, (0x05, sp)
      00824E 89               [ 2]  444 	pushw	x
      00824F CD 96 BC         [ 4]  445 	call	_strlen
      008252 5B 02            [ 2]  446 	addw	sp, #2
      008254 9F               [ 1]  447 	ld	a, xl
      008255 6B 01            [ 1]  448 	ld	(0x01, sp), a
                                    449 ;	../src/LCD.c: 70: for (i = 0; i < len; i++)
      008257 0F 02            [ 1]  450 	clr	(0x02, sp)
      008259                        451 00103$:
      008259 7B 02            [ 1]  452 	ld	a, (0x02, sp)
      00825B 11 01            [ 1]  453 	cp	a, (0x01, sp)
      00825D 24 11            [ 1]  454 	jrnc	00105$
                                    455 ;	../src/LCD.c: 72: c = s[i];
      00825F 5F               [ 1]  456 	clrw	x
      008260 7B 02            [ 1]  457 	ld	a, (0x02, sp)
      008262 97               [ 1]  458 	ld	xl, a
      008263 72 FB 05         [ 2]  459 	addw	x, (0x05, sp)
      008266 F6               [ 1]  460 	ld	a, (x)
                                    461 ;	../src/LCD.c: 73: LCD_putc(c);
      008267 88               [ 1]  462 	push	a
      008268 CD 81 FE         [ 4]  463 	call	_LCD_putc
      00826B 84               [ 1]  464 	pop	a
                                    465 ;	../src/LCD.c: 70: for (i = 0; i < len; i++)
      00826C 0C 02            [ 1]  466 	inc	(0x02, sp)
      00826E 20 E9            [ 2]  467 	jra	00103$
      008270                        468 00105$:
                                    469 ;	../src/LCD.c: 75: }
      008270 5B 02            [ 2]  470 	addw	sp, #2
      008272 81               [ 4]  471 	ret
                                    472 	.area CODE
      008273                        473 _font6x8:
      008273 00                     474 	.db #0x00	; 0
      008274 00                     475 	.db #0x00	; 0
      008275 00                     476 	.db #0x00	; 0
      008276 00                     477 	.db #0x00	; 0
      008277 00                     478 	.db #0x00	; 0
      008278 00                     479 	.db #0x00	; 0
      008279 00                     480 	.db #0x00	; 0
      00827A 00                     481 	.db #0x00	; 0
      00827B 00                     482 	.db #0x00	; 0
      00827C 2F                     483 	.db #0x2f	; 47
      00827D 00                     484 	.db #0x00	; 0
      00827E 00                     485 	.db #0x00	; 0
      00827F 00                     486 	.db #0x00	; 0
      008280 00                     487 	.db #0x00	; 0
      008281 02                     488 	.db #0x02	; 2
      008282 05                     489 	.db #0x05	; 5
      008283 02                     490 	.db #0x02	; 2
      008284 00                     491 	.db #0x00	; 0
      008285 00                     492 	.db #0x00	; 0
      008286 14                     493 	.db #0x14	; 20
      008287 7F                     494 	.db #0x7f	; 127
      008288 14                     495 	.db #0x14	; 20
      008289 7F                     496 	.db #0x7f	; 127
      00828A 14                     497 	.db #0x14	; 20
      00828B 00                     498 	.db #0x00	; 0
      00828C 24                     499 	.db #0x24	; 36
      00828D 2A                     500 	.db #0x2a	; 42
      00828E 7F                     501 	.db #0x7f	; 127
      00828F 2A                     502 	.db #0x2a	; 42
      008290 12                     503 	.db #0x12	; 18
      008291 00                     504 	.db #0x00	; 0
      008292 62                     505 	.db #0x62	; 98	'b'
      008293 64                     506 	.db #0x64	; 100	'd'
      008294 08                     507 	.db #0x08	; 8
      008295 13                     508 	.db #0x13	; 19
      008296 23                     509 	.db #0x23	; 35
      008297 00                     510 	.db #0x00	; 0
      008298 36                     511 	.db #0x36	; 54	'6'
      008299 49                     512 	.db #0x49	; 73	'I'
      00829A 55                     513 	.db #0x55	; 85	'U'
      00829B 22                     514 	.db #0x22	; 34
      00829C 50                     515 	.db #0x50	; 80	'P'
      00829D 00                     516 	.db #0x00	; 0
      00829E 00                     517 	.db #0x00	; 0
      00829F 05                     518 	.db #0x05	; 5
      0082A0 03                     519 	.db #0x03	; 3
      0082A1 00                     520 	.db #0x00	; 0
      0082A2 00                     521 	.db #0x00	; 0
      0082A3 00                     522 	.db #0x00	; 0
      0082A4 00                     523 	.db #0x00	; 0
      0082A5 1C                     524 	.db #0x1c	; 28
      0082A6 22                     525 	.db #0x22	; 34
      0082A7 41                     526 	.db #0x41	; 65	'A'
      0082A8 00                     527 	.db #0x00	; 0
      0082A9 00                     528 	.db #0x00	; 0
      0082AA 00                     529 	.db #0x00	; 0
      0082AB 41                     530 	.db #0x41	; 65	'A'
      0082AC 22                     531 	.db #0x22	; 34
      0082AD 1C                     532 	.db #0x1c	; 28
      0082AE 00                     533 	.db #0x00	; 0
      0082AF 00                     534 	.db #0x00	; 0
      0082B0 14                     535 	.db #0x14	; 20
      0082B1 08                     536 	.db #0x08	; 8
      0082B2 3E                     537 	.db #0x3e	; 62
      0082B3 08                     538 	.db #0x08	; 8
      0082B4 14                     539 	.db #0x14	; 20
      0082B5 00                     540 	.db #0x00	; 0
      0082B6 08                     541 	.db #0x08	; 8
      0082B7 08                     542 	.db #0x08	; 8
      0082B8 3E                     543 	.db #0x3e	; 62
      0082B9 08                     544 	.db #0x08	; 8
      0082BA 08                     545 	.db #0x08	; 8
      0082BB 00                     546 	.db #0x00	; 0
      0082BC 00                     547 	.db #0x00	; 0
      0082BD 00                     548 	.db #0x00	; 0
      0082BE A0                     549 	.db #0xa0	; 160
      0082BF 60                     550 	.db #0x60	; 96
      0082C0 00                     551 	.db #0x00	; 0
      0082C1 00                     552 	.db #0x00	; 0
      0082C2 08                     553 	.db #0x08	; 8
      0082C3 08                     554 	.db #0x08	; 8
      0082C4 08                     555 	.db #0x08	; 8
      0082C5 08                     556 	.db #0x08	; 8
      0082C6 08                     557 	.db #0x08	; 8
      0082C7 00                     558 	.db #0x00	; 0
      0082C8 00                     559 	.db #0x00	; 0
      0082C9 60                     560 	.db #0x60	; 96
      0082CA 60                     561 	.db #0x60	; 96
      0082CB 00                     562 	.db #0x00	; 0
      0082CC 00                     563 	.db #0x00	; 0
      0082CD 00                     564 	.db #0x00	; 0
      0082CE 20                     565 	.db #0x20	; 32
      0082CF 10                     566 	.db #0x10	; 16
      0082D0 08                     567 	.db #0x08	; 8
      0082D1 04                     568 	.db #0x04	; 4
      0082D2 02                     569 	.db #0x02	; 2
      0082D3 00                     570 	.db #0x00	; 0
      0082D4 3E                     571 	.db #0x3e	; 62
      0082D5 51                     572 	.db #0x51	; 81	'Q'
      0082D6 49                     573 	.db #0x49	; 73	'I'
      0082D7 45                     574 	.db #0x45	; 69	'E'
      0082D8 3E                     575 	.db #0x3e	; 62
      0082D9 00                     576 	.db #0x00	; 0
      0082DA 00                     577 	.db #0x00	; 0
      0082DB 42                     578 	.db #0x42	; 66	'B'
      0082DC 7F                     579 	.db #0x7f	; 127
      0082DD 40                     580 	.db #0x40	; 64
      0082DE 00                     581 	.db #0x00	; 0
      0082DF 00                     582 	.db #0x00	; 0
      0082E0 42                     583 	.db #0x42	; 66	'B'
      0082E1 61                     584 	.db #0x61	; 97	'a'
      0082E2 51                     585 	.db #0x51	; 81	'Q'
      0082E3 49                     586 	.db #0x49	; 73	'I'
      0082E4 46                     587 	.db #0x46	; 70	'F'
      0082E5 00                     588 	.db #0x00	; 0
      0082E6 21                     589 	.db #0x21	; 33
      0082E7 41                     590 	.db #0x41	; 65	'A'
      0082E8 45                     591 	.db #0x45	; 69	'E'
      0082E9 4B                     592 	.db #0x4b	; 75	'K'
      0082EA 31                     593 	.db #0x31	; 49	'1'
      0082EB 00                     594 	.db #0x00	; 0
      0082EC 18                     595 	.db #0x18	; 24
      0082ED 14                     596 	.db #0x14	; 20
      0082EE 12                     597 	.db #0x12	; 18
      0082EF 7F                     598 	.db #0x7f	; 127
      0082F0 10                     599 	.db #0x10	; 16
      0082F1 00                     600 	.db #0x00	; 0
      0082F2 27                     601 	.db #0x27	; 39
      0082F3 45                     602 	.db #0x45	; 69	'E'
      0082F4 45                     603 	.db #0x45	; 69	'E'
      0082F5 45                     604 	.db #0x45	; 69	'E'
      0082F6 39                     605 	.db #0x39	; 57	'9'
      0082F7 00                     606 	.db #0x00	; 0
      0082F8 3C                     607 	.db #0x3c	; 60
      0082F9 4A                     608 	.db #0x4a	; 74	'J'
      0082FA 49                     609 	.db #0x49	; 73	'I'
      0082FB 49                     610 	.db #0x49	; 73	'I'
      0082FC 30                     611 	.db #0x30	; 48	'0'
      0082FD 00                     612 	.db #0x00	; 0
      0082FE 01                     613 	.db #0x01	; 1
      0082FF 71                     614 	.db #0x71	; 113	'q'
      008300 09                     615 	.db #0x09	; 9
      008301 05                     616 	.db #0x05	; 5
      008302 03                     617 	.db #0x03	; 3
      008303 00                     618 	.db #0x00	; 0
      008304 36                     619 	.db #0x36	; 54	'6'
      008305 49                     620 	.db #0x49	; 73	'I'
      008306 49                     621 	.db #0x49	; 73	'I'
      008307 49                     622 	.db #0x49	; 73	'I'
      008308 36                     623 	.db #0x36	; 54	'6'
      008309 00                     624 	.db #0x00	; 0
      00830A 06                     625 	.db #0x06	; 6
      00830B 49                     626 	.db #0x49	; 73	'I'
      00830C 49                     627 	.db #0x49	; 73	'I'
      00830D 29                     628 	.db #0x29	; 41
      00830E 1E                     629 	.db #0x1e	; 30
      00830F 00                     630 	.db #0x00	; 0
      008310 00                     631 	.db #0x00	; 0
      008311 36                     632 	.db #0x36	; 54	'6'
      008312 36                     633 	.db #0x36	; 54	'6'
      008313 00                     634 	.db #0x00	; 0
      008314 00                     635 	.db #0x00	; 0
      008315 00                     636 	.db #0x00	; 0
      008316 00                     637 	.db #0x00	; 0
      008317 56                     638 	.db #0x56	; 86	'V'
      008318 36                     639 	.db #0x36	; 54	'6'
      008319 00                     640 	.db #0x00	; 0
      00831A 00                     641 	.db #0x00	; 0
      00831B 00                     642 	.db #0x00	; 0
      00831C 08                     643 	.db #0x08	; 8
      00831D 14                     644 	.db #0x14	; 20
      00831E 22                     645 	.db #0x22	; 34
      00831F 41                     646 	.db #0x41	; 65	'A'
      008320 00                     647 	.db #0x00	; 0
      008321 00                     648 	.db #0x00	; 0
      008322 14                     649 	.db #0x14	; 20
      008323 14                     650 	.db #0x14	; 20
      008324 14                     651 	.db #0x14	; 20
      008325 14                     652 	.db #0x14	; 20
      008326 14                     653 	.db #0x14	; 20
      008327 00                     654 	.db #0x00	; 0
      008328 00                     655 	.db #0x00	; 0
      008329 41                     656 	.db #0x41	; 65	'A'
      00832A 22                     657 	.db #0x22	; 34
      00832B 14                     658 	.db #0x14	; 20
      00832C 08                     659 	.db #0x08	; 8
      00832D 00                     660 	.db #0x00	; 0
      00832E 02                     661 	.db #0x02	; 2
      00832F 01                     662 	.db #0x01	; 1
      008330 51                     663 	.db #0x51	; 81	'Q'
      008331 09                     664 	.db #0x09	; 9
      008332 06                     665 	.db #0x06	; 6
      008333 00                     666 	.db #0x00	; 0
      008334 32                     667 	.db #0x32	; 50	'2'
      008335 49                     668 	.db #0x49	; 73	'I'
      008336 59                     669 	.db #0x59	; 89	'Y'
      008337 51                     670 	.db #0x51	; 81	'Q'
      008338 3E                     671 	.db #0x3e	; 62
      008339 00                     672 	.db #0x00	; 0
      00833A 7C                     673 	.db #0x7c	; 124
      00833B 12                     674 	.db #0x12	; 18
      00833C 11                     675 	.db #0x11	; 17
      00833D 12                     676 	.db #0x12	; 18
      00833E 7C                     677 	.db #0x7c	; 124
      00833F 00                     678 	.db #0x00	; 0
      008340 7F                     679 	.db #0x7f	; 127
      008341 49                     680 	.db #0x49	; 73	'I'
      008342 49                     681 	.db #0x49	; 73	'I'
      008343 49                     682 	.db #0x49	; 73	'I'
      008344 36                     683 	.db #0x36	; 54	'6'
      008345 00                     684 	.db #0x00	; 0
      008346 3E                     685 	.db #0x3e	; 62
      008347 41                     686 	.db #0x41	; 65	'A'
      008348 41                     687 	.db #0x41	; 65	'A'
      008349 41                     688 	.db #0x41	; 65	'A'
      00834A 22                     689 	.db #0x22	; 34
      00834B 00                     690 	.db #0x00	; 0
      00834C 7F                     691 	.db #0x7f	; 127
      00834D 41                     692 	.db #0x41	; 65	'A'
      00834E 41                     693 	.db #0x41	; 65	'A'
      00834F 22                     694 	.db #0x22	; 34
      008350 1C                     695 	.db #0x1c	; 28
      008351 00                     696 	.db #0x00	; 0
      008352 7F                     697 	.db #0x7f	; 127
      008353 49                     698 	.db #0x49	; 73	'I'
      008354 49                     699 	.db #0x49	; 73	'I'
      008355 49                     700 	.db #0x49	; 73	'I'
      008356 41                     701 	.db #0x41	; 65	'A'
      008357 00                     702 	.db #0x00	; 0
      008358 7F                     703 	.db #0x7f	; 127
      008359 09                     704 	.db #0x09	; 9
      00835A 09                     705 	.db #0x09	; 9
      00835B 09                     706 	.db #0x09	; 9
      00835C 01                     707 	.db #0x01	; 1
      00835D 00                     708 	.db #0x00	; 0
      00835E 3E                     709 	.db #0x3e	; 62
      00835F 41                     710 	.db #0x41	; 65	'A'
      008360 49                     711 	.db #0x49	; 73	'I'
      008361 49                     712 	.db #0x49	; 73	'I'
      008362 7A                     713 	.db #0x7a	; 122	'z'
      008363 00                     714 	.db #0x00	; 0
      008364 7F                     715 	.db #0x7f	; 127
      008365 08                     716 	.db #0x08	; 8
      008366 08                     717 	.db #0x08	; 8
      008367 08                     718 	.db #0x08	; 8
      008368 7F                     719 	.db #0x7f	; 127
      008369 00                     720 	.db #0x00	; 0
      00836A 00                     721 	.db #0x00	; 0
      00836B 41                     722 	.db #0x41	; 65	'A'
      00836C 7F                     723 	.db #0x7f	; 127
      00836D 41                     724 	.db #0x41	; 65	'A'
      00836E 00                     725 	.db #0x00	; 0
      00836F 00                     726 	.db #0x00	; 0
      008370 20                     727 	.db #0x20	; 32
      008371 40                     728 	.db #0x40	; 64
      008372 41                     729 	.db #0x41	; 65	'A'
      008373 3F                     730 	.db #0x3f	; 63
      008374 01                     731 	.db #0x01	; 1
      008375 00                     732 	.db #0x00	; 0
      008376 7F                     733 	.db #0x7f	; 127
      008377 08                     734 	.db #0x08	; 8
      008378 14                     735 	.db #0x14	; 20
      008379 22                     736 	.db #0x22	; 34
      00837A 41                     737 	.db #0x41	; 65	'A'
      00837B 00                     738 	.db #0x00	; 0
      00837C 7F                     739 	.db #0x7f	; 127
      00837D 40                     740 	.db #0x40	; 64
      00837E 40                     741 	.db #0x40	; 64
      00837F 40                     742 	.db #0x40	; 64
      008380 40                     743 	.db #0x40	; 64
      008381 00                     744 	.db #0x00	; 0
      008382 7F                     745 	.db #0x7f	; 127
      008383 02                     746 	.db #0x02	; 2
      008384 0C                     747 	.db #0x0c	; 12
      008385 02                     748 	.db #0x02	; 2
      008386 7F                     749 	.db #0x7f	; 127
      008387 00                     750 	.db #0x00	; 0
      008388 7F                     751 	.db #0x7f	; 127
      008389 04                     752 	.db #0x04	; 4
      00838A 08                     753 	.db #0x08	; 8
      00838B 10                     754 	.db #0x10	; 16
      00838C 7F                     755 	.db #0x7f	; 127
      00838D 00                     756 	.db #0x00	; 0
      00838E 3E                     757 	.db #0x3e	; 62
      00838F 41                     758 	.db #0x41	; 65	'A'
      008390 41                     759 	.db #0x41	; 65	'A'
      008391 41                     760 	.db #0x41	; 65	'A'
      008392 3E                     761 	.db #0x3e	; 62
      008393 00                     762 	.db #0x00	; 0
      008394 7F                     763 	.db #0x7f	; 127
      008395 09                     764 	.db #0x09	; 9
      008396 09                     765 	.db #0x09	; 9
      008397 09                     766 	.db #0x09	; 9
      008398 06                     767 	.db #0x06	; 6
      008399 00                     768 	.db #0x00	; 0
      00839A 3E                     769 	.db #0x3e	; 62
      00839B 41                     770 	.db #0x41	; 65	'A'
      00839C 51                     771 	.db #0x51	; 81	'Q'
      00839D 21                     772 	.db #0x21	; 33
      00839E 5E                     773 	.db #0x5e	; 94
      00839F 00                     774 	.db #0x00	; 0
      0083A0 7F                     775 	.db #0x7f	; 127
      0083A1 09                     776 	.db #0x09	; 9
      0083A2 19                     777 	.db #0x19	; 25
      0083A3 29                     778 	.db #0x29	; 41
      0083A4 46                     779 	.db #0x46	; 70	'F'
      0083A5 00                     780 	.db #0x00	; 0
      0083A6 46                     781 	.db #0x46	; 70	'F'
      0083A7 49                     782 	.db #0x49	; 73	'I'
      0083A8 49                     783 	.db #0x49	; 73	'I'
      0083A9 49                     784 	.db #0x49	; 73	'I'
      0083AA 31                     785 	.db #0x31	; 49	'1'
      0083AB 00                     786 	.db #0x00	; 0
      0083AC 01                     787 	.db #0x01	; 1
      0083AD 01                     788 	.db #0x01	; 1
      0083AE 7F                     789 	.db #0x7f	; 127
      0083AF 01                     790 	.db #0x01	; 1
      0083B0 01                     791 	.db #0x01	; 1
      0083B1 00                     792 	.db #0x00	; 0
      0083B2 3F                     793 	.db #0x3f	; 63
      0083B3 40                     794 	.db #0x40	; 64
      0083B4 40                     795 	.db #0x40	; 64
      0083B5 40                     796 	.db #0x40	; 64
      0083B6 3F                     797 	.db #0x3f	; 63
      0083B7 00                     798 	.db #0x00	; 0
      0083B8 1F                     799 	.db #0x1f	; 31
      0083B9 20                     800 	.db #0x20	; 32
      0083BA 40                     801 	.db #0x40	; 64
      0083BB 20                     802 	.db #0x20	; 32
      0083BC 1F                     803 	.db #0x1f	; 31
      0083BD 00                     804 	.db #0x00	; 0
      0083BE 3F                     805 	.db #0x3f	; 63
      0083BF 40                     806 	.db #0x40	; 64
      0083C0 38                     807 	.db #0x38	; 56	'8'
      0083C1 40                     808 	.db #0x40	; 64
      0083C2 3F                     809 	.db #0x3f	; 63
      0083C3 00                     810 	.db #0x00	; 0
      0083C4 63                     811 	.db #0x63	; 99	'c'
      0083C5 14                     812 	.db #0x14	; 20
      0083C6 08                     813 	.db #0x08	; 8
      0083C7 14                     814 	.db #0x14	; 20
      0083C8 63                     815 	.db #0x63	; 99	'c'
      0083C9 00                     816 	.db #0x00	; 0
      0083CA 07                     817 	.db #0x07	; 7
      0083CB 08                     818 	.db #0x08	; 8
      0083CC 70                     819 	.db #0x70	; 112	'p'
      0083CD 08                     820 	.db #0x08	; 8
      0083CE 07                     821 	.db #0x07	; 7
      0083CF 00                     822 	.db #0x00	; 0
      0083D0 61                     823 	.db #0x61	; 97	'a'
      0083D1 51                     824 	.db #0x51	; 81	'Q'
      0083D2 49                     825 	.db #0x49	; 73	'I'
      0083D3 45                     826 	.db #0x45	; 69	'E'
      0083D4 43                     827 	.db #0x43	; 67	'C'
      0083D5 00                     828 	.db #0x00	; 0
      0083D6 00                     829 	.db #0x00	; 0
      0083D7 7F                     830 	.db #0x7f	; 127
      0083D8 41                     831 	.db #0x41	; 65	'A'
      0083D9 41                     832 	.db #0x41	; 65	'A'
      0083DA 00                     833 	.db #0x00	; 0
      0083DB 00                     834 	.db #0x00	; 0
      0083DC 55                     835 	.db #0x55	; 85	'U'
      0083DD 2A                     836 	.db #0x2a	; 42
      0083DE 55                     837 	.db #0x55	; 85	'U'
      0083DF 2A                     838 	.db #0x2a	; 42
      0083E0 55                     839 	.db #0x55	; 85	'U'
      0083E1 00                     840 	.db #0x00	; 0
      0083E2 00                     841 	.db #0x00	; 0
      0083E3 41                     842 	.db #0x41	; 65	'A'
      0083E4 41                     843 	.db #0x41	; 65	'A'
      0083E5 7F                     844 	.db #0x7f	; 127
      0083E6 00                     845 	.db #0x00	; 0
      0083E7 00                     846 	.db #0x00	; 0
      0083E8 04                     847 	.db #0x04	; 4
      0083E9 02                     848 	.db #0x02	; 2
      0083EA 01                     849 	.db #0x01	; 1
      0083EB 02                     850 	.db #0x02	; 2
      0083EC 04                     851 	.db #0x04	; 4
      0083ED 00                     852 	.db #0x00	; 0
      0083EE 40                     853 	.db #0x40	; 64
      0083EF 40                     854 	.db #0x40	; 64
      0083F0 40                     855 	.db #0x40	; 64
      0083F1 40                     856 	.db #0x40	; 64
      0083F2 40                     857 	.db #0x40	; 64
      0083F3 00                     858 	.db #0x00	; 0
      0083F4 00                     859 	.db #0x00	; 0
      0083F5 01                     860 	.db #0x01	; 1
      0083F6 02                     861 	.db #0x02	; 2
      0083F7 04                     862 	.db #0x04	; 4
      0083F8 00                     863 	.db #0x00	; 0
      0083F9 00                     864 	.db #0x00	; 0
      0083FA 20                     865 	.db #0x20	; 32
      0083FB 54                     866 	.db #0x54	; 84	'T'
      0083FC 54                     867 	.db #0x54	; 84	'T'
      0083FD 54                     868 	.db #0x54	; 84	'T'
      0083FE 78                     869 	.db #0x78	; 120	'x'
      0083FF 00                     870 	.db #0x00	; 0
      008400 7F                     871 	.db #0x7f	; 127
      008401 48                     872 	.db #0x48	; 72	'H'
      008402 44                     873 	.db #0x44	; 68	'D'
      008403 44                     874 	.db #0x44	; 68	'D'
      008404 38                     875 	.db #0x38	; 56	'8'
      008405 00                     876 	.db #0x00	; 0
      008406 38                     877 	.db #0x38	; 56	'8'
      008407 44                     878 	.db #0x44	; 68	'D'
      008408 44                     879 	.db #0x44	; 68	'D'
      008409 44                     880 	.db #0x44	; 68	'D'
      00840A 20                     881 	.db #0x20	; 32
      00840B 00                     882 	.db #0x00	; 0
      00840C 38                     883 	.db #0x38	; 56	'8'
      00840D 44                     884 	.db #0x44	; 68	'D'
      00840E 44                     885 	.db #0x44	; 68	'D'
      00840F 48                     886 	.db #0x48	; 72	'H'
      008410 7F                     887 	.db #0x7f	; 127
      008411 00                     888 	.db #0x00	; 0
      008412 38                     889 	.db #0x38	; 56	'8'
      008413 54                     890 	.db #0x54	; 84	'T'
      008414 54                     891 	.db #0x54	; 84	'T'
      008415 54                     892 	.db #0x54	; 84	'T'
      008416 18                     893 	.db #0x18	; 24
      008417 00                     894 	.db #0x00	; 0
      008418 08                     895 	.db #0x08	; 8
      008419 7E                     896 	.db #0x7e	; 126
      00841A 09                     897 	.db #0x09	; 9
      00841B 01                     898 	.db #0x01	; 1
      00841C 02                     899 	.db #0x02	; 2
      00841D 00                     900 	.db #0x00	; 0
      00841E 18                     901 	.db #0x18	; 24
      00841F A4                     902 	.db #0xa4	; 164
      008420 A4                     903 	.db #0xa4	; 164
      008421 A4                     904 	.db #0xa4	; 164
      008422 7C                     905 	.db #0x7c	; 124
      008423 00                     906 	.db #0x00	; 0
      008424 7F                     907 	.db #0x7f	; 127
      008425 08                     908 	.db #0x08	; 8
      008426 04                     909 	.db #0x04	; 4
      008427 04                     910 	.db #0x04	; 4
      008428 78                     911 	.db #0x78	; 120	'x'
      008429 00                     912 	.db #0x00	; 0
      00842A 00                     913 	.db #0x00	; 0
      00842B 44                     914 	.db #0x44	; 68	'D'
      00842C 7D                     915 	.db #0x7d	; 125
      00842D 40                     916 	.db #0x40	; 64
      00842E 00                     917 	.db #0x00	; 0
      00842F 00                     918 	.db #0x00	; 0
      008430 40                     919 	.db #0x40	; 64
      008431 80                     920 	.db #0x80	; 128
      008432 84                     921 	.db #0x84	; 132
      008433 7D                     922 	.db #0x7d	; 125
      008434 00                     923 	.db #0x00	; 0
      008435 00                     924 	.db #0x00	; 0
      008436 7F                     925 	.db #0x7f	; 127
      008437 10                     926 	.db #0x10	; 16
      008438 28                     927 	.db #0x28	; 40
      008439 44                     928 	.db #0x44	; 68	'D'
      00843A 00                     929 	.db #0x00	; 0
      00843B 00                     930 	.db #0x00	; 0
      00843C 00                     931 	.db #0x00	; 0
      00843D 41                     932 	.db #0x41	; 65	'A'
      00843E 7F                     933 	.db #0x7f	; 127
      00843F 40                     934 	.db #0x40	; 64
      008440 00                     935 	.db #0x00	; 0
      008441 00                     936 	.db #0x00	; 0
      008442 7C                     937 	.db #0x7c	; 124
      008443 04                     938 	.db #0x04	; 4
      008444 18                     939 	.db #0x18	; 24
      008445 04                     940 	.db #0x04	; 4
      008446 78                     941 	.db #0x78	; 120	'x'
      008447 00                     942 	.db #0x00	; 0
      008448 7C                     943 	.db #0x7c	; 124
      008449 08                     944 	.db #0x08	; 8
      00844A 04                     945 	.db #0x04	; 4
      00844B 04                     946 	.db #0x04	; 4
      00844C 78                     947 	.db #0x78	; 120	'x'
      00844D 00                     948 	.db #0x00	; 0
      00844E 38                     949 	.db #0x38	; 56	'8'
      00844F 44                     950 	.db #0x44	; 68	'D'
      008450 44                     951 	.db #0x44	; 68	'D'
      008451 44                     952 	.db #0x44	; 68	'D'
      008452 38                     953 	.db #0x38	; 56	'8'
      008453 00                     954 	.db #0x00	; 0
      008454 FC                     955 	.db #0xfc	; 252
      008455 24                     956 	.db #0x24	; 36
      008456 24                     957 	.db #0x24	; 36
      008457 24                     958 	.db #0x24	; 36
      008458 18                     959 	.db #0x18	; 24
      008459 00                     960 	.db #0x00	; 0
      00845A 18                     961 	.db #0x18	; 24
      00845B 24                     962 	.db #0x24	; 36
      00845C 24                     963 	.db #0x24	; 36
      00845D 18                     964 	.db #0x18	; 24
      00845E FC                     965 	.db #0xfc	; 252
      00845F 00                     966 	.db #0x00	; 0
      008460 7C                     967 	.db #0x7c	; 124
      008461 08                     968 	.db #0x08	; 8
      008462 04                     969 	.db #0x04	; 4
      008463 04                     970 	.db #0x04	; 4
      008464 08                     971 	.db #0x08	; 8
      008465 00                     972 	.db #0x00	; 0
      008466 48                     973 	.db #0x48	; 72	'H'
      008467 54                     974 	.db #0x54	; 84	'T'
      008468 54                     975 	.db #0x54	; 84	'T'
      008469 54                     976 	.db #0x54	; 84	'T'
      00846A 20                     977 	.db #0x20	; 32
      00846B 00                     978 	.db #0x00	; 0
      00846C 04                     979 	.db #0x04	; 4
      00846D 3F                     980 	.db #0x3f	; 63
      00846E 44                     981 	.db #0x44	; 68	'D'
      00846F 40                     982 	.db #0x40	; 64
      008470 20                     983 	.db #0x20	; 32
      008471 00                     984 	.db #0x00	; 0
      008472 3C                     985 	.db #0x3c	; 60
      008473 40                     986 	.db #0x40	; 64
      008474 40                     987 	.db #0x40	; 64
      008475 20                     988 	.db #0x20	; 32
      008476 7C                     989 	.db #0x7c	; 124
      008477 00                     990 	.db #0x00	; 0
      008478 1C                     991 	.db #0x1c	; 28
      008479 20                     992 	.db #0x20	; 32
      00847A 40                     993 	.db #0x40	; 64
      00847B 20                     994 	.db #0x20	; 32
      00847C 1C                     995 	.db #0x1c	; 28
      00847D 00                     996 	.db #0x00	; 0
      00847E 3C                     997 	.db #0x3c	; 60
      00847F 40                     998 	.db #0x40	; 64
      008480 30                     999 	.db #0x30	; 48	'0'
      008481 40                    1000 	.db #0x40	; 64
      008482 3C                    1001 	.db #0x3c	; 60
      008483 00                    1002 	.db #0x00	; 0
      008484 44                    1003 	.db #0x44	; 68	'D'
      008485 28                    1004 	.db #0x28	; 40
      008486 10                    1005 	.db #0x10	; 16
      008487 28                    1006 	.db #0x28	; 40
      008488 44                    1007 	.db #0x44	; 68	'D'
      008489 00                    1008 	.db #0x00	; 0
      00848A 1C                    1009 	.db #0x1c	; 28
      00848B A0                    1010 	.db #0xa0	; 160
      00848C A0                    1011 	.db #0xa0	; 160
      00848D A0                    1012 	.db #0xa0	; 160
      00848E 7C                    1013 	.db #0x7c	; 124
      00848F 00                    1014 	.db #0x00	; 0
      008490 44                    1015 	.db #0x44	; 68	'D'
      008491 64                    1016 	.db #0x64	; 100	'd'
      008492 54                    1017 	.db #0x54	; 84	'T'
      008493 4C                    1018 	.db #0x4c	; 76	'L'
      008494 44                    1019 	.db #0x44	; 68	'D'
      008495 14                    1020 	.db #0x14	; 20
      008496 14                    1021 	.db #0x14	; 20
      008497 14                    1022 	.db #0x14	; 20
      008498 14                    1023 	.db #0x14	; 20
      008499 14                    1024 	.db #0x14	; 20
      00849A 14                    1025 	.db #0x14	; 20
                                   1026 	.area INITIALIZER
                                   1027 	.area CABS (ABS)
