                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.7.0 #10231 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module uart
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _UART_rx_cur_i
                                     12 	.globl _UART_rx_start_i
                                     13 	.globl _UART_rx
                                     14 	.globl _uart_init
                                     15 	.globl _uart_write
                                     16 	.globl _uart_read
                                     17 	.globl _UART_read_byte
                                     18 ;--------------------------------------------------------
                                     19 ; ram data
                                     20 ;--------------------------------------------------------
                                     21 	.area DATA
      00002B                         22 _UART_rx::
      00002B                         23 	.ds 30
                                     24 ;--------------------------------------------------------
                                     25 ; ram data
                                     26 ;--------------------------------------------------------
                                     27 	.area INITIALIZED
      00004D                         28 _UART_rx_start_i::
      00004D                         29 	.ds 1
      00004E                         30 _UART_rx_cur_i::
      00004E                         31 	.ds 1
                                     32 ;--------------------------------------------------------
                                     33 ; absolute external ram data
                                     34 ;--------------------------------------------------------
                                     35 	.area DABS (ABS)
                                     36 ;--------------------------------------------------------
                                     37 ; global & static initialisations
                                     38 ;--------------------------------------------------------
                                     39 	.area HOME
                                     40 	.area GSINIT
                                     41 	.area GSFINAL
                                     42 	.area GSINIT
                                     43 ;--------------------------------------------------------
                                     44 ; Home
                                     45 ;--------------------------------------------------------
                                     46 	.area HOME
                                     47 	.area HOME
                                     48 ;--------------------------------------------------------
                                     49 ; code
                                     50 ;--------------------------------------------------------
                                     51 	.area CODE
                                     52 ;	../src/uart.c: 8: void uart_init()
                                     53 ;	-----------------------------------------
                                     54 ;	 function uart_init
                                     55 ;	-----------------------------------------
      008A48                         56 _uart_init:
                                     57 ;	../src/uart.c: 11: tmp = UART1->SR;	// reading these two registers seems to be required
      008A48 AE 52 30         [ 2]   58 	ldw	x, #0x5230
      008A4B F6               [ 1]   59 	ld	a, (x)
                                     60 ;	../src/uart.c: 12: tmp = UART1->DR;
      008A4C AE 52 31         [ 2]   61 	ldw	x, #0x5231
      008A4F F6               [ 1]   62 	ld	a, (x)
                                     63 ;	../src/uart.c: 13: UART1->CR1 = 0x00;
      008A50 35 00 52 34      [ 1]   64 	mov	0x5234+0, #0x00
                                     65 ;	../src/uart.c: 15: UART1->BRR2 = 0x03; //((div >> 8) & 0xF0) + (div & 0x0F);
      008A54 35 03 52 33      [ 1]   66 	mov	0x5233+0, #0x03
                                     67 ;	../src/uart.c: 16: UART1->BRR1 = 0x6a; //div >> 4;
      008A58 35 6A 52 32      [ 1]   68 	mov	0x5232+0, #0x6a
                                     69 ;	../src/uart.c: 17: UART1->CR2 = UART1_CR2_TEN | UART1_CR2_REN | UART1_CR2_RIEN; // Allow RX/TX, generate ints on rx
      008A5C 35 2C 52 35      [ 1]   70 	mov	0x5235+0, #0x2c
                                     71 ;	../src/uart.c: 18: }
      008A60 81               [ 4]   72 	ret
                                     73 ;	../src/uart.c: 20: void uart_write(uint8_t data)
                                     74 ;	-----------------------------------------
                                     75 ;	 function uart_write
                                     76 ;	-----------------------------------------
      008A61                         77 _uart_write:
                                     78 ;	../src/uart.c: 22: UART1->DR = data;
      008A61 AE 52 31         [ 2]   79 	ldw	x, #0x5231
      008A64 7B 03            [ 1]   80 	ld	a, (0x03, sp)
      008A66 F7               [ 1]   81 	ld	(x), a
                                     82 ;	../src/uart.c: 23: while (!(UART1->SR & UART1_SR_TC)) ;
      008A67                         83 00101$:
      008A67 C6 52 30         [ 1]   84 	ld	a, 0x5230
      008A6A A5 40            [ 1]   85 	bcp	a, #0x40
      008A6C 27 F9            [ 1]   86 	jreq	00101$
                                     87 ;	../src/uart.c: 24: UART1->SR &= ~(UART1_SR_TC);
      008A6E A4 BF            [ 1]   88 	and	a, #0xbf
      008A70 C7 52 30         [ 1]   89 	ld	0x5230, a
                                     90 ;	../src/uart.c: 25: }
      008A73 81               [ 4]   91 	ret
                                     92 ;	../src/uart.c: 27: uint8_t uart_read()	// the uart rx interrupt is what's really used
                                     93 ;	-----------------------------------------
                                     94 ;	 function uart_read
                                     95 ;	-----------------------------------------
      008A74                         96 _uart_read:
                                     97 ;	../src/uart.c: 29: while (!(UART1->SR & UART1_SR_RXNE)) ;
      008A74                         98 00101$:
      008A74 C6 52 30         [ 1]   99 	ld	a, 0x5230
      008A77 A5 20            [ 1]  100 	bcp	a, #0x20
      008A79 27 F9            [ 1]  101 	jreq	00101$
                                    102 ;	../src/uart.c: 30: return UART1->DR;
      008A7B C6 52 31         [ 1]  103 	ld	a, 0x5231
                                    104 ;	../src/uart.c: 31: }
      008A7E 81               [ 4]  105 	ret
                                    106 ;	../src/uart.c: 38: uint8_t UART_read_byte(uint8_t *byte)
                                    107 ;	-----------------------------------------
                                    108 ;	 function UART_read_byte
                                    109 ;	-----------------------------------------
      008A7F                        110 _UART_read_byte:
      008A7F 52 02            [ 2]  111 	sub	sp, #2
                                    112 ;	../src/uart.c: 40: if(UART_rx_start_i == UART_rx_cur_i) // buffer is empty
      008A81 C6 00 4E         [ 1]  113 	ld	a, _UART_rx_cur_i+0
      008A84 C1 00 4D         [ 1]  114 	cp	a, _UART_rx_start_i+0
      008A87 26 03            [ 1]  115 	jrne	00102$
                                    116 ;	../src/uart.c: 41: return 0;
      008A89 4F               [ 1]  117 	clr	a
      008A8A 20 23            [ 2]  118 	jra	00108$
      008A8C                        119 00102$:
                                    120 ;	../src/uart.c: 42: *byte = UART_rx[UART_rx_start_i++];
      008A8C 16 05            [ 2]  121 	ldw	y, (0x05, sp)
      008A8E AE 00 2B         [ 2]  122 	ldw	x, #_UART_rx+0
      008A91 1F 01            [ 2]  123 	ldw	(0x01, sp), x
      008A93 C6 00 4D         [ 1]  124 	ld	a, _UART_rx_start_i+0
      008A96 72 5C 00 4D      [ 1]  125 	inc	_UART_rx_start_i+0
      008A9A 5F               [ 1]  126 	clrw	x
      008A9B 97               [ 1]  127 	ld	xl, a
      008A9C 72 FB 01         [ 2]  128 	addw	x, (0x01, sp)
      008A9F F6               [ 1]  129 	ld	a, (x)
      008AA0 90 F7            [ 1]  130 	ld	(y), a
                                    131 ;	../src/uart.c: 43: check_UART_pointer(UART_rx_start_i);
      008AA2 C6 00 4D         [ 1]  132 	ld	a, _UART_rx_start_i+0
      008AA5 A1 1E            [ 1]  133 	cp	a, #0x1e
      008AA7 26 04            [ 1]  134 	jrne	00106$
      008AA9 72 5F 00 4D      [ 1]  135 	clr	_UART_rx_start_i+0
      008AAD                        136 00106$:
                                    137 ;	../src/uart.c: 44: return 1;
      008AAD A6 01            [ 1]  138 	ld	a, #0x01
      008AAF                        139 00108$:
                                    140 ;	../src/uart.c: 45: }
      008AAF 5B 02            [ 2]  141 	addw	sp, #2
      008AB1 81               [ 4]  142 	ret
                                    143 	.area CODE
                                    144 	.area INITIALIZER
      0099F1                        145 __xinit__UART_rx_start_i:
      0099F1 00                     146 	.db #0x00	; 0
      0099F2                        147 __xinit__UART_rx_cur_i:
      0099F2 00                     148 	.db #0x00	; 0
                                    149 	.area CABS (ABS)
