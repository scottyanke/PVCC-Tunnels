                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.7.0 #10231 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module interrupts
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _TLI_IRQHandler
                                     12 	.globl _AWU_IRQHandler
                                     13 	.globl _CLK_IRQHandler
                                     14 	.globl _EXTI_PORTA_IRQHandler
                                     15 	.globl _EXTI_PORTB_IRQHandler
                                     16 	.globl _EXTI_PORTC_IRQHandler
                                     17 	.globl _EXTI_PORTD_IRQHandler
                                     18 	.globl _EXTI_PORTE_IRQHandler
                                     19 	.globl _SPI_IRQHandler
                                     20 	.globl _TIM1_UPD_OVF_TRG_BRK_IRQHandler
                                     21 	.globl _TIM1_CAP_COM_IRQHandler
                                     22 	.globl _TIM2_UPD_OVF_BRK_IRQHandler
                                     23 	.globl _TIM2_CAP_COM_IRQHandler
                                     24 	.globl _UART1_TX_IRQHandler
                                     25 	.globl _UART1_RX_IRQHandler
                                     26 	.globl _I2C_IRQHandler
                                     27 	.globl _ADC1_IRQHandler
                                     28 	.globl _TIM4_UPD_OVF_IRQHandler
                                     29 	.globl _EEPROM_EEC_IRQHandler
                                     30 ;--------------------------------------------------------
                                     31 ; ram data
                                     32 ;--------------------------------------------------------
                                     33 	.area DATA
                                     34 ;--------------------------------------------------------
                                     35 ; ram data
                                     36 ;--------------------------------------------------------
                                     37 	.area INITIALIZED
                                     38 ;--------------------------------------------------------
                                     39 ; absolute external ram data
                                     40 ;--------------------------------------------------------
                                     41 	.area DABS (ABS)
                                     42 ;--------------------------------------------------------
                                     43 ; global & static initialisations
                                     44 ;--------------------------------------------------------
                                     45 	.area HOME
                                     46 	.area GSINIT
                                     47 	.area GSFINAL
                                     48 	.area GSINIT
                                     49 ;--------------------------------------------------------
                                     50 ; Home
                                     51 ;--------------------------------------------------------
                                     52 	.area HOME
                                     53 	.area HOME
                                     54 ;--------------------------------------------------------
                                     55 ; code
                                     56 ;--------------------------------------------------------
                                     57 	.area CODE
                                     58 ;	../src/interrupts.c: 27: INTERRUPT_HANDLER(TLI_IRQHandler, 0){}
                                     59 ;	-----------------------------------------
                                     60 ;	 function TLI_IRQHandler
                                     61 ;	-----------------------------------------
      008157                         62 _TLI_IRQHandler:
      008157 80               [11]   63 	iret
                                     64 ;	../src/interrupts.c: 30: INTERRUPT_HANDLER(AWU_IRQHandler, 1){}
                                     65 ;	-----------------------------------------
                                     66 ;	 function AWU_IRQHandler
                                     67 ;	-----------------------------------------
      008158                         68 _AWU_IRQHandler:
      008158 80               [11]   69 	iret
                                     70 ;	../src/interrupts.c: 33: INTERRUPT_HANDLER(CLK_IRQHandler, 2){}
                                     71 ;	-----------------------------------------
                                     72 ;	 function CLK_IRQHandler
                                     73 ;	-----------------------------------------
      008159                         74 _CLK_IRQHandler:
      008159 80               [11]   75 	iret
                                     76 ;	../src/interrupts.c: 36: INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3){}
                                     77 ;	-----------------------------------------
                                     78 ;	 function EXTI_PORTA_IRQHandler
                                     79 ;	-----------------------------------------
      00815A                         80 _EXTI_PORTA_IRQHandler:
      00815A 80               [11]   81 	iret
                                     82 ;	../src/interrupts.c: 39: INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4){}
                                     83 ;	-----------------------------------------
                                     84 ;	 function EXTI_PORTB_IRQHandler
                                     85 ;	-----------------------------------------
      00815B                         86 _EXTI_PORTB_IRQHandler:
      00815B 80               [11]   87 	iret
                                     88 ;	../src/interrupts.c: 42: INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5){
                                     89 ;	-----------------------------------------
                                     90 ;	 function EXTI_PORTC_IRQHandler
                                     91 ;	-----------------------------------------
      00815C                         92 _EXTI_PORTC_IRQHandler:
                                     93 ;	../src/interrupts.c: 43: }
      00815C 80               [11]   94 	iret
                                     95 ;	../src/interrupts.c: 46: INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6){
                                     96 ;	-----------------------------------------
                                     97 ;	 function EXTI_PORTD_IRQHandler
                                     98 ;	-----------------------------------------
      00815D                         99 _EXTI_PORTD_IRQHandler:
                                    100 ;	../src/interrupts.c: 47: }
      00815D 80               [11]  101 	iret
                                    102 ;	../src/interrupts.c: 50: INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7){}
                                    103 ;	-----------------------------------------
                                    104 ;	 function EXTI_PORTE_IRQHandler
                                    105 ;	-----------------------------------------
      00815E                        106 _EXTI_PORTE_IRQHandler:
      00815E 80               [11]  107 	iret
                                    108 ;	../src/interrupts.c: 66: INTERRUPT_HANDLER(SPI_IRQHandler, 10){}
                                    109 ;	-----------------------------------------
                                    110 ;	 function SPI_IRQHandler
                                    111 ;	-----------------------------------------
      00815F                        112 _SPI_IRQHandler:
      00815F 80               [11]  113 	iret
                                    114 ;	../src/interrupts.c: 69: INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11){
                                    115 ;	-----------------------------------------
                                    116 ;	 function TIM1_UPD_OVF_TRG_BRK_IRQHandler
                                    117 ;	-----------------------------------------
      008160                        118 _TIM1_UPD_OVF_TRG_BRK_IRQHandler:
                                    119 ;	../src/interrupts.c: 70: }
      008160 80               [11]  120 	iret
                                    121 ;	../src/interrupts.c: 73: INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12){}
                                    122 ;	-----------------------------------------
                                    123 ;	 function TIM1_CAP_COM_IRQHandler
                                    124 ;	-----------------------------------------
      008161                        125 _TIM1_CAP_COM_IRQHandler:
      008161 80               [11]  126 	iret
                                    127 ;	../src/interrupts.c: 85: INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13){
                                    128 ;	-----------------------------------------
                                    129 ;	 function TIM2_UPD_OVF_BRK_IRQHandler
                                    130 ;	-----------------------------------------
      008162                        131 _TIM2_UPD_OVF_BRK_IRQHandler:
                                    132 ;	../src/interrupts.c: 86: }
      008162 80               [11]  133 	iret
                                    134 ;	../src/interrupts.c: 90: INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14){
                                    135 ;	-----------------------------------------
                                    136 ;	 function TIM2_CAP_COM_IRQHandler
                                    137 ;	-----------------------------------------
      008163                        138 _TIM2_CAP_COM_IRQHandler:
                                    139 ;	../src/interrupts.c: 91: }
      008163 80               [11]  140 	iret
                                    141 ;	../src/interrupts.c: 106: INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17){}
                                    142 ;	-----------------------------------------
                                    143 ;	 function UART1_TX_IRQHandler
                                    144 ;	-----------------------------------------
      008164                        145 _UART1_TX_IRQHandler:
      008164 80               [11]  146 	iret
                                    147 ;	../src/interrupts.c: 109: INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18){
                                    148 ;	-----------------------------------------
                                    149 ;	 function UART1_RX_IRQHandler
                                    150 ;	-----------------------------------------
      008165                        151 _UART1_RX_IRQHandler:
      008165 52 02            [ 2]  152 	sub	sp, #2
                                    153 ;	../src/interrupts.c: 111: if(UART1->SR & UART1_SR_RXNE){ // data received
      008167 C6 52 30         [ 1]  154 	ld	a, 0x5230
      00816A A5 20            [ 1]  155 	bcp	a, #0x20
      00816C 27 3A            [ 1]  156 	jreq	00115$
                                    157 ;	../src/interrupts.c: 112: rb = UART1->DR; // read received byte & clear RXNE flag
      00816E C6 52 31         [ 1]  158 	ld	a, 0x5231
                                    159 ;	../src/interrupts.c: 114: UART_rx[UART_rx_cur_i++] = rb; // put received byte into cycled buffer
      008171 AE 00 2B         [ 2]  160 	ldw	x, #_UART_rx+0
      008174 1F 01            [ 2]  161 	ldw	(0x01, sp), x
      008176 41               [ 1]  162 	exg	a, xl
      008177 C6 00 4E         [ 1]  163 	ld	a, _UART_rx_cur_i+0
      00817A 41               [ 1]  164 	exg	a, xl
      00817B 72 5C 00 4E      [ 1]  165 	inc	_UART_rx_cur_i+0
      00817F 02               [ 1]  166 	rlwa	x
      008180 4F               [ 1]  167 	clr	a
      008181 01               [ 1]  168 	rrwa	x
      008182 72 FB 01         [ 2]  169 	addw	x, (0x01, sp)
      008185 F7               [ 1]  170 	ld	(x), a
                                    171 ;	../src/interrupts.c: 115: if(UART_rx_cur_i == UART_rx_start_i){ // Oops: buffer overflow! Just forget old data
      008186 C6 00 4D         [ 1]  172 	ld	a, _UART_rx_start_i+0
      008189 C1 00 4E         [ 1]  173 	cp	a, _UART_rx_cur_i+0
      00818C 26 0F            [ 1]  174 	jrne	00110$
                                    175 ;	../src/interrupts.c: 116: UART_rx_start_i++;
      00818E 72 5C 00 4D      [ 1]  176 	inc	_UART_rx_start_i+0
                                    177 ;	../src/interrupts.c: 117: check_UART_pointer(UART_rx_start_i);
      008192 C6 00 4D         [ 1]  178 	ld	a, _UART_rx_start_i+0
      008195 A1 1E            [ 1]  179 	cp	a, #0x1e
      008197 26 04            [ 1]  180 	jrne	00110$
      008199 72 5F 00 4D      [ 1]  181 	clr	_UART_rx_start_i+0
                                    182 ;	../src/interrupts.c: 119: check_UART_pointer(UART_rx_cur_i);
      00819D                        183 00110$:
      00819D C6 00 4E         [ 1]  184 	ld	a, _UART_rx_cur_i+0
      0081A0 A1 1E            [ 1]  185 	cp	a, #0x1e
      0081A2 26 04            [ 1]  186 	jrne	00115$
      0081A4 72 5F 00 4E      [ 1]  187 	clr	_UART_rx_cur_i+0
      0081A8                        188 00115$:
                                    189 ;	../src/interrupts.c: 121: }
      0081A8 5B 02            [ 2]  190 	addw	sp, #2
      0081AA 80               [11]  191 	iret
                                    192 ;	../src/interrupts.c: 125: INTERRUPT_HANDLER(I2C_IRQHandler, 19){}
                                    193 ;	-----------------------------------------
                                    194 ;	 function I2C_IRQHandler
                                    195 ;	-----------------------------------------
      0081AB                        196 _I2C_IRQHandler:
      0081AB 80               [11]  197 	iret
                                    198 ;	../src/interrupts.c: 149: INTERRUPT_HANDLER(ADC1_IRQHandler, 22){
                                    199 ;	-----------------------------------------
                                    200 ;	 function ADC1_IRQHandler
                                    201 ;	-----------------------------------------
      0081AC                        202 _ADC1_IRQHandler:
                                    203 ;	../src/interrupts.c: 151: }
      0081AC 80               [11]  204 	iret
                                    205 ;	../src/interrupts.c: 159: void TIM4_UPD_OVF_IRQHandler() __interrupt(23){
                                    206 ;	-----------------------------------------
                                    207 ;	 function TIM4_UPD_OVF_IRQHandler
                                    208 ;	-----------------------------------------
      0081AD                        209 _TIM4_UPD_OVF_IRQHandler:
                                    210 ;	../src/interrupts.c: 160: if(TIM4->SR1 & TIM4_SR1_UIF){ // update interrupt
      0081AD C6 53 44         [ 1]  211 	ld	a, 0x5344
      0081B0 44               [ 1]  212 	srl	a
      0081B1 24 1B            [ 1]  213 	jrnc	00102$
                                    214 ;	../src/interrupts.c: 161: Global_time++; // increase timer 1 tick every ms
      0081B3 CE 00 03         [ 2]  215 	ldw	x, _Global_time+2
      0081B6 1C 00 01         [ 2]  216 	addw	x, #0x0001
      0081B9 C6 00 02         [ 1]  217 	ld	a, _Global_time+1
      0081BC A9 00            [ 1]  218 	adc	a, #0x00
      0081BE 90 97            [ 1]  219 	ld	yl, a
      0081C0 C6 00 01         [ 1]  220 	ld	a, _Global_time+0
      0081C3 A9 00            [ 1]  221 	adc	a, #0x00
      0081C5 90 95            [ 1]  222 	ld	yh, a
      0081C7 CF 00 03         [ 2]  223 	ldw	_Global_time+2, x
      0081CA 90 CF 00 01      [ 2]  224 	ldw	_Global_time+0, y
      0081CE                        225 00102$:
                                    226 ;	../src/interrupts.c: 163: TIM4->SR1 = 0; // clear all interrupt flags
      0081CE 35 00 53 44      [ 1]  227 	mov	0x5344+0, #0x00
                                    228 ;	../src/interrupts.c: 164: }
      0081D2 80               [11]  229 	iret
                                    230 ;	../src/interrupts.c: 168: INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24){}
                                    231 ;	-----------------------------------------
                                    232 ;	 function EEPROM_EEC_IRQHandler
                                    233 ;	-----------------------------------------
      0081D3                        234 _EEPROM_EEC_IRQHandler:
      0081D3 80               [11]  235 	iret
                                    236 	.area CODE
                                    237 	.area INITIALIZER
                                    238 	.area CABS (ABS)
