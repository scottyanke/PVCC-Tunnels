// This is the monitor for the tunnels, checking temperature and humidity
// using AM2320 sensors.  Each monitor has an id that is set using dip
// switches to select the character it responds to after an escape character.
// The monitor responds to requests when it receives and escape character
// followed by the id character.  It does not send at other times, except
// when it is booted.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <Hstm8/stm8s.h>
#include <delay.h>
#include <uart.h>
#include <i2c.h>
#include "interrupts.h"
#include "mcp23017.h"
#include "build_defs.h"

/**
 * STM8S103F3
 *
 * PB5 -> SDA
 * PB4 -> SCL
 * PD5 (tx) -> RS485 DI
 * PD6 (rx) -> RS485 RO
 * PD4 -> RS485 DE/RE
 */
#define AM2320_ADDR        (0x5cU << 1)
#define rs485xmit_on() GPIOD->ODR |= GPIO_PIN_4
#define rs485xmit_off() GPIOD->ODR &= ~(GPIO_PIN_4)
#define reset_watchdog() IWDG->KR = 0xaa

volatile unsigned long Global_time; // global time in ms
unsigned long last_measure = 0L;
uint32_t light_time;
uint8_t buf[20];
uint8_t esc, address;
uint8_t version[12];

int putchar(int c) {
    uart_write(c);
    return c;
}

//--------------------------------------------------------------------------------
//
//  Setup the system clock to run at 16MHz using the internal oscillator.
//
void InitialiseSystemClock()
{
    CLK->ICKR = 0;                       //  Reset the Internal Clock Register.
    CLK->ICKR = CLK_ICKR_HSIEN;          //  Enable the HSI.
    CLK->ECKR = 0;                       //  Disable the external clock.
    while (!(CLK->ICKR & CLK_ICKR_HSIRDY)); //  Wait for the HSI to be ready for use.
    CLK->CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
    CLK->PCKENR1 = 0xff; //CLK_PCKENR1_TIM4 | CLK_PCKENR1_UART1 | CLK_PCKENR1_SPI | CLK_PCKENR1_I2C ;  //  Enable select peripheral clocks.
    CLK->PCKENR2 = 0xff; //CLK_PCKENR2_AWU;      //  Only enable the AWU watchdog service
    CLK->CCOR = 0;                       //  Turn off CCO.
    CLK->HSITRIMR = 0;                   //  Turn off any HSIU trimming.
    CLK->SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
    CLK->SWR = 0xe1;                     //  Use HSI as the clock source.
    CLK->SWCR = 0;                       //  Reset the clock switch control register.
    CLK->SWCR |= CLK_SWCR_SWEN;          //  Enable switching.
    while (CLK->SWCR & CLK_SWCR_SWBSY);  //  Pause while the clock switch is busy.
}
//--------------------------------------------------------------------------------
//
//  Initialize the Independent Watchdog (IWDG)
//
void InitialiseIWDG()
{
    IWDG->KR = 0xcc;         //  Start the independent watchdog.
    IWDG->KR = 0x55;         //  Allow the IWDG registers to be programmed.
    IWDG->PR = 0x06;         //  Prescaler is 6 => each count is 1.02 second with RLR = 0xff
    IWDG->RLR = 0xff;        //  Reload counter.  T = 2 x TLSI x PR x R LR
    IWDG->KR = 0xaa;         //  Reset the counter.
}

void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, GPIO_Mode_TypeDef GPIO_Mode)
{
  /* Reset corresponding bit to GPIO_Pin in CR2 register */
  GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
  /*-----------------------------*/
  /* Input/Output mode selection */
  /*-----------------------------*/
  if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x80) != (uint8_t)0x00) /* Output mode */
  {
    if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x10) != (uint8_t)0x00) /* High level */
      GPIOx->ODR |= (uint8_t)GPIO_Pin;
    else /* Low level */
      GPIOx->ODR &= (uint8_t)(~(GPIO_Pin));
    /* Set Output mode */
    GPIOx->DDR |= (uint8_t)GPIO_Pin;
  }
  else /* Input mode */
    GPIOx->DDR &= (uint8_t)(~(GPIO_Pin));
  /*------------------------------------------------------------------------*/
  /* Pull-Up/Float (Input) or Push-Pull/Open-Drain (Output) modes selection */
  /*------------------------------------------------------------------------*/
  if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x40) != (uint8_t)0x00) /* Pull-Up or Push-Pull */
    GPIOx->CR1 |= (uint8_t)GPIO_Pin;
  else /* Float or Open-Drain */
    GPIOx->CR1 &= (uint8_t)(~(GPIO_Pin));
  /*-----------------------------------------------------*/
  /* Interrupt (Input) or Slope (Output) modes selection */
  /*-----------------------------------------------------*/
  if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x20) != (uint8_t)0x00) /* Interrupt or Slow slope */
    GPIOx->CR2 |= (uint8_t)GPIO_Pin;
  else /* No external interrupt or No slope control */
    GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
}

void measure(uint8_t tell)	// the measure() function talks to the AM2320 via i2c
{
    uint16_t humidity, temp;

    // starting here is where we get the info from the AM2320
    if (i2c_start())	// have to check to see if the start was successful
    {
		i2c_write_addr(AM2320_ADDR | I2C_WRITE);  // this is just to wake the AM2320 up
		i2c_stop();

		delay_ms(15);	// the AM2320 needs this time to initialize itselt

		i2c_start();								// now we ask for a reading
		i2c_write_addr(AM2320_ADDR | I2C_WRITE);
		i2c_write(0x03);	// the the AM2320 we want 4 bytes from address 0
		i2c_write(0x00);
		i2c_write(0x04);
		i2c_stop();
		delay_ms(2);

		/* Start reading at DATA_OUT */
		i2c_start();
		i2c_write_addr(AM2320_ADDR | I2C_READ);
		i2c_read_arr(buf, 6);	// the first two bytes are useless
		humidity = (buf[2] << 8) + buf[3];	// get the 16-bit humidity
		temp = (buf[4] << 8) + buf[5];		// and the 16-bit temperature
		temp = temp * 1.8 + 320;	// convert temperature to fahrenheit
		if (buf[4] & 0x80)  // is it negative?
			temp *= -1;
		if (tell)	// if tell is set, transmit via rs485
		{
			rs485xmit_on();	// turn the RS485 chips transmitter on
			delay_ms(30);	// wait for everything to be ready
			printf("%c:%2d\.%1d%%:%3d\.%1d\r\n",address,humidity / 10,humidity % 10, temp / 10, temp %10);
			rs485xmit_off(); // turn the transmitter back off
		}
		reset_watchdog();  // reset the watchdog timer

    }
	reset_watchdog();  // reset the watchdog timer
	last_measure = Global_time;	// when was the last time we transmitted anything
}

void main() {
	unsigned long esc_time = 0L;

	uint8_t rb;		// rb is received byte

	esc = 0;

	sprintf(version,"%02d%02d%02d-%02d%02d", BUILD_YEAR - 2000, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN);

	disableInterrupts();
	InitialiseSystemClock();
	InitialiseIWDG();		// not really necessary, but what the heck...
    GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);  // rs485 xmit indicator

    rs485xmit_off();	// turn off the D4 pin so we can receive rs485 data
    /*
	// Configure clocking
	CLK->CKDIVR = 0x18; // F_HSI = 16MHz, f_CPU = 16MHz
	*/
	CFG->GCR |= 1; // disable SWIM

    // Timer 4 (8 bit) used as system tick timer
	TIM4->PSCR = 7;   // prescaler
	TIM4->ARR = 125;  // auto reload register
	// interrupts: update
	TIM4->IER = TIM4_IER_UIE;
	// auto-reload + interrupt on overflow + enable
	TIM4->CR1 = TIM4_CR1_ARPE | TIM4_CR1_URS | TIM4_CR1_CEN;

	reset_watchdog();  // reset the watchdog timer

	Global_time = 0L;	// used as an internal clock
    uart_init();		// setup for 9600 8-N-1
    i2c_init();			// talk to the AM2320 and the MCP23017 to read switches
    reset_watchdog();
    // enable all interrupts
    enableInterrupts();
	reset_watchdog();  // reset the watchdog timer

    delay_ms(250);
	rs485xmit_on();	// turn the RS485 chips transmitter on
	delay_ms(30);
	printf("Initializing...%s\r\n",version);
	rs485xmit_off(); // turn the transmitter back off

	// get this devices address by using i2c to talk to the MCP23017.
	// dip switches are connected to the MCP23017 that set the address
	// to any 8-bit character we want.
    address = init_mcp23017();

	reset_watchdog();  // reset the watchdog timer

	delay_ms(50 * (address - 0x30));	//  do this to stagger startup announcements
	reset_watchdog();  // reset the watchdog timer
	rs485xmit_on();	// turn the RS485 chips transmitter on
	delay_ms(30);
	printf("%c:Running:%s:%02x\r\n",address,version,address);
	rs485xmit_off(); // turn the transmitter back off

    // Loop
    do{
    	reset_watchdog();  // reset the watchdog timer
	    if(UART_read_byte(&rb)){ // buffer isn't empty
		    switch(rb){
			    case 0x1b: // escape
				    esc = 1;	// set the flag to show an escpage character was received
				    esc_time = Global_time;	// only wait 2 seconds for the next character after the escape
				    break;
			    default:
				    if (rb == address && esc)  // address must match the switches read by mcp23017
				    {
				    	Global_time = 0L;   // when was the last time we were called?
				    	measure(1);			// do a measurement, and send the results
				    	last_measure = Global_time;
				    }
				    esc = 0;	// reset the escape character flag
		    }
	    }
	    if (esc && (Global_time - esc_time > 2000))  // give it 2 seconds
	    	esc = 0;  // reset the esc, since it should have been followed by the id right away
	    if (Global_time - last_measure > 600000L)  // every 10 minutes take a silent measurement
	    {			// probably don't have to, but it helps for debugging
	    	measure(0);
	    	last_measure = Global_time;
	    }
    }while(1);
}
