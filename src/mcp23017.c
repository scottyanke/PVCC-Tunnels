// talk to the MCP23017 chip using i2c.  This serves the function
// of reading the dip switches to get the id of this monitoring
// device.  The input lines are weakly pulled high, and brought
// low by the switches being turned on.  Only gpioB is used.
// From a wiring perspective, it was easier than gpioA.
// The default i2c address is used.  Don't forget to tie the
// reset line high, or it just won't work.
#include <Hstm8/stm8s.h>
#include <i2c.h>
#include "mcp23017.h"

uint8_t init_mcp23017() {
  uint8_t rb;
  i2c_clear();
  if (i2c_start())
  {
	  i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	  i2c_write(0x00);  // gpioA as output
	  i2c_write(0x00);
	  i2c_stop();
	  i2c_start();
	  i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	  i2c_write(0x01);  // gpioB as input
	  i2c_write(0xff);
	  i2c_stop();
	  i2c_start();
	  i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	  i2c_write(0x0d);  // gpioB has internal pull-ups
	  i2c_write(0xff);
	  i2c_stop();
	  i2c_start();
	  i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	  i2c_write(0x03);  // gpioB has reversed polarity
	  i2c_write(0xff);
	  i2c_stop();
	  i2c_start();
	  i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	  i2c_write(0x13);  // looking at getting from GPIOB
	  i2c_stop();
	  i2c_start();
	  i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	  i2c_stop();
	  i2c_start();
	  i2c_write_addr(MCP23017_ADDR + I2C_READ);
	  rb = i2c_read();
	  return rb;
  }
  else
	  return 0xff;

}
void backlight_on()		// was part of the original design that had an LCD
{
	i2c_start();
	i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
	i2c_write(0x12);  // gpioA register
	i2c_write(0x80);   // set the backlight pin on
	i2c_stop();
}
void backlight_off()
{
    i2c_start();
    i2c_write_addr(MCP23017_ADDR + I2C_WRITE);
    i2c_write(0x12);  // gpioA register
    i2c_write(0x00);   // set the backlight pin off
    i2c_stop();
}

