This part of the project is the monitor that checks the humidity and
temperature in the tunnels where the steam pipes are.  The idea is to 
catch leaks before they go on too long.
 
An AM2320 is used with an STM8S103F3 MCU to get both the humidity and
the temperature.  The AM2320 talks using i2c, and spends most of its
time shutdown.

The monitor gets its device address by reading dip switches connected
to an MCP23017 i/o expander.  There aren't a lot of gpios on the STM8S
chip.  The MCP23017 is only used when the STM8S is booted.  For normal
operation it is ignored after startup.
 
This monitor only responds to requests from the master program running
on a Raspberry Pi and talking via RS485.  The requests are formatted
as escape followed by the character set by the dip switches (ascii).
Requests for other addresses are ignored.  This monitor only pushes 
out unsolicited messages at boot time.

Anyone can use this code as examples for their own projects, 'cause
much of it came from elsewhere anyway.

Eclipse is used as the IDE, and SDCC is the compiler.  stm8flash and
a ST LINK/V2 are used to program the chips.
 
Hardware-wise, this monitor consists of a STM8S103F3, a MCP23017, an
8-wide set of dip switches, an RS485 module, and an AM2320 temp/humidity
sensor.  Power is supplied at 5 volts.
