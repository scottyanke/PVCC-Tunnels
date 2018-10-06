// i2c functions for the STM8S
//
#include "i2c.h"
#include "Hstm8/stm8s.h"

void i2c_init() {
    I2C->FREQR = 0x16;
    I2C->CCRL = 0xa0; // 100kHz
    I2C->CCRH &= ~(0x80); // standard mode
    I2C->OARH = 0x40;  // I2C_OARH_ADDMODE; // 7-bit addressing
    I2C->CR1 = I2C_CR1_PE;
}

void i2c_clear() {
	(void) I2C->SR1;
	I2C->DR = 0x00;
}

uint8_t i2c_start() {
	uint16_t i;
    I2C->CR2 |= I2C_CR2_START;
    i = 0;	// this is done to set a limit on how long to wait
    while (!(I2C->SR1 & I2C_SR1_SB) && i < 800)
		i++;
    if (i > 799)
    	return 0;
    else
    	return 1;
}

uint8_t i2c_stop() {
	uint16_t i;
    I2C->CR2 |= I2C_CR2_STOP;
    i = 0;
    while ((I2C->SR3 & I2C_SR3_MSL) && i < 800)
    	i++;
    if (i > 799)
    	return 0;
    else
    	return 1;
}

void i2c_write(uint8_t data) {
	uint16_t i;
    I2C->DR = data;
    i = 0;
    while (!(I2C->SR1 & I2C_SR1_TXE) && i < 800)
		i++;
}

void i2c_write_addr(uint8_t addr) {
	uint16_t i;
    I2C->DR = addr;
    i = 0;
    while (!(I2C->SR1 & I2C_SR1_ADDR) && i < 800)
		i++;;
    (void) I2C->SR3; // check BUS_BUSY
    I2C->CR2 |= (I2C_CR2_ACK);
}

uint8_t i2c_read() {
    I2C->CR2 &= ~(I2C_CR2_ACK);
    i2c_stop();
    while (!(I2C->SR1 & I2C_SR1_RXNE)) ;
    return I2C->DR;
}

void i2c_read_arr(uint8_t *buf, uint8_t len) {
    while (len-- > 1) {
        I2C->CR2 |= (1 << I2C_CR2_ACK);
        while (!(I2C->SR1 & I2C_SR1_RXNE)) ;
        *(buf++) = I2C->DR;
    }
    *buf = i2c_read();
}
