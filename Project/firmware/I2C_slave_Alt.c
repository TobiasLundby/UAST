#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#include "I2C_slave.h"

#define TWI_clear_interrupt() TWCR |= (1 << TWINT);

#define TWI_enable_ack() TWCR |= (1 << TWEA);
#define TWI_disable_ack() TWCR &= ~(1 << TWEA);

#define TWI_enable() TWCR |= (1 << TWEN);
#define TWI_disable() TWCR &= ~(1 << TWEN);

#define TWI_enable_interrupt() TWCR |= (1 << TWIE);
#define TWI_disable_interrupt() TWCR &= ~(1 << TWIE);

uint8_t TWI_input_buffer_size = 0xff;
uint8_t TWI_output_buffer_size = 0xff;
uint8_t TWI_buffer_address = 0xff;

inline void TWI_set_slave_address(uint8_t address){
	TWAR = (address << 1);
}

inline uint8_t TWI_slave_address_received(){
	return (TWSR & 0xf8) == TW_SR_SLA_ACK;
}

inline uint8_t TWI_slave_rx_cmd_received(){
	return (TWSR & 0xF8) == TW_SR_DATA_ACK;
}

inline uint8_t TWI_slave_tx_cmd_received(){
	return (TWSR & 0xF8) == TW_ST_DATA_ACK;
}

void I2C_init(uint8_t address, uint8_t input_buffer_size, uint8_t output_buffer_size){
	TWI_set_slave_address(address);
	TWI_input_buffer_size = input_buffer_size;
	TWI_output_buffer_size = output_buffer_size;
	TWI_clear_interrupt();
	TWI_enable_ack();
	TWI_enable();
	TWI_enable_interrupt();
}

void I2C_stop(void){
	TWI_disable();
	TWI_disable_ack();
}

ISR(TWI_vect){
	if (TWI_slave_address_received()){
		TWI_buffer_address = TWI_input_buffer_size;
		TWI_enable_interrupt();
		TWI_clear_interrupt();
		TWI_enable_ack();
		TWI_enable();
	}
	else if (TWI_slave_rx_cmd_received()){
		TWI_input_reg[0x10] = 'b';
		uint8_t data = TWDR;
		if (TWI_buffer_address == TWI_input_buffer_size){
			TWI_buffer_address = data;
			TWI_enable_interrupt();
			TWI_clear_interrupt();
			TWI_enable_ack();
			TWI_enable();
		}
		else{
			TWI_input_reg[TWI_buffer_address++] = data;
			if (TWI_buffer_address < TWI_input_buffer_size){
				TWI_enable_interrupt();
				TWI_clear_interrupt();
				TWI_enable_ack();
				TWI_enable();
			}
			else{
				TWI_enable_interrupt();
				TWI_clear_interrupt();
				TWI_enable();
			}
		}
	}
	/*
	else if (TWI_slave_tx_cmd_received()){
		//TWI_input_reg[0x00] = 'a';
		uint8_t data = TWDR;
		if (TWI_buffer_address == TWI_output_buffer_size){
			TWI_buffer_address = data;
		}
		TWDR = TWI_output_reg[TWI_buffer_address++];
		if (buffer_address < TWI_output_buffer_size){
			TWI_clear_interrupt();
			TWI_enable_interrupt();
			TWI_enable();
			TWI_enable_ack();
		}
		else{
			TWI_clear_interrupt();
			TWI_enable_interrupt();
			TWI_enable();
		}
	}
	else{
		TWI_enable_interrupt();
		TWI_enable();
		TWI_enable_ack();
	}
	*/
}
