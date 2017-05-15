#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#include "I2C_slave.h"

void I2C_init(uint8_t address){
	TWAR = (address << 1); // load address into TWI address register
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN); // set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
}

void I2C_stop(void){
	// clear acknowledge and enable bits
	TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
}

ISR(TWI_vect){
	uint8_t data; // temporary stores the received data
	if( (TWSR & 0xF8) == TW_SR_SLA_ACK ){ // own address has been acknowledged
		buffer_address = 0xFF;
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	// clear TWI interrupt flag, prepare to receive next byte and acknowledge
	}
	else if( (TWSR & 0xF8) == TW_SR_DATA_ACK ){ // data has been received in slave receiver mode
		rxbuffer[0] = 'b';
		data = TWDR; // save the received byte inside data
		if(buffer_address == 0xFF){ // check wether an address has already been transmitted or not
			buffer_address = data;
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); // clear TWI interrupt flag, prepare to receive next byte and acknowledge
		}
		else{ // if a databyte has already been received
			rxbuffer[buffer_address] = data; // store the data at the current address
			buffer_address++; // increment the buffer address
			if(buffer_address < 0xFF){ // if there is still enough space inside the buffer
				TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); // clear TWI interrupt flag, prepare to receive next byte and acknowledge
			}
			else{
				TWCR |= (1<<TWIE) | (1<<TWINT) | (0<<TWEA) | (1<<TWEN);	// clear TWI interrupt flag, prepare to receive last byte and don't acknowledge
			}
		}
	}
	else if( (TWSR & 0xF8) == TW_ST_DATA_ACK ){ // device has been addressed to be a transmitter
		data = TWDR;// copy data from TWDR to the temporary memory
		if( buffer_address == 0xFF ){ // if no buffer read address has been sent yet
			buffer_address = data;
		}
		// copy the specified buffer address into the TWDR register for transmission
		TWDR = 0x48;//txbuffer[buffer_address];
		buffer_address++; // increment buffer read address
		if(buffer_address < 0xFF){ // if there is another buffer address that can be sent
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); // clear TWI interrupt flag, prepare to send next byte and receive acknowledge
		}
		else{
			TWCR |= (1<<TWIE) | (1<<TWINT) | (0<<TWEA) | (1<<TWEN); // clear TWI interrupt flag, prepare to send last byte and receive not acknowledge
		}
	}
	else{
		// if none of the above apply prepare TWI to be addressed again
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
	}
}
