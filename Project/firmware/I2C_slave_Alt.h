#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

volatile uint8_t buffer_address;
volatile uint8_t TWI_output_reg[0xFF];
volatile uint8_t TWI_input_reg[0xFF];

void I2C_init(uint8_t address, uint8_t input_buffer_size, uint8_t output_buffer_size);
void I2C_stop(void);
ISR(TWI_vect);

#endif // I2C_SLAVE_H
