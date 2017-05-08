/****************************************************************************
# Frobit FroboMind Controller interface
# Copyright (c) 2012-2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
# File: main.c
# Project: Frobit FroboMind Controller interface
# Platform: FroboMind Controller Feb. 2014 http://www.frobomind.org
# Microcontroller: AT90CAN128
# Author: Kjeld Jensen <kjeld@frobomind.org>
# Created:  2012-08-15 Kjeld Jensen
# Modified: 2013-02-04 Kjeld Jensen, Migrated to the BSD license
# Modified: 2014-04-17 Kjeld Jensen, switched to an updated serial driver
# Modified: 2014-08-21 Kjeld Jensen, Ported to FroboMind Controller
# Forged: 2017-04-24 Tobias Lundby, for use in our UAST project at SDU
# Modified: 2017-04-24 Tobias Lundby, cleaned up for unnecessary FroboMind stuff
****************************************************************************/
/* includes */
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "fmctrl_def.h"

#include "avr_serial.h"
#include "I2C_slave.h"

/***************************************************************************/
/* defines */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define false					0
#define true					1

/* defines for the timer/counter0 interrupt */
#define INT0_CNT_TOP			249 /* corresponds on an interrupt0 each 1ms */
// #define FLIPBIT					PE1
// #define FLIPBIT_PORT			PORTE
// #define FLIPBIT_DDR				DDRE

/* signal led defines */
#define LED_STATE_OFF			0
#define LED_STATE_ON			1
#define LED_DELAY				100 /* times the cycle */

/***************************************************************************/
/* global and static variables */

/* timer1 and scheduler variables */
volatile unsigned char t1ms;
unsigned short t1ms_cnt;

/* system variables */
static unsigned char reset_source; /* 0=pow,1=ext,2=brownout,3=wd,4=jtag */
char state;

/* user interface variables*/
char led_state;
char led_signal;
char led_count;
char but1;

// buffer used to convert integer to string
char buffer[3];

/***************************************************************************/
void sched_init(void)
{
	/* timer 0 interrupt init (each 1ms) */
	t1ms = 0;
	t1ms_cnt = 0;
    TIMSK0 = BV(OCIE0A); /* enable interrupt */
    TCCR0A = BV(CS00) | BV(CS01) | BV(WGM01); /* clk/64 (BV(CS00) | BV(CS01): prescale clock with 64 ), TOS is defined as OCR0A (BV(WGM01): clear timer on compare match) */
    OCR0A = INT0_CNT_TOP; /* put compare value into register (output compare register) */
	/* PB_OUT (FLIPBIT_DDR, FLIPBIT); */ /* set 1ms flipbit as output */
}
/***************************************************************************/
/*ISR(SIG_OUTPUT_COMPARE0A) */
ISR (TIMER0_COMP_vect)
{
	t1ms++;
	/* PB_FLIP (FLIPBIT_PORT, FLIPBIT); */ /* time to flip the flip bit */
}
/***************************************************************************/
void led_update(void)
{
	/* led_state = state; */
	switch (led_state) {
		case LED_STATE_ON:
			led_state = LED_STATE_OFF;
			INT_LED_OFF;
			break;

		case LED_STATE_OFF:
			led_count++;
			if (led_count <= led_signal)
			{
				INT_LED_ON;
				led_state = LED_STATE_ON;
			}
			else if (led_count > led_signal + LED_DELAY)
			{
				led_count = 0;
			}
			break;
	}
}
/***************************************************************************/
void led_init(void)
{
	INT_LED_INIT;
	led_count = 0;
	led_signal = 1;

	led_state = LED_STATE_OFF;
}
/***************************************************************************/
void button_update(void)
{
	but1 = PB_IS_HIGH (PIND, PIND7); /* button enabled if logic zero */
}
/***************************************************************************/
void button_init(void)
{
	PB_PULL_UP (PORTD, PD7); /* enable pull-up resistor */
	button_update();
}
/***************************************************************************/
void sched_update (void)
/* This function is run every 1ms */
{
	t1ms_cnt++;
	if (t1ms_cnt == 10000) /* Clear t1ms_cnt (not the same as t1ms) after 10s*/
		t1ms_cnt = 0;

	/* each 10 ms */
	if (t1ms_cnt % 10 == 0) /* each 10 ms (but note delay (100) in function) */
	{
		wdt_reset(); /* reset watchdog */
		//led_update();
	}

	if (t1ms_cnt % 10 == 0) /* each 10 ms (but note delay in function) */
	{
		// convert receiver buffer index 0 to character array and send it via UART
		//itoa(rxbuffer[0], buffer, 10);
		//buffer[0] = '1';
		//buffer[1] = '0';
		//buffer[2] = '0';
		//serial_tx_string(buffer);
		serial_tx('2');
		if (serial_rx_avail()) {
			while (serial_rx_avail()) {
				char tmp_char = serial_rx();
				//serial_tx(tmp_char);
			}

			// if (led_state == LED_STATE_ON) {
			// 	led_state = LED_STATE_OFF;
			// 	INT_LED_OFF;
			// } else {
			// 	led_state = LED_STATE_ON;
			// 	INT_LED_ON;
			// }
		}
		txbuffer[0] = '1';
		txbuffer[1] = '2';
		txbuffer[2] = '3';
		txbuffer[3] = '4';
		txbuffer[4] = '5';
		if (rxbuffer[0] == 'a' ) {
			led_state = LED_STATE_OFF;
			INT_LED_OFF;
		} else if (rxbuffer[0] == 'b') {
			led_state = LED_STATE_ON;
			INT_LED_ON;
		}
	}


	/* EXAMPLE: running a task periodically */
		// if (t1ms_cnt % 20 == 0) /* each 20 ms */
		// {
		// 	wheel_update_ticks_buffers();
		// }
}
/***************************************************************************/
void save_reset_source(void)
{
	char reset_reg = MCUSR; /* save the source of the latest reset */
	MCUSR = 0;
	switch (reset_reg)
	{
		case 1: /* power on */
			reset_source = 0; break;
		case 2: /* reset activated */
			reset_source = 1; break;
		case 4: /* brown out */
			reset_source = 2; break;
		case 8: /* watchdog */
			reset_source = 3; break;
		case 16: /* jtag */
			reset_source = 4; break;
	}
}
/***************************************************************************/
int main(void)
{
	save_reset_source(); /* determine the cause of the startup */
	sched_init(); /* initialize the scheduler */
	led_init(); /* initialize led */
	//button_init(); /* initialize button */

	serial_init(); /* initialize serial communication */

	I2C_init(0x32); // initalize as slave with address 0x32 (just select you address)

	sei(); /* enable interrupts */
	wdt_enable (WDTO_15MS); /* enable watchdog reset at approx 15 ms (ref. p.58) */ /* This is reset every 10ms when the uC is running as intended */

	for (;;) /* go into an endless loop */
	{
		if (t1ms != 0) /* if the interrupt has timed out after 1ms */
		{
			t1ms --;
			sched_update(); /* run the scheduler */
		}
	}
	return 0; /* just for the principle as we never get here */
}
/***************************************************************************/
