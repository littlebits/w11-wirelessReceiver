/*
 *	w11_wirelessReceiver.c
 *
 *	Created: 5/6/2013 10:17:32 AM
 *  Author: littleBits Electronics, Inc.
 *
 * Copyright 2014 littleBits Electronics
 *
 * This file is part of w11-wirelessReceiver.
 *
 * w11-wirelessReceiver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * w11-wirelessReceiver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 
 * This file incorporates work covered by the following copyright:
 *
 * RFD21733 RFDP8 3-Channel Triple-Mode Wireless Decoder
 * Copyright (c) RF Digital Corporation
 * 1601 Pacific Coast Highway, Suite 290
 * Hermosa Beach, CA, 90254
 * www.rfdigital.com
 * Version: 2.0
 * Date: May 2nd, 2013
 */

#include "3ch_analog_receiver.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>

volatile unsigned char counter_1_25ms = 0;
volatile unsigned char channel_counter = 0;
volatile unsigned char FIFO_buffer[FIFO_BUFFER_SIZE];
volatile unsigned char FIFO_head = 0;
volatile unsigned char FIFO_tail = 0;
volatile BOOL FIFO_full = FALSE;
volatile BOOL Wait_buffer_half_fill = TRUE;

// Initialize UART0 module in Double Speed Mode (U2X0 = 1)
void Init_USART_DS_Mode(unsigned int ubrr);			

void Init_Timer1(void);

// Initialize 3 PWM channels with 1 MHz / 256 = 3906.25 Hz frequency
void Init_PWM(void);

void FIFO_write_byte(unsigned char byte);

unsigned char FIFO_read_byte(void);

unsigned char FIFO_bytes_used(void);

void FIFO_flush(void);

void USART_Transmit( unsigned char data );

void USART_Flush(void);


int main(void)
{
	Init_USART_DS_Mode(MYUBRR_DS_MODE);		// Initialize serial communications at 9600 bps
	USART_Flush();
	Init_Timer1();							// Initialize Timer1 to generate interrupts at a rate of 800 Hz
	Init_PWM();								// Initialize PWM outputs
	FIFO_flush();
	wdt_enable(WDTO_60MS);					// Enable the WDT for 60mS
	wdt_reset();							// Reset the WDT counter
	sei();									// Enable Interrupts
	
	while(1)
	{
		if (Wait_buffer_half_fill)
		{
			while (FIFO_bytes_used() < (FIFO_BUFFER_SIZE/2))
			{	// Waiting for FIFO buffer filling half
					if (counter_1_25ms == 0)
					{	// Timeout occurs - clear OCR registers
						OCR0A = 0;
						OCR0B = 0;
						OCR2A = 0;
					}
			}
			Wait_buffer_half_fill = FALSE;
		}
		
		wdt_reset();	// Reset the WDT counter
	}
}


ISR(USART_RX_vect)
{
	FIFO_write_byte(UDR0);
}


ISR(TIMER1_COMPA_vect)
{	// Timer/Counter1 Output Compare A Match Interrupt Vector
	unsigned char temp;
	if (!Wait_buffer_half_fill)
	{
		if (FIFO_full || !(FIFO_head == FIFO_tail))
		{	// FIFO contains at least one byte
			temp = FIFO_read_byte();
			if (temp & 0x01)
			{
				channel_counter = 0;
			}
			switch (channel_counter)
			{	// Update OCR registers
				case 0:
					OCR0A = temp;
				break;
				case 1:
					OCR0B = temp;
				break;
				case 2:
					OCR2A = temp;
				break;
			}
			if (++channel_counter >= C_CHANNEL_NUM)
			{
				channel_counter = 0;
			}
			counter_1_25ms = (RECEIVE_TIMEOUT_MS / TIMER1_CLOCK_PERIOD_MS);	// Reset receiver timeout
		}
		else
		{	// FIFO is empty: Communication failure with the transmitter,
			// reset the channel_counter for synchronization
			FIFO_flush();
			channel_counter = 0;
			Wait_buffer_half_fill = TRUE;
		}
	}
	if (counter_1_25ms > 0)
	{
		counter_1_25ms--;
	}
}


// Initialize UART0 module in Double Speed Mode (U2X0 = 1)
void Init_USART_DS_Mode(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr>>8);				// Set baud rate
	UBRR0L = (unsigned char)ubrr;
	UCSR0A = (1 << U2X0);							// Double the USART Transmission Speed
	UCSR0B = (1 << RXCIE0)|(1<<RXEN0)|(1<<TXEN0);	// RX Complete Interrupt Enable, Enable receiver and transmitter
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);				// Set frame format: 8data, 2stop bit
}


void Init_Timer1(void)
{
	TCNT1 = 0;
	OCR1A = (unsigned int)(F_CPU/800 - 1);			// f = 800 Hz
	TCCR1B = (1 << WGM12)|(1 << CS10);				// CTC Mode, Clock source = clkI/O/1 (No prescaling)
	TIMSK1 = (1 << OCIE1A);							// Timer/Counter1, Output Compare A Match Interrupt Enable
	TIFR1 |= (1 << OCF1A);							// Timer/Counter1, Output Compare A Match Interrupt Reset
}



// Initialize 3 PWM channels with 1 MHz / 256 = 3906.25 Hz frequency
void Init_PWM(void)
{
	// Timer0 configuration
	TCCR0A = (1 << COM0A1)|(1 << COM0B1)|(1 << WGM01)|(1 << WGM00);	// Fast PWM, Clear OC0A and OC0B on Compare Match, set OC0A and OC0B at BOTTOM, (non-inverting mode)
	TCCR0B = (1 << CS00);											// clkI/O/(No prescaling)
	ANALOG_OUT_DDR_1 |= (1 << ANALOG_OUT_PIN_1);					// Set PD5 and PD6 as output pin
	ANALOG_OUT_DDR_2 |= (1 << ANALOG_OUT_PIN_2);
	
	// Timer2 configuration
	TCCR2A = (1 << COM2A1)|(1 << WGM21)|(1 << WGM20);				// Fast PWM, Clear OC2A on Compare Match, set OC2A at BOTTOM, (non-inverting mode)
	TCCR2B = (1 << CS20);											// clkT2S/(No prescaling)
	ANALOG_OUT_DDR_3 |= (1 << ANALOG_OUT_PIN_3);					// Set PB3 as output pin
}


void FIFO_write_byte(unsigned char byte)
{
	if (!FIFO_full)
	{
		FIFO_buffer[FIFO_head] = byte;
		if (++FIFO_head >= FIFO_BUFFER_SIZE)
		{
			FIFO_head = 0;
		}
		if (FIFO_head == FIFO_tail)
		{
			FIFO_full = TRUE;
		}
	}
}


unsigned char FIFO_read_byte(void)
{
	unsigned char return_byte = 0;
	if (FIFO_full || !(FIFO_head == FIFO_tail))
	{
		return_byte = FIFO_buffer[FIFO_tail];
		if (++FIFO_tail >= FIFO_BUFFER_SIZE)
		{
			FIFO_tail = 0;
		}
		FIFO_full = FALSE;
	}
	return(return_byte);
}


unsigned char FIFO_bytes_used(void)
{
	unsigned int return_value;
	if (FIFO_full)
	{
		return_value = FIFO_BUFFER_SIZE;
	}
	else
	{
		if (FIFO_head > FIFO_tail)
		{
			return_value = FIFO_head - FIFO_tail;
		}
		else if (FIFO_head < FIFO_tail)
		{
			return_value = FIFO_head + FIFO_BUFFER_SIZE - FIFO_tail;
		}
		else
		{
			return_value = 0;
		}
	}
	return((unsigned char)return_value);
}


void FIFO_flush(void)
{
	FIFO_full = FALSE;
	FIFO_head = 0;
	FIFO_tail = 0;
}

void USART_Transmit( unsigned char data )
{
	while ( !( UCSR0A & (1<<UDRE0)) );		// Wait for empty transmit buffer
	UDR0 = data;							// Put data into buffer, sends the data
}


void USART_Flush(void)
{
	unsigned char temp;
	while ( (UCSR0A & (1<<RXC0)) )
	{
		temp = UDR0;
	}
}