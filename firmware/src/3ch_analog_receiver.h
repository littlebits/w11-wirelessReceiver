/*
 *	w11_wirelessReceiver.h
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

#define SetBit(a,b)    ((a) |= (1<<(b)))
#define ClrBit(a,b)    ((a) &= ~(1<<(b)))
#define ToggleBit(a,b) ((a) ^= (1<<(b)))
#define TestBit(a,b)   ((a) & (1<<(b)))

typedef enum{FALSE=0,TRUE} BOOL;
	
#define F_CPU							1000000 // Clock Speed in Hz
#define BAUD							9600
#define MYUBRR_DS_MODE					F_CPU/8/BAUD-1
#define TIMER1_CLOCK_PERIOD_MS			1.25
#define RECEIVE_TIMEOUT_MS				100	// Timeout in milliseconds of clear the PWM outputs if there is no new data

#define C_DATA_LEN          12  // Data frame length
#define C_CHANNEL_NUM       3   // Total ADC/DAC Channels quantity
#define FIFO_BUFFER_SIZE	48

#define ANALOG_OUT_PIN_1	6 // Analog Out 1: PD6 (OC0A)
#define ANALOG_OUT_DDR_1	DDRD
#define ANALOG_OUT_PIN_2	5 // Analog Out 2: PD5 (OC0B)
#define ANALOG_OUT_DDR_2	DDRD
#define ANALOG_OUT_PIN_3	3 // Analog Out 3: PB3 (OC2A)
#define ANALOG_OUT_DDR_3	DDRB

