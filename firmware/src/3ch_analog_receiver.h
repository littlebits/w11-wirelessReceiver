/*
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

