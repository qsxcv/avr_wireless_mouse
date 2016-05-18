/* firmware for wireless mouse receiver: teensy 2.0 (atmega32u4) + nrf24l01+
 *
 * Copyright (c) 2016 qsxcv
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * how stuff's wired:
 *
 * GND			Vcc (3.3V)
 * B0	SS		F0
 * B1	SCLK		F1
 * B2	MOSI		F4
 * B3	MISO		F5
 * B7	IRQ		F6
 * D0	CE		F7
 * D1			B6
 * D2			B5
 * D3			B4
 * C6			D7
 * C7			D6	teensy LED/debug
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "usb_mouse.h"

#define delay_us(t) __builtin_avr_delay_cycles((t) * F_CPU/1000000)
#define delay_ms(t) __builtin_avr_delay_cycles((t) * F_CPU/1000)


#define PORT_SPI PORTB
#define DDR_SPI	DDRB

#define DD_SCK	1
#define DD_MOSI	2
#define DD_MISO	3

#define SS_NRF24_LOW	(PORTB &= ~(1<<0))
#define SS_NRF24_HIGH	(PORTB |= (1<<0))

// use this instead of bitshifts or LSB/MSB macros.
union motion_data {
	int16_t all;
	struct { uint8_t lo, hi; };
};


// depends on how the fpc connector is connected to the mcu
static void pins_init(void)
{
	// debug LED
	DDRD |= (1<<6);

	// spi
	DDRB |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<0); // outputs
	SS_NRF24_HIGH;
	DDRB &= ~(1<<DD_MISO); PORTB |= (1<<DD_MISO); // pullup input

	//nrf24
	DDRB |= (1<<0); PORTB |= (1<<0);// SS
	DDRD |= (1<<0); // CE output
	DDRB &= ~(1<<7); // IRQ input
	PCMSK0 |= (1<<PCINT7);
}

static void spi_setnrf24mode(void)
{
	// enable spi, master mode, mode 0, clock rate = fck/2 = 4MHz
	SPCR = (1<<SPE) | (1<<MSTR);
	SPSR |= (1<<SPI2X);
}

static inline void spi_send(const uint8_t b)
{
	SPDR = b;
	while(!(SPSR & (1<<SPIF)));
}

static inline uint8_t spi_recv(void)
{
	spi_send(0x00);
	return SPDR;
}


static void nrf24_init(void)
{
	spi_setnrf24mode();
	// power down first
	SS_NRF24_LOW;
	spi_send(0x20 | 0x00); // CONFIG == 0x00
	spi_send(0b00111101);
	SS_NRF24_HIGH;
	delay_ms(10);
	// power up, set PRIM_RX high, only enable RX_DR IRQ, 2 byte CRC
	SS_NRF24_LOW;
	spi_send(0x20 | 0x00); // CONFIG == 0x00
	spi_send(0b00111111);
	SS_NRF24_HIGH;
	delay_ms(10); // required value is 2ms or something
	// flush tx, rx
	SS_NRF24_LOW;
	spi_send(0b11100001);
	SS_NRF24_HIGH;
	SS_NRF24_LOW;
	spi_send(0b11100010);
	SS_NRF24_HIGH;
	// write channel 77
	SS_NRF24_LOW;
	spi_send(0x20 | 0x05); // RF_CH == 0x05
	spi_send(77);
	SS_NRF24_HIGH;
	// dynamic payloads
	SS_NRF24_LOW;
	spi_send(0x20 | 0x1c); // DYNPD
	spi_send(0b00000011);
	SS_NRF24_HIGH;
	// enable all features
	SS_NRF24_LOW;
	spi_send(0x20 | 0x1d); // FEATURE
	spi_send(0b00000111);
	SS_NRF24_HIGH;
	// addresses (use default 0xe7e7e7e7e7)

	// set CE high
	PORTD |= (1<<0);
}


EMPTY_INTERRUPT(PCINT0_vect);
EMPTY_INTERRUPT(TIMER1_COMPA_vect);

int main(void)
{
	// set clock prescaler for 8MHz
	CLKPR = 0x80;
	CLKPR = 0x01;

	set_sleep_mode(SLEEP_MODE_IDLE);

	pins_init();

	usb_init();
	while (!usb_configured());
	delay_ms(456); // arbitrary

	nrf24_init();

	// binary OR of all button states since previous usb transmission
	uint8_t btn_usb = 0x00;
	// previously transmitted button state
	uint8_t btn_usb_prev = 0x00;
	// previous received state
	union motion_data px = {0}, py = {0};
	int8_t pwhl = 0;

	// set up timer1 to set OCF0A in TIFR0 after 0.875ms
	TCCR1A = 0x00;
	TCCR1B = 0x01; // 8MHz
	OCR1A = 7199;

	// set px, py, pwhl to new data first time
	for (uint8_t first = 1; ;) {
		const uint8_t intr_state = SREG;

		// sync main loop to usb frames (1ms) by going idle until woken
		// by SOFI interrupt
		UDINT &= ~(1<<SOFI);
		UDIEN |= (1<<SOFE);
		sleep_mode();
		cli();
		UDIEN &= ~(1<<SOFE);
		// reset timer
		TCNT1 = 0;

		// enable pin change interrupt and timer overflow interrupt
		PCIFR |= (1<<PCIF0);
		PCICR |= (1<<PCIE0);
		TIFR1 |= (1<<OCF1A);
		TIMSK1 |= (1<<OCIE1A);
		SREG = intr_state;
		// go idle until one or the other happens.
		sleep_mode();
		// disable interrupts
		cli();
		TIMSK1 &= ~(1<<OCIE1A);
		PCICR &= ~(1<<PCIE0);
		// if IRQ is still high, no packet was received, start over.
		if (PINB & (1<<7)) {
			SREG = intr_state;
			continue;
		}

		// get offset from ideal timing
		const union motion_data offset = {.all = (TCNT1 - 7000)};

		// new state variables
		uint8_t _btn;
		union motion_data nx, ny;
		int8_t nwhl;

		while (1) {
			// clear IRQ
			SS_NRF24_LOW;
			spi_send(0x20 | 0x07); // status
			spi_send(0b01110000);
			SS_NRF24_HIGH;

			// read out payload
			SS_NRF24_LOW;
			spi_send(0b01100000);
			spi_send(0x00);
			SS_NRF24_HIGH;
			SS_NRF24_LOW;
			spi_send(0b01100001);
			_btn = spi_recv();
			nx.lo = spi_recv();
			nx.hi = spi_recv();
			ny.lo = spi_recv();
			ny.hi = spi_recv();
			nwhl = spi_recv();
			SS_NRF24_HIGH;

			// initialize "previous state" using first packet after
			// either receiver or mouse boots
			if (first || (_btn & 0x80)) {
				px.all = nx.all;
				py.all = ny.all;
				pwhl = nwhl;
			}

			// check if there's new data in rx fifo
			SS_NRF24_LOW;
			spi_send(0xff); // NOP, read STATUS register
			const uint8_t status = SPDR;
			SS_NRF24_HIGH;
			if ((status & (0x0e)) == 0x0e) // if RX FIFO empty
				break;
		}

		// calculate deltas
		const union motion_data dx = {.all = (nx.all - px.all)};
		const union motion_data dy = {.all = (ny.all - py.all)};
		const int8_t dwhl = nwhl - pwhl;

		// load tx with offset
		if ((_btn & 0x40) && (offset.all > 24 || offset.all < -24)) {
PORTD |= (1<<6);
			// flush tx
			SS_NRF24_LOW;
			spi_send(0b11100001);
			SS_NRF24_HIGH;
			// load ack payload with offset timing data
			SS_NRF24_LOW;
			spi_send(0b10101000);
			spi_send(offset.lo);
			spi_send(offset.hi);
			SS_NRF24_HIGH;
PORTD &= ~(1<<6);
		}

		// usb
		// first make sure it's configured
		while (!usb_configured())
			SREG = intr_state;
		cli();

		UENUM = MOUSE_ENDPOINT;
		if (UESTA0X & (1<<NBUSYBK0)) { // untransmitted data still in bank
			UEINTX |= (1<<RXOUTI); // kill bank; RXOUTI == KILLBK
			while (UEINTX & (1<<RXOUTI));
		} else {
			// transmission's finished, or the data that should be in the
			// bank is exactly the same as what was previously transmitted
			// so that there was nothing worth transmitting before.
			btn_usb_prev = btn_usb;
			btn_usb = 0x00;
			px.all = nx.all;
			py.all = ny.all;
			pwhl = nwhl;
		}
		btn_usb |= _btn & 0x07; // mask all but l, r, m
		// only transmit if there's something worth transmitting
		if ((btn_usb != btn_usb_prev)|| dx.all || dy.all || dwhl) {
			UEDATX = btn_usb;
			UEDATX = dx.lo;
			UEDATX = dx.hi;
			UEDATX = dy.lo;
			UEDATX = dy.hi;
			UEDATX = dwhl;
			UEINTX = 0x3a;
		}

		first = 0;
		SREG = intr_state;
	}
}
