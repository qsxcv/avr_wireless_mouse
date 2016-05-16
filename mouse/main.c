/* firmware for wireless mouse: teensy2.0 (atmega32u4) + pmw3366 + nrf24l01+
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
 * how stuff's wired (test board):
 *
 * GND			Vcc (3.3V)
 * B0			F0	IRQ(nrf24)
 * B1	SCLK(shared)	F1	CE(nrf24)
 * B2	MOSI(shared)	F4
 * B3	MISO(shared)	F5
 * B7	SS(nrf24)	F6
 * D0	left		F7
 * D1	right		B6	SS(3366)
 * D2	middle		B5	???(pin 2 of 3366 board)
 * D3	MOT (3366)	B4	NRESET(3366)
 * C6	wheel A		D7
 * C7	wheel B		D6	teensy LED/debug
 *
 *
 * how stuff's wired (actual mouse):
 *
 * GND			Vcc (3.3V)
 * B0			F0
 * B1	SCLK(shared)	F1
 * B2	MOSI(shared)	F4	CE(nrf24)
 * B3	MISO(shared)	F5	NRESET(3366)
 * B7	MOT(3366)	F6	SS(3366)
 * D0	left		F7	SS(nrf24)
 * D1	right		B6	???(pin 2 of 3366 board)
 * D2	middle		B5	wheel B
 * D3			B4	wheel A
 * C6			D7
 * C7	IRQ(nrf24)	D6	teensy LED/debug
 */

// probably should have used the USART SPI and taken advantage of double buffered transmitter. oh well

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include "srom_3366_0x09.h"

#define delay_us(t) __builtin_avr_delay_cycles((t) * F_CPU/1000000)
#define delay_ms(t) __builtin_avr_delay_cycles((t) * F_CPU/1000)

#define DD_SCK	1
#define DD_MOSI	2
#define DD_MISO	3

// how many ms a click must be released to be considered completely released.
// only affects release latency, not click latency
#define DEBOUNCE_TIME 16

// how many multiples of 256ms before going into powersave mode
#define AFK_TIMEOUT 240

// use this instead of bitshifts or LSB/MSB macros.
union motion_data {
	int16_t all;
	struct { uint8_t lo, hi; };
};


//#define TEST_BOARD
#ifdef TEST_BOARD
	#define SS_3366_LOW	(PORTB &= ~(1<<6))
	#define SS_3366_HIGH	(PORTB |= (1<<6))

	#define SS_NRF24_LOW	(PORTB &= ~(1<<7))
	#define SS_NRF24_HIGH	(PORTB |= (1<<7))

	#define CE_LOW		(PORTF &= ~(1<<1))
	#define CE_HIGH		(PORTF |= (1<<1))

	#define IRQ_IS_LOW	(!(PINF & (1<<0)))

	#define WHL_A_IS_HIGH	(!!(PINC & (1<<6)))
	#define WHL_B_IS_HIGH	(!!(PINC & (1<<7)))
	static void pins_init(void)
	{
		// debug LED
		DDRD |= (1<<6);

		// set everything as pullup input (except debug led)
		PORTB = 0xff;
		PORTC = 0xff;
		PORTD = 0xff & ~(1<<6);
		PORTE = 0xff;
		PORTF = 0xff;

		// buttons (this and some stuff below is redundant due to above)
		DDRD &= ~(0x07);
		PORTD |= 0x07; // D0, D1, D2 pullup inputs for L, R, M

		// wheel (mechanical encoder, quadrature outputs A/B)
		DDRC &= ~((1<<6) | (1<<7));
		PORTC |= (1<<6) | (1<<7);

		// spi
		PORTB &= ~(1<<0); // keep default SS low output to enable SPI
		DDRB |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<0); // outputs
		DDRB &= ~(1<<DD_MISO);// PORTB |= (1<<DD_MISO); // pullup input

		// 3366
		DDRB |= (1<<6); PORTB |= (1<<6); // SS
		DDRB |= (1<<4); PORTB |= (1<<4); // NRESET(3366) high output
		DDRB |= (1<<5); PORTB |= (1<<5); // ???(pin 2 of 3366 board), B7 high output
		DDRD &= ~(1<<3); // MOT

		//nrf24
		DDRB |= (1<<7); PORTB |= (1<<7); // SS
		PORTF &= ~(1<<1); DDRF |= (1<<1); // CE output
		DDRF &= ~(1<<0); // IRQ input
	}
#else
	#define SS_3366_LOW	(PORTF &= ~(1<<6))
	#define SS_3366_HIGH	(PORTF |= (1<<6))

	#define SS_NRF24_LOW	(PORTF &= ~(1<<7))
	#define SS_NRF24_HIGH	(PORTF |= (1<<7))

	#define CE_LOW		(PORTF &= ~(1<<4))
	#define CE_HIGH		(PORTF |= (1<<4))

	#define IRQ_IS_LOW	(!(PINC & (1<<7)))

	#define WHL_A_IS_HIGH	(!!(PINB & (1<<4)))
	#define WHL_B_IS_HIGH	(!!(PINB & (1<<5)))
	static void pins_init(void)
	{
		// debug LED
		DDRD |= (1<<6);

		// set everything as pullup input (except debug led)
		PORTB = 0xff;
		PORTC = 0xff;
		PORTD = 0xff & ~(1<<6);
		PORTE = 0xff;
		PORTF = 0xff;

		// buttons (this and some stuff below is redundant due to above)
		DDRD &= ~(0x07);
		PORTD |= 0x07; // D0, D1, D2 pullup inputs for L, R, M

		// wheel (mechanical encoder, quadrature outputs A/B)
		DDRB &= ~((1<<4) | (1<<5));
		PORTB |= (1<<4) | (1<<5);

		// spi
		PORTB &= ~(1<<0); // keep default SS low output to enable SPI
		DDRB |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<0); // outputs
		DDRB &= ~(1<<DD_MISO);// PORTB |= (1<<DD_MISO); // pullup input

		// 3366
		DDRF |= (1<<6); PORTF |= (1<<6); // SS
		DDRF |= (1<<5); PORTF |= (1<<5); // NRESET(3366) high output
		DDRB |= (1<<6); PORTB |= (1<<6); // ???(pin 2 of 3366 board), B7 high output
		DDRB &= ~(1<<7); // MOT

		//nrf24
		DDRF |= (1<<7); PORTF |= (1<<7); // SS
		PORTF &= ~(1<<4); DDRF |= (1<<4); // CE output
		DDRC &= ~(1<<7); // IRQ input


		EICRA = 0b00010101; // generate interrupt request on any edge of D0/D1/D2
		EIMSK = 0; // but don't enable any actual interrupts
		EIFR = 0b00000111; // clear EIFR
		PCMSK0 = 0b10110000; // PCINT0 for wheel and sensor for sleep mode.
		PCICR = 0; // don't enable PCINT0 yet
	}
#endif


// spi functions
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


static inline void spi_set3366mode(void)
{
	// enable spi, master mode, mode 3, clock rate = fck/4 = 2MHz
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA);
	SPSR &= ~(1<<SPI2X);
}


static inline void spi_3366_write(const uint8_t addr, const uint8_t data)
{
	spi_send(addr | 0x80);
	spi_send(data);
	delay_us(180); // maximum of t_SWW, t_SWR
}

static inline uint8_t spi_3366_read(const uint8_t addr)
{
	spi_send(addr);
	delay_us(160); // t_SRAD
	uint8_t data = spi_recv();
	delay_us(20);
	return data;
}


static void pmw3366_init(const uint8_t dpi)
{
	spi_set3366mode();
	SS_3366_HIGH;
	delay_ms(3);

	// shutdown first
	SS_3366_LOW;
	spi_3366_write(0x3b, 0xb6);
	SS_3366_HIGH;
	delay_ms(300);

	// drop and raise ncs to reset spi port
	SS_3366_LOW;
	delay_us(40);
	SS_3366_HIGH;
	delay_us(40);

	// power up reset
	SS_3366_LOW;
	spi_3366_write(0x3a, 0x5a);
	SS_3366_HIGH;
	delay_ms(50);

	// flip on and off the clock tuning. not sure purpose...
	SS_3366_LOW;
	spi_3366_write(0x3d, 0x95);
	SS_3366_HIGH;
	delay_ms(1);
	SS_3366_LOW;
	spi_3366_write(0x3d, 0x15);
	SS_3366_HIGH;
	delay_ms(1);

	// read from 0x02 to 0x06
	SS_3366_LOW;
	spi_3366_read(0x02);
	spi_3366_read(0x03);
	spi_3366_read(0x04);
	spi_3366_read(0x05);
	spi_3366_read(0x06);

	spi_3366_write(0x10, 0x00); // disable rest mode
	spi_3366_write(0x22, 0x00); // ???

	// srom download
	spi_3366_write(0x13, 0x1d);
	SS_3366_HIGH;
	delay_ms(10);
	SS_3366_LOW;
	spi_3366_write(0x13, 0x18);

	const uint8_t *psrom = srom;
	spi_send(0x62 | 0x80);
	for (uint16_t i = 0; i < SROM_LENGTH; i++) {
		delay_us(15);
		spi_send(pgm_read_byte(psrom++));
	}
	delay_us(18);
	SS_3366_HIGH;
	delay_us(200);

	// rest mode settings (copied from g900 :P)
	SS_3366_LOW;
	spi_3366_write(0x10, 0x20); // 0x20 enables rest mode after ~10s of inactivity
	spi_3366_write(0x14, 0xff); // how long to wait before going to rest mode. 0xff is max (~10 seconds)
	spi_3366_write(0x15, 0x00); // default
	spi_3366_write(0x16, 0x00); // default
	spi_3366_write(0x17, 0xff);
	spi_3366_write(0x18, 0x63); // default
	spi_3366_write(0x19, 0x00); // default
	spi_3366_write(0x1a, 0x5e);
	spi_3366_write(0x1b, 0x8f);
	spi_3366_write(0x1c, 0x01);
	SS_3366_HIGH;


	// "manual" clock tuning
	delay_ms(1); // arbitrary padding
	SS_3366_LOW;
	spi_3366_write(0x3d, 0x93);
	spi_3366_write(0x3d, 0x13); // increase this to increase the clock frequency during run mode
	SS_3366_HIGH;
	delay_ms(1); // arbitrary padding
	SS_3366_LOW;
	spi_3366_write(0x4f, 0x91);
	spi_3366_write(0x4f, 0x11); // increase this to increase the clock frequency during rest mode
	SS_3366_HIGH;
	delay_ms(1); // arbitrary padding

	SS_3366_LOW;
	spi_3366_write(0x0f, dpi);
	spi_3366_write(0x42, 0x00); // angle snapping
	//spi_3366_write(0x63, 0x03); // 3mm lod
	SS_3366_HIGH;
}


static inline void spi_setnrf24mode(void)
{
	// enable spi, master mode, mode 0, clock rate = fck/2 = 4MHz
	SPCR = (1<<SPE) | (1<<MSTR);
	SPSR |= (1<<SPI2X);
}

static void nrf24_init(void)
{
	spi_setnrf24mode();
	// power up, only enable RX_DR IRQ, 2 byte CRC
	SS_NRF24_LOW;
	spi_send(0x20 | 0x00); // CONFIG
	spi_send(0b00111110);
	SS_NRF24_HIGH;
	delay_ms(20);
	// flush tx, rx
	SS_NRF24_LOW;
	spi_send(0b11100001);
	SS_NRF24_HIGH;
	SS_NRF24_LOW;
	spi_send(0b11100010);
	SS_NRF24_HIGH;
	// disable auto retransmit
	SS_NRF24_LOW;
	spi_send(0x20 | 0x04); // SETUP_RETR
	spi_send(0b00000000);
	SS_NRF24_HIGH;
	// write channel 77
	SS_NRF24_LOW;
	spi_send(0x20 | 0x05); // RF_CH
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
}


EMPTY_INTERRUPT(TIMER1_COMPA_vect);
EMPTY_INTERRUPT(TIMER1_COMPB_vect);
EMPTY_INTERRUPT(TIMER1_COMPC_vect);
EMPTY_INTERRUPT(INT0_vect);
EMPTY_INTERRUPT(INT1_vect);
EMPTY_INTERRUPT(INT2_vect);
EMPTY_INTERRUPT(PCINT0_vect);

int main(void)
{
	// set clock prescaler for 8MHz
	CLKPR = 0x80;
	CLKPR = 0x01;

	cli();

	power_all_disable();
	power_spi_enable();
	power_timer1_enable();
	set_sleep_mode(SLEEP_MODE_IDLE);

	pins_init();

	delay_ms(345); // arbitrary

	uint8_t dpi = 0x0f; // default to 800
	if (!(PIND & (1<<0))) // if left is pressed at boot
		dpi = 0xff; // set to 12800

	pmw3366_init(dpi);
	nrf24_init();

	// button stuff
	// previous debounced state
	uint8_t btn_prev = ~(PIND);
	// time (in 125us) button has been unpressed.
	// consider button to be released if this time exceeds DEBOUNCE_TIME.
	uint8_t btn_time[3] = {0, 0, 0};

	// absolute positions. relies on integer overflow
	union motion_data x = {0}, y = {0};

	// wheel stuff
	uint8_t whl_prev_same = 0; // what A was the last time A == B
	uint8_t whl_prev_diff = 0; // what A was the last time A != B
	// absolute scroll position. relies on integer overflow
	int8_t whl = 0;

	// begin burst mode for 3366
	spi_set3366mode();
	SS_3366_LOW;
	spi_3366_write(0x50, 0x00);
	SS_3366_HIGH;

	// set up timer1 to set OCF0A in TIFR0 every 1ms
	TCCR1A = 0x00;
	TCCR1B = 0x09; // CTC, 8MHz
	OCR1A = 7999; // main loop nominal period (7999 + 1) / 8MHz = 1ms
	OCR1B = 320; // timing of when to read burst mode data from sensor
	OCR1C = 800; // timing of when to load nrf24l01+ with data

	// let receiver know if it's the first time sending data, so that it
	// can reset the reference for absolute position and that there's no
	// jump when rebooting the mouse
	// uint8_t first = 0x80; // transmitted as MSB with button data below.

	// when sync reaches 0, always send a packet with bit 6 in btn set, to
	// tell the receiver to calculate the timing offset.
	// when sync reaches 1, always send a packet requesting ACK to load the
	// timing offset
	// i.e. when it overflows, so 256ms periodicity.

	// when sync reaches 0, afk increments.
	// afk is cleared by any motion or button press.
	// when afk reaches AFK_TIMEOUT, go into powerdown mode.
	for (uint8_t first = 0x80, sync = 0, afk = 0; ; first = 0, sync++) {
		// sync to 1ms intervals using timer1
	//	if (TIFR1 & (1<<OCF1A)) PORTD |= (1<<6);
		TIMSK1 |= (1<<OCIE1A);
		sei(); sleep_mode(); cli();
		TIMSK1 &= ~(1<<OCIE1A);
		TIFR1 |= (1<<OCF1A); TIFR1 |= (1<<OCF1B); TIFR1 |= (1<<OCF1C);

		// begin burst mode read
		spi_set3366mode();
		SS_3366_LOW;
		spi_send(0x50);
		// do stuff here instead of busy waiting for 35us

		// read wheel
		int8_t dwhl = 0;
		const uint8_t whl_a = WHL_A_IS_HIGH;
		const uint8_t whl_b = WHL_B_IS_HIGH;
	//	if (whl_a == whl_b) {
	//		if (whl_a != whl_prev_same) {
	//			dwhl = 2 * (whl_a ^ whl_prev_diff) - 1;
	//			whl += dwhl;
	//			whl_prev_same = whl_a;
	//		}
	//	} else
	//		whl_prev_diff = whl_a;
		if (whl_a != whl_b)
			whl_prev_diff = whl_a;
		else if (whl_a != whl_prev_same) {
			dwhl = 2 * (whl_a ^ whl_prev_diff) - 1;
			whl += dwhl;
			whl_prev_same = whl_a;
		}

		// read buttons
		/*
		PIND 0 EIFR 0: low, no edges -> is low
		PIND 0 EIFR 1: low, edge -> is low
		PIND 1 EIFR 0: high, no edges -> always high during last 1ms
		PIND 1 EIFR 1: high, edge -> low at some point in the last 1ms
		*/
		const uint8_t btn_unpressed = PIND & ~(EIFR);
		EIFR = 0b00000111; // clear EIFR
		// manual loop debouncing for every button
		uint8_t btn_dbncd = 0x00;
		#define DEBOUNCE(index) \
		if ((btn_prev & (1<<index)) && (btn_unpressed & (1<<index))) { \
			btn_time[index]++; \
			if (btn_time[index] < DEBOUNCE_TIME) \
				btn_dbncd |= (1<<index); \
		} else { \
			btn_time[index] = 0; \
			btn_dbncd |= (~btn_unpressed) & (1<<index); \
		}

		DEBOUNCE(0);
		DEBOUNCE(1);
		DEBOUNCE(2);
		#undef DEBOUNCE

		// wait until 35us have elapsed since spi_send(0x50)
	//	if (TIFR1 & (1<<OCF1B)) PORTD |= (1<<6);
		TIMSK1 |= (1<<OCIE1B);
		sei(); sleep_mode(); cli();
		TIMSK1 &= ~(1<<OCIE1B);

		union motion_data dx, dy;
		spi_send(0x00); // motion, not used
		spi_send(0x00); // observation, not used
		dx.lo = spi_recv();
		dx.hi = spi_recv();
		dy.lo = spi_recv();
		dy.hi = spi_recv();
		SS_3366_HIGH;

		x.all += dx.all;
		y.all += dy.all;

		if (sync == 0) afk++;
		const uint8_t changed = (btn_dbncd != btn_prev) || dx.all || dy.all || dwhl;
		if (changed) afk = 0;

		if (changed || (sync <= 1)) {
			btn_prev = btn_dbncd;
			// W_TX_PAYLOAD if sync == 1, W_TX_PAYLOAD_NOACK otherwise
			const uint8_t mode = (sync == 1) ? 0b10100000 : 0b10110000;
			// send miscellaneous info using top bits of btn byte
			uint8_t btn_send = btn_dbncd | first; // first is either 0x80 or 0
			if (sync == 0) btn_send |= 0x40;

			// try to transmit at the same time every frame
	//		if (TIFR1 & (1<<OCF1C)) {PORTD |= (1<<6);}
			TIMSK1 |= (1<<OCIE1C);
			sei(); sleep_mode(); cli();
			TIMSK1 &= ~(1<<OCIE1C);

			spi_setnrf24mode();
			SS_NRF24_LOW;
			spi_send(0x20 | 0x07); // STATUS
			spi_send(0b01110000); // clear IRQ
			SS_NRF24_HIGH;
			SS_NRF24_LOW;
			spi_send(0b11100001); // flush tx
			SS_NRF24_HIGH;
			SS_NRF24_LOW;
			spi_send(0b11100010); // flush rx
			SS_NRF24_HIGH;

			SS_NRF24_LOW;
			spi_send(mode);
			spi_send(btn_send);
			spi_send(x.lo);
			spi_send(x.hi);
			spi_send(y.lo);
			spi_send(y.hi);
			spi_send(whl);
			SS_NRF24_HIGH;

			// pulse CE to transmit
			CE_HIGH;
			delay_us(12);
			CE_LOW;

			if (sync == 1) { // get ack payload of timing offset
				delay_us(400);
				if (IRQ_IS_LOW) {
					// recycle motion_data union for timing
					union motion_data offset;
					SS_NRF24_LOW;
					spi_send(0b01100001);
					offset.lo = spi_recv();
					offset.hi = spi_recv();
					SS_NRF24_HIGH;
					// shift TCNT1 by the offset, plus a
					// little more because of the time it
					// takes to add stuff to TCNT1.
					TCNT1 += offset.all + 11;
				}
			}
		}

		// power down if afk
		if (afk > AFK_TIMEOUT) {
			// enable external interrupts on INT0/1/2/3, PCINT0
			EIMSK = 0b00000111;
			PCICR = 0x01;
			// go power down mode; wake up on interrupt
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			sei(); sleep_mode(); cli();
			// disable external interrupts
			PCICR = 0;
			EIMSK = 0;
			// restore state
			set_sleep_mode(SLEEP_MODE_IDLE);
			sync = 0;
			afk = 0;
		}
	}
}
