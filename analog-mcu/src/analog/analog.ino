/* ADC to SPI firmware for Attiny424 Targeting ST-7i92 PnP BoB
 * Author: Jason Kercher and Justin White
 * License: GPLv2
*/

#include <SPI.h>
#include <Arduino.h>

#include "common/util.h"
#include "common/types.h"

#define DEBUG

// 14-pin SOIC
#define ADC1_PIN 4  // AIN6  ?
#define ADC2_PIN 5  // AIN7  ?
#define ADC3_PIN 9  // AIN11 ?
#define ADC4_PIN 8  // AIN10 ?

struct Reply {
	u16 header;
	u16 adc[4];
	u16 crc;
};

static Reply _reply;
static u8    _reply_idx = sizeof(Reply);

static void
_printf(const char* fmt, ...) {
	static char buf[256];
	va_list     args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof buf, fmt, args);
	va_end(args);
	Serial.print(buf);
}

void
setup() {
	Serial.begin(9600);

	// reply_data pins
	pinMode(ADC1_PIN, INPUT);
	pinMode(ADC2_PIN, INPUT);
	pinMode(ADC3_PIN, INPUT);
	pinMode(ADC4_PIN, INPUT);

	// SPI pins, Slave configuration
	pinMode(SCK, INPUT);
	pinMode(MOSI, INPUT);
	pinMode(MISO, OUTPUT);
	pinMode(SS, INPUT_PULLUP);  // define idle pin state

	SPI0.CTRLA &= ~0x20;   // slave mode
	SPI0.INTCTRL |= 0x81;  // interrupt on receive

#ifdef DEBUG
	// header
	_printf("iter | ADC 1 | ADC 2 | ADC 3 | ADC 4\r\n");
#endif /* DEBUG */
}

void
loop() {
	if (digitalRead(SS) == HIGH) {
		return;
	}

	static enum {
		Analog_Reading,
		Waiting_To_Load,
		Waiting_To_Send,
	} state;

	static struct Reply next_reply = {
	    0xa55a,        // header
	    {0, 0, 0, 0},  // adc
	    0xffff,        // crc
	};

	switch (state) {
	case Analog_Reading:
		next_reply.adc[0] = analogRead(ADC1_PIN);
		next_reply.adc[1] = analogRead(ADC2_PIN);
		next_reply.adc[2] = analogRead(ADC3_PIN);
		next_reply.adc[3] = analogRead(ADC4_PIN);
		// TODO: CRC
		state = Waiting_To_Load;
		fallthrough;

	case Waiting_To_Load:
		if (_reply_idx < sizeof(Reply)) {
			return;
		}
		_reply = next_reply;
		fallthrough;

	case Waiting_To_Send:
		if (digitalRead(SS) == LOW) {
			return;
		}
		_reply_idx = 0;
		state = Analog_Reading;

		break;
	}

#ifdef DEBUG
	static u16 iter = 0;
	_printf(
	    "%04x | %-5u | %-5u | %-5u | %-5u\r",
	    iter++,
	    _reply.adc[0],
	    _reply.adc[1],
	    _reply.adc[2],
	    _reply.adc[3]);
#endif /* DEBUG */
}

ISR(__vector_SPI_STC) {
	u8 spi_recv = SPI0.DATA;
	(void)spi_recv;

	if (_reply_idx < sizeof(_reply)) {
		const u8* bytes = (u8*)&_reply;
		SPI0.DATA = bytes[_reply_idx++];
	}
}
