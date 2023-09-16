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
#define ADC1_PIN PIN_PA6  // AIN6 
#define ADC2_PIN PIN_PA7  // AIN7 
#define ADC3_PIN PIN_PB0  // AIN11
#define ADC4_PIN PIN_PB1  // AIN10

struct Reply {
	u8 adc0_low;
	u8 adc1_low;
	u8 adc2_low;
	u8 adc3_low;
	u8 adc01_high;
	u8 adc23_high;
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

	analogReference(EXTERNAL);
	analogClockSpeed(1500);

	// SPI pins, Slave configuration
	pinMode(PIN_PA3, INPUT);         // SCK
	pinMode(PIN_PA1, INPUT);         // MOSI
	pinMode(PIN_PA2, OUTPUT);        // MISO
	pinMode(PIN_PA4, INPUT_PULLUP);  // define idle pin state

	SPI0.CTRLA &= ~0x20;   // slave mode
	SPI0.INTCTRL |= 0x81;  // interrupt on receive

#ifdef DEBUG
	// header
	_printf("iter | ADC 1 | ADC 2 | ADC 3 | ADC 4\r\n");
#endif /* DEBUG */
}

void
loop() {
	static enum {
		Analog_Reading,
		Waiting_To_Load,
		Waiting_To_Send,
	} state;

	static struct Reply next_reply;
	
	u16 adc[4];
	switch (state) {
	case Analog_Reading:
		adc[0] = analogRead(ADC1_PIN);
		adc[1] = analogRead(ADC2_PIN);
		adc[2] = analogRead(ADC3_PIN);
		adc[3] = analogRead(ADC4_PIN);

		next_reply.adc0_low = adc[0];
		next_reply.adc1_low = adc[1];
		next_reply.adc2_low = adc[2];
		next_reply.adc3_low = adc[3];

		next_reply.adc01_high = adc[0] >> 8 | adc[1] >> 4;
		next_reply.adc23_high = adc[2] >> 8 | adc[3] >> 4;

		state = Waiting_To_Load;
		fallthrough;

	case Waiting_To_Load:
		if (_reply_idx < sizeof(Reply)) {
			return;
		}
		_reply = next_reply;
		state = Waiting_To_Send;
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
	    adc[0],
	    adc[1],
	    adc[2],
	    adc[3]);
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
