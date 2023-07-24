/* ADC to SPI firmware for Attiny424 Targeting ST-7i92 PnP BoB
 * Author: Jason Kercher and Justin White
 * License: GPLv2
*/

#include <SPI.h>
#include <Arduino.h>

#include "common/types.h"

// looks like 14-pin SOIC??
#define ADC1_PIN 4 // AIN6  ?
#define ADC2_PIN 5 // AIN7  ?
#define ADC3_PIN 9 // AIN11 ?
#define ADC4_PIN 8 // AIN10 ?

#define SPI_CTRLA    _SFR_MEM8(0x08C0)
#define SPI_CTRLB    _SFR_MEM8(0x08C1)
#define SPI_INTCTRL  _SFR_MEM8(0x08C2)
#define SPI_INTFLAGS _SFR_MEM8(0x08C3)
#define SPI_DATA     _SFR_MEM8(0x08C4)

struct Reply {
	uint16_t header;
	uint16_t values[4];
	uint16_t crc;
};

static struct Reply _reply_data = {.header = 0xa55a};
static uint8_t _reply_idx;

void
setup() {
	Serial.begin(9600);

	// What does this print?!
	char buf[64];
	snprintf(buf, sizeof(buf), "sizeof(void*): %zu", sizeof(void*));
	Serial.print(buf);

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

	SPI_CTRLA &= ~0x20;  // slave mode
	SPI_INTCTRL |= 0x81; // interrupt on receive

	SPI.begin();
}

void
loop() {
	if (digitalRead(SS) == HIGH) { return; }
	if (_reply_idx < sizeof(_reply_data)) { return; }

	_reply_data.values[0] = analogRead(ADC1_PIN);
	_reply_data.values[1] = analogRead(ADC2_PIN);
	_reply_data.values[2] = analogRead(ADC3_PIN);
	_reply_data.values[3] = analogRead(ADC4_PIN);
	_reply_idx = 0;
}

ISR(__vector_SPI_STC) {
	uint8_t spi_recv = SPI_DATA;
	(void)spi_recv;

	if (_reply_idx < sizeof(_reply_data)) {
		const uint8_t* bytes = (uint8_t*)&_reply_data;
		SPI_DATA = bytes[_reply_idx];
		_reply_idx += 1;
	}
}
