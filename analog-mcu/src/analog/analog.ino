#include <SPI.h>

#include "common/fifo.h"

// looks like 14-pin SOIC??
#define ADC1_PIN 4 // AIN6  ?
#define ADC2_PIN 5 // AIN7  ?
#define ADC3_PIN 9 // AIN11 ?
#define ADC4_PIN 8 // AIN10 ?

struct Adc_Data { u16 values[4]; };

typedef Fifo(u8, 64) Byte_Fifo;

void setup()
{
	Serial.begin(9600);
	Serial.print("Hello\r\n");

	// adc_data pins
	pinMode(ADC1_PIN, INPUT);
	pinMode(ADC2_PIN, INPUT);
	pinMode(ADC3_PIN, INPUT);
	pinMode(ADC4_PIN, INPUT);

	// SPI pins
	pinMode(MISO, OUTPUT);

	// SPI Control Register
	SPCR |= (1 << SPE);    // set slave mode
	SPCR |= (1 << SPIE);   // interrupt enable
	SPI.attachInterrupt();

}

static Byte_Fifo _spi_buffer;
static struct Adc_Data _adc_data;
static u8 _adc_idx;

void loop()
{
	if (digitalRead(SS) == HIGH) {
		return;
	}

	u8 spi_data = 0xff;
	if (!fifo_is_empty(_spi_buffer)) {
		spi_data = fifo_get(&_spi_buffer);
		// do something with data?
	}

	if (_adc_idx < sizeof(_adc_data)) {
		return;
	}

	_adc_data = (struct Adc_Data) {
		.values = {
			analogRead(ADC1_PIN),
			analogRead(ADC2_PIN),
			analogRead(ADC3_PIN),
			analogRead(ADC4_PIN),
		},
	};
	_adc_idx = 0;
}

ISR(SPI_STC_vect)
{
	uint8_t spi_recv = SPDR;
	if (!fifo_is_full(_spi_buffer)) {
		fifo_add(&_spi_buffer, spi_recv);
	}

	if (_adc_idx < sizeof(_adc_data)) {
		const u8* adc_bytes = (u8*)&_adc_data;
		SPDR = adc_bytes[_adc_idx];
		_adc_idx += 1;
	}
}
