#include <SPI.h>
#include "pins_arduino.h"
#include <Arduino.h>

#include "common/fifo.h"
#include "common/types.h"

// looks like 14-pin SOIC??
#define ADC1_PIN 4 // AIN6  ?
#define ADC2_PIN 5 // AIN7  ?
#define ADC3_PIN 9 // AIN11 ?
#define ADC4_PIN 8 // AIN10 ?

// TODO: Apparently the arduino libs do some hand
//       holding here. These are defined for us
//#define MOSI_PIN 11
//#define MISO_PIN 12
//#define SCK_PIN  13
//#define SS_PIN   2

typedef Fifo(u8, 64) Byte_Fifo;

static Byte_Fifo _spi_buffer;

void setup()
{
	Serial.begin(9600);
	Serial.printf("Hello\r\n");

	// ADC pins
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

void loop()
{
	int16_t adc_val[4] = {0,0,0,0};
	adc_val[0] = analogRead(ADC1_PIN);
	adc_val[1] = analogRead(ADC2_PIN);
	adc_val[2] = analogRead(ADC3_PIN);
	adc_val[3] = analogRead(ADC4_PIN);


}

ISR(SPI_STC_vect)
{
	uint8_t spi_recv = SPDR;
}
