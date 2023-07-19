#include <SPI.h>

// looks like 14-pin SOIC??
#define ADC1_PIN 4 // AIN6  ?
#define ADC2_PIN 5 // AIN7  ?
#define ADC3_PIN 9 // AIN11 ?
#define ADC4_PIN 8 // AIN10 ?

struct Reply {
	u16 header;
	u16 values[4];
	u16 crc;
};

static struct Reply _reply_data = {.header = 0xa55a};
static u8 _reply_idx;

void
setup() {
	Serial.begin(9600);
	Serial.print("Hello\r\n");

	// reply_data pins
	pinMode(ADC1_PIN, INPUT);
	pinMode(ADC2_PIN, INPUT);
	pinMode(ADC3_PIN, INPUT);
	pinMode(ADC4_PIN, INPUT);

	// SPI pins
	pinMode(MISO, OUTPUT);

	// SPI Control Register
	SPCR |= (1 << SPE);   // set slave mode
	SPCR |= (1 << SPIE);  // interrupt enable
	SPI.attachInterrupt();
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

ISR(SPI_STC_vect) {
	u8 spi_recv = SPDR;
	(void)spi_recv;

	if (_reply_idx < sizeof(_reply_data)) {
		const u8* bytes = (u8*)&_reply_data;
		SPDR = bytes[_reply_idx];
		_reply_idx += 1;
	}
}
