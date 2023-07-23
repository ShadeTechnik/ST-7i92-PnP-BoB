#include <SPI.h>

#include "common/fifo.h"
#include "common/types.h"

typedef Fifo(u8, 64) Byte_Fifo;

void
setup() {
	Serial.begin(9600);
	Serial.print("Hello\r\n");

	// SPI pins
	pinMode(MISO, OUTPUT);

	// SPI Control Register
	SPCR |= (1 << SPE);   // set slave mode
	SPCR |= (1 << SPIE);  // interrupt enable
	SPI.attachInterrupt();
}

static Byte_Fifo _spi_buffer;

void
loop() {
	if (digitalRead(SS) == HIGH) {
		return;
	}

	u8 spi_data = 0xff;
	if (!fifo_is_empty(_spi_buffer)) {
		spi_data = fifo_get(&_spi_buffer);
		// do something with data?
		// for now, just print to debug...
		char buf[5];
		snprintf(buf, "%02x ", spi_data);
		Serial.print(buf);
	}
}

ISR(SPI_STC_vect) {
	u8 spi_recv = SPDR;
	if (!fifo_is_full(_spi_buffer)) {
		fifo_add(&_spi_buffer, spi_recv);
	}

	static u8 x = 0;
	SPDR = x++;
}
