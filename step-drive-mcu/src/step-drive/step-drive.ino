
#include <SPI.h>
#include "TMC2209.h"

#include "common/fifo.h"
#include "common/types.h"

#define STEP_DRIVE_SERIAL Serial1

// Instantiate TMC2209
TMC2209 stepper_driver;

typedef Fifo(u8, 64) Byte_Fifo;
static Byte_Fifo _spi_buffer;

struct Drive_Config {
	u16 microsteps_per_step;
	u8 reply_delay;
	u8 run_current;
	u8 hold_current;
	u8 coolstep_lower;
	u8 coolstep_upper;
	u8 stall_guard_threshold;
	u8 pin_number;
	bool use_external_sense_resistor;
	bool drive_present;
};

static const struct Drive_Config DEFAULT_CONFIG = {
    16,     // microsteps_per_step
    4,      // reply delay
    50,     // run_current
    20,     // hold_current
    50,     // coolstep_lower
    50,     // coolstep_upper
    50,     // stall_guard_threshold
    0,      // pin_number (set in setup)
    true,   // use_external_sense_resistor
    false,  // drive_present
};

#define DRIVE_COUNT 8
static struct Drive_Config _config[DRIVE_COUNT] = {
    DEFAULT_CONFIG,
    DEFAULT_CONFIG,
    DEFAULT_CONFIG,
    DEFAULT_CONFIG,
    DEFAULT_CONFIG,
    DEFAULT_CONFIG,
    DEFAULT_CONFIG,
    DEFAULT_CONFIG,
};

static const u8 BFR_CTRL_PIN = PIN_PA3;

void
send_drive_config(int idx) {
	Drive_Config* dc = &_config[idx];

	//stepper_driver.setup(STEP_DRIVE_SERIAL);
	stepper_driver.setReplyDelay(dc->reply_delay);
	stepper_driver.setRunCurrent(dc->run_current);
	stepper_driver.setHoldCurrent(dc->hold_current);
	stepper_driver.setMicrostepsPerStep(dc->microsteps_per_step);
	stepper_driver.enableCoolStep(dc->coolstep_lower, dc->coolstep_upper);
	stepper_driver.setStallGuardThreshold(dc->stall_guard_threshold);
	if (dc->use_external_sense_resistor) {
		stepper_driver.useExternalSenseResistors();
	} else {
		stepper_driver.useInternalSenseResistors();
	}
}

void
setup() {
	_config[0].drive_present = true;
	_config[1].drive_present = true;
	_config[2].drive_present = true;
	_config[3].drive_present = true;
	_config[4].drive_present = true;

	_config[0].pin_number = PIN_PA4;
	_config[1].pin_number = PIN_PA5;
	_config[2].pin_number = PIN_PA6;
	_config[3].pin_number = PIN_PA7;
	_config[4].pin_number = PIN_PB0;
	_config[5].pin_number = PIN_PB1;
	_config[6].pin_number = PIN_PB4;
	_config[7].pin_number = PIN_PB5;

	Serial.begin(9600);     // UART0, Debug or external control
	Serial1.begin(115200);  // UART1, Drive Control

	Serial.print("Debug Console Running\r\n");

	stepper_driver.setup(STEP_DRIVE_SERIAL);

	// Stepper drive UART switch select GPIO
	pinMode(BFR_CTRL_PIN, OUTPUT);  // UART1 Buffer control
	digitalWrite(BFR_CTRL_PIN, HIGH);
	for (int i = 0; i < DRIVE_COUNT; ++i) {
		pinMode(_config[i].pin_number, OUTPUT);
		digitalWrite(_config[i].pin_number, LOW);
	}

	// TODO: load from eeprom
	//       if 0xffs, load default:
	// Configure Drives
	for (int i = 0; i < DRIVE_COUNT; ++i) {
		if (!_config[i].drive_present) {
			continue;
		}
		Serial.print("Setting Drive ");
		Serial.print(i);
		Serial.println(" Now......");
		digitalWrite(_config[i].pin_number, HIGH);  // connect drive to UART0
		digitalWrite(BFR_CTRL_PIN, LOW);            // drive buffer to transmit data

		send_drive_config(i);

		delay(10);

		Serial.println("Retrieving Drive %d Settings.....(), (driveNumber)");
		TMC2209::Settings settings = stepper_driver.getSettings();
		delay(10);
		digitalWrite(_config[i].pin_number, HIGH);  // drive buffer to receive data
		Serial.print("settings.is_setup = ");
		Serial.println(settings.is_setup);
		Serial.print("settings.microsteps_per_step = ");
		Serial.println(settings.microsteps_per_step);
		Serial.print("settings.stealth_chop_enabled = ");
		Serial.println(settings.stealth_chop_enabled);
		Serial.print("settings.irun_percent = ");
		Serial.println(settings.irun_percent);
		Serial.print("settings.ihold_percent = ");
		Serial.println(settings.ihold_percent);
		Serial.print("settings.iholddelay_percent = ");
		Serial.println(settings.iholddelay_percent);
		Serial.print("settings.cool_step_enabled = ");
		Serial.println(settings.cool_step_enabled);
		Serial.print("settings.internal_sense_resistors_enabled = ");
		Serial.println(settings.internal_sense_resistors_enabled);
		Serial.println("*************************");
		Serial.println();
		delay(10);
		digitalWrite(_config[i].pin_number, LOW);  // disconnect drive from UART0
	}

	// SPI pins, Slave configuration
	SPI.swap(1);                     // Portmux alternate SPI pins
	pinMode(PIN_PC0, INPUT);         // SCK
	pinMode(PIN_PC2, INPUT);         // MOSI
	pinMode(PIN_PC1, OUTPUT);        // MISO
	pinMode(PIN_PC3, INPUT_PULLUP);  // SS, define idle pin state

	SPI0.CTRLA &= ~0x20;   // slave mode
	SPI0.INTCTRL |= 0x81;  // interrupt on receive
}

int driveNumber = 0;

void
loop() {
	if (digitalRead(SS) == HIGH) {
		return;
	}

	u8 spi_data = 0xff;
	while (!fifo_is_empty(_spi_buffer)) {
		spi_data = fifo_get(&_spi_buffer);
		// do something with data?
		// for now, just print to debug...
		char buf[5];
		snprintf(buf, 5, "%02x ", spi_data);
		Serial.print(buf);
	}
	Serial.print("\r\n");
}

ISR(__vector_SPI_STC) {
	u8 spi_recv = SPI0.DATA;
	if (!fifo_is_full(_spi_buffer)) {
		fifo_add(&_spi_buffer, spi_recv);
	}

	static u8 x = 0;
	SPI0.DATA = x++;
}
