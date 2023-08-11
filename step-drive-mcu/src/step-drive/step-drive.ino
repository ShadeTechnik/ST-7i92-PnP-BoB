
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
	u8   reply_delay;
	u8   run_current;
	u8   hold_current;
	u16  microsteps;
	bool enable_coolstep;
	u8   coolstep_lower;
	u8   coolstep_upper;
	u8   stall_guard_threshold;
	bool use_external_sense_resistor;
	bool enable_stealth_chop;
	u32  stealth_chop_duration_threshold;
	bool enable_automatic_current_scaling;
	u8   pin_number;
	bool drive_present;
};

static const struct Drive_Config NEMA23_CONFIG = {
	2,      // reply delay
	100,     // run_current
	50,     // hold_current
	32,     // microsteps
	true,   // enable_coolstep
	5,     // coolstep_lower
	2,     // coolstep_upper
	50,     // stall_guard_threshold
	true,   // use_external_sense_resistor
	true,   // enable_stealth_chop
	20,    // stealth_chop_duration_threshold (does not print back)
	true,   // enable_automatic_current_scaling
	0,      // pin_number (set in setup)
	false,  // drive_present
};

static const struct Drive_Config NEMA17_CONFIG = {
	2,      // reply delay
	50,     // run_current
	30,     // hold_current
	32,     // microsteps
	true,   // enable_coolstep
	5,     // coolstep_lower
	2,     // coolstep_upper
	50,     // stall_guard_threshold
	true,   // use_external_sense_resistor
	true,   // enable_stealth_chop
	20,      // stealth_chop_duration_threshold (does not print back)
	true,   // enable_automatic_current_scaling
	0,      // pin_number (set in setup)
	false,  // drive_present
};

static const struct Drive_Config NEMA8_CONFIG = {
	2,      // reply delay
	20,     // run_current
	20,     // hold_current
	16,     // microsteps
	true,   // enable_coolstep
	5,     // coolstep_lower
	2,     // coolstep_upper
	50,     // stall_guard_threshold
	true,   // use_external_sense_resistor
	true,   // enable_stealth_chop
	20,    // stealth_chop_duration_threshold (does not print back)
	true,   // enable_automatic_current_scaling
	0,      // pin_number (set in setup)
	false,  // drive_present
};

#define DRIVE_COUNT 8
static struct Drive_Config _config[DRIVE_COUNT] = {
	NEMA17_CONFIG,
	NEMA17_CONFIG,
	NEMA8_CONFIG,
	NEMA8_CONFIG,
	NEMA8_CONFIG,
	NEMA8_CONFIG,
	NEMA8_CONFIG, // Not Populated
	NEMA8_CONFIG, // Not Populated
};

static const u8 BFR_CTRL_PIN = PIN_PA3;

void
send_drive_config(int idx) {
	Drive_Config* dc = &_config[idx];

	stepper_driver.setReplyDelay(dc->reply_delay);
	stepper_driver.setRunCurrent(dc->run_current);
	stepper_driver.setHoldCurrent(dc->hold_current);
	stepper_driver.setMicrostepsPerStep(dc->microsteps);
	if (dc->enable_coolstep){
		stepper_driver.enableCoolStep(dc->coolstep_lower, dc->coolstep_upper);
	} else {
		stepper_driver.disableCoolStep();
	}
	stepper_driver.enableCoolStep(dc->coolstep_lower, dc->coolstep_upper);
	stepper_driver.setStallGuardThreshold(dc->stall_guard_threshold);
	if (dc->use_external_sense_resistor) {
		stepper_driver.useExternalSenseResistors();
	} else {
		stepper_driver.useInternalSenseResistors();
	}
	if (dc->enable_stealth_chop) {
		stepper_driver.enableStealthChop();
	stepper_driver.setStealthChopDurationThreshold(dc->stealth_chop_duration_threshold);
	} else {
		stepper_driver.disableStealthChop();
	}
	if (dc->enable_automatic_current_scaling) {
	stepper_driver.enableAutomaticCurrentScaling();
	} else {
	stepper_driver.disableAutomaticCurrentScaling();
	}
  stepper_driver.enable();
}

void
setup() {
	_config[0].drive_present = true;
	_config[1].drive_present = true;
	_config[2].drive_present = false;
	_config[3].drive_present = false;
	_config[4].drive_present = true;
	_config[5].drive_present = true;
	_config[6].drive_present = false; // Not Populated
	_config[7].drive_present = false; // Not Populated

	_config[0].pin_number = PIN_PA4;
	_config[1].pin_number = PIN_PA5;
	_config[2].pin_number = PIN_PA6;
	_config[3].pin_number = PIN_PA7;
	_config[4].pin_number = PIN_PB0;
	_config[5].pin_number = PIN_PB1;
	_config[6].pin_number = PIN_PB4; // Not Populated
	_config[7].pin_number = PIN_PB5; // Not Populated

  pinMode(PIN_PA4, OUTPUT);
  pinMode(PIN_PA5, OUTPUT);
  pinMode(PIN_PA6, OUTPUT);
  pinMode(PIN_PA7, OUTPUT);
  pinMode(PIN_PB0, OUTPUT);
  pinMode(PIN_PB1, OUTPUT);
  pinMode(PIN_PB4, OUTPUT);
  pinMode(PIN_PB5, OUTPUT);

  // SPI pins, Slave configuration
  SPI.swap(1);                     // Portmux alternate SPI pins
  pinMode(PIN_PC0, INPUT);         // SCK
  pinMode(PIN_PC2, INPUT);         // MOSI
  pinMode(PIN_PC1, OUTPUT);        // MISO
  pinMode(PIN_PC3, INPUT_PULLUP);  // SS, define idle pin state

  SPI0.CTRLA &= ~0x20;   // slave mode
  SPI0.INTCTRL |= 0x81;  // interrupt on receive

	Serial.begin(9600);     // UART0, Debug or external control
	Serial1.begin(115200);  // UART1, Drive Control

	Serial.print("Debug Console Running....\r\n\n");

	stepper_driver.setup(STEP_DRIVE_SERIAL);

	// Stepper drive UART switch select GPIO
	for (int i = 0; i < DRIVE_COUNT; ++i) {
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
		delay(200);
		send_drive_config(i);

		Serial.print("Retrieving Drive ");
		Serial.print(i);
		Serial.println(" Settings......");
		TMC2209::Settings settings = stepper_driver.getSettings();
		Serial.print("settings.is_setup = ");
		Serial.println(settings.is_setup);
		Serial.print("settings.irun_percent = ");
		Serial.println(settings.irun_percent);
		Serial.print("settings.ihold_percent = ");
		Serial.println(settings.ihold_percent);
		Serial.print("settings.microsteps_per_step = ");
		Serial.println(settings.microsteps_per_step);
		Serial.print("settings.cool_step_enabled = ");
		Serial.println(settings.cool_step_enabled);
		Serial.print("stall_guard_threshold = ");
		Serial.println(_config[i].stall_guard_threshold);
		Serial.print("settings.internal_sense_resistors_enabled = ");
		Serial.println(settings.internal_sense_resistors_enabled);
		Serial.print("settings.stealth_chop_enabled = ");
		Serial.println(settings.stealth_chop_enabled);
		Serial.print("settings.automatic_current_scaling_enabled = ");
		Serial.println(settings.automatic_current_scaling_enabled);
		Serial.println("*************************");
		Serial.println();
		digitalWrite(_config[i].pin_number, LOW);  // disconnect drive from UART0
	}


}

int driveNumber = 0;

void
loop() {
	if (digitalRead(PIN_PC3) == HIGH) {
		return;
	}

	u8 spi_data = 0xff;
	while (!fifo_is_empty(_spi_buffer)) {
		spi_data = fifo_get(&_spi_buffer);
		// do something with data?
		// for now, just print to debug...
		char buf[5];
		snprintf(buf, 5, "%02x ", spi_data);
//		Serial.print(buf);
	}
//	Serial.print("\r\n");
}

ISR(__vector_SPI_STC) {
	u8 spi_recv = SPI0.DATA;
	if (!fifo_is_full(_spi_buffer)) {
		fifo_add(&_spi_buffer, spi_recv);
	}

	static u8 x = 0;
	SPI0.DATA = x++;
}
