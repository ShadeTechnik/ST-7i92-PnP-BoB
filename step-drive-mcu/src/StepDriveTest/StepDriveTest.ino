#include <SPI.h>
#include "TMC2209.h"
#include "common/fifo.h"
#include "common/types.h"

#define STEP_DRIVE_SERIAL Serial1

typedef Fifo(u8, 64) Byte_Fifo;
static Byte_Fifo _spi_buffer;

struct Drive_Config {
 u8 reply_delay;
 u8 irun_percent;
 u8 ihold_percent;
 u16 microsteps;
 bool enable_coolstep;
 u8 coolstep_lower;
 u8 coolstep_upper;
 u8 stall_guard_threshold;
 bool use_external_sense_resistor;
 bool enable_stealth_chop;
 u32 stealth_chop_duration_threshold;
 bool enable_automatic_current_scaling;
};

struct Drive {
  Drive_Config config;
  u8 switch_pin;
  bool present;
};


static const Drive_Config NEMA23_CONFIG = {
  .reply_delay = 2,
  .irun_percent = 100,
  .ihold_percent = 50,
  .microsteps = 32,
  .enable_coolstep = true,
  .coolstep_lower = 5,
  .coolstep_upper = 2,
  .stall_guard_threshold = 50,
  .use_external_sense_resistor = true,
  .enable_stealth_chop = true,
  .stealth_chop_duration_threshold = 20,
  .enable_automatic_current_scaling = true
};

static const Drive_Config NEMA17_CONFIG = {
  .reply_delay = 2,
  .irun_percent = 50,
  .ihold_percent = 30,
  .microsteps = 32,
  .enable_coolstep = true,
  .coolstep_lower = 5,
  .coolstep_upper = 2,
  .stall_guard_threshold = 50,
  .use_external_sense_resistor = true,
  .enable_stealth_chop = true,
  .stealth_chop_duration_threshold = 20,
  .enable_automatic_current_scaling = true
};

static const Drive_Config NEMA8_CONFIG = {
  .reply_delay = 2,
  .irun_percent = 20,
  .ihold_percent = 20,
  .microsteps = 16,
  .enable_coolstep = false,
  .coolstep_lower = 5,
  .coolstep_upper = 2,
  .stall_guard_threshold = 50,
  .use_external_sense_resistor = true,
  .enable_stealth_chop = true,
  .stealth_chop_duration_threshold = 20,
  .enable_automatic_current_scaling = false
};

static Drive drives[] = {
  { NEMA8_CONFIG, PIN_PA4, true },  // Drive 0
  { NEMA8_CONFIG, PIN_PA5, true },  // Drive 1
  { NEMA17_CONFIG, PIN_PA6, true },  // Drive 2
  { NEMA17_CONFIG, PIN_PA7, true },  // Drive 3
  { NEMA23_CONFIG, PIN_PB0, true },  // Drive 4
  { NEMA23_CONFIG, PIN_PB1, true },  // Drive 5
  { NEMA8_CONFIG, PIN_PB2, false }, // Drive 6 (Not Available)
  { NEMA8_CONFIG, PIN_PB3, false }, // Drive 7 (Not Available)
};

TMC2209 stepper_drivers[sizeof(drives) / sizeof(drives[0])];

void configureDrive(const Drive_Config& config, TMC2209& driver) {
  driver.setRunCurrent(config.irun_percent);
  driver.setHoldCurrent(config.ihold_percent);
  driver.setMicrostepsPerStep(config.microsteps);
  if (config.enable_stealth_chop) {
    driver.enableStealthChop();
  }
  if (config.enable_coolstep) {
    driver.enableCoolStep();
    driver.setCoolStepDurationThreshold(config.coolstep_lower);
  }
  if (config.enable_automatic_current_scaling) {
    driver.enableAutomaticCurrentScaling();
  }
  driver.setStallGuardThreshold(config.stall_guard_threshold);
}

void setup() {
  Serial.begin(115200);
  STEP_DRIVE_SERIAL.begin(115200);

  for (size_t i = 0; i < sizeof(drives)/sizeof(drives[0]); ++i) {
    pinMode(drives[i].switch_pin, OUTPUT);
    digitalWrite(drives[i].switch_pin, LOW);
  }

  delay(100);

  for (size_t i = 0; i < sizeof(drives)/sizeof(drives[0]); ++i) {
    if (!drives[i].present) continue;
    digitalWrite(drives[i].switch_pin, HIGH);
    delay(20);
    stepper_drivers[i].setup(STEP_DRIVE_SERIAL);
    configureDrive(drives[i].config, stepper_drivers[i]);
    delay(drives[i].config.reply_delay * 10); // Allow config to settle
    digitalWrite(drives[i].switch_pin, LOW);
  }
}

void loop() {
  static bool already_printed = false;
  if (already_printed) return;

  for (size_t i = 0; i < sizeof(drives)/sizeof(drives[0]); ++i) {
    if (!drives[i].present) continue;
    Serial.print("--- DRIVE "); Serial.print(i); Serial.println(" ---");
    digitalWrite(drives[i].switch_pin, HIGH);
    delay(20);
    TMC2209::Settings settings = stepper_drivers[i].getSettings();
    digitalWrite(drives[i].switch_pin, LOW);

    Serial.print("is_communicating: ");
    Serial.println(settings.is_communicating);
    // Serial.print("is_setup: ");  // Skipped, often unreliable
    // Serial.println(settings.is_setup);
    Serial.print("irun_percent: ");
    Serial.println(settings.irun_percent);
    Serial.print("ihold_percent: ");
    Serial.println(settings.ihold_percent);
    Serial.print("microsteps: ");
    Serial.println(stepper_drivers[i].getMicrostepsPerStep());
    Serial.println();
    delay(100);
  }

  already_printed = true;
}
