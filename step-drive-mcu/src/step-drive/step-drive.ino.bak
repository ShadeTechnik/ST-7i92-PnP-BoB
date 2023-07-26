/* Firmware for Attiny426 for programming Trinamic TMC2209 step drive ICs, Targeting the ST-7i92 PnP BoB
 * Author: Jason Kercher and Justin White
 * License: GPLv2
*/

#include <SPI.h>
#include "common/TMC2209.h"
HardwareSerial & serial_stream = Serial1;

#include "common/fifo.h"
#include "common/types.h"

#define BfrCtlPin      19
#define Drv0_SrlSelPin  2
#define Drv1_SrlSelPin  3
#define Drv2_SrlSelPin  4
#define Drv3_SrlSelPin  5
#define Drv4_SrlSelPin 11
#define Drv5_SrlSelPin 10
#define Drv6_SrlSelPin  7
#define Drv7_SrlSelPin  6

// Stepper Drive variables

// Global
const int  Drv_RepDel    =  4; // Drive UART Repy Delay (2-15)

//X Axis
const bool Drv0_Present  =  1; // 1 if drive is present
int        Drv0_RunCur   = 60; // Run Current  (Percent)
int        Drv0_HldCur   = 30; // Hold Current (Percent)
int        Drv0_HldDel   = 30; // Hold Delay   (Percent)
int        Drv0_uStp     = 16; // uSteps perStep
bool       Drv0_Stlchp   =  1; // StealthChop  1=enabled
bool       Drv0_ClStpEn  =  1; // CoolStep     1=enabled
int        Drv0_ClStpTH  = 20; // CoolStep Threshold    
int        Drv0_StlGrdTH = 50; // Stall Gaurd Threshold  
const bool Drv0_ExtRes   =  1; // Module External Sense Resistors 1=enabled


//Y Axis
const bool Drv1_Present  =  1; // 1 if drive is present
int        Drv1_RunCur   = 80; // Run Current  (Percent)
int        Drv1_HldCur   = 30; // Hold Current (Percent)
int        Drv1_HldDel   = 30; // Hold Delay   (Percent)
int        Drv1_uStp     = 16; // uSteps perStep
bool       Drv1_Stlchp   =  1; // StealthChop  1=enabled
bool       Drv1_ClStpEn  =  1; // CoolStep     1=enabled
int        Drv1_ClStpTH  = 20; // CoolStep Threshold    
int        Drv1_StlGrdTH = 50; // Stall Gaurd Threshold  
const bool Drv1_ExtRes   =  1; // Module External Sense Resistors 1=enabled


//Z Axis, Nozzle See-saw
const bool Drv2_Present  =  1; // 1 if drive is present
int        Drv2_RunCur   = 50; // Run Current  (Percent)
int        Drv2_HldCur   = 20; // Hold Current (Percent)
int        Drv2_HldDel   = 30; // Hold Delay   (Percent)
int        Drv2_uStp     = 16; // uSteps perStep
bool       Drv2_Stlchp   =  1; // StealthChop  1=enabled
bool       Drv2_ClStpEn  =  1; // CoolStep     1=enabled
int        Drv2_ClStpTH  = 20; // CoolStep Threshold    
int        Drv2_StlGrdTH = 50; // Stall Gaurd Threshold  
const bool Drv2_ExtRes   =  1; // Module External Sense Resistors 1=enabled


//A Axis, Nozzle 1
const bool Drv3_Present  =  1; // 1 if drive is present
int        Drv3_RunCur   = 20; // Run Current  (Percent)
int        Drv3_HldCur   = 10; // Hold Current (Percent)
int        Drv3_HldDel   = 30; // Hold Delay   (Percent)
int        Drv3_uStp     = 16; // uSteps perStep
bool       Drv3_Stlchp   =  1; // StealthChop  1=enabled
bool       Drv3_ClStpEn  =  1; // CoolStep     1=enabled
int        Drv3_ClStpTH  = 20; // CoolStep Threshold    
int        Drv3_StlGrdTH = 50; // Stall Gaurd Threshold  
const bool Drv3_ExtRes   =  1; // Module External Sense Resistors 1=enabled


//B Axis, Nozzle 2
const bool Drv4_Present  =  1; // 1 if drive is present
int        Drv4_RunCur   = 20; // Run Current  (Percent)
int        Drv4_HldCur   = 10; // Hold Current (Percent)
int        Drv4_HldDel   = 30; // Hold Delay   (Percent)
int        Drv4_uStp     = 16; // uSteps perStep
bool       Drv4_Stlchp   =  1; // StealthChop  1=enabled
bool       Drv4_ClStpEn  =  1; // CoolStep     1=enabled
int        Drv4_ClStpTH  = 20; // CoolStep Threshold    
int        Drv4_StlGrdTH = 50; // Stall Gaurd Threshold  
const bool Drv4_ExtRes   =  1; // Module External Sense Resistors 1=enabled


// Module Not Present on BoB
const bool Drv5_Present  =  0; // 1 if drive is present
int        Drv5_RunCur   = 60; // Run Current  (Percent)
int        Drv5_HldCur   = 30; // Hold Current (Percent)
int        Drv5_HldDel   = 30; // Hold Delay   (Percent)
int        Drv5_uStp     = 16; // uSteps perStep
bool       Drv5_Stlchp   =  1; // StealthChop  1=enabled
bool       Drv5_ClStpEn  =  1; // CoolStep     1=enabled
int        Drv5_ClStpTH  = 20; // CoolStep Threshold    
int        Drv5_StlGrdTH = 50; // Stall Gaurd Threshold  
const bool Drv5_ExtRes   =  1; // Module External Sense Resistors 1=enabled

// Instantiate TMC2209
TMC2209 stepper_driver;


typedef Fifo(u8, 64) Byte_Fifo;

u8 SPDR;

void
setup() {
	Serial.begin(9600);           // UART0, Debug or external control
	Serial1.begin(115200);        // UART1, Drive Control
	
	Serial.print("Debug Console Running\r\n");
	
	stepper_driver.setup(serial_stream);
	
	// Stepper drive UART switch select GPIO
	pinMode(BfrCtlPin, OUTPUT);   // UART1 Buffer Control 
	pinMode(Drv0_SrlSelPin, OUTPUT); // UART1 Drive Select 0 
	pinMode(Drv1_SrlSelPin, OUTPUT); // UART1 Drive Select 1 
	pinMode(Drv2_SrlSelPin, OUTPUT); // UART1 Drive Select 2 
	pinMode(Drv3_SrlSelPin, OUTPUT); // UART1 Drive Select 3 
	pinMode(Drv4_SrlSelPin, OUTPUT); // UART1 Drive Select 4 
	pinMode(Drv5_SrlSelPin, OUTPUT); // UART1 Drive Select 5 *Module not present
	pinMode(Drv6_SrlSelPin, OUTPUT); // UART1 Drive Select 6 *Not on PCB
	pinMode(Drv7_SrlSelPin, OUTPUT); // UART1 Drive Select 7 *Not on PCB
	/* Buffer control switch is "Active Low" (Set LOW to enable/transmit, Set HIGH to disable/recieve)
	   Drive select pins are "Active High" (Set high to connect UART1 to each drive individually) */
	
	// Setup Initial Pin Sates
	digitalWrite(BfrCtlPin, HIGH);
	digitalWrite(Drv0_SrlSelPin, LOW);
	digitalWrite(Drv1_SrlSelPin, LOW);
	digitalWrite(Drv2_SrlSelPin, LOW);
	digitalWrite(Drv3_SrlSelPin, LOW);
	digitalWrite(Drv4_SrlSelPin, LOW);
	digitalWrite(Drv5_SrlSelPin, LOW);
	digitalWrite(Drv6_SrlSelPin, LOW);
	digitalWrite(Drv7_SrlSelPin, LOW);

	// SPI pins, Slave configuration
	SPI.swap(1);                 // Portmux alternate SPI pins
	pinMode(SCK, INPUT);
	pinMode(MOSI, INPUT);
	pinMode(MISO, OUTPUT);
	pinMode(SS, INPUT_PULLUP);  // define idle pin state
	
	// Setup drives over UART
	if (Drv0_Present) {
		Serial.println("Setting Drive0 Now......");
		digitalWrite(Drv0_SrlSelPin, HIGH); //connect drive
		digitalWrite(BfrCtlPin, LOW); // transmit data
		delay(10);
		stepper_driver.setReplyDelay(Drv_RepDel);
		stepper_driver.setup(serial_stream);
		stepper_driver.setRunCurrent(Drv0_RunCur);
		stepper_driver.setHoldCurrent(Drv0_HldCur);
		stepper_driver.setHoldDelay(Drv0_HldDel);
		stepper_driver.setMicrostepsPerStep(Drv0_uStp);
		stepper_driver.enableCoolStep(Drv0_ClStpEn);
		stepper_driver.setCoolStepDurationThreshold(Drv0_ClStpTH);
		if (Drv0_ExtRes) {
			stepper_driver.useExternalSenseResistors();
		} else {
			stepper_driver.useInternalSenseResistors();
		}
    
    Serial.println("Retrieving Drive0 Settings.....()");
		TMC2209::Settings settings = stepper_driver.getSettings();
		delay(10);
		digitalWrite(BfrCtlPin, HIGH); // receive data
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
		digitalWrite(Drv0_SrlSelPin, LOW); // disconnect drive
	}
    
    if (Drv1_Present) {
      Serial.println("Setting Drive1 Now......");
      digitalWrite(Drv1_SrlSelPin, HIGH); //connect drive
      digitalWrite(BfrCtlPin, LOW); // transmit data
      delay(10);
      stepper_driver.setReplyDelay(Drv_RepDel);
      stepper_driver.setup(serial_stream);
      stepper_driver.setRunCurrent(Drv1_RunCur);
      stepper_driver.setHoldCurrent(Drv1_HldCur);
      stepper_driver.setHoldDelay(Drv1_HldDel);
      stepper_driver.setMicrostepsPerStep(Drv1_uStp);
      stepper_driver.enableCoolStep(Drv1_ClStpEn);
      stepper_driver.setCoolStepDurationThreshold(Drv1_ClStpTH);
      if (Drv1_ExtRes) {
        stepper_driver.useExternalSenseResistors();
      } else {
        stepper_driver.useInternalSenseResistors();
      }
      Serial.println("Retrieving Drive1 Settings.....()");
      TMC2209::Settings settings = stepper_driver.getSettings();
      delay(10);
      digitalWrite(BfrCtlPin, HIGH); // receive data
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
      digitalWrite(Drv1_SrlSelPin, LOW); // disconnect drive
    }

    if (Drv2_Present) {
      Serial.println("Setting Drive2 Now......");
      digitalWrite(Drv2_SrlSelPin, HIGH); //connect drive
      digitalWrite(BfrCtlPin, LOW); // transmit data
      delay(10);
      stepper_driver.setReplyDelay(Drv_RepDel);
      stepper_driver.setup(serial_stream);
      stepper_driver.setRunCurrent(Drv2_RunCur);
      stepper_driver.setHoldCurrent(Drv2_HldCur);
      stepper_driver.setHoldDelay(Drv2_HldDel);
      stepper_driver.setMicrostepsPerStep(Drv2_uStp);
      stepper_driver.enableCoolStep(Drv2_ClStpEn);
      stepper_driver.setCoolStepDurationThreshold(Drv2_ClStpTH);
      if (Drv2_ExtRes) {
        stepper_driver.useExternalSenseResistors();
      } else {
        stepper_driver.useInternalSenseResistors();
      }
      
      Serial.println("Retrieving Drive3 Settings.....()");
      TMC2209::Settings settings = stepper_driver.getSettings();
      delay(10);
      digitalWrite(BfrCtlPin, HIGH); // receive data
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
      digitalWrite(Drv3_SrlSelPin, LOW); // disconnect drive
    }
    
    if (Drv4_Present) {
      Serial.println("Setting Drive4 Now......");
      digitalWrite(Drv4_SrlSelPin, HIGH); //connect drive
      digitalWrite(BfrCtlPin, LOW); // transmit data
      delay(10);
      stepper_driver.setReplyDelay(Drv_RepDel);
      stepper_driver.setup(serial_stream);
      stepper_driver.setRunCurrent(Drv4_RunCur);
      stepper_driver.setHoldCurrent(Drv4_HldCur);
      stepper_driver.setHoldDelay(Drv4_HldDel);
      stepper_driver.setMicrostepsPerStep(Drv4_uStp);
      stepper_driver.enableCoolStep(Drv4_ClStpEn);
      stepper_driver.setCoolStepDurationThreshold(Drv4_ClStpTH);
      if (Drv4_ExtRes) {
        stepper_driver.useExternalSenseResistors();
      } else {
        stepper_driver.useInternalSenseResistors();
      }
      Serial.println("Retrieving Drive1 Settings.....()");
      TMC2209::Settings settings = stepper_driver.getSettings();
      delay(10);
      digitalWrite(BfrCtlPin, HIGH); // receive data
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
      digitalWrite(Drv4_SrlSelPin, LOW); // disconnect drive
    }
    
    if (Drv5_Present) {
      Serial.println("Setting Drive4 Now......");
      digitalWrite(Drv5_SrlSelPin, HIGH); //connect drive
      digitalWrite(BfrCtlPin, LOW); // transmit data
      delay(10);
      stepper_driver.setReplyDelay(Drv_RepDel);
      stepper_driver.setup(serial_stream);
      stepper_driver.setRunCurrent(Drv5_RunCur);
      stepper_driver.setHoldCurrent(Drv5_HldCur);
      stepper_driver.setHoldDelay(Drv5_HldDel);
      stepper_driver.setMicrostepsPerStep(Drv5_uStp);
      stepper_driver.enableCoolStep(Drv5_ClStpEn);
      stepper_driver.setCoolStepDurationThreshold(Drv5_ClStpTH);
      if (Drv5_ExtRes) {
        stepper_driver.useExternalSenseResistors();
      } else {
        stepper_driver.useInternalSenseResistors();
      }
      Serial.println("Retrieving Drive5 Settings.....()");
      TMC2209::Settings settings = stepper_driver.getSettings();
      delay(10);
      digitalWrite(BfrCtlPin, HIGH); // receive data
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
      digitalWrite(Drv5_SrlSelPin, LOW); // disconnect drive
  } 
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
    snprintf(buf, 5, "%02x ", spi_data);
    Serial.print(buf);
  }
}

ISR(__vector_SPI_STC) {
  u8 spi_recv = SPDR;
  if (!fifo_is_full(_spi_buffer)) {
    fifo_add(&_spi_buffer, spi_recv);
  }

  static u8 x = 0;
  SPDR = x++;
}
