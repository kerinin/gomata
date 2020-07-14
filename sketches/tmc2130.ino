#include <TMC2130Stepper.h>

#define EN_PIN    7   // Nano v3:  16 Mega: 38  //enable (CFG6)
#define DIR_PIN   8   //      19            55  //direction
#define STEP_PIN  9   //      18            54  //step
#define CS_PIN    10  //      17            64  //chip select
TMC2130Stepper TMC2130 = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);

#include <ConfigurableFirmata.h>

#include <DigitalInputFirmata.h>
DigitalInputFirmata digitalInput;

#include <DigitalOutputFirmata.h>
DigitalOutputFirmata digitalOutput;

#include <AnalogInputFirmata.h>
AnalogInputFirmata analogInput;

#include <AnalogOutputFirmata.h>
AnalogOutputFirmata analogOutput;

#include <AccelStepperFirmata.h>
AccelStepperFirmata accelStepper;

#include <FirmataExt.h>
FirmataExt firmataExt;

#include <AnalogWrite.h>

#include <FirmataReporting.h>
FirmataReporting reporting;

void systemResetCallback()
{
  for (byte i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_ANALOG(i)) {
      Firmata.setPinMode(i, ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      Firmata.setPinMode(i, OUTPUT);
    }
  }
  firmataExt.reset();
}

void initTransport()
{
  // Uncomment to save a couple of seconds by disabling the startup blink sequence.
  // Firmata.disableBlinkVersion();
  Firmata.begin(9600);
}

void initFirmata()
{
  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);

  firmataExt.addFeature(digitalInput);
  firmataExt.addFeature(digitalOutput);
  firmataExt.addFeature(analogInput);
  firmataExt.addFeature(analogOutput);
  firmataExt.addFeature(accelStepper);
  firmataExt.addFeature(reporting);

  Firmata.attach(SYSTEM_RESET, systemResetCallback);
}

void setup()
{
  TMC2130.begin(); // Initiate pins and registeries
  TMC2130.SilentStepStick2130(600); // Set stepper current to 600mA
  TMC2130.stealthChop(1); // Enable extremely quiet stepping
  TMC2130.microsteps(2);
  TMC2130.interpolate(1);
  TMC2130.pwm_autoscale(1);
  TMC2130.stealth_gradient(1);
  TMC2130.fullstep_threshold(1);
  //  digitalWrite(EN_PIN, LOW);
  
  initFirmata();

  initTransport();

  Firmata.parse(SYSTEM_RESET);
}

void loop()
{
  digitalInput.report();

  while(Firmata.available()) {
    Firmata.processInput();
  }

  if (reporting.elapsed()) {
    analogInput.report();
  }

  accelStepper.update();
}
