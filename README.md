# Golang Firmata Protocol

This implements the Firmata protocol for Go. It includes protocol extensions for
AccelStepperFirmata and some other components of the configurable firmata
protocols.

The `sketches` directory contains example Arduino sketches based on the source
code generated by the [Firmata Builder](http://firmatabuilder.com/).

## Usage

1. Upload an appropraite Arduino sketch. The sketch will listen on the serial
   pins and respond to commands issued to it.
2. Open the Arduino's serial port on the controlling computer.
3. Connect gomata to the opened serial port.
4. Issue commands to the Arduino
5. Consume replies from the issued commands using `PinStates()` and friends.

## NOTES

* Reporting analog appears to only work if the pin mode is set to analog. 
* Analog values appear to be delivered on the sampling interval regardless of changes
