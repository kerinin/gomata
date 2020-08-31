package main

import (
	"context"
	"flag"
	"os"
	"os/signal"

	"github.com/huin/goserial"
	"github.com/kerinin/gomata"
	log "github.com/sirupsen/logrus"
)

const (
	devID = 1
)

var (
	serialPort   = flag.String("serial-port", "/dev/tty.usbmodem14301", "The serial port for Firmata commands")
	dirPin       = flag.Int("dir", 8, "Arduino pin generating DIR signals")
	stepPin      = flag.Int("step", 9, "Arduino pin generating STEP signals")
	enPin        = flag.Int("en", 0, "Arduino pin generating EN signals")
	speed        = flag.Float64("speed", 400.0, "Max speed for stepper")
	target       = flag.Float64("target", 1000, "Target in steps")
	acceleration = flag.Float64("acceleration", 50.0, "Acceleration")
	baud         = flag.Int("baud", 9600, "Serial port Baud Rate")
)

func main() {
	flag.Parse()
	log.SetLevel(log.DebugLevel)

	log.Infof("opening port")
	config := &goserial.Config{Name: *serialPort, Baud: *baud}
	port, err := goserial.OpenPort(config)
	if err != nil {
		log.Fatalf("failed to open serial port: %s", err)
	}
	defer port.Close()
	log.Infof("opened port, creating firmata...")

	f := gomata.New()

	err = f.Connect(port)
	if err != nil {
		log.Fatalf("failed to connect to serial port: %s", err)
	}
	log.Infof("Created firmata, configuring stepper...")

	log.Infof("configuring stepper step:%d dir:%d en:%d", *stepPin, *dirPin, *enPin)
	err = f.StepperConfigure(devID, gomata.Driver, gomata.WholeStep, gomata.EnablePin, *stepPin, *dirPin, 0, 0, *enPin, 0)
	if err != nil {
		log.Fatalf("failed to configure stepper: %s", err)
	}

	var spd = float32(*speed)
	log.Infof("Configured stepper, setting speed %f", spd)

	err = f.StepperSetSpeed(devID, spd)
	if err != nil {
		log.Fatalf("failed to set speed: %s", err)
	}

	log.Infof("setting acceleration %f", *acceleration)
	err = f.StepperSetAcceleration(devID, float32(*acceleration))
	if err != nil {
		log.Fatalf("failed to set acceleration: %s", err)
	}

	log.Infof("enabling stepper")
	err = f.StepperEnable(devID, gomata.Enabled)
	if err != nil {
		log.Fatalf("failed to set enabled: %s", err)
	}

	log.Infof("moving to %f", *target)
	err = f.StepperTo(devID, int32(*target))
	if err != nil {
		log.Fatalf("failed to request move: %s", err)
	}

	ctx := context.Background()
	ctx, cancel := context.WithCancel(ctx)
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)
	defer func() {
		signal.Stop(c)
		cancel()
	}()
	go func() {
		select {
		case <-c:
			cancel()
		case <-ctx.Done():
		}
	}()

	select {
	case <-ctx.Done():
		err = f.StepperStop(devID)
		if err != nil {
			log.Fatalf("failed to stop motor: %s", err)
		}
	case msg := <-f.StepperMoveCompletions():
		log.Infof("move complete: %+v", msg)
		/*
			log.Infof("disabling stepper: %+v", msg)
			err = f.StepperEnable(devID, gomata.NotEnabled)
			if err != nil {
				log.Fatalf("failed to set enabled: %s", err)
			}
		*/
	}
}
