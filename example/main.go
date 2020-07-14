package main

import (
	"context"
	"os"
	"os/signal"

	"github.com/kerinin/gomata"
	log "github.com/sirupsen/logrus"
	"github.com/tarm/serial"
)

const (
	devID = 1
)

func main() {
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

	f := gomata.New()

	log.Infof("opening port")
	port, err := serial.OpenPort(&serial.Config{Name: "/dev/tty.usbmodem14301", Baud: 9600})
	if err != nil {
		log.Fatalf("failed to open serial port: %s", err)
	}

	err = f.Connect(port)
	if err != nil {
		log.Fatalf("failed to connect to serial port: %s", err)
	}

	log.Infof("configuring stepper")
	err = f.StepperConfigure(devID, gomata.Driver, gomata.WholeStep, gomata.EnablePin, 9, 8, 0, 0, 7, 0)
	if err != nil {
		log.Fatalf("failed to configure stepper: %s", err)
	}

	var spd = float32(400.0)
	log.Infof("setting speed %f", spd)
	err = f.StepperSetSpeed(devID, spd)
	if err != nil {
		log.Fatalf("failed to set speed: %s", err)
	}

	var acc = spd / 8
	log.Infof("setting acceleration %f", acc)
	err = f.StepperSetAcceleration(devID, acc)
	if err != nil {
		log.Fatalf("failed to set acceleration: %s", err)
	}

	// NOTE: The motor is enabled by default
	log.Infof("enabling stepper")
	err = f.StepperEnable(devID, gomata.Enabled)
	if err != nil {
		log.Fatalf("failed to set enabled: %s", err)
	}

	var to = (spd * 5)
	log.Infof("moving to %f", to)
	err = f.StepperTo(devID, int32(to))
	if err != nil {
		log.Fatalf("failed to request move: %s", err)
	}

	select {
	case <-ctx.Done():
		err = f.StepperStop(devID)
		if err != nil {
			log.Fatalf("failed to stop motor: %s", err)
		}
	case msg := <-f.StepperMoveCompletions():
		log.Infof("disabling stepper: %+v", msg)
		err = f.StepperEnable(devID, gomata.NotEnabled)
		if err != nil {
			log.Fatalf("failed to set enabled: %s", err)
		}
	}
}
