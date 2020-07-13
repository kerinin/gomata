package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"time"

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

	f := New()

	log.Printf("opening port")
	port, err := serial.OpenPort(&serial.Config{Name: "/dev/tty.usbmodem14301", Baud: 9600})
	if err != nil {
		log.Fatalf("failed to open serial port: %s", err)
	}

	err = f.Connect(port)
	if err != nil {
		log.Fatalf("failed to connect to serial port: %s", err)
	}

	log.Printf("configuring stepper")
	err = f.StepperConfigure(devID, Driver, WholeStep, EnablePin, 9, 8, 0, 0, 7, 0)
	if err != nil {
		log.Fatalf("failed to configure stepper: %s", err)
	}

	// log.Printf("setting acceleration")
	// err = f.StepperSetAcceleration(devID, 0.0)
	// if err != nil {
	// 	log.Fatalf("failed to set acceleration: %s", err)
	// }

	log.Printf("setting speed")
	err = f.StepperSetSpeed(devID, 200.0)
	if err != nil {
		log.Fatalf("failed to set speed: %s", err)
	}

	// NOTE: The motor is enabled by default
	// log.Printf("enabling stepper")
	// err = f.StepperEnable(devID, Enabled)
	// if err != nil {
	// 	log.Fatalf("failed to set enabled: %s", err)
	// }
	// log.Printf("disabling stepper")
	// err = f.StepperEnable(devID, NotEnabled)
	// if err != nil {
	// 	log.Fatalf("failed to set enabled: %s", err)
	// }

	log.Printf("moving to 5000")
	err = f.StepperTo(devID, 5000)
	if err != nil {
		log.Fatalf("failed to request move: %s", err)
	}

	ticker := time.NewTicker(500 * time.Millisecond)
	for {
		select {
		case <-ticker.C:
			// err = f.StepperReport(devID)
			// if err != nil {
			// 	log.Fatalf("failed to request report: %s", err)
			// }
		case <-ctx.Done():
			return
		}
	}

}
