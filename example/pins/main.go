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
	serialPort  = flag.String("serial-port", "/dev/tty.usbmodem14301", "The serial port for Firmata commands")
	analog      = flag.Bool("analog", false, "If set, report analog values")
	digitalMode = flag.Int("digital-mode", gomata.InputPin, "If analog is not set, the digital mode to use for the pin. Valid valus are 0 (INPUT) or 11 (INPUT_PULLUP)")
	pin         = flag.Int("pin", 0, "Pin to read")
	baud        = flag.Int("baud", 9600, "Serial port Baud Rate")
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
	log.Infof("Created firmata")

	err = f.SamplingInterval(500)
	if err != nil {
		log.Fatalf("failed to set sampling interval: %s", err)
	}

	if *analog {
		err = f.SetPinMode(*pin, gomata.AnalogPin)
	} else {
		err = f.SetPinMode(*pin, *digitalMode)
	}
	if err != nil {
		log.Fatalf("failed to set pin mode: %s", err)
	}
	for _, pin := range f.Pins() {
		log.Infof("Pin: %+v", pin)
	}

	if *analog {
		err = f.ReportAnalog(*pin, 1)
		if err != nil {
			log.Fatalf("failed to report analog: %w", err)
		}

	} else {
		err = f.ReportDigital(*pin, 1)
		if err != nil {
			log.Fatalf("failed to report digital: %w", err)
		}
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
		return
	}
}
