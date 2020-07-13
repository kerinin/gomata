package main

import (
	"fmt"
	"math"
	"reflect"
	"testing"
)

func TestFloat1(t *testing.T) {
	var expected = []byte{0x31, 0x45, 0x29, 0x05}
	var floatBits = mkFloatBits(0, -10, 2777777)

	var actual = floatBytes(
		math.Float32frombits(floatBits),
	)
	if !reflect.DeepEqual(expected, actual) {
		t.Fatalf("%v != %v", expected, actual)
	}
}

func TestFloat2(t *testing.T) {
	var expected = []byte{0x01, 0x00, 0x00, 0x34}
	var floatBits = mkFloatBits(0, 2, 1)

	var actual = floatBytes(
		math.Float32frombits(floatBits),
	)
	if !reflect.DeepEqual(expected, actual) {
		t.Fatalf("%v != %v", expected, actual)
	}
}

func TestConversion(t *testing.T) {
	fmt.Printf("starting....\n")
	var expected = float32(200.0)
	var b = floatBytes(expected)
	var actual = floatFromBytes(b[0], b[1], b[2], b[3])
	if expected != actual {
		t.Fatalf("%v != %v", expected, actual)
	}
}

func mkFloatBits(sign uint32, exponent int32, significand uint32) uint32 {
	var biasedExponent = uint32(exponent + 127)
	return (sign << 31) | (biasedExponent << 23) | (significand)
}
