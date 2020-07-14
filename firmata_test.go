package main

import (
	"math"
	"testing"
)

const float32EqualityThreshold = 1e-9

func TestIntegerFromBytes(t *testing.T) {
	var expected = int32(2000)
	var actual = integerFromBytes(0x50, 0x0F, 0x00, 0x00, 0x00)
	if expected != actual {
		t.Fatalf("%v != %v", expected, actual)
	}
}

func TestIntegerConversion(t *testing.T) {
	var expected = int32(2000)
	var b = integerBytes(expected)
	var actual = integerFromBytes(b[0], b[1], b[2], b[3], b[4])
	if expected != actual {
		t.Fatalf("%v != %v", expected, actual)
	}
}

func TestFloatFromBytes(t *testing.T) {
	var expected = float32(1) / float32(60) / float32(60)
	var actual = floatFromBytes(0x31, 0x45, 0x29, 0x05)
	if !almostEqual(expected, actual) {
		t.Fatalf("%v != %v", expected, actual)
	}
}

func TestFloatFromBytes2(t *testing.T) {
	var expected = float32(100)
	var actual = floatFromBytes(0x01, 0x00, 0x00, 0x34)
	if !almostEqual(expected, actual) {
		t.Fatalf("%v != %v", expected, actual)
	}
}

func TestConversion(t *testing.T) {
	var expected = float32(100)
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

func almostEqual(a, b float32) bool {
	return float32(math.Abs(float64(a-b))) <= float32EqualityThreshold
}
