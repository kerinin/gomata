package gomata

import (
	"strings"
	"testing"
)

func ReadLogTest(t *testing.T) {
	data := "abc"
	rl := NewReadLog(2, strings.NewReader(data))

	if len(rl.Bytes()) != 0 {
		t.Fatal("initial bytes was non-empty")
	}

	res := make([]byte, 0, 2)
	read, err := rl.Read(res)
	if err != nil {
		t.Fatalf("reading from rl: %s", err)
	}
	if read != 2 {
		t.Fatal("read data has incorrect length")
	}
	if len(res) != 2 {
		t.Fatal("read data has incorrect length")
	}
	if rl.String() != "ab" {
		t.Fatal("unexpected buffered data")
	}

	read, err = rl.Read(res)
	if err != nil {
		t.Fatalf("reading from rl: %s", err)
	}
	if read != 1 {
		t.Fatal("read data has incorrect length")
	}
	if len(res) != 1 {
		t.Fatal("read data has incorrect length")
	}
	if rl.String() != "bc" {
		t.Fatal("unexpected buffered data")
	}
}
