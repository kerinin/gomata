package gomata

import (
	"fmt"
	"io"

	"github.com/smallnest/ringbuffer"
)

type ReadLog struct {
	buf *ringbuffer.RingBuffer
	r   io.Reader
}

func NewReadLog(capacity int, in io.Reader) *ReadLog {
	var (
		buf = ringbuffer.New(capacity)
		r   = io.TeeReader(in, buf)
	)

	return &ReadLog{buf, r}
}

func (l *ReadLog) Read(p []byte) (n int, err error) {
	if len(p) > l.buf.Capacity() {
		return 0, fmt.Errorf("read data won't fit in buffer")
	}

	if toDrop := l.buf.Length() + len(p) - l.buf.Capacity(); toDrop > 0 {
		dropped, err := l.buf.Read(make([]byte, 0, toDrop))
		if err != nil {
			return 0, fmt.Errorf("reading from buffer: %w", err)
		}
		if dropped != toDrop {
			return 0, fmt.Errorf("failed to drop %d bytes from buffer", toDrop-dropped)
		}
	}
	return l.r.Read(p)
}

func (l *ReadLog) Bytes() []byte {
	return l.buf.Bytes()
}

func (l *ReadLog) String() string {
	return string(l.buf.Bytes())
}
