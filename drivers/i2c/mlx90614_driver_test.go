package i2c

import (
	"bytes"
	"errors"
	"testing"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/gobottest"
)

var _ gobot.Driver = (*MLX90614Driver)(nil)

// --------- HELPERS
func initTestMLX90614Driver() (driver *MLX90614Driver) {
	driver, _ = initTestMLX90614DriverWithStubbedAdaptor()
	return
}

func initTestMLX90614DriverWithStubbedAdaptor() (*MLX90614Driver, *i2cTestAdaptor) {
	adaptor := newI2cTestAdaptor()
	return NewMLX90614Driver(adaptor), adaptor
}

// --------- TESTS

func TestNewMLX90614Driver(t *testing.T) {
	// Does it return a pointer to an instance of MLX90614Driver?
	var mlx90614 interface{} = NewMLX90614Driver(newI2cTestAdaptor())
	_, ok := mlx90614.(*MLX90614Driver)
	if !ok {
		t.Errorf("NewMLX90614Driver() should have returned a *MLX90614Driver")
	}
}

func TestMLX90614Driver(t *testing.T) {
	mlx90614 := initTestMLX90614Driver()
	gobottest.Refute(t, mlx90614.Connection(), nil)
}

func TestMLX90614DriverStart(t *testing.T) {
	mlx90614, adaptor := initTestMLX90614DriverWithStubbedAdaptor()
	adaptor.i2cReadImpl = func(b []byte) (int, error) {
		// Simulate return of SMBus slave address
		buf := new(bytes.Buffer)
		buf.Write([]byte{0x00, 0x5A, 0xC9}) // LSB, MSB, PEC
		copy(b, buf.Bytes())
		return buf.Len(), nil
	}
	gobottest.Assert(t, mlx90614.Start(), nil)
}

func TestMLX90614ReadAddress(t *testing.T) {
	mlx90614, adaptor := initTestMLX90614DriverWithStubbedAdaptor()
	adaptor.i2cReadImpl = func(b []byte) (int, error) {
		// Simulate return of SMBus slave address
		buf := new(bytes.Buffer)
		buf.Write([]byte{0x00, 0x5A, 0xC9})
		copy(b, buf.Bytes())
		return buf.Len(), nil
	}
	mlx90614.Start()
	sa8, err := mlx90614.ReadAddress()
	gobottest.Assert(t, err, nil)
	gobottest.Assert(t, sa8, uint8(0x5A))
}

func TestMLX90614ReadID(t *testing.T) {
	mlx90614, adaptor := initTestMLX90614DriverWithStubbedAdaptor()
	adaptor.i2cReadImpl = func(b []byte) (int, error) {
		// Simulate return of SMBus slave address
		buf := new(bytes.Buffer)

		switch adaptor.written[len(adaptor.written)-1] {
		case mlx90614RegisterID0:
			buf.Write([]byte{0xBE, 0xEF, 0x4E})
		case mlx90614RegisterID1:
			buf.Write([]byte{0xBE, 0xEF, 0xB7})
		case mlx90614RegisterID2:
			buf.Write([]byte{0xBE, 0xEF, 0xBB})
		case mlx90614RegisterID3:
			buf.Write([]byte{0xBE, 0xEF, 0x42})
		}

		copy(b, buf.Bytes())
		return buf.Len(), nil
	}

	mlx90614.Start()
	id64, err := mlx90614.ReadID()
	gobottest.Assert(t, err, nil)
	gobottest.Assert(t, id64, []byte{0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF})
}

func TestMLX90614StartConnectError(t *testing.T) {
	d, adaptor := initTestMLX90614DriverWithStubbedAdaptor()
	adaptor.Testi2cConnectErr(true)
	gobottest.Assert(t, d.Start(), errors.New("Invalid i2c connection"))
}

func TestMLX90614DriverStartWriteError(t *testing.T) {
	mlx90614, adaptor := initTestMLX90614DriverWithStubbedAdaptor()
	adaptor.i2cWriteImpl = func([]byte) (int, error) {
		return 0, errors.New("write error")
	}
	gobottest.Refute(t, mlx90614.Start(), nil)
}

func TestMLX90614DriverStartReadError(t *testing.T) {
	mlx90614, adaptor := initTestMLX90614DriverWithStubbedAdaptor()
	adaptor.i2cReadImpl = func(b []byte) (int, error) {
		return 0, errors.New("read error")
	}
	gobottest.Refute(t, mlx90614.Start(), nil)
}

func TestMLX90614DriverHalt(t *testing.T) {
	mlx90614 := initTestMLX90614Driver()

	gobottest.Assert(t, mlx90614.Halt(), nil)
}

func TestMLX90614DriverSetName(t *testing.T) {
	b := initTestMLX90614Driver()
	b.SetName("TESTME")
	gobottest.Assert(t, b.Name(), "TESTME")
}

func TestMLX90614DriverOptions(t *testing.T) {
	b := NewMLX90614Driver(newI2cTestAdaptor(), WithBus(2))
	gobottest.Assert(t, b.GetBusOrDefault(1), 2)
}
