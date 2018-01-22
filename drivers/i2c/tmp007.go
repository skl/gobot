package i2c

import (
	"bytes"
	"encoding/binary"
	"errors"
	"time"

	"gobot.io/x/gobot"
)

const tmp007Address = 0x40

const (
	tmp007RegisterVSensor    = 0x00
	tmp007RegisterTDie       = 0x01
	tmp007RegisterConfig     = 0x02
	tmp007RegisterTObj       = 0x03
	tmp007RegisterStatus     = 0x04
	tmp007RegisterStatusMask = 0x05
	tmp007RegisterDeviceID   = 0x1F

	tmp007ConfigReset     = 0x8000
	tmp007ConfigConvEN    = 0x1000
	tmp007Config1Sample   = 0x0000
	tmp007Config2Sample   = 0x0200
	tmp007Config4Sample   = 0x0400
	tmp007Config8Sample   = 0x0600
	tmp007Config16Sample  = 0x0800
	tmp007ConfigAlertEN   = 0x0100
	tmp007ConfigAlertFlag = 0x0080
	tmp007ConfigTCEN      = 0x0040

	tmp007MaskALRTEN = 0x8000
	tmp007MaskCRTEN  = 0x4000
)

type deviceID struct {
	didrid uint16
}

// TMP007Driver is a driver for the TMP007 contact-less thermopile sensor.
type TMP007Driver struct {
	name       string
	connector  Connector
	connection Connection
	Config
	deviceID *deviceID
}

// NewTMP007Driver creates a new driver with specified i2c interface.
// Params:
//		conn Connector - the Adaptor to use with this driver
//
// Optional params:
//		i2c.WithBus(int):	bus to use with this driver
//		i2c.WithAddress(int):	address to use with this driver
//
func NewTMP007Driver(c Connector, options ...func(Config)) *TMP007Driver {
	t := &TMP007Driver{
		name:      gobot.DefaultName("TMP007"),
		connector: c,
		Config:    NewConfig(),
		deviceID:  &deviceID{},
	}

	for _, option := range options {
		option(t)
	}

	return t
}

// Name returns the name of the device.
func (d *TMP007Driver) Name() string {
	return d.name
}

// SetName sets the name of the device.
func (d *TMP007Driver) SetName(n string) {
	d.name = n
}

// DeviceID returns the 16bit Device ID register value.
func (d *TMP007Driver) DeviceID() uint16 {
	return d.deviceID.didrid
}

// Connection returns the connection of the device.
func (d *TMP007Driver) Connection() gobot.Connection {
	return d.connector.(gobot.Connection)
}

// Start initialises the TMP007
func (d *TMP007Driver) Start() (err error) {
	bus := d.GetBusOrDefault(d.connector.GetDefaultBus())
	address := d.GetAddressOrDefault(tmp007Address)

	if d.connection, err = d.connector.GetConnection(address, bus); err != nil {
		return err
	}

	return d.initialization()
}

func (d *TMP007Driver) initialization() (err error) {
	var deviceID []byte
	if deviceID, err = d.read(tmp007RegisterDeviceID, 2); err != nil {
		return err
	}

	buf := bytes.NewBuffer(deviceID)
	binary.Read(buf, binary.BigEndian, &d.deviceID.didrid)

	if d.deviceID.didrid != 0x78 {
		return errors.New("TMP007 not detected")
	}

	// Enable ADC conversion
	configInt := uint16(tmp007ConfigConvEN)
	if err = d.write(tmp007RegisterConfig, configInt); err != nil {
		return err
	}

	// Enable conversion ready alert flag
	statusMaskInt := uint16(tmp007MaskCRTEN)
	return d.write(tmp007RegisterStatusMask, statusMaskInt)
}

func (d *TMP007Driver) read(address byte, n int) ([]byte, error) {
	if _, err := d.connection.Write([]byte{address}); err != nil {
		return nil, err
	}

	buf := make([]byte, n)

	var err error
	var bytesRead int
	numRetries := 50
	for numRetries > 0 {
		bytesRead, err = d.connection.Read(buf)
		if bytesRead == n && err == nil {
			break
		}
		numRetries--
	}

	return buf, err
}

func (d *TMP007Driver) write(address byte, data uint16) error {
	b := make([]byte, 2)
	binary.BigEndian.PutUint16(b[0:], data)

	return d.connection.WriteBlockData(address, b)
}

func nDVtoBool(raw int16) bool {
	if (raw & 0x01) != 0 {
		return false
	}
	return true
}

// ReadDieTempC returns the die temperature in degrees Celcius
func (d *TMP007Driver) ReadDieTempC() (float64, error) {
	b, err := d.read(tmp007RegisterTDie, 2)
	if err != nil {
		return 0.0, err
	}
	raw := int16(b[0])<<8 | int16(b[1])

	isValid := nDVtoBool(raw)
	if !isValid {
		return 0.0, errors.New("nDV bit is set")
	}
	raw = raw >> 2 // trim nDV[0] and empty bit[1]

	// 0.03125°C per LSB
	tDie := float64(raw) * 0.03125 // convert to Celcius

	return tDie, nil
}

// ReadObjTempC returns the object temperature in degrees Celcius
func (d *TMP007Driver) ReadObjTempC() (float64, error) {
	b, err := d.read(tmp007RegisterTObj, 2)
	if err != nil {
		return 0.0, err
	}
	raw := int16(b[0])<<8 | int16(b[1])

	isValid := nDVtoBool(raw)
	if !isValid {
		return 0.0, errors.New("nDV bit is set")
	}
	raw = raw >> 2 // trim nDV[0] and empty bit[1]

	// 0.03125°C per LSB
	tObj := float64(raw) * 0.03125 // convert to Celcius

	return tObj, err
}

// ReadSensorVoltage returns the sensor voltage output value in microvolts
func (d *TMP007Driver) ReadSensorVoltage() (float64, error) {
	b, err := d.read(tmp007RegisterVSensor, 2)
	if err != nil {
		return 0.0, err
	}
	raw := int16(b[0])<<8 | int16(b[1])

	// Resolution: 156.25 nV/LSB
	vSensor := (float64(raw) * 156.25) / 1000 // convert to uV

	return vSensor, err
}

// ReadStatus returns the 16-bit status register
func (d *TMP007Driver) ReadStatus() (uint16, error) {
	b, err := d.read(tmp007RegisterStatus, 2)
	if err != nil {
		return 0, err
	}
	raw := uint16(b[0])<<8 | uint16(b[1])

	return raw, err
}

// WaitOnConversionReady polls status register and returns when CRTF bit is set
func (d *TMP007Driver) WaitOnConversionReady() {
	for {
		s, err := d.ReadStatus()
		if ((tmp007MaskCRTEN & s) != 0) && err == nil {
			return
		}
		time.Sleep(65 * time.Millisecond) // 65 = min conv. time 260ms / 4
	}
}
