package i2c

import (
	"fmt"

	"gobot.io/x/gobot"
)

const (
	calipileDefaultSlaveAddress = 0x0C

	// CaliPile (TM) ASIC Registers
	calipileRegisterTPObject1            = 0x01 // r
	calipileRegisterTPObject3            = 0x02 // r
	calipileRegisterTPObject4            = 0x03 // r
	calipileRegisterTPAmbient1           = 0x03 // r
	calipileRegisterTPAmbient2           = 0x04 // r
	calipileRegisterTPObjLP11            = 0x05 // r
	calipileRegisterTPObjLP12            = 0x06 // r
	calipileRegisterTPObjLP13            = 0x07 // r
	calipileRegisterTPObjLP21            = 0x07 // r
	calipileRegisterTPObjLP22            = 0x08 // r
	calipileRegisterTPObjLP23            = 0x09 // r
	calipileRegisterTPAmbLP31            = 0x10 // r
	calipileRegisterTPAmbLP32            = 0x11 // r
	calipileRegisterTPObjLP2Frozen1      = 0x12 // r
	calipileRegisterTPObjLP2Frozen2      = 0x13 // r
	calipileRegisterTPObjLP2Frozen3      = 0x14 // r
	calipileRegisterTPPresence           = 0x15 // r
	calipileRegisterTPMotion             = 0x16 // r
	calipileRegisterTPAmbShock           = 0x17 // r
	calipileRegisterInterruptStatus      = 0x18 // r (autoclear)
	calipileRegisterChipStatus           = 0x19 // r
	calipileRegisterSLP1                 = 0x20 // r/w
	calipileRegisterSLP2                 = 0x20 // r/w
	calipileRegisterSLP3                 = 0x21 // r/w
	calipileRegisterTPPresenceThreshold  = 0x22 // r/w
	calipileRegisterTPMotionThreshold    = 0x23 // r/w
	calipileRegisterTPAmbShockThreshold  = 0x24 // r/w
	calipileRegisterInterruptMask        = 0x25 // r/w
	calipileRegisterCycleTimeForMotion   = 0x26 // r/w
	calipileRegisterSRCSelectForPresence = 0x26 // r/w
	calipileRegisterTPOTDirection        = 0x26 // r/w
	calipileRegisterTimerInterrupt       = 0x27 // r/w
	calipileRegisterTPOTThreshold1       = 0x28 // r/w
	calipileRegisterTPOTThreshold2       = 0x29 // r/w
	calipileRegisterEEPROMControl        = 0x31 // r/w - 0x80 to enable read, 0x00 to disable

	// CaliPile (TM) EEPROM Content Registers
	calipileRegisterEEPROMProtocol     = 0x32
	calipileRegisterEEPROMChecksum1    = 0x33
	calipileRegisterEEPROMChecksum2    = 0x34
	calipileRegisterEEPROMLookup       = 0x41
	calipileRegisterEEPROMPTAT251      = 0x42
	calipileRegisterEEPROMPTAT252      = 0x43
	calipileRegisterEEPROMM1           = 0x44
	calipileRegisterEEPROMM2           = 0x45
	calipileRegisterEEPROMU01          = 0x46
	calipileRegisterEEPROMU01          = 0x47
	calipileRegisterEEPROMUOut11       = 0x48
	calipileRegisterEEPROMUOut12       = 0x49
	calipileRegisterEEPROMTObj1        = 0x50
	calipileRegisterEEPROMSlaveAddress = 0x63 // r
)

// CalipileDriver is an I2C driver for the Excelitas CaliPile TPiS 1385 thermopile sensor
type CalipileDriver struct {
	name       string
	connector  Connector
	connection Connection
	Config
}

// NewCalipileDriver creates a new driver with specified i2c interface.
func NewCalipileDriver(c Connector, options ...func(Config)) *CalipileDriver {
	t := &CalipileDriver{
		name:      gobot.DefaultName("CaliPile"),
		connector: c,
		Config:    NewConfig(),
	}

	for _, option := range options {
		option(t)
	}

	return t
}

// Name returns the name of the device.
func (d *CalipileDriver) Name() string {
	return d.name
}

// SetName sets the name of the device.
func (d *CalipileDriver) SetName(n string) {
	d.name = n
}

// Connection returns the connection of the device.
func (d *CalipileDriver) Connection() gobot.Connection {
	return d.connector.(gobot.Connection)
}

// Start initialises the CaliPile
func (d *CalipileDriver) Start() (err error) {
	bus := d.GetBusOrDefault(d.connector.GetDefaultBus())
	address := d.GetAddressOrDefault(calipileDefaultSlaveAddress)
	if d.connection, err = d.connector.GetConnection(address, bus); err != nil {
		return err
	}
	return d.initialization()
}

// Halt halts the device.
func (d *CalipileDriver) Halt() (err error) {
	return nil
}

// Verify comms link to slave device
func (d *CalipileDriver) initialization() (err error) {
	addr8, err := d.ReadAddress()
	if err != nil {
		return fmt.Errorf("CalipileDriver::initialization() failed to communicate with slave device:\n\t%v", err)
	}

	addrExpected := d.GetAddressOrDefault(calipileDefaultSlaveAddress)
	if int(addr8) != addrExpected {
		return fmt.Errorf("CalipileDriver::initialization() SMBus slave address command returned mismatch, expected:%X actual:%X", addrExpected, int(addr8))
	}

	return nil
}

// ReadAddress returns the 7-bit I2C slave address from EEPROM
func (d *CalipileDriver) ReadAddress() (uint8, error) {
	val16, err := d.read(calipileRegisterEEPROMSlaveAddress)
	if err != nil {
		return 0, fmt.Errorf("CalipileDriver::ReadAddress() failed:\n\t%v", err)
	}

	return uint8(val16), nil
}
