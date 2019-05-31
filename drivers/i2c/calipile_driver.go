package i2c

import (
	"fmt"
	"math"

	"gobot.io/x/gobot"
)

const (
	calipileDefaultSlaveAddress = 0x0C

	// CaliPile (TM) ASIC Registers
	calipileRegisterTPObject1            = 1  // r
	calipileRegisterTPObject2            = 2  // r
	calipileRegisterTPObject3            = 3  // r, bit 7 only
	calipileRegisterTPAmbient1           = 3  // r, bits [6:0]
	calipileRegisterTPAmbient2           = 4  // r
	calipileRegisterTPObjLP11            = 5  // r
	calipileRegisterTPObjLP12            = 6  // r
	calipileRegisterTPObjLP13            = 7  // r
	calipileRegisterTPObjLP21            = 7  // r
	calipileRegisterTPObjLP22            = 8  // r
	calipileRegisterTPObjLP23            = 9  // r
	calipileRegisterTPAmbLP31            = 10 // r
	calipileRegisterTPAmbLP32            = 11 // r
	calipileRegisterTPObjLP2Frozen1      = 12 // r
	calipileRegisterTPObjLP2Frozen2      = 13 // r
	calipileRegisterTPObjLP2Frozen3      = 14 // r
	calipileRegisterTPPresence           = 15 // r
	calipileRegisterTPMotion             = 16 // r
	calipileRegisterTPAmbShock           = 17 // r
	calipileRegisterInterruptStatus      = 18 // r (autoclear)
	calipileRegisterChipStatus           = 19 // r
	calipileRegisterSLP1                 = 20 // r/w
	calipileRegisterSLP2                 = 20 // r/w
	calipileRegisterSLP3                 = 21 // r/w
	calipileRegisterTPPresenceThreshold  = 22 // r/w
	calipileRegisterTPMotionThreshold    = 23 // r/w
	calipileRegisterTPAmbShockThreshold  = 24 // r/w
	calipileRegisterInterruptMask        = 25 // r/w
	calipileRegisterCycleTimeForMotion   = 26 // r/w
	calipileRegisterSRCSelectForPresence = 26 // r/w
	calipileRegisterTPOTDirection        = 26 // r/w
	calipileRegisterTimerInterrupt       = 27 // r/w
	calipileRegisterTPOTThreshold1       = 28 // r/w
	calipileRegisterTPOTThreshold2       = 29 // r/w
	calipileRegisterEEPROMControl        = 31 // r/w - 0x80 to enable read, 0x00 to disable

	// CaliPile (TM) EEPROM Content Registers
	calipileRegisterEEPROMProtocol     = 32
	calipileRegisterEEPROMChecksum1    = 33
	calipileRegisterEEPROMChecksum2    = 34
	calipileRegisterEEPROMLookup       = 41
	calipileRegisterEEPROMPTAT251      = 42
	calipileRegisterEEPROMPTAT252      = 43
	calipileRegisterEEPROMM1           = 44
	calipileRegisterEEPROMM2           = 45
	calipileRegisterEEPROMU01          = 46
	calipileRegisterEEPROMU02          = 47
	calipileRegisterEEPROMUOut11       = 48
	calipileRegisterEEPROMUOut12       = 49
	calipileRegisterEEPROMTObj1        = 50
	calipileRegisterEEPROMSlaveAddress = 63 // r
)

type eeprom struct {
	protocol     uint8
	checksum     uint16
	lookup       uint8
	ptat25       uint16
	m            uint16
	u0           uint16
	uOut1        uint32
	tObj1        uint8
	slaveAddress uint8
	k            float64
}

// CalipileDriver is an I2C driver for the Excelitas CaliPile TPiS 1385 thermopile sensor
type CalipileDriver struct {
	name       string
	connector  Connector
	connection Connection
	Config
	eeprom *eeprom
}

// NewCalipileDriver creates a new driver with specified i2c interface.
func NewCalipileDriver(c Connector, options ...func(Config)) *CalipileDriver {
	t := &CalipileDriver{
		name:      gobot.DefaultName("CaliPile"),
		connector: c,
		Config:    NewConfig(),
		eeprom:    &eeprom{},
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

	tmpConnection, err := d.connector.GetConnection(0x00, bus)
	if err != nil {
		return fmt.Errorf("CalipileDriver::initialization() failed to create connection to 0x00:\n\t%v", err)
	}

	// Issue general call (to slave address 0x00 from tmpConnection) and reload command
	err = tmpConnection.WriteByteData(0x04, 0x00)
	if err != nil {
		return fmt.Errorf("CalipileDriver::initialization() failed to issue general call and reload command:\n\t%v", err)
	}

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
	if err = d.ReadEEPROM(); err != nil {
		return fmt.Errorf("CalipileDriver::initialization() failed to initialize device:\n\t%v", err)
	}

	addrExpected := uint8(d.GetAddressOrDefault(calipileDefaultSlaveAddress))
	if d.eeprom.slaveAddress != addrExpected {
		return fmt.Errorf("CalipileDriver::initialization() I2C slave address mismatch, expected:%02X actual:%02X", addrExpected, d.eeprom.slaveAddress)
	}

	// See datasheet section 8.5
	d.eeprom.k = (float64(d.eeprom.uOut1) - float64(d.eeprom.u0)) / (math.Pow(float64(d.eeprom.tObj1)+273.15, 3.8) - math.Pow(25.0+273.15, 3.8))

	fmt.Println(d.eeprom)

	return err
}

// ReadEEPROM returns the 7-bit I2C slave address from EEPROM
func (d *CalipileDriver) ReadEEPROM() error {
	// Enable EEPROM read access
	d.connection.WriteByteData(calipileRegisterEEPROMControl, 0x80)

	val8, err := d.connection.ReadByteData(calipileRegisterEEPROMProtocol)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read PROTOCOL:\n\t%v", err)
	}
	d.eeprom.protocol = val8

	val8, err = d.connection.ReadByteData(calipileRegisterEEPROMSlaveAddress)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read SLAVE ADD:\n\t%v", err)
	}
	d.eeprom.slaveAddress = val8 & 0x7F // Clear external addressing bit to retrieve 7-bit address

	val8, err = d.connection.ReadByteData(calipileRegisterEEPROMLookup)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read LOOKUP#:\n\t%v", err)
	}
	d.eeprom.lookup = val8

	val8h, err := d.connection.ReadByteData(calipileRegisterEEPROMPTAT251)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read PTAT25:\n\t%v", err)
	}
	val8l, err := d.connection.ReadByteData(calipileRegisterEEPROMPTAT252)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read PTAT25:\n\t%v", err)
	}
	d.eeprom.ptat25 = uint16(val8h)<<8 | uint16(val8l)

	val8h, err = d.connection.ReadByteData(calipileRegisterEEPROMM1)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read M:\n\t%v", err)
	}
	val8l, err = d.connection.ReadByteData(calipileRegisterEEPROMM2)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read M:\n\t%v", err)
	}
	d.eeprom.m = uint16(val8h)<<8 | uint16(val8l)
	d.eeprom.m /= 100 // M = RegVal / 100

	val8h, err = d.connection.ReadByteData(calipileRegisterEEPROMU01)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read U0:\n\t%v", err)
	}
	val8l, err = d.connection.ReadByteData(calipileRegisterEEPROMU02)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read U0:\n\t%v", err)
	}
	d.eeprom.u0 = uint16(val8h)<<8 | uint16(val8l)
	d.eeprom.u0 += 32768 // U0 = RegVal + 32768

	val8h, err = d.connection.ReadByteData(calipileRegisterEEPROMUOut11)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read Uout1:\n\t%v", err)
	}
	val8l, err = d.connection.ReadByteData(calipileRegisterEEPROMUOut12)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read Uout1:\n\t%v", err)
	}
	d.eeprom.uOut1 = uint32(val8h)<<8 | uint32(val8l)
	d.eeprom.uOut1 *= 2 // Uout1 = RegVal * 2

	val8, err = d.connection.ReadByteData(calipileRegisterEEPROMTObj1)
	if err != nil {
		return fmt.Errorf("CalipileDriver::ReadEEPROM() failed to read Tobj1:\n\t%v", err)
	}
	d.eeprom.tObj1 = val8

	// Disable EEPROM read access
	d.connection.WriteByteData(calipileRegisterEEPROMControl, 0x00)

	return err
}

// ReadRawTPObject returns the 17-bit raw ADC value of TPobject
func (d *CalipileDriver) ReadRawTPObject() (tpObjectRaw uint32, err error) {
	val8a, err := d.connection.ReadByteData(calipileRegisterTPObject1)
	if err != nil {
		return 0, fmt.Errorf("CalipileDriver::ReadRawTPObject() failed:\n\t%v", err)
	}
	val8b, err := d.connection.ReadByteData(calipileRegisterTPObject2)
	if err != nil {
		return 0, fmt.Errorf("CalipileDriver::ReadRawTPObject() failed:\n\t%v", err)
	}
	val8c, err := d.connection.ReadByteData(calipileRegisterTPObject3)
	if err != nil {
		return 0, fmt.Errorf("CalipileDriver::ReadRawTPObject() failed:\n\t%v", err)
	}

	// Read 17-bit value
	tpObjectRaw = ((uint32(val8a) << 24) | (uint32(val8b) << 16) | (uint32(val8c&0x80) << 8)) >> 15

	return tpObjectRaw, err
}

// ReadRawTPAmbient returns the 15-bit raw ambient temperature sensor value (PTAT)
func (d *CalipileDriver) ReadRawTPAmbient() (tpAmbientRaw uint16, err error) {
	val8h, err := d.connection.ReadByteData(calipileRegisterTPAmbient1)
	if err != nil {
		return 0, fmt.Errorf("CalipileDriver::ReadRawTPAmbient() failed:\n\t%v", err)
	}
	val8l, err := d.connection.ReadByteData(calipileRegisterTPAmbient2)
	if err != nil {
		return 0, fmt.Errorf("CalipileDriver::ReadRawTPAmbient() failed:\n\t%v", err)
	}

	// Read 15-bit value
	tpAmbientRaw = (uint16(val8h&0x7F) << 8) | uint16(val8l)

	return tpAmbientRaw, err
}

// CalculateTPAmbient returns the Calculated Ambient Temperature in degrees Kelvin
func (d *CalipileDriver) CalculateTPAmbient() (tpAmbient float64, err error) {
	amb16, err := d.ReadRawTPAmbient()
	if err != nil {
		return 0, fmt.Errorf("CalipileDriver::CalculateTPAmbient() failed:\n\t%v", err)
	}

	// Tamb [K] = (25 + 273.15) + (TPambient − PTAT25) · (1/M)
	tpAmbient = 298.15 + (float64(amb16)-float64(d.eeprom.ptat25))*(1.0/float64(d.eeprom.m))

	return tpAmbient, err
}

// CalculateTPAmbientC returns TPAmbient in degrees Celcius
func (d *CalipileDriver) CalculateTPAmbientC() (tpAmbient float64, err error) {
	tpAmbient, err = d.CalculateTPAmbientC()
	tpAmbient -= 273.15

	return tpAmbient, err
}

// CalculateTPObject returns the Calculated Object Temperature in degres Kelvin
func (d *CalipileDriver) CalculateTPObject() (tpObject float64, err error) {
	obj32, err := d.ReadRawTPObject()
	if err != nil {
		return 0, fmt.Errorf("CalipileDriver::CalculateTPObject() failed:\n\t%v", err)
	}

	tpAmbient, err := d.CalculateTPAmbient()
	if err != nil {
		return 0, fmt.Errorf("CalipileDriver::CalculateTPObject() failed:\n\t%v", err)
	}

	temp0 := math.Pow(tpAmbient, 3.8)
	temp1 := (float64(obj32) - float64(d.eeprom.u0)) / d.eeprom.k
	tpObject = math.Pow(temp0+temp1, 0.2631578947)

	return tpObject, err
}

// CalculateTPObjectC returns TPObject in degrees Celcius
func (d *CalipileDriver) CalculateTPObjectC() (tpObject float64, err error) {
	tpObject, err = d.CalculateTPObject()
	tpObject -= 273.15

	return tpObject, err
}
