package i2c

import (
	"errors"
	"fmt"
	"log"
	"time"

	"gobot.io/x/gobot"
)

const (
	// N.B. Slave address changes require POR and can only by done using mlx90614RegisterAddress
	mlx90614DefaultSlaveAddress = 0x5A

	// RAM read only, opcode 000x_xxxx
	mlx90614RegisterRaw1  = 0x04 // RAM Channel 1
	mlx90614RegisterTA    = 0x06 // Linearized ambient temperature TA
	mlx90614RegisterTObj1 = 0x07 // Linearized object temperature Tobj1
	mlx90614RegisterTObj2 = 0x08 // Linearized object temperature Tobj2

	// EEPROM with write access, opcode 001x_xxxx
	mlx90614RegisterTOMax   = 0x20
	mlx90614RegisterTOMin   = 0x21
	mlx90614RegisterPWMCtrl = 0x22
	mlx90614RegisterTARange = 0x23
	mlx90614RegisterKE      = 0x24 // Should not be altered!
	mlx90614RegisterAddress = 0x2E

	// N.B. care must be taken not to alter other bits in that cell as they contain factory calibration settings!
	//
	// Settling time can be calculated as follows:
	// Tset = 9.719 + [ IIRSetting x ( FIRSetting + 5.26) ] + [ IIRSetting x (FIRSetting + 12.542) ] + [ Dual x IIRSetting x (FIRSetting + 12.542) ] , (milliseconds)
	//   where Dual = 1 for dual zone (bit 6)
	//
	// See datasheet Table 7!
	//
	// bit 15:		1 = Enable sensor test		0 = Disable sensor test
	// bit 14:		1 = Negative sign of Kt2	0 = Positive sign of Kt2		GUARD
	// bit 13..11:	GAIN														GUARD
	// bit 10..8:	FIR															MUST be >= 0x04
	// bit 7:		1 = Negative sign of Ks		0 = Positive sign of Ks			GUARD
	// bit 6:		1 = Dual IR Sensor			0 = Single IR Sensor			GUARD
	// bit 5..4:	PWM config of Ta, Tobj1, Tobj2
	// bit 3:		1 = Repeat sensor test "ON"	0 = Repeat sensor test "OFF"	GUARD
	// bit 2..0:	IIR
	//
	// N.B. Bits marked GUARD should not be altered in order to keep the factory calibration relevant
	mlx90614RegisterConfig = 0x25

	// EPPROM read only, opcode 001x_xxxx
	mlx90614RegisterID0 = 0x3C
	mlx90614RegisterID1 = 0x3D
	mlx90614RegisterID2 = 0x3E
	mlx90614RegisterID3 = 0x3F

	// FLAGS command, opcode 1111_0000
	//
	// Datasheet:
	// Data[7] - EEBUSY - the previous write/erase EEPROM access is still in progress. High active. Data[6] - Unused
	// Data[5] - EE_DEAD - EEPROM double error has occurred. High active.
	// Data[4] - INIT - POR initialization routine is still ongoing. Low active.
	// Data[3] - Not implemented.
	// Data[2...0] and Data[8...15] - All zeros.
	// Flag read is a diagnostic feature. The MLX90614 can be used regardless of these flags.
	mlx90614RegisterFlags = 0xF0

	// SLEEP mode command, opcode 1111_1111
	mlx90614RegisterSleep = 0xFF
)

// MLX90614Driver is an SMBus driver for the Melexis MLX90614 infra red thermometer
type MLX90614Driver struct {
	name       string
	connector  Connector
	connection Connection
	Config
}

// NewMLX90614Driver creates a new driver with specified i2c interface.
// Golang port of the excellent SparkFun Arduino Library:
// https://github.com/sparkfun/SparkFun_MLX90614_Arduino_Library
//
// Params:
//		conn Connector - the Adaptor to use with this driver
//
// Optional params:
//		i2c.WithBus(int):		bus to use with this driver
//		i2c.WithAddress(int):	address to use with this driver
//
func NewMLX90614Driver(c Connector, options ...func(Config)) *MLX90614Driver {
	t := &MLX90614Driver{
		name:      gobot.DefaultName("MLX90614"),
		connector: c,
		Config:    NewConfig(),
	}

	for _, option := range options {
		option(t)
	}

	return t
}

// Name returns the name of the device.
func (d *MLX90614Driver) Name() string {
	return d.name
}

// SetName sets the name of the device.
func (d *MLX90614Driver) SetName(n string) {
	d.name = n
}

// Connection returns the connection of the device.
func (d *MLX90614Driver) Connection() gobot.Connection {
	return d.connector.(gobot.Connection)
}

// Start initialises the MLX90614
func (d *MLX90614Driver) Start() (err error) {
	bus := d.GetBusOrDefault(d.connector.GetDefaultBus())
	address := d.GetAddressOrDefault(mlx90614DefaultSlaveAddress)
	if d.connection, err = d.connector.GetConnection(address, bus); err != nil {
		return err
	}
	return d.initialization()
}

// Halt halts the device.
func (d *MLX90614Driver) Halt() (err error) {
	return nil
}

// Verify comms link to slave device
func (d *MLX90614Driver) initialization() (err error) {
	addr8, err := d.ReadAddress()
	if err != nil {
		return fmt.Errorf("MLX90614::initialization() failed to communicate with slave device:\n\t%v", err)
	}

	addrExpected := d.GetAddressOrDefault(mlx90614DefaultSlaveAddress)
	if int(addr8) != addrExpected {
		return fmt.Errorf("MLX90614::initialization() SMBus slave address command returned mismatch, expected:%X actual:%X", addrExpected, int(addr8))
	}

	return nil
}

// Datasheet says "Read Word" is only supported SMBus read command, so ReadWordData()
func (d *MLX90614Driver) read(address byte) (uint16, error) {
	var err error
	var val16 uint16
	numRetries := 10

	for numRetries > 0 {
		val16, err = d.connection.ReadWordData(address)
		if err == nil {
			break
		}
		numRetries--
	}

	if err != nil {
		return 0x00, fmt.Errorf("MLX90614::read() error:\n\t%v", err)
	}

	return val16, err
}

// Datasheet says "Write Word" is only supported SMBus write command, so WriteWordData()
func (d *MLX90614Driver) write(address byte, data uint16) error {
	switch address {
	case mlx90614RegisterKE:
		fallthrough
	case 0x19:
		fallthrough
	case 0x0F:
		return errors.New("Refusing to write to protected register, see datasheet")
	}

	log.Printf("Writing: %X (%b) to %X (%b)\n", data, data, address, address)

	// For the Raspberry Pi platform, "repeated start" is automatically triggered by the i2c_bcm2835 module
	// else "combined transactions" should be enabled: https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=15840&start=25
	return d.connection.WriteWordData(address, data)
}

func (d *MLX90614Driver) writeEEPROM(address byte, data uint16) error {
	// "A write of 0x0000 must be done prior to writing in EEPROM in order to erase the
	// EEPROM cell content."
	if err := d.write(address, 0); err != nil {
		return fmt.Errorf("MLX90614::writeEEPROM() failed to erase cell content:\n\t%v", err)
	}

	time.Sleep(5 * time.Millisecond) // Delay tErase = 5ms

	if err := d.write(address, data); err != nil {
		return fmt.Errorf("MLX90614::writeEEPROM() failed to write new cell content:\n\t%v", err)
	}

	time.Sleep(5 * time.Millisecond) // Delay tWrite = 5ms

	return nil
}

func reset() {
	// 1. Wake up request
	// 2. SCL pin high
	// 3. SDA pin low for at least tDDQ > 33ms
}

func sleep() {
	// NOTE: In order to limit the current consumption to the typical 2.5μA Melexis
	// recommends that the SCL pin is kept low during sleep as there is leakage current
	// trough the internal synthesized zener diode connected to SCL pin. This may be
	// achieved by configuring the MD driver of SCL pin as Push-Pull and not having
	// Pull-Up resistor connected on SCL line.
}

func wake() {
	// After wake up the first data is available after 0.25 seconds (typical)
}

// ReadAddress returns the 7-bit SMBus slave address from EEPROM
func (d *MLX90614Driver) ReadAddress() (uint8, error) {
	val16, err := d.read(mlx90614RegisterAddress)
	if err != nil {
		return 0, fmt.Errorf("MLX90614::ReadAddress() failed:\n\t%v", err)
	}

	return uint8(val16), nil
}

// WriteAddress stores a new 7-bit SMBus slave address into EEPROM, reset required
func (d *MLX90614Driver) WriteAddress(newAddress uint8) error {
	// https://learn.adafruit.com/i2c-addresses/the-list
	if newAddress >= 0x78 || newAddress <= 0x07 {
		return errors.New("MLX90614::WriteAddress() invalid or reserved SMBus address")
	}

	return d.writeEEPROM(mlx90614RegisterAddress, uint16(newAddress))
}

// ReadID returns the Device ID registers into a slice of bytes
func (d *MLX90614Driver) ReadID() (uint64, error) {
	var id64 uint64

	// Construct ID word by word
	for i := byte(0); i < 4; i++ {
		id16, err := d.read(mlx90614RegisterID0 + i)
		if err != nil {
			return id64, fmt.Errorf("MLX90614::ReadID() failed on part %d:\n\t%v", i, err)
		}
		id64 |= uint64(id16) << (i * 16)
		log.Printf("ID%d:%X (%b)\n%b\n", i, id16, id16, id64)
	}

	return id64, nil
}

// ReadEmissivity returns the emissivity value from EEPROM, normalised between 0.1 and 1.0
func (d *MLX90614Driver) ReadEmissivity() (float64, error) {
	ke, err := d.read(mlx90614RegisterKE)
	if err != nil {
		return 0, fmt.Errorf("MLX90614::ReadEmissivity() failed:\n\t%v", err)
	}

	return (float64(ke) / 65535.0), nil
}

// ReadReg returns the raw reg data
func (d *MLX90614Driver) ReadReg(reg byte) (int16, error) {
	raw, err := d.read(reg)
	if err != nil {
		return 0, fmt.Errorf("MLX90614::ReadReg() failed:\n\t%v", err)
	}

	var rawSigned int16
	if (raw & 0x8000) != 0 {
		rawSigned = -(int16(raw) & 0x00FF)
	} else {
		rawSigned = int16(raw)
	}

	return rawSigned, nil
}

// ReadIRChannel1 returns the raw IR data register value
func (d *MLX90614Driver) ReadIRChannel1() (int16, error) {
	rawObj, err := d.read(mlx90614RegisterRaw1)
	if err != nil {
		return 0, fmt.Errorf("MLX90614::ReadIRChannel1() failed:\n\t%v", err)
	}
	// log.Printf("rawObj = %X", rawObj)

	var irSigned int16
	if (rawObj & 0x8000) != 0 {
		irSigned = -(int16(rawObj) & 0x00FF)
	} else {
		irSigned = int16(rawObj)
	}

	return irSigned, nil
}

// ReadTObj1Raw returns the raw TObj1 data
func (d *MLX90614Driver) ReadTObj1Raw() (uint16, error) {
	rawObj, err := d.read(mlx90614RegisterTObj1)
	if err != nil {
		return 0, fmt.Errorf("MLX90614::ReadTObj1Raw() failed:\n\t%v", err)
	}

	if (rawObj & 0x8000) != 0 {
		return 0, errors.New("MLX90614::ReadTObj1Raw() TObj1 error flag set")
	}

	return rawObj, nil
}

// ReadTObj2Raw returns the raw TObj2 data
func (d *MLX90614Driver) ReadTObj2Raw() (uint16, error) {
	rawObj, err := d.read(mlx90614RegisterTObj2)
	if err != nil {
		return 0, fmt.Errorf("MLX90614::ReadTObj2Raw() failed:\n\t%v", err)
	}

	if (rawObj & 0x8000) != 0 {
		return 0, errors.New("MLX90614::ReadTObj2Raw() TObj1 error flag set")
	}

	return rawObj, nil
}

// ReadAmbientTempRaw returns the raw TA data
func (d *MLX90614Driver) ReadAmbientTempRaw() (uint16, error) {
	rawAmb, err := d.read(mlx90614RegisterTA)
	if err != nil {
		return 0, fmt.Errorf("MLX90614::ReadAmbientTempRaw() failed:\n\t%v", err)
	}

	return rawAmb, nil
}

// ReadTOMinRaw returns the raw TO Min data from EEPROM
func (d *MLX90614Driver) ReadTOMinRaw() (uint16, error) {
	toMin, err := d.read(mlx90614RegisterTOMin)
	if err != nil {
		return 0, fmt.Errorf("MLX90614::ReadTOMinRaw() failed:\n\t%v", err)
	}

	return toMin, nil
}

// ReadTOMaxRaw returns the raw TO Max data from EEPROM
func (d *MLX90614Driver) ReadTOMaxRaw() (uint16, error) {
	toMax, err := d.read(mlx90614RegisterTOMax)
	if err != nil {
		return 0, fmt.Errorf("MLX90614::ReadTOMaxRaw() failed:\n\t%v", err)
	}

	return toMax, nil
}

// ConvertRawTempToDegK converts raw 16bit data to degrees kelvin
func (d *MLX90614Driver) ConvertRawTempToDegK(raw uint16) float64 {
	return float64(raw) * 0.02
}

// ConvertRawTempToDegC converts raw 16bit data to degrees celcius
func (d *MLX90614Driver) ConvertRawTempToDegC(raw uint16) float64 {
	return float64(int16(raw))*0.02 - 273.15
}

// ConvertRawTempToDegF converts raw 16bit data to degrees farenheit
func (d *MLX90614Driver) ConvertRawTempToDegF(raw uint16) float64 {
	return (float64(int16(raw))*0.02-273.15)*9/5 + 32
}

// ReadConfig returns the 16bit Config1 register from EEPROM
func (d *MLX90614Driver) ReadConfig() (uint16, error) {
	return d.read(mlx90614RegisterConfig)
}

// WriteFilterConfig stores the supplied FIR and IIR settings into EEPROM, returns
// the settling time in milliseconds.
func (d *MLX90614Driver) WriteFilterConfig(fir uint8, iir uint8) (float64, error) {
	// Guard against FIR 000...011
	if fir < 0x04 {
		return 0, errors.New("MLX90614::WriteFilterConfig() FIR out of range: must be at least 0x04")
	}

	// Check for 3bit overflow
	if fir > 0x07 || iir > 0x07 {
		return 0, errors.New("MLX90614::WriteFilterConfig() FIR or IIR out of range: each must be less than 0x08")
	}

	config, err := d.ReadConfig()
	if err != nil {
		return 0, fmt.Errorf("MLX90614::WriteFilterConfig() failed to read config:\n\t%v", err)
	}

	tSet := d.SettlingTimeFromConfig(config)

	// Overwrite FIR bits 10..8
	fir16 := uint16(fir) << 8
	config &= 0xF8FF
	config |= fir16

	// Overwrite IIR IIR bits 2..0
	iir16 := uint16(iir)
	config &= 0xFFF8
	config |= iir16

	// Store new filter settings
	err = d.writeEEPROM(mlx90614RegisterConfig, config)
	if err != nil {
		return tSet, fmt.Errorf("MLX90614::WriteFilterConfig() failed:\n\t%v", err)
	}

	return tSet, nil
}

// SettlingTimeFromConfig calculates the settling time in milliseconds
func (d *MLX90614Driver) SettlingTimeFromConfig(config uint16) float64 {
	// Single IR sensor
	dual := float64(0)
	if (config & 0x40) != 0 {
		// Dual IR sensor
		dual = float64(1)
	}

	fir := uint8(config & 0x700 >> 8)
	iir := uint8(config & 0x7)

	firSettingsMap := map[uint8]float64{
		0x04: 5.184,
		0x05: 9.28,
		0x06: 17.472,
		0x07: 33.856,
	}

	iirSettingsMap := map[uint8]float64{
		0x00: 10,
		0x01: 10,
		0x02: 10,
		0x03: 10,
		0x04: 1,
		0x05: 4,
		0x06: 8,
		0x07: 9,
	}

	firS := firSettingsMap[fir]
	iirS := iirSettingsMap[iir]

	// Settling time, in milliseconds. Formula from digital signal filters appnote.
	return 9.719 + (iirS * (firS + 5.26)) + (iirS * (firS + 12.542)) + (dual * iirS * (firS + 12.542))
}
