/** =========================================================================
 * @file ChangeParity.ino
 * @author Anthony Aufdenkampe
 * @copyright Stroud Water Research Center
 * This example is published under the BSD-3 license.
 *
 * @brief This sketch uses hardware serial to connect with GroPoint Profile and change
 * default modbus settings from 19200 8E1 to 9600 8N1.
 *
 * @note GroPoint Profile sensor default Modbus communication settings are 19200 Baud, 8
 * Bits, Even Parity, one Stop Bit (8-E-1). See p35 of manual. 8-N-1 (no parity) is the
 * most common configuration for serial communications.
 *
 * @warning Neither SoftwareSerial, AltSoftSerial, nor NeoSoftwareSerial will support
 * either even or odd parity!
 *
 * This sketch does not depend on the GropointModbus library, but only on the
 * SensorModbusMaster library and is based on it's example sketch:
 * SensorModbusMaster/examples/readWriteRegister/readWriteRegister.ino
 * ======================================================================= */

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <SensorModbusMaster.h>

// Turn on debugging outputs (i.e. raw Modbus requests & responsds)
// by uncommenting next line (i.e. `#define DEBUG`)
#define DEBUG

// ==========================================================================
//  Sensor Settings
// ==========================================================================

// Define the sensor's modbus address, or SlaveID
// NOTE: The GroPoint User Manual presents SlaveID and registers as decimal
// integers, whereas EnviroDIY and most other modbus systems present it in
// hexadecimal form. Use an online "HEX to DEC Converter".
byte defaultModbusAddress = 0x01;  // GroPoint ships sensors with a default ID of 0x01.
byte newModbusAddress     = 0x19;  // Hex 0x19 = Decimal 25 is unique in ModularSensors.

// The Modbus baud rate the sensor uses
int32_t defaultModbusBaud = 19200;  // GroPoint default baud rate is 19200.
int32_t newModbusBaud     = 9600;   // 9600 is a safer baud rate to use

// The Modbus parity the sensor uses
// see
// https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/
// for allowable configurations.
uint8_t defaultModbusParity = SERIAL_8E1;  // 8-E-1 is GroPoint default parity.
uint8_t newModbusParity     = SERIAL_8N1;  // 8-N-1 is the only allowable parity for
                                       // AltSoftSerial, NeoSWSerial, & SoftwareSerial

// Sensor Timing. Edit these to explore!
#define WARM_UP_TIME 350  // milliseconds for sensor to respond to commands.
// GroPoint Profile User Manual page 7:
// The time from application of power to the SDI-12 power bus until the
// sensor is ready to receive a command is approximately 350 ms.

#define STABILIZATION_TIME 100  // milliseconds for readings to stablize.

#define MEASUREMENT_TIME 200  // milliseconds to complete a measurement.
// GroPoint Profile User Manual page 39:
// Moisture measurements take approximately 200 ms per segment.
// Temperature measurements take approximately 200 ms per sensor.


// ==========================================================================
//  Data Logging Options
// ==========================================================================
const int32_t serialBaud = 115200;  // Baud rate for serial monitor

// Define pin number variables
const int sensorPwrPin  = 10;  // The pin sending power to the sensor
const int adapterPwrPin = 22;  // The pin sending power to the RS485 adapter
const int DEREPin       = -1;  // The pin controlling Recieve Enable & Driver Enable
                               // on the RS485 adapter, if applicable (else, -1)
                               // Setting HIGH enables the driver (arduino) to send
                               // Setting LOW enables the receiver (sensor) to send

// Construct software serial object for Modbus
// To access HardwareSerial on the Mayfly use a Grove to Male Jumpers cable
// or other set of jumpers to connect Grove D5 & D6 lines to the hardware
// serial TX1 (from D5) and RX1 (from D6) pins on the left 20-pin header.
// This is just a assigning another name to the same port, for convienence
HardwareSerial* modbusSerial = &Serial1;

// Construct a SensorModbusMaster class instance, from
// https://github.com/EnviroDIY/SensorModbusMaster
modbusMaster modbus;


// ==========================================================================
// Working Functions
// ==========================================================================
// A function for pretty-printing the Modbuss Address in Hexadecimal notation,
// from ModularSensors `sensorLocation()`
String prettyprintAddressHex(byte _modbusAddress) {
    String addressHex = F("0x");
    if (_modbusAddress < 0x10) addressHex += "0";
    addressHex += String(_modbusAddress, HEX);
    return addressHex;
}


// Get modbus slave ID or Sensor Modbus Address from
// holding register 40201, decimal offset 200 (hexadecimal 0x00C8)
byte getSensorAddress(void) {
    byte getAddressCommand[8] = {
        defaultModbusAddress, 0x03, 0x00, 0xC8, 0x00, 0x01, 0x0000
        //  address,                 ReadHolding, startRegister, numRegisters,  CRC
    };
    int16_t numRegisters = 1;

    // The size of the returned frame should be:
    // # Registers X 2 bytes/register + 5 bytes of modbus RTU frame

    // Try up to 5 times to get the right results
    int     tries    = 0;
    int16_t respSize = 0;
    while ((respSize != (numRegisters * 2 + 5) && tries < 5)) {
        // Send out the command (this adds the CRC)
        respSize = modbus.sendCommand(getAddressCommand, 8);
        tries++;

        delay(50);
    }
    if (respSize == (numRegisters * 2 + 5)) {
        return modbus.byteFromFrame(4);
    }  // 2nd byte in register
    else {
        return 0x00;
    }
}

// Set sensor modbus baud
// from holding register 40203, decimal offset 202 (hexadecimal 0x00CA).
// valid values: 0=19200, 1=9600, 2=4800, 3=2400, 4=1200, 5=600, 6=300.
bool setSensorBaud(byte newBaudCode) {
    byte dataToSend[2] = {0x00, newBaudCode};
    return modbus.setRegisters(0x00CA, 1, dataToSend, true);
}

// Set sensor modbus serial parity
// from holding register 40204, decimal offset 0203 (hexadecimal 0x00CB)
// Parity setting (0=none, 1=odd, 2=even)
bool setSensorParity(byte newParityCode) {
    byte dataToSend[2] = {0x00, newParityCode};
    return modbus.setRegisters(0x00CB, 1, dataToSend, true);
}


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
void setup() {
    // Setup power pins
    if (sensorPwrPin > 0) {
        pinMode(sensorPwrPin, OUTPUT);
        digitalWrite(sensorPwrPin, HIGH);
    }
    if (adapterPwrPin > 0) {
        pinMode(adapterPwrPin, OUTPUT);
        digitalWrite(adapterPwrPin, HIGH);
    }
    if (DEREPin > 0) { pinMode(DEREPin, OUTPUT); }

    // Turn on the "main" serial port for debugging via USB Serial Monitor
    Serial.begin(serialBaud);

    // Setup your modbus hardware serial port
    modbusSerial.begin(defaultModbusBaud, defaultModbusParity);

    // Setup the modbus instance
    modbus.begin(defaultModbusAddress, modbusSerial, DEREPin);

// Turn on debugging
#ifdef DEBUG
    modbus.setDebugStream(&Serial);
#endif

    // Start up note
    Serial.println("\nChange Parity utility for GroPoint Profile sensors ");

    // Allow the sensor and converter to warm up
    Serial.println("Waiting for sensor and adapter to be ready.");
    Serial.print("  Warm up time (ms): ");
    Serial.println(WARM_UP_TIME);
    Serial.println();
    delay(WARM_UP_TIME);

    // Confirm Modbus Address
    Serial.println("Default sensor modbus address:");
    Serial.print("  Decimal: ");
    Serial.print(defaultModbusAddress, DEC);
    Serial.print(", Hexidecimal: ");
    Serial.println(prettyprintAddressHex(defaultModbusAddress));
    Serial.println();

    // Read Sensor Modbus Address from holding register 40201 (0x9D09)
    Serial.println("Read sensor modbus address.");
    byte id = getSensorAddress();
    Serial.print("  Decimal: ");
    Serial.print(id, DEC);
    Serial.print(", Hexidecimal: ");
    Serial.println(prettyprintAddressHex(id));
    Serial.println();

    // Get Sensor Modbus Baud from holding register 40203,
    // decimal offset 202 (hexadecimal 0x00CA)
    Serial.println("Get sensor modbus baud setting.");
    Serial.println(
        "  Valid values: 0=19200, 1=9600, 2=4800, 3=2400, 4=1200, 5=600, 6=300.");
    int16_t sensorBaudCode = -9999;
    sensorBaudCode         = modbus.int16FromRegister(0x03, 0x00CA, bigEndian);
    Serial.print("    Baud Code: ");
    Serial.println(sensorBaudCode);
    Serial.println();

    // Get Sensor Modbus Parity from holding register 40204,
    // decimal offset 0203 (hexadecimal 0x00CB)
    Serial.println("Get sensor modbus parity setting.");
    int16_t sensorParity = -9999;
    sensorParity         = modbus.int16FromRegister(0x03, 0x00CB, bigEndian);
    Serial.print("    Parity: ");
    Serial.println(sensorParity);
    Serial.println();

    // Delay to give time to restart with serial monitor turned on.
    delay(10000);

    // Set sensor modbus baud
    Serial.print("Set sensor modbus baud to ");
    Serial.println(newModbusBaud);
    // valid values: 0=19200, 1=9600, 2=4800, 3=2400, 4=1200, 5=600, 6=300.
    Serial.println(setSensorBaud(0x01));
    Serial.println();


    // Set sensor modbus parity
    Serial.println("Set sensor modbus baud.");
    // Parity setting (0=none, 1=odd, 2=even)
    Serial.println(setSensorParity(0x00));
    Serial.println();
}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop() {
    // Get data values from read-only input registers (0x04)

    // // Segment 1 Moisture
    // int16_t M1 = -9999;
    // M1 = modbus.int16FromRegister(0x04, 0x0000, bigEndian);
    // float valueM1 = 0.1 * M1;
    // Serial.print("Segment 1 Moisture: ");
    // Serial.print(valueM1, 1);
    // Serial.println(" % volumetric soil moisture");
    // Serial.println();

    // // Temperature Sensor 1
    // int16_t T1 = -9999;
    // T1 = modbus.int16FromRegister(0x04, 0x0064, bigEndian);
    // float valueT1 = 0.1 * T1;
    // Serial.print("Temperature Sensor 1: ");
    // Serial.print(valueT1, 1);
    // Serial.println(" degrees C");
    // Serial.println();

    // int16_t startRegister = 0x0000; // Moisture
    int16_t startRegister = 0x0064;  // Temperature
    int16_t numRegisters  = 13;
    // 1 registers had 1 "no response" (3 requests)
    // 2 registers had 1 "no response"
    // 3 registers had 4 "no response" (6 requests)
    // 4 registers had 4 "no response"
    // 6 registers had 4 "no response"
    // 7 registers had 4 "no response"
    // 8 registers had 4 "no response"
    // 13 registers had 4 "no response"
    Serial.print("Requested Registers: ");
    Serial.println(numRegisters);
    modbus.getRegisters(0x04, startRegister, numRegisters);
    Serial.println();

    // Delay between readings
    delay(MEASUREMENT_TIME);
}
