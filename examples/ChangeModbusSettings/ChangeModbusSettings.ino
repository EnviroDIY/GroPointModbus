/** =========================================================================
 * @file ChangeModbusSettings.ino
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
 * This sketch depends on the GropointModbus library and also loosly on the
 * SensorModbusMaster library via it's example sketch:
 * SensorModbusMaster/examples/readWriteRegister/readWriteRegister.ino
 * ======================================================================= */

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <GroPointModbus.h>

// Turn on debugging outputs (i.e. raw Modbus requests & responsds)
// by uncommenting next line (i.e. `#define DEBUG`)
// #define DEBUG

// ==========================================================================
//  Sensor Settings
// ==========================================================================
// Define the sensor type
gropointModel model = GPLP8;  // The sensor model number

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

// Construct the gropoint sensor instance
gropoint sensor;
bool     success;

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

    // Setup the sensor instance
    sensor.begin(model, defaultModbusAddress, &modbusSerial, DEREPin);

// Turn on debugging
#ifdef DEBUG
    sensor.setDebugStream(&Serial);
#endif

    // Start up note
    Serial.println("\nChange Modbus Settings for GroPoint Profile sensors ");

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
    byte id = sensor.getSensorAddress();
    Serial.print("  Decimal: ");
    Serial.print(id, DEC);
    Serial.print(", Hexidecimal: ");
    Serial.println(prettyprintAddressHex(id));
    Serial.println();

    // Get Sensor Modbus Baud
    Serial.println("Get sensor modbus baud setting.");
    int16_t sensorBaud = sensor.getSensorBaud();
    Serial.print("  Baud: ");
    Serial.println(sensorBaud);
    Serial.println("  Change is effective immediately, so change last.");
    Serial.println();

    // Get Sensor Modbus Parity
    Serial.println("Get sensor modbus parity setting.");
    String sensorParity = sensor.getSensorParity();
    Serial.print("    Parity: ");
    Serial.println(sensorParity);
    Serial.println();

    // Delay to give time to restart with serial monitor turned on.
    Serial.println("Resetting Modbus Serial Settings.");
    Serial.println("  Additional 5s delay time to turn on serial monitor.\n");
    delay(5000);

    // Set sensor modbus baud
    Serial.print("Set sensor modbus baud to ");
    Serial.println(newModbusBaud);
    success = sensor.setSensorBaud(newModbusBaud);
    if (success) {
        Serial.println(
            "  Success! New baud will take effect once sensor is power cycled.");
    } else
        Serial.println("  Baud reset failed!");
    Serial.println();

    // Set sensor modbus parity
    Serial.print("Set sensor modbus parity to ");
    Serial.println(newModbusParity);
    success = sensor.setSensorParity(newModbusParity);
    if (success) {
        Serial.println(
            "  Success! New parity will take effect once sensor is power cycled.");
    } else
        Serial.println("  Parity reset failed!");
    Serial.println();

    // Set sensor modbus address
    Serial.print("Set sensor modbus address to ");
    Serial.println(prettyprintAddressHex(newModbusAddress));
    success = sensor.setSensorAddress(newModbusAddress);
    if (success)
        Serial.println("  Success! New modbus address will take effect immediately.");
    else
        Serial.println("  Address reset failed!");
    Serial.println("\n");

    Serial.println("Upload a sketch with the new Modbus settings ");
    Serial.println("to reestablish communications with the sensor.");
}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop() {
    // Empty loop function
}
