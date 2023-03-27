/*****************************************************************************
ChangeModbusSettings.ino

This scetch uses hardware serial to connect with GroPoint Profile and 
change default modbus settings from 19200 8E1 to 9600 8N1.

NOTE: GroPoint Profile sensor default Modbus communication settings are 
19200 Baud, 8 Bits, Even Parity, one Stop Bit (8-E-1). See p35 of manual.
8-N-1 (no parity) is the most common configuration for serial communications.

NOTE:  that neither SoftwareSerial, AltSoftSerial, nor NeoSoftwareSerial
will support either even or odd parity!

This sketch depends on the GropointModbus library and also loosly on the
SensorModbusMaster library via it's example sketch:
SensorModbusMaster/examples/readWriteRegister/readWriteRegister.ino
*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <GroPointModbus.h>

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
int32_t newModbusBaud     = 9600;  // 9600, to match the default for YosemiTech & Keller.


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
const int sensorPwrPin = 10;  // The pin sending power to the sensor
const int adapterPwrPin = 22; // The pin sending power to the RS485 adapter
const int DEREPin = -1; // The pin controlling Recieve Enable & Driver Enable
                        // on the RS485 adapter, if applicable (else, -1)
                        // Setting HIGH enables the driver (arduino) to send
                        // Setting LOW enables the receiver (sensor) to send

// Construct software serial object for Modbus
    // To access HardwareSerial on the Mayfly use a Grove to Male Jumpers cable
    // or other set of jumpers to connect Grove D5 & D6 lines to the hardware
    // serial TX1 (from D5) and RX1 (from D6) pins on the left 20-pin header.
    HardwareSerial& modbusSerial = Serial1;

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
    if (DEREPin > 0) {
        pinMode(DEREPin, OUTPUT);
    }

    // Turn on the "main" serial port for debugging via USB Serial Monitor
    Serial.begin(serialBaud);

    // Turn on your modbus serial port
    // modbusSerial.begin(modbusBaudRate, SERIAL_8O1);
    // ^^ use this for 8 data bits - odd parity - 1 stop bit
    modbusSerial.begin(defaultModbusBaud, SERIAL_8E1);
    // ^^ use this for 8 data bits - even parity - 1 stop bit
    // modbusSerial.begin(modbusBaudRate, SERIAL_8N2);
    // ^^ use this for 8 data bits - no parity - 2 stop bits
    // modbusSerial.begin(modbusBaudRate);
    // ^^ use this for 8 data bits - no parity - 1 stop bits
    // Despite being technically "non-compliant" with the modbus specifications
    // 8N1 parity is very common.

    // Setup the sensor instance
    sensor.begin(model, modbusAddress, &modbusSerial, DEREPin);

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
    Serial.print("  integer: ");
    Serial.print(defaultModbusAddress, DEC);
    Serial.print(", hexidecimal: ");
    Serial.println(prettyprintAddressHex(defaultModbusAddress));
    Serial.println();

    // Read Sensor Modbus Address from holding register 40201 (0x9D09)
    Serial.println("Read sensor modbus address.");
    byte id = sensor.getSensorAddress();
    Serial.print("  integer: ");
    Serial.print(id, DEC);
    Serial.print(", hexidecimal: ");
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
    Serial.println("  Additional 10s delay time to turn on serial monitor");
    delay(10000);


    // Set sensor modbus baud
    Serial.print("Set sensor modbus baud to ");
    Serial.println(newModbusBaud);
    if (sensor.setSensorBaud(newModbusBaud)) {
        Serial.println("  Success! New baud will take effect once sensor is power cycled.");
    } else {
        Serial.println("  Failed!");
    }
    Serial.println();



    // Set sensor modbus parity
    Serial.println("Set sensor modbus baud to ");
    // Parity setting (0=none, 1=odd, 2=even)
    Serial.println(setSensorParity(0x00));
    Serial.println();


}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop()
{
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
    int16_t startRegister = 0x0064; // Temperature
    int16_t numRegisters = 13;
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
    getInputRegisters(startRegister, numRegisters);
    Serial.println();

    // Delay between readings
    delay(MEASUREMENT_TIME);
}