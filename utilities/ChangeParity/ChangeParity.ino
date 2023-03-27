/*****************************************************************************
ChangeParity.ino

This scetch uses hardware serial to connect with GroPoint Profile and 
change the parity to "none".

NOTE: GroPoint Profile sensor default Modbus communication settings are 
19200 Baud, 8 Bits, Even Parity, one Stop Bit (8-E-1). See p35 of manual.
8-N-1 (no parity) is the most common configuration for serial communications.

NOTE:  that neither SoftwareSerial, AltSoftSerial, nor NeoSoftwareSerial
will support either even or odd parity!

This sketch does not depend on the GropointModbus library, but only on the 
SensorModbusMaster library and is based on it's example sketch:
SensorModbusMaster/examples/readWriteRegister/readWriteRegister.ino
*****************************************************************************/

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
// NOTE: The GroPoint User Manual presents SlaveID and registers as integers (decimal),
// whereas EnviroDIY and most other modbus systems present it in hexadecimal form.
// Use an online "HEX to DEC Converter".
byte modbusAddress = 0x01;  // GroPoint ships sensors with a default ID of 0x01.
// The Modbus baud rate the sensor uses
const int32_t modbusBaudRate = 19200;  // GroPoint default baud is 19200.

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
const int DEREPin = -1;       // The pin controlling Recieve Enable and Driver Enable
                              // on the RS485 adapter, if applicable (else, -1)
                              // Setting HIGH enables the driver (arduino) to send text
                              // Setting LOW enables the receiver (sensor) to send text

// Construct software serial object for Modbus
    // To access HardwareSerial on the Mayfly, use jumpers to connect 
    // the Modbus adapter pins that would go to D5 & D6 to 
    // TX1 and RX1 respectivley on the left 20-pin header

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
byte getSlaveID(void) {
    byte getAddressCommand[8] = {
        0x01,    0x03,        0x00, 0xC8,    0x00, 0x01,    0x0000
    //  address, ReadHolding, startRegister, numRegisters,  CRC
    };                   
    int16_t numRegisters = 1;
    
    // The size of the returned frame should be:
    // # Registers X 2 bytes/register + 5 bytes of modbus RTU frame

    // Try up to 5 times to get the right results
    int tries = 0;
    int16_t respSize = 0;
    while ((respSize != (numRegisters*2 + 5) && tries < 5))
    {
        // Send out the command (this adds the CRC)
        respSize = modbus.sendCommand(getAddressCommand, 8);
        tries++;

        delay(50);
    }
    if (respSize == (numRegisters*2 + 5))
        return modbus.byteFromFrame(4); // 2nd byte in register
    else return 0x00;
}

// Get sensor information
// Page 35 of GroPoint Profile User Manual says:
    // Function 17 (0x11) returns the ASCII encoded string 
    // ‘RIOTTECHGPLPTC vvvSNnnnnnn’ where 
    // vvv is the firmware version (v.v.v) and 
    // nnnnnn is the probe serial number.
String getSensorInfo(void) {
    byte command[4] = {
        0x01,    0x11,     0x00, 0x00
    //  address, function, CRC
    };
    int respSize = modbus.sendCommand(command, 4);

    return modbus.StringFromFrame(respSize, 5);
}

// This restarts communications, 
// using the modbus diagnostic command 08 (0x08) with subfunction 01.
// A request data field contents of FF 00 hex causes the port’s Communications 
// Event Log to be cleared also. Contents of 00 00 leave the log as it was 
// prior to the restart.
bool restartCommunications(void) {
    byte command[8] = {
        modbusAddress, 0x08,     0x00, 0x01,  0x00, 0x00, 0x00, 0x00
    //  address,  function, subfunction, data field, CRC
    };
    int16_t respSize = 0;
    respSize = modbus.sendCommand(command, 8);
    if (respSize == 8)
        return true;
    else return false;
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


// Get input registers with GroPoint measurements
// Based off modbusMaster::getRegisters(), 
// but with trigger followed by a 200 ms delay before read request
// Page 39 of GroPoint Profile User Manual says:
//   A new measurement is triggered by a read request to the input registers 
//   (either read single or read multiple). The first read command will initiate 
//   the start of a measurement (soil moisture or soil temperature) and will 
//   respond to the master with an ACKNOWLEDGE (05) exception response. 
//   Moisture measurements take approximately 200 ms per segment. 
//   Temperature measurements take approximately 200 ms per sensor. 
//   The master should wait for this amount of time to expire before attempting 
//   to retrieve the measurement values with another read command.
bool getInputRegisters(int16_t startRegister, int16_t numRegisters) {
    byte readCommand = 0x04;  // For an input register = 0x04

    // Create an array for the command
    byte command[8];

    // Put in the slave id and the command
    command[0] = modbusAddress;
    command[1] = readCommand;

    // Put in the starting register
    leFrame fram = {{0,}};
    fram.Int16[0] = startRegister;
    command[2] = fram.Byte[1];
    command[3] = fram.Byte[0];

    // Put in the number of registers
    fram.Int16[1] = numRegisters;
    command[4] = fram.Byte[3];
    command[5] = fram.Byte[2];

    // The size of the returned frame should be:
    // # Registers X 2 bytes/register + 5 bytes of modbus RTU frame

    // Try up to 5 times to get the right results
    int tries = 0;
    int16_t respSize = 0;
    while ((respSize != (numRegisters*2 + 5) && tries < 20)) {
        // Send out the command (this adds the CRC)
        respSize = modbus.sendCommand(command, 8);
        tries++;
        delay(25);  // 
    }

    if (respSize == (numRegisters*2 + 5))
        return true;
    else return false;
}



// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
void setup() {
    if (sensorPwrPin > 0)    {
        pinMode(sensorPwrPin, OUTPUT);
        digitalWrite(sensorPwrPin, HIGH);
    }
    if (adapterPwrPin > 0)    {
        pinMode(adapterPwrPin, OUTPUT);
        digitalWrite(adapterPwrPin, HIGH);
    }
    if (DEREPin > 0)    {
        pinMode(DEREPin, OUTPUT);
    }

    // Turn on the "main" serial port for debugging via USB Serial Monitor
    Serial.begin(serialBaud);

    // Turn on your modbus serial port
    // Serial1.begin(modbusBaudRate, SERIAL_8O1);
    // ^^ use this for 8 data bits - odd parity - 1 stop bit
    Serial1.begin(modbusBaudRate, SERIAL_8E1);
    // ^^ use this for 8 data bits - even parity - 1 stop bit
    // Serial1.begin(modbusBaudRate, SERIAL_8N2);
    // ^^ use this for 8 data bits - no parity - 2 stop bits
    // Serial1.begin(modbusBaudRate);
    // ^^ use this for 8 data bits - no parity - 1 stop bits
    // Despite being technically "non-compliant" with the modbus specifications
    // 8N1 parity is very common.

    // Setup the modbus instance
    modbus.begin(modbusAddress, modbusSerial, DEREPin);

    // Turn on debugging
    #ifdef DEBUG
        modbus.setDebugStream(&Serial);
    #endif

    // Start up note
    Serial.println("\nChange Parity utility for GroPoint Profile sensors ");

    // Allow the sensor and converter to warm up
    Serial.println("Waiting for sensor and adapter to be ready.");
    Serial.print("    Warm up time (ms): ");
    Serial.println(WARM_UP_TIME);
    Serial.println();
    delay(WARM_UP_TIME);

    // Confirm Modbus Address 
    Serial.println("Selected modbus address:");
    Serial.print("    integer: ");
    Serial.print(modbusAddress, DEC);
    Serial.print(", hexidecimal: ");
    Serial.println(prettyprintAddressHex(modbusAddress));
    Serial.println();

    // Read Sensor Modbus Address from holding register 40201 (0x9D09)
    Serial.println("Get sensor modbus address.");
    byte id = getSlaveID();
    Serial.print("    integer: ");
    Serial.print(id, DEC);
    Serial.print(", hexidecimal: ");
    Serial.println(prettyprintAddressHex(id));
    Serial.println();

    // Get Sensor Information
    Serial.println("Get sensor information.");
    Serial.println(getSensorInfo());
    Serial.println();

    // Get Sensor Modbus Baud from holding register 40203, 
    // decimal offset 202 (hexadecimal 0x00CA)
    Serial.println("Get sensor modbus baud setting.");
    int16_t sensorBaud = -9999;
    sensorBaud = modbus.int16FromRegister(0x03, 0x00CA, bigEndian);
    Serial.print("    Baud: ");
    Serial.println(sensorBaud);
    Serial.println();

    // Get Sensor Modbus Parity from holding register 40204, 
    // decimal offset 0203 (hexadecimal 0x00CB)
    Serial.println("Get sensor modbus parity setting.");
    int16_t sensorParity = -9999;
    sensorParity = modbus.int16FromRegister(0x03, 0x00CB, bigEndian);
    Serial.print("    Parity: ");
    Serial.println(sensorParity);
    Serial.println();

    // Reset communication
    Serial.println("Resetting sensor communications.");
    if (restartCommunications()) {
        Serial.println("    Communications reset succeeded.\n");
    } else {
        Serial.println("    Communications reset failed.\n");
    }

    delay(10000);

    // Set sensor modbus baud
    Serial.println("Set sensor modbus baud.");
    // valid values: 0=19200, 1=9600, 2=4800, 3=2400, 4=1200, 5=600, 6=300. 
    Serial.println(setSensorBaud(0x01));

    // Set sensor parity
    Serial.println("Set sensor modbus baud.");
    // Parity setting (0=none, 1=odd, 2=even)
    Serial.println(setSensorParity(0x00));


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