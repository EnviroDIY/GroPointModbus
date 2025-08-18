/**
 * @file GroPointModbus.h
 * Part of the EnviroDIY GroPointModbus library for Arduino.
 * @license This library is published under the BSD-3 license.
 * @author Anthony Aufdenkampe
 *
 * @brief Contains the GroPointModbus class definitions.
 *
 * Tested with GPLP-8
 */

#include "GroPointModbus.h"


//----------------------------------------------------------------------------
//                          PUBLIC SENSOR FUNCTIONS
//----------------------------------------------------------------------------


// This function sets up the communication
// It should be run during the arduino "setup" function.
// The "stream" device must be initialized and begun prior to running this.
bool gropoint::begin(gropointModel model, byte modbusSlaveID, Stream* stream,
                     int enablePin) {
    // Give values to variables;
    _model   = model;
    _slaveID = modbusSlaveID;
    // Start up the modbus instance
    bool success = modbus.begin(modbusSlaveID, stream, enablePin);
    // Get the model type from the serial number if it's not known
    // if (_model == UNKNOWN) getSerialNumber();

    return success;
}
bool gropoint::begin(gropointModel model, byte modbusSlaveID, Stream& stream,
                     int enablePin) {
    return begin(model, modbusSlaveID, &stream, enablePin);
}


// This returns a pretty string with the model information
String gropoint::getModel(void) {
    switch (_model) {
        case GPLP2: return "GPLP-2";
        case GPLP3: return "GPLP-3";
        case GPLP4: return "GPLP-4";
        case GPLP5: return "GPLP-5";
        case GPLP6: return "GPLP-6";
        case GPLP8: return "GPLP-8";
        default: return "Unknown";
    }
}


// This returns a pretty string with the parameter measured.
String gropoint::getParameter(void) {
    switch (_model) {
        case GPLP8: return "M1,  M2,  M3,  M4,  M5,  M6,  M7,  M8";
        default: return "Unknown";
    }
}

// This returns a pretty string with the 2nd parameter measured.
String gropoint::getParameter1(void) {
    switch (_model) {
        case GPLP8: {
            return "T1,  T2,  T3,  T4,  T5,  T6,  T7,  T8,  T9, T10, T11, T12, T13";
        }
        default: return "Unknown";
    }
}


// This returns a pretty string with the measurement units for Parameter.
String gropoint::getUnits(void) {
    switch (_model) {
        case GPLP8: return "%,   %,   %,   %,   %,   %,   %,   %";
        default: return "Unknown";
    }
}


// This returns a pretty string with the measurement units for Parameter1.
String gropoint::getUnits1(void) {
    switch (_model) {
        case GPLP8: {
            return "°C,  °C,  °C,  °C,  °C,  °C,  °C,  °C,  °C,  °C,  °C,  °C,  °C";
        }
        default: return "Unknown";
    }
}

// This gets the modbus slave ID or Sensor Modbus Address.
// from holding register 40201, decimal offset 200 (hexadecimal 0x00C8)
// Does not seem to work with a broadcast address of 0x00 or 0xFF
byte gropoint::getSensorAddress(void) {
    // Based on internals of modbusMaster::getRegisters(), rather than using it
    // to enable testing with broadcast addresses
    byte getAddressCommand[8] = {_slaveID, 0x03, 0x00,  0xC8,
                                 0x00,     0x01, 0x0000

    };
    //  ^^ address, ReadHolding, startRegister, numRegisters,  CRC
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
        // 2nd byte in register
        return modbus.byteFromFrame(4);
    } else {
        return 0x00;
    }
}


// This sets a new modbus slave ID or Sensor Modbus Address.
// From holding register 40201, decimal offset 200 (hexadecimal 0x00C8)
bool gropoint::setSensorAddress(byte newSensorAddress) {
    // 8-bit data values are stored in the lower 8 bits of the 16-bit registers.
    byte dataToSend[2] = {0x00, newSensorAddress};
    return modbus.setRegisters(0x00C8, 1, dataToSend, true);
}

// Get sensor modbus baud
// from holding register 40203, decimal offset 202 (hexadecimal 0x00CA)
int16_t gropoint::getSensorBaud(void) {
    int16_t sensorBaudCode = -9999;
    int32_t sensorBaud     = -9999;
    sensorBaudCode         = modbus.int16FromRegister(0x03, 0x00CA, bigEndian);
    // valid values: 0=19200, 1=9600, 2=4800, 3=2400, 4=1200, 5=600, 6=300.
    if (sensorBaudCode == 0) {
        sensorBaud = 19200;
    } else if (sensorBaudCode == 1) {
        sensorBaud = 9600;
    } else if (sensorBaudCode == 2) {
        sensorBaud = 4800;
    } else if (sensorBaudCode == 3) {
        sensorBaud = 2400;
    } else if (sensorBaudCode == 4) {
        sensorBaud = 1200;
    } else if (sensorBaudCode == 5) {
        sensorBaud = 600;
    } else if (sensorBaudCode == 6) {
        sensorBaud = 300;
    } else {
        Serial.println(F("Error"));
    }
    return sensorBaud;
}

// Set sensor modbus baud
// from holding register 40203, decimal offset 202 (hexadecimal 0x00CA).
bool gropoint::setSensorBaud(int32_t newSensorBaud) {
    byte newSensorBaudCode = 0xFF;  // Error code
    // Convert valid values to codes:
    // 0=19200, 1=9600, 2=4800, 3=2400, 4=1200, 5=600, 6=300.
    switch (newSensorBaud) {
        case 19200: newSensorBaudCode = 0x00; break;
        case 9600: newSensorBaudCode = 0x01; break;
        case 4800: newSensorBaudCode = 0x02; break;
        case 2400: newSensorBaudCode = 0x03; break;
        case 1200: newSensorBaudCode = 0x04; break;
        case 600: newSensorBaudCode = 0x05; break;
        case 300: newSensorBaudCode = 0x06; break;
        default: Serial.println(F("  Input value not valid!"));
    }
    byte dataToSend[2] = {0x00, newSensorBaudCode};
    return modbus.setRegisters(0x00CA, 1, dataToSend, true);
}


// Get sensor modbus serial parity
// from holding register 40204, decimal offset 0203 (hexadecimal 0x00CB)
String gropoint::getSensorParity(void) {
    int16_t sensorParityCode = -9999;
    String  sensorParity     = "error";
    sensorParityCode         = modbus.int16FromRegister(0x03, 0x00CB, bigEndian);
    // valid values: 0=None, 1=Odd, 2=Even
    if (sensorParityCode == 0) {
        sensorParity = "None";
    } else if (sensorParityCode == 1) {
        sensorParity = "Odd";
    } else if (sensorParityCode == 2) {
        sensorParity = "Even";
    } else {
        sensorParity = "Error";
    }
    return sensorParity;
}

// Set sensor modbus serial parity
// from holding register 40204, decimal offset 0203 (hexadecimal 0x00CB)
bool gropoint::setSensorParity(String newSensorParity) {
    byte newSensorParityCode = 0xFF;
    // Convert valid values to codes: 0=None, 1=Odd, 2=Even
    if (newSensorParity == "None") {
        newSensorParityCode = 0;
    } else if (newSensorParity == "Odd") {
        newSensorParityCode = 1;
    } else if (newSensorParity == "Even") {
        newSensorParityCode = 2;
    } else {
        Serial.println(F("  Input value not valid!"));
    }
    byte dataToSend[2] = {0x00, newSensorParityCode};
    return modbus.setRegisters(0x00CB, 1, dataToSend, true);
}


// Gets sensor information as a String
// Page 35 of GroPoint Profile User Manual says:
// Function 17 (0x11) returns the ASCII encoded string
// ‘RIOTTECHGPLPTC vvvSNnnnnnn’ where
// vvv is the firmware version (v.v.v) and
// nnnnnn is the probe serial number.
String gropoint::getSensorInfo(void) {
    byte command[4] = {
        _slaveID, 0x11, 0x00, 0x00
        //  address,  function, CRC
    };
    int16_t respSize = 0;
    respSize         = modbus.sendCommand(command, 4);

    return modbus.StringFromFrame(respSize, 5);
}

// This gets the instrument serial number as a String
// Not fully tested
String gropoint::getSerialNumber(void) {
    String SN;
    SN = getSensorInfo();
    // Select the Serial Number
    SN = SN.substring(17, 25);

    // Return the serial number
    return SN;
}


// This gets the hardware and software version of the sensor
// Not fully tested
bool gropoint::getVersion(String& hardwareVersion, String& softwareVersion) {
    String info = getSensorInfo();

    hardwareVersion = info.substring(8, 14);
    softwareVersion = info.substring(14, 17);
    return true;
}


// This restarts communications,
// using the modbus diagnostic command 08 (0x08) with subfunction 01.
// A request data field contents of FF 00 hex causes the port’s Communications
// Event Log to be cleared also. Contents of 00 00 leave the log as it was
// prior to the restart.
bool gropoint::restartCommunications(void) {
    byte command[8] = {
        _slaveID, 0x08, 0x00, 0x01, 0x00,
        0x00,     0x00, 0x00
        //  address,  function, subfunction, data field, CRC
    };
    int16_t respSize = 0;
    respSize         = modbus.sendCommand(command, 8);
    if (respSize == 8) {
        return true;
    } else {
        return false;
    }
}


// This tells the sensors to begin taking measurements
// Page 39 of GroPoint Profile User Manual says:
//   A new measurement is triggered by a read request to the input registers
//   (either read single or read multiple). The first read command will initiate
//   the start of a measurement (soil moisture or soil temperature) and will
//   respond to the master with an ACKNOWLEDGE (05) exception response.
//   Moisture measurements take approximately 200 ms per segment.
//   Temperature measurements take approximately 200 ms per sensor.
//   The master should wait for this amount of time to expire before attempting
//   to retrieve the measurement values with another read command.
// Starting at input register 0x7531 (30001)
bool gropoint::startMeasurement(void) {
    // A start command is not in their Modbus Manuals.
    // However, Start/Stop functions are required to get these to work in
    // ModularSensors.
    // Instead, just confirm that sensor in communicating with a getSensorAddress()
    // command
    if (getSensorAddress() != 0x00) {
        return true;
    } else {
        return false;
    }
}


// This tells the sensors to stop taking measurements
bool gropoint::stopMeasurement(void) {
    // A stop command is not in their Modbus Manuals.
    // However, Start/Stop functions are required to get these to work in
    // ModularSensors.
    return true;
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
bool gropoint::getInputRegisters(int16_t startRegister, int16_t numRegisters) {
    byte readCommand = 0x04;  // For an input register = 0x04

    // Create an array for the command
    byte command[8];

    // Put in the slave id and the command
    command[0] = _slaveID;
    command[1] = readCommand;

    // Put in the starting register
    leFrame fram  = {{
        0,
    }};
    fram.Int16[0] = startRegister;
    command[2]    = fram.Byte[1];
    command[3]    = fram.Byte[0];

    // Put in the number of registers
    fram.Int16[1] = numRegisters;
    command[4]    = fram.Byte[3];
    command[5]    = fram.Byte[2];

    // The size of the returned frame should be:
    // # Registers X 2 bytes/register + 5 bytes of modbus RTU frame

    // Try up to 5 times to get the right results
    int     tries    = 0;
    int16_t respSize = 0;
    while ((respSize != (numRegisters * 2 + 5) && tries < 15)) {
        // Send out the command (this adds the CRC)
        respSize = modbus.sendCommand(command, 8);
        tries++;
        // Manual says measurements take ~200 ms per segment/sensor after trigger
        // request
        delay(160);  // 160 ms gives results fastest by trial and error
                     // Requires 5 requests (3 no responses) to get a result in ~5 sec.
                     // Shorter delays give 4 no responses but do not shorten result.
                     // Longer delays reduce no responses (2080 required to get none),
                     // but take much longer to get an answer.
    }
    if (respSize == (numRegisters * 2 + 5)) {
        return true;
    } else {
        return false;
    }
}

// This gets soil moisture values back from the sensor
// Page 37-38 of GroPoint Profile User Manual says:
//   The input registers are read-only registers containing soil moisture or
//   temperature measurement values. Moisture and temperature readings are stored
//   as 16-bit signed integers and will be returned without a decimal place. In
//   order to obtain the true result, the decimal place setting must be applied.
//   The default setting for this is 1 decimal place, so for example a decimal
//   value of 123 indicates 12.3 % volumetric moisture content for volumetric soil
//   moisture measurements, or 12.3 °C for the soil temperature sensor measurements.
// Starting at input register 30001, decimal offset 0 (hexadecimal 0x0000)
bool gropoint::getValues(float& valueM1, float& valueM2, float& valueM3, float& valueM4,
                         float& valueM5, float& valueM6, float& valueM7,
                         float& valueM8) {
    // Set values to -9999 before asking for the result
    valueM1 = valueM2 = valueM3 = valueM4 = -9999;
    valueM5 = valueM6 = valueM7 = valueM8 = -9999;

    int16_t startRegister = 0x0000;

    switch (_model) {
        case GPLP8: {
            int16_t numRegisters = 8;
            if (getInputRegisters(startRegister, numRegisters)) {
                // Multiply by 0.1 to convert from int to float as per manual
                valueM1 = 0.1 * modbus.int16FromFrame(bigEndian, 3);
                valueM2 = 0.1 * modbus.int16FromFrame(bigEndian, 5);
                valueM3 = 0.1 * modbus.int16FromFrame(bigEndian, 7);
                valueM4 = 0.1 * modbus.int16FromFrame(bigEndian, 9);
                valueM5 = 0.1 * modbus.int16FromFrame(bigEndian, 11);
                valueM6 = 0.1 * modbus.int16FromFrame(bigEndian, 13);
                valueM7 = 0.1 * modbus.int16FromFrame(bigEndian, 15);
                valueM8 = 0.1 * modbus.int16FromFrame(bigEndian, 17);
                return true;
            }
            break;
        }
        default: {
            Serial.println(F("Other sensors not yet implemented."));
            break;
        }
    }
    // If something fails, we'll get here
    return false;
}

// This gets soil temperature values back from the sensor
// Page 37-38 of GroPoint Profile User Manual says:
//   The input registers are read-only registers containing soil moisture or
//   temperature measurement values. Moisture and temperature readings are stored
//   as 16-bit signed integers and will be returned without a decimal place. In
//   order to obtain the true result, the decimal place setting must be applied.
//   The default setting for this is 1 decimal place, so for example a decimal
//   value of 123 indicates 12.3 % volumetric moisture content for volumetric soil
//   moisture measurements, or 12.3 °C for the soil temperature sensor measurements.
// Starting at input register 30101, decimal offset 100 (hexadecimal 0x0064)
bool gropoint::getTemperatureValues(float& valueT1, float& valueT2, float& valueT3,
                                    float& valueT4, float& valueT5, float& valueT6,
                                    float& valueT7, float& valueT8, float& valueT9,
                                    float& valueT10, float& valueT11, float& valueT12,
                                    float& valueT13) {
    // Set values to -9999 before asking for the result
    valueT1 = valueT2 = valueT3 = valueT4 = valueT5 = valueT6 = valueT7 = -9999;
    valueT8 = valueT9 = valueT10 = valueT11 = valueT12 = valueT13 = -9999;

    int16_t startRegister = 0x0064;

    switch (_model) {
        case GPLP8: {
            int16_t numRegisters = 13;
            if (getInputRegisters(startRegister, numRegisters)) {
                // Multiply by 0.1 to convert from int to float as per manual
                valueT1  = 0.1 * modbus.int16FromFrame(bigEndian, 3);
                valueT2  = 0.1 * modbus.int16FromFrame(bigEndian, 5);
                valueT3  = 0.1 * modbus.int16FromFrame(bigEndian, 7);
                valueT4  = 0.1 * modbus.int16FromFrame(bigEndian, 9);
                valueT5  = 0.1 * modbus.int16FromFrame(bigEndian, 11);
                valueT6  = 0.1 * modbus.int16FromFrame(bigEndian, 13);
                valueT7  = 0.1 * modbus.int16FromFrame(bigEndian, 15);
                valueT8  = 0.1 * modbus.int16FromFrame(bigEndian, 17);
                valueT9  = 0.1 * modbus.int16FromFrame(bigEndian, 19);
                valueT10 = 0.1 * modbus.int16FromFrame(bigEndian, 21);
                valueT11 = 0.1 * modbus.int16FromFrame(bigEndian, 23);
                valueT12 = 0.1 * modbus.int16FromFrame(bigEndian, 25);
                valueT13 = 0.1 * modbus.int16FromFrame(bigEndian, 27);
                return true;
            }
            break;
        }
        default: {
            Serial.println(F("Other sensors not yet implemented."));
            break;
        }
    }
    // If something fails, we'll get here
    return false;
}

// cspell: ignore fram RIOTTECHGPLPTC
