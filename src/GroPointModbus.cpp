/*
 * GroPointModbus.cpp

  Written by Anthony Aufdenkampe 

  Tested with a GPLP-8
*/

#include "GroPointModbus.h"


//----------------------------------------------------------------------------
//                          PUBLIC SENSOR FUNCTIONS
//----------------------------------------------------------------------------


// This function sets up the communication
// It should be run during the arduino "setup" function.
// The "stream" device must be initialized and begun prior to running this.
bool gropoint::begin(gropointModel model, byte modbusSlaveID, Stream *stream, int enablePin)
{
    // Give values to variables;
    _model = model;
    _slaveID = modbusSlaveID;
    // Start up the modbus instance
    bool success = modbus.begin(modbusSlaveID, stream, enablePin);
    // Get the model type from the serial number if it's not known
    if (_model == UNKNOWN) getSerialNumber();

    return success;
}
bool gropoint::begin(gropointModel model, byte modbusSlaveID, Stream &stream, int enablePin)
{return begin(model, modbusSlaveID, &stream, enablePin);}


// This returns a pretty string with the model information
String gropoint::getModel(void)
{
    switch (_model)
    {
        case GPLP_2: {return "GPLP-2";}
        case GPLP_3: {return "GPLP-3";}
        case GPLP_4: {return "GPLP-4";}
        case GPLP_5: {return "GPLP-5";}
        case GPLP_6: {return "GPLP-6";}
        case GPLP_8: {return "GPLP-8";}
        default:  {return "Unknown";}
    }
}


// This gets the modbus slave ID or Sensor Modbus Address.
// The slaveID is in register 0x9D09 (40201)
byte gropoint::getSlaveID(void)
{
    byte command[4] = {0x00, 0x03, 0x9D, 0x09};
    int respSize = modbus.sendCommand(command, 4);

    return modbus.responseBuffer[3];
}


// This sets a new modbus slave ID or Sensor Modbus Address.
// The slaveID is in register 0x9D09 (40201)
bool gropoint::setSlaveID(byte newSlaveID)
{
    byte dataToSend[2] = {newSlaveID, 0x00};
    return modbus.setRegisters(0x9D09, 1, dataToSend, true);
}


// This gets the instrument make, model, firmware version, and serial number as a String
// Page 35 of GroPoint Profile User Manual says:
// Returns the ASCII encoded string ‘RIOTTECHGPLPTC vvvSNnnnnnn’ 
// where vvv is the firmware version (v.v.v) and nnnnnn is the probe serial number.
String gropoint::getSerialNumber(void)
{
    byte command[2] = {_slaveID, 0x11};
    int respSize = modbus.sendCommand(command, 2);

    // return modbus.responseBuffer[respSize];
    return "getSerialNumber() not implemented";
}


// This tells the sensors to begin taking measurements
// Page 35 of GroPoint Profile User Manual says:
// A new measurement is triggered by a read request to the input registers 
// (either read single or read multiple). The first read command will initiate 
// the start of a measurement (soil moisture or soil temperature) and will 
// respond to the master with an ACKNOWLEDGE (05) exception response. 
bool gropoint::startMeasurement(void)
{
    int16_t valueM1, valueM2 = -9999;

    if (getValues(valueM1, valueM2))
    {
        return true;
    }
    else return false;
}


// This tells the optical sensors to stop taking measurements
bool gropoint::stopMeasurement(void)
{
    // A stop command is not in the Modbus Manual.
    // However, a Stop function is required to get these to work in ModularSensors.
    return true;
}


// This gets values back from the sensor
// The input registers are read-only registers containing soil moisture or 
// temperature measurement values. Moisture and temperature readings are stored 
// as 16-bit signed integers and will be returned without a decimal place. In 
// order to obtain the true result, the decimal place setting must be applied. 
// The default setting for this is 1 decimal place, so for example a decimal 
// value of 123 indicates 12.3 % volumetric moisture content for volumetric 
// soil moisture measure- ments, or 12.3 °C for the soil temperature sensor measurements.
// Starting at input register 0x7531 (30001)
bool gropoint::getValues(int16_t &valueM1, int16_t &valueM2)
{
    // Set values to -9999 and error flagged before asking for the result
    valueM1 = -9999;
    valueM2 = -9999;

    // Trigger read request
    Serial.println("Trigger read request");
    modbus.getRegisters(0x04, 0x7531, 2);
    delay(200);

    switch(_model)
    {
        case GPLP_2: 
        {
            if (modbus.getRegisters(0x04, 0x7531, 2))
            {
                valueM1 = modbus.int16FromFrame(littleEndian, 3);
                valueM2 = modbus.int16FromFrame(littleEndian, 5);
                return true;
            }
            else return false;
        }
    }
}
