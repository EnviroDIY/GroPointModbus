/*
 *GroPointModbus.h

 Written by Anthony Aufdenkampe 

 Tested with a GPLP-8
*/

#ifndef GroPointModbus_h
#define GroPointModbus_h

#include <Arduino.h>
#include <SensorModbusMaster.h>

// The various GroPoint GPLP-X Moisture/Temperature Profiling Probes 
// supported by this library
typedef enum gropointModel {
    GPLP_2 = 2,  // GroPoint Profiling Probe with 2 segments (30 cm)
    GPLP_3 = 3,  // GroPoint Profiling Probe with 3 segments (45 cm)
    GPLP_4 = 4,  // GroPoint Profiling Probe with 4 segments (60 cm)
    GPLP_5 = 5,  // GroPoint Profiling Probe with 5 segments (75 cm)
    GPLP_6 = 6,  // GroPoint Profiling Probe with 6 segments (90 cm)
    GPLP_8 = 8,  // GroPoint Profiling Probe with 8 segments (120 cm)
    UNKNOWN   // Use if the sensor model is unknown. Doing this is generally a
              // bad idea, but it can be helpful for doing things like getting
              // the serial number of an unknown model.
} gropointModel;

class gropoint {

public:

    // This function sets up the communication
    // It should be run during the arduino "setup" function.
    // The "stream" device must be initialized prior to running this.
    bool begin(gropointModel model, byte modbusSlaveID, Stream *stream, int enablePin = -1);
    bool begin(gropointModel model, byte modbusSlaveID, Stream &stream, int enablePin = -1);

    // This returns a pretty string with the model information
    // NOTE:  This is only based on the model input from the "begin" fxn.
    // The sensor itself does not return its model information.
    String getModel(void);

    // This gets the modbus sensor (slave) address. 
    // Does not seem to work with a broadcast address of 0x00 or 0xFF
    byte getSensorAddress(void);

    // This sets the modbus sensor (slave) address.
    // The address change will take effect immediately, 
    // so any subsequent commands must use the new address.
    // This register cannot be updated using the broadcast address.
    bool setSensorAddress(byte newSlaveID);

    // This gets sensor information as a String
    // ‘RIOTTECHGPLPTC vvvSNnnnnnn’ where 
    // vvv is the firmware version (v.v.v) and 
    // nnnnnn is the probe serial number.
    String getSensorInfo(void);

    // This gets the instrument serial number as a String
    String getSerialNumber(void);

    // This gets the firmware version of the sensor
    String getVersion(void);

    // This restarts communications, 
    // using the modbus diagnostic command 08 (0x08) with subfunction 01.
    bool restartCommunications(void);

    // This gets sensor modbus baud rate
    // The factory default value is 0, corresponding to a baud rate of 19200.
    // Valid values: 0=19200, 1=9600, 2=4800, 3=2400, 4=1200, 5=600, 6=300. 
    int16_t getSensorBaud(void);

    // This sets sensor modbus baud  
    // Change this value to any of the following valid values: 0=19200, 1=9600, 
    // 2=4800, 3=2400, 4=1200, 5=600, 6=300. The new baud rate does not take 
    // effect until the sensor is power cycled, or if the restart 
    // communications diagnostic command (08, with subfunction 01) is received.
    int16_t setSensorBaud(void);

    // This gets sensor modbus serial parity 
    // The factory default value is 2 corresponding to Even parity.
    String getSensorParity(void);

    // This sets sensor modbus serial parity 
    // Change this value to any of the following valid values:
    // 0=None, 1=Odd, 2=Even. The new parity setting 
    // does not take effect until the sensor is power cycled, or if the restart 
    // communications diagnostic command (08, with subfunction 01) is received.
    String setSensorParity(void);

    // This tells the sensors to begin taking measurements
    bool startMeasurement(void);

    // This tells the sensors to stop taking measurements
    bool stopMeasurement(void);

    // This gets read-only input registers containing measured values
    // Similar to modbusMaster::getRegisters(), but with a trigger request
    // followed by a 200 ms delay before the read measurement request
    bool getInputRegisters(int16_t startRegister, int16_t numRegisters);

    // This gets values back from the sensor as a float
    // The float variables for must be initialized prior to calling this function.
    // The reference (&) is needed when declaring this function so that
    // the function is able to modify the actual input floats rather than
    // create and destroy copies of them.
    // There is no need to add the & when actually using the function.
    bool getValues(float &valueM1, float &valueM2, float &valueM3, 
                   float &valueM4, float &valueM5, float &valueM6, 
                   float &valueM7, float &valueM8);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3, float &valueM4);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3, float &valueM4, float &valueM5);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3, float &valueM4, float &valueM5, float &valueM6);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3, float &valueM4, float &valueM5, float &valueM6, float &valueM7, float &valueM8);

    // This returns the temperature values from a sensor as a float
    bool getTemperatureValues(float &valueT1, float &valueT2,  
            float &valueT3, float &valueT4, float &valueT5, float &valueT6, 
            float &valueT7, float &valueT8, float &valueT9, float &valueT10, 
            float &valueT11, float &valueT12, float &valueT13);

    // This sets a stream for debugging information to go to;
    void setDebugStream(Stream *stream){modbus.setDebugStream(stream);}
    void stopDebugging(void){modbus.stopDebugging();}


private:
    int _model;  // The sensor model

    byte _slaveID;

    // Class from EnviroDIY SensorModbusMaster library, https://github.com/EnviroDIY/SensorModbusMaster
    modbusMaster modbus;  
};

#endif
