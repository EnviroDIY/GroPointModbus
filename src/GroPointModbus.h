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
typedef enum gropointModel
{
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

class gropoint
{

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

    // This gets the modbus slave ID.  Not supported by many sensors.
    byte getSlaveID(void);

    // This sets a new modbus slave ID
    bool setSlaveID(byte newSlaveID);

    // This gets the instrument serial number as a String
    String getSerialNumber(void);

    // This gets the hardware and software version of the sensor
    // The float variables for the hardware and software versions must be
    // initialized prior to calling this function.
    // The reference (&) is needed when declaring this function so that
    // the function is able to modify the actual input floats rather than
    // create and destroy copies of them.
    // There is no need to add the & when actually using the function.
    bool getVersion(float &hardwareVersion, float &softwareVersion);

    // This tells the sensors to begin taking measurements
    bool startMeasurement(void);

    // This tells the optical sensors to stop taking measurements
    bool stopMeasurement(void);

    // This gets values back from the sensor
    bool getValues(int16_t &valueM1, int16_t &valueM2);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3, float &valueM4);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3, float &valueM4, float &valueM5);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3, float &valueM4, float &valueM5, float &valueM6);
    // bool getValues(float &valueM1, float &valueM2, float &valueM3, float &valueM4, float &valueM5, float &valueM6, float &valueM7, float &valueM8);

    // This returns the temperature values from a sensor as a float
    bool getValues(float &valueT1, float &valueT2, float &valueT3, float &valueT4);
    // bool getValues(float &valueT1, float &valueT2, float &valueT3, float &valueT4, float &valueT5, float &valueT6);
    // bool getValues(float &valueT1, float &valueT2, float &valueT3, float &valueT4, float &valueT5, float &valueT6, float &valueT7);
    // bool getValues(float &valueT1, float &valueT2, float &valueT3, float &valueT4, float &valueT5, float &valueT6, float &valueT7, float &valueT8, float &valueT9);
    // bool getValues(float &valueT1, float &valueT2, float &valueT3, float &valueT4, float &valueT5, float &valueT6, float &valueT7, float &valueT8, float &valueT9, float &valueT10);
    // bool getValues(float &valueT1, float &valueT2, float &valueT3, float &valueT4, float &valueT5, float &valueT6, float &valueT7, float &valueT8, float &valueT9, float &valueT10, float &valueT11, float &valueT12, float &valueT13);



    // This sets a stream for debugging information to go to;
    void setDebugStream(Stream *stream){modbus.setDebugStream(stream);}
    void stopDebugging(void){modbus.stopDebugging();}


private:
    int _model;  // The sensor model

    byte _slaveID;

    modbusMaster modbus;  // Class from EnviroDIY SensorModbusMaster library, https://github.com/EnviroDIY/SensorModbusMaster
};

#endif
