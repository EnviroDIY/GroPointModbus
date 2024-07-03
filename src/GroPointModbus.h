/**
 * @file GroPointModbus.h
 * Part of the EnviroDIY GroPointModbus library for Arduino.
 * @license This library is published under the BSD-3 license.
 * @author Anthony Aufdenkampe
 *
 * @brief Contains the GroPointModbus class declarations.
 *
 * Tested with GPLP-8
 */

#ifndef GroPointModbus_h
#define GroPointModbus_h

#include <Arduino.h>
#include <SensorModbusMaster.h>


/**
 * @brief The various GroPoint GPLP-X Moisture/Temperature Profiling Probes supported by
 * this library
 */
typedef enum gropointModel {
    GPLP2 = 2,  ///< GroPoint Profiling Probe with 2 segments (30 cm)
    GPLP3 = 3,  ///< GroPoint Profiling Probe with 3 segments (45 cm)
    GPLP4 = 4,  ///< GroPoint Profiling Probe with 4 segments (60 cm)
    GPLP5 = 5,  ///< GroPoint Profiling Probe with 5 segments (75 cm)
    GPLP6 = 6,  ///< GroPoint Profiling Probe with 6 segments (90 cm)
    GPLP8 = 8,  ///< GroPoint Profiling Probe with 8 segments (120 cm)
    GPLPX = 99  ///< Use if the sensor model is unknown. Doing this is generally a bad
                ///< idea, but it can be helpful for doing things like getting the
                ///< serial number of an unknown model.
} gropointModel;

/**
 * @brief The Main Class
 *
 * This is the class for communication with GroPoint sensors via modbus.
 */
class gropoint {

 public:

    /**
     * @brief This function sets up the communication.
     *
     * It should be run during the arduino "setup" function.
     * The "stream" device must be initialized prior to running this.
     *
     * @param model The model of the GroPoint sensor, from #gropointModel
     * @param modbusSlaveID The byte identifier of the modbus slave device.
     * @param stream A pointer to the Arduino stream object to communicate with.
     * @param enablePin A pin on the Arduino processor to use to send an enable signal
     * to an RS485 to TTL adapter. Use a negative number if this does not apply.
     * Optional with a default value of -1.
     * @return *bool* True if the starting communication was successful, false if not.
     */
    bool begin(gropointModel model, byte modbusSlaveID, Stream* stream,
               int enablePin = -1);
    /**
     * @brief This function sets up the communication.
     *
     * It should be run during the arduino "setup" function.
     * The "stream" device must be initialized prior to running this.
     *
     * @param model The model of the GroPoint sensor, from #gropointModel
     * @param modbusSlaveID The byte identifier of the modbus slave device.
     * @param stream A reference to the Arduino stream object to communicate with.
     * @param enablePin A pin on the Arduino processor to use to send an enable signal
     * to an RS485 to TTL adapter. Use a negative number if this does not apply.
     * Optional with a default value of -1.
     * @return *bool* True if the starting communication was successful, false if not.
     */
    bool begin(gropointModel model, byte modbusSlaveID, Stream& stream,
               int enablePin = -1);

    /**
     * @anchor metadata_fxns
     * @name Functions to get and set sensor addresses and metadata
     */
    /**@{*/

    /**
     * @brief Returns a pretty string with the model information
     *
     * @note This is only based on the model input from the "begin" fxn.
     * The sensor itself does not return its model information.
     *
     * @return *String* The GroPoint sensor model
     */
    String getModel(void);

    /**
     * @brief Returns a pretty string with the parameters measured by group 0 (soil
     * moisture).
     *
     * @note This is only based on the model input from the "begin" fxn.
     * The sensor itself does not return this information.
     *
     * @return *String* The primary parameter being measured on this GroPoint sensor
     * model.
     */
    String getParameter(void);

    /**
     * @brief Returns a pretty string with the parameters measured by group 1
     * (temperature).
     *
     * @note This is only based on the model input from the "begin" fxn.
     * The sensor itself does not return this information.
     *
     * @return *String* The secondary parameter being measured on this GroPoint sensor
     * model.
     */
    String getParameter1(void);

    /**
     * @brief Returns a pretty string with the measurement units for parameter group 0
     * (soil moisture).
     *
     * @note This is only based on the model input from the "begin" fxn.
     * The sensor itself does not return this information.
     *
     * @return *String* The units of primary parameter being measured on this GroPoint
     * sensor model.
     */
    String getUnits(void);

    /**
     * @brief Returns a pretty string with the measurement units for parameter group 1
     * (temperature).
     *
     * @note This is only based on the model input from the "begin" fxn.
     * The sensor itself does not return this information.
     *
     * @return *String* The units of secondary parameter being measured on this GroPoint
     * sensor model.
     */
    String getUnits1(void);

    /**
     * @brief Gets the modbus sensor (slave) address.
     *
     * The address is in holding register 40201, decimal offset 200 (hexadecimal
     * 0x00C8).
     *
     * Does not seem to work with a broadcast address of 0x00 or 0xFF
     *
     * @return *byte* The slave ID of the GroPoint sensor
     */
    byte getSensorAddress(void);
    /**
     * @brief Set a new modbus sensor (slave) address.
     *
     * The address is in holding register 40201, decimal offset 200 (hexadecimal
     * 0x00C8).
     *
     * The address change will take effect immediately, so any subsequent commands must
     * use the new address.
     *
     * This register cannot be updated using the broadcast address.
     *
     * @param newSensorAddress  The new address (slave ID) for the GroPoint sensor
     * @return *bool* True if the slave ID was successfully set, false if not.
     */
    bool setSensorAddress(byte newSensorAddress);

    /**
     * @brief Get the sensor modbus baud rate
     *
     * The baud rate is in holding register 40203, decimal offset 202 (hexadecimal
     * 0x00CA)
     *
     * The factory default value is 0, corresponding to a baud rate of 19200.
     *
     * Valid values: 0=19200, 1=9600, 2=4800, 3=2400, 4=1200, 5=600, 6=300.
     *
     * @return *int16_t* A code for the baud rate
     */
    int16_t getSensorBaud(void);

    /**
     * @brief Set the sensor modbus baud
     *
     * The baud rate is in holding register 40203, decimal offset 202 (hexadecimal
     * 0x00CA)
     *
     * Change this value to any of the following valid values: 19200, 9600, 4800, 2400,
     * 1200, 600, 300.
     *
     * The new baud rate **does not take effect** until the sensor is power cycled, or
     * if the restart communications diagnostic command (08, with subfunction 01) is
     * received.
     *
     * @param newSensorBaud The new baud rate to use
     * @return *bool* True if the baud rate was successfully set, false if not.
     */
    bool setSensorBaud(int32_t newSensorBaud);

    /**
     * @brief Get the Sensor modbus serial parity
     *
     * The parity is in holding register 40204, decimal offset 0203 (hexadecimal
     * 0x00CB).
     *
     * The factory default value is 2 corresponding to Even parity.
     *
     * @return *String* The parity configuration
     */
    String getSensorParity(void);

    /**
     * @brief Set the sensor modbus serial parity
     *
     * The parity is in holding register 40204, decimal offset 0203 (hexadecimal
     * 0x00CB).
     *
     * Change this value to any of the following valid values:
     *    "None", "Odd", "Even".
     *
     * The new parity setting **does not take effect** until the sensor is power cycled,
     * or if the restart communications diagnostic command (08, with subfunction 01) is
     * received.
     *
     * @param newSensorParity the new parity, "None", "Odd", or "Even"
     * @return *bool* True if the parity was successfully set, false if not.
     */
    bool setSensorParity(String newSensorParity);

    /**
     * @brief Get the sensor information as a String
     *
     * Page 35 of GroPoint Profile User Manual says: Function 17 (0x11) returns the
     * ASCII encoded string
     *
     * @return *String* The sensor information in the format ‘RIOTTECHGPLPTC
     * vvvSNnnnnnn’ where vvv is the firmware version (v.v.v) and nnnnnn is the probe
     * serial number.
     */
    String getSensorInfo(void);

    /**
     * @brief Gets the instrument serial number as a String
     *
     * This is pulled as a substring from the sensor information.
     *
     * @return *String* The serial number of the GroPoint sensor
     */
    String getSerialNumber(void);

    /**
     * @brief Gets the hardware and software version of the sensor
     *
     * This is pulled as a substring from the sensor information.
     *
     * @param hardwareVersion A reference to a String object to be modified with the
     * hardware version.
     * @param softwareVersion A reference to a String object to be modified with the
     * software version.
     * @return *bool* True if the hardware and software versions were successfully
     * updated, false if not.
     */
    bool getVersion(String& hardwareVersion, String& softwareVersion);

    /**
     * @brief Restarts communication with the sensor using the modbus diagnostic command
     * 08 (0x08) with subfunction 01.
     *
     * A request data field contents of FF 00 hex causes the port’s Communications Event
     * Log to be cleared also. Contents of 00 00 leave the log as it was prior to the
     * restart.
     *
     * @return *bool* True if the commuication was restarted, false if not.
     */
    bool restartCommunications(void);
    /**@}*/

    /**
     * @anchor measurement_fxns
     * @name Functions to start and stop measurements
     *
     * Page 39 of GroPoint Profile User Manual says:
     *      A new measurement is triggered by a read request to the input registers
     * (either read single or read multiple). The first read command will initiate the
     * start of a measurement (soil moisture or soil temperature) and will respond to
     * the master with an ACKNOWLEDGE (05) exception response. Moisture measurements
     * take approximately 200 ms per segment. Temperature measurements take
     * approximately 200 ms per sensor. The master should wait for this amount of time
     * to expire before attempting to retrieve the measurement values with another read
     * command.
     */
    /**@{*/


    /**
     * @brief Tells the sensors to begin taking measurements
     *
     * A start command is not in the GroPoint Modbus Manuals.
     * However, Start/Stop functions are required to get these to work in
     * ModularSensors.
     * Instead, just confirm that sensor in communicating with a
     * gropoint::getSensorAddress() command
     *
     * @return *bool* True if the measurements were successfully started, false if not.
     */
    bool startMeasurement(void);

    /**
     * @brief Tells the sensors to stop taking measurements
     *
     * A stop command is not in the GroPoint Modbus Manuals; always returns true.
     *
     * @return *bool* True if the measurements were successfully started, false if not.
     */
    bool stopMeasurement(void);
    /**@}*/

    /**
     * @anchor value_fetching
     * @name Functions to get one or more values from a sensor.
     *
     * Based off modbusMaster::getRegisters(), but with trigger followed by a 200 ms
     * delay before read request
     *
     * Page 39 of GroPoint Profile User Manual says:
     *      A new measurement is triggered by a read request to the input registers
     * (either read single or read multiple). The first read command will initiate the
     * start of a measurement (soil moisture or soil temperature) and will respond to
     * the master with an ACKNOWLEDGE (05) exception response. Moisture measurements
     * take approximately 200 ms per segment. Temperature measurements take
     * approximately 200 ms per sensor. The master should wait for this amount of time
     * to expire before attempting to retrieve the measurement values with another read
     * command.
     *
     *
     * Page 37-38 of GroPoint Profile User Manual says:
     *      The input registers are read-only registers containing soil moisture or
     * temperature measurement values. Moisture and temperature readings are stored as
     * 16-bit signed integers and will be returned without a decimal place. In order to
     * obtain the true result, the decimal place setting must be applied. The default
     * setting for this is 1 decimal place, so for example a decimal value of 123
     * indicates 12.3 % volumetric moisture content for volumetric soil moisture
     * measurements, or 12.3 °C for the soil temperature sensor measurements.
     */
    /**@{*/

    /**
     * @brief Get read-only input registers containing measured values
     *
     * Similar to modbusMaster::getRegisters(), but with a trigger request followed by a
     * 200 ms delay before the read measurement request
     *
     * @see value_fetching
     *
     * @param startRegister The first register to read.
     * @param numRegisters The total number of registers to read
     * @return *bool* True if the registers were read successfully, false if not.
     */
    bool getInputRegisters(int16_t startRegister, int16_t numRegisters);

    /**
     * @brief Gets parameter group 0 (soil moisture) values back from the sensor
     *
     * Values starting at input register 30001, decimal offset 0 (hexadecimal 0x0000)
     *
     * @see value_fetching
     *
     * @param valueM1 The first value from parameter group 0 (soil moisture)
     * @param valueM2 The second value from parameter group 0 (soil moisture)
     * @param valueM3 The third value from parameter group 0 (soil moisture)
     * @param valueM4 The fourth value from parameter group 0 (soil moisture)
     * @param valueM5 The fifth value from parameter group 0 (soil moisture)
     * @param valueM6 The sixth value from parameter group 0 (soil moisture)
     * @param valueM7 The seventh value from parameter group 0 (soil moisture)
     * @param valueM8 The eighth value from parameter group 0 (soil moisture)
     * @return *bool* True if the measurements were successfully obtained, false if not.
     */
    bool getValues(float& valueM1, float& valueM2, float& valueM3, float& valueM4,
                   float& valueM5, float& valueM6, float& valueM7, float& valueM8);

    /**
     * @brief Gets parameter group 1 (temperature) values back from the sensor
     *
     * @param valueT1 The first value from parameter group 1 (temperature)
     * @param valueT2 The second value from parameter group 1 (temperature)
     * @param valueT3 The third value from parameter group 1 (temperature)
     * @param valueT4 The fourth value from parameter group 1 (temperature)
     * @param valueT5 The fifth value from parameter group 1 (temperature)
     * @param valueT6 The sixth value from parameter group 1 (temperature)
     * @param valueT7 The seventh value from parameter group 1 (temperature)
     * @param valueT8 The eighth value from parameter group 1 (temperature)
     * @param valueT9 The ninth value from parameter group 1 (temperature)
     * @param valueT10 The tenth value from parameter group 1 (temperature)
     * @param valueT11 The eleventh value from parameter group 1 (temperature)
     * @param valueT12 The twelvth value from parameter group 1 (temperature)
     * @param valueT13 The thirteenth value from parameter group 1 (temperature)
     * @return *bool* True if the measurements were successfully obtained, false if not.
     */
    bool getTemperatureValues(float& valueT1, float& valueT2, float& valueT3,
                              float& valueT4, float& valueT5, float& valueT6,
                              float& valueT7, float& valueT8, float& valueT9,
                              float& valueT10, float& valueT11, float& valueT12,
                              float& valueT13);
    /**@}*/

    /**
     * @anchor debugging
     * @name Debugging functions
     */
    /**@{*/

    /**
     * @brief Set a stream for debugging information to go to.
     *
     * @param stream An Arduino stream object
     */
    void setDebugStream(Stream* stream) {
        modbus.setDebugStream(stream);
    }
    /**
     * @copydoc gropoint::setDebugStream(Stream* stream)
     */
    void setDebugStream(Stream& stream) {
        modbus.setDebugStream(stream);
    }
    /**
     * @brief Un-set the stream for debugging information to go to; stop debugging.
     */
    void stopDebugging(void) {
        modbus.stopDebugging();
    }
    /**@}*/


 private:
    int  _model;    ///< the sensor model
    byte _slaveID;  ///< the sensor slave id

    // Class from EnviroDIY SensorModbusMaster library,
    // https://github.com/EnviroDIY/SensorModbusMaster
    modbusMaster modbus;  ///< an internal reference to the modbus communication object.
};

#endif
