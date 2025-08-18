/** =========================================================================
 * @example{lineno} ChangeModbusSettings.ino
 * @author Anthony Aufdenkampe
 * @license This example is published under the BSD-3 license.
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
 * This sketch depends on the GropointModbus library and also loosely on the
 * SensorModbusMaster library via it's example sketch:
 * SensorModbusMaster/examples/readWriteRegister/readWriteRegister.ino
 * ======================================================================= */

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <GroPointModbus.h>

// Turn on debugging outputs (i.e. raw Modbus requests & responds)
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
// 8-E-1 is GroPoint default parity.
// 8-N-1 is the recommended parity because it is more widely supported and the only
// allowable parity for AltSoftSerial, NeoSWSerial, & SoftwareSerial and several other
// board serial libraries.
#if defined(ESP8266)
#include <SoftwareSerial.h>
uint8_t defaultModbusParity = SWSERIAL_8E1;
uint8_t newModbusParity     = SWSERIAL_8N1;
String  newModbusParity_str = "None";
// NOTE:  See
// https://github.com/plerup/espsoftwareserial/blob/40038df/src/SoftwareSerial.h#L120-L160
// for a list of data/parity/stop bit configurations that apply to the ESP8266's
// implementation of SoftwareSerial
#else
uint8_t defaultModbusParity = SERIAL_8E1;
uint8_t newModbusParity     = SERIAL_8N1;
String  newModbusParity_str = "None";
// NOTE: See
// https://github.com/arduino/ArduinoCore-avr/blob/321fca0bac806bdd36af8afbc13587f4b67eb5f1/cores/arduino/HardwareSerial.h#L68-L91
// for a list of data/parity/stop bit configurations that apply to AVR and most other
// HardwareSerial instances.
#endif

// Sensor Timing. Edit these to explore!
#define WARM_UP_TIME 350  // milliseconds for sensor to respond to commands.
// GroPoint Profile User Manual page 7:
// The time from application of power to the SDI-12 power bus until the
// sensor is ready to receive a command is approximately 350 ms.

#define STABILIZATION_TIME 100  // milliseconds for readings to stabilize.

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
const int DEREPin       = -1;  // The pin controlling Receive Enable and Driver Enable
                               // on the RS485 adapter, if applicable (else, -1)
                               // Setting HIGH enables the driver (arduino) to send text
                               // Setting LOW enables the receiver (sensor) to send text

// Construct a Serial object for Modbus
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_FEATHER328P)
// The Uno only has 1 hardware serial port, which is dedicated to communication with the
// computer. If using an Uno, you will be restricted to using AltSofSerial or
// SoftwareSerial
#include <SoftwareSerial.h>
const int SSRxPin = 10;  // Receive pin for software serial (Rx on RS485 adapter)
const int SSTxPin = 11;  // Send pin for software serial (Tx on RS485 adapter)
#pragma message("Using Software Serial for the Uno on pins 10 and 11")
SoftwareSerial modbusSerial(SSRxPin, SSTxPin);
// AltSoftSerial modbusSerial;
#elif defined(ESP8266)
#pragma message("Using Software Serial for the ESP8266")
SoftwareSerial modbusSerial;
#elif defined(NRF52832_FEATHER) || defined(ARDUINO_NRF52840_FEATHER)
#pragma message("Using TinyUSB for the NRF52")
#include <Adafruit_TinyUSB.h>
HardwareSerial& modbusSerial = Serial1;
#elif !defined(NO_GLOBAL_SERIAL1) && !defined(STM32_CORE_VERSION)
// This is just a assigning another name to the same port, for convience
// Unless it is unavailable, always prefer hardware serial.
#pragma message("Using HardwareSerial / Serial1")
HardwareSerial& modbusSerial = Serial1;
#else
// This is just a assigning another name to the same port, for convience
// Unless it is unavailable, always prefer hardware serial.
#pragma message("Using HardwareSerial / Serial")
HardwareSerial& modbusSerial = Serial;
#endif

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
    if (sensorPwrPin >= 0) {
        pinMode(sensorPwrPin, OUTPUT);
        digitalWrite(sensorPwrPin, HIGH);
    }
    if (adapterPwrPin >= 0) {
        pinMode(adapterPwrPin, OUTPUT);
        digitalWrite(adapterPwrPin, HIGH);
    }
    if (DEREPin >= 0) { pinMode(DEREPin, OUTPUT); }

    // Turn on the "main" serial port for debugging via USB Serial Monitor
    Serial.begin(serialBaud);

    // Turn on your modbus serial port
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_FEATHER328P) || \
    defined(ARDUINO_SAM_DUE) || not defined(SERIAL_8E1)
    Serial.println(F("THIS DEVICE CANNOT BE USED TO CHANGE THE DEFAULT SERIAL PARITY"));
    return;
    // NOTE:  The AVR implementation of SoftwareSerial only supports 8N1
    // The hardware serial implementation of the Due also only supports 8N1
#elif defined(ESP8266)
    const int SSRxPin = 13;  // Receive pin for software serial (Rx on RS485 adapter)
    const int SSTxPin = 14;  // Send pin for software serial (Tx on RS485 adapter)
    modbusSerial.begin(defaultModbusBaud, defaultModbusParity, SSRxPin, SSTxPin, false);
#else
    // Setup your modbus hardware serial port
    modbusSerial.begin(defaultModbusBaud, defaultModbusParity);
#endif

    // Setup the sensor instance
    sensor.begin(model, defaultModbusAddress, &modbusSerial, DEREPin);

// Turn on debugging
#ifdef DEBUG
    sensor.setDebugStream(&Serial);
#endif

    // Start up note
    Serial.println(F("\nChange Modbus Settings for GroPoint Profile sensors "));

    // Allow the sensor and converter to warm up
    Serial.println(F("Waiting for sensor and adapter to be ready."));
    Serial.print(F("  Warm up time (ms): "));
    Serial.println(WARM_UP_TIME);
    Serial.println();
    delay(WARM_UP_TIME);

    // Confirm Modbus Address
    Serial.println(F("Default sensor modbus address:"));
    Serial.print(F("  Decimal: "));
    Serial.print(defaultModbusAddress, DEC);
    Serial.print(F(", Hexidecimal: "));
    Serial.println(prettyprintAddressHex(defaultModbusAddress));
    Serial.println();

    // Read Sensor Modbus Address
    Serial.println(F("Get sensor modbus address."));
    byte id = sensor.getSensorAddress();
    Serial.print(F("  Decimal: "));
    Serial.print(id, DEC);
    Serial.print(F(", Hexidecimal: "));
    Serial.println(prettyprintAddressHex(id));
    Serial.println();

    // Get Sensor Modbus Baud
    Serial.println(F("Get sensor modbus baud setting."));
    int16_t sensorBaud = sensor.getSensorBaud();
    Serial.print(F("  Baud: "));
    Serial.println(sensorBaud);
    Serial.println(F("  Change is effective immediately, so change last."));
    Serial.println();

    // Get Sensor Modbus Parity
    Serial.println(F("Get sensor modbus parity setting."));
    String sensorParity = sensor.getSensorParity();
    Serial.print(F("    Parity: "));
    Serial.println(sensorParity);
    Serial.println();

    // Delay to give time to restart with serial monitor turned on.
    Serial.println(F("Resetting Modbus Serial Settings."));
    Serial.println(F("  Additional 5s delay time to turn on serial monitor.\n"));
    delay(5000);

    // Set sensor modbus baud
    Serial.print(F("Set sensor modbus baud to "));
    Serial.println(newModbusBaud);
    success = sensor.setSensorBaud(newModbusBaud);
    if (success) {
        Serial.println("  Success! New baud will take effect once sensor is "
                       "power cycled.");
    } else
        Serial.println(F("  Baud reset failed!"));
    Serial.println();

    // Set sensor modbus parity
    Serial.print(F("Set sensor modbus parity to "));
    Serial.println(newModbusParity_str);
    success = sensor.setSensorParity(newModbusParity_str);
    if (success) {
        Serial.println(
            "  Success! New parity will take effect once sensor is power cycled.");
    } else
        Serial.println(F("  Parity reset failed!"));
    Serial.println();

    // Set sensor modbus address
    Serial.print(F("Set sensor modbus address to "));
    Serial.println(prettyprintAddressHex(newModbusAddress));
    success = sensor.setSensorAddress(newModbusAddress);
    if (success) {
        Serial.println(
            F("  Success! New modbus address will take effect immediately."));
    } else
        Serial.println(F("  Address reset failed!"));
    Serial.println(F("\n"));

    Serial.println(F("Upload a sketch with the new Modbus settings "));
    Serial.println(F("to reestablish communications with the sensor."));
}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop() {
    // Empty loop function
}
