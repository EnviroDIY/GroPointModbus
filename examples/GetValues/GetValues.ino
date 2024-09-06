/** =========================================================================
 * @example{lineno} GetValues.ino
 * @author Anthony Aufdenkampe
 * @license This example is published under the BSD-3 license.
 *
 * @brief This prints basic meta-data about a sensor to the first serial port and then
 * begins taking measurements from the sensor.
 *
 * The sensor model and address can easily be modified to use this sketch with any
 * GroPoint Profile modbus sensor.
 * @note GroPoint Profile sensor default Modbus communication settings are 19200 Baud, 8
 * Bits, Even Parity, one Stop Bit (8-E-1). See p35 of manual. 8-N-1 (no parity) is the
 * most common configuration for serial communications.
 *
 * @warning Neither SoftwareSerial, AltSoftSerial, nor NeoSoftwareSerial will support
 * either even or odd parity!
 *
 * @m_examplenavigation{example_get_values,}
 * @m_footernavigation
 * ======================================================================= */

// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
#include <Arduino.h>
#include <GroPointModbus.h>


// ==========================================================================
//  Sensor Settings
// ==========================================================================
// Define the sensor type
gropointModel model = GPLP8;  // The sensor model number

// Define the sensor's modbus address, or SlaveID
// NOTE: The GroPoint User Manual presents SlaveID and registers as decimal
// integers, whereas EnviroDIY and most other modbus systems present it in
// hexadecimal form. Use an online "HEX to DEC Converter".
byte modbusAddress = 0x19;  // HEX 0x01 is the GroPoint default modbus address.

// The Modbus baud rate the sensor uses
int32_t modbusBaud = 9600;  // 19200 is GroPoint default baud rate.

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
uint8_t modbusParity = SWSERIAL_8N1;
// NOTE:  See
// https://github.com/plerup/espsoftwareserial/blob/40038df/src/SoftwareSerial.h#L120-L160
// for a list of data/parity/stop bit configurations that apply to the ESP8266's
// implementation of SoftwareSerial
#else
uint8_t modbusParity = SERIAL_8N1;
// NOTE: See
// https://github.com/arduino/ArduinoCore-avr/blob/321fca0bac806bdd36af8afbc13587f4b67eb5f1/cores/arduino/HardwareSerial.h#L68-L91
// for a list of data/parity/stop bit configurations that apply to AVR and most other
// HardwareSerial instances.
#endif

// Sensor Timing
// Edit these to explore
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
//  Data Logger Options
// ==========================================================================
const int32_t serialBaud = 115200;  // Baud rate for serial monitor

// Define pin number variables
const int sensorPwrPin  = 10;  // The pin sending power to the sensor
const int adapterPwrPin = 22;  // The pin sending power to the RS485 adapter
const int DEREPin       = -1;  // The pin controlling Recieve Enable and Driver Enable
                               // on the RS485 adapter, if applicable (else, -1)
                               // Setting HIGH enables the driver (arduino) to send text
                               // Setting LOW enables the receiver (sensor) to send text

// Turn on debugging outputs (i.e. raw Modbus requests & responses)
// by uncommenting next line (i.e. `#define DEBUG`)
// #define DEBUG


// ==========================================================================
// Create and Assign a Serial Port for Modbus
// ==========================================================================
// Harware serial ports are prefered when available.
// AltSoftSerial is the most stable alternative for modbus.
//   Select over alternatives with the define below.
#define BUILD_ALTSOFTSERIAL // Comment-out if you prefer alternatives

#if defined(BUILD_ALTSOFTSERIAL)
#include <AltSoftSerial.h>
AltSoftSerial modbusSerial;

#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_FEATHER328P)
// The Uno only has 1 hardware serial port, which is dedicated to comunication with the
// computer. If using an Uno, you will be restricted to using AltSofSerial or
// SoftwareSerial
#include <SoftwareSerial.h>
const int SSRxPin = 10;  // Receive pin for software serial (Rx on RS485 adapter)
const int SSTxPin = 11;  // Send pin for software serial (Tx on RS485 adapter)
#pragma message("Using Software Serial for the Uno on pins 10 and 11")
SoftwareSerial modbusSerial(SSRxPin, SSTxPin);

#elif defined(ESP8266)
#include <SoftwareSerial.h>
#pragma message("Using Software Serial for the ESP8266")
SoftwareSerial modbusSerial;

#elif defined(NRF52832_FEATHER) || defined(ARDUINO_NRF52840_FEATHER)
#pragma message("Using TinyUSB for the NRF52")
#include <Adafruit_TinyUSB.h>
HardwareSerial& modbusSerial = Serial1;

#elif !defined(NO_GLOBAL_SERIAL1) && !defined(STM32_CORE_VERSION)
// This is just a assigning another name to the same port, for convienence
// Unless it is unavailable, always prefer hardware serial.
#pragma message("Using HarwareSerial / Serial1")
HardwareSerial& modbusSerial = Serial1;

#else
// This is just a assigning another name to the same port, for convienence
// Unless it is unavailable, always prefer hardware serial.
#pragma message("Using HarwareSerial / Serial")
HardwareSerial& modbusSerial = Serial;
#endif

// Construct the gropoint sensor instance
gropoint sensor;
bool     success;


// ==========================================================================
// Working Functions
// ==========================================================================
// A function for pretty-printing the Modbuss Address in Hexadecimal notation,
// from ModularSensors `sensorLocation()`
String prettyprintAddressHex(byte _modbusAddress) {
    String addressHex = F("0x");
    if (_modbusAddress < 0x10) { addressHex += "0"; }
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

    // Turn on your modbus serial port
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_FEATHER328P) || \
    defined(ARDUINO_SAM_DUE) || not defined(SERIAL_8E1)
    modbusSerial.begin(modbusBaud);
    // NOTE:  The AVR implementation of SoftwareSerial only supports 8N1
    // The hardware serial implementation of the Due also only supports 8N1
#elif defined(ESP8266)
    const int SSRxPin = 13;  // Receive pin for software serial (Rx on RS485 adapter)
    const int SSTxPin = 14;  // Send pin for software serial (Tx on RS485 adapter)
    modbusSerial.begin(modbusBaud, modbusParity, SSRxPin, SSTxPin, false);
#elif defined(BUILD_ALTSOFTSERIAL)
    modbusSerial.begin(modbusBaud);
#else // For Hardware Serial
    modbusSerial.begin(modbusBaud, modbusParity);
#endif

    // Start up the GroPoint sensor
    sensor.begin(model, modbusAddress, &modbusSerial, DEREPin);

// Turn on debugging
#ifdef DEBUG
    sensor.setDebugStream(&Serial);
#endif

    // Start up note
    Serial.print(F("\nGroPoint Profile Soil Moisture and Temperature \nModel: "));
    Serial.println(sensor.getModel());
    Serial.println();

    // Allow the sensor and converter to warm up
    Serial.println(F("Waiting for sensor and adapter to be ready."));
    Serial.print(F("  Warm up time (ms): "));
    Serial.println(WARM_UP_TIME);
    Serial.println();
    delay(WARM_UP_TIME);

    // Confirm Modbus Address
    Serial.println(F("Selected modbus address:"));
    Serial.print(F("  Decimal: "));
    Serial.print(modbusAddress, DEC);
    Serial.print(F(", Hexidecimal: "));
    Serial.println(prettyprintAddressHex(modbusAddress));
    Serial.println();

    // Read Sensor Modbus Address from holding register 40201 (0x9D09)
    Serial.println(F("Get sensor modbus address."));
    byte id = sensor.getSensorAddress();
    Serial.print(F("  Decimal: "));
    Serial.print(id, DEC);
    Serial.print(F(", Hexidecimal: "));
    Serial.println(prettyprintAddressHex(id));
    Serial.println();

    // Get Sensor Information
    Serial.println(F("Get sensor information."));
    Serial.print(F("    "));
    Serial.println(sensor.getSensorInfo());
    Serial.println();

    // Get the sensor serial number
    Serial.println(F("Getting sensor serial number."));
    String SN = sensor.getSerialNumber();
    Serial.print(F("    Serial Number: "));
    Serial.println(SN);

    // Get the sensor's hardware and software version
    Serial.println(F("Getting sensor version numbers."));
    String hardwareV, softwareV;
    sensor.getVersion(hardwareV, softwareV);
    Serial.print(F("    Current Hardware Version: "));
    Serial.println(hardwareV);
    Serial.print(F("    Current Software Version: "));
    Serial.println(softwareV);
    Serial.println();

    // Get Sensor Modbus Baud
    Serial.println(F("Get sensor modbus baud setting."));
    int16_t sensorBaud = sensor.getSensorBaud();
    Serial.print(F("  Baud: "));
    Serial.println(sensorBaud);
    Serial.println();

    // Get Sensor Modbus Parity
    Serial.println(F("Get sensor modbus parity setting."));
    String sensorParity = sensor.getSensorParity();
    Serial.print(F("    Parity: "));
    Serial.println(sensorParity);
    Serial.println();

    // Tell the sensor to start taking measurements
    Serial.println(F("Starting sensor measurements"));
    success = sensor.startMeasurement();
    if (success)
        Serial.println(F("    Measurements started."));
    else
        Serial.println(F("    Failed to start measuring!"));
    Serial.println();

    Serial.println(F("Waiting for sensor to stabilize.."));
    Serial.print(F("    Stabilization time (ms): "));
    Serial.println(STABILIZATION_TIME);
    for (int i = (STABILIZATION_TIME + 500) / 1000; i > 0; i--) {  // +500 to round up
        Serial.print(i);
        delay(250);
        Serial.print(F("."));
        delay(250);
        Serial.print(F("."));
        delay(250);
        Serial.print(F("."));
        delay(250);
    }
    Serial.println(F("\n"));


    // Print table headers
    switch (model) {
        case GPLP8: {
            // Variable Names
            Serial.print(F("Time(ms)  "));
            Serial.print(sensor.getParameter());
            Serial.print(F("  |  "));
            Serial.println(sensor.getParameter1());
            // Variable Units
            Serial.print(F("ms         "));
            Serial.print(sensor.getUnits());
            Serial.print(F("  |  "));
            Serial.println(sensor.getUnits1());
            break;
        }
        default: {
            Serial.println(F("Other sensors not yet implemented."));
        }
    }
}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop() {
    // send the command to get the values
    switch (model) {
        case GPLP8: {
            float M1, M2, M3, M4, M5, M6, M7, M8 = -9999;
            sensor.getValues(M1, M2, M3, M4, M5, M6, M7, M8);

            float T1, T2, T3, T4, T5, T6, T7, T8 = -9999;
            float T9, T10, T11, T12, T13 = -9999;
            sensor.getTemperatureValues(T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11,
                                        T12, T13);

            Serial.print(millis());
            Serial.print(F("     "));
            Serial.print(M1, 1);
            Serial.print(F("  "));
            Serial.print(M2, 1);
            Serial.print(F("  "));
            Serial.print(M3, 1);
            Serial.print(F("  "));
            Serial.print(M4, 1);
            Serial.print(F("  "));
            Serial.print(M5, 1);
            Serial.print(F("  "));
            Serial.print(M6, 1);
            Serial.print(F("  "));
            Serial.print(M7, 1);
            Serial.print(F(" "));
            Serial.print(M8, 1);
            Serial.print(F("  | "));
            Serial.print(T1, 1);
            Serial.print(F(" "));
            Serial.print(T2, 1);
            Serial.print(F(" "));
            Serial.print(T3, 1);
            Serial.print(F(" "));
            Serial.print(T4, 1);
            Serial.print(F(" "));
            Serial.print(T5, 1);
            Serial.print(F(" "));
            Serial.print(T6, 1);
            Serial.print(F(" "));
            Serial.print(T7, 1);
            Serial.print(F(" "));
            Serial.print(T8, 1);
            Serial.print(F(" "));
            Serial.print(T9, 1);
            Serial.print(F(" "));
            Serial.print(T10, 1);
            Serial.print(F(" "));
            Serial.print(T11, 1);
            Serial.print(F(" "));
            Serial.print(T12, 1);
            Serial.print(F(" "));
            Serial.print(T13, 1);
            Serial.println();
            break;
        }
        default: {
            Serial.println(F("Only GPLP-8 has been implemented."));
        }
    }
    // Delay between readings is built into the getInputRegisters() function
    // delay(MEASUREMENT_TIME);
}
