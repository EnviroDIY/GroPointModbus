/*****************************************************************************
GetValues.ino

This prints basic meta-data about a sensor to the first serial port and then
begins taking measurements from the sensor.

The sensor model and address can easily be modified to use this sketch with any
GroPoint Profile modbus sensor.

NOTE: GroPoint Profile sensor default Modbus communication settings are 
19200 Baud, 8 Bits, Even Parity, one Stop Bit (8-E-1). See p35 of manual.
8-N-1 (no parity) is the most common configuration for serial communications.

NOTE:  that neither SoftwareSerial, AltSoftSerial, nor NeoSoftwareSerial
will support either even or odd parity!
*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <GroPointModbus.h>

#if defined __AVR__
#include <AltSoftSerial.h>
// #include <SoftwareSerial.h>
#endif

#if defined ESP8266
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#endif

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
byte modbusAddress = 0x01;  // GroPoint ships sensors with a default ID of 0x01.

// The Modbus baud rate the sensor uses
int32_t modbusBaud = 9600;  // GroPoint default baud rate is 19200.

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
// Pins for `SoftwareSerial` only. Not for `AltSoftSerial`, which uses fixed pins.
const int SSRxPin = 13;  // Receive pin for software serial (Rx on RS485 adapter)
const int SSTxPin = 14;  // Send pin for software serial (Tx on RS485 adapter)

// Construct software serial object for Modbus
    // Hardware serial is the only serial stream that can have parity (even or odd).
    // The Uno only has 1 hardware serial port, which is dedicated to comunication with the computer
    // If using an Uno, you will be restricted to using AltSofSerial or SoftwareSerial

// If using the Mayfly, you can use the hardware Serial1 port with the following define:
// #define HARDWARE_MODBUS_SERIAL
    // To access HardwareSerial on the Mayfly use a Grove to Male Jumpers cable
    // or other set of jumpers to connect Grove D5 & D6 lines to the hardware
    // serial TX1 (from D5) and RX1 (from D6) pins on the left 20-pin header.

#if defined HARDWARE_MODBUS_SERIAL
    HardwareSerial& modbusSerial = Serial1;
#elif defined __AVR__
    // SoftwareSerial modbusSerial(SSRxPin, SSTxPin);
    AltSoftSerial modbusSerial;
#elif defined ESP8266
    SoftwareSerial modbusSerial;
#elif defined(NRF52832_FEATHER) || defined(ARDUINO_NRF52840_FEATHER)
    #include <Adafruit_TinyUSB.h>
    HardwareSerial& modbusSerial = Serial1;
#elif !defined(NO_GLOBAL_SERIAL1)
    HardwareSerial& modbusSerial = Serial1;
#else
    HardwareSerial& modbusSerial = Serial;
#endif

// Construct the gropoint sensor instance
gropoint sensor;
bool success;

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

    // Setup your modbus serial port
    #if defined HARDWARE_MODBUS_SERIAL
        modbusSerial.begin(modbusBaud, SERIAL_8E1);  // The modbus serial stream.
        // ^^ use this for 8 data bits - even parity - 1 stop bit
    #elif defined ESP8266
        modbusSerial.begin(modbusBaud, SWSERIAL_8N1, SSRxPin, SSTxPin, false, 256); // The modbus serial stream
    #else
        modbusSerial.begin(modbusBaud); // The modbus serial stream
        // ^^ use this for 8 data bits - no parity - 1 stop bits
        // Despite being technically "non-compliant" with the modbus specifications
        // 8N1 parity is very common and the EnviroDIY default.
    #endif

    // Setup the sensor instance
    sensor.begin(model, modbusAddress, &modbusSerial, DEREPin);

    // Turn on debugging
    #ifdef DEBUG
        sensor.setDebugStream(&Serial);
    #endif

    // Start up note
    Serial.print("\nGroPoint Profile Soil Moisture and Temperature \nModel: ");
    Serial.println(sensor.getModel());
    Serial.println();

    // Allow the sensor and converter to warm up
    Serial.println("Waiting for sensor and adapter to be ready.");
    Serial.print("  Warm up time (ms): ");
    Serial.println(WARM_UP_TIME);
    Serial.println();
    delay(WARM_UP_TIME);

    // Confirm Modbus Address 
    Serial.println("Selected modbus address:");
    Serial.print("  integer: ");
    Serial.print(modbusAddress, DEC);
    Serial.print(", hexidecimal: ");
    Serial.println(prettyprintAddressHex(modbusAddress));
    Serial.println();

    // Read Sensor Modbus Address from holding register 40201 (0x9D09)
    Serial.println("Get sensor modbus address.");
    byte id = sensor.getSensorAddress();
    Serial.print("  integer: ");
    Serial.print(id, DEC);
    Serial.print(", hexidecimal: ");
    Serial.println(prettyprintAddressHex(id));

     // Get Sensor Information
    Serial.println("Get sensor information.");
    Serial.println(sensor.getSensorInfo());
    Serial.println();
    /* Next two functions NOT YET IMPLEMENTED
    // Get the sensor serial number
    Serial.println("\nGetting sensor serial number.");
    String SN = sensor.getSerialNumber();
    Serial.print("    Serial Number: ");
    Serial.println(SN);
    
    // Get the sensor's hardware and software version
    Serial.println("Getting sensor version numbers.");
    float hardwareV, softwareV;
    sensor.getVersion(hardwareV, softwareV);
    Serial.print("    Current Hardware Version: ");
    Serial.println(hardwareV);
    Serial.print("    Current Software Version: ");
    Serial.println(softwareV);
    */

    // Get Sensor Modbus Baud
    Serial.println("Get sensor modbus baud setting.");
    int16_t sensorBaud = sensor.getSensorBaud();
    Serial.print("  Baud: ");
    Serial.println(sensorBaud);

    // Get Sensor Modbus Parity
    Serial.println("Get sensor modbus parity setting.");
    String sensorParity = sensor.getSensorParity();
    Serial.print("    Parity: ");
    Serial.println(sensorParity);
    Serial.println();

    // Tell the sensor to start taking measurements
    Serial.println("Starting sensor measurements");
    success = sensor.startMeasurement();
    if (success) Serial.println("    Measurements started.");
    else Serial.println("    Failed to start measuring!");
    Serial.println();

    Serial.println("Waiting for sensor to stabilize..");
    Serial.print("    Stabilization time (ms): ");
    Serial.println(STABILIZATION_TIME);
    for (int i = (STABILIZATION_TIME+500)/1000; i > 0; i--) {  // +500 to round up
        Serial.print(i);
        delay (250);
        Serial.print(".");
        delay (250);
        Serial.print(".");
        delay (250);
        Serial.print(".");
        delay (250);
    }
    Serial.println("\n");


    // Print table headers
    switch (model) {
        case GPLP8: {
            // Variable Names
            Serial.print("Time(ms)");
            Serial.print("  M1   M2   M3   M4   M5   M6   M7   M8   |");
            Serial.print("  T1   T2   T3   T4   T5   T6   T7   T8   ");
            Serial.println("T9   T10  T11  T12  T13");
            // Variable Units
            Serial.print("ms       ");
            Serial.print("  %    %    %    %    %    %    %    %   |");
            Serial.print("  °C   °C   °C   °C   °C   °C   °C   °C   ");
            Serial.println("°C   °C   °C   °C   °C");
            break;
        }
        default: {
            Serial.println("Other sensors not yet implemented.");
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
            sensor.getTemperatureValues(T1, T2, T3, T4, T5, T6, T7, T8,
                                        T9, T10, T11, T12, T13);

            Serial.print(millis());
            Serial.print("     ");
            Serial.print(M1, 1);
            Serial.print("  ");
            Serial.print(M2, 1);
            Serial.print("  ");
            Serial.print(M3, 1);
            Serial.print("  ");
            Serial.print(M4, 1);
            Serial.print("  ");
            Serial.print(M5, 1);
            Serial.print("  ");
            Serial.print(M6, 1);
            Serial.print("  ");
            Serial.print(M7, 1);
            Serial.print(" ");
            Serial.print(M8, 1);
            Serial.print("  | ");
            Serial.print(T1, 1);
            Serial.print(" ");
            Serial.print(T2, 1);
            Serial.print(" ");
            Serial.print(T3, 1);
            Serial.print(" ");
            Serial.print(T4, 1);
            Serial.print(" ");
            Serial.print(T5, 1);
            Serial.print(" ");
            Serial.print(T6, 1);
            Serial.print(" ");
            Serial.print(T7, 1);
            Serial.print(" ");
            Serial.print(T8, 1);
            Serial.print(" ");
            Serial.print(T9, 1);
            Serial.print(" ");
            Serial.print(T10, 1);
            Serial.print(" ");
            Serial.print(T11, 1);
            Serial.print(" ");
            Serial.print(T12, 1);
            Serial.print(" ");
            Serial.print(T13, 1);
            Serial.println();
            break;
        }
        default: {
            Serial.println("Only GPLP-8 has been implemented.");
        }
    }
    // Delay between readings is built into the getInputRegisters() function
    // delay(MEASUREMENT_TIME);
}
