/*****************************************************************************
testing.ino

This prints basic meta-data about a sensor to the first serial port and then
begins taking measurements from the sensor.

The sensor model and address can easily be modified to use this sketch with any
GroPoint Profile modbus sensor.
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
gropointModel model = GPLP_2;  // The sensor model number

// Define the sensor's modbus address, or SlaveID
// NOTE: The GroPoint User Manual presents SlaveID and registers as integers (decimal),
// whereas EnviroDIY and most other modbus systems present it in hexadecimal form.
// Use an online "HEX to DEC Converter".
byte modbusAddress = 0x01;  // Yosemitech ships sensors with a default ID of 0x01.

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
//  Data Logging Options
// ==========================================================================
const int32_t serialBaud = 115200;  // Baud rate for serial monitor

// Define pin number variables
const int sensorPwrPin = 11;  // The pin sending power to the sensor
const int adapterPwrPin = 22;  // The pin sending power to the RS485 adapter
const int DEREPin = -1;   // The pin controlling Driver Enable and Recieve Enable
                          // on the RS485 adapter, if applicable (else, -1)
                          // Setting HIGH enables the driver (arduino) to send text
                          // Setting LOW enables the receiver (sensor) to send text
// Pins for `SoftwareSerial` only. Not for `AltSoftSerial`, which uses fixed pins.
const int SSRxPin = 13;  // Receive pin for software serial (Rx on RS485 adapter)
const int SSTxPin = 14;  // Send pin for software serial (Tx on RS485 adapter)

// Construct software serial object for Modbus
#if defined __AVR__
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


// ==========================================================================
// Working Functions
// ==========================================================================
// A function for pretty-printing the Modbuss Address, from ModularSensors
String sensorLocation(byte _modbusAddress) {
    String sensorLocation = F("0x");
    if (_modbusAddress < 0x10) sensorLocation += "0";
    sensorLocation += String(_modbusAddress, HEX);
    return sensorLocation;
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

    Serial.begin(serialBaud); // Main serial port for debugging via USB Serial Monitor

    // Setup Modbus serial stream
    #if defined ESP8266
        modbusSerial.begin(9600, SWSERIAL_8N1, SSRxPin, SSTxPin, false, 256); // The modbus serial stream - Baud rate MUST be 9600.
    #else
        modbusSerial.begin(9600);  // The modbus serial stream - Baud rate MUST be 9600.
    #endif

    // Start up the sensor
    sensor.begin(model, modbusAddress, &modbusSerial, DEREPin);

    // Turn on debugging
    #ifdef DEBUG
        sensor.setDebugStream(&Serial);
    #endif

    // Start up note
    Serial.print("\nGroPoint Profile Soil Moisture and Temperature");
    Serial.print(sensor.getModel());

    // Allow the sensor and converter to warm up
    Serial.println("\nWaiting for sensor and adapter to be ready.");
    Serial.print("    Warm up time (ms): ");
    Serial.println(WARM_UP_TIME);
    delay(WARM_UP_TIME);

    // Confirm Modbus Address 
    Serial.println("\nSelected modbus address:");
    Serial.print("    integer: ");
    Serial.print(modbusAddress, DEC);
    Serial.print(", hexidecimal: ");
    Serial.println(sensorLocation(modbusAddress));

    Serial.println("Discovered modbus address.");
    Serial.print("    integer: ");
    byte id = sensor.getSlaveID();
    Serial.print(id, DEC);
    Serial.print(", hexidecimal: ");
    // Serial.print(id, HEX);
    Serial.println(sensorLocation(id));

    // if (id != modbusAddress){
    //     Serial.print("Updating sensor modbus address to: ");
    //     modbusAddress = id;
    //     Serial.println(sensorLocation(modbusAddress));
    //     Serial.println();
    //     // Restart the sensor
    //     sensor.begin(model, modbusAddress, &modbusSerial, DEREPin);
    //     delay(1500);
    // };

    // Get the sensor serial number
    Serial.println("\nGetting sensor serial number.");
    String SN = sensor.getSerialNumber();
    Serial.print("    Serial Number: ");
    Serial.println(SN);
    
    // // Get the sensor's hardware and software version
    // Serial.println("Getting sensor version numbers.");
    // float hardwareV, softwareV;
    // sensor.getVersion(hardwareV, softwareV);
    // Serial.print("    Current Hardware Version: ");
    // Serial.println(hardwareV);
    // Serial.print("    Current Software Version: ");
    // Serial.println(softwareV);

    // // Get the sensor calibration equation / status (pH only)
    // switch(model)
    // {
    //     case Y532:          // pH, calibration status
    //     {
    //         Serial.println("Getting sensor calibration status.");
    //         byte status = sensor.pHCalibrationStatus();
    //         Serial.print("    Status: 0x0");
    //         Serial.println(status, HEX);
    //     }
    //     case Y4000:
    //     {
    //         Serial.println("For Y4000, use YosemiTech software to get calibration parameters.");
    //         break;
    //     }
    //     default:  // Get the sensor's current calibration values
    //     {
    //         Serial.println("Getting sensor calibration equation.");
    //         float Kval = 0;
    //         float Bval = 0;
    //         sensor.getCalibration(Kval, Bval);
    //         Serial.print("    Current Calibration Equation: final = ");
    //         Serial.print(Kval);
    //         Serial.print("*raw + ");
    //         Serial.println(Bval);
    //     }
    //     Serial.println();
    // }


    // Tell the sensor to start taking measurements
    Serial.println("Starting sensor measurements");
    success = sensor.startMeasurement();
    if (success) Serial.println("    Measurements started.");
    else Serial.println("    Failed to start measuring!");

    Serial.println("Waiting for sensor to stabilize..");
    Serial.print("    Stabilization time (ms): ");
    Serial.println(STABILIZATION_TIME);
    for (int i = (STABILIZATION_TIME+500)/1000; i > 0; i--)  // +500 to round up
    {
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
    switch (model)
    {
        case GPLP_2:
        {
            Serial.print("Time(ms) ");
            // Serial.println(sensor.getParameter());
            Serial.println(" M1,  M2,  T1,  T2,  T3,  T4");
            Serial.print("ms       ");
            // Serial.println(sensor.getUnits());
            Serial.println(" % ,  % ,  *C,  *C,  *C,  *C");
            break;
        }
        default:
        {
        }
    }
}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop()
{
    // send the command to get the values
    switch (model)
    {
        case GPLP_2:
        {
            int16_t M1,  M2,  T1,  T2,  T3,  T4 = -9999;

            sensor.getValues(M1,  M2);

            Serial.print(millis());
            Serial.print("    ");
            Serial.print(M1, 3);
            Serial.print("  ");
            Serial.print(M2, 3);
            Serial.println();
            break;
        }
        default:
        {
        }
    }

    // Delay between readings
    delay(MEASUREMENT_TIME);
}
