# Getting Values <!-- {#example_get_values} -->

This prints basic meta-data about a sensor to the first serial port and then begins taking measurements from the sensor.

The sensor model and address can easily be modified to use this sketch with any GroPoint Profile modbus sensor.
**NOTE** GroPoint Profile sensor default Modbus communication settings are 19200 Baud, 8 Bits, Even Parity, one Stop Bit (8-E-1); see p35 of manual.
Neither SoftwareSerial, AltSoftSerial, nor NeoSoftwareSerial will support either even or odd parity!

8-N-1 (no parity) is the most common configuration for serial communications.

_______

[//]: # ( @section example_get_values_pio_config PlatformIO Configuration )

[//]: # ( @include{lineno} GetValues/platformio.ini )

[//]: # ( @section example_get_values_code The Complete Code )

[//]: # ( @include{lineno} GetValues/GetValues.ino )
