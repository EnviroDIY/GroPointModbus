# Changing Modbus Settings <!-- {#example_change_modbus_settings} -->

This sketch uses hardware serial to connect with GroPoint Profile and change the default modbus settings from 19200 8E1 to 9600 8N1.

You will probably need to use this example to prepare a new sensor to communicate with most Arduino streams that only support 8N1.

**NOTE** GroPoint Profile sensor default Modbus communication settings are 19200 Baud, 8 Bits, Even Parity, one Stop Bit (8-E-1); see p35 of manual.
Neither SoftwareSerial, AltSoftSerial, nor NeoSoftwareSerial will support either even or odd parity!

8-N-1 (no parity) is the most common configuration for serial communications.

This sketch depends on the GropointModbus library and also loosely on the SensorModbusMaster library via it's example sketch: SensorModbusMaster/examples/readWriteRegister/readWriteRegister.ino

_______

[//]: # ( @section example_change_modbus_settings_pio_config PlatformIO Configuration )

[//]: # ( @include{lineno} ChangeModbusSettings/platformio.ini )

[//]: # ( @section example_change_modbus_settings_code The Complete Code )

[//]: # ( @include{lineno} ChangeModbusSettings/ChangeModbusSettings.ino )
