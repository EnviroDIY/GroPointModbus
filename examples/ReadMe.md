# Examples using the GroPoint Modbus Library<!--! {#page_the_examples} -->

These example programs demonstrate how to use the GroPoint Modbus library.

___

<!--! @if GITHUB -->

- [Examples using the GroPoint Modbus Library](#examples-using-the-gropoint-modbus-library)
  - [Getting Sensor Values](#getting-sensor-values)
  - [Changing Modbus Serial Settings](#changing-modbus-serial-settings)

<!--! @endif -->

<!--! @tableofcontents -->

<!--! @m_footernavigation -->

## Getting Sensor Values<!--! {#examples_get_values} -->

This prints basic meta-data about a sensor to the first serial port and then begins taking measurements from the sensor.

- [Instructions for the get values example](https://envirodiy.github.io/GroPointModbus/example_get_values.html)
- [The get values example on GitHub](https://github.com/EnviroDIY/GroPointModbus/tree/master/examples/GetValues)

## Changing Modbus Serial Settings<!--! {#examples_change_modbus_settings} -->

This sketch uses hardware serial to connect with GroPoint Profile and change default modbus settings from 19200 8E1 to 9600 8N1.

You will probably need to use this example to prepare a new sensor to communicate with most Arduino streams that only support 8N1.

- [Instructions for the changing settings example](https://envirodiy.github.io/GroPointModbus/example_change_modbus_settings.html)
- [The display values example on GitHub](https://github.com/EnviroDIY/GroPointModbus/tree/master/examples/ChangeModbusSettings)

<!--! @m_innerpage{example_get_values} -->
<!--! @m_innerpage{example_change_modbus_settings} -->
