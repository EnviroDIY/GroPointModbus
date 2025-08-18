# Changing the parity <!-- {#extras_change_parity} -->

This sketch uses hardware serial to connect with GroPoint Profile and change default modbus settings from 19200 8E1 to 9600 8N1.

> [!note]
> GroPoint Profile sensor default Modbus communication settings are 19200 Baud, 8 Bits, Even Parity, one Stop Bit (8-E-1).
> See p35 of manual. 8-N-1 (no parity) is the most common configuration for serial communications.

> [!warning]
> Neither SoftwareSerial, AltSoftSerial, nor NeoSoftwareSerial will support either even or odd parity!
