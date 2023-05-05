This is a heavily modified version of the DIY_PRO_V3_7.ino sketch from
https://github.com/airgradienthq/arduino.

Important: This code is only for the DIY PRO PCB Version 3.7 that has a push
button mounted.

This code supports the original ESP8266 and an ESP32-C3 as the microcontroller
It supports the original CO2, PM, TVOC and temperature/humidity sensors.
In addition, it supports a BME280/BMP280 sensor for air pressure.

Build Instructions for the hardware:
https://www.airgradient.com/open-airgradient/instructions/diy-pro-v37/

Kits (including a pre-soldered version) are available:
https://www.airgradient.com/open-airgradient/kits/

The codes needs the following libraries installed:
- “WifiManager by tzapu, tablatronix” tested with version 2.0.11-beta
- “U8g2” by oliver tested with version 2.32.15
- "Sensirion I2C SGP41" by Sensation Version 0.1.0
- "Sensirion Gas Index Algorithm" by Sensation Version 3.2.1
- "SparkFun BME280" Version 2.0.9
- "ArduinoJson" Version 6.21.1
- "modbus-esp8266" Version 4.1.0

In addition, the AirGradient libraries are needed. At this point, the orignal
version at https://github.com/airgradienthq/arduino does _not_ work with this
code. It is suggested to use the `lalufu/merge` branch at
https://github.com/Lalufu/airgradienthq-arduino/tree/lalufu/merge instead.

At this time, the branch of the Airgradient library
has the following changes on top of the original code:

- Use the `modbus-esp8266` library for communication with the CO2 sensor
- Support for the ESP32-C3 microcontroller
- Support for returning a float reading for humidity.

Compared with the original sketch, this file has the following functional
changes:

- ESP32 support
- Support for sending data to MQTT, with configurable endpoints and topics
- Configurable HTTP endpoint for sending data via HTTP
- Support for BME/BMP280 sensors for air pressure measurement

Internal changes:

- Use of `ArduinoJSON` for generating and reading all JSON data
- Store configuraton data in LittleFS
- 4 line display with scrollable output for long lines

This code uses platformio as a build system, which will install all
dependencies automatically. Compiling with Arduino Studio is also possible,
but will require manual installation of dependencies.

CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
