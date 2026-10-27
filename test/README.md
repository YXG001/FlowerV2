# Firmware tests

This directory is reserved for PlatformIO unit tests (`pio test`).

There are **no automated tests yet**. The firmware has not been confirmed on physical ESP32-S3 hardware.

Future tests should cover:
- Seed and kettle switch hysteresis
- Potentiometer 0/1/2 position boundaries
- Windmill pulse counting, RPM and stopped-state timeout
- The seven-field serial packet format

Test-only code should not interfere with the normal Max/MSP serial data stream.
