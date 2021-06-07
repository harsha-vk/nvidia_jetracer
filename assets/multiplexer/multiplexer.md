# Multiplexer

**Arduino Pro Mini 3.3v** is used as multiplexer instead of PWM Servo Module and RC Servo multiplexer.

- **Serial UART** is used to communicate with Nvidia Jetson Nano.

## Perfboard Wiring

![Muliplexer](multiplexer.jpg)

## UART Connection for Nvidia Jetson Nano

Nvidia Jetson Nano | Arduino Pro Mini
------------------ | ----------------
J41 Pin 8 (TXD) -> | RX
J41 Pin 10 (RXD) -> | TX
J41 Pin 6 (GND) -> | GND

## Code

Arduino code for multiplexer is [here](multiplexer.ino).
