This project uses an ESP32 microcontroller to emulate a CD changer for VW radios, with support for Bluetooth A2DP audio streaming. Music can be streamed wirelessly from a smartphone and controlled directly from the radio head unit (e.g., skipping tracks).

To achieve high-quality sound, the design includes a PCM5102A DAC, which supports up to 32-bit resolution. This ensures excellent audio performance.

Voltage note:
It is recommended to use a voltage divider or step-down converter for the clock signal from the radio (5 V → 3.3 V), as the ESP32 is not 5 V-tolerant. Data In and Data Out lines operate at safe, lower voltage levels for the ESP32.

Example Pin Connections

Radio → ESP32
------------------
DATA IN → GPIO21

DATA OUT → GPIO18

CLOCK → GPIO33

PCM5102A → ESP32
------------------
BCK → GPIO26

WS → GPIO25

DATA OUT → GPIO22

Features
------------------
Wireless Bluetooth A2DP streaming

Two-way communication with VW radio

Control playback from the radio (e.g., skip tracks)

Tested and working with VW Gamma V radio

To Do
------------------
Create and make PCB board
