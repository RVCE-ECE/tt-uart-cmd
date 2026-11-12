<!---
This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.
--->

## How it works

A host sends 5-byte packets over UART: start byte 0xA5, command, address, data, and a CRC-8. The chip rebuilds each byte, checks the CRC, runs the command (write or read a register, or ping), and replies with ACK (0x06) or NAK (0x15). Registers control four GPIO pins, a PWM output, and a bad-packet counter.

## How to test

Send a valid packet and check that `pkt_ok` goes high and an ACK byte comes out on `tx`. Send a packet with a wrong CRC and check that `pkt_err` goes high and a NAK comes back. Simulation tests are in `test/test.py` (run `make -B` in `test/`).

## External hardware

A USB-to-serial adapter (3.3 V) or the Tiny Tapeout demo board microcontroller can drive `rx` and read `tx`.
