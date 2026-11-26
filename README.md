# CRC-Protected UART Command Interface

![gds](../../workflows/gds/badge.svg) ![docs](../../workflows/docs/badge.svg) ![test](../../workflows/test/badge.svg)

A digital ASIC for the [Tiny Tapeout](https://tinytapeout.com) SKY26e shuttle (SKY130, 1x1 tile). A host sends 5-byte command packets over a UART. The chip checks each packet with a CRC-8 in hardware, executes the command on a small register file (4 GPIO pins and an 8-bit PWM output), and replies with ACK or NAK. Corrupted or invalid packets are rejected and counted, so a bad command can never change the outputs.

Full description, register map, block diagram and implementation results: [docs/info.md](docs/info.md)

## Features

- UART receiver and transmitter (8N1), four selectable baud rates
- CRC-8 (polynomial 0x07) checked in hardware, one byte per clock
- Packet FSM with ACK (0x06) / NAK (0x15) replies
- Register file: GPIO output, GPIO direction, PWM duty, status
- 8-bit PWM generator and 4 bidirectional GPIO pins
- Bad-packet counter, receive timeout, and automatic resynchronization after junk bytes

## Packet format

`[0xA5][CMD][ADDR][DATA][CRC8]`

| CMD | Action |
|---|---|
| `0x01` | Write DATA to register ADDR |
| `0x02` | Read register ADDR (reply: ACK + value) |
| `0x03` | Ping (reply: ACK) |

Example: ping = `A5 03 00 00 BD`, reply = `06`.

## Pins

| Pin | Function |
|---|---|
| `ui[0]` | Serial input (rx) |
| `ui[2:1]` | Baud select: 00 = 115200, 01 = 57600, 10 = 230400, 11 = fast mode (simulation) |
| `uo[0]` | Serial output (tx) |
| `uo[1]` | Packet OK |
| `uo[2]` | Packet error |
| `uo[3]` | PWM output |
| `uo[4]` | Byte received pulse (debug) |
| `uio[3:0]` | GPIO, direction set by the GPIO_DIR register |

## Repository layout

| Path | Contents |
|---|---|
| `src/project.v` | All RTL: UART RX/TX, CRC-8, packet FSM, register file, PWM, top level |
| `test/test.py` | cocotb tests (host model, bad CRC, junk bytes, timeout, all baud rates) |
| `test/tb.v` | Simulation wrapper |
| `docs/info.md` | Datasheet |
| `info.yaml` | Tiny Tapeout project description and pinout |

## Run the tests

```bash
cd test
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
make -B
```

## Credits

This project was executed by Author 1, Author 2, Author 3, Author 4, and Shylashree N from RVCE.

We gratefully acknowledge the Center of Excellence (CoE) in Integrated Circuits and Systems (ICAS) and the Department of Electronics and Communication Engineering (ECE) for providing the necessary resources and guidance.

Special thanks to Dr. H V Ravish Aradhya (HoD - ECE), Dr. K R Usha Rani (Associate Dean - PG), Dr. K. S. Geetha (Vice Principal) and Dr. K. N. Subramanya (Principal) for their constant encouragement and support in facilitating this Tiny Tapeout SKY26e submission.
