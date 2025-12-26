# PONG with FPGA and ESP32

## Overview
This project implements a classic Pong game using an Intel MAX10 FPGA for
real-time VGA rendering, while delegating game logic computation to an ESP32.
The FPGA receives object coordinates via UART and renders paddles and the ball
at 640x480 @ 60Hz.

<img width="1639" height="1529" alt="Pong Sketch_bb" src="https://github.com/user-attachments/assets/a9a4c2fe-7392-4550-9ec4-cdb43907a40e" />

## System Architecture
- FPGA (MAX10)
  - VGA signal generation (RGB + HS/VS)
  - UART packet parsing
  - Real-time rendering pipeline
- ESP32
  - Game logic (ball physics, paddle movement)
  - Periodic coordinate transmission via UART (115200 bps)

## UART Packet Format
```
0xA5,
p1_y_hi, p1_y_lo,
p2_y_hi, p2_y_lo,
ball_x_hi, ball_x_lo,
ball_y_hi, ball_y_lo,
0x5A
```
- Header/Footer chosen as bitwise inverses for robustness
- All coordinates clamped on FPGA side to visible range

## VGA Output
- Resolution: 640x480 @ 60Hz
- Pixel clock: 25 MHz (derived from 50 MHz)
- Color depth: 4-bit RGB (4096 colors)

## Key Design Decisions
- **HW/SW separation**: FPGA handles timing-critical rendering, ESP32 handles logic
- **Non-blocking rendering**: VGA pipeline independent from UART reception
- **Clamping functions** prevent overflow and underflow artifacts

## Build
- Toolchain: Quartus Prime
- Target board: Intel MAX10
- UART baud rate: 115200

## Pin Planner
<img width="2534" height="696" alt="pin_planner" src="https://github.com/user-attachments/assets/763d1c4d-fe21-4537-b7fc-e88378b8c7c6" />
The pin configuration follows the MAX10 VGA DAC layout, separating RGB signals
across I/O banks to satisfy voltage and timing constraints.

- RGB channels mapped to Bank 2 and Bank 3 for signal integrity
- HS/VS signals placed near VGA connector pins
- UART RX isolated to reduce switching noise
- All outputs use 3.3V LVTTL standard

This layout reflects practical constraints rather than arbitrary assignment.

## Demo
[https://youtu.be/8N_bbvLrMmY](https://youtu.be/8N_bbvLrMmY)

## Contributors
This project was developed collaboratively.
Detailed role attribution will be added.

## Development Notes
This project was developed as a learning-focused hardware/software co-design
exercise. An AI tool (ChatGPT) was used as a reference for explanation,
documentation refinement, and design discussion, while all implementation and
debugging decisions were made by the author(s).
