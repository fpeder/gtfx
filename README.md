# GTFX

FPGA-based guitar effects processor implemented in SystemVerilog for the Xilinx Arty A7-100T. Processes 24-bit/48kHz audio in real-time through a configurable chain of four classic effects.

## Signal Chain

```
I2S RX -> [Tremolo] -> [Phaser] -> [Chorus] -> [Delay] -> I2S TX
```

Each effect can be independently bypassed via hardware switches.

## Effects

| Effect | Style | Key Parameters |
|--------|-------|----------------|
| **Tremolo** | Optical/Fender | Rate (0.5-12 Hz), depth, waveform (sine/triangle) |
| **Phaser** | MXR Phase 90 | Speed (0.5-5 Hz), 4-stage all-pass, optional feedback |
| **Chorus** | BOSS CE-5 | Stereo output, rate, depth, 2-band EQ |
| **Delay** | Boss DD-3 | Up to 1s, tone control, feedback |

## Hardware

- **Board:** Arty A7-100T (xc7a100tcsg324-1)
- **Audio:** I2S interface on Pmod JA (24-bit, 48 kHz)
- **Control:** UART (115200 baud) for parameter updates, 4 switches for effect bypass
- **Clocks:** 100 MHz system, 12.288 MHz audio (via Clock Wizard)

### Pin Mapping

| Function | Pmod/Port | Signals |
|----------|-----------|---------|
| I2S TX | JA upper | MCLK, LRCLK, SCLK, DATA |
| I2S RX | JA lower | MCLK, LRCLK, SCLK, DATA |
| UART | USB | TX (A9), RX (D10) |
| Switches | SW0-SW3 | Tremolo, Phaser, Chorus, Delay enable |
| LEDs | LD0-LD3 | Mirror switch states |

## Prerequisites

- Xilinx Vivado Design Suite
- Arty A7-100T board
- I2S audio codec connected to Pmod JA

## Build

```bash
make proj        # Create Vivado project and generate Clock Wizard IP
make bitstream   # Run synthesis, implementation, and bitstream generation
make prog        # Program the FPGA
make sim         # Run testbenches in XSim
make open        # Open project in Vivado GUI
make clean       # Remove build artifacts
```

## UART Command Interface

Connect via serial terminal at 115200 baud. The command processor accepts hex-encoded parameter updates for each effect. Responses are echoed with `OK` or `Err` status.

```
Arty> <command>
OK
```

## Project Structure

```
src/           SystemVerilog source modules
  top.sv         Top-level module and signal chain
  tremolo.sv     Tremolo effect
  phaser.sv      Phaser effect
  chorus.sv      Chorus effect
  dd3.sv         Digital delay effect
  i2s2.sv        I2S protocol driver
  uart.sv        UART transceiver
  cmd_proc.sv    Command parser and parameter registers
  audio.sv       Audio interface wrapper
  volume.sv      Volume control
sim/           Testbenches
tcl/           Vivado build scripts
constrs/       XDC pin constraints
```

## License

Apache License 2.0
