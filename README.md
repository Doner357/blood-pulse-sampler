# Blood Pulse Sampler

Firmware for a Raspberry Pi Pico 2 W based blood pulse sampling device. The application samples three pressure channels, controls pneumatic pump/valve pairs for the Cun, Guan, and Chi positions, and exposes control/status/sample data over a custom BLE GATT service.

## Target Platform

- Board: Raspberry Pi Pico 2 W (`pico2_w`)
- SDK: Raspberry Pi Pico SDK 2.1.1 configuration is expected by the CMake project
- RTOS: FreeRTOS Kernel, included as a Git submodule at `freertos/freertos-kernel`
- Language standards: C17 and C++23
- Build system: CMake

## Features

- BLE peripheral using BTstack and the CYW43 wireless stack.
- FreeRTOS tasks for BLE service, sampler service, and three pressure controllers.
- Three pressure sensor channels mapped to Cun, Guan, and Chi.
- TCA9548A I2C multiplexer support for reading multiple XGZP6857D pressure sensors.
- PWM pump/valve control for each pressure channel.
- BLE commands for starting sampling, stopping sampling, setting pressure targets, and resetting pressure targets.

## Repository Layout

```text
.
|-- blood-pulse-sampler.cpp       # Application entry point
|-- CMakeLists.txt                # Top-level Pico SDK project
|-- bps/
|   |-- ble_service/              # BLE service and custom GATT server
|   |-- sampler_service/          # Sampler state machine
|   |-- logger/                   # Logging helpers
|   |-- common.hpp                # Shared command, status, and sample types
|   `-- queue.hpp                 # FreeRTOS queue wrappers
|-- freertos/
|   |-- CMakeLists.txt
|   |-- FreeRTOSConfig.h
|   `-- freertos-kernel/          # FreeRTOS Kernel submodule
`-- pico_sdk_import.cmake
```

## Hardware Connections

### Pressure Sensors

The firmware reads three XGZP6857D pressure sensors through a TCA9548A I2C multiplexer.

| Signal | Pico GPIO | Notes |
| --- | ---: | --- |
| I2C0 SDA | GPIO4 | 400 kHz, pull-up enabled in firmware |
| I2C0 SCL | GPIO5 | 400 kHz, pull-up enabled in firmware |
| TCA9548A address | `0x70` | Default address |
| XGZP6857D address | `0x6D` | Sensor address behind the mux |

| Position | Mux channel |
| --- | ---: |
| Cun | 0 |
| Guan | 1 |
| Chi | 2 |

### Pneumatic PWM Outputs

Each position uses one PWM channel for a pump and one PWM channel for a valve.

| Position | Pump GPIO | Valve GPIO |
| --- | ---: | ---: |
| Cun | GPIO6 | GPIO7 |
| Guan | GPIO8 | GPIO9 |
| Chi | GPIO10 | GPIO11 |

PWM is configured with a clock divider of `200.0` and a wrap value of `20000`.

## BLE Interface

The device advertises the local name `BSP`. Its GAP device name characteristic is `PULSE_SAMPLER_SERVER`.

### Custom Service

| Item | UUID | Properties |
| --- | --- | --- |
| Pulse Sampler service | `652C47C0-C653-41BC-8828-30200EF3350A` | Primary service |
| Command Packet | `652C47C1-C653-41BC-8828-30200EF3350A` | Write |
| Machine Status Packet | `652C47C2-C653-41BC-8828-30200EF3350A` | Read, notify |
| Pulse Data Packet | `652C47C3-C653-41BC-8828-30200EF3350A` | Read, notify |

### Command Packet

The command characteristic is a 13-byte packet.

| Offset | Size | Type | Description |
| ---: | ---: | --- | --- |
| 0 | 1 | `uint8_t` | Command type |
| 1 | 4 | `float32` | Cun target pressure in Pa, only used by `SetPressure` |
| 5 | 4 | `float32` | Guan target pressure in Pa, only used by `SetPressure` |
| 9 | 4 | `float32` | Chi target pressure in Pa, only used by `SetPressure` |

Command type values:

| Value | Command |
| ---: | --- |
| `0x01` | Stop sampling |
| `0x02` | Start sampling |
| `0x03` | Set pressure targets |
| `0x04` | Reset pressure targets to zero |

Multi-byte values should be encoded as little-endian values when sent from BLE clients.

### Machine Status Packet

The machine status characteristic is a 1-byte packet.

| Value | Status |
| ---: | --- |
| `0x01` | Idle |
| `0x02` | Sampling |
| `0x03` | Setting pressure |

### Pulse Data Packet

The pulse data characteristic is a 20-byte packet.

| Offset | Size | Type | Description |
| ---: | ---: | --- | --- |
| 0 | 8 | `uint64_t` | Pico absolute timestamp |
| 8 | 4 | `float32` | Cun pressure in Pa |
| 12 | 4 | `float32` | Guan pressure in Pa |
| 16 | 4 | `float32` | Chi pressure in Pa |

Pulse data is serialized as little-endian values.

## Build Prerequisites

Install or configure:

- Raspberry Pi Pico SDK
- CMake 3.13 or newer
- A supported Arm GNU toolchain
- Python 3
- Git submodules

If you are not using the Raspberry Pi Pico VS Code extension, set `PICO_SDK_PATH` so `pico_sdk_import.cmake` can find the SDK.

## Setup

Clone with submodules:

```sh
git clone --recurse-submodules <repository-url>
cd blood-pulse-sampler
```

If the repository was cloned without submodules:

```sh
git submodule update --init --recursive
```

## Build

Configure and build with CMake:

```sh
cmake -S . -B build -DPICO_BOARD=pico2_w -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

The build produces Pico firmware outputs under `build/`, including `blood-pulse-sampler.uf2`.

## Flash

1. Hold the Pico 2 W `BOOTSEL` button while connecting it over USB.
2. Copy `build/blood-pulse-sampler.uf2` to the mounted `RPI-RP2` drive.
3. The board reboots and starts the firmware.

## Runtime Flow

At startup, the firmware:

1. Initializes logging.
2. Initializes the BLE GATT server and starts advertising.
3. Samples initial pressure baselines for Cun, Guan, and Chi.
4. Initializes the pneumatic controllers.
5. Connects queues between the BLE service and sampler service.
6. Starts the BLE, sampler, and pressure controller FreeRTOS tasks.

The sampler starts in `Idle`. A BLE `StartSampling` command switches it to `Sampling`, where pressure samples are forwarded to BLE notifications. A `SetPressure` command switches it to `Setting pressure`, drives the pneumatic controllers until all three channels report stable, and then returns to `Idle`.

## Development Notes

- Debug builds enable USB stdio and extra compiler warnings.
- UART stdio is disabled in the current top-level CMake configuration.
- The checked-in GATT files are under `bps/ble_service/gatt_server/`.
- Pressure readings are baseline-corrected and clamped to zero before being reported.
- The pressure controller turns pump and valve PWM off if current pressure exceeds `90000 Pa`.
