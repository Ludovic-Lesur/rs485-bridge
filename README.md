# Description

The **RS485-BRIDGE** board is an RS485 to UART/USB interface, which can be used as a debug board to monitor the data transfered on an **RS485 bus**. It embeds the following features:

* Optional RS485 bus **power** supply.
* USB and RS485 bus voltages **measurements**.
* **UNA systems** support including nodes scanning.

# Hardware

The boards were designed on **Circuit Maker V2.0**. Below is the list of hardware revisions:

| Hardware revision | Description | `cmake_board` | `cmake_hw_version` | Status |
|:---:|:---:|:---:|:---:|:---:|
| [DIM HW1.0](https://365.altium.com/files/3F3B832D-FFF6-457E-A74F-EDA6BAF90587) | Initial version. | `DIM` | `HW1_0` | :x: |
| [DIM HW1.1](https://365.altium.com/files/D0E36E2E-D212-4D50-BA3B-173AD1895161) | Add transistor on VRS voltage measurement to save energy consumption. Add jumper on RS485 bus power supply. | `DIM` | `HW1_1` | :white_check_mark: |
| [RS485-BRIDGE HW1.0](https://365.altium.com/files/87E26F6B-C53E-4FF7-9692-B11B183856CE) | Upgrade based on STM32G4 MCU. | `RS485_BRIDGE` | `HW1_0` | :white_check_mark: |

# Embedded software

## Environment

The embedded software is developed under **Eclipse IDE** version 2024-09 (4.33.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

> [!WARNING]
> To compile any version under `sw3.0`, the `git_version.sh` script must be patched when `sscanf` function is called: the `SW` prefix must be replaced by `sw` since Git tags have been renamed in this way.

## Target

The boards are based on the **STM32L031F6P6** and **STM32G441KBU6**  microcontrollers of the STMicroelectronics L0/G4 families. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected board version.

## Structure

The project is organized as follow:

* `drivers` :
    * `device` : MCU **startup** code and **linker** script.
    * `registers` : MCU **registers** address definition.
    * `peripherals` : internal MCU **peripherals** drivers.
    * `mac` : **medium access control** driver.
    * `components` : external **components** drivers.
    * `utils` : **utility** functions.
* `middleware` :
    * `analog` : High level **analog measurements** driver.
    * `cli` : **AT commands** implementation.
    * `node` : **UNA** nodes interface implementation.
    * `power` : Board **power tree** manager.
* `application` : Main **application**.

## Build

The project can be compiled by command line with `cmake`.

```bash
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="script/cmake-arm-none-eabi/toolchain.cmake" \
      -DTOOLCHAIN_PATH="<arm_none_eabi_gcc_path>" \
      -DRS485_BRIDGE_BOARD="<cmake_board>" \
      -DRS485_BRIDGE_HW_VERSION="<cmake_hw_version>" \
      -DRS485_BRIDGE_MODE_LOW_BAUD_RATE=ON \
      -DRS485_BRIDGE_ENABLE_UNA_AT=ON \
      -DRS485_BRIDGE_ENABLE_UNA_R4S8CR=ON \
      -G "Unix Makefiles" ..
make all
```
