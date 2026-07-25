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
| [DIM HW1.1](https://365.altium.com/files/D0E36E2E-D212-4D50-BA3B-173AD1895161) | Add transistor on RS485 bus voltage measurement to save energy consumption. Add jumper on RS485 bus power supply. | `DIM` | `HW1_1` | :white_check_mark: |
| [RS485-BRIDGE HW1.0](https://365.altium.com/files/87E26F6B-C53E-4FF7-9692-B11B183856CE) | Upgrade based on STM32G4 MCU. | `RS485_BRIDGE` | `HW1_0` | :white_check_mark: |

# Embedded software

## Environment

The firmware is developed under **Eclipse IDE** and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

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

## Flash

### Preparation

* **Build** the desired version (with IDE or `cmake`) or **download** a specific [firmware release](https://github.com/Ludovic-Lesur/rs485-bridge/releases) (expand the `Assets` menu, download the corresponding artifact and extract the binary files from the `zip`).
* Connect the flashing tool to the **P2** (DIM) or **P5** (RS485-BRIDGE) **connector** located in the corner of the PCB (standard SWD pinout).

### ST-Link on Nucleo board

* Make sure that the ST-LINK/NUCLEO jumpers (generally designated by **CN2**) are not fitted, in order to **select the external programming connector** instead of the internal MCU.
* An **MSC disk** named `NODE_XXXXXX` should be mounted by the system after USB plugging. If not, download the [ST Cube Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) software which will install the required drivers. If the MSC disk is still not mounted, follow the ST-Link probe procedure thereafter.
* **Copy/paste** or **click/drop** the `bin` file into the disk.

### ST-Link probe

* Download the [ST Cube Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) software.
* Launch the software (it might be necessary to run it as **root** or to install specific **USB rules** for the probe to be recognized).
* In the right panel, select `ST-LINK` and click `Connect`.
* Click on the `Open file` tab and select the `hex` file to flash.
* Click on the `Download` button.
* Perform a **memory check** with the `Verify` button located under the `Download` button menu.
* If the operation completed successfully, click on `Disconnect` in the right panel.

### Segger J-Link probe

* Download the [Segger J-Link](https://www.segger.com/downloads/jlink/) software.
* Launch the `JFlashLite` tool.
* Set target device to **STM32L031F6** (DIM) or **STM32G441KB** (RS485-BRIDGE), target interface to **SWD**, speed to **4000kHz** and click `OK`.
* Open the `hex` file to flash.
* Click on the `Program Device` button.

### Final steps

* Check thanks to the `ATI` command if the board has properly rebooted with the **expected firmware version**.
