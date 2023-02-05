# Summary
The DIM board is an RS485 to UART/USB interface module. It can be used as a debug board for other DINFox modules, with the following features:
* Optional RS bus **power** supply.
* USB and RS bus voltage **measurements**.
* Dynamic **addressed or direct** mode.
* RS485 **node scanning**.

# Hardware
The board was designed on **Circuit Maker V2.0**. Hardware documentation and design files are available @ https://circuitmaker.com/Projects/Details/Ludovic-Lesur/DIMHW1-1

# Embedded software

## Environment
The embedded software was developed under **Eclipse IDE** version 2019-06 (4.12.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target
The boards are based on the **STM32L011F4P3** of the STMicroelectronics L0 family microcontrollers. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected target.

## Structure
The project is organized as follow:
* `inc` and `src`: **source code** split in 5 layers:
    * `registers`: MCU **registers** adress definition.
    * `peripherals`: internal MCU **peripherals** drivers.
    * `utils`: **utility** functions.
    * `components`: external **components** drivers.
    * `applicative`: high-level **application** layers.
* `startup`: MCU **startup** code (from ARM).
* `linker`: MCU **linker** script (from ARM).
