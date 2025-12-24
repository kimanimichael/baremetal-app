# BAREMETAL ARM PRINTF
This is a bring up of a simple baremetal application using STM32 upto using printf for debugging. This project has been developed on a Linux operating system. Development has not been tested on any other operating system.

## Hardware Requirements

1. [NUCLEO-F429ZI board](https://www.st.com/en/evaluation-tools/nucleo-f429zi.html#samplebuy-scroll)
2. USB-X to USB 2.0 Micro B cable

Although this project is built for the NUCLEO-F429ZI development board, users can modify it, if they can, to fit other development boards


## Installation
### Prerequisites
The following are required to build this program:
1. [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/gnu-rm)
2. [Python](https://www.python.org/downloads/) NB: If you do not have python installed, you can use the legacy build option described at the bottom of this file.

The following are required to flash this program onto a microcontroller
1. [stlink](https://github.com/stlink-org/stlink)

Run the following command to install python requirements:
```console
pip install -r requirements.txt
```

## Build
### Build without a bootloader
```console
python3 build.py build-application --compile
```
### Rebuild the bootloader and build the app
```console
python3 build.py build-application --bootloader --compile
```
## Flash

Connect the board to your PC and run this in the terminal
```console
python3 build.py build-application --flash
```
# Compile and Flash
```console
python3 build.py build-application --compile --flash
```

## Legacy Build
This build does not require the use of python
```console
./build.sh
```
## Legacy Build and Flash
```console
./build_flash.sh
```

