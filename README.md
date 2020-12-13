# RE:Flex Dance Panel v2 Firmware

## Source

This repository contains source files for the firmware of the RE:Flex Dance Panel board. I suggest the cross-platform editor [Visual Studio Code](https://code.visualstudio.com/) to work with this project. The environment used in this project can be configured using [this tutorial](https://hbfsrobotics.com/blog/configuring-vs-code-arm-development-stm32cubemx) as a template, without the need for the 'STM32 Workspace Setup' or 'CubeMX Blink Example' sections.

- **panel-board.ioc / .mxproject** - These files are projects files for [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html). They are used in generating initial skeleton code of the project if a new project is desired from the base requirements.
- **Makefile / STM32F334K8Tx_FLASH.ld / startup_stm32f334x8.s** - The makefile / linker script build and correctly flash the executable to the microcontroller. The startup script will initialize the microcontroller and its peripherals on boot, and will then jump to the main code execution.
- **STM32F3x4.svd** - This file contains a list of register maps and device information for the microcontroller. It allows the cortex-debug extension to monitor the microcontroller's internal values for the sake of debugging.
- **Src/Inc/Drivers Folders** - These are the source code files for the project. Everything in here is the meat of the project, driving peripherals and defining core behaviour. All code is eventually built into an executable that runs solely on the microcontroller in the Panel board.
- **.vscode** - This folder contains files for Visual Studio Code's plugins to build, debug and flash the project. I've included my setup as an example, however your files here may vary from mine depending on your build environment.

## Release / Firmware Programming

The release contains firmware to program the RE:Flex Dance Panel board. At current, this is best accomplished via an [ST-Link/V2 programmer](https://www.st.com/en/development-tools/st-link-v2.html). You can check the panel boards pinout to connect the device for flashing. The tutorial listed above also provides some methods for making/flashing the firmware via hotkeys in VS Code. 

## Future Improvements

- Implementation of UART firmware update mechanism (along with the necessary board changes to the UART transceivers, and the python interface) will eliminate the need for the ST-Link programmer, and make the board update-able via the I/O board. This would improve project accessibility for end users.
- When the panel boards have DIP switches for board addressing, reading these and being able to receive data addressed to the uniquely addressed panel would be useful.
- A new UART addressing method will be required in order to daisy chain boards together. Multiprocessor mode on STM32 devices seems like a good candidate.
- Being able to turn light data on/off in firmware would be helpful for debugging, and for future revisions of panel boards without LEDs.
- The UART bus system has a limited command set. More commands could be implemented for testing, debugging, jumping into the UART programmer, resetting the boards, etc.
- Currently if the board receives data to display on the LEDs, and the connection is then removed, the LEDs will remain active. This is a bit of a power waste and an eyesore. The panel board could overcome this by checking if it has received by implementing a time out if data hasn't been sent by the PC in some amount of time (maybe a second or so). Then the timeout could trigger an LED update to turn all LEDs off.
- EEPROM emulation for setting storage and a command to toggle status LEDs on or off would be good for switching debugging off to prevent status LEDs from being unsightly.

## License

For license details, see LICENSE file