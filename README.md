# stm32f3-hal-sdadc
Host PC to STM32F3 device UART interface with CLI

## Summary
- Send bytes individually using a terminal emulator like [Putty](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html), [Tera Term](https://teratermproject.github.io/index-en.html), [CoolTerm](https://freeware.the-meiers.org/), etc.
- Send bytes as a packet ('\r' terminated) using a RS-232 terminal like [Termite](https://www.compuphase.com/software_termite.htm) or [Yet Another Terminal](https://sourceforge.net/projects/y-a-terminal/files/latest/download), or using Python via the [pyserial](https://pyserial.readthedocs.io/) library

## Requirements
- [MSYS2 (UCRT64 environment)](https://www.msys2.org/)
- [ARM GNU toolchain](https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-arm-none-eabi-gcc)
- [Make](https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-make)
- [CMake](https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-cmake)

### Optional
- [Git for Windows within the MSYS2 shell](https://stackoverflow.com/questions/78011652/installing-git-within-msys2-on-windows-11)

## Build

Cross-compile the project using Windows 11 and MSYS2 (UCRT64 environment)

``` Bash
mkdir Build
cd Build
cmake -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE:PATH=".\cmake\gcc-arm-none-eabi.cmake"  ..
make -j4
```

To rebuild, first clean using ```make clean```, then execute ```make -j4```

On CMakeLists.txt update, remain in the Build directory and execute:

``` Bash
rm -rf *
cmake -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE:PATH=".\cmake\gcc-arm-none-eabi.cmake"  ..
make -j4
```

## Program MCU

Download and install the [STM32 ST-LINK utility](https://www.st.com/en/development-tools/stsw-link004.html). The CLI application "ST-LINK_CLI.exe" is included in the installation.

``` bash
"ST-LINK_CLI.exe" -c SN=55FF6C064882485358622187 SWD FREQ=4000 LPM -P "C:\path\to\file.hex" 0x08000000 -V "after_programming" -Run
```

- [DB3068 STM32 ST-LINK Utility for STM32 MCUs](https://www.st.com/en/development-tools/stsw-link004.html#documentation) Documentation explaining how to use ST-LINK_CLI.exe

### Potential Alternatives

- [openocd](https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-openocd)
- [stlink](https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-stlink?repo=ucrt64)

## External
- [embedded-cli](https://github.com/FARLY7/embedded-cli)
- [printf](https://github.com/eyalroz/printf)
- [Ring-Buffer](https://github.com/AndersKaloer/Ring-Buffer)
 -[lwrb](https://github.com/MaJerle/lwrb/tree/develop)

## References
- [UART_ReceptionToIdle_CircularDMA](https://github.com/STMicroelectronics/STM32CubeF4/tree/master/Projects/STM32446E-Nucleo/Examples/UART/UART_ReceptionToIdle_CircularDMA)
- [stm32-usart-uart-dma-rx-tx](https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx)
- [LwRB latest-develop documentation](https://docs.majerle.eu/projects/lwrb/en/latest/)
- [Artful Bytes - Printf on a Microcontroller](https://www.youtube.com/watch?v=ZdDxx1GIFj0&t=1590s)
