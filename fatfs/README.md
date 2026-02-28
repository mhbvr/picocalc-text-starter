FAT filesystem for PicoCalc SD card
===================================

It is using [FatFS](https://elm-chan.org/fsw/ff/) for the filesystem implemetation and SD card SPI interface for low level block IO.

SD SPI intreface docs
=====================

[Raspberry Pi Pico SDK SPI functions](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_spi)

[C++ FAT lib for Pico based on Mbed os](https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico)

[SDcard simplified specifications](https://www.sdcard.org/downloads/pls/)
One need "Physical Layer Simplified Specification" for details about SPI commands

[Short SPi modes description](https://elm-chan.org/docs/spi_e.html)

[Short but detailed description on the SD card SPI interface](https://elm-chan.org/docs/mmc/mmc_e.html)