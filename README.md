sample Nuvoton NUC126 USB device code
=====================================

The [Nuvoton NUC126](http://www.nuvoton.com/hq/products/microcontrollers/arm-cortex-m0-mcus/nuc126-usb-series/) is an ARM Cortex-M0 based USB microcontroller capable of crystal-less USB operation.  It could be compared to the Atmel/Microchip SAMD21.

At present, source code is provided for these sample devices:

* USB CDC to UART bridge
* USB HID mouse emulator

To their credit, Nuvoton does provide sample code in their [NUC126_Series_BSP_CMSIS](https://www.nuvoton.com/hq/products/microcontrollers/arm-cortex-m0-mcus/nuc126-usb-series/Software/?__locale=en&resourcePage=Y) download.  However, that code is specifically for IAR and Keil toolchains.

This code, in contrast, is written for gcc and clang.

It uses a new USB device stack written specifically for the NUC126 and uses an API modeled on [vcp](https://github.com/ataradov/vcp).  vcp was written for the SAMD11 / SAMD21.  Since the USB stack APIs are nearly the same, code can be more easily ported between the ataradov vcp USB stack and this NUC126usb one (as well as [NUC121](https://github.com/majbthrd/NUC121usb/)).  Another advantage of this approach is that the code size is a little more efficient than the Nuvoton reference code.

## Build Requirements

One approach is to use [Rowley Crossworks for ARM](http://www.rowley.co.uk/arm/) to compile this code.  It is not free software, but has been my favorite go-to ARM development tool for a decade and counting.  Rowley does not officially support the Nuvoton NUC126, but you can [download an open-source CPU support package for the NUC126](https://github.com/majbthrd/MCUmisfits/).

*OR*

I've modified the build environment provided by [mcu-starter-projects](https://github.com/ataradov/mcu-starter-projects) to work with the NUC126.  With this approach, the code can be built using only open-source software.  In Ubuntu-derived distributions, this is likely achieved with as little as:

```
sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```
