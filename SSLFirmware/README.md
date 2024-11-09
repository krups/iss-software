# This is a Dev location for re created the KFC fireware but making it modular for all different needs 

## Architecture
This firmware is designed to be modular with macros configuring complitatin based on the hardware and mission specificed. Due to this **be very careful and mindful when modifying them**. This document provides explanation for each file. The 'main.c' file calls different modules and functions as needed. Each addional hardware or behavior is held in a different module. 

## main.c 
This file serves as the entry point for the firmware. This is where each of the other files are called. This file is configured by the 'config.h' file.

## config.h
This file specifies using macros what hardware, pinout, and mission is being compiled to. Modify the define on line 34 in order to change the mission and hardware. Add extra else if statements to add other hardware.

Each hardware is defined in the 'config.h' file as follows:
- if(current option)
  - define if i2c is enabled
  - define if spi is enabled
  - define if sensors are enabled
  - define which communication is enabled
  - define usb pins
  - define I2C pins
  - define I2C devices
  - define SPI pins
  - define SPU devices
  - define Special Pins for different devices