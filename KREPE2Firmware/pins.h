// KREPE-2 flight computer pin mapping
// matt ruffner Nov 2022
// on feather m4 epress

#ifndef PINS_H
#define PINS_H

#define PIN_NEOPIXEL              8  // neopixwl on the feather m4 board

#define PIN_EXT_INT                14 // continuity interrupt for KREM release

// RFM69 connections (plus SPI)
#define PIN_RADIO_SS               10 // RFM69 chip select line
#define PIN_RADIO_RESET            9  // RFM69 reset line
#define PIN_RADIO_INT              5  // radio interrupt pin connected to DIO 0 on RFM69 

#define PIN_3V32_CONTROL           6  // enable pin to 3v3 regulator (which powers the RS232 IC and RFM69 radio)
#define PIN_GATE_IR                11 // positive logic enable for external 5v rail (powering the Iridium and GPS ports, and spec and bsms)

// J1 POL Serial1, designated for NanoPi 
#define PIN_RX1                    0
#define PIN_TX1                    1

// RIRI and TIRI, the serial pins to the iridium port
#define PIN_RX2                   13
#define PIN_TX2                   12

// RGPS and TGPS, the serial pins to the GPS port
// SERCOM4 
#define PIN_RX3                   17
#define PIN_TX3                   16

// RX4 and TX4, the serial pins on the BSMS port
#define PIN_RX4                   15
#define PIN_TX4                   18


// SD card connections
#define PIN_SD_CS                 19
#define PIN_SD_SCK                25
#define PIN_SD_MOSI               24
#define PIN_SD_MISO               23

// VBAT voltage divided by two is present on this pin
#define PIN_VBAT                  A6

// I2C mux connections for ported pressure sensors
#define MUXCHAN_PS1 0
#define MUXCHAN_PS2 1
#define MUXCHAN_PS3 2
#define MUXCHAN_PS4 3
#define MUXCHAN_PS5 4


#endif
