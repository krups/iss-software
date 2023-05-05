#ifndef PINS_H
#define PINS_H

// pins 0 (RX) and 1 (TX) connect to the 4 pin header on the spectrometer wing 1.0
// and go to the main flight computer PCB

// spectrometer connections for the spectrometer wing PCB v1.0
// pinout for feather m4 (samd51)
#define SPEC_CLK    22
#define SPEC_ST     23
#define SPEC_TRIG   24
#define SPEC_EOS    A5 // same as A5
#define SPEC_VIDEO  A4 // same as A4

// spectrometer connections for the spectrometer wing PCB v1.0
// pinout for feather m0 (samd21)
// #define SPEC_CLK     4
// #define SPEC_ST     A1
// #define SPEC_VIDEO  A0 // same as A4

#endif