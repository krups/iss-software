#ifndef PINS_H
#define PINS_H

// TC pins for safety processor and teensy
#if defined(ADAFRUIT_TRINKET_M0) || defined(__MK64FX512__)
  // switch pin defs based depending on if this
  // code is running on the safety processor or the 
  // main Teensy 3.5
  #if defined(ADAFRUIT_TRINKET_M0)
    #define CS_TC1      1
    #define CS_TC2      13
    #define SEC_CTRL_2  0
    #define SEC_ACT     7
    #define SEC_CTRL_1  8
    #define MUX0        23
    #define MUX1        24
    #define TC1_FAULT   21
    #define TC2_FAULT   22
    #define SPI_US SEC_CTRL_2
    #define SPI_THEM SEC_CTRL_1
  #elif defined(__MK64FX512__) // teensy 3.5
    #define CS_TC1 20
    #define CS_TC2 21
    #define SEC_CTRL_2  27
    #define SEC_CTRL_1  39
    #define MUX0 16
    #define MUX1 17
    #define TC1_FAULT 25
    #define TC2_FAULT 26
    #define SPI_US SEC_CTRL_1
    #define SPI_THEM SEC_CTRL_2
  #else
  //#error "Unsupported TC pin config, plz fix TcInterface.h"
  #endif
#endif


// RADIO PIN DEFS for capsule and base station
// Feather M0 w/Radio
// aka basetation receiver
#if defined(ADAFRUIT_FEATHER_M0)
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
// teensy 3.5
#elif defined(__MK64FX512__)
// flight comp v1.1
  #define RFM69_CS      9
  #define RFM69_INT     29
  #define RFM69_RST     28
#else
#error "Unsupported target device for radio pin config, plz fix RadioLogger.h"
#endif


#define CAPSENSE0       1		            // input, read with touchRead(pin)
#define CAPSENSE1       0	              // input, read with touchRead(pin)
#define CAPSENSE2       15	            // input, read with touchRead(pin)
#define CAPSENSE3	      30              // input, read with touchRead(pin)

#define LED_IR_ON       3               // iridium modem powered on
#define LED_IR_SIG      4               // iridium modem signal quality
#define LED_IR_TX       5               // iridium modem transmitting
#define LED_ISM_TX      6               // ism radio on/transmitting
#define LED_ACT         7               // general activity

#define PIN_IR_ENABLE   23               // iridium activation signal

#define CS_DSP          8		            // active low chip select for DSP310 pressure sensor

#define BAT_STAT	      14              // high when charging
#define BAT_SENSE	      A8              // battery voltage / 2 (halved by voltage divider)

#define BUZZER          24              // outout to piezo through 100ohm r

#define RADIO_OFF_SIG   33              // pulled high when 3.3v to ism radio is present

#define IMU_FSYNC       34	            // sync signal (not necessary for functionality)
#define IMU_INT         35	            // interrupt pin (not necessary for functionality)
#define ACC_X           A17             // x axis high g accel output
#define ACC_Y           A18             // y axis high g accel output
#define ACC_Z           A19             // x axis high g accel output

#define IRIDIUM_TX_ACT	A21	            // iridium output indicating the module is actively transmitting RF
#define IRIDIUM_STATUS	A22	            //iridium output indicating an SBD message has arrived


#endif
