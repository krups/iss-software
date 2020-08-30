#ifndef RADIO_LOGGER_H
#define RADIO_LOGGER_H

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define STATION_ADDRESS 1

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

#define RF69_FREQ 915.0


class RadioLogger {
public:
  RadioLogger(){};
  
  bool begin() {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    
    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    
    Serial.println("initialziing");
    
    if (!rf69_manager.init()) {
      Serial.println("RFM69 radio init failed");
      return false;
    }
    
    Serial.println("Setting frequency");
    if (!rf69.setFrequency(RF69_FREQ)) {
      Serial.println("setFrequency failed");
      return false;
    }
    
    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    // The encryption key has to be the same as the one in the server
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);
    
    Serial.println("Setup done");
    return true;
  };

  static RH_RF69 rf69;
  static RHReliableDatagram rf69_manager;

};



#endif
