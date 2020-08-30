#ifndef RADIO_LOGGER_STATION_H
#define RADIO_LOGGER_STATION_H

#include "RadioLogger.h"

class RadioLoggerStation : public RadioLogger {
public:
  RadioLoggerStation() : RadioLogger() {};

  bool waitForAndPrintMessage() {
    if (rf69_manager.available())
    {
      // Wait for a message addressed to us from the client
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (rf69_manager.recvfromAck(buf, &len, &from)) {
        Serial.print("got "); Serial.print(len); Serial.println(" bytes");
        Serial.print("last byte is: "); Serial.println(buf[len-1], HEX);
        buf[len] = 0; // zero out remaining string
        lastPacketFrom = from;
        lastRSSI = rf69.lastRssi();
        Serial.write((char*)buf);
      }
      return true;
    } else {
      return false;
    }
  };
private:
  int lastRSSI;
  int lastPacketFrom;
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
};

RH_RF69 RadioLogger::rf69 = RH_RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RadioLogger::rf69_manager = RHReliableDatagram(RadioLogger::rf69, STATION_ADDRESS);

#endif
