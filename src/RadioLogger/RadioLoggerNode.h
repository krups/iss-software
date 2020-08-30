#ifndef RADIO_LOGGER_NODE_H
#define RADIO_LOGGER_NODE_H

#include "RadioLogger.h"

#ifndef NODE_ADDRESS
#error "Please define NODE_ADDRESS in main code"
#endif

class RadioLoggerNode : public RadioLogger {
public:
  RadioLoggerNode() : RadioLogger() {};
  
  void log(String data) {
    Serial.print("Sending "); Serial.print(data);
    
    // Send a message to the DESTINATION!
    if (rf69_manager.sendtoWait((uint8_t *)data.c_str(), data.length()+1, STATION_ADDRESS)) {
      Serial.println(" -> sent");
      // Not expecting a reply from the server
      
    } else {
      Serial.println("Sending failed (no ack)");
    }
  };
};

RH_RF69 RadioLogger::rf69 = RH_RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RadioLogger::rf69_manager = RHReliableDatagram(RadioLogger::rf69, NODE_ADDRESS);

#endif
