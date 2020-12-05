#ifndef RADIO_LOGGER_NODE_H
#define RADIO_LOGGER_NODE_H

#include "RadioLogger.h"

#ifndef NODE_ADDRESS
#error "Please define NODE_ADDRESS in main code"
#endif

class RadioLoggerNode : public RadioLogger {
public:
  RadioLoggerNode() : RadioLogger() {};
  
  bool log(String data) {
    //safePrint("Sending "); safePrintln(data);
    
    // Send a message to the DESTINATION!
    if (rf69_manager.sendtoWait((uint8_t *)data.c_str(), data.length()+1, STATION_ADDRESS)) {
      //safePrintln(" -> sent");
      return true;      
    } else {
      //safePrintln("Sending failed (no ack)");
      return false;
    }
  };
};

RH_RF69 RadioLogger::rf69 = RH_RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RadioLogger::rf69_manager = RHReliableDatagram(RadioLogger::rf69, NODE_ADDRESS);

#endif
