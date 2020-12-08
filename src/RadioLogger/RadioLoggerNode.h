#ifndef RADIO_LOGGER_NODE_H
#define RADIO_LOGGER_NODE_H

#include "RadioLogger.h"

#ifndef NODE_ADDRESS
#error "Please define NODE_ADDRESS in main code"
#endif

#define SEND_BUF_SIZE RH_RF69_MAX_MESSAGE_LEN

class RadioLoggerNode : public RadioLogger {
public:
  RadioLoggerNode() : RadioLogger(), buflen(0) {};
  
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
  }
  
  bool log(Packet *p) {
    // Send a message to the DESTINATION!
    // if the packet will fit in the send buffer
    buflen = 0;
    buf[0] = p->type(); // so the receiver knows what to expect 
    memcpy( &buf[1], p->data(), p->size() );
    buflen = p->size() + 1;
    
    if (rf69_manager.sendtoWait(buf, buflen, STATION_ADDRESS)) {
      //safePrintln(" -> sent");
      return true;      
    } else {
      //safePrintln("Sending failed (no ack)");
      return false;
    }
    
    
  }

private:
  uint8_t buflen;
  uint8_t buf[SEND_BUF_SIZE];
};

RH_RF69 RadioLogger::rf69 = RH_RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RadioLogger::rf69_manager = RHReliableDatagram(RadioLogger::rf69, NODE_ADDRESS);

#endif
