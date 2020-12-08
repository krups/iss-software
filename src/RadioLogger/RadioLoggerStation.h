#ifndef RADIO_LOGGER_STATION_H
#define RADIO_LOGGER_STATION_H

#include "RadioLogger.h"
//#include "packets.h"

#define PACKET_BUFFER_NUM_PACKETS 50

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
        //Serial.print("got "); Serial.print(len); Serial.println(" bytes");
        //Serial.print("last byte is: "); Serial.println(buf[len-1], HEX);
        buf[len] = 0; // zero out remaining string
        lastPacketFrom = from;
        lastRSSI = rf69.lastRssi();
        //Serial.write((char*)buf);
    
        decodePackets(len);
      }     
      
      return true;
    } else {
      return false;
    }
  };
private:

  bool decodePackets(uint8_t len) {    
    // detect number of packets containted in buffer
    uint8_t num_packets = 0;
    
    for( int i=0; i<len; i++ ){
      switch(buf[i]){
        case PTYPE_TELEM:
          num_packets++;
          i += TELEM_T_SIZE + 1;
          break;
        case PTYPE_ACCELSTATS:
          num_packets++;
          i += ACC_STAT_T_SIZE + 1;
          break;
        case PTYPE_TC:
          num_packets++;
          i += TC_T_SIZE + 1;
          break;
        case PTYPE_ACCELSINGLE:
          num_packets++;
          i += ACC_T_SIZE + 1;
          break;
        case PTYPE_IMUSTATS:
          Serial.print("got IMU_STATS packet, dont know what to do ,,,");
          break;
        default:
          Serial.println("ERAWR: lost track of what packet was which n where n what not");
      }
    }
    
    Serial.print("received "); Serial.print(num_packets); Serial.println(" packets");
    
    
    // now build packets
    int pcount = 0;   // current count of instantiated packets
    uint8_t idx = 0;  // current position within buffer
    
    while( idx < len-1 ){      
      switch(buf[idx++]){
      
        case PTYPE_TELEM:
          Serial.print("got TELEM packet");
          pbuf[pcount] = new AccPacket(&buf[idx]);
          pcount++;
          idx += TELEM_T_SIZE;
          break;
          
        case PTYPE_ACCELSTATS:
          Serial.print("got ACEL_STATS packet");
          pbuf[pcount] = new AccStatsPacket(&buf[idx]);
          pcount++;
          idx += ACC_STAT_T_SIZE;
          break;
          
        case PTYPE_TC:
          Serial.print("got TC packet");
          pbuf[pcount] = new TcPacket(&buf[idx]);
          pcount++;
          idx += TC_T_SIZE;
          break;
          
        case PTYPE_ACCELSINGLE:
          Serial.println("got ACCEL packet");
          Serial.print("  ");
          Serial.println(*((uint32_t*)&buf[idx+1])); 
          Serial.println(*((uint32_t*)&buf[idx+5]));
          Serial.println(*((uint32_t*)&buf[idx+7]));
          Serial.println(*((uint32_t*)&buf[idx+9]));
          pbuf[pcount] = new AccPacket(&buf[idx]);
          pcount++;
          idx += ACC_T_SIZE;
          break;
          
        case PTYPE_IMUSTATS:
          Serial.print("got IMU_STATS packet NoW WhAHt");
          break;
          
        default:
          Serial.println(" OUT OF WACHK ON THE PACKET DECODE");
      }
    
    }
    
  }

  void printTCPacket(tc_t data) {
  
  }
  
  Packet *pbuf[PACKET_BUFFER_NUM_PACKETS];
  
  int lastRSSI;
  int lastPacketFrom;
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
};

RH_RF69 RadioLogger::rf69 = RH_RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RadioLogger::rf69_manager = RHReliableDatagram(RadioLogger::rf69, STATION_ADDRESS);

#endif
