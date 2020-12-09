#ifndef RADIO_LOGGER_H
#define RADIO_LOGGER_H

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define CAPSULE_ADDRESS               2
#define STATION_ADDRESS               1
#define RF69_FREQ                     915.0
#define PACKET_BUFFER_NUM_PACKETS     50
#define SEND_BUF_SIZE                 2*RH_RF69_MAX_MESSAGE_LEN

extern void safePrintln(String s);
extern void safePrint(String s);

class RadioLogger {
public:
  RadioLogger() : buflen(0), rssi(0), lastFrom(0), pcount(0) {};
  
  bool begin() {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    
    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    
    safePrintln("initialziing");
    
    if (!rf69_manager.init()) {
      safePrintln("RFM69 radio init failed");
      return false;
    }
    
    safePrintln("Setting frequency");
    if (!rf69.setFrequency(RF69_FREQ)) {
      safePrintln("setFrequency failed");
      return false;
    }
    
    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    // The encryption key has to be the same as the one in the server
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);
    
    safePrintln("Setup done");
    return true;
  };

  bool log(String data, uint8_t addr) {
    safePrint("Sending "); safePrintln(data);
    
    // Send a message to the DESTINATION!
    if (rf69_manager.sendtoWait((uint8_t *)data.c_str(), data.length()+1, addr)) {
      safePrintln(" -> ack");
      return true;      
    } else {
      safePrintln(" -> no ack");
      return false;
    }
  }

  bool send(Packet *p, uint8_t addr) {
    // Send a message to the DESTINATION!
    
    // if the packet will fit in the send buffer
    if( p->size() >= RH_RF69_MAX_MESSAGE_LEN ){
      safePrintln("sending super long messages not supported yet");
      return false;
    }
    
    buflen = 0;
    buf[0] = p->type(); // so the receiver knows what to expect 
    
    memcpy( &buf[1], p->data(), p->size() );
    buflen = p->size() + 1;
    
    safePrintln("about to send");
    
    if (rf69_manager.sendtoWait(buf, buflen, addr)) {
      safePrintln(" -> sent");
      return true;      
    } else {
      safePrintln(" -> failed (no ack)");
      return false;
    }
    
    
  }
  
  bool receivePackets() {
    if( available() ){
      uint8_t len = 0;
      uint8_t from = 0;
      
      if (rf69_manager.recvfromAck(buf, &len, &from)) {
        safePrint("got "); safePrint(String(len)); safePrintln(" bytes");

        lastFrom = from;
        rssi = rf69.lastRssi();
    
        // store the number of bytes we just received
        buflen = len;
    
        // turn them into packets
        decodePackets(len);
        
        return true;
      }
    }
  }
  
  void decodePackets(uint8_t len) {    
    // detect number of packets containted in buffer
    int num_packets = 0;
    
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
          safePrintln("got IMU_STATS packet, dont know what to do ,,,");
          break;
        default:
          safePrintln("ERAWR: lost track of what packet was which n where n what not");
      }
    }
    
    safePrint("received "); safePrint(String(num_packets)); safePrintln(" packets");
    
    
    // now build packets
    pcount = 0;   // current count of instantiated packets
    uint8_t idx = 0;  // current position within buffer
    
    while( pcount < num_packets ){  
      
      //safePrint("idx=");
      //safePrintln(idx);
      //safePrint("pcount=");
      //safePrintln(pcount);
      
      // telemetry packet
      if( buf[idx] == PTYPE_TELEM ){
        safePrintln("got TELEM packet");
        pbuf[pcount] = new AccPacket(&buf[idx+1]);
        pcount++;
        idx += TELEM_T_SIZE+1;
      } 
      
      // min/max stats over a period of time from the high g accelerometer
      else if( buf[idx] == PTYPE_ACCELSTATS ){
        safePrintln("got ACEL_STATS packet");
        pbuf[pcount] = new AccStatsPacket(&buf[idx+1]);
        pcount++;
        idx += ACC_STAT_T_SIZE+1;
      } 
      
      // timestamped thermocouple packet (containing TC_COUNT readings)
      else if( buf[idx] == PTYPE_TC ){
        safePrintln("got TC packet");
        pbuf[pcount] = new TcPacket(&buf[idx+1]);
        pcount++;
        idx += TC_T_SIZE+1;
        continue;
      } 
      
      // singe timestamped high g accelerometer reading
      else if( buf[idx] == PTYPE_ACCELSINGLE ){
        safePrintln("got ACCEL packet");
        pbuf[pcount] = new AccPacket(&buf[idx+1]);
        pcount++;
        idx += ACC_T_SIZE+1;
      } 
      
      // unrecognized packet     
      else {
        safePrintln(" OUT OF WACHK ON THE PACKET DECODE, discarding entire buffer (for now)");
        break;
      }
    }
    
    if( pcount == num_packets ){
      safePrintln(" --> parsed all packets");
    }
    
  }

  void printPackets() {
    safePrintln("*** printing packet...");
    for( int i=0; i<pcount; i++ ){
      safePrintln(pbuf[i]->toString());
    }
    safePrintln("done");
  }
  
  void deletePackets() {
    for( int i=0; i<pcount; i++ ){
      delete pbuf[i];
    }
    pcount = 0;
  }

  bool available() { return rf69_manager.available(); }

  void setRetries(uint8_t re) { rf69_manager.setRetries(re); }
  uint8_t getRetries() { return rf69_manager.retries(); }


protected:
  uint8_t buflen;                           // current number of bytes in buffer
  uint8_t buf[SEND_BUF_SIZE];               // buffer
  uint8_t rssi;                             // signal strength of last received message
  uint8_t lastFrom;                         // address last message was from
  
  int pcount;                               // how many packets we are holding
  Packet *pbuf[PACKET_BUFFER_NUM_PACKETS];  // the packets we are holding
  static RH_RF69 rf69;
  static RHReliableDatagram rf69_manager;
};


RH_RF69 RadioLogger::rf69 = RH_RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RadioLogger::rf69_manager = RHReliableDatagram(RadioLogger::rf69, NODE_ADDRESS);

#endif
