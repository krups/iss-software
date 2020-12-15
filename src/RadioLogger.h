#ifndef RADIO_LOGGER_H
#define RADIO_LOGGER_H

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define CAPSULE_ADDRESS               2
#define STATION_ADDRESS               1
#define RF69_FREQ                     915.0
#define PACKET_BUFFER_NUM_PACKETS     5
#define SEND_BUF_SIZE                 RH_RF69_MAX_MESSAGE_LEN
#define RECV_BUF_SIZE                 2*RH_RF69_MAX_MESSAGE_LEN

extern void safePrintln(String s);
extern void safePrint(String s);

class RadioLogger {
public:
  RadioLogger() : sbuflen(0), rssi(0), lastFrom(0), pcount(0) {};
  
  bool begin() {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    
    // manual reset
    //digitalWrite(RFM69_RST, HIGH);
    //delay(10);
    //digitalWrite(RFM69_RST, LOW);
    //delay(10);
    
    //safePrintln("initialziing");
    
    if (!rf69_manager.init()) {
      //safePrintln("RFM69 radio init failed");
      return false;
    }
    
    safePrintln("Setting frequency");
    if (!rf69.setFrequency(RF69_FREQ)) {
      //safePrintln("setFrequency failed");
      return false;
    }
    
    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    // The encryption key has to be the same as the one in the server
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);
    
    //safePrintln("Setup done");
    return true;
  };

  bool log(String data, uint8_t addr) {
    safePrint("Sending "); safePrintln(data);
    
    // Send a message to the DESTINATION!
    if (rf69_manager.sendtoWait((uint8_t *)data.c_str(), data.length()+1, addr)) {
      //safePrintln(" -> ack");
      return true;      
    } else {
      //safePrintln(" -> no ack");
      return false;
    }
  }

  bool send(Packet *p, uint8_t addr) {
    // Send a message to the DESTINATION!
    
    // if the packet will fit in the send buffer
    if( p->size() >= RH_RF69_MAX_MESSAGE_LEN ){
      //safePrintln("sending super long messages not supported yet");
      return false;
    }
    
    sbuflen = 0;
    sendbuf[0] = p->type(); // so the receiver knows what to expect 
    
    memcpy( &sendbuf[1], p->data(), p->size() );
    sbuflen = p->size() + 1;
    
    //safePrint("about to send");
    //safePrint(String(sbuflen));
    //safePrintln(" bytes");
    
    if (rf69_manager.sendtoWait(sendbuf, sbuflen, addr)) {
      //safePrintln(" -> sent");
      return true;      
    } else {
      //safePrintln(" -> failed (no ack)");
      return false;
    }
    
    
  }
  
  bool receivePackets() {
    uint8_t to, id, flags, len;
    
    if( rf69_manager.recvfromAck(recvbuf, &len, &lastFrom, &to, &id, &flags) ){
      rssi = rf69.lastRssi();

      rbuflen = len;

      //delay(500);
      safePrint("got "); safePrint(String(len)); safePrint(" bytes of ");
      safePrint(" from "); safePrint(String(lastFrom)); safePrint(", to="); safePrint(String(to));
      safePrint(", id="); safePrint(String(id)); safePrint(", flags = "); safePrintln(String(flags));
  
      // turn them into packets
      //decodePackets(rbuflen);
      
      return true;
    } else {
      safePrintln("receive failed");
      rbuflen = 0;
      return false;
    }
  }
  
  void decodePackets() {    
    if( rbuflen == 0) return;
  
    // detect number of packets containted in buffer
    int num_packets = 0;
    
    int i = 0;
    
    //safePrint("decodePackets() len= ");
    //safePrintln(String(rbuflen));
    
    for( i=0; i<rbuflen; i++ ){
      switch(recvbuf[i]){
        case PTYPE_TELEM:
          num_packets++;
          i += TELEM_T_SIZE;
          break;
        case PTYPE_ACCELSTATS:
          num_packets++;
          i += ACC_STAT_T_SIZE;
          break;
        case PTYPE_TC:
          num_packets++;
          i += TC_T_SIZE;
          break;
        case PTYPE_ACCEL:
          num_packets++;
          i += ACC_T_SIZE;
          break;
        case PTYPE_COMMAND:
          num_packets++;
          i += CMD_T_SIZE;
          break;
        case PTYPE_IMU:
          num_packets++;
          i += IMU_T_SIZE;
          break;
        default:
          safePrintln("ERAWR: lost track of what packet was which n where n what not");
      }
      
      if( i > rbuflen ){
        safePrint("expecting another "); safePrintln(String(i-rbuflen));
      }
    }
    
    //safePrint("received "); safePrint(String(num_packets)); safePrint(" packets, ");
    //safePrint("   i="); safePrint(String(i)); safePrint(", len="); safePrintln(String(rbuflen));
    
    
    
    // NOW BUILD PACKETS
    // this assumes the start of the buffer is the start of a packet
    // TODO: keep track of packet completion and read start of buffer as potentially an ongoing 
    //       transmission of a single packet
    
    //pcount = 0;   // current count of instantiated packets
    uint8_t idx = 0;  // current position within buffer
    
    while( pcount < num_packets ){  
      
      // telemetry packet
      if( recvbuf[idx] == PTYPE_TELEM ){
        safePrintln("got TELEM packet");
        pbuf[pcount] = new TelemPacket(&recvbuf[idx+1]);
        pcount++;
        idx += TELEM_T_SIZE+1;
      } 
      
      // min/max stats over a period of time from the high g accelerometer
      else if( recvbuf[idx] == PTYPE_ACCELSTATS ){
        safePrintln("got ACEL_STATS packet");
        pbuf[pcount] = new AccStatsPacket(&recvbuf[idx+1]);
        pcount++;
        idx += ACC_STAT_T_SIZE+1;
      } 
      
      // timestamped thermocouple packet (containing TC_COUNT readings)
      else if( recvbuf[idx] == PTYPE_TC ){
        safePrintln("got TC packet");
        pbuf[pcount] = new TcPacket(&recvbuf[idx+1]);
        pcount++;
        idx += TC_T_SIZE+1;
        continue;
      } 
      
      // singe timestamped high g accelerometer reading
      else if( recvbuf[idx] == PTYPE_ACCEL ){
        safePrintln("got ACCEL packet");
        pbuf[pcount] = new AccPacket(&recvbuf[idx+1]);
        pcount++;
        idx += ACC_T_SIZE+1;
      } 
      
      // singe timestamped imu reading
      else if( recvbuf[idx] == PTYPE_IMU ){
        safePrintln("got IMU packet");
        pbuf[pcount] = new IMUPacket(&recvbuf[idx+1]);
        pcount++;
        idx += IMU_T_SIZE+1;
      } 
      
      // COMMAND PACKET WOOHOO
      else if( recvbuf[idx] == PTYPE_COMMAND ){
        safePrintln("got command packet oh yuh");
        pbuf[pcount] = new CommandPacket(&recvbuf[idx+1]);
        pcount++;
        idx += CMD_T_SIZE+1;
      }
      
      // unrecognized packet     
      else {
        safePrintln(" OUT OF WACHK ON THE PACKET DECODE, discarding entire buffer (for now)");
        break;
      }
    }
    
    if( pcount == num_packets ){
      //safePrintln(" --> parsed all packets");
      rbuflen = 0;
    }
    
  }

  void printPackets() {
    //safePrintln("*** printing packet...");
    if( pcount == 0) return;
    for( int i=0; i<pcount; i++ ){
      safePrintln(pbuf[i]->toString(true));
    }
    //safePrintln("done");
  }
  
  void deletePackets() {
  if( pcount == 0 ) return;
    for( int i=0; i<pcount; i++ ){
      delete pbuf[i];
    }
    pcount = 0;
  }

  int numPackets() { return pcount; } 
  
  Packet** packets() { return pbuf; }

  bool available() { return rf69_manager.available(); }

  void setRetries(uint8_t re) { rf69_manager.setRetries(re); }
  uint8_t getRetries() { return rf69_manager.retries(); }

  uint8_t sbuflen;                          // current number of bytes in send buffer
  uint8_t rbuflen;                          // current number of bytes in receive buffer
  uint8_t sendbuf[SEND_BUF_SIZE];           // send buffer
  uint8_t recvbuf[RECV_BUF_SIZE];           // receive buffer
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
