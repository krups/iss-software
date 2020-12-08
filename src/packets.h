/*
* KREPE ISS Mission
* Dec 2020
*
* SDLogger supports writing to multiple files, referenced by a file ID.
* 
*/

#ifndef PACKETS_H
#define PACKETS_H

#include "config.h"

extern void safePrintln(String s);
extern void safePrint(String s);

/****************************
Packet types
*/
#define PTYPE_TELEM        'M'
#define PTYPE_ACCELSTATS   'A'
#define PTYPE_TC           'T'
#define PTYPE_ACCELSINGLE  'B'
#define PTYPE_IMUSTATS     'I'
// board telemetry packet

typedef struct {
  unsigned long t;          // 4B, timestamp
  float batt;               // 4B, battery voltage
  float tc1_temp;           // 4B, TC converter 1 cold junction temperature
  float tc2_temp;           // 4B, TC converter 2 cold junction temperature
} telem_t;
#define TELEM_T_SIZE    16  // just enough for data, not including struct padding

// high g accel stats packet
typedef struct {
  unsigned long tmin;  // 4B
  unsigned long tmax;  // 4B
  uint16_t x_min;      // 2B
  uint16_t x_max;      // 2B   
  uint16_t y_min;      // 2B
  uint16_t y_max;      // 2B
  uint16_t z_min;      // 2B
  uint16_t z_max;      // 2B
} acc_stat_t;
#define ACC_STAT_T_SIZE    20 // just enough for data, not including struct padding

// "compact" thermocouple data packet
// keep track of time in seconds for packet ID
typedef struct {
  float data[TC_COUNT];  // tc measurements, 4 * TC_COUNT bytes
  unsigned long time;       // in seconds,        4 bytes
} tc_t;
#define TC_T_SIZE    (4*TC_COUNT) + 4 // just enough for data, not including struct padding

// high g accel data, one x/y/z sample
typedef struct {
  unsigned long t; // 4B
  uint16_t x;      // 2B
  uint16_t y;      // 2B 
  uint16_t z;      // 2B
} acc_t;           // --- 
#define ACC_T_SIZE    10 // just enough for data, not including struct padding

// "verbose" thermocouple data packet
// not for compressing or sending, 
// includes extra information about fault condition
// on max31856 when measurement was taken
// no timestamp because this is only used in reading
// before logging
typedef struct {
  float data[TC_COUNT];
  byte fault[TC_COUNT];
} tcv_t;

/**********************
* base packet class
*/
class Packet {
public:
  Packet() {}
  Packet(char type, int size) : _type(type), _size(size) {
    _data = new uint8_t[size];
  }
  ~Packet() { delete _data; safePrintln("deleted packet");}
  
  virtual String toString () {};
  virtual String toString (bool timestamp) {};
  char type() const { return _type; }
  int size() const { return _size; }
  uint8_t* data() const { return _data; }

protected: 
  uint8_t *_data; 
  char _type;
  int _size;
};

/*************************
* telemetry packet class
*/
class TelemPacket : public Packet {
public:
  TelemPacket(uint8_t *buf) : Packet( PTYPE_TELEM, TELEM_T_SIZE ) {
    memcpy(_data, buf, _size);
  }
  TelemPacket(unsigned long t, float batt, float tc1_temp, float tc2_temp) : Packet( PTYPE_TELEM, TELEM_T_SIZE ) {
    *(unsigned long*)(&_data[0]) = t;
    *(float*)(&_data[4])         = batt;
    *(float*)(&_data[8])         = tc1_temp;
    *(float*)(&_data[12])        = tc2_temp;
  }
  
  String toString() { return toString(false); }
  
  String toString(bool with_timestamp) {
    String a;
    if( with_timestamp ){
      a = String(t()) + "\t";
    }
    a += String(batt()) + "\t" + String(tc1_temp()) + "\t" + String(tc2_temp());
    
    return a;
  }
  
  unsigned long t(){return *((unsigned long*)&_data[0]);}
  float batt(){return *((float*)&_data[4]);}
  float tc1_temp(){return *((float*)&_data[8]);}
  float tc2_temp(){return *((float*)&_data[12]);}
};

/**************************
* acc sample packet class
*/
class AccPacket : public Packet {
public:
  AccPacket(uint8_t *buf) : Packet( PTYPE_ACCELSINGLE, ACC_T_SIZE ) {
    memcpy(_data, buf, _size);
  }
  
  AccPacket(uint16_t x, uint16_t y, uint16_t z, unsigned long t) : Packet( PTYPE_ACCELSINGLE, ACC_T_SIZE ) {
    *(unsigned long*)(&_data[0]) = t;
    *(uint16_t*)(&_data[4])      = x;
    *(uint16_t*)(&_data[6])      = y;
    *(uint16_t*)(&_data[8])      = z;
  }

  String toString() { return toString(false); }

  String toString(bool with_timestamp) {
    String a;
    if( with_timestamp ){
      a = String(t()) + "\t";
    }
    a += String(x()) + "\t" + String(y()) + "\t" + String(z());
    return a;
  }

  unsigned long t(){return *((unsigned long*)&_data[0]);}
  uint16_t x(){return *((uint16_t*)&_data[4]);}
  uint16_t y(){return *((uint16_t*)&_data[6]);}
  uint16_t z(){return *((uint16_t*)&_data[8]);}
};

// acc stats sample class
class AccStatsPacket : public Packet {
public:
  AccStatsPacket( uint8_t *buf ) : Packet(PTYPE_ACCELSTATS, ACC_STAT_T_SIZE){
    memcpy(_data, buf, _size);
  }
  
  AccStatsPacket(
      unsigned long tmin, unsigned long tmax,
      uint16_t x_min, uint16_t x_max,
      uint16_t y_min, uint16_t y_max,
      uint16_t z_min, uint16_t z_max) : 
      Packet(PTYPE_ACCELSTATS, ACC_STAT_T_SIZE){
    *(unsigned long*)(&_data[0]) = tmin;
    *(unsigned long*)(&_data[4]) = tmax;
    *(uint16_t*)(&_data[8]) = x_min;
    *(uint16_t*)(&_data[10]) = x_max;
    *(uint16_t*)(&_data[12]) = y_min;
    *(uint16_t*)(&_data[14]) = y_max;
    *(uint16_t*)(&_data[16]) = z_min;
    *(uint16_t*)(&_data[18]) = z_max;
  }
  
  String toString() { return toString(false); }
  
  String toString(bool with_timestamp) {
    String a;
    if( with_timestamp ){
      a = String(tmin()) + "\t" + String(tmax());
    }
    a += String(x_min()) + "\t" + String(x_max()) + "\t" + String(y_min()) + "\t" + String(y_max()) + "\t" + String(z_min()) +" \t" + String(z_max());
    
    return a;
  }
  
  unsigned long tmin(){return *((unsigned long*)&_data[0]);}
  unsigned long tmax(){return *((unsigned long*)&_data[4]);}
  uint16_t x_min(){return *((uint16_t*)&_data[8]);}
  uint16_t x_max(){return *((uint16_t*)&_data[10]);}
  uint16_t y_min(){return *((uint16_t*)&_data[12]);}
  uint16_t y_max(){return *((uint16_t*)&_data[14]);}
  uint16_t z_min(){return *((uint16_t*)&_data[16]);}
  uint16_t z_max(){return *((uint16_t*)&_data[18]);}
};

// Thermocouple packet class
class TcPacket : public Packet {
public:
  TcPacket(uint8_t *buf) : Packet(PTYPE_TC, TC_T_SIZE){
    memcpy(_data, buf, _size);
  }
  
  TcPacket(float* data, unsigned long time) : Packet(PTYPE_TC, TC_T_SIZE){
    for(int i = 0; i < TC_COUNT; i++){
      ((float*)_data)[i] = data[i];
      safePrintln(String(((float*)_data)[i]));
    }
    *(unsigned long*)(&_data[4*TC_COUNT]) = time;
  }

  String toString() { return toString(false); }
  
  String toString(bool with_timestamp) {
    String a;
    if( with_timestamp ){
      a = String(time()) + "\t";
    }
    for( int i=0; i < TC_COUNT; i++ ){
      a += String(data()[i]) + ((i==TC_COUNT-1) ? "" : "\t");
    }
    
    return a;
  }


  float* data(){ return (float *)_data; }
  unsigned long time(){return *((unsigned long*)&_data[4*TC_COUNT]);}
};

// iridium packet class
class IridiumPacket {
public:
  IridiumPacket() {;}
  ~IridiumPacket() {;}
};

#endif
