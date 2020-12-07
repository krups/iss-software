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
#include <IridiumSBD.h>

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
  uint16_t time;       // in seconds,        2 bytes
} tc_t;
#define TC_T_SIZE    4*TC_COUNT + 2 // just enough for data, not including struct padding



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
    _data = new uint8_t[size+1];
    _data[0] = type;
  }
  ~Packet() { delete _data; safePrintln("deleted packet");}
  
  char type() const { return _type; }
  int size() const { return _size; }
  uint8_t* data() const { return _data; }

protected: 
  uint8_t *_data; 

private:
  char _type;
  int _size;
};


/**************************
* acc sample packet class
*/
class AccPacket : public Packet {
public:
  AccPacket(uint16_t x, uint16_t y, uint16_t z, unsigned long t) : Packet( PTYPE_ACCELSINGLE, ACC_T_SIZE ) {
    *(unsigned long*)(&_data[1]) = t;
    *(uint16_t*)(&_data[5])      = x;
    *(uint16_t*)(&_data[7])      = y;
    *(uint16_t*)(&_data[9])      = z;
  }
};

// acc stats sample class
class AccStatsPacket : public Packet {
public:
  AccStatsPacket(
      unsigned long tmin, unsigned long tmax,
      uint16_t x_min, uint16_t x_max,
      uint16_t y_min, uint16_t y_max,
      uint16_t z_min, uint16_t z_max) : 
      Packet(PTYPE_ACCELSTATS, ACC_STAT_T_SIZE){
      *(unsigned long*)(&_data[1]) = tmin;
      *(unsigned long*)(&_data[5]) = tmax;
      *(uint16_t*)(&_data[9]) = x_min;
      *(uint16_t*)(&_data[11]) = x_max;
      *(uint16_t*)(&_data[13]) = y_min;
      *(uint16_t*)(&_data[15]) = y_max;
      *(uint16_t*)(&_data[17]) = z_min;
      *(uint16_t*)(&_data[19]) = z_max;
  }
};

// iridium packet class
class IridiumPacket {
public:
  IridiumPacket() {;}
  ~IridiumPacket() {;}
};

#endif
