/*
* KREPE ISS Mission
* Dec 2020
*
* Packet definitions
* 
*/

#ifndef PACKETS_H
#define PACKETS_H

#include "config.h"
#include "commands.h"

/****************************
Packet types
*/
#define PTYPE_ACCELSTATS   'A'
#define PTYPE_COMMAND      'C'
#define PTYPE_ACCEL        'B'
#define PTYPE_TELEM        'M'
#define PTYPE_IMU          'I'
#define PTYPE_TC           'T'


/************************************************************************************
* Packet struct definitions for passing data between threads. Not used for sending or
* logging since structs are word aligned and waste space
*/

// command packet
typedef struct {
  uint8_t cmdid;
  uint8_t args;
  uint8_t data[4];
} cmd_t;
#define CMD_T_SIZE      2 + NUM_CMD_PAYLOAD_BYTES

// board telemetry packet
typedef struct {
  uint32_t t;          // 4B, timestamp
  float batt;               // 4B, battery voltage
  float tc1_temp;           // 4B, TC converter 1 cold junction temperature
  float tc2_temp;           // 4B, TC converter 2 cold junction temperature
  uint8_t ir_sig;           // 1B, iridium signal strength (0-5)
  uint8_t log_num;          // 1B, current logfile id
} telem_t;
#define TELEM_T_SIZE    18  // just enough for data, not including struct padding

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

// thermocouple measurement packet
typedef struct {
  float data[TC_COUNT];  // tc measurements, 4 * TC_COUNT bytes
  uint32_t time;       // in ms
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

// imu measurement
typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;
  unsigned long t;
} imu_t;
#define IMU_T_SIZE   40 // 9 * 4B + 4B

// thermocouple data packet with fault conditions
typedef struct {
  float data[TC_COUNT];
  byte fault[TC_COUNT];
} tcv_t;

/* end packet structure definitions */
/***********************************************************************************/


/**********************
* base packet class
*/
class Packet {
public:
  Packet() {}
  Packet(char type, int size) : _type(type), _size(size) {
    _data = new uint8_t[size];
  }
  ~Packet() { delete _data; }
  
  char type() const { return _type; }
  int size() const { return _size; }
  uint8_t* data() const { return _data; }

protected: 
  uint8_t *_data; 
  char _type;
  int _size;
};





/******************************************************************************************
* Class definitions for packet types define above. Provides member access and stringification 
*/
#define NUM_CMD_PAYLOAD_BYTES 4
// command packet
class CommandPacket : public Packet{
public:
  CommandPacket(uint8_t *buf) : Packet(PTYPE_COMMAND, CMD_T_SIZE) {
    memcpy(_data, buf, _size);
  }

  CommandPacket(char cmd, uint8_t args, uint8_t *payload) : Packet(PTYPE_COMMAND, CMD_T_SIZE ) {
    _data[0] = cmd;
    _data[1] = args;
    if( args == 0 ){
      memset(&_data[2], 0, NUM_CMD_PAYLOAD_BYTES);
    } else {
      // tries to copy a full payload from payload buffer, make it is atleast that big (NUM_CMD_PAYLOAD_BYTES)
      memcpy(&_data[2], payload, NUM_CMD_PAYLOAD_BYTES);
    }
  }
  CommandPacket(char cmd) : Packet(PTYPE_COMMAND, CMD_T_SIZE ){
    _data[0] = cmd;
    memset(&_data[2], 0, NUM_CMD_PAYLOAD_BYTES);
  }
  
  uint8_t cmd() { return _data[0]; }
  uint8_t args() { return _data[1]; }
  uint8_t* payload() { return &_data[3]; }
};


/*
// string log packet
class StringPacket {
public:
  StringPacket(String s) : Packet(PTYPE_STRING, s.length()+1) {
    memcpy(_data, s.c_str(), s.length());
  }
};
*/

// board telemetry packet
class TelemPacket : public Packet {
public:
  TelemPacket(uint8_t *buf) : Packet( PTYPE_TELEM, TELEM_T_SIZE ) {
    memcpy(_data, buf, _size);
  }
  TelemPacket(uint32_t t, float batt, float tc1_temp, float tc2_temp, uint8_t ir_sig, uint8_t log_num) : Packet( PTYPE_TELEM, TELEM_T_SIZE ) {
    *(uint32_t*)(&_data[0]) = t;
    *(float*)(&_data[4])         = batt;
    *(float*)(&_data[8])         = tc1_temp;
    *(float*)(&_data[12])        = tc2_temp;
    *(uint8_t*)(&_data[16])      = ir_sig;
    *(uint8_t*)(&_data[17])      = log_num;
  }
  
  uint32_t t(){return *((uint32_t*)&_data[0]);}
  float batt(){return *((float*)&_data[4]);}
  float tc1_temp(){return *((float*)&_data[8]);}
  float tc2_temp(){return *((float*)&_data[12]);}
  uint8_t ir_sig(){return ((uint8_t)_data[16]); }
  uint8_t log_num(){ return ((uint8_t)_data[17]); }
};

// once accel sample
class AccPacket : public Packet {
public:
  AccPacket(uint8_t *buf) : Packet( PTYPE_ACCEL, ACC_T_SIZE ) {
    memcpy(_data, buf, _size);
  }
  
  AccPacket(uint16_t x, uint16_t y, uint16_t z, unsigned long t) : Packet( PTYPE_ACCEL, ACC_T_SIZE ) {
    *(uint32_t*)(&_data[0]) = t;
    *(uint16_t*)(&_data[4])      = x;
    *(uint16_t*)(&_data[6])      = y;
    *(uint16_t*)(&_data[8])      = z;
  }

  uint32_t t(){return *((uint32_t*)&_data[0]);}
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
  
  TcPacket(float* data, uint32_t time) : Packet(PTYPE_TC, TC_T_SIZE){
    for(int i = 0; i < TC_COUNT; i++){
      ((float*)_data)[i] = data[i];
    }
    *(uint32_t*)(&_data[4*TC_COUNT]) = time;
  }



  float* data(){ return (float *)_data; }
  uint32_t time(){return *((uint32_t*)&_data[4*TC_COUNT]);}
};

// IMU packet
class IMUPacket : public Packet {
public:
  IMUPacket(float ax, float ay, float az, 
            float gx, float gy, float gz, 
            float mx, float my, float mz, 
            uint32_t t) : Packet(PTYPE_IMU, IMU_T_SIZE) {    
    *(float*)(&_data[0]) = ax;
    *(float*)(&_data[4]) = ay;
    *(float*)(&_data[8]) = az;
    *(float*)(&_data[12]) = gx;
    *(float*)(&_data[16]) = gy;
    *(float*)(&_data[20]) = gz;
    *(float*)(&_data[24]) = mx;
    *(float*)(&_data[28]) = my;
    *(float*)(&_data[32]) = mz;
    *(uint32_t*)(&_data[36]) = t;
  } 
  IMUPacket(uint8_t *buf) : Packet(PTYPE_IMU, IMU_T_SIZE) {  
    memcpy(_data, buf, _size);
  }
};

// iridium packet class
class IridiumPacket {
public:
  IridiumPacket() {;}
  ~IridiumPacket() {;}
};

#endif
