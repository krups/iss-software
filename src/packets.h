#ifndef PACKETS_H
#define PACKETS_H

#include "config.h"
#include <IridiumSBD.h>

// board telemetry packet
typedef struct {
  float batt;       // battery voltage
  float tc1_temp;   // TC converter 1 cold junction temperature
  float tc2_temp;    // TC converter 2 cold junction temperature
} telem_t;

// high g accel packet
typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} acc_t;

// high g accel stats packet
typedef struct {
  uint16_t x_min;
  uint16_t x_max;
  uint16_t y_min;
  uint16_t y_max;
  uint16_t z_min;
  uint16_t z_max;
} acc_stat_t;

// "compact" thermocouple data packet
// keep track of time in seconds for packet ID
typedef struct {
  uint16_t time;       // in seconds
  float data[TC_COUNT];  // tc measurements
} tc_t;

// "verbose" thermocouple data packet
// includes extra information about fault condition
// on max31856 when measurement was taken
// no timestamp because this is only used in reading
// before logging
typedef struct {
  float data[TC_COUNT];
  byte fault[TC_COUNT];
} tcv_t;

// iridium packet class
class IridiumPacket {
public:
  IridiumPacket() {}
  ~IridiumPacket() {}

private:
    

};

#endif
