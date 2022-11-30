#ifndef PACKET_H
#define PACKET_H

// logging packet structure
#include "config.h"

#define PTYPE_GGA  1  // nmea::GgaData
#define PTYPE_RMC  2  // nmea::RmcData
#define PTYPE_ACC  3
#define PTYPE_IMU  4
#define PTYPE_TC   5
#define PTYPE_PRS  6
#define PTYPE_SPEC 7
#define PTYPE_BAR  10
#define PTYPE_CMD  16
#define PTYPE_QUAT 17
#define PTYPE_PACKET 99 // compressed packet written to logfile

#define MAX_CMD_ARGS 10

// type PTYPE_CMD
struct cmd_t {
  uint8_t cmdid;
  uint8_t argc;
  uint8_t argv[MAX_CMD_ARGS];
};

// type PTYPE_PACKET
struct packet_t {
  uint16_t t;    // in seconds
  uint16_t size; // actual data in packet
  char data[SBD_TX_SZ];
};

// high g accel
// type PTYPE_ACC
struct acc_t {
  uint16_t t;      // in seconds
  int16_t data[3]; // hold data * 10
}; // 4 bytes

// type PTYPE_IMU
struct imu_t {
  uint16_t t; // in seconds
  // if  ok & 0xF0 then high g accel booted
  // if ok & 0x0F then imu booted
  uint16_t ok;
  int16_t data[6]; // in 10 * m/s/s and 10 * deg/s
}; // 7 bytes

// type PTYPE_QUAT
struct quat_t {
  uint16_t t; // in seconds
  // if  ok & 0xF0 then high g accel booted
  // if ok & 0x0F then imu booted
  uint16_t ok;
  float data[4]; // in 10 * m/s/s and 10 * deg/s
}; // 20 bytes

// type PTYPE_TMP
struct tc_t {
  uint16_t t;        // in seconds
  int16_t internal; // in 10 * deg C
  int16_t data[NUM_TC_CHANNELS]; // in 10 * deg C
}; // NUM_TC_CHANNELS + 1 bytes

//type PTYPE_BAR
struct bar_t {
  uint16_t t;  // seconds
  int16_t prs; // 10 * hPa
  int16_t alt; // in meters
  int16_t tmp; // in 10 * deg C
}; // 4 bytes

// type PTYPE_PRS
struct prs_t {
  uint16_t t; // in seconds
  uint16_t data[NUM_PRS_CHANNELS];
};

struct rmc_t {
  uint32_t t; // microprocessor time in ms
  uint16_t time[4]; // hh:mm:ss:us GPS time
  float lat;
  float lon;
  float speed;
  float course;
};

struct gga_t {
  uint32_t t;
  uint16_t time[4];
  float lat;
  float lon;
  float hdop;
  float alt;
};

#ifdef USE_GPS
struct tlm_t {
  uint32_t t; // system time when packet was sent in # of scheduler ticks (ms)
  float lat;     // gps latitude
  float lon;     // gps longitude
  float vel;     // gps velocity
  float alt_gps; // gps altitude
  float alt_bar; // barometer altitude
  float barp;    // capsule internal barometric pressure
  float tmp;     // capsule internal temperature
  float bat;     // battery voltage
  int   irsig;   // iridium signal strength
  bool  pardep;  // parachute deployed yes/no
  tc_t tc;      // thermocouple data
};

#else
struct tlm_t {
  uint32_t t; // system time when packet was sent in # of scheduler ticks (ms)
  float tmp;     // capsule internal temperature
  float bat;     // battery voltage
  int   irsig;   // iridium signal strength
  bool  pardep;  // parachute deployed yes/no
  tc_t tc;      // thermocouple data
};
#endif

#ifdef USE_SPECTROMETER
struct spec_t {
  uint32_t t;
  float ch1;
  float ch2;
};
#endif

// not a packet type, used in the groundstation firmware to hold the extra radio receive info
struct rxtlm_t {
  tlm_t tlm;
  int rssi;
  int snr;
};


// helper to format binary data as CSV strings for printing
// needed to send data to pi.
// this needs to be manually updated if packet structure changes in packet.h
int writePacketAsPlaintext(char *dest, uint8_t ptype, uint8_t* data, size_t size) {
  int ret = -1;

  if( ptype == PTYPE_TC) {
    tc_t td;
    memcpy(&td, data, size);
    ret = sprintf(dest,
                  "TC: %d, %d, %d, %d, %d, %d, %d\n", 
                  td.t, 
                  td.data[0], 
                  td.data[1], 
                  td.data[2],
                  td.data[3],
                  td.data[4],
                  td.data[5]);

  } else if(  ptype == PTYPE_IMU ){
    imu_t imu;
    memcpy(&imu, data, size);
    ret = sprintf(dest,
                  "IMU: %d, %d, %d, %d, %d, %d, %d\n", 
                  imu.t, 
                  imu.data[0], 
                  imu.data[1], 
                  imu.data[2],
                  imu.data[3],
                  imu.data[4],
                  imu.data[5]);

  } else if ( ptype == PTYPE_ACC ){
    acc_t acc;
    memcpy(&acc, data, size);
    ret = sprintf(dest, 
                  "ACC: %d, %d, %d, %d\n", 
                  acc.t, 
                  acc.data[0], 
                  acc.data[1], 
                  acc.data[2]);
  } else if ( ptype == PTYPE_PRS ){
    prs_t prs;
    memcpy(&prs, data, size);
    ret = sprintf(dest,
            "PRS: %d, %d, %d, %d, %d, %d\n", 
            prs.t, 
            prs.data[0], 
            prs.data[1], 
            prs.data[2],
            prs.data[3],
            prs.data[4]);

  } else if(  ptype == PTYPE_GGA ){
    gga_t gga;
    memcpy(&gga, data, size);
    ret = sprintf(dest,
            "GGA: TBD\n");

  } else if( ptype == PTYPE_RMC ){
    rmc_t rmc;
    memcpy(&rmc, data, size);
    ret = sprintf(dest,
            "RMC: TBD\n");

  } else if( ptype == PTYPE_SPEC ){
    spec_t spec;
    memcpy(&spec, data, size);
    ret = sprintf(dest,
            "SPEC: TBD\n");
  } else {
    ret = sprintf(dest,
            "Unknown packet type!\n");
  }

  // return number of bytes written to buffer
  return ret;
}

int writePacketAsJson() {

  return 0;
}

#endif
