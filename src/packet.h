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
#define PTYPE_CMD  16
#define PTYPE_QUAT 17
#define PTYPE_BATT 18
#define PTYPE_PACKET 99 // compressed packet written to logfile

#define PTYPE_PACKET_REQUEST 8 // sent from flight computer to nano pi to request a packet

#define PTYPE_LS_T 100

#define PTYPE_FILE_START  110
#define PTYPE_BLOCK_START 111
#define PTYPE_BLOCK_DATA  112

#define CMD_PAYLOAD_BYTES 11

// type PTYPE_CMD
struct cmd_t {
  uint8_t cmdid;
  uint8_t data[CMD_PAYLOAD_BYTES];
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
}; // 8 bytes

// type PTYPE_IMU
struct imu_t {
  uint16_t t; // in seconds
  // if  ok & 0xF0 then high g accel booted
  // if ok & 0x0F then imu booted
  uint16_t ok;
  int16_t data[6]; // in 10 * m/s/s and 10 * deg/s
}; // 16 bytes

// type PTYPE_QUAT
struct quat_t {
  uint16_t t; // in seconds
  // if  ok & 0xF0 then high g accel booted
  // if ok & 0x0F then imu booted
  uint16_t ok;
  float data[4]; // order is real (r), i, j, k in the data array
}; // 20 bytes

// type PTYPE_TMP
struct tc_t {
  uint16_t t;        // in seconds
  int16_t internal; // in 10 * deg C
  int16_t data[NUM_TC_CHANNELS]; // in 10 * deg C
}; // 16 bytes for 6 channels

// type PTYPE_PRS
struct prs_t {
  uint16_t t; // in seconds
  uint16_t data[NUM_PRS_CHANNELS];
}; // 12 bytes for 5 channels

// type PTYPE_RMC
struct rmc_t {
  uint16_t t; // microprocessor time in seconds * 10
  uint16_t time[4]; // hh:mm:ss:us UTC GPS time
  float lat;
  float lon;
  float speed;
  float course;
}; // 26 bytes

// type PTYPE_GGA
struct gga_t {
  uint16_t t; // microprocessror time in 10*seconds
  uint16_t time[4]; // utc time hh:mm:ss:us 
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
  uint16_t t;
  uint32_t itime;
  uint8_t data[6];
  uint8_t peaks[6];
};
#endif

struct batt_t {
  uint16_t t;
  uint16_t data[BATT_MEASUREMENTS];
};

// not a packet type, used in the groundstation firmware to hold the extra radio receive info
struct rxtlm_t {
  tlm_t tlm;
  int rssi;
  int snr;
};

// holds a response to the LS command, indicating how many file are on the SD card
struct ls_t {
  uint16_t numFiles;
};

// header for data about to be received over radio
struct file_t {
  uint8_t num;
  int blocks;
};

#endif
