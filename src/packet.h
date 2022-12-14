#ifndef PACKET_H
#define PACKET_H

// logging packet structure
#include "config.h"
#include <avr/dtostrf.h>

#define PTYPE_GGA  1  // nmea::GgaData
#define PTYPE_RMC  2  // nmea::RmcData
#define PTYPE_ACC  3
#define PTYPE_IMU  4
#define PTYPE_TC   5
#define PTYPE_PRS  6
#define PTYPE_SPEC 7
#define PTYPE_CMD  16
#define PTYPE_QUAT 17
#define PTYPE_PACKET 99 // compressed packet written to logfile

#define PTYPE_PACKET_REQUEST 8 // sent from flight computer to nano pi to request a packet

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

// type PTYPE_RMC
struct rmc_t {
  uint32_t t; // microprocessor time in ms
  uint16_t time[4]; // hh:mm:ss:us GPS time
  float lat;
  float lon;
  float speed;
  float course;
};

// type PTYPE_GGA
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
int writePacketAsPlaintext(char *dest, uint8_t ptype, uint8_t* data, size_t size, bool json = false) {
  int ret = -1;

  if( ptype == PTYPE_TC) {
    tc_t td;
    memcpy(&td, data, size);

    char tc1Buf[10], tc2Buf[10], tc3Buf[10], tc4Buf[10], tc5Buf[10], tc6Buf[10];
    dtostrf( (float)td.data[0] / UNIT_SCALE, 7, 5, tc1Buf );
    dtostrf( (float)td.data[1] / UNIT_SCALE, 7, 5, tc2Buf );
    dtostrf( (float)td.data[2] / UNIT_SCALE, 7, 5, tc3Buf );
    dtostrf( (float)td.data[3] / UNIT_SCALE, 7, 5, tc4Buf );
    dtostrf( (float)td.data[4] / UNIT_SCALE, 7, 5, tc5Buf );
    dtostrf( (float)td.data[5] / UNIT_SCALE, 7, 5, tc6Buf );

    if( json ){
      ret = sprintf(dest,
                    "{\"id\":\"tc1\", \"value\":%s}\n{\"id\":\"tc2\", \"value\":%s}\n{\"id\":\"tc3\", \"value\":%s}\n{\"id\":\"tc4\", \"value\":%s}\n{\"id\":\"tc5\", \"value\":%s}\n{\"id\":\"tc6\", \"value\":%s}\n",
                    tc1Buf,
                    tc2Buf,
                    tc3Buf,
                    tc4Buf,
                    tc5Buf,
                    tc6Buf
                    );
    } else {
      ret = sprintf(dest,
                    "%d, %d, %d, %d, %d, %d, %d, %d\n", 
                    ptype,
                    td.t, 
                    td.data[0], 
                    td.data[1], 
                    td.data[2],
                    td.data[3],
                    td.data[4],
                    td.data[5]);
    }
    
  } 
  
  // IMU data
  else if(  ptype == PTYPE_IMU ){
    imu_t imu;
    memcpy(&imu, data, size);

    char axBuf[10], ayBuf[10], azBuf[10], gxBuf[10], gyBuf[10], gzBuf[10];
    dtostrf( (float)imu.data[0] / UNIT_SCALE, 7, 5, axBuf );
    dtostrf( (float)imu.data[1] / UNIT_SCALE, 7, 5, ayBuf );
    dtostrf( (float)imu.data[2] / UNIT_SCALE, 7, 5, azBuf );
    dtostrf( (float)imu.data[3] / UNIT_SCALE, 7, 5, gxBuf );
    dtostrf( (float)imu.data[4] / UNIT_SCALE, 7, 5, gyBuf );
    dtostrf( (float)imu.data[5] / UNIT_SCALE, 7, 5, gzBuf );

    if( json ){
      ret = sprintf(dest,
                    "{\"id\": \"ax\", \"value\": %s}\n{\"id\": \"ay\", \"value\": %s}\n{\"id\": \"az\", \"value\": %s}\n{\"id\": \"gx\", \"value\": %s}\n{\"id\": \"gy\", \"value\": %s}\n{\"id\": \"gz\", \"value\": %s}\n", 
                    axBuf,
                    ayBuf,
                    azBuf,
                    gxBuf,
                    gyBuf,
                    gzBuf);
    } else {
      ret = sprintf(dest,
                    "%d, %d, %d, %d, %d, %d, %d, %d\n", 
                    ptype,
                    imu.t, 
                    imu.data[0], 
                    imu.data[1], 
                    imu.data[2],
                    imu.data[3],
                    imu.data[4],
                    imu.data[5]);
    }

  } 
  
  // high g accel data
  else if ( ptype == PTYPE_ACC ){
    acc_t acc;
    memcpy(&acc, data, size);
    char haxBuf[10], hayBuf[10], hazBuf[10];
    dtostrf( (float)acc.data[0] / UNIT_SCALE, 7, 5, haxBuf );
    dtostrf( (float)acc.data[1] / UNIT_SCALE, 7, 5, hayBuf );
    dtostrf( (float)acc.data[2] / UNIT_SCALE, 7, 5, hazBuf );

    if( json ){
      ret = sprintf(dest, 
                    "{\"id\": \"hax\", \"value\": %s}\n{\"id\": \"hay\", \"value\": %s}\n{\"id\": \"haz\", \"value\": %s}\n", 
                    haxBuf,
                    hayBuf,
                    hazBuf);
    } else {
      ret = sprintf(dest, 
                    "%d, %d, %d, %d, %d\n", 
                    ptype,
                    acc.t, 
                    acc.data[0], 
                    acc.data[1], 
                    acc.data[2]);
    }
  } 
  
  // pressure data
  else if ( ptype == PTYPE_PRS ){
    prs_t prs;
    memcpy(&prs, data, size);
    char p1Buf[10], p2Buf[10], p3Buf[10], p4Buf[10], p5Buf[10];
    dtostrf( (float)prs.data[0] / PRS_UNIT_SCALE, 7, 5, p1Buf );
    dtostrf( (float)prs.data[1] / PRS_UNIT_SCALE, 7, 5, p2Buf );
    dtostrf( (float)prs.data[2] / PRS_UNIT_SCALE, 7, 5, p3Buf );
    dtostrf( (float)prs.data[2] / PRS_UNIT_SCALE, 7, 5, p4Buf );
    dtostrf( (float)prs.data[2] / PRS_UNIT_SCALE, 7, 5, p5Buf );

    if( json ){
      ret = sprintf(dest,
              "{\"id\": \"prs1\", \"value\": %s}\n{\"id\": \"prs2\", \"value\": %s}\n{\"id\": \"prs3\", \"value\": %s}\n{\"id\": \"prs4\", \"value\": %s}\n{\"id\": \"prs5\", \"value\": %s}\n", 
              p1Buf,
              p2Buf,
              p3Buf,
              p4Buf,
              p5Buf);
    } else {
      ret = sprintf(dest,
              "%d, %d, %d, %d, %d, %d, %d\n", 
              ptype,
              prs.t, 
              prs.data[0], 
              prs.data[1], 
              prs.data[2],
              prs.data[3],
              prs.data[4]);
    }
  
  // GGA DATA
  } else if(  ptype == PTYPE_GGA ){
    gga_t gga;
    char latBuf[10], lonBuf[10], altBuf[10], hdopBuf[10];
    memcpy(&gga, data, size);

    dtostrf( gga.lat, 7, 5, latBuf );
    dtostrf( gga.lon, 7, 5, lonBuf );
    dtostrf( gga.hdop, 7, 5, hdopBuf );
    dtostrf( gga.alt, 7, 5, altBuf );

    // TODO: fix timestamps in printed string
    if( json ){
      ret = sprintf(dest,
                    "{\"time\": %d, \"lat\": %s, \"lon\": %s, \"alt\": %s, \"hdop\": %s, \"utc\": %d:%d:%d.%d}\n",
                    gga.t, // system time
                    latBuf,
                    lonBuf,
                    altBuf,
                    hdopBuf,
                    gga.time[0],
                    gga.time[1],
                    gga.time[2],
                    gga.time[3]
);
    } else {
      ret = sprintf(dest,
                    "%d, %d, %d,%d,%d,%d, %s, %s, %s, %s\n",
                    ptype,
                    gga.t, // system time
                    gga.time[0],
                    gga.time[1],
                    gga.time[2],
                    gga.time[3],
                    latBuf,
                    lonBuf,
                    altBuf,
                    hdopBuf);
    }

  // RMC DATA
  } else if( ptype == PTYPE_RMC ){
    rmc_t rmc;
    char latBuf[10], lonBuf[10], spdBuf[10], crsBuf[10];
    memcpy(&rmc, data, size);
    
    dtostrf( rmc.lat, 7, 5, latBuf );
    dtostrf( rmc.lon, 7, 5, lonBuf );
    dtostrf( rmc.speed, 7, 5, spdBuf );
    dtostrf( rmc.course, 7, 5, crsBuf );

    // TODO: fix timestamps in printed string
    ret = sprintf(dest,
                  "%d, %d, %d,%d,%d,%d, %s, %s, %s, %s\n",
                  ptype,
                  rmc.t, // system time
                  rmc.time[0],
                  rmc.time[1],
                  rmc.time[2],
                  rmc.time[3],
                  latBuf,
                  lonBuf,
                  spdBuf,
                  crsBuf);

  } 
  
  // spectrometer
  else if( ptype == PTYPE_SPEC ){
    spec_t spec;
    memcpy(&spec, data, size);
    ret = sprintf(dest,
            "SPEC: TBD\n");
  } 

  // packet request to pi
  else if( ptype == PTYPE_PACKET_REQUEST ){
    ret = sprintf(dest, "%d\n", ptype);
  } 
  
  // unknown 
  else {
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
