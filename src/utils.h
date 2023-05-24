#ifndef UTILS_H

#include "packet.h"
#include <avr/dtostrf.h>


// helper to format binary data as CSV strings for printing
// needed to send data to pi.
// this needs to be manually updated if packet structure changes in packet.h
int writePacketAsPlaintext(char *dest, uint8_t ptype, uint8_t* data, size_t size, bool json = false) {
  int ret = -1;

  if( ptype == PTYPE_LS_T ) {
    ls_t ls;
    memcpy(&ls, data, size);
    if( json ){
      ret = sprintf(dest,
                    "{\"id\":\"numFiles\", \"value\":%d}\n",
                    ls.numFiles
                    );
    } else {
      ret = sprintf(dest,
                    "%d, %d\n", 
                    ptype,
                    ls.numFiles);
    }
  }

  if( ptype == PTYPE_TC) {
    tc_t td;
    memcpy(&td, data, size);

    // convert floats to strings with dtostrf()
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
    dtostrf( (float)prs.data[3] / PRS_UNIT_SCALE, 7, 5, p4Buf );
    dtostrf( (float)prs.data[4] / PRS_UNIT_SCALE, 7, 5, p5Buf );

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
                    "{\"id\": \"gga\", \"time\": %d, \"lat\": \"%s\", \"lon\": \"%s\", \"alt\": \"%s\", \"hdop\": \"%s\", \"gps_utc\": \"%d:%d:%d.%d\"}\n",
                    gga.t, // system time
                    latBuf,
                    lonBuf,
                    altBuf,
                    hdopBuf,
                    gga.time[0],
                    gga.time[1],
                    gga.time[2],
                    gga.time[3]);
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
    //dtostrf( rmc.course, 7, 5, crsBuf );

    if ( json ){
      ret = sprintf(dest,
                    "{\"id\": \"rmc\", \"time\": %d, \"lat\": \"%s\", \"lon\": \"%s\", \"vel\": \"%s\", \"gps_utc\": \"%d:%d:%d.%d\"}\n",
                    rmc.t, // system time
                    latBuf,
                    lonBuf,
                    spdBuf,
                    rmc.time[0],
                    rmc.time[1],
                    rmc.time[2],
                    rmc.time[3]);
    } else {
      ret = sprintf(dest,
                  "%d, %d, %d,%d,%d,%d, %s, %s, %s\n",
                  ptype,
                  rmc.t, // system time
                  rmc.time[0],
                  rmc.time[1],
                  rmc.time[2],
                  rmc.time[3],
                  latBuf,
                  lonBuf,
                  spdBuf
                  );
    }
    

  } 
  
  // quaternion orientation
  else if( ptype == PTYPE_QUAT ){
    quat_t quat;
    memcpy(&quat, data, size);
    char q0Buf[10], q1Buf[10], q2Buf[10], q3Buf[10];
    dtostrf( quat.data[0], 7, 5, q0Buf ); // w (real)
    dtostrf( quat.data[1], 7, 5, q1Buf ); // i 
    dtostrf( quat.data[2], 7, 5, q2Buf ); // j
    dtostrf( quat.data[3], 7, 5, q3Buf ); // k

    if( json ){
      ret = sprintf(dest,
              "{\"id\": \"quat_x\", \"value\": %s}\n{\"id\": \"quat_y\", \"value\": %s}\n{\"id\": \"quat_z\", \"value\": %s}\n{\"id\": \"quat_w\", \"value\": %s}\n", 
              q1Buf,
              q2Buf,
              q3Buf,
              q0Buf);
    } else {
      ret = sprintf(dest, "%d, %s, %s, %s, %s\n", quat.t, q0Buf, q1Buf, q2Buf, q3Buf);
    }
  } 

  // spectrometer
  else if( ptype == PTYPE_SPEC ){
    spec_t spec;
    memcpy(&spec, data, size);
    if( json ){
      ret = sprintf(dest,
              "{\"id\": \"spec\", \"time\": %d, \"itime\": %d, \"bin1\": %d, \"bin2\": %d, \"bin3\": %d, \"bin4\": %d, \"bin5\": %d, \"bin6\": %d}\n", 
              spec.t,
              spec.itime,
              spec.data[0],
              spec.data[1],
              spec.data[2],
              spec.data[3],
              spec.data[4],
              spec.data[5]);
    } else {
      ret = sprintf(dest,"SPEC: %d, %d, %d, %d, %d, %d, %d, %d\n", spec.t, spec.itime, spec.data[0],spec.data[1],spec.data[2],spec.data[3],spec.data[4],spec.data[5]);
    }
  } 

  // packet request to pi
  else if( ptype == PTYPE_PACKET_REQUEST ){
    if( json ){
      ret = 0;
    } else {
      ret = sprintf(dest, "%d\n", ptype);
    }
  } 
  
  // unknown 
  else {
    if( json ){
      ret = 0;
    } else {
      ret = sprintf(dest,
            "Unknown packet type!\n");
    }
  }

  // return number of bytes written to buffer
  return ret;
}

int writePacketAsJson() {

  return 0;
}


#endif