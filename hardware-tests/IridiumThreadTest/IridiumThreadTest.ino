/* Thermocouple thread measurement test

   Like TcTest but with measurement in a timesliced task using TeensyThreads

   see the following link regarding sending structs
   https://arduino.stackexchange.com/questions/45066/radiohead-library-nrf24-sending-and-receiving-struct-data-problem
*/

// radio logger node class needs this defined
#define NODE_ADDRESS CAPSULE_ADDRESS

#include "src/packets.h"
#include "src/SDLogger.h"
#include "src/TcInterface.h"
#include "src/RadioLogger.h"
#include <IridiumSBD.h>
#include <TeensyThreads.h>
#include <Snooze.h>
#include <ICM_20948.h>

#include <queue>

// IMU
ICM_20948_I2C myICM;

// Load drivers for low power snooze library
// don't load snoozeUSBSerial helper class unless we have usb debugging enabbled
//SnoozeUSBSerial usb;
SnoozeSPI       snoozeSPI;
SnoozeTimer     timer;
#if USBSERIAL_DEBUG
SnoozeUSBSerial snoozeSerial;
SnoozeBlock     config_teensy35(snoozeSPI, timer, snoozeSerial);
#else
SnoozeBlock     config_teensy35(snoozeSPI, timer);
#endif

Threads::Mutex ns_lock; // need sleep lock, access to needSleep var
volatile bool needSleep = false; // is the sleep thread requesting that other threads sleep

Threads::Mutex sr_radio_lock;
volatile bool sr_radio = false;  // is the radio thread ready for sleep

Threads::Mutex sr_tc_lock;
volatile bool sr_tc = false;     // is the tc thread ready to sleep

Threads::Mutex sr_acc_lock;
volatile bool sr_acc = false;    // acccel is ready to sleep

Threads::Mutex sr_imu_lock;
volatile bool sr_imu = false;  // imu is ready to sleep

Threads::Mutex sr_sd_lock;    // is the sd logger ready to sleep
volatile bool sr_sd = false;  // ready to sleep means  all packets written to logfiles

Threads::Mutex sr_ir_lock;
volatile bool sr_ir = false;  // iridium thread is ready to sleep

Threads::Mutex sr_telem_lock;
volatile bool sr_telem = false; // is the telem thread ready to sleep

Threads::Mutex drp_lock;
volatile bool debugRadioPresent = false; // is the debug radio powered?

// for checking if we have a syslog file yet or not
Threads::Mutex syslog_lock;
volatile bool sysLogCreated = false;

// are there packets in the command packet queue
Threads::Mutex cmdPacket_lock;
volatile int cmdPacketCount = 0;
volatile Packet* cmdPackets[MAX_CMDQ_SIZE];

// have we detected activation
volatile bool activation = false;
Threads::Mutex act_lock;


// Thermocouple to digital converter interface with mux control
TcInterface tc(TC_TYPE_STRING);


// sd log interface, supports creating and writing to files
SDLogger sdlog;

////////////////////////
// SD LOG PACKET QUEUES
// packet queues to be taken from for SD logs and added to by
// data generating threads
volatile TcPacket*    tcdataq[MAX_TCQ_SIZE];
volatile AccPacket*   accdataq[MAX_ACCQ_SIZE];
volatile IMUPacket*   imudataq[MAX_IMUQ_SIZE];
volatile TelemPacket* telemdataq[MAX_TELEMQ_SIZE];
// the number of items in each packet queue and mutexes to protect them
volatile int tcq_count    = 0;
volatile int accq_count   = 0;
volatile int imuq_count   = 0;
volatile int telemq_count = 0;
// read and write indexes to use the data arrays as queues with a ring buffer
volatile int tcq_read     = 0;
volatile int tcq_write    = 0;
volatile int accq_read    = 0;
volatile int accq_write   = 0;
volatile int imuq_read    = 0;
volatile int imuq_write   = 0;
volatile int telemq_read  = 0;
volatile int telemq_write = 0;
// MUTEXES FOR ABOVE QUEUE VARS
Threads::Mutex tcq_lock;
Threads::Mutex accq_lock;
Threads::Mutex imuq_lock;
Threads::Mutex telemq_lock;
// END SD LOG PACKET QUEUES
///////////////////////////////


//////////////////////////////
// ISM DEBUG LOG PACKET QUEUES
volatile TcPacket*    logtcdataq[MAX_TCQ_SIZE];
volatile AccPacket*   logaccdataq[MAX_ACCQ_SIZE];
volatile IMUPacket*   logimudataq[MAX_IMUQ_SIZE];
volatile TelemPacket* logtelemdataq[MAX_TELEMQ_SIZE];
// the number of items in each packet queue and mutexes to protect them
volatile int logtcq_count    = 0;
volatile int logaccq_count   = 0;
volatile int logimuq_count   = 0;
volatile int logtelemq_count = 0;
// read and write indexes to use the data arrays as queues with a ring buffer
volatile int logtcq_read     = 0;
volatile int logtcq_write    = 0;
volatile int logaccq_read    = 0;
volatile int logaccq_write   = 0;
volatile int loglogimuq_read = 0;
volatile int logimuq_write   = 0;
volatile int logtelemq_read  = 0;
volatile int logtelemq_write = 0;
// MUTEXES FOR ABOVE QUEUE VARS
Threads::Mutex logtcq_lock;
Threads::Mutex logaccq_lock;
Threads::Mutex logimuq_lock;
Threads::Mutex logtelemq_lock;
// END ISM DEBUG PACKET QUEUES
///////////////////////////////



// ism debug radio interface
static RadioLogger logNode;



// data ready flags for each data generating threads
volatile bool dr_tc    = false;
volatile bool dr_telem = false;
volatile bool dr_imu   = false;
volatile bool dr_acc   = false;
//volatile bool icm_int  = false; // used by the imu thread with the icm-20948
// data ready flag mutex
Threads::Mutex dr_tc_lock;
Threads::Mutex dr_telem_lock;
Threads::Mutex dr_imu_lock;
Threads::Mutex dr_acc_lock;


// Mutex for hardware resources
Threads::Mutex sd_lock;
Threads::Mutex millis_lock;
Threads::Mutex spi_lock;
Threads::Mutex ser_lock;
Threads::Mutex i2c_lock;


// enable flags for ism debug logging from individual threads
volatile bool logen_tc = true;
volatile bool logen_telem = true;


// IRIDIUM VARS
#define IRIDIUM_SERIAL Serial4
#define DIAGNOSTICS false// Change this to see diagnostics
IridiumSBD modem(Serial4);
int signalQuality = -1;
int irerr;
Threads::Mutex ircmd_lock;    // for giving commands to the iridium thread
volatile int ircmd = 0;
uint8_t signalBrightness[6] = {0, 2, 10, 50, 100, 250};


/**************************************************************************
   Thread safe helpers for setting getting and updating flags and printing
*/
// thread safe copy of src boolean to dest boolean.
// dont use do assign a direct value to dest, use safeAssign for that
void safeUpdate(bool *dest, bool *src, Threads::Mutex *m) {
  // wait forever in 1000 ms intervals to do the assignment
  while ( !m->lock(100) );
  *dest = *src;
  m->unlock();
}
// thread safe assignement of a direct truth val to dest
// pass this function a direct truth value
void safeAssign(bool *dest, bool src, Threads::Mutex *m) {
  // wait forever in 1000 ms intervals to do the assignment
  while ( !m->lock(100) );
  *dest = src;
  m->unlock();
}
// thread safe global read
bool safeRead(bool *src, Threads::Mutex *m) {
  bool ret = false;
  while ( !m->lock(100) );
  ret = *src;
  m->unlock();
  return ret;
}
// thread safe access to the current run clock
unsigned long safeMillis() {
  unsigned long m = 0;
  while ( !millis_lock.lock(5) );
  m = millis();
  millis_lock.unlock();
  return m;
}
void safePrint(String s) {
#if USBSERIAL_DEBUG
  while ( !ser_lock.lock(100) );
  Serial.print(s);
  ser_lock.unlock();
#endif
}
void safePrintln(String s) {
#if USBSERIAL_DEBUG
  while ( !ser_lock.lock(100) );
  Serial.println(s);
  ser_lock.unlock();
#endif
}
// end thread safe helpers
/**********************************************************************************/



/***********************************************************************************
   IMU thread helpers
*/
void imuSleep() {
  while ( !i2c_lock.lock(1000) );
  myICM.lowPower(true);
  myICM.sleep(true);
  i2c_lock.unlock();
}
void imuWake() {
  while ( !i2c_lock.lock(1000) );
  myICM.sleep(false);
  myICM.lowPower(false);
  i2c_lock.unlock();
}
bool imuInit() {
  bool initialized = false;
  while ( !initialized ) {

    // start ICM-20948
    if (USBSERIAL_DEBUG) safePrintln("IMU: trying to start ICM-20948");
    while ( !i2c_lock.lock(100) );
    myICM.begin( Wire, 0 ); // i2c coms, address bit=0

    if ( myICM.status != ICM_20948_Stat_Ok ) {
      i2c_lock.unlock();
      threads.delay(100);
    } else {
      i2c_lock.unlock();
      initialized = true;
    }

//    // software reset for known state
//    while ( !i2c_lock.lock(100) );
//    myICM.swReset( );
//    if ( myICM.status != ICM_20948_Stat_Ok) {
//      //safePrintln("IMU: software reset upon init failed");
//      initialized = false;
//    }
//    i2c_lock.unlock();

    imuWake();

    // Set full scale range struct for both acc and gyr
    ICM_20948_fss_t myFSS;
    myFSS.a = gpm16; // set +/-16g sensitivity
    myFSS.g = dps2000; // set 2000 dps
    while ( !i2c_lock.lock(100) );
    myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );
    if ( myICM.status != ICM_20948_Stat_Ok) {
      //safePrintln("IMU: failed to set full scale range");
      initialized = false;
    }
//    myICM.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous );
//    if ( myICM.status != ICM_20948_Stat_Ok) {
//      safePrintln("IMU: failed to set sample mode");
//      initialized = false;
//    }
    ICM_20948_smplrt_t mySmplrt;
    // this is 4.4 hz output data rate for the imu
    // calculate datarate with 1125 / (1+ODR) in Hz
    mySmplrt.a = 255;
    mySmplrt.g = 255;
    myICM.setSampleRate( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt );
    if ( myICM.status != ICM_20948_Stat_Ok) {
      safePrintln("IMU:failed to set sample rate");
      initialized = false;
    }

    // configure interrupts to drive the line low when raw data is ready
    //myICM.cfgIntActiveLow(true);
    //myICM.cfgIntOpenDrain(false);
    //myICM.cfgIntLatch(true);
    //if ( myICM.status != ICM_20948_Stat_Ok) {
    //  safePrintln("IMU: failed to set interrupt logic level");
    //  initialized = false;
    //}
//    myICM.intEnableRawDataReady(true);
//    if ( myICM.status != ICM_20948_Stat_Ok) {
//      safePrintln("IMU: failed to set interrupt source");
//      initialized = false;
//    }
    
    i2c_lock.unlock();
  }
  return initialized;
}
//void imuISR() {
//  icm_int = true;
//}
// imu thread
void imu_thread(int inc) {
  // IMU SETUP
  if ( imuInit() ) {
    if (USBSERIAL_DEBUG) safePrintln("IMU: INIT OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("IMU: INIT FAILED");
  }

  // local copy of sleep status
  bool mNeedSleep = false;

  // dont sleep again until we have logged a measurement
  bool tmss = false;

  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;

  threads.delay(5000);

  // IMU DATA COLLECTION, CONTROL, AND LOGGING
  while (1) {

    // check if we need to prepare to sleep
    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      safePrintln("IMU: sleeping");
      imuSleep();

      // safely let the sleep thread know we are ready
      while ( !sr_imu_lock.lock(1000) );
      sr_imu = true;
      sr_imu_lock.unlock();

      // now we are waiting to be put to sleep, and checking if we've woken up
      while (1) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        // if we woke up, reconfigure the IMU
        if ( !mNeedSleep ) {
          safePrintln("IMU: waking up");
          imuWake();

          // clear the sleep ready flag
          while ( !sr_imu_lock.lock(1000) );
          sr_imu = false;
          sr_imu_lock.unlock();
          break;
        }
      }
    }


    // collect IMU data
    bool gotData = false;
    while ( !i2c_lock.lock(100) );
    if ( myICM.dataReady() ) {
      //safePrintln("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
      myICM.getAGMT();      // The values are only updated when you call 'getAGMT'
      ax = myICM.accX();
      ay = myICM.accY();
      az = myICM.accZ();
      gx = myICM.gyrX();
      gy = myICM.gyrX();
      gz = myICM.gyrX();
      mx = myICM.magX();
      my = myICM.magY();
      mz = myICM.magZ();
      gotData = true;
    }
    i2c_lock.unlock();

    // log IMU data
    if ( gotData ) {
      //safePrintln("IMU: got data");
      unsigned long nn = safeMillis();
      // if radio debug log is enabled, copy the packet into the queue to be sent
      // also conditional on whethere IMU logging is enabled in the ism log settings

      while ( !imuq_lock.lock(100) ); // get lock for telem packet log queue
      if ( imuq_count + 1 < MAX_IMUQ_SIZE ) {
        //safePrintln("creating imu telem packet ");
        imudataq[imuq_write] = new IMUPacket(ax, ay, az, gx, gy, gz, mx, my, mz, nn );
        imuq_write = (imuq_write + 1) % MAX_IMUQ_SIZE;
        imuq_count += 1;
      }
      imuq_lock.unlock();
    }
    threads.delay(500);
  }
}

/**********************************************************************************
   SD Logging thread
*/

void syslog(String s) {
  bool mSysLogCreated = false;
  safeUpdate(&mSysLogCreated, &sysLogCreated, &syslog_lock);
  if ( mSysLogCreated ) {
    unsigned long m = safeMillis();
    while ( !sd_lock.lock(100) );
    sdlog.logMsg(LOGID_SYS, String(m) + "\t" + s);
    sd_lock.unlock();
  }
}

void sd_thread(int inc) {
  bool ret = false; // for return values
  bool mNeedSleep = false;

  Packet *p;
  
  if (USBSERIAL_DEBUG) safePrintln("SD: thread starting...");

  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  sdlog.begin();
  spi_lock.unlock();
  sd_lock.unlock();

  threads.delay(100);

  // create logfile for accelerometer data
  if (USBSERIAL_DEBUG) safePrintln("SD: Creating logfile for accelerometer data...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  String logname = LOGNAME_ACC;
  ret = sdlog.createLog(logname, LOGID_ACC);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: ACC FILE OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create acc file");
  }

  threads.delay(500);

  // create log file for tc data
  if (USBSERIAL_DEBUG) safePrintln("Creating logfile for thermocouple data...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  logname = LOGNAME_TC;
  ret = sdlog.createLog(logname, LOGID_TC);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: TC FILE OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create tc file");
  }

  threads.delay(500);

  // create logfile for telemetry data
  if (USBSERIAL_DEBUG) safePrintln("Creating logfile for telemetry data...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  logname = LOGNAME_TELEM;
  ret = sdlog.createLog(logname, LOGID_TELEM);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: TELEM FILE OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create telem file");
  }

  threads.delay(500);

  // create logfile for imu data
  if (USBSERIAL_DEBUG) safePrintln("Creating logfile for IMU data...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  logname = LOGNAME_IMU;
  ret = sdlog.createLog(logname, LOGID_IMU);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: IMU FILE OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create IMU file");
  }

  threads.delay(500);

  // create syslog file
  if (USBSERIAL_DEBUG) safePrintln("Creating logfile for syslog messages...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  logname = LOGNAME_SYS;
  ret = sdlog.createLog(logname, LOGID_SYS);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: SYSLOG FILE OK");
    safeAssign(&sysLogCreated, true, &syslog_lock);
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create SYSLOG file");
  }


  while ( 1 ) {
    // SLEEP HANDLING
    // check if we need to prepare to sleep
    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      safePrintln("SD: sleeping");

      // safely let the sleep thread know we are ready
      while ( !sr_sd_lock.lock(1000) );
      sr_sd = true;
      sr_sd_lock.unlock();

      // now we are waiting to be put to sleep, and checking if we've woken up
      while (1) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        // if we woke up, reconfigure the IMU
        if ( !mNeedSleep ) {
          safePrintln("SD: waking up");


          // clear the sleep ready flag
          while ( !sr_sd_lock.lock(1000) );
          sr_sd = false;
          sr_sd_lock.unlock();
          break;
        }
      }
    }
    
    
    ///////////////////////
    // MAIN LOGGING

    // telem logging
    // copy packet from the queue and delete its entry in the queue
    // this way we dont hold the telemq_lock unnecessarily while SD ops happen
    if ( telemq_lock.lock() ){
      if ( telemq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new TelemPacket(telemdataq[telemq_read]->data());
        delete telemdataq[telemq_read];
        telemq_read = (telemq_read + 1) % MAX_TELEMQ_SIZE;
        telemq_count -= 1;
        telemq_lock.unlock();
        // now try logging packet
        while( !sd_lock.lock() );
        byte ret = sdlog.logBin(LOGID_TELEM, p);
        delete p;
        sd_lock.unlock();
      } else {
        telemq_lock.unlock();
      }
    }

    //threads.delay(10);

    // acc packet logging
    // check for accel stat data to log, w muttex
    if ( accq_lock.lock() ){
      if ( accq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new AccPacket(accdataq[accq_read]->data());
        delete accdataq[accq_read];
        accq_read = (accq_read + 1) % MAX_ACCQ_SIZE;
        accq_count -= 1;
        accq_lock.unlock();
        // now try logging packet
        while( !sd_lock.lock() );
        byte ret = sdlog.logBin(LOGID_ACC, p);
        delete p;
        sd_lock.unlock();
      } else {
        accq_lock.unlock();
      }
    }

    //threads.delay(10);

    // imu packet logging
    if ( imuq_lock.lock() ){
      if ( imuq_count > 0 ) {
        safePrintln("got imu packet to log to SD");
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new IMUPacket(imudataq[imuq_read]->data());
        delete imudataq[imuq_read];
        imuq_read = (imuq_read + 1) % MAX_IMUQ_SIZE;
        imuq_count -= 1;
        imuq_lock.unlock();
        // now try logging packet
        while( !sd_lock.lock() );
        byte ret = sdlog.logBin(LOGID_IMU, p);
        delete p;
        sd_lock.unlock();
      } else {
        imuq_lock.unlock();
      }
    }


    //threads.delay(10);

    // thermocouple data logging
    if ( tcq_lock.lock() ){
      if ( tcq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new TcPacket((uint8_t*)tcdataq[tcq_read]->data());
        delete tcdataq[tcq_read];
        tcq_read = (tcq_read + 1) % MAX_TCQ_SIZE;
        tcq_count -= 1;
        tcq_lock.unlock();
        // now try logging packet
        while( !sd_lock.lock() );
        byte ret = sdlog.logBin(LOGID_TC, p);
        delete p;
        sd_lock.unlock();
      } else {
        tcq_lock.unlock();
      }
    }
    
    // TODO: check for request to sample a log file
/*
      rdy = false;
      }

      if( once && millis() > 30000 ){
      safePrintln("#####################################\n###########################\nsampling acc log############################3");
      unsigned long sz = 10*((int)TC_T_SIZE);
      safePrint("TC_T_SIZE = "); safePrintln(TC_T_SIZE);
      safePrint("sz = "); safePrintln(sz);
      uint8_t buf[sz];
      while( !sd_lock.lock() );
      sdlog.sample(tcBinLogId, buf, sz);
      sd_lock.unlock();
      once = false;
      }
    */
  }
}



/**********************************************************************************
   Accelerometer thread

*/
void acc_thread(int inc) {

  bool tmss = false;

  while (1) {

  }
}


/***********************************************************************************
   sleep thread
*/
void sleep_thread(int inc) {
  // CONFIGURE SLEEP PARAMS

  int  who;          // output of sleep function
  bool mAct = false; // our copy of activation status
  bool mDrp = false; // our copy of debug radio present;
  int  state = 0;    // state for pre-activation sleep routine

  bool ss_radio = false,
       ss_tc    = false,
       ss_acc   = false,
       ss_imu   = false,
       ss_ir    = false;

  safePrintln("SLEEP: starting sleep thread");

  threads.delay(10000);

  timer.setTimer(10000);


  // SLEEP CONTROL prior to activation
  while (1) {

    // check activation status
    safeUpdate(&mAct, &activation, &act_lock);
    if ( mAct ) {
      safePrintln("SLEEP: DETECTED ACTIVATION");
      break;
    }

    switch (state) {
      // not activated, need to sleep
      case 0:
        // TODO:
        // need logic to prevent sleep thread starting sleep before we have had a
        // chance to make a TC measurement and update our decicision about activation
        threads.delay(10000);

        while ( !ns_lock.lock(10) );
        needSleep = 1;
        ns_lock.unlock();
        safePrintln("SLEEP: requesting threads prepare for sleep");
        state = 1;
        break;

      // needing sleep, have to wait for each thread to be ready
      // need to be kind about checking status of each threads readiness
      case 1:
        // update our copy of each threads sleep status
        safeUpdate(&ss_radio, &sr_radio, &sr_radio_lock);
        safeUpdate(&ss_tc,    &sr_tc,    &sr_tc_lock);
        safeUpdate(&ss_acc,   &sr_acc,   &sr_acc_lock);
        safeUpdate(&ss_imu,   &sr_imu,   &sr_imu_lock);
        safeUpdate(&ss_ir,    &sr_ir,    &sr_ir_lock);

        // the radio thread is always ready to sleep when no radio is present
        safeUpdate(&mDrp, &debugRadioPresent, &drp_lock);
        if ( !ss_radio && mDrp ) {
          ss_radio = false;
        } else {
          ss_radio = true;
        }

        // if all the threads are ready to sleep
        if ( ss_tc && ss_imu && ss_radio) {
          safePrintln("SLEEP: all threads acknowledged request for sleep, proceeding");
          state = 2;
        }
        break;

      // all threads have prepared for sleep, lets do it
      case 2:
        // now we go to sleep
        who = Snooze.hibernate( config_teensy35 ); // return module that woke processor

        // we're awake! let the threads know sleeping has finished... for now
        while ( !ns_lock.lock(10) );
        needSleep = 0;
        ns_lock.unlock();

        safePrintln("SLEEP: WOKE UP AND SET NEED SLEEP = 0");

        // back to needing sleep
        state = 0;
        break;
    }

    threads.delay(50);
  }

  // now we are activated!
  while (1) {
    // mybe just end the thread instead of looping here?
  }
}



/***********************************************************************************
   command processing thread
   receive a packet over debug radio, process it here
   only applies to test missions where receiving commands is applicable
*/
void command_thread(int inc) {



}


/***********************************************************************************
   Thermocouple manager thread
   keeps conversions happening as fast as possible
   and updates a shared struct with new data
*/
void tc_thread(int inc) {
  tcv_t mTcReadings; // for communicating with the TcInterface class
  unsigned long lastData = safeMillis();

  bool mNeedSleep = false;
  bool mAct = false;
  bool ret = false;

  safePrintln("TC: starting thread");

  // don't pass go unless we can init the thermocouples
  while ( !ret ) {
    while ( !spi_lock.lock(1000) );
    ret = tc.enable();
    spi_lock.unlock();

    if ( ret ) {
      if (USBSERIAL_DEBUG) safePrintln("TC: INIT OK");
    } else {
      if (USBSERIAL_DEBUG) safePrintln("TC: INIT FAIL!!");
    }
  }

  // main execution loop
  while (1) {

    // check if we need to prepare to sleep
    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      if (USBSERIAL_DEBUG) safePrintln("TC: sleeping");

      while ( !spi_lock.lock(1000) );
      tc.disable();
      spi_lock.unlock();

      // safely let the sleep thread know we are ready
      while ( !sr_tc_lock.lock(1000) );
      sr_tc = true;
      sr_tc_lock.unlock();

      // now we are waiting to be put to sleep, and checking if we've woken up
      while (1) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        // if we woke up, enable the TCs again
        if ( !mNeedSleep ) {
          if (USBSERIAL_DEBUG) safePrintln("TC: waking up");
          while ( !spi_lock.lock(1000) );
          ret = tc.enable();
          spi_lock.unlock();
          if ( ret ) {
            if (USBSERIAL_DEBUG) safePrintln("TC: successfully reconfigured TC chips");
          } else {
            if (USBSERIAL_DEBUG) safePrintln("TC: [ERROR] could not reconfigure TC chips!");
          }

          // clear the sleep ready flag
          while ( !sr_tc_lock.lock(1000) );
          sr_tc = false;
          sr_tc_lock.unlock();
          break;
        }
      }
    }

    // NOW START TAKING  MEASUREMENTS

    // forceStart prevents TC read hangs if we havent gotten any data in 5 seconds
    bool forceStart = safeMillis() - lastData > 5000 ? true : false;
    if ( forceStart ) lastData = safeMillis();

    // make call to TC interface
    while ( !spi_lock.lock(1000) );
    //safePrintln(String(safeMillis())+ "  TC: mutex was unlocked, got it ");
    bool gotData = tc.read_all(&mTcReadings, forceStart);
    spi_lock.unlock();
    //safePrintln("TC: read vals");

    if ( gotData ) {
      if (USBSERIAL_DEBUG) safePrintln("TC: got TC readings");

      digitalWrite(LED_ACT, HIGH);

      // this data's timestamp
      unsigned long nn = safeMillis();

      // check activation status
      safeUpdate(&mAct, &activation, &act_lock);
      if ( !mAct ) {
        // check for activation conditions
        int exceeds = 0;
        for ( int i = 0; i < TC_COUNT; i++ ) {
          if ( mTcReadings.data[i] >= TC_THRESHOLD ) {
            exceeds += 1;
          }
        }
        if ( exceeds >= TC_CONSENSUS ) {
          if (USBSERIAL_DEBUG) safePrintln("TC: DETECTED ACTIVATION");
          // log this over radio too
          mAct = true;
          while ( !act_lock.lock(1000) );
          activation = true;
          act_lock.unlock();
        }
      }

      // LOG THE DATA 
      // push packet into tc data queue to be logged
      while ( !tcq_lock.lock(100) ); // get lock for telem packet log queue
      if ( tcq_count + 1 < MAX_TCQ_SIZE ) {
        safePrintln("creating tc packet ");
        tcdataq[tcq_write] = new TcPacket(mTcReadings.data, nn);
        tcq_write = (tcq_write + 1) % MAX_TCQ_SIZE;
        tcq_count += 1;
      }
      tcq_lock.unlock();

      // radio log
      // push another copy of our data into queue for radio log if enabled
      if( ISM_DEBUG && logen_tc){
        while ( !logtcq_lock.lock(100) ); // get lock for telem packet log queue
        if ( logtcq_count + 1 < MAX_TCQ_SIZE ) {
          safePrintln("creating radio tc packet ");
          logtcdataq[logtcq_write] = new TcPacket(mTcReadings.data, nn);
          logtcq_write = (logtcq_write + 1) % MAX_TCQ_SIZE;
          logtcq_count += 1;
        }
        logtcq_lock.unlock();
      }
      
      lastData = nn;
      digitalWrite(LED_ACT, LOW);

      
    } else {
      // no new tc data
    }
    threads.delay(100);
  }
}

/***********************************************************************************
   radio interface thread
   publishes any debug messages and packets generated in other threads
   listens for command packets and dispatches them to command thread
*/
void radio_thread(int inc) {
  bool ret = false;
  Packet *p;

  while ( !spi_lock.lock(1000) );
  ret = logNode.begin();
  logNode.setRetries(0);
  spi_lock.unlock();

  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("ISM: log node started");
    analogWrite(LED_ISM_TX, 5); // dim lit
  } else {
    if (USBSERIAL_DEBUG) safePrintln("ISM: log node failed to start, idling thread");
    while (1) threads.delay(1000);
  }

  bool mNeedSleep = false;
  bool newData = false;
  bool sent = false;

  while (1) {

    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      if (USBSERIAL_DEBUG) safePrintln("ISM: sleeping");
      safeAssign(&sr_radio, true, &sr_radio_lock);

      while ( mNeedSleep ) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        if ( !mNeedSleep ) {
          if (USBSERIAL_DEBUG) safePrintln("ISM: waking up");
          safeAssign(&sr_radio, false, &sr_radio_lock);
        }
        threads.delay(50);
      }
    }
    
    // check for a packet in the logtcqueue
    newData = false;
    if ( logtcq_lock.lock() ){
      if ( logtcq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new TcPacket((uint8_t*)logtcdataq[logtcq_read]->data());
        delete logtcdataq[logtcq_read];
        logtcq_read = (logtcq_read + 1) % MAX_TCQ_SIZE;
        logtcq_count -= 1;
        logtcq_lock.unlock();
        newData = true;
      } else {
        logtcq_lock.unlock();
      }
    }
    // radio log the tc packet if we have one
    if( newData ){
      // get SPI mutex and send packet over radio
      if (USBSERIAL_DEBUG) safePrintln("ISM:  about to send ism packet");
      analogWrite(LED_ISM_TX, 100);

      sent = false;
      while ( !sent ) {
        if (  spi_lock.lock(1000) ) {
          if ( logNode.send(p, RH_BROADCAST_ADDRESS) ) {
            sent = true;
          }
          spi_lock.unlock();
          if ( sent ) {
            if (USBSERIAL_DEBUG) safePrintln("sent");
          } else {
            if (USBSERIAL_DEBUG) safePrintln("failed");
          }
        }
      }
      delete p;
      analogWrite(LED_ISM_TX, 5);
    }

    // telem radio log
    // check for a telem packet in the logtelemqueue
    newData = false;
    if ( logtelemq_lock.lock() ){
      if ( logtelemq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new TelemPacket(logtelemdataq[logtelemq_read]->data());
        delete logtelemdataq[logtelemq_read];
        logtelemq_read = (logtelemq_read + 1) % MAX_TELEMQ_SIZE;
        logtelemq_count -= 1;
        logtelemq_lock.unlock();
        newData = true;
      } else {
        logtelemq_lock.unlock();
      }
    }
    
    // send new telem packet if there is one
    if ( newData ) {
      analogWrite(LED_ISM_TX, 100);
      sent = false;
      while ( !sent ) {
        if (  spi_lock.lock(1000) ) {
          if ( logNode.send(p, RH_BROADCAST_ADDRESS) ) {
            sent = true;
          }
          spi_lock.unlock();
          if ( sent ) {
            if (USBSERIAL_DEBUG) safePrintln("sent");
          } else {
            if (USBSERIAL_DEBUG) safePrintln("failed");
          }
        }
      }
      delete p;
      analogWrite(LED_ISM_TX, 5);
    }
    
    threads.delay(100);
  }
}


/***********************************************************************************
   Telem gathering thread for board info
*/
int batt = 0;
void telem_thread(int inc) {
  bool mNeedSleep = false;
  unsigned long ts = 0;
  float vbat = 0; 
  int batt = 0;
  
  if (USBSERIAL_DEBUG) safePrintln("TELEM: starting thread");

  threads.delay(5000);

  while (1) {
    // check for need to sleep and handle properly
    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      if (USBSERIAL_DEBUG) safePrintln("TELEM: sleeping");
      safeAssign(&sr_telem, true, &sr_telem_lock);

      while ( mNeedSleep ) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        if ( !mNeedSleep ) {
          if (USBSERIAL_DEBUG) safePrintln("TELEM: waking up");
          safeAssign(&sr_telem, false, &sr_telem_lock);
        }
        threads.delay(50);
      }
    }

    // process analog conversion to read battery voltage
    // vbat is split through two 100k resistors and connected to PIN_BAT_SENSE
    // resoltution configurable
    batt = analogRead( PIN_BAT_SENSE ); //PIN_BAT_SENSE
    vbat = ((float)(batt)) / 1023.0;
    vbat = vbat * 3.3 * 2;
    

    ts = safeMillis(); // get current as possible timestamp


    while ( !telemq_lock.lock(100) ); // get lock for telem packet log queue
    if ( telemq_count + 1 < MAX_TELEMQ_SIZE ) {
      safePrintln("creating telem packet " + String(batt) + " " + String(vbat));
      telemdataq[telemq_write] = new TelemPacket(ts, vbat, 0.0, 0.0, (uint8_t)0);  // TODO: fill in tc1 and tc2 temp and irsig
      telemq_write = (telemq_write + 1) % MAX_TELEMQ_SIZE;
      telemq_count += 1;
    }
    telemq_lock.unlock();

    if( ISM_DEBUG  && logen_telem ){
      while ( !logtelemq_lock.lock(100) ); // get lock for telem packet log queue
      if ( logtelemq_count + 1 < MAX_TELEMQ_SIZE ) {
        //safePrintln("creating ism telem packet " + String(batt) + " " + String(vbat));
        logtelemdataq[logtelemq_write] = new TelemPacket(ts, vbat, 0.0, 0.0, (uint8_t)0);  // TODO: fill in tc1 and tc2 temp and irsig
        logtelemq_write = (logtelemq_write + 1) % MAX_TELEMQ_SIZE;
        logtelemq_count += 1;
      }
      logtelemq_lock.unlock();
    }

    threads.delay(2000);

    // if ism log enabled and  telem log enabled in wireless log settings,
    // create a packet and add it to the outgoing log msg queue
    // the data is copied into that buffer so the


    // its okay to reuse that same packet object and copy it into the logging threads
    // queue of packets to be logged
    // then the packet goes out of scope and is deleted
  }
}

void iridium_thread(int inc) {

  bool mAct = false;

  // pre activation routine
  while (1) {
    // update the activation state
    safeUpdate(&mAct, &activation, &act_lock);

    // if we are activated, break and move on
    if ( mAct ) {
      if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: activation detected");
      // debug this over radio log too
      break;
    }

    threads.delay(100);
  }

  // now we are activated!

  // Start the serial port connected to the satellite modem
  IRIDIUM_SERIAL.begin(9600);

  if (USBSERIAL_DEBUG) safePrint("IRIDIUM: Powering on modem...");
  digitalWrite(PIN_IR_ENABLE, HIGH);
  //analogWrite(LED_IR_ON, 5); // not blinding
  threads.delay(2000);

  // Begin satellite modem operation
  if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: Starting modem...");
  irerr = modem.begin();
  if (irerr != ISBD_SUCCESS)
  {
    if (USBSERIAL_DEBUG) safePrint("IRIDIUM: Begin failed: error ");
    if (USBSERIAL_DEBUG) safePrintln(irerr);
    if (irerr == ISBD_NO_MODEM_DETECTED)
      if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: No modem detected: check wiring.");
    return;
  }

  // Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  irerr = modem.getSignalQuality(signalQuality);
  if (irerr != ISBD_SUCCESS)
  {
    if (USBSERIAL_DEBUG) safePrint("IRIDIUM: SignalQuality failed: error ");
    if (USBSERIAL_DEBUG) safePrintln(irerr);
    //TODO: error handling
    //return;
  }
  if (USBSERIAL_DEBUG) safePrint("IRIDIUM: On a scale of 0 to 5, signal quality is currently ");
  if (USBSERIAL_DEBUG) safePrint(signalQuality);
  if ( signalQuality >= 0 && signalQuality <= 5 ) {
    //analogWrite(LED_IR_SIG, signalBrightness[signalQuality]);
  }

  int cmd = 0;
  unsigned long curMillis = 0;
  unsigned long signalCheckInterval = 15000;
  unsigned long lastSignalCheck = safeMillis();

  // start main packet send loop
  while (1) {
    threads.delay(50);

    // check for a command set in the command variable
    ircmd_lock.lock(50);
    if (ircmd_lock.getState()) {
      if ( ircmd != 0) {
        cmd = ircmd;
        ircmd = 0;
      }
      ircmd_lock.unlock();
    }

    /////////////////////
    // PROCESS TEST SEND COMMAND
    /////////////////////
    if ( cmd == CMDID_IR_TEST ) {
      // Send the message
      safePrintln("IRIDIUM: sending test iridium message. This might take several minutes.\r\n");
      //analogWrite(LED_IR_TX, 20);
      irerr = modem.sendSBDText("Hello, world!");
      analogWrite(LED_IR_TX, 0);
      if (irerr != ISBD_SUCCESS)
      {
        if (USBSERIAL_DEBUG) safePrint("IRIDIUM: sendSBDText failed: error ");
        if (USBSERIAL_DEBUG) safePrintln(irerr);
        if (irerr == ISBD_SENDRECEIVE_TIMEOUT)
          if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: Try again with a better view of the sky.");
      }

      else
      {
        if (USBSERIAL_DEBUG) safePrintln("*****************");
        if (USBSERIAL_DEBUG) safePrintln("*** MESSAGE SENT *");
        if (USBSERIAL_DEBUG) safePrintln("*****************");

        // clear command flag on success
        cmd = 0;
      }
    }

    ///////////////////
    // CHECK THE SINGL QUALITY PERIODICALLY
    ////////////////////
    curMillis = safeMillis();
    if ( curMillis - lastSignalCheck > signalCheckInterval ) {
      irerr = modem.getSignalQuality(signalQuality);
      if (irerr != ISBD_SUCCESS) {
        if (USBSERIAL_DEBUG) safePrint("IRIDIUM: SignalQuality failed: error ");
        safePrintln(irerr);
        //TODO: error handling
        //return;
      } else {
        if (USBSERIAL_DEBUG) safePrint("IRIDIUM: On a scale of 0 to 5, signal quality is currently ");
        if (USBSERIAL_DEBUG) safePrint(signalQuality);
        if (USBSERIAL_DEBUG) safePrintln(".");
        if ( signalQuality >= 0 && signalQuality <= 5 ) {
          analogWrite(LED_IR_SIG, signalBrightness[signalQuality]);
        }
      }
      lastSignalCheck = curMillis;
    }

  }
}


void setup() {
#if USBSERIAL_DEBUG
  Serial.begin(115200);
#endif


  // LED pin setup
  pinMode(LED_IR_ON, OUTPUT);
  pinMode(LED_IR_SIG, OUTPUT);
  pinMode(LED_IR_TX, OUTPUT);
  pinMode(LED_ISM_TX, OUTPUT);
  pinMode(LED_ACT, OUTPUT);


  // iridium control pin setup
  pinMode(PIN_IR_ENABLE, OUTPUT);
  digitalWrite(PIN_IR_ENABLE, LOW);
  pinMode(PIN_IRIDIUM_TX_ACT, INPUT);
  pinMode(PIN_IRIDIUM_STATUS, INPUT);
  
  // debug radio present (active high)
  pinMode(PIN_ISM_PRESENT, INPUT);

  // high g accel analog input config
  analogReadAveraging(64);
  analogReadResolution(10);
  pinMode(PIN_ACC_X, INPUT);
  pinMode(PIN_ACC_Y, INPUT);
  pinMode(PIN_ACC_Z, INPUT);

  // battery monitoring input config
  // (charging/not charging and level)
  pinMode(PIN_BAT_STAT, INPUT);
  pinMode(PIN_BAT_SENSE, INPUT);

  // config for IMU, start I2C bus and falling edge data ready interrupt pin
  //attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuISR, FALLING);
  pinMode(PIN_IMU_INT, INPUT);
  Wire.begin();
  Wire.setClock(400000);



  delay(2000);

  // start necessary threads
  // the '1' argument has no purpose :)
  // third arg is stack size (default is 1k)
  threads.addThread(sd_thread,      1, 4096);
  threads.addThread(telem_thread,   1, 4096);
  threads.addThread(tc_thread,      1, 4096);
  threads.addThread(acc_thread,     1, 4096);
  threads.addThread(iridium_thread, 1, 4096);
  threads.addThread(imu_thread,     1, 4096);

  threads.addThread(sleep_thread,   1);

  // start the radio thread if its powered on and enabled in config
  // TODO: this thread doesn't like being started first?????
  //   tried increasing the thread memory
  //   all radio spi transactions are done in a mutex
  //   haven't tested absolute order against TC thread which is
  //   the only other thread using spi...
  if ( ISM_DEBUG ) {
    if ( digitalRead(PIN_ISM_PRESENT) ) {
      if (USBSERIAL_DEBUG) safePrintln("ISM: present");
      debugRadioPresent = true;
      threads.addThread(radio_thread, 1, 20000);
    } else {
      debugRadioPresent = false;
      if (USBSERIAL_DEBUG) safePrintln("ISM: NOT PRESENT");
    }
  } else {
    debugRadioPresent = false;
  }

  threads.setSliceMillis(5);
}

void loop() {
  // put your main code here, to run repeatedly:

  //  ser_lock.lock();
  //  char a = Serial.read();
  //  ser_lock.unlock();
  //
  //  if( a == '1' ){
  //    ircmd_lock.lock();
  //    ircmd = 1;
  //    ircmd_lock.unlock();
  //  }

}
