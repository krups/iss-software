// This test sketch asynchronosly logs to a file using teensyThreads

#include "SDLogger.h"
#include "packets.h"

// Teensy 3.5 schematic https://www.pjrc.com/teensy/schematic36.png
// shows SD card on separate pins as peripheral SPI bus
// I think this means we don't have to mutex SD calls from clashing
// with TC reading calls
Threads::Mutex sd_lock;
Threads::Mutex ser_lock; // for courteous debug printing
Threads::Mutex acc_lock; // for accessing accel data between acc_thread and sd_thread
Threads::Mutex tc_lock;

volatile bool acc_ready = false;
volatile acc_t acc_data;

volatile bool tc_ready = false;
volatile tc_t tc_data;

volatile bool sampleRequest = false;

SDLogger sdlog;
int accBinLogId, tcBinLogId;

#define TC_LOG_INTERVAL_MS    1000
#define ACCEL_LOG_INTERVAL_MS 250


void safePrint(String s) {
  ser_lock.lock();
  Serial.print(s);
  ser_lock.unlock();
}

void safePrintln(String s) {
  ser_lock.lock();
  Serial.println(s);
  ser_lock.unlock();
}

void sd_thread(int inc) {
  if(USBSERIAL_DEBUG) safePrintln("SD thread starting...");

  while( !sd_lock.lock() );
  sdlog.begin();
  sd_lock.unlock();
  
  threads.delay(100);
  
  
  if(USBSERIAL_DEBUG) safePrint("Creating logfile for accelerometer data...");
  while( !sd_lock.lock() );
  String logname = "acc";
  accBinLogId = sdlog.createLog(logname);
  sd_lock.unlock();
  if( accBinLogId >= 0 ){
    if(USBSERIAL_DEBUG) safePrint("ok, file id = ");
    if(USBSERIAL_DEBUG) safePrintln(accBinLogId);
  } else {
    if(USBSERIAL_DEBUG) safePrintln("fail");
  }

  if(USBSERIAL_DEBUG) safePrint("Creating logfile for thermocouple data...");
  while( !sd_lock.lock() );
  logname = "tc";
  tcBinLogId = sdlog.createLog(logname);
  sd_lock.unlock();
  if( tcBinLogId >= 0 ){
    if(USBSERIAL_DEBUG) safePrint("success, file id = ");
    if(USBSERIAL_DEBUG) safePrintln(tcBinLogId);
  } else {
    if(USBSERIAL_DEBUG) safePrintln("fail");
  }

  bool once = true;

  while( 1 ){
    // check for accel data to log, w muttex
    bool rdy = false;
    while( !acc_lock.lock() );
    if( acc_ready ){
      rdy = true;
      acc_ready = false;
    }
    acc_lock.unlock();

    // start logging if we have data
    if( rdy ){
      if(USBSERIAL_DEBUG) safePrint("* writing acc data");

      // get mutex on acc data structure and build a packet from it
      unsigned long now = millis();
      while( !acc_lock.lock() );
      AccPacket p(acc_data.x, acc_data.y, acc_data.z, now);
      acc_lock.unlock();

      // try logging packet
      while( !sd_lock.lock() );
      byte ret = sdlog.logBin(accBinLogId, &p);
      sd_lock.unlock();
      
      if( ret > 0 ){
        if(USBSERIAL_DEBUG) safePrintln("ok");
      } else {
        if(USBSERIAL_DEBUG) safePrintln("fail");
      }
      
      rdy = false;
    } 

    while( !tc_lock.lock() );
    if( tc_ready ){
      rdy = true;
      tc_ready = false;
    }
    tc_lock.unlock();
    
    if( rdy ){
      if(USBSERIAL_DEBUG) safePrint("* writing to TC file...");

      // get tc data
      while( !tc_lock.lock() );
      TcPacket p(tc_data.data, millis());
      tc_lock.unlock();

      // try to log to binary file
      while( !sd_lock.lock() );
      byte ret = sdlog.logBin(tcBinLogId, &p);
      sd_lock.unlock();

      if( ret > 0 ){
        if(USBSERIAL_DEBUG) safePrintln("ok");
      } else {
        if(USBSERIAL_DEBUG) safePrintln("fail");
      }
      
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
    
    threads.yield();
  }
}

void tc_thread(int inc) {
  unsigned long lastRead = 0;

  safePrintln("TC thread starting..");
  
  while( 1 ) {
    unsigned long now = millis();
    if( now - lastRead > TC_LOG_INTERVAL_MS ){
      while( !tc_lock.lock() );
      tc_data.data[0] = (float)analogRead(A3);
      tc_data.data[1] = (float)analogRead(A4);
      tc_data.data[2] = (float)analogRead(A5);
      tc_data.data[3] = (float)analogRead(A6);
      if( TC_COUNT > 4)
        tc_data.data[4] = (float)analogRead(A7);
      tc_ready = true;
      tc_lock.unlock();
      lastRead = now;
    }
    threads.yield();
  }
}

void acc_thread(int inc) {
  unsigned long lastRead = 0;

  safePrintln("accel thread starting..");
  
  while( 1 ) {
    unsigned long now = millis();
    if( now - lastRead > ACCEL_LOG_INTERVAL_MS ){
      acc_lock.lock();
      acc_data.x = analogRead(A0);
      acc_data.y = analogRead(A1);
      acc_data.z = analogRead(A2);
      acc_ready = true;
      acc_lock.unlock();
      lastRead = now;
    }
    threads.yield();
  }
}

void setup() {
  // put your setup code here, to run once:
  if(USBSERIAL_DEBUG){
    Serial.begin(115200);
    while(!Serial);
  }

  analogReadResolution(12); // 0-4095
  analogReadAveraging(64);  // average 64 samples
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  
  safePrint("sizeof(telem_t)="); safePrintln(String(sizeof(telem_t)));
  safePrint("sizeof(acc_t)="); safePrintln(String(sizeof(acc_t)));
  safePrint("sizeof(acc_stat_t)="); safePrintln(String(sizeof(acc_stat_t)));
  safePrint("sizeof(tc_t)="); safePrintln(String(sizeof(tc_t)));
  safePrint("sizeof(telem_t)="); safePrintln(String(sizeof(telem_t)));
  safePrint("sizeof(tcv_t)="); safePrintln(String(sizeof(tcv_t)));

  
  threads.addThread(sd_thread, 1);
  threads.addThread(acc_thread, 1);
  threads.addThread(tc_thread, 1);
}

int count = 0;

void loop() {
  // put your main code here, to run repeatedly:
 
}
