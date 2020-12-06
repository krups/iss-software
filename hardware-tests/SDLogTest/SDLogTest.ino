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

volatile bool acc_ready = false;
volatile acc_t acc_data;

SDLogger sdlog;
int tcLogId, imuLogId, infoLogId;

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
  safePrintln("SD thread starting...");

  sd_lock.lock();
  sdlog.begin();
  sd_lock.unlock();
  
  threads.delay(100);
  
  
  safePrint("Creating IMU logfile...");
  
  sd_lock.lock();
  String logname = "imulog";
  imuLogId = sdlog.createLog(logname);
  sd_lock.unlock();
  
  if( imuLogId >= 0 ){
    safePrint("success, file id = ");
    safePrintln(imuLogId);
  } else {
    
  }

  while( 1 ){
    // check for accel data to log, w muttex
    bool rdy = false;
    acc_lock.lock();
    if( acc_ready ){
      rdy = true;
      acc_ready = false;
    }
    acc_lock.unlock();

    // start logging if we have data
    if( rdy ){
      safePrintln("### writing to tc file###");

      // get mutex on acc data structure and build a data string from it
      String logmsg;
      acc_lock.lock();
      logmsg = String(millis()) + " " + String(acc_data.x) + " " + String(acc_data.y) + " " + String(acc_data.z);
      acc_lock.unlock();
      
      sd_lock.lock();
      byte ret = sdlog.logMsg(imuLogId, logmsg);
      sd_lock.unlock();
      
      if( ret > 0 ){
        safePrint("  wrote "); safePrint(ret); safePrint(" bytes to the file");
      } else {
        safePrintln(" :( wrote 0 bytes to file");
      }
      rdy = false;
    } else {
      //safePrintln("no data ready");
    }
    threads.yield();
  }
}

void acc_thread(int inc) {
  // replace with actual accel analog input pins
  analogReadResolution(12); // 0-4095
  analogReadAveraging(64);  // average 64 samples
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

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
  Serial.begin(115200);
  while(!Serial);


  threads.addThread(sd_thread, 1);
  threads.addThread(acc_thread, 1);
}

int count = 0;

void loop() {
  // put your main code here, to run repeatedly:
 
}
