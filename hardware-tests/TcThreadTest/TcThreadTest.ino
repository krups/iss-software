/* Thermocouple thread measurement test
 *  
 * Like TcTest but with measurement in a timesliced task using TeensyThreads
 * 
 * see the following link regarding sending structs
 * https://arduino.stackexchange.com/questions/45066/radiohead-library-nrf24-sending-and-receiving-struct-data-problem
 */

// radio logger node class needs this defined
#define NODE_ADDRESS 2

#include <TeensyThreads.h>
#include "src/TcInterface/TcInterface.h"
#include "src/RadioLogger/RadioLoggerNode.h"

#define LED 6
#define ACT 7

RadioLoggerNode logNode;
char types[8] = "KKKKKKKK";
TcInterface tc(types);

Threads::Mutex spi_lock;

struct tc_reading {
  float data;
  bool valid;
};

volatile tc_reading tcVals[8];

void tc_thread(int inc) {

  Serial.println("TC thread starting");

  spi_lock.lock();
  tc.enable();
  spi_lock.unlock();
  
  while(1) {
    float vals[8];

    spi_lock.lock();
    tc.read_all(vals);
    spi_lock.unlock();
    
    for( int i=0; i<8; i++ ){
      tcVals[i].data = vals[i];
      Serial.print(vals[i]); Serial.print(", ");
    }
    Serial.println();
  }
}

void radio_thread(int inc) {
  spi_lock.lock();
  
  if( logNode.begin() ){
    Serial.println("log node started");
    digitalWrite(LED, HIGH);
  } else {
    Serial.println("log node failed to start");
  }
  spi_lock.unlock();

  while(1) {
    threads.delay(1000);
    String data("TC Vals:");
    for( int i=0; i<8; i++ ){
      data += " ";
      data += tcVals[i].data;
    } data += "\n";
    spi_lock.lock();
    logNode.log(data);
    spi_lock.unlock();
  }
}

void setup() {
  Serial.begin(115200);

  delay(2000);

  threads.addThread(tc_thread, 1);
  threads.addThread(radio_thread, 1);
  //threads.setSliceMillis(10);
}

void loop() {
  // put your main code here, to run repeatedly:

}
