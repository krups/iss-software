/* RadioLoggerStation sketch
 *  
 * Matt Ruffner @KREPE 2020
 * 
 * This code is meant to run on a Feather M0 w/ Radio tethered to a computer
 * this acts as a transparent serial bridge via RFM69 radio
 * all it does is wait for a packet and dump it verbatim to the serial console.
 */


#include "packets.h"
#include "RadioLogger.h"

#define LED 13

RadioLogger station;


void safePrint(String s) {
  //ser_lock.lock();
  Serial.print(s);
  //ser_lock.unlock();
}

void safePrintln(String s) {
  //ser_lock.lock();
  Serial.println(s);
  //ser_lock.unlock();
}


void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to compute

  pinMode(LED, OUTPUT);
  
  if( station.begin() ){
    digitalWrite(LED, HIGH);
    Serial.println("radio initialized");
  } else {
    digitalWrite(LED, LOW);
    Serial.println("radio failed to init");
  }
}

void loop() {
  if( station.available() ){

    if( station.receivePackets() ){

      station.printPackets();

      station.deletePackets();

      digitalWrite(LED, !digitalRead(LED));
    }
    
  }
}
