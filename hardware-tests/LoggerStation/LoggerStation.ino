/* RadioLoggerStation sketch
 *  
 * Matt Ruffner @KREPE 2020
 * 
 * This code is meant to run on a Feather M0 w/ Radio tethered to a computer
 * this acts as a transparent serial bridge via RFM69 radio
 * all it does is wait for a packet and dump it verbatim to the serial console.
 */

#define NODE_ADDRESS STATION_ADDRESS

#include "packets.h"
#include "RadioLogger.h"

#define LED 13

RadioLogger station;

#define USBSERIAL_DEBUG 1

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

  Serial.setTimeout(10);

  pinMode(LED, OUTPUT);

  delay(3000);
  
  if( station.begin() ){
    digitalWrite(LED, HIGH);
    Serial.println("radio initialized");
  } else {
    digitalWrite(LED, LOW);
    Serial.println("radio failed to init");
  }

  station.setRetries(2);
}

void loop() {
  if( station.available() ){
    if( station.receivePackets() ){

      station.decodePackets();

      //Serial.println("printing...");
      
      station.printPackets();

      station.deletePackets();

      digitalWrite(LED, !digitalRead(LED));
    } else {
      Serial.println("failed to receive packets");
    }
    
  }

  if( Serial.available() ){
    String cmd = Serial.readStringUntil('\n');

    //Serial.println("read in " + cmd);

    // send activation command packet
    if( cmd.equals("act") || cmd.equals("ACT") ){
      CommandPacket p(CMDID_ACTIVATE);
      station.send(&p, CAPSULE_ADDRESS);
      Serial.println("  sent activation command");
    }

    // build packet command 
    if( cmd.equals("bp") || cmd.equals("BP") ){
      CommandPacket p(CMDID_IR_BP);
      station.send(&p, CAPSULE_ADDRESS);
      Serial.println("  sent build packet command");
    }

    // trigger capsule sleep
    if( cmd.equals("sleep") || cmd.equals("SLEEP") ){
      CommandPacket p(CMDID_SLEEP);
      station.send(&p, CAPSULE_ADDRESS);
      Serial.println("   sent sleep command");
    }
  }
  
  //delay(10);
}
