/* LoggerNode sketch
 *  
 * Matt Ruffner @KREPE 2020
 * 
 * This is code to test the wireless debug log functionality of the flight computers
 * RFM69_HCW ISM band radio
 * 
 * meant to be loaded onto the flight computer
 * 
 * see the following link regarding sending structs
 * https://arduino.stackexchange.com/questions/45066/radiohead-library-nrf24-sending-and-receiving-struct-data-problem
 */

// radio logger node class needs this defined
#define NODE_ADDRESS 2

#include "src/RadioLogger/RadioLoggerNode.h"

RadioLoggerNode logNode;

#define LED 6
#define ACT 7

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  pinMode(ACT, OUTPUT);

  delay(2000);
  digitalWrite(ACT, HIGH);
  
  if( logNode.begin() ){
    Serial.println("log node started");
    digitalWrite(LED, HIGH);
  } else {
    Serial.println("log node failed to start");
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  String data("Hello base station\n");
  logNode.log(data);
  
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
}
