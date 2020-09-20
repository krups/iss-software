/* Thermocouple measurement test
 *  
 * make sure to only load this on either the safety processor OR the teensy
 * to avoid both devices driving the shared SPI bus at the same time
 */

#include "src/TcInterface/TcInterface.h"

char types[9] = "KKKKKKKK";

TcInterface tc(types);

void setup() {
  Serial.begin(115200);

  delay(5000);
  

  Serial.println("Starting TC Example");

  // put your setup code here, to run once:
  if( tc.enable() ){
    Serial.println("TC OK!");
  } else {
    Serial.println("problem init TC");
  }

delay(100);
  
}

unsigned long readStart = 0;

void loop() {
  // put your main code here, to run repeatedly:
  float vals[8];

  readStart = millis();
  bool forceStart  = false;
  do {
    delay(100);
    forceStart = millis() - readStart > 5000 ? true : false;
    if( forceStart ) readStart = millis();
  } while(!tc.read_all(vals, forceStart));

  for( int i=0; i<8; i++ ){
    Serial.print(vals[i]); Serial.print(", ");
  }
  
  Serial.println();
  delay(10);
}
