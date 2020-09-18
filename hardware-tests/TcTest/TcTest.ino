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

  delay(2000);
  // put your setup code here, to run once:
  tc.enable();
}

void loop() {
  // put your main code here, to run repeatedly:
  float vals[8];

  while(!tc.read_all(vals)){}

  for( int i=0; i<8; i++ ){
    Serial.print(vals[i]); Serial.print(", ");
  }
  
  Serial.println();
  delay(10);
}
