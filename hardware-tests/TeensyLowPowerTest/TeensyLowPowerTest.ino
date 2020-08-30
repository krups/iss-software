/***************************************
  This shows all the wakeups for deepSleep

  Supported Micros: T-LC/3.x/4.0
****************************************/
#include <Snooze.h>
#include "ICM_20948.h"
#include <RFM69.h>

#define LED_ACT 7
#define LED_ISM 6

#define CS_ISM  9
#define INT_ISM 28
#define PRI_ACT 23



// Load drivers
//SnoozeUSBSerial usb;
SnoozeTimer timer;
SnoozeBlock config_teensy35(timer);

// IMU
ICM_20948_I2C myICM;

// debug radio
//RFM69 radio(CS_ISM, INT_ISM, false, &SPI);

void setup() {
  pinMode(LED_ACT, OUTPUT);
  pinMode(LED_ISM, OUTPUT);
  pinMode(PRI_ACT, OUTPUT);
  digitalWrite(PRI_ACT, LOW);
  
  // init low power usb serial driver
  //while (!usb) { delay(10); }
  //delay(100);
  //usb.println("Starting...");
  //Serial.begin(115200);

  //while(!Serial);
  
  delay(1000);
  //Serial.println("Starting...");
  // set timer in milliseconds
  timer.setTimer(10000);

  Wire.begin();
  Wire.setClock(400000);

  /*if( radio.initialize(RF69_915MHZ,2,100) ){
    digitalWrite(LED_ISM, HIGH);
    delay(200);
    digitalWrite(LED_ISM, LOW);
    delay(200);
    digitalWrite(LED_ISM, HIGH);
    delay(200);
    digitalWrite(LED_ISM, LOW);
  }
  radio.sleep();  */

  initIMU();
  delay(200);
}

void loop() {
  int who;
  /********************************************************
    feed the sleep function its wakeup parameters. Then go
    to deepSleep.
  ********************************************************/
  // prepare for sleep
  prepToSleep();
  // go to sleeeep
  who = Snooze.hibernate( config_teensy35 );// return module that woke processor

  delay(10);
  prepFromSleep();
  delay(500);
  
  //Serial.println("wok up");
  if(myICM.dataReady()){
    digitalWrite(LED_ACT, HIGH);
    delay(500);
    digitalWrite(LED_ACT, LOW);
    //Serial.print("IMU data available");
  }
}

void prepToSleep() {
  myICM.lowPower(true);
  myICM.sleep(true);
}

void prepFromSleep() {
  //Serial.begin(115200);
  myICM.sleep(false);
  myICM.lowPower(false);
}

void initIMU() {
  bool initialized = false;
  while( !initialized ){

    myICM.begin( Wire, 0 ); // i2c coms, address bit=0

    Serial.print( F("Initialization of the sensor returned: ") );
    Serial.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      Serial.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }
  Serial.println("initialized imu");
}
