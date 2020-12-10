/* Thermocouple thread measurement test
 *  
 * Like TcTest but with measurement in a timesliced task using TeensyThreads
 * 
 * see the following link regarding sending structs
 * https://arduino.stackexchange.com/questions/45066/radiohead-library-nrf24-sending-and-receiving-struct-data-problem
 */

// radio logger node class needs this defined
#define NODE_ADDRESS CAPSULE_ADDRESS

#include "src/packets.h"
#include "src/TcInterface.h"
#include "src/RadioLogger.h"
#include <IridiumSBD.h>
#include <TeensyThreads.h>
#include <Snooze.h>


RadioLogger logNode;


char types[8] = "KKKKKKKK";
TcInterface tc(types);

Threads::Mutex millis_lock;
Threads::Mutex spi_lock;
Threads::Mutex ser_lock;
Threads::Mutex ircmd_lock;
Threads::Mutex tcdata_lock;

volatile tc_t tcdata;

////////////////////////////////////
//           IRIDIUM VARS
////////////////////////////////////
#define IridiumSerial Serial4
#define DIAGNOSTICS false// Change this to see diagnostics
// Declare the IridiumSBD object
IridiumSBD modem(IridiumSerial);
int signalQuality = -1;
int irerr;
int ircmd = 0;
// command variables
#define IR_SEND_TEST 1
// #define IR_SEND_PACKET etc. 
uint8_t signalBrightness[6] = {0, 2, 10, 50, 100, 250};

unsigned long safeMillis() {
  unsigned long m = 0;
  millis_lock.lock(5);
  m = millis();
  millis_lock.unlock();
  return m;
}

void safePrint(String s) {
  ser_lock.lock(10);
  Serial.print(s);
  ser_lock.unlock();
}

void safePrintln(String s) {
  ser_lock.lock();
  Serial.println(s);
  ser_lock.unlock();
}


// simple thermocouple reading thread keeps conversions happening as fast as possible
// and updates a shared struct with new data
void tc_thread(int inc) {

  safePrintln("TC thread starting");

  while( !spi_lock.lock() );
  if( tc.enable() ){
    safePrintln("TC INIT OK");
  } else {
    safePrintln("TC INIT FAIL!!");
  }
  spi_lock.unlock();

  float vals[8] = {0,0,0,0,0,0,0,0};
  uint8_t faults[8] = {0,0,0,0,0,0,0,0}; 
  unsigned long lastData = safeMillis();

  safePrintln("HERE");
  
  while(1) {

    while( !spi_lock.lock() );
    bool forceStart = safeMillis() - lastData > 5000 ? true : false;
    if( forceStart ) lastData = safeMillis();
    bool gotData = tc.read_all(vals, faults, forceStart);
    spi_lock.unlock();

    if( gotData ){
      digitalWrite(LED_ACT, HIGH);

      unsigned long nn = safeMillis();  

      tcdata_lock.lock(1000);
      if( tcdata_lock.getState() ){
        for( int i=0; i<TC_COUNT; i++ ){
          tcdata.data[i] = vals[i];
        }
        tcdata.time = nn;
      }
      tcdata_lock.unlock();
      

      lastData = nn;
      digitalWrite(LED_ACT, LOW);
    } else {
      
    }
    threads.delay(100);
  }
}


// simple radio thread takes thermocouple data every 5 seconds, builds a packet
// and debugs the packet over ism radio
void radio_thread(int inc) {
  
  while ( !spi_lock.lock() );
  if( logNode.begin() ){
    safePrintln("log node started");
    analogWrite(LED_ISM_TX, 5);
    logNode.setRetries(5);
  } else {
    safePrintln("log node failed to start");
  }
  spi_lock.unlock();

  while(1) {
    threads.delay(5000);

    // copy shared data into local vars
    while( !tcdata_lock.lock() );
    TcPacket *p = new TcPacket(tcdata.data, tcdata.time);
    tcdata_lock.unlock();

    // get SPI mutex and send packet over radio
    safePrintln("* about to send ism packet");
    spi_lock.lock(1000);
    if( spi_lock.getState() ){
      analogWrite(LED_ISM_TX, 100);
      logNode.send(p, STATION_ADDRESS);
      analogWrite(LED_ISM_TX, 5);
      spi_lock.unlock();
    }

    delete p;
  }
}


void iridium_thread(int inc) {

  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(9600);

  safePrint("Powering on modem...");
  digitalWrite(PIN_IR_ENABLE, HIGH);
  analogWrite(LED_IR_ON, 5); // not blinding
  threads.delay(2000);
  safePrintln("done.");


  // Begin satellite modem operation
  safePrintln("Starting modem...");
  irerr = modem.begin();
  if (irerr != ISBD_SUCCESS)
  {
    safePrint("Begin failed: error ");
    safePrintln(irerr);
    if (irerr == ISBD_NO_MODEM_DETECTED)
      safePrintln("No modem detected: check wiring.");
    return;
  }

  // Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  irerr = modem.getSignalQuality(signalQuality);
  if (irerr != ISBD_SUCCESS)
  {
    safePrint("SignalQuality failed: error ");
    safePrintln(irerr);
    //TODO: error handling
    //return;
  }
  safePrint("On a scale of 0 to 5, signal quality is currently ");
  safePrint(signalQuality);
  if( signalQuality >= 0 && signalQuality <= 5 ){
    analogWrite(LED_IR_SIG, signalBrightness[signalQuality]);
  }
  safePrintln(".");

  int cmd = 0;
  unsigned long curMillis = 0;
  unsigned long signalCheckInterval = 15000;
  unsigned long lastSignalCheck = safeMillis();
  
  while(1) {
    threads.delay(50);

    // check for a command set in the command variable
    ircmd_lock.lock(50);
    if(ircmd_lock.getState()){
      if( ircmd != 0){
        cmd = ircmd;
        ircmd = 0;
      }
      ircmd_lock.unlock();
    }
    
    /////////////////////
    // PROCESS TEST SEND COMMAND
    /////////////////////
    if( cmd == IR_SEND_TEST ){
      // Send the message
      safePrintln("* sending test iridium message. This might take several minutes.\r\n");
      analogWrite(LED_IR_TX, 20);
      irerr = modem.sendSBDText("Hello, world!");
      analogWrite(LED_IR_TX, 0);
      if (irerr != ISBD_SUCCESS)
      {
        safePrint("sendSBDText failed: error ");
        safePrintln(irerr);
        if (irerr == ISBD_SENDRECEIVE_TIMEOUT)
          safePrintln("Try again with a better view of the sky.");
      }
    
      else
      {
        safePrintln("*****************");
        safePrintln("*** MESSAGE SENT *");
        safePrintln("*****************");

        // clear command flag on success
        cmd = 0;
      }
    }

    ///////////////////
    // CHECK THE SINGL QUALITY PERIODICALLY
    ////////////////////
    curMillis = safeMillis();
    if( curMillis - lastSignalCheck > signalCheckInterval ){
      irerr = modem.getSignalQuality(signalQuality);
      if (irerr != ISBD_SUCCESS) {
        safePrint("SignalQuality failed: error ");
        safePrintln(irerr);
        //TODO: error handling
        //return;
      } else {
        safePrint("On a scale of 0 to 5, signal quality is currently ");
        safePrint(signalQuality);
        safePrintln(".");
        if( signalQuality >= 0 && signalQuality <= 5 ){
          analogWrite(LED_IR_SIG, signalBrightness[signalQuality]);
        }
      }
      lastSignalCheck = curMillis;
    }
    
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(LED_IR_ON, OUTPUT);
  pinMode(LED_IR_SIG, OUTPUT);
  pinMode(LED_IR_TX, OUTPUT);
  pinMode(LED_ISM_TX, OUTPUT);
  pinMode(LED_ACT, OUTPUT);

  digitalWrite(PIN_IR_ENABLE, LOW);
  pinMode(PIN_IR_ENABLE, OUTPUT);

  delay(2000);

  threads.addThread(tc_thread, 1);
  threads.addThread(radio_thread, 1);

  safePrintln("* starting iridium thread");
  threads.addThread(iridium_thread, 1);
  
  //threads.setSliceMicros(500);
}

void loop() {
  // put your main code here, to run repeatedly:

  threads.delay(1000);

  ser_lock.lock();
  char a = Serial.read();
  ser_lock.unlock();

  if( a == '1' ){
    ircmd_lock.lock();
    ircmd = 1;
    ircmd_lock.unlock();
  }
  
}
