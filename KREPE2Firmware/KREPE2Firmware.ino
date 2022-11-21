/*
 * KREPE-2 Firmware
 *
 * Matt Ruffner, University of Kentucky Fall 2022
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MCP9600.h>
#include <ArduinoNmeaParser.h>
#include <FreeRTOS_SAMD51.h>
#include <Honeywell_ABP.h>
#include <IridiumSBD.h>
#include <semphr.h>
#include <SD.h>
#include <SPI.h>
#include <RFM69.h>
#include <Servo.h>
#include "wiring_private.h"

#include "H3LIS100.h"      // high g accel driver
#include "delay_helpers.h" // rtos delay helpers
#include "config.h"        // project wide defs
#include "packet.h"        // packet definitions
#include "commands.h"      // command spec
#include "pins.h"          // system pinouts
#include "serial_headers.h" // Headers for serial print
#include "brieflz.h"

#define I2CMUX_ADDR (0x70) 

/* Serial 2
 The GPIO on the SAMD processor support multipler serial protocols
 on various compbinations of pins for more flexibility
 this configures pin 13 as RX and 12 as TX of SERCOM3 in hardware.
 These handlers then in turn trigger the handler for the Serial2 object
 in Arduino
*/
Uart Serial2( &sercom3, 13, 12, SERCOM_RX_PAD_1, UART_TX_PAD_0 ) ;
void SERCOM3_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_3_Handler()
{
  Serial2.IrqHandler();
}

// Serial3
Uart Serial3 (&sercom4, A3, A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM4_0_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM4_1_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM4_2_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM4_3_Handler()
{
  Serial3.IrqHandler();
}

// Serial4
Uart Serial4 (&sercom0, A1, A4, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_0_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM0_1_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM0_2_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM0_3_Handler()
{
  Serial4.IrqHandler();
}

// GPS update callbacks
void onRmcUpdate(nmea::RmcData const);
void onGgaUpdate(nmea::GgaData const);

// rename serial ports
#define SERIAL      Serial  // debug serial (USB) all uses should be conditional on DEBUG define
#define SERIAL_PI   Serial1 // UART to Neo Pi port
#define SERIAL_IRD  Serial2 // to iridium modem
#define SERIAL_GPS  Serial3 // to GPS
#define SERIAL_BSMS Serial4 // to BSMS 

// TC to digital objects
Adafruit_MCP9600 mcps[6];

// I2C addresses on MCP breakout board, use this to assign proper channels
// need to specify per breakout if addressing not consistent between boards
const uint8_t MCP_ADDRS[6] = {0x60, 0x61, 0x62, 0x63, 0x64, 0x67};

// freertos task handles
TaskHandle_t Handle_compTask; // compression task handle
TaskHandle_t Handle_tcTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_imuTask; // imu and acc task
TaskHandle_t Handle_prsTask; // barometric sensor task
TaskHandle_t Handle_radTask; // telem radio task handle
TaskHandle_t Handle_monitorTask; // debug running task stats over uart task
TaskHandle_t Handle_bsmsTask; // battery and spectrometer measurement (BSMS) task

// freeRTOS semaphores
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port access semaphore
SemaphoreHandle_t gpsSerSem; // gps serial port acces
SemaphoreHandle_t irdSerSem; // iridium serial semaphore
SemaphoreHandle_t depSem; // deployment status protector
SemaphoreHandle_t irbSem; // iridium buffer protector
SemaphoreHandle_t sigSem; // iridium signal quality protector
SemaphoreHandle_t wbufSem; // SD buffer write semaphore
SemaphoreHandle_t ledSem; // neopixel sepaphore
SemaphoreHandle_t sdSem; // sd card access

// receive and send buffers for iridium transmission
uint8_t rbuf[RBUF_SIZE];
char sbuf[SBUF_SIZE];

// variables for the debug radio
static uint8_t radioTxBuf[RADIO_TX_BUFSIZE]; // packets queued for sending (telem to groundstation)
static uint16_t radioTxBufSize = 0;
static uint8_t radioRxBuf[RADIO_RX_BUFSIZE]; // data 
static uint16_t radioRxBufSize = 0;
#ifdef USE_DEBUG_RADIO
RFM69 radio; // debug radio object
#endif

// variables for the ping pong loging buffers
static uint8_t logBuf1[LOGBUF_BLOCK_SIZE];
static uint8_t logBuf2[LOGBUF_BLOCK_SIZE];
volatile uint32_t logBuf1Pos = 0; // current write index in buffer1
volatile uint32_t logBuf2Pos = 0; // current write index in buffer2
volatile uint8_t activeLog = 1;   // which buffer should be used fo writing, 1 or 2
volatile bool gb1Full = false, gb2Full = false;

// variables relating to coordinating the sending of packets and iridium modem signal status
volatile bool globalDeploy = false;
volatile bool irSig = 0;
volatile bool packetReady = 0;
int gPacketSize = 0;
char gIrdBuf[SBD_TX_SZ];

// for compression thread
uint8_t c_buf[SBD_TX_SZ];      // Compressed buffer
uint8_t prev_c_buf[SBD_TX_SZ]; // last compressed buffer if we exceed threshold
uint8_t uc_buf[10*SBD_TX_SZ];  // Uncompressed buffer
uint32_t workmem[(1UL << (BLZ_HASH_BITS))]; // lookup table for compression

// log file name
char filename[LOGFILE_NAME_LENGTH] = LOGFILE_NAME;

// include here to things are already defined
#include "sample_datfile.h"


// GPS parser object
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);

// low g acc and gyro
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

Adafruit_NeoPixel led(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// honeywell pressure sensor objects
// create Honeywell_ABP instances
Honeywell_ABP ps1(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps2(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps3(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps4(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps5(0x78, 0, 160.0, "kpa");

//  high g accel
H3LIS100 highg = H3LIS100(0x1234); // 0x1234 is an arbitrary sensor ID, not an I2C address

// IRIDIUM MODEM OBJECT
IridiumSBD modem(SERIAL_IRD);

static void BSMSThread(void *pvParameters)
{
  // TODO: receive data over UART from BSMS

  float vbat = 0.0;

  while (1)
  {
    // measure VBAT voltage
    vbat = analogRead(PIN_VBAT) / 1023.0 * 3.3 * 2.0;

    // vin = measurement from BSMS

    // read in spectrometer over Serial4
  
    myDelayMs(1000);
  }

  vTaskDelete( NULL );
}

// communicate with NeoPi for packet creation
static void piThread(void *pvParameters)
{
  // TODO: receive data over UART from BSMS Serial1

  while (1)
  {
    // monitor the overflow buffer and send all that information over the pi serial so it can log it 
    // and use it to build packets

    // send the magic byte to ask for a binary packet back to send, then place that in the iridium buffer. 
    
  }

  vTaskDelete( NULL );
}


// radio thread
// in charge of the RFM69 debug radio 
// sending packets out as telemetry and receiving commands and 
// dispatching them to other threads
// see node demo sketch for reference: https://github.com/LowPowerLab/RFM69/blob/master/Examples/Node/Node.ino

static void radThread(void *pvParameters)
{
  bool initOk = true;
  
  radio.initialize(FREQUENCY, NODE_ADDRESS_TESTNODE, NETWORK_ID);
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
  radio.encrypt(ENCRYPTKEY);

  int rxBufPos = 0;
  int txBufPos = 0;

  while( 1 ){

    // first get access to SPI bus
    if ( xSemaphoreTake( sdSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      
      // check for any received packets, but only if they will fit in our RX buffer
      if (radio.receiveDone())
      {
        if( (radioRxBufSize + radio.DATALEN) < RADIO_RX_BUFSIZE) {
          memcpy(radioRxBuf, radio.DATA, radio.DATALEN);

          #ifdef DEBUG_RADIO
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
            Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
            for (byte i = 0; i < radio.DATALEN; i++)
              Serial.print((char)radio.DATA[i]);
            Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
            xSemaphoreGive( dbSem );
          }
          #endif
        }

        // ack even if we didb't copy the data to our buffer
        if (radio.ACKRequested())
        {
          radio.sendACK();
          #ifdef DEBUG_RADIO
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
            //Serial.print(" - ACK sent");
            xSemaphoreGive( dbSem );
          }
          #endif
        }
          
        }
        
      
      xSemaphoreGive( sdSem );
    } // end sd access

    // now check if we have any packets in the TX buffer.

    // if there are packets of data in the Tx buffer 
    // the get the sd semaphore and try to send all that are in there
    // sending with retry can take time, so put the spi access semaphore inside the send loop
    txBufPos = 0;
    // need to get access to send buffer and SD buffer
    if( radioTxBufSize > 0 ){
      
      // we can only send 60 bytes at a time, so look through the buffer and send as much as possible 
      // at once until the buffer is empty
      while (radioTxBufSize - txBufPos > 0)
      {
        // first get access to SPI bus
        if ( xSemaphoreTake( sdSem, ( TickType_t ) 1000 ) == pdTRUE ) {

          // if there is more than 60 bytes in the send buffer, send 60 of it over and over
          if( (radioTxBufSize - txBufPos) > 60 ){
            if (radio.sendWithRetry(NODE_ADDRESS_STATION, &radioTxBuf[txBufPos], 60 )){
              // success
              //Serial.print(" ok!");
              txBufPos += 60;
            } else {
              // sending failed
              Serial.print(" nothing...");
            }
          } else { // else if there is 60 or less bytes in the send buffer, just send what is there
            if (radio.sendWithRetry(NODE_ADDRESS_STATION, &radioTxBuf[txBufPos], (radioTxBufSize - txBufPos) )){
              // send success
              //Serial.print(" ok!");
              txBufPos += (radioTxBufSize - txBufPos);
            } else {
              // sending failed 
              Serial.print(" nothing..."); 
            }
          }

          

          xSemaphoreGive( sdSem );
        } // end access to the SPI bus
      }

      radioTxBufSize = 0;
    } // end if radioTxBufSize > 0
  }

  vTaskDelete( NULL );
}

void ledColor(int type) {
  switch (type) {
  case ERR_BOOT:
    led.setPixelColor(0, led.Color(150, 0, 0));
    break;
  case ERR_2:
    led.setPixelColor(0, led.Color(150, 0, 150));
    break;
  case ERR_3:
    led.setPixelColor(0, led.Color(150, 150, 0));
    break;
  case ERR_4:
    led.setPixelColor(0, led.Color(0, 150, 150));
    break;
  default:
    led.setPixelColor(0, led.Color(100, 100, 100));
  }
  led.show();
}

void ledOk() {
  ledColor(ERR_BOOT);
}


void onRmcUpdate(nmea::RmcData const rmc)
{
  rmc_t data;

  #ifdef DEBUG_GPS
  //if (rmc.is_valid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeRmc(rmc, SERIAL);
      xSemaphoreGive( dbSem );
    }
  //}
  #endif

  // TODO: write data to log buffer
  data.t = xTaskGetTickCount();
  data.time[0] = (uint16_t)rmc.time_utc.hour;
  data.time[1] = (uint16_t)rmc.time_utc.minute;
  data.time[2] = (uint16_t)rmc.time_utc.second;
  data.time[3] = (uint16_t)rmc.time_utc.microsecond;
  data.lat = rmc.latitude;
  data.lon = rmc.longitude;
  data.speed = rmc.speed;
  data.course = rmc.course;

  // try to write the gps rmc data to the SD log buffer
  while( writeToLogBuf(PTYPE_RMC, &data, sizeof(rmc_t)) > 0);
}

// helper to print RMC messages to a stream
void writeRmc(nmea::RmcData const rmc, Stream &pipe)
{
  pipe.print("RMC ");

  if      (rmc.source == nmea::RmcSource::GPS)     pipe.print("GPS");
  else if (rmc.source == nmea::RmcSource::GLONASS) pipe.print("GLONASS");
  else if (rmc.source == nmea::RmcSource::Galileo) pipe.print("Galileo");
  else if (rmc.source == nmea::RmcSource::GNSS)    pipe.print("GNSS");

  pipe.print(" ");
  pipe.print(rmc.time_utc.hour);
  pipe.print(":");
  pipe.print(rmc.time_utc.minute);
  pipe.print(":");
  pipe.print(rmc.time_utc.second);
  pipe.print(".");
  pipe.print(rmc.time_utc.microsecond);

  if (rmc.is_valid)
  {
    pipe.print(" : LON ");
    pipe.print(rmc.longitude, 5);
    pipe.print(" ° | LAT ");
    pipe.print(rmc.latitude, 5);
    pipe.print(" ° | VEL ");
    pipe.print(rmc.speed);
    pipe.print(" m/s | HEADING ");
    pipe.print(rmc.course);
    pipe.print(" °");
  }

  pipe.println();
}

// helper to write formatted gps string to stream 
void writeGga(nmea::GgaData const gga, Stream &pipe)
{
  pipe.print("GGA ");

  if      (gga.source == nmea::GgaSource::GPS)     pipe.print("GPS");
  else if (gga.source == nmea::GgaSource::GLONASS) pipe.print("GLONASS");
  else if (gga.source == nmea::GgaSource::Galileo) pipe.print("Galileo");
  else if (gga.source == nmea::GgaSource::GNSS)    pipe.print("GNSS");

  pipe.print(" ");
  pipe.print(gga.time_utc.hour);
  pipe.print(":");
  pipe.print(gga.time_utc.minute);
  pipe.print(":");
  pipe.print(gga.time_utc.second);
  pipe.print(".");
  pipe.print(gga.time_utc.microsecond);

  if (gga.fix_quality != nmea::FixQuality::Invalid)
  {
    pipe.print(" : LON ");
    pipe.print(gga.longitude, 5);
    pipe.print(" ° | LAT ");
    pipe.print(gga.latitude, 5);
    pipe.print(" ° | Num Sat. ");
    pipe.print(gga.num_satellites);
    pipe.print(" | HDOP =  ");
    pipe.print(gga.hdop);
    pipe.print(" m | Altitude ");
    pipe.print(gga.altitude);
    pipe.print(" m | Geoidal Separation ");
    pipe.print(gga.geoidal_separation);
    pipe.print(" m");
  }

  pipe.println();
}

// hopefully not called from an interrup by the GPS parser library
// this function takes the gps fix data and pushes it into the logging queue
void onGgaUpdate(nmea::GgaData const gga)
{
  gga_t data;

  #ifdef DEBUG_GPS
  //if (gga.fix_quality != nmea::FixQuality::Invalid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeGga(gga, Serial);
      xSemaphoreGive( dbSem );
    }
  //}
  #endif

  // TODO: write data to log buffer
  data.t = xTaskGetTickCount();
  data.time[0] = (uint16_t)gga.time_utc.hour;
  data.time[1] = (uint16_t)gga.time_utc.minute;
  data.time[2] = (uint16_t)gga.time_utc.second;
  data.time[3] = (uint16_t)gga.time_utc.microsecond;
  data.lat = gga.latitude;
  data.lon = gga.longitude;
  data.hdop = gga.hdop;
  data.alt = gga.altitude;

  // try to write the gps gga data to the SD log buffer
  while( writeToLogBuf(PTYPE_GGA, &data, sizeof(gga_t)) > 0);
}

// thread safe copy of src boolean to dest boolean.
// dont use do assign a direct value to dest, use safeAssign for that
bool safeUpdate(bool &dest, bool &src, SemaphoreHandle_t &m) {
  // wait100 ticks to do the assignment
  if ( xSemaphoreTake( m, ( TickType_t ) 100 ) == pdTRUE ) {
    dest = src;
    xSemaphoreGive( m ); //  free the mutex
    return true; // assignment success
  } else {
    return false;  // if no assignment was made
  }
}

// thread safe assignement of a direct truth val to dest
// pass this function a direct truth value
bool safeAssign(bool &dest, bool src, SemaphoreHandle_t &m) {
  // wait100 ticks to do the assignment
  if ( xSemaphoreTake( m, ( TickType_t ) 100 ) == pdTRUE ) {
    dest = src;
    xSemaphoreGive( m ); //  free the mutex
    return true; // assignment success
  } else {
    return false;  // if no assignment was made
  }
}

// thread safe global read
bool safeRead(bool &src, SemaphoreHandle_t &m) {
  bool ret = false;
  // wait100 ticks to do the assignment
  if ( xSemaphoreTake( m, ( TickType_t ) 100 ) == pdTRUE ) {
    ret = src;
    xSemaphoreGive( m ); //  free the mutex
  }
  return ret;
}

void select_i2cmux_channel(uint8_t c)
{
  if( c > 7 ) c = 7;
  uint8_t val = 1 << c;
  Wire.beginTransmission(I2CMUX_ADDR);
  Wire.write(val);
  Wire.endTransmission();
}

/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * GPS serial monitoring thread
*/

static void gpsThread( void *pvParameters )
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("GPS thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {
    //if ( xSemaphoreTake( gpsSerSem, ( TickType_t ) 100 ) == pdTRUE ) {
      while (SERIAL_GPS.available()) {
        parser.encode((char)SERIAL_GPS.read());
      }
    //  xSemaphoreGive( gpsSerSem );
    //}

      vTaskDelay(10);
  }

  vTaskDelete( NULL );
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * IMU Monitoring thread
 *
 * Collect IMU data and store statistics on it to the log file
 * 
 * TODO: update for BNO088, H3LIS100 and IMC204
 *
*/
static void imuThread( void *pvParameters )
{
  bool bno_ok = false, highg_ok = false;
  imu::Vector<3> rawAcc;
  imu::Vector<3> rawGyr;
  sensors_event_t event;
  acc_t accData;
  imu_t imuData;
  // TODO: implement
  // unsigned long sample_period_ms = 20;
  unsigned long last_sample_time = 0;


  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("IMU thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
    // init BNO055 IMU
    if( bno.begin() ) {
      bno_ok = true;

      // now configure the bno to be +/-16g (4g is default)
      // gyr setting of +/-2000 dps is default
      //bno.setAccFSR(Adafruit_BNO055::ACC_FSR_16G);
    }

    // init H3LIS100 accelerometer
    if( highg.begin() ){
      highg_ok = true;
    }

    xSemaphoreGive( i2c1Sem );
  }

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.print("IMU: init BNO: ");
    Serial.print(bno_ok);
    Serial.print(", init H3LIS: ");
    Serial.println(highg_ok);
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {
    myDelayMs(IMU_SAMPLE_PERIOD_MS);

    // try to acquire lock on i2c bus and take measurements
    // from the IMU and high g accelerometer
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      // get low g acceleration and gyro measurements
      rawAcc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      rawGyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

      // get high g accel measurements
      highg.getEvent(&event);

      xSemaphoreGive( i2c1Sem );
    }

    last_sample_time = xTaskGetTickCount();

    // copy high g acc data (h3lis100) into logging structs
    accData.t = last_sample_time / TIME_SCALE;
    accData.data[0] = event.acceleration.x * UNIT_SCALE;
    accData.data[1] = event.acceleration.y * UNIT_SCALE;
    accData.data[2] = event.acceleration.z * UNIT_SCALE;

    // copy low g and gyro data (bno055 imu)
    imuData.t = last_sample_time / TIME_SCALE;
    imuData.data[0] = rawAcc.x() * UNIT_SCALE;
    imuData.data[1] = rawAcc.y() * UNIT_SCALE;
    imuData.data[2] = rawAcc.z() * UNIT_SCALE;
    imuData.data[3] = rawGyr.x() * UNIT_SCALE;
    imuData.data[4] = rawGyr.y() * UNIT_SCALE;
    imuData.data[5] = rawGyr.z() * UNIT_SCALE;

    // report sensor init status
    imuData.ok = 0;
    if( bno_ok )   imuData.ok |= 0x0F;
    if( highg_ok ) imuData.ok |= 0xF0;

    // try to write the imu and acc structs to the SD log buffer
    while( writeToLogBuf(PTYPE_IMU, &imuData, sizeof(imu_t)) > 0);
    while( writeToLogBuf(PTYPE_ACC, &accData, sizeof(acc_t)) > 0);



    #ifdef DEBUG_IMU
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("BNO055 aX: ");
      Serial.print(rawAcc.x(), 4);
      Serial.print("\taY: ");
      Serial.print(rawAcc.y(), 4);
      Serial.print("\taZ: ");
      Serial.println(rawAcc.z(), 4);

      Serial.print("BNO055 gX: ");
      Serial.print(rawGyr.x(), 4);
      Serial.print("\tgY: ");
      Serial.print(rawGyr.y(), 4);
      Serial.print("\tgZ: ");
      Serial.println(rawGyr.z(), 4);

      Serial.print("H3LIS100 aX: ");
      Serial.print(event.acceleration.x, 4);
      Serial.print("\taY: ");
      Serial.print(event.acceleration.y, 4);
      Serial.print("\taZ: ");
      Serial.println(event.acceleration.z, 4);

      xSemaphoreGive( dbSem );
    }
    #endif

  }

  vTaskDelete( NULL );
}


/**********************************************************************************
 * barometric pressure monitoring thread
 * 
 * uses I2C mux to communicate with 5 honeywell ported pressure sensors
*/
static void prsThread( void *pvParameters )
{
  bool init = false;
  prs_t data;
  uint16_t pressures[5];  
  unsigned long lastRead = 0;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Perssure thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {

    if( xTaskGetTickCount() - lastRead > PRS_SAMPLE_PERIOS_MS ) {  
      lastRead = xTaskGetTickCount(); 

      if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
        select_i2cmux_channel(MUXCHAN_PS1);
        ps1.update();
        select_i2cmux_channel(MUXCHAN_PS2);
        ps2.update();
        select_i2cmux_channel(MUXCHAN_PS3);
        ps3.update();
        select_i2cmux_channel(MUXCHAN_PS4);
        ps4.update();
        select_i2cmux_channel(MUXCHAN_PS5);
        ps5.update();
        xSemaphoreGive( i2c1Sem ); 
      }
      
      #ifdef DEBUG_PRESSURE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        Serial.print("Pressures: ");
        SERIAL.print(ps1.pressure()); SERIAL.print(", ");
        SERIAL.print(ps2.pressure()); SERIAL.print(", ");
        SERIAL.print(ps3.pressure()); SERIAL.print(", ");
        SERIAL.print(ps4.pressure()); SERIAL.print(", ");
        SERIAL.print(ps5.pressure()); SERIAL.println();
        xSemaphoreGive( dbSem ); 
      }
      #endif
      
      pressures[0] = (uint16_t)(ps1.pressure()*PRS_UNIT_SCALE);
      pressures[1] = (uint16_t)(ps2.pressure()*PRS_UNIT_SCALE);
      pressures[2] = (uint16_t)(ps3.pressure()*PRS_UNIT_SCALE);
      pressures[3] = (uint16_t)(ps4.pressure()*PRS_UNIT_SCALE);
      pressures[4] = (uint16_t)(ps5.pressure()*PRS_UNIT_SCALE);
      
      // copy pressure data to struct to be sent
      prs_t data;
      data.t = (uint16_t)(xTaskGetTickCount() / TIME_SCALE);
      for( int i=0; i<NUM_PRS_CHANNELS; i++ ){
        data.data[i] = pressures[i];
      }

      // try to write this data to the log buffer
      while( writeToLogBuf(PTYPE_PRS, &data, sizeof(prs_t)) > 0);

      myDelayMs(1000);
    } else {
      taskYIELD();
    }
  }
    
  vTaskDelete( NULL );  
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * Iridium thread
*/
static void irdThread( void *pvParameters )
{
  bool fix_valid = false;
  uint8_t buf[SBD_TX_SZ];
  int bufLen = 0;
  bool pready = false;
  int mSq = 0, irerr; // signal quality, modem operation return code
  unsigned long lastSignalCheck = 0, lastPacketSend = 0;

  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Iridium thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  myDelayMs(1000); // give modem time to power up

  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Iridium thread: trying to start modem!...");
    xSemaphoreGive( dbSem );
  }
  #endif

  // init the iridium library and check signal strength
  irerr = modem.begin();

  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("IRIDIUM: called modem.begin()");
    xSemaphoreGive( dbSem );
  }
  #endif


  while( irerr != ISBD_SUCCESS ){
    #ifdef DEBUG_IRD
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: Begin failed: error " + String(irerr));
      xSemaphoreGive( dbSem );
    }
    #endif

    if( irerr == ISBD_NO_MODEM_DETECTED ){
      #ifdef DEBUG_IRD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.println("IRIDIUM: No modem detected: check wiring. ");
        xSemaphoreGive( dbSem );
      }
      #endif
    }

    irerr = modem.begin();

    myDelayMs(1000);
  }

  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("IRIDIUM: modem initialized!");
    xSemaphoreGive( dbSem );
  }
  #endif


  // Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  irerr = modem.getSignalQuality(mSq);
  if( irerr != ISBD_SUCCESS ){

    #ifdef DEBUG_IRD
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: SignalQuality failed: error " + String(irerr));
      //syslog("IRIDIUM: failed to get first signal strength reading");
      //TODO: error handling
      //return;
      xSemaphoreGive( dbSem );
    }
    #endif


  } else {
    #ifdef DEBUG_IRD
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: first signal strength: " + String(mSq));
      //syslog("IRIDIUM: failed to get first signal strength reading");
      //TODO: error handling
      //return;
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  //
  // MAIN TASK LOOP
  //
  while(1) {
    // handle a thread asking to send a packet, also periodically check the signal strength

    // periodically check the signal strength
    if( xTaskGetTickCount() - lastSignalCheck > CHECK_SIGNAL_PERIOD ){
      irerr = modem.getSignalQuality(mSq);
      if( irerr != ISBD_SUCCESS ){

        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: get SignalQuality failed: error " + String(irerr));
          //syslog("IRIDIUM: failed to get first signal strength reading");
          //TODO: error handling
          //return;
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: signal strength: " + String(mSq));
          //syslog("IRIDIUM: failed to get first signal strength reading");
          //TODO: error handling
          //return;
          xSemaphoreGive( dbSem );
        }
        #endif
      }

      if ( xSemaphoreTake( sigSem, ( TickType_t ) 100 ) == pdTRUE ) {
        irSig = mSq;
        xSemaphoreGive( sigSem );
      }

      lastSignalCheck = xTaskGetTickCount();
    }

    // check the global flag to see if there is a packet ready
    if ( xSemaphoreTake( irbSem, ( TickType_t ) 10 ) == pdTRUE ) {
      if( packetReady ){
       pready = true;
      }
      xSemaphoreGive( irbSem );
    }

    // IS IT TIME TO SEND A PACKKAGE??
    if( xTaskGetTickCount() - lastPacketSend > IRIDIUM_PACKET_PERIOD &&
        (mSq > 0) &&
        SEND_PACKETS &&
        pready){

      #ifdef DEBUG_IRD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: sending packet");
      xSemaphoreGive( dbSem );
      }
      #endif


      // fill the buff with compressed data if the global flag
      // is set telling us there is one available from the compression thread
      if ( xSemaphoreTake( irbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        if( packetReady ){
          memcpy(buf, gIrdBuf, gPacketSize);
          bufLen = gPacketSize;

          // tell the compression thread to go ahead and refill the buffer
          // with the next compressed packet
          packetReady = false;
        }
        xSemaphoreGive( irbSem );
      }

      // try to send the packet
      irerr = modem.sendSBDBinary(buf, bufLen);

      if (irerr != ISBD_SUCCESS) { // sending failed
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: failed to send packet :( error " + String(irerr));
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      else { // send success
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: successfully sent a packet!!!!!!");
          xSemaphoreGive( dbSem );
        }
        #endif
        // only update lastPacketSned timestamp if we were successful so that
        // the modem will try again asap
        lastPacketSend = xTaskGetTickCount();

        pready = false; // only set this if the send succeeds
      }
    }

    myDelayMs(50);

  } // end task loop

  vTaskDelete( NULL );
}

/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * TPM Serial monitoring thread and helpers
*/

/* safely read from an MCP device
 *
*/
float readMCP(int id) {
  float hot=0.0;//ambient=0.0,adcval=0.0;

  if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {

      hot = mcps[id].readThermocouple();

      xSemaphoreGive( i2c1Sem );
  }

  return hot;
}

// initialize an MCP device
// made to run before task scheduler is started
bool initMCP(int id) {
  bool ok = true;
  uint8_t addr = MCP_ADDRS[id];
  uint8_t tries = 0;
  uint8_t max_tries = 100;

  #if DEBUG
  SERIAL.print("Starting MCP #");SERIAL.print(id);
  #endif

  delay(10);

  while ( !mcps[id].begin(addr) && tries < max_tries) {
    delay(10);
    tries++;
  }

  if( tries == max_tries) {
    ok = false;
    #if DEBUG
    SERIAL.println("Sensor not found. Check wiring!");
    #endif
  } else {
    #if DEBUG
    SERIAL.println("  Found MCP9600!");
    #endif
  }

  mcps[id].setADCresolution(MCP9600_ADCRESOLUTION_14);

  #ifdef DEBUG_MCP_STARTUP
  SERIAL.print("ADC resolution set to ");
  switch (mcps[id].getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   SERIAL.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   SERIAL.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   SERIAL.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   SERIAL.print("12"); break;
  }
  SERIAL.println(" bits");
  #endif

  mcps[id].setThermocoupleType(MCP9600_TYPE_K);

  #ifdef DEBUG_MCP_STARTUP
  SERIAL.print("Thermocouple type set to ");
  switch (mcps[id].getThermocoupleType()) {
    case MCP9600_TYPE_K:  SERIAL.print("K"); break;
    case MCP9600_TYPE_J:  SERIAL.print("J"); break;
    case MCP9600_TYPE_T:  SERIAL.print("T"); break;
    case MCP9600_TYPE_N:  SERIAL.print("N"); break;
    case MCP9600_TYPE_S:  SERIAL.print("S"); break;
    case MCP9600_TYPE_E:  SERIAL.print("E"); break;
    case MCP9600_TYPE_B:  SERIAL.print("B"); break;
    case MCP9600_TYPE_R:  SERIAL.print("R"); break;
  }
  #endif


  // TODO: what is the default filter coefficient
  // https://cdn.sparkfun.com/assets/9/0/b/0/3/MCP9600_Datasheet.pdf
  //mcps[id].setFilterCoefficient(1);
  //SERIAL.print("Filter coefficient value set to: ");
  //SERIAL.println(mcps[id].getFilterCoefficient());

  #ifdef DEBUG
  SERIAL.println("started mcp");
  #endif

  mcps[id].enable(true);

  return ok;
}

/*
void safeKick() {
  if ( xSemaphoreTake( dogSem, ( TickType_t )  0) == pdTRUE ) {
   Watchdog.reset();
   xSemaphoreGive( dogSem );
  }
}*/

static void compressionThread( void * pvParameters )
{
  int pack_size = 0;
  int input_size = 0;
  unsigned long actual_read;
  int timesCompSame = 0, lastCompressSize = 0, lastGoodPackSize = 0, lastGoodInputSize = 0;
  bool acceptShort = false;
  int temp = 0;
  int bytesRead = 0;
  packet_t packet;

  // choose based on expected size of packets you are sampling
  // and the assumption that compressing wont make the data bigger
  int packetsToSample = 1;

  // packet types are powers of two so you can OR them together
  // to get bit mask of packet types to sample
  uint16_t ptypeToSample = 0;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Compression thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  myDelayMs(25000);

  while(1) {
    pack_size = 0;
    input_size = 0;
    actual_read = 0;
    packetsToSample = 4; // start with 5, var gets incremented before first use
    acceptShort = false; // have we gotten the same compression values multiple times in a row?
    lastCompressSize = 0;
    timesCompSame = 0;
    bytesRead = 0;


    // prefill with the most recent data from several sources before adding in
    // as much TC data as possible so we dont start from empty buffer each time
    if( ( temp = sample_datfile(PTYPE_GGA, 1, uc_buf) ) != ERR_SD_BUSY)
      bytesRead = temp;
    if( ( temp = sample_datfile(PTYPE_RMC, 1, &uc_buf[bytesRead] )) != ERR_SD_BUSY )
      bytesRead += temp;
    /*if( ( temp = sample_datfile(PTYPE_IMU, 1, &uc_buf[bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;
    if( ( temp = sample_datfile(PTYPE_ACC, 1, &uc_buf[bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;
    if( ( temp = sample_datfile(PTYPE_BAR, 1, &uc_buf[bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;
    if( ( temp = sample_datfile(PTYPE_SPEC, 1, &uc_buf[bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;*/

    input_size = bytesRead;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("COMP: starting compression, input_size = ");
      Serial.println(input_size);
      xSemaphoreGive( dbSem );
    }
    #endif

    // keep sampling and compressing data until compressed data
    // is larger than 338 bytes
    while(pack_size < (SBD_TX_SZ - 2) && !acceptShort ){

      lastGoodInputSize = input_size + bytesRead;

      bytesRead = 0;
      packetsToSample++;

//      #ifdef DEBUG
//      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//        Serial.print("calling sample_datfile(), asking for ");
//        Serial.print(packetsToSample);
//        Serial.println(" packets.");
//        xSemaphoreGive( dbSem );
//      }
//      #endif

      // sample the datfile, requesting packetsToSample samples of type TC data
      if( (temp = sample_datfile(PTYPE_TC,   packetsToSample, &uc_buf[input_size + bytesRead])) != ERR_SD_BUSY )
        bytesRead += temp;

      // sample some IMU data, placing it in the uc buffer after the previous reading
      if( (temp = sample_datfile(PTYPE_IMU,  packetsToSample, &uc_buf[input_size + bytesRead])) != ERR_SD_BUSY )
        bytesRead += temp;

      // sample some ACC data
      if( (temp = sample_datfile(PTYPE_ACC,  packetsToSample, &uc_buf[input_size + bytesRead])) != ERR_SD_BUSY )
        bytesRead += temp;

      // sample some spectro data
      if( (temp = sample_datfile(PTYPE_SPEC, packetsToSample, &uc_buf[input_size + bytesRead])) != ERR_SD_BUSY )
        bytesRead += temp;

//      #ifdef DEBUG
//      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//        Serial.print("Got ");
//        Serial.print(bytesRead);
//        Serial.println(" bytes back in buffer, compressing...");
//        xSemaphoreGive( dbSem );
//      }
//      #endif

//      if( bytesRead == ERR_SD_BUSY ){
//        #ifdef DEBUG
//        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//          Serial.println(" ERR SD BUSY");
//          xSemaphoreGive( dbSem );
//        }
//        #endif

//        break;
//      }

      // compress the contents of uc_buf and place them into memory starting at
      // c_buf+2 address, the number of bytes in the input is bytesRead, according to
      // the return value of the sample_datfile() function

      // keep last run in case this one exceeds
      lastGoodPackSize = pack_size;
      memcpy(prev_c_buf, c_buf, SBD_TX_SZ);

      //taskENTER_CRITICAL();
      pack_size = blz_pack(uc_buf, c_buf+2, bytesRead + input_size, workmem);
      //taskEXIT_CRITICAL();


      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print(bytesRead+input_size);
        Serial.print(" compressed (");
        Serial.print(packetsToSample);
        Serial.print(") down to ");
        Serial.print(pack_size);
        Serial.println(" bytes.");
        xSemaphoreGive( dbSem );
      }
      #endif

      if ( lastCompressSize == 0 ){
        lastCompressSize = pack_size;
      } else {
        if( pack_size == lastCompressSize ) {
          timesCompSame++;
        } else {
          lastCompressSize = pack_size;
          timesCompSame = 0;
        }
        if( timesCompSame > 5 ) acceptShort = true;
      }

      //myDelayMs(5); // wait 5ms between compression runs
    }

    // put it in reverse turry, use previous data
    if( pack_size > (SBD_TX_SZ - 2) ){
      pack_size = lastGoodPackSize;
      memcpy(c_buf, prev_c_buf, SBD_TX_SZ);
      // fill in size of compressed buffer into the first 2 bytes
      *(uint16_t*)(&c_buf[0]) = (uint16_t)(lastGoodInputSize);
    } else {
      // this was a short read, use most recent pack size and buffer
      // fill in size of compressed buffer into the first 2 bytes
      *(uint16_t*)(&c_buf[0]) = (uint16_t)(bytesRead + input_size);
    }

    // copy into packet structure
    packet.t = xTaskGetTickCount();
    packet.size = pack_size+2;
    memset(packet.data, 0, SBD_TX_SZ);
    memcpy(packet.data, c_buf, pack_size+2);

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("COMP: ready to send. Compressed ");
      Serial.print(*(uint16_t*)(&c_buf[0]));
      Serial.print(" bytes (");
      Serial.print(acceptShort ? packetsToSample : (packetsToSample-1) );
      Serial.print(" packets) down to  ");
      Serial.print(pack_size+2);
      Serial.println(" bytes.");
      xSemaphoreGive( dbSem );
    }
    #endif

    // if there is not already data in it, copy this compressed packet
    // to the global iridium buffer to be sent asap
    if ( xSemaphoreTake( irbSem, ( TickType_t ) 10 ) == pdTRUE ) {
      if( !packetReady ){
        packetReady = true;
        memcpy(gIrdBuf, c_buf, pack_size+2);
        gPacketSize = pack_size+2;
      }
      xSemaphoreGive( irbSem );
    }

    // try to write this data to the log buffer
    while( writeToLogBuf(PTYPE_PACKET, &packet, sizeof(packet_t)) > 0);

    // build a packet periodically
    myDelayMs(PACKET_BUILD_PERIOD);
  }

  vTaskDelete( NULL );
}

// thread definition
static void tcThread( void *pvParameters )
{
  tc_t data;
  float current_temps[NUM_TC_CHANNELS];
  unsigned long lastDebug = 0;
  unsigned long lastRead = 0;
  uint8_t  ledState = 0;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("TC thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {
    if( xTaskGetTickCount() - lastRead > TC_SAMPLE_PERIOD_MS ){
      lastRead = xTaskGetTickCount();
      //safeKick();

      if ( xSemaphoreTake( ledSem, ( TickType_t ) 100 ) == pdTRUE ) {
        ledState = (ledState + 1) % 4;
        ledColor( ledState );
        xSemaphoreGive( ledSem );
      }

      // assign tc temps from MCP objects to local vars
      for( int i=0; i<NUM_TC_CHANNELS; i++ ){
        current_temps[i] = readMCP(i);
        myDelayMs(1);
        //safeKick();
      }

      #ifdef DEBUG_TC
      if( xTaskGetTickCount() - lastDebug > 1000 ){
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          // print tc temps over serial
          SERIAL.print("TC temps: ");
          for( int i=0; i<NUM_TC_CHANNELS; i++ ){
            SERIAL.print(current_temps[i]); if(i<NUM_TC_CHANNELS-1) SERIAL.print(", ");
          }
          SERIAL.println();
          xSemaphoreGive( dbSem );
        }
        lastDebug = xTaskGetTickCount();
      }
      #endif

      // build data struct to send over serial
      data.t = xTaskGetTickCount() / TIME_SCALE ;
      for( int i=0; i<NUM_TC_CHANNELS; i++ ){
        data.data[i] = (int16_t)(current_temps[i] * UNIT_SCALE);
      }

      // try to write this data to the log buffer
      while( writeToLogBuf(PTYPE_TC, &data, sizeof(tc_t)) > 0);
      
    }

    taskYIELD();
  }

  vTaskDelete( NULL );
}

// helper to write a packet of data to the logfile
// ptype is a one byte id, where data holds size bytes of a struct
int writeToLogBuf(uint8_t ptype, void* data, size_t size) {
  // try to write the tc data to the SD log buffer that is available
  if ( xSemaphoreTake( wbufSem, ( TickType_t ) 10 ) == pdTRUE ) {
    if( activeLog == 1 ){
      // is this the last data we will put in before considering the
      // buffer full?
      logBuf1[logBuf1Pos++] = ptype; // set packet type byte
      memcpy(&logBuf1[logBuf1Pos], data, size);
      logBuf1Pos += size;
      if( logBuf1Pos >= LOGBUF_FULL_SIZE ){
        activeLog = 2;
        logBuf1Pos = 0;
        gb1Full = true;
      }
    } else if( activeLog == 2 ){
      // is this the last data we will put in before considering the
      // buffer full?
      logBuf2[logBuf2Pos++] = ptype; // set packet type byte
      memcpy(&logBuf2[logBuf2Pos], data, size);
      logBuf2Pos += size;
      if( logBuf2Pos >= LOGBUF_FULL_SIZE ){
        activeLog = 1;
        logBuf2Pos = 0;
        gb2Full = true;
      }
    }
    xSemaphoreGive( wbufSem );
    return 0;
  } else {
    // we did not get the semaphore and could not write to buffer
    return 1;
  }
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * sd card logging thread
*/
static void logThread( void *pvParameters )
{
  static bool ready = false;
  bool b1full = false, b2full = false;
  uint32_t written = 0;
  uint32_t filesize = 0;
  File logfile;

  #ifdef DEBUG_LOG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("sd logging thread thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  // wait indefinitely for the SD mutex
  while( xSemaphoreTake( sdSem, portMAX_DELAY ) != pdPASS );

  // INIT CARD
  while (!SD.begin(PIN_SD_CS)) {
    #if DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.println("ERROR: sd logging thread couldn't init sd card");
      xSemaphoreGive( dbSem );
    }
    #endif
    ready = false;
    myDelayMs(1000);
  }
  //else {
    #if DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.println("SD CARD INIT OK");
      xSemaphoreGive( dbSem );
    }
    #endif
    ready = true;
  //}


#if DEBUG
if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
  Serial.print("About to decide on filename, init is: ");
  Serial.println(filename);
  xSemaphoreGive( dbSem );
}
#endif

  // CREATE UNIQUE FILE NAMES (UP TO 1000)
  for( int i=0; i < 1000; i++) {
    filename[2] = '0' + (int)(i/100);
    filename[3] = '0' + (i-((int)(i/100)))/10;
    filename[4] = '0' + i%10;

    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  // write a header block to the file to start
  // the first two bytes are the block size
  logfile = SD.open(filename, FILE_WRITE);
  *(uint16_t*)(&logBuf1[0]) = (uint16_t)(LOGBUF_BLOCK_SIZE);
  logfile.write(logBuf1, LOGBUF_BLOCK_SIZE);
  logfile.close();

  xSemaphoreGive( sdSem );

  #if DEBUG_LOG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.print("Wrote header to filename: ");
    Serial.println(filename);
    xSemaphoreGive( dbSem );
  }
  #endif

  // main thread loop
  // check if a buffer is full and if it is switch the active buffer that the threads dump data to to
  // be the other buffern and start writing this one (ping pong system)
  while(1) {

    if( !gb1Full && !gb2Full ){
      myDelayMs(10);
      taskYIELD();
      continue;
    }

    // wait indefinitely for the SD mutex
    //while( xSemaphoreTake( sdSem, portMAX_DELAY ) != pdPASS );
    if ( xSemaphoreTake( sdSem, ( TickType_t ) 1000 ) == pdTRUE ) {

      // open log file for all telem (single log file configuration)
      logfile = SD.open(filename, FILE_WRITE);

      filesize = logfile.size();

      // if we could not open the file
      if( ! logfile ) {
        #if DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("ERROR: sd logging thread couldn't open logfile for writing: ");
          Serial.println(filename);
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        #if DEBUG_LOG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("SDLOG: logfile size is ");
          Serial.print(filesize);
          Serial.println(" bytes");
          xSemaphoreGive( dbSem );
        }
        #endif
      }

      // we assume we can open it and start writing the buffer
      // check if one of the buffers is full and lets write it
      if( gb1Full ){

        logfile.seek(logfile.size());

        written = logfile.write(logBuf1, LOGBUF_BLOCK_SIZE);

        // copy this data to the logBufPrev buffer for sending over radio 

        // clean the buffer so we are gauranteed to see zeros where there hasn't
        // been recetn data written to
        memset(logBuf1, 0, LOGBUF_BLOCK_SIZE);

        #if DEBUG_LOG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("SD: wrote  ");
          Serial.print(written);
          Serial.print("/");
          Serial.print(LOGBUF_BLOCK_SIZE);
          Serial.println(" bytes from logBuf1");
          xSemaphoreGive( dbSem );
        }
        #endif

        gb1Full = false;
      }

      // check the other buffer
      if( gb2Full ){

        logfile.seek(logfile.size());

        written = logfile.write(logBuf2, LOGBUF_BLOCK_SIZE);

        // clean the buffer so we are gauranteed to see zeros where there hasn't
        // been recetn data written to
        memset(logBuf2, 0, LOGBUF_BLOCK_SIZE);

        #if DEBUG_LOG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("SD: wrote  ");
          Serial.print(written);
          Serial.print("/");
          Serial.print(LOGBUF_BLOCK_SIZE);
          Serial.println(" bytes from logBuf2");
          xSemaphoreGive( dbSem );
        }
        #endif

        gb2Full = false;
      }

      // done writing to this file
      logfile.close();

      xSemaphoreGive( sdSem );
    }

    myDelayMs(10);
  }

  vTaskDelete( NULL );
}

/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
void setup() {

  // should we sleep here? or initialize all of the peripherals and then sleep..




  #if DEBUG
  SERIAL.begin(115200); // init debug serial
  #endif

  led.begin();
  led.show();

  for( int i=0; i<10; i++ ){
    ledColor(i%5);
    delay(200);
  }

  for( int i=0; i<25; i++ ){
    ledColor(i%5);
    delay(100);
  }

  for( int i=0; i<50; i++ ){
    ledColor(i%5);
    delay(50);
  }

  delay(100);
  SERIAL_GPS.begin(9600); // init gps serial
  delay(10);
  SERIAL_PI.begin(115200); // init serial to NanoPi
  delay(10);
  SERIAL_IRD.begin(115200); // Iridium radio connection
  delay(10);
  SERIAL_BSMS.begin(115200); // serial connection to bsms

  // Assign SERCOM functionality to enable 3 more UARTs
  pinPeripheral(A1, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);
  pinPeripheral(A2, PIO_SERCOM_ALT);
  pinPeripheral(A3, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);


  // reset pin for lora radio
  pinMode(PIN_RADIO_RESET, OUTPUT);
  pinMode(PIN_RADIO_SS, OUTPUT);
  pinMode(PIN_3V32_CONTROL, OUTPUT);
  pinMode(PIN_GATE_IR, OUTPUT);
  pinMode(PIN_GATE_PI, OUTPUT);
  pinMode(PIN_GATE_SPEC, OUTPUT);

  // battery voltage divider
  pinMode(PIN_VBAT, INPUT);

  // manual reset on RFM69 radio, won't do anything if unpowered (3v32_ctrl pin)
  pinMode(PIN_RADIO_RESET, OUTPUT);
  digitalWrite(PIN_RADIO_RESET, HIGH);
  delay(10);
  digitalWrite(PIN_RADIO_RESET, LOW);
  delay(10);

  Wire.begin();
  Wire.setClock(100000); // safer?

  delay(1000);

  #ifdef DEBUG
  SERIAL.println("Starting...");
  #endif

  // zero out log buffers
  memset(logBuf1, 0, LOGBUF_BLOCK_SIZE);
  memset(logBuf2, 0, LOGBUF_BLOCK_SIZE);

  ledColor(ERR_BOOT);

  // TODO: make this fool proof and reset if the init fails, up to a certain number of times

  // initialize all MCP9600 chips
  bool ok = true;
  for( int i=0; i<NUM_TC_CHANNELS; i++) {
    ok &= initMCP(i);
    delay(100);
    ledColor(ERR_2);
    //Watchdog.reset();
  }
  if (!ok) {
    #ifdef USELEDS
    ledColor();
    #endif

    #ifdef DEBUG
    SERIAL.println("failed to start all MCP devices");
    #endif
  }

  // SETUP RTOS SEMAPHORES
  // setup debug serial log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  // setup i2c port semaphore
  if ( i2c1Sem == NULL ) {
    i2c1Sem = xSemaphoreCreateMutex();  // create mutex
    if ( ( i2c1Sem ) != NULL )
      xSemaphoreGive( ( i2c1Sem ) );  // make available
  }
  // setup gps serial semaphore
  if ( gpsSerSem == NULL ) {
    gpsSerSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( gpsSerSem ) != NULL )
      xSemaphoreGive( ( gpsSerSem ) );  // make available
  }
  // setup iridium serial semaphore
  if ( irdSerSem == NULL ) {
    irdSerSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( irdSerSem ) != NULL )
      xSemaphoreGive( ( irdSerSem ) );  // make available
  }
  // setup DEPLOYMENT bool protector
  if ( depSem == NULL ) {
    depSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( depSem ) != NULL )
      xSemaphoreGive( ( depSem ) );  // make available
  }
  // setup iridium buffer protector
  if ( irbSem == NULL ) {
    irbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( irbSem ) != NULL )
      xSemaphoreGive( ( irbSem ) );  // make available
  }
  // setup iridium signal protector
  if ( sigSem == NULL ) {
    sigSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( sigSem ) != NULL )
      xSemaphoreGive( ( sigSem ) );  // make available
  }
  // setup write buffer sem
  if ( wbufSem == NULL ) {
    wbufSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( wbufSem ) != NULL )
      xSemaphoreGive( ( wbufSem ) );  // make available
  }
  // setup led sem
  if ( ledSem == NULL ) {
    ledSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( ledSem ) != NULL )
      xSemaphoreGive( ( ledSem ) );  // make available
  }
  // setup sd sem
  if ( sdSem == NULL ) {
    sdSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( sdSem ) != NULL )
      xSemaphoreGive( ( sdSem ) );  // make available
  }

  #ifdef DEBUG
  SERIAL.println("Created semaphores...");
  #endif


  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(tcThread,   "TC Measurement", 512, NULL, tskIDLE_PRIORITY, &Handle_tcTask);
  xTaskCreate(logThread,  "SD Logging", 512, NULL, tskIDLE_PRIORITY-1, &Handle_logTask);
  xTaskCreate(BSMSThread, "BSMS", 512, NULL, tskIDLE_PRIORITY, &Handle_bsmsTask);
  xTaskCreate(irdThread,  "Iridium", 512, NULL, tskIDLE_PRIORITY, &Handle_irdTask);
  xTaskCreate(prsThread,  "Pressure Measurement", 512, NULL, tskIDLE_PRIORITY, &Handle_prsTask);
  xTaskCreate(imuThread,  "IMU reading", 512, NULL, tskIDLE_PRIORITY, &Handle_imuTask);
  xTaskCreate(gpsThread,  "GPS Reception", 512, NULL, tskIDLE_PRIORITY, &Handle_gpsTask);
  xTaskCreate(compressionThread, "Data Compression", 512, NULL, tskIDLE_PRIORITY, &Handle_compTask);
  xTaskCreate(radThread, "Telem radio", 512, NULL, tskIDLE_PRIORITY, &Handle_radTask);
  
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  while(1)
  {
    SERIAL.println("Scheduler Failed! \n");
    delay(1000);
  }


}

void loop() {
  // tasks!
}
