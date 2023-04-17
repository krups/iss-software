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
#include <Adafruit_BNO08x.h>
#include <Adafruit_MCP9600.h>
#include <Adafruit_SleepyDog.h>
#include <ArduinoNmeaParser.h>
#include <SerialCommands.h>
#include <FreeRTOS_SAMD51.h>
#include <Honeywell_ABP.h>
#include <ArduinoJson.h>
#include <IridiumSBD.h>
#include <semphr.h>
#include <SD.h>
#include <SPI.h>
#include <RFM69.h>
#include <Servo.h>
#include "wiring_private.h"

#include "src/H3LIS100.h"      // high g accel driver
#include "src/delay_helpers.h" // rtos delay helpers
#include "src/config.h"        // project wide defs
#include "src/packet.h"        // packet definitions
#include "src/commands.h"      // command definitions
#include "pins.h"              // flight computer pinouts
#include "src/serial_headers.h"// Headers for serial print
#include "src/brieflz.h"
#include "src/utils.h"         // 


// Serial 2
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
#define SERIAL_PI   Serial4 // UART to Neo Pi port
#define SERIAL_IRD  Serial2 // to iridium modem
#define SERIAL_GPS  Serial3 // to GPS
#define SERIAL_BSMS Serial1 // to BSMS 

// TC to digital objects
Adafruit_MCP9600 mcps[6];

// I2C addresses on KREPE flight computer v2.0+
// Use this to assign proper channel numbering
const uint8_t MCP_ADDRS[6] = {0x60, 0x61, 0x62, 0x63, 0x64, 0x67};

// freertos task handles
TaskHandle_t Handle_packetBuildTask; // packet building task
TaskHandle_t Handle_tcTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_imuTask; // imu and acc task
TaskHandle_t Handle_prsTask; // barometric sensor task
TaskHandle_t Handle_radTask; // telem radio task handle
TaskHandle_t Handle_monitorTask; // debug running task stats over uart task
TaskHandle_t Handle_bsmsTask; // battery and spectrometer measurement (BSMS) task
TaskHandle_t Handle_piTask;  // nanoPi data sending task
TaskHandle_t Handle_dumpTask; // data dump thread

// freeRTOS semaphores
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port access semaphore
SemaphoreHandle_t gpsSerSem; // gps serial port acces
SemaphoreHandle_t irdSerSem; // iridium serial semaphore
SemaphoreHandle_t radBufSem; // radio buffer access
SemaphoreHandle_t irbSem; // iridium buffer protector
SemaphoreHandle_t sigSem; // iridium signal quality protector
SemaphoreHandle_t wbufSem; // SD buffer write semaphore
SemaphoreHandle_t ledSem; // neopixel semaphore
SemaphoreHandle_t sdSem; // sd card access
SemaphoreHandle_t piBufSem; // access to the pi send buffer

// sample periods start off as preconfigured then slowly decrease after re-entry
uint16_t imu_sample_period = IMU_SAMPLE_PERIOD_MS;
uint16_t tc_sample_period = TC_SAMPLE_PERIOD_MS;
uint16_t prs_sample_period = PRS_SAMPLE_PERIOD_MS;

// buffers for the pi thread, containing recently created data to be sent to the Pi
// each lines is a null terminated string to be filled by sprintf
#define PIBUF1 0
#define PIBUF2 1
char  ptxBuf1[PI_BUFFER_LINES][PI_LINE_SIZE];
char ptxBuf2[PI_BUFFER_LINES][PI_LINE_SIZE];
int  ptxBuf1Size = 0;
int  ptxBuf2Size = 0;
int ptxBuf1LineSize[PI_BUFFER_LINES];
int ptxBuf2LineSize[PI_BUFFER_LINES];
int  ptxActiveBuf = PIBUF1;
volatile bool   ptxBuf1Full = false;
volatile bool   ptxBuf2Full = false;

// receive and send buffers for iridium transmission
//uint8_t rbuf[RBUF_SIZE];
char sbuf[SBUF_SIZE];

// variables for the debug radio
#ifdef USE_DEBUG_RADIO
static uint8_t radioTxBuf[RADIO_TX_BUFSIZE]; // packets queued for sending (telem to groundstation)
volatile uint16_t radioTxBufSize = 0;
static uint8_t radioTxBuf2[RADIO_TX_BUFSIZE]; // copy of buffer for the radio to read from 
volatile uint16_t radioTxBufSize2 = 0;
static uint8_t radioRxBuf[RADIO_RX_BUFSIZE]; // data 
volatile uint16_t radioRxBufSize = 0;
static char radioTmpBuf[RADIO_RX_BUFSIZE]; // for moving fragments of packets
RFM69 radio(PIN_RADIO_SS, PIN_RADIO_INT, true, &SPI); // debug radio object
volatile bool radlog_tc = true, radlog_prs = false, radlog_imu = true, radlog_quat = true;
volatile bool radlog_gga = false, radlog_rmc = false, radlog_acc = false;
#endif

// debug message buffer
char printBuffer[600];

// variables for the ping pong loging buffers
static uint8_t logBuf1[LOGBUF_BLOCK_SIZE];
static uint8_t logBuf2[LOGBUF_BLOCK_SIZE];
volatile uint32_t logBuf1Pos = 0; // current write index in buffer1
volatile uint32_t logBuf2Pos = 0; // current write index in buffer2
volatile uint8_t activeLog = 1;   // which buffer should be used fo writing, 1 or 2
volatile bool gb1Full = false, gb2Full = false;

// variables relating to coordinating the sending of packets and iridium modem signal status
packet_t packet, piPacket; // private to the packet build thread
uint8_t buf[SBD_TX_SZ]; // private to the iridium sending thread
volatile bool globalDeploy = false;
volatile bool irSig = 0;
volatile bool packetReady = 0;
volatile bool internalBuildPacket = false;
volatile bool autoBuildInternal = false;
volatile bool autoBuildPi = true;
volatile uint16_t abint_period = IRIDIUM_PACKET_PERIOD;
int gPacketSize = 0;
char gIrdBuf[SBD_TX_SZ];

// for packet compression thread
uint8_t uc_buf[2*SBD_TX_SZ];  // Uncompressed buffer

// log file name
char filename[LOGFILE_NAME_LENGTH] = LOGFILE_NAME;

// include here to things are already defined
#include "src/sample_datfile.h"


// GPS parser object
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);

// low g IMU
Adafruit_BNO08x bno08x(-1);

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

// incoming spectrometer data
spec_t spec_data;


void printDirectory(SerialCommands* sender, File dir, int numTabs);

void initSD(SerialCommands* sender)
{
	// list files on SD card
	// sender->GetSerial()->print("Initializing SD card...");
  if (!SD.begin(PIN_SD_CS)) {
    #if DEBUG
    sender->GetSerial()->println("initialization failed. Things to check:");
    sender->GetSerial()->println("1. is a card inserted?");
    sender->GetSerial()->println("2. is your wiring correct?");
    sender->GetSerial()->println("3. did you change the chipSelect pin to match your shield or module?");
    sender->GetSerial()->println("Note: press reset or reopen this serial monitor after fixing your issue!");
    #endif
    while (true){
      ledColor(0);
      myDelayMs(500);
      ledColor(1);
      myDelayMs(500);
    }
  }
}

// remove files on SD card
void cmd_rm(SerialCommands* sender)
{
  initSD(sender);
  File root = SD.open("/");
  clearFiles(sender, root);
  root.close();
  sender->GetSerial()->println();
}

// list files on SD card
void cmd_ls(SerialCommands* sender)
{
  initSD(sender);
  File root = SD.open("/");
  printDirectory(sender, root, 0);
  root.close();
  sender->GetSerial()->println();
}

void cmd_dump(SerialCommands* sender)
{
	//Note: Every call to Next moves the pointer to next parameter

	char* file_str = sender->Next();
	if (file_str == NULL)
	{
		sender->GetSerial()->println("Need filename as argument");
		return;
	}
	
	initSD(sender);
  
  File dataFile = SD.open(file_str);
  if (dataFile) {
    while (dataFile.available()) {
      sender->GetSerial()->write(dataFile.read());
    }
    dataFile.close();
  } else {
    sender->GetSerial()->print("error opening ");
    sender->GetSerial()->println(file_str);
  }
  sender->GetSerial()->println();
}

void clearFiles(SerialCommands* sender, File dir) {

  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    if (entry.isDirectory()) {
      clearFiles(sender, entry);
    } else {
      if ( strcmp(entry.name(), "CONFIG.TXT") != 0){
        SD.remove(entry.name());
      }
    }
  }
}

void printDirectory(SerialCommands* sender, File dir, int numTabs) {

  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      sender->GetSerial()->print('\t');
    }
    sender->GetSerial()->print(entry.name());
    if (entry.isDirectory()) {
      sender->GetSerial()->println("/");
      printDirectory(sender, entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      sender->GetSerial()->print("\t\t");
      sender->GetSerial()->println(entry.size(), DEC);
    }
    entry.close();
  }
}

//This is the default handler, and gets called when no other command matches. 
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

char serial_command_buffer_[32];
SerialCommands serial_commands_(&SERIAL, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");
SerialCommand cmd_ls_("ls", cmd_ls);
SerialCommand cmd_dump_("dump", cmd_dump);
SerialCommand cmd_rm_("rm", cmd_rm);


static void dumpThread( void *pvParameters )
{
  // this thread starts USB serial regardless of DEBUG flag becuase that is the
  // data dump port (for now)
  SERIAL.begin(115200);
 
  myDelayMs(5000);


  serial_commands_.SetDefaultHandler(cmd_unrecognized);
	serial_commands_.AddCommand(&cmd_ls_);
	serial_commands_.AddCommand(&cmd_rm_);
	serial_commands_.AddCommand(&cmd_dump_);
  
  while (1) {
    serial_commands_.ReadSerial();
    if( digitalRead(PIN_EXT_INT) == LOW ){
      myDelayMs(2000);
      if( digitalRead(PIN_EXT_INT) == LOW )
        NVIC_SystemReset();
    }
  }

  vTaskDelete( NULL ); 
}



// print out a big line of spectrometer data, and timestamp and the number of channels
// not final by any means (timestamp first?)
void printData(uint16_t *data)
{ // Print the NUM_SPEC_CHANNELS data, then print the current time, the current color, and the number of channels.
    //SERIAL_DEBUG.print(id);
    //SERIAL_DEBUG.print(',');
    for (int i = 0; i < NUM_SPEC_CHANNELS-1; i++)
    {
        //    data_matrix(i) = data[i];
        Serial.print(data[i], DEC);
        Serial.print(',');
    }
    Serial.println(NUM_SPEC_CHANNELS-1);
    //SERIAL_DEBUG.print(result[0] + result[1]);
    //SERIAL_DEBUG.print(',');
    //SERIAL_DEBUG.print(xTaskGetTickCount());
    //SERIAL_DEBUG.print(',');
    //SERIAL_DEBUG.print(NUM_SPEC_CHANNELS);
}
static void BSMSThread(void *pvParameters)
{
  float vbat = 0.0;
  
  #ifdef DEBUG_SPEC
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 5 ) == pdTRUE ) {
      Serial.print("Sizeof(spec_t) is ");
      Serial.println(sizeof(spec_t));
      xSemaphoreGive( dbSem );
    }
    #endif


  while (1)
  {
    // measure VBAT voltage
    //vbat = analogRead(PIN_VBAT) / 1023.0 * 3.3 * 2.0;

    // vin = measurement from BSMS

    // read in spectrometer over Serial4
    while(SERIAL_BSMS.available()){

      // #ifdef DEBUG_SPEC
      // if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      //   Serial.println("bsms serial available");
      //   xSemaphoreGive( dbSem );
      // }
      // #endif

      memset((uint8_t*)(&spec_data), 0, sizeof(spec_t));
      uint8_t *p = (uint8_t*)(&spec_data);
      int bread = 0;
      uint8_t type = SERIAL_BSMS.read();

      if( type == PTYPE_SPEC ){

        #ifdef DEBUG_SPEC
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 5 ) == pdTRUE ) {
          Serial.println("ptype is right");
          xSemaphoreGive( dbSem );
        }
        #endif

        while(bread < sizeof(spec_t)){
          if(SERIAL_BSMS.available()) {
            *p = SERIAL_BSMS.read();
            p++;
            bread++;
          } 
        }

        #ifdef DEBUG_SPEC
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 10 ) == pdTRUE ) {
          Serial.println("read");
          Serial.print(bread); Serial.println(" bytes from spec");
          printData(spec_data.data);
          xSemaphoreGive( dbSem );
        }
        #endif 

        
      }
      // for(int i = 0; i < 288; i++){
      //   spec[i] = SERIAL_PI.read();
      // }
    }
	  
    // data.spec_data = spec;
    // TODO: put the spectrometer data into bins
    //myDelayMs(200);
  }

  vTaskDelete( NULL );
}

// helper to write a packet of data to the radio tx buffer
int writeToRadBuf(uint8_t ptype, void* data, size_t size) {
  bool overflow = false;

  if ( xSemaphoreTake( radBufSem, ( TickType_t ) 100 ) == pdTRUE ) {

    if( radioTxBufSize + size + 1 >= RADIO_TX_BUFSIZE ){
      overflow = true;
    } else {
      // copy data into the radio tx buffer for sending
      radioTxBuf[radioTxBufSize++] = ptype;
      memcpy(&radioTxBuf[radioTxBufSize], data, size);
      radioTxBufSize += size;
    }

    xSemaphoreGive( radBufSem );

    if( overflow ){
      #ifdef DEBUG_RADIO
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
        Serial.println("radio tx buffer overflow");
        xSemaphoreGive( dbSem );
      }
      #endif
      return 1;
    }

    return 0;
  } else {
    #ifdef DEBUG_RADIO
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      Serial.println("radio tx buffer overflow");
      xSemaphoreGive( dbSem );
    }
    #endif
    return 1;
  }

}

// helper to write a packet of data to the logfile
// ptype is a one byte id, where data holds size bytes of a struct
int writeToPtxBuf(uint8_t ptype, void* data, size_t size) {
  // get access to ptx buffers for writing
  if ( xSemaphoreTake( piBufSem, ( TickType_t ) 200 ) == pdTRUE ) {
    if( ptxActiveBuf == PIBUF1 ){

      // write packet data as plain csv to an entry in the ptxBuf1 list
      // incremennt ptxBuf1 entry size by 1
      ptxBuf1LineSize[ptxBuf1Size] = writePacketAsPlaintext((char*) ptxBuf1[ptxBuf1Size], ptype, (uint8_t*)data, size);
      ptxBuf1LineSize[ptxBuf1Size] = strlen(ptxBuf1[ptxBuf1Size]);
      ptxBuf1Size++;

      // check if ptxbuf1 is full, if so set full flag and change active buffer 
      if( ptxBuf1Size >= PI_BUFFER_LINES ){
        ptxActiveBuf = PIBUF2;
        ptxBuf1Full = true;
      }
    } else if( ptxActiveBuf == PIBUF2 ){

      // write packet data as plain csv to an entry in the ptxBuf12 list
      // incremennt ptxBuf2 entry size by 1
      ptxBuf2LineSize[ptxBuf2Size] = writePacketAsPlaintext( (char*) ptxBuf2[ptxBuf2Size++], ptype, (uint8_t*)data, size );
      ptxBuf2LineSize[ptxBuf2Size] = strlen(ptxBuf2[ptxBuf2Size]);
      ptxBuf2Size++;

      // check if ptxbuf2 is full, if so set full flag and change active buffer 
      if( ptxBuf2Size >= PI_BUFFER_LINES ){
        ptxActiveBuf = PIBUF1;
        ptxBuf2Full = true;
      }
    }
    xSemaphoreGive( piBufSem );

    // return success of writing packet to buffer
    return 0;
  } else {
    // we did not get the semaphore and could not write to buffer
    return 1;
  }
}

// communicate with NeoPi for packet creation
// PI boots up after capsule initialization, when the 5v rail is turned on 
static void piThread(void *pvParameters)
{
  // zero out tx lines and line sizes
  int i;
  for( i=0; i<PI_BUFFER_LINES; i++ ){
    memset(ptxBuf1[i], 0, PI_LINE_SIZE);
    memset(ptxBuf2[i], 0, PI_LINE_SIZE);
    ptxBuf1LineSize[i] = 0;
    ptxBuf2LineSize[i] = 0;
  }

  while (1)
  {

    // check if we have been asked to send anything to the Pi
    // if there are no lines to send, check later
    if( !ptxBuf1Full && !ptxBuf2Full ){
      //vTaskDelay(10);
      myDelayMs(10);
      continue;
    } else {
      // else there are lines to send 

      if ( xSemaphoreTake( piBufSem, ( TickType_t ) 200 ) == pdTRUE ) {
        // if buffer 1 is full, send all the lines that are queued
        if( ptxBuf1Full ){
          for( i=0; i<ptxBuf1Size; i++ ){
            SERIAL_PI.write(ptxBuf1[i], ptxBuf1LineSize[i]);
          }
          ptxBuf1Size = 0;
          ptxBuf1Full = false;
        }
        
        // if buffer 2 is full, send all of its lines
        if( ptxBuf2Full ){
          for( i=0; i<ptxBuf2Size; i++ ){
            SERIAL_PI.write(ptxBuf2[i], ptxBuf2LineSize[i]);
          }
          ptxBuf2Size = 0;
          ptxBuf2Full = false;
        }

        xSemaphoreGive( piBufSem );
      }
    }


    // TODO: check if data (a packet) has been sent to us from the Pi
    if ( SERIAL_PI.available() ){

      // how many bytes are we expecting
      // read in that many bytes.

    }
  }

  vTaskDelete( NULL );
}

// dispacth received commands over radio to various system threads
// only for ground operations, the debug radio is not for use during the actual mission
void dispatchCommand(int senderId, cmd_t command)
{ 
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    SERIAL.print("Got command from address [");
    SERIAL.print(senderId, HEX);
    SERIAL.print("], of value [");
    SERIAL.print(command.cmdid);
    SERIAL.println("]");
    xSemaphoreGive( dbSem );
  }
  #endif

  if( command.cmdid == CMDID_AB_INTERNAL_ON ){
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("CMD: setting internal autobuild to on...");
      xSemaphoreGive( dbSem );
    }
    autoBuildInternal = true; // protect with semaphore?
    #endif
  }

  if( command.cmdid == CMDID_AB_INTERNAL_OFF ){
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("CMD: setting internal autobuild to off...");
      xSemaphoreGive( dbSem );
    }
    autoBuildInternal = false; // protect with semaphore?
    #endif
  }

  if( command.cmdid == CMDID_AB_INTERNAL_PER ){
    abint_period = *((uint16_t*)(&command.data));

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting autobuild_internal sample period to: ");
      SERIAL.print(abint_period);
      SERIAL.println(" ms");
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_IR_BP ){
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("CMD: creating packet internally...");
      xSemaphoreGive( dbSem );
    }
    internalBuildPacket = true; // protect with semaphore?
    #endif
  }

  if( command.cmdid == CMDID_PI_BP ){
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("CMD: asking pi to build packet...");
      xSemaphoreGive( dbSem );
    }
    #endif

    // put a request into the buffer to go out to the pi
    writeToPtxBuf(PTYPE_PACKET_REQUEST, 0, 0);
  }

  if( command.cmdid == CMDID_LS ){
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("CMD: asking for list of sd files...");
      xSemaphoreGive( dbSem );
    }
    #endif

    // figure out how many log files on SD card
    // assume every file is a logfile
    // and that the first logfile is names LG000.DAT and there are 
    // no gaps in naming, i.e. if there are 2 files on the card they will be
    // LG000.DAT and LG001.DAT

    int numFiles = 0;
    if ( xSemaphoreTake( sdSem, ( TickType_t ) 1000 ) == pdTRUE ) {

      File root = SD.open("/");
      while( root.openNextFile() ){
        numFiles++;
      }
      root.close();
      xSemaphoreGive( sdSem );
    }

    ls_t ls;
    ls.numFiles = numFiles;

    writeToRadBuf(PTYPE_LS_T, &ls, sizeof(ls_t));

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: found ");
      SERIAL.print(numFiles);
      SERIAL.println(" on SD card!");
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_5VPWR_ON ){
    digitalWrite(PIN_GATE_IR, HIGH);

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: turning ON the PHOTOMOS ");
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_5VPWR_OFF ){
    digitalWrite(PIN_GATE_IR, LOW);

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: turning OFF the PHOTOMOS ");
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_3VPWR_ON ){
    digitalWrite(PIN_3V32_CONTROL, HIGH);

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: turning ON the external 3v3 reg ");
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_3VPWR_OFF ){
    digitalWrite(PIN_3V32_CONTROL, LOW);

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: turning OFF the external 3v3 reg ");
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGON_TC ){
    radlog_tc = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio TC log to: ");
      SERIAL.println(radlog_tc);
      xSemaphoreGive( dbSem );
    }
    #endif
  }
  

  if( command.cmdid == CMDID_SET_IMU_PER ){
    imu_sample_period = *((uint16_t*)(&command.data));

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting IMU sample period to: ");
      SERIAL.print(imu_sample_period);
      SERIAL.println(" ms");
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_SET_PRS_PER ){
    prs_sample_period = *((uint16_t*)(&command.data));

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting PRS sample period to: ");
      SERIAL.print(prs_sample_period);
      SERIAL.println(" ms");
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_SET_TC_PER ){
    tc_sample_period = *((uint16_t*)(&command.data));

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting TC sample period to: ");
      SERIAL.print(tc_sample_period);
      SERIAL.println(" ms");
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGON_TC ){
    radlog_tc = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio TC log to: ");
      SERIAL.println(radlog_tc);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGOFF_TC ){
    radlog_tc = false;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio TC log to: ");
      SERIAL.println(radlog_tc);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGOFF_PRS ){
    radlog_prs = false;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio PRS log to: ");
      SERIAL.println(radlog_prs);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGON_PRS ){
    radlog_prs = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio PRS log to: ");
      SERIAL.println(radlog_prs);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGOFF_QUAT ){
    radlog_quat = false;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio QUAT log to: ");
      SERIAL.println(radlog_quat);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGON_QUAT ){
    radlog_quat = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio QUAT log to: ");
      SERIAL.println(radlog_quat);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGOFF_IMU ){
    radlog_imu = false;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio QUAT log to: ");
      SERIAL.println(radlog_imu);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGON_IMU ){
    radlog_imu = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio QUAT log to: ");
      SERIAL.println(radlog_imu);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGON_GGA ){
    radlog_gga = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio GGA log to: ");
      SERIAL.println(radlog_gga);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGOFF_GGA ){
    radlog_gga = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio GGA log to: ");
      SERIAL.println(radlog_gga);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGON_RMC ){
    radlog_rmc = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio RMC log to: ");
      SERIAL.println(radlog_rmc);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGOFF_RMC ){
    radlog_rmc = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio RMC log to: ");
      SERIAL.println(radlog_rmc);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGOFF_ACC ){
    radlog_acc = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio ACC log to: ");
      SERIAL.println(radlog_acc);
      xSemaphoreGive( dbSem );
    }
    #endif
  }

  if( command.cmdid == CMDID_RADLOGON_ACC ){
    radlog_acc = true;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("CMD: setting radio ACC log to: ");
      SERIAL.println(radlog_acc);
      xSemaphoreGive( dbSem );
    }
    #endif
  }
}


// radio thread
// in charge of the RFM69 debug radio 
// sending packets out as telemetry and receiving commands and 
// dispatching them to other threads
// see node demo sketch for reference: https://github.com/LowPowerLab/RFM69/blob/master/Examples/Node/Node.ino
static void radThread(void *pvParameters)
{
  cmd_t inCmd;
  int fromNode = -1;
  bool sendOkay = false;

  myDelayMs(1000);

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    SERIAL.println("RAD: Powering on Radio!");
    xSemaphoreGive( dbSem );
  }
  #endif

  myDelayMs(1000);

  // manual reset on RFM69 radio, won't do anything if unpowered (3v32_ctrl pin)
  digitalWrite(PIN_RADIO_RESET, HIGH);
  myDelayMs(10);
  digitalWrite(PIN_RADIO_RESET, LOW);
  myDelayMs(100);

  uint8_t temp = 0;

  if ( xSemaphoreTake( sdSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    radio.initialize(FREQUENCY, NODE_ADDRESS, NETWORK_ID);
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
    radio.encrypt(ENCRYPTKEY);
    temp = radio.readTemperature();
    xSemaphoreGive( sdSem );
  }

  #ifdef DEBUG_RADIO
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    SERIAL.println("RAD: Initialized Radio!");
    SERIAL.print("RAD: temperature is: ");
    SERIAL.println(temp, DEC);
    xSemaphoreGive( dbSem );
  }
  #endif

  int rxBufPos = 0;
  int txBufPos = 0;

  while( 1 ){

    // #ifdef DEBUG_TICK
    // if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    //   SERIAL.println("TICK RADIO");
    //   xSemaphoreGive( dbSem );
    // }
    // #endif

    // first get access to SPI bus
    if ( xSemaphoreTake( sdSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      
      // check for any received packets, but only if they will fit in our RX buffer
      if (radio.receiveDone())
      {
        fromNode = radio.SENDERID;

        if( (radioRxBufSize + radio.DATALEN) < RADIO_RX_BUFSIZE) {
          memcpy(radioRxBuf, radio.DATA, radio.DATALEN);
          radioRxBufSize += radio.DATALEN;

          #ifdef DEBUG
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

    // interpret received data, if any
    // since max reception size is 60 bytes and we parse it right after we 
    // receive it, precautions would have to be taken to received payloads
    // larger than 60 bytes
    // this approach assumes all commands are less than 60 bytes and also 
    // that the first byte of the rx buffer is always a packet ID byte
    if( radioRxBufSize > 0 ){
      
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
        Serial.print("RADIO: got ");
        Serial.print(radioRxBufSize);
        Serial.println("bytes");
        xSemaphoreGive( dbSem );
      }
      #endif

      // parse the buffer
      // control capsule functionality based on command packets
      // or other functionality...
      bool goon = true;
      int radrxbufidx = 0;
      while( goon ){

        // receiving a command
        if( radioRxBuf[radrxbufidx] == PTYPE_CMD ){

          #ifdef DEBUG
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
            Serial.println("RADIO: got CMD packet");
            xSemaphoreGive( dbSem );
          }
          #endif

          if( sizeof(cmd_t) > (radioRxBufSize - radrxbufidx + 1)){
            goon = false;

            #ifdef DEBUG
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
              Serial.println("RADIO: setting goon false");
              xSemaphoreGive( dbSem );
            }
            #endif

            break;
          } else {
            memcpy(&inCmd, &radioRxBuf[radrxbufidx + 1], sizeof(cmd_t));
            radrxbufidx += sizeof(cmd_t) + 1;

            #ifdef DEBUG
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
              Serial.print("RADIO: dispatching packet");
              xSemaphoreGive( dbSem );
            }
            #endif

            dispatchCommand(fromNode, inCmd);
          }
        } else {
          radrxbufidx++;
          #ifdef DEBUG
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
            Serial.print("RADIO:  received unknown command byte: ");
            Serial.println(radioRxBuf[radrxbufidx], HEX);
            xSemaphoreGive( dbSem );
          }
          #endif
        }

        if( radrxbufidx == radioRxBufSize ){
          goon = false;
        }
      }

      // if there was a fragment of a packet left in the buffer
      // move the remaining bytes to the beginning of the buffer and
      // set the buffer size accordingly
      // ignore if a fragment is all thats left in the buffer
      if( radrxbufidx < radioRxBufSize && radrxbufidx > 0){

        // copy remaining bytes to temp buf then back to begining of rx buffer
        memcpy(radioTmpBuf, &radioRxBuf[radrxbufidx], radioRxBufSize - radrxbufidx);
        memcpy(radioRxBuf, radioTmpBuf, radioRxBufSize - radrxbufidx);
        radioRxBufSize = radioRxBufSize - radrxbufidx;

        #ifdef DEBUG_RADIO
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
          Serial.println("RADIO:  copyied fragment");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if( radrxbufidx == radioRxBufSize && radrxbufidx > 0 ){
        // received data was complete packets, no fragments
        radioRxBufSize = 0;
      }

      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
        Serial.println("RADIO: end of got data");
        xSemaphoreGive( dbSem );
      }
      #endif
    }

    // now check if we have any packets in the TX buffer.
    // copy to our local buffer
    if( xSemaphoreTake( radBufSem, (TickType_t) 200 ) == pdTRUE ){
      if( radioTxBufSize > 0 ){
        radioTxBufSize2 = radioTxBufSize;
        memcpy(radioTxBuf2, radioTxBuf, radioTxBufSize);
        radioTxBufSize = 0;
      }
      xSemaphoreGive( radBufSem );
    }
    // if there are packets of data in the Tx buffer 
    // the get the sd semaphore and try to send all that are in there
    // sending with retry can take time, so put the spi access semaphore inside the send loop
    txBufPos = 0;
    // need to get access to send buffer and SD buffer
    if( radioTxBufSize2 > 0 ){
      
      #ifdef DEBUG_RADIO
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.print("RAD: have data to send!");
        xSemaphoreGive( dbSem );
      }
      #endif

      // we can only send 60 bytes at a time, so look through the buffer and send as much as possible 
      // at once until the buffer is empty
      sendOkay = true;
      while ((radioTxBufSize2 - txBufPos) > 0)
      {

        #ifdef DEBUG_RADIO
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.print("RAD: have ");
          SERIAL.print(radioTxBufSize2);
          SERIAL.print(" bytes to send...");
          xSemaphoreGive( dbSem );
        }
        #endif

        // first get access to SPI bus
        if ( xSemaphoreTake( sdSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          // if there is more than 60 bytes in the send buffer, send 60 of it over and over
          if( (radioTxBufSize2 - txBufPos) >= 60 ){
            //radio.send((uint16_t)NODE_ADDRESS_STATION, &radioTxBuf2[txBufPos], 60, true);
            //txBufPos += 60;
            if (radio.sendWithRetry((uint16_t)NODE_ADDRESS_STATION, &radioTxBuf2[txBufPos], 60)){
              // success
              sendOkay &= true;
              txBufPos += 60;
            } else {
              // sending failed 
              sendOkay &= false;
              txBufPos += 60;
            }
          } else { // else if there is 60 or less bytes in the send buffer, just send what is there
            //radio.send((uint16_t)NODE_ADDRESS_STATION, &radioTxBuf2[txBufPos], (uint8_t)(radioTxBufSize2 - txBufPos), true );
            //txBufPos += (radioTxBufSize2 - txBufPos);
            if (radio.sendWithRetry((uint16_t)NODE_ADDRESS_STATION, &radioTxBuf2[txBufPos], (uint8_t)(radioTxBufSize2 - txBufPos) )){
              // send success
              sendOkay &= true;
              txBufPos += (radioTxBufSize2 - txBufPos);
            } else {
              // sending failed
              sendOkay &= false;    
              txBufPos += (radioTxBufSize2 - txBufPos);
            }
          }
          xSemaphoreGive( sdSem );
        } // end access to the SPI bus
      }

      if( sendOkay ){
        #ifdef DEBUG_RADIO
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          Serial.println(" debug radio send ok");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        #ifdef DEBUG_RADIO
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          Serial.print(" debug radio send fail"); 
          xSemaphoreGive( dbSem );
        }
        #endif
      }

      radioTxBufSize2 = 0;
    } // end if radioTxBufSize > 0

    myDelayMs(50);
  }

  vTaskDelete( NULL );
}

void ledColor(int type) {
  switch (type) {
  case ERR_BOOT:
    led.setPixelColor(0, led.Color(5, 0, 0));
    break;
  case ERR_2:
    led.setPixelColor(0, led.Color(5, 0, 5));
    break;
  case ERR_3:
    led.setPixelColor(0, led.Color(5, 5, 0));
    break;
  case ERR_4:
    led.setPixelColor(0, led.Color(0, 5, 5));
    break;
  default:
    led.setPixelColor(0, led.Color(5, 5, 5));
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
  writeToLogBuf(PTYPE_RMC, &data, sizeof(rmc_t));

  // try to write to pi output buffer
  writeToPtxBuf(PTYPE_RMC, &data, sizeof(rmc_t));

  // try to write to radio debug buffer
  //while( writeToRadBuf(PTYPE_RMC, &data, sizeof(rmc_t)) > 0);
  if( radlog_rmc ) writeToRadBuf(PTYPE_RMC, &data, sizeof(rmc_t));
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
      writeGga(gga, SERIAL);
      xSemaphoreGive( dbSem );
    }
  //}
  #endif

  data.t = (uint16_t) xTaskGetTickCount() / TIME_SCALE;
  data.time[0] = (uint16_t)gga.time_utc.hour;
  data.time[1] = (uint16_t)gga.time_utc.minute;
  data.time[2] = (uint16_t)gga.time_utc.second;
  data.time[3] = (uint16_t)gga.time_utc.microsecond;
  data.lat = gga.latitude;
  data.lon = gga.longitude;
  data.hdop = gga.hdop;
  data.alt = gga.altitude;

  // try to write the gps gga data to the SD log buffer
  writeToLogBuf(PTYPE_GGA, &data, sizeof(gga_t));

  // try to write to pi output buffer
  writeToPtxBuf(PTYPE_GGA, &data, sizeof(gga_t));

  // try to write to radio debug buffer
  //while( writeToRadBuf(PTYPE_GGA, &data, sizeof(gga_t)) > 0);
  if( radlog_gga ) writeToRadBuf(PTYPE_GGA, &data, sizeof(gga_t));
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
    // semaphore should not be needed since this is the only thread 
    // accessing the hardware
    //if ( xSemaphoreTake( gpsSerSem, ( TickType_t ) 100 ) == pdTRUE ) {
      while (SERIAL_GPS.available()) {
        //SERIAL.write((char)SERIAL_GPS.read());
        parser.encode((char)SERIAL_GPS.read());
      }
    //  xSemaphoreGive( gpsSerSem );
    //}

      //vTaskDelay(10);
      myDelayMs(10);
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
  bool bno_ok = false, highg_ok = false, 
       bno_accel_init = false, bno_gyro_init = false, bno_quat_init = false;
  sensors_event_t h3event; // struct to hold high g accel data
  sh2_SensorValue_t bnoValue; // struct to hold low g imu data
  acc_t accData;
  imu_t imuData;
  quat_t quatData;

  // unsigned long sample_period_ms = 20;
  unsigned long last_sample_time = 0, lastlast_sample_time = 0;

  accData.t = 0;
  accData.data[0] = 0;
  accData.data[1] = 0;
  accData.data[2] = 0;

  imuData.t = 0;
  imuData.data[0] = 0;
  imuData.data[1] = 0;
  imuData.data[2] = 0;
  imuData.data[3] = 0;
  imuData.data[4] = 0;
  imuData.data[5] = 0;

  quatData.t = 0;
  quatData.data[0] = 1.0; // r component
  quatData.data[1] = 0.0; // i ( all floats )
  quatData.data[2] = 0.0; // j 
  quatData.data[3] = 0.0; // k


  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("IMU thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
    // init BNO055 IMU
    // Try to initialize!
    if (bno08x.begin_I2C()) {
      bno_ok = true;
    }

    //bno08x.enableReport(SH2_ACCELEROMETER);
    //bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
    if( bno08x.enableReport(SH2_ARVR_STABILIZED_RV)) {
      bno_quat_init = true;
    }
    // if (bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    //   bno_quat_init = true;
    // }
    if (bno08x.enableReport(SH2_ACCELEROMETER)) {
      bno_accel_init = true;
    }
    if (bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
      bno_gyro_init = true;
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
    Serial.print(", BNO accel init: ");
    Serial.print(bno_accel_init);
    Serial.print(", BNO gyro init: ");
    Serial.print(bno_gyro_init);
    Serial.print(", BNO quat init: ");
    Serial.print(bno_quat_init);
    Serial.print(", init H3LIS: ");
    Serial.println(highg_ok);
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {
    myDelayMs(20);

    // #ifdef DEBUG_TICK
    // if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    //   SERIAL.println("TICK IMU");
    //   xSemaphoreGive( dbSem );
    // }
    // #endif

    // try to acquire lock on i2c bus and take measurements
    // from the IMU and high g accelerometer
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 10 ) == pdTRUE ) {
      
      // if (bno08x.wasReset()) {
      //   Serial.print("sensor was reset ");
      //   setReports();
      // }

      // get high g accel measurements
      highg.getEvent(&h3event);


      // if (!bno08x.getSensorEvent(&sensorValue)) {
      //   continue;
      // }

      bno08x.getSensorEvent(&bnoValue);

      xSemaphoreGive( i2c1Sem );
    }

    switch (bnoValue.sensorId) {
      case SH2_ACCELEROMETER:
        //ledColor(ERR_2);
        // in units of gravity
        imuData.data[0] = (int16_t) (bnoValue.un.accelerometer.x * UNIT_SCALE);
        imuData.data[1] = (int16_t) (bnoValue.un.accelerometer.y * UNIT_SCALE);
        imuData.data[2] = (int16_t) (bnoValue.un.accelerometer.z * UNIT_SCALE);
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        //ledColor(ERR_3);
        // in radians, convert to degreess before storing
        imuData.data[3] = (int16_t) (bnoValue.un.gyroscope.x * RAD2DEG * UNIT_SCALE);
        imuData.data[4] = (int16_t) (bnoValue.un.gyroscope.y * RAD2DEG * UNIT_SCALE);
        imuData.data[5] = (int16_t) (bnoValue.un.gyroscope.z * RAD2DEG * UNIT_SCALE);
        break;
      case SH2_GEOMAGNETIC_ROTATION_VECTOR:
        // store quat data
        quatData.data[0] = bnoValue.un.gameRotationVector.real;
        quatData.data[1] = bnoValue.un.gameRotationVector.i;
        quatData.data[2] = bnoValue.un.gameRotationVector.j;
        quatData.data[3] = bnoValue.un.gameRotationVector.k; 
        break;
      case SH2_ARVR_STABILIZED_RV:
      // store quat data
        quatData.data[0] = bnoValue.un.arvrStabilizedRV.real;
        quatData.data[1] = bnoValue.un.arvrStabilizedRV.i;
        quatData.data[2] = bnoValue.un.arvrStabilizedRV.j;
        quatData.data[3] = bnoValue.un.arvrStabilizedRV.k; 
        break;
    }

    last_sample_time = xTaskGetTickCount();

    // copy high g acc data (h3lis100) into logging structs
    accData.t =  last_sample_time / TIME_SCALE;
    imuData.t =  last_sample_time / TIME_SCALE;
    quatData.t = last_sample_time / TIME_SCALE;

    accData.data[0] = h3event.acceleration.x * UNIT_SCALE;
    accData.data[1] = h3event.acceleration.y * UNIT_SCALE;
    accData.data[2] = h3event.acceleration.z * UNIT_SCALE;

    // quat data not scaled

    // report sensor init status
    imuData.ok = 0;
    if( bno_ok )   imuData.ok |= 0x0F;
    if( highg_ok ) imuData.ok |= 0xF0;


    if( last_sample_time - lastlast_sample_time > (imu_sample_period) ){
      lastlast_sample_time = last_sample_time;

      // #ifdef DEBUG
      // if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      //   Serial.println("IMU: writing to SD log");
      //   xSemaphoreGive( dbSem );
      // }
      // #endif

      // try to write the imu and acc structs to the SD log buffer
      writeToLogBuf(PTYPE_IMU, &imuData, sizeof(imu_t));
      writeToLogBuf(PTYPE_ACC, &accData, sizeof(acc_t));
      writeToLogBuf(PTYPE_QUAT, &quatData, sizeof(quat_t));

      // #ifdef DEBUG
      // if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      //   Serial.println("IMU: writing to PTX buffer");
      //   xSemaphoreGive( dbSem );
      // }
      // #endif

      // try to write these data to the pi buffer lines to be sent out over serial
      writeToPtxBuf( PTYPE_IMU, &imuData, sizeof(imu_t));
      writeToPtxBuf( PTYPE_ACC, &accData, sizeof(acc_t));
      //writeToPtxBuf(PTYPE_QUAT, &quatData, sizeof(quat_t));

      // #ifdef DEBUG
      // if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      //   Serial.println("IMU: writing to RADIO buffer");
      //   xSemaphoreGive( dbSem );
      // }
      // #endif

      // try to write to radio debug buffer
      //while( writeToRadBuf(PTYPE_IMU, &imuData, sizeof(imu_t)) > 0);
      //while( writeToRadBuf(PTYPE_ACC, &accData, sizeof(acc_t)) > 0);
      if( radlog_imu ) writeToRadBuf(PTYPE_IMU, &imuData, sizeof(imu_t));
      if( radlog_acc ) writeToRadBuf(PTYPE_ACC, &accData, sizeof(acc_t));
      if( radlog_quat ) writeToRadBuf(PTYPE_QUAT, &quatData, sizeof(quat_t));
    }

    #ifdef DEBUG_IMU
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("BNO088 aX: ");
      Serial.print((float)(imuData.data[0] / (float)UNIT_SCALE), 4);
      Serial.print("\taY: ");
      Serial.print(imuData.data[1] / UNIT_SCALE, 4);
      Serial.print("\taZ: ");
      Serial.println(imuData.data[2] / UNIT_SCALE, 4);

      Serial.print("BNO088 gX: ");
      Serial.print(bnoValue.un.rawGyroscope.x, 4);
      Serial.print("\tgY: ");
      Serial.print(imuData.data[4] / UNIT_SCALE, 4);
      Serial.print("\tgZ: ");
      Serial.println(imuData.data[5] / UNIT_SCALE, 4);

      Serial.print("H3LIS100 aX: ");
      Serial.print(accData.data[0] / UNIT_SCALE, 4);
      Serial.print("\taY: ");
      Serial.print(accData.data[1] / UNIT_SCALE, 4);
      Serial.print("\taZ: ");
      Serial.println(accData.data[2] / UNIT_SCALE, 4);

      xSemaphoreGive( dbSem );
    }
    #endif

  }

  vTaskDelete( NULL );
}


/**********************************************************************************
 * flushed air data sensings pressure monitoring thread
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
    Serial.println("Pressure thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {

    if( xTaskGetTickCount() - lastRead > prs_sample_period ) {  
      lastRead = xTaskGetTickCount(); 

      if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
        select_i2cmux_channel(MUXCHAN_PS1);
        myDelayUs(100);
        ps1.update();
        select_i2cmux_channel(MUXCHAN_PS2);
        myDelayUs(100);
        ps2.update();
        select_i2cmux_channel(MUXCHAN_PS3);
        myDelayUs(100);
        ps3.update();
        select_i2cmux_channel(MUXCHAN_PS4);
        myDelayUs(100);
        ps4.update();
        select_i2cmux_channel(MUXCHAN_PS5);
        myDelayUs(100);
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
      writeToLogBuf(PTYPE_PRS, &data, sizeof(prs_t));

      // try to write to pi output buffer
      writeToPtxBuf(PTYPE_PRS, &data, sizeof(prs_t));

      // try to write to the radio send buffer
      //while( writeToRadBuf(PTYPE_PRS, &data, sizeof(prs_t)) > 0);
      if( radlog_prs ) writeToRadBuf(PTYPE_PRS, &data, sizeof(prs_t));

      //myDelayMs(1000);

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
    if( xTaskGetTickCount() - lastPacketSend > abint_period &&
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

// thread to build packets very simply, no compression
//  uniform samples of different data
static void packetBuildThread( void * pvParameters )
{
  int pack_size = 0;
  int input_size = 0;
  unsigned long actual_read;
  int temp = 0;
  int bytesRead = 0;
  // choose based on expected size of packets you are sampling
  // and the assumption that compressing wont make the data bigger
  int packetsToSample = 1;
  unsigned long lastPiAutoBuild = 0;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Packet build thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  myDelayMs(5000);

  while(1) {

     
    // periodically send the magic byte to ask for a binary packet back to send, 
    // then place that in the iridium buffer. 
    if( autoBuildPi && xTaskGetTickCount() - lastPiAutoBuild > PACKET_BUILD_PERIOD ){
      lastPiAutoBuild = xTaskGetTickCount();

      // put a request into the buffer to go out to the pi
      writeToPtxBuf(PTYPE_PACKET_REQUEST, 0, 0);

      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.print("sending packet build request for autobuild");
        xSemaphoreGive( dbSem );
      }
      #endif
    }    

    if( !internalBuildPacket && autoBuildInternal == false ){
      myDelayMs(1000);
      continue;
    }

    input_size = 0;
    actual_read = 0;
    packetsToSample = 30; // start with 5, var gets incremented before first use
    bytesRead = 0;


    // prefill with the most recent data from several sources before adding in
    // as much TC data as possible so we dont start from empty buffer each time
    if( ( temp = sample_datfile(PTYPE_GGA, 1, uc_buf) ) != ERR_SD_BUSY)
      bytesRead = temp;
    myDelayMs(10);
    if( ( temp = sample_datfile(PTYPE_RMC, 1, &uc_buf[bytesRead] )) != ERR_SD_BUSY )
      bytesRead += temp;
    myDelayMs(10);
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
      Serial.print("COMP: 2 gps packets are size: ");
      Serial.println(input_size);
      xSemaphoreGive( dbSem );
    }
    #endif

    // not using compression, just fill with a predetermined structure

    // sample the datfile, requesting packetsToSample samples of type TC data
    if( (temp = sample_datfile(PTYPE_TC,   packetsToSample, &uc_buf[input_size + bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;
    
    myDelayMs(10);
    // sample some IMU data, placing it in the uc buffer after the previous reading
    if( (temp = sample_datfile(PTYPE_IMU,  packetsToSample, &uc_buf[input_size + bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;
   
    myDelayMs(10);
    // sample some ACC data
    if( (temp = sample_datfile(PTYPE_ACC,  packetsToSample, &uc_buf[input_size + bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;

    myDelayMs(10);
    // sample some spectro data
    if( (temp = sample_datfile(PTYPE_SPEC, packetsToSample, &uc_buf[input_size + bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;

    myDelayMs(10);
    // sample some spectro data
    if( (temp = sample_datfile(PTYPE_PRS, packetsToSample, &uc_buf[input_size + bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;

    // overall size in bytes of our data packet, uncompressed
    input_size = bytesRead;

    // copy into packet structure
    // just copy the uncompressed data into the packet structure
    // in practice, we see very little compression with the
    // limited RAM on microcontrollers, combined with the fact that the data is already binary 
    // with little repetition in the data
    packet.t = xTaskGetTickCount();
    packet.size = input_size;
    memset(packet.data, 0, SBD_TX_SZ);
    memcpy(packet.data, uc_buf, input_size);

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("COMP: made an uncompressed packet size:  ");
      Serial.print(input_size);
      Serial.println(" bytes.");
      xSemaphoreGive( dbSem );
    }
    #endif

    // if there is not already data in it, copy this packet
    // to the global iridium buffer to be sent asap
    if ( xSemaphoreTake( irbSem, ( TickType_t ) 10 ) == pdTRUE ) {
      if( !packetReady ){
        packetReady = true;
        memcpy(gIrdBuf, uc_buf, input_size);
        gPacketSize = input_size;
      }
      xSemaphoreGive( irbSem );
    }

    // try to write this data to the log buffer
    // TODO: don't try forever
    writeToLogBuf(PTYPE_PACKET, &packet, sizeof(packet_t));

    internalBuildPacket = false;

    
    // build a packet periodically
    if( autoBuildInternal ){
      myDelayMs(PACKET_BUILD_PERIOD);
    }
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
    if( xTaskGetTickCount() - lastRead > tc_sample_period ){
      lastRead = xTaskGetTickCount();
      //safeKick();

    #ifdef DEBUG_TICK
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("TICK TC");
      xSemaphoreGive( dbSem );
    }
    #endif

      if ( xSemaphoreTake( ledSem, ( TickType_t ) 100 ) == pdTRUE ) {
        ledState = (ledState + 1) % 4;
        ledColor( ledState );
        xSemaphoreGive( ledSem );
      }

      // assign tc temps from MCP objects to local vars
      for( int i=0; i<NUM_TC_CHANNELS; i++ ){
        current_temps[i] = readMCP(i);
        myDelayUs(100);
        //myDelayMs(1);
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
      writeToLogBuf(PTYPE_TC, &data, sizeof(tc_t));

      // try to write this data to the pi buff 
      writeToPtxBuf(PTYPE_TC, &data, sizeof(tc_t));

      // try to put in the radio send buffer
      //while( writeToRadBuf(PTYPE_TC, &data, sizeof(tc_t)) > 0);
      if( radlog_tc ) writeToRadBuf(PTYPE_TC, &data, sizeof(tc_t));
      
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

      #if DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print(filename);
        Serial.print(" exists on card! ");
        xSemaphoreGive( dbSem );
      }
      #endif
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

    #ifdef DEBUG_TICK
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("TICK LOG");
      xSemaphoreGive( dbSem );
    }
    #endif

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

/*
 * sleep ISR for pre-activation sleep
*/
void sleepISR(){

}

/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
void setup() {

  // should we sleep here? or initialize all of the peripherals and then sleep..

  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(25, INPUT);
  pinMode(24, INPUT);
  pinMode(23, INPUT);
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(4, INPUT);

  pinMode(13, INPUT);
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  pinMode(10, INPUT);
  pinMode(9, INPUT);
  pinMode(6, INPUT);
  pinMode(5, INPUT);
  pinMode(22, INPUT);
  pinMode(21, INPUT);

  led.begin();
  led.setPixelColor(0, led.Color(0, 0, 0));
  led.show();

  pinMode(PIN_EXT_INT, INPUT_PULLUP);
  delay(10);
  attachInterrupt(PIN_EXT_INT, sleepISR, RISING);

  bool woke = digitalRead(PIN_EXT_INT);

  if( woke ){

    ledColor(0);

    // put the radio in reset state
    pinMode(PIN_RADIO_RESET, OUTPUT);
    pinMode(PIN_RADIO_SS, OUTPUT);
    digitalWrite(PIN_RADIO_RESET, HIGH);
    digitalWrite(PIN_RADIO_SS, HIGH);

    // enable 3.3v to the SD card
    pinMode(PIN_3V32_CONTROL, OUTPUT);
    digitalWrite(PIN_3V32_CONTROL, HIGH);
    pinMode(PIN_GATE_IR, OUTPUT);
    digitalWrite(PIN_GATE_IR, HIGH);

    delay(2000);

    xTaskCreate(dumpThread, "Data dump", 1024, NULL, tskIDLE_PRIORITY, &Handle_dumpTask);
    //xTaskCreate(radThread, "Telem radio", 1024, NULL, tskIDLE_PRIORITY, &Handle_radTask);
    
    // Start the RTOS, this function will never return and will schedule the tasks.
    vTaskStartScheduler();

    // error scheduler failed to start
    while(1)
    {
      SERIAL.println("Scheduler Failed! \n");
      delay(1000);
    }

  } else {
    //USB->DEVICE.CTRLA.bit.ENABLE = 0;                   // Shutdown the USB peripheral
    //while(USB->DEVICE.SYNCBUSY.bit.ENABLE);             // Wait for synchronization
    
    PM->SLEEPCFG.bit.SLEEPMODE = 0x4;             // Set up SAMD51 to enter low power STANDBY mode
    while(PM->SLEEPCFG.bit.SLEEPMODE != 0x4);
    
    while ( !woke ) {
        // go to sleep for some period of time
        // about 16 seconds is max before watchdog in testing
        int sleepMS = Watchdog.sleep();
        // now asleep
        woke = digitalRead(PIN_EXT_INT);
    }
  }

  


  #if DEBUG
  SERIAL.begin(115200); // init debug serial
  #endif


  for( int i=0; i<10; i++ ){
    ledColor(i%5);
    delay(200);
  }

  for( int i=0; i<25; i++ ){
    ledColor(i%5);
    delay(100);
  }


  delay(100);
  SERIAL_GPS.begin(38400); // init gps serial
  delay(10);
  SERIAL_PI.begin(115200); // init serial to NanoPi
  delay(10);
  SERIAL_IRD.begin(9600); // Iridium radio connection
  delay(10);
  SERIAL_BSMS.begin(115200); // serial connection to bsms

  // Assign SERCOM functionality to enable 3 more UARTs
  pinPeripheral(A1, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);
  pinPeripheral(A2, PIO_SERCOM_ALT);
  pinPeripheral(A3, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);


  // reset pin for RFM69 radio
  pinMode(PIN_RADIO_RESET, OUTPUT);
  pinMode(PIN_RADIO_SS, OUTPUT);
  pinMode(PIN_3V32_CONTROL, OUTPUT);
  pinMode(PIN_GATE_IR, OUTPUT);

  // keep radio in reset state until sd thread is started 
  digitalWrite(PIN_RADIO_RESET, HIGH);
  digitalWrite(PIN_RADIO_SS, HIGH);

  // turn on 5v rail through photoMOS and enable the 3vr3 reg that
  // powers the rs232 level shifter
  digitalWrite( PIN_GATE_IR, HIGH );
  digitalWrite( PIN_3V32_CONTROL, HIGH);


  // battery voltage divider
  pinMode(PIN_VBAT, INPUT);

  Wire.begin();
  Wire.setClock(100000); // mcp9600 spec is 1Mhz

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
  // setup radio tx buffer  protector
  if ( radBufSem == NULL ) {
    radBufSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( radBufSem ) != NULL )
      xSemaphoreGive( ( radBufSem ) );  // make available
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
  // setup sd sem
  if ( piBufSem == NULL ) {
    piBufSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( piBufSem ) != NULL )
      xSemaphoreGive( ( piBufSem ) );  // make available
  }

  #ifdef DEBUG
  SERIAL.println("Created semaphores...");
  #endif


  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(tcThread,   "TC Measurement", 512, NULL, tskIDLE_PRIORITY, &Handle_tcTask);
  xTaskCreate(BSMSThread, "BSMS", 512, NULL, tskIDLE_PRIORITY, &Handle_bsmsTask);
  xTaskCreate(irdThread,  "Iridium", 512, NULL, tskIDLE_PRIORITY, &Handle_irdTask);
  xTaskCreate(prsThread,  "Pressure Measurement", 1024, NULL, tskIDLE_PRIORITY, &Handle_prsTask);
  xTaskCreate(imuThread,  "IMU reading", 512, NULL, tskIDLE_PRIORITY, &Handle_imuTask);
  xTaskCreate(gpsThread,  "GPS Reception", 512, NULL, tskIDLE_PRIORITY, &Handle_gpsTask);
  xTaskCreate(packetBuildThread, "Default packet building", 512, NULL, tskIDLE_PRIORITY + 1, &Handle_packetBuildTask);
  xTaskCreate(piThread,  "NanoPi Packet Building", 512, NULL, tskIDLE_PRIORITY, &Handle_piTask);
  xTaskCreate(radThread, "Telem radio", 1024, NULL, tskIDLE_PRIORITY, &Handle_radTask);
  xTaskCreate(logThread,  "SD Logging", 1024, NULL, tskIDLE_PRIORITY+3, &Handle_logTask);

  
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
