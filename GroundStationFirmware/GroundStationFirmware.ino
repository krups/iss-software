// Groundstation firmware for KREPE-2 ISS mission
// Matt Ruffner 2022
// This software runs on the gateway node,receiving in flight telemetry from the capsule
// and sending over serial in JSON 

// https://medium.com/@benjaminmbrown/real-time-data-visualization-with-d3-crossfilter-and-websockets-in-python-tutorial-dba5255e7f0e

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_NeoPixel.h>
#include <FreeRTOS_SAMD21.h>
#include <SerialCommands.h>
#include <ArduinoJson.h>
#include <semphr.h>
#include <RFM69.h>
#include <SD.h>

#define DEBUG 1
#ifdef DEBUG
  #define DEBUG_RADIO 1
  #define DEBUG_SEND 1
#endif

//#define PRINT_RX_STATS 1

#include "src/delay_helpers.h" // rtos delay helpers
#include "src/config.h"        // project wide defs
#include "src/packet.h"        // data packet defs
#include "src/commands.h"      // command spec
#include "pins.h"                  // groundstation system pinouts

#define PC_SERIAL Serial // PC connection to send plain text over

// freertos task handles
TaskHandle_t Handle_radTask;
TaskHandle_t Handle_serTask;

// freeRTOS semaphores
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t spiSem; // semaphore for spi to radio
SemaphoreHandle_t cmdSem; // semaphore to access the command structure we want to send over the radio

// variables for the debug radio
uint8_t targetNode = NODE_ADDRESS_TESTNODE;
static uint8_t radioTxBuf[RADIO_TX_BUFSIZE]; // packets queued for sending (telem to groundstation)
static uint16_t radioTxBufSize = 0;
static uint8_t radioRxBuf[RADIO_RX_BUFSIZE]; // data 
static uint16_t radioRxBufSize = 0;
RFM69 radio(PIN_RADIO_SS, PIN_RADIO_INT, &SPI); // debug radio object

// for printing data to serial
StaticJsonDocument<1024> doc;

volatile bool newCmdToSend = false;
static cmd_t cmdToSend;

// serial command  variables
char serial_command_buffer_[32]; // max received command length
// parser object
SerialCommands serial_commands_(&PC_SERIAL, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");


// serial command andler to set targe node address
void cmd_set_target(SerialCommands* sender)
{
  //Note: Every call to Next moves the pointer to next parameter
	char* targetNodeStr = sender->Next();
	if (targetNodeStr == NULL) {
    // if no arg supplied, set target node to TESTNODE
		targetNode = NODE_ADDRESS_TESTNODE;
		return;
	}
  
  // set node address to TESTNODE if supplied argument is invalid
	if (targetNode = atoi(targetNodeStr) == 0 ){
    targetNode = NODE_ADDRESS_TESTNODE;
  }

  #ifdef DEBUG_SEND
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    sender->GetSerial()->print("setting target node to ");
    sender->GetSerial()->println(targetNode);
    xSemaphoreGive( dbSem );
  }
  #endif 
}

// serial command handler for initiating a send of the C02 paraachute deploy
void cmd_build_packet(SerialCommands* sender)
{
  #ifdef DEBUG_SEND
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    sender->GetSerial()->print("sending parachute deploy...");
    xSemaphoreGive( dbSem );
  }
  #endif

  // get semaphore access to command struct
  if ( xSemaphoreTake( cmdSem, ( TickType_t ) 500 ) == pdTRUE ) {

    // set all entries to zero (this sets argc to zero)
    memset(&cmdToSend, 0, sizeof(cmd_t));
    cmdToSend.cmdid = CMDID_IR_BP; // build packet command ID
    newCmdToSend = true;
    xSemaphoreGive( cmdSem );
  }
}

// serial command handler for initiating a send of the pyro cutter firing
void cmd_send_packet(SerialCommands* sender)
{
  #ifdef DEBUG_SEND
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    sender->GetSerial()->print("sending 'send packet' command...");
    xSemaphoreGive( dbSem );
  }
  #endif

  // get semaphore access to command struct
  if ( xSemaphoreTake( cmdSem, ( TickType_t ) 500 ) == pdTRUE ) {

    // set all entries to zero (this sets argc to zero)
    memset(&cmdToSend, 0, sizeof(cmd_t));
    cmdToSend.cmdid = CMDID_IR_SP; // send packet command ID
    newCmdToSend = true;
    xSemaphoreGive( cmdSem );
  }

}


// This is the default handler, and gets called when no other command matches. 
void cmd_help(SerialCommands* sender, const char* cmd)
{
  #ifdef DEBUG_SEND
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    sender->GetSerial()->print("Unrecognized command [");
    sender->GetSerial()->print(cmd);
    sender->GetSerial()->println("]");

    // TODO: print list of available commands..

    xSemaphoreGive( dbSem );
  }
  #endif
}

SerialCommand cmd_bp_("bp", cmd_build_packet); // build iridium packet
SerialCommand cmd_sp_("sp", cmd_send_packet); // send iridium packet
SerialCommand cmd_st_("target", cmd_send_packet); // set target


// dispacth a command received from the capsule (if any, could be used as ACK / heartbeat)
void dispatchCommand(int senderId, cmd_t command )
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    PC_SERIAL.print("Got command from address [");
    PC_SERIAL.print(senderId, HEX);
    PC_SERIAL.print("], of value [");
    PC_SERIAL.print(command.cmdid);
    PC_SERIAL.println("]");
    xSemaphoreGive( dbSem );
  }
  #endif
}

/// @brief radioThread interfaces with the RFM69HCW to communicate with the capsule
/// @param param unused
void radioThread( void *param ){
  int fromNode = -1;
  cmd_t inCmd;
  
  radio.initialize(FREQUENCY, NODE_ADDRESS_STATION, NETWORK_ID);
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
  radio.encrypt(ENCRYPTKEY);

  int rxBufPos = 0;
  int txBufPos = 0;

  while( 1 ){

    // first get access to SPI bus
    if ( xSemaphoreTake( spiSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      
      // check for any received packets, but only if they will fit in our RX buffer
      if (radio.receiveDone())
      {
        if( (radioRxBufSize + radio.DATALEN) < RADIO_RX_BUFSIZE) {
          memcpy(radioRxBuf, radio.DATA, radio.DATALEN);
          radioRxBufSize += radio.DATALEN;

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
        
      
      xSemaphoreGive( spiSem );
    } // end spi access

    // interpret received data, if any
    // since max reception size is 60 bytes and we parse it right after we 
    // receive it, precautions would have to be taken to received payloads
    // larger than 60 bytes
    // this approach assumes all commands are less than 60 bytes and also 
    // that the first byte of the rx buffer is always a packet ID byte
    if( radioRxBufSize > 0 ){
      
      #ifdef DEBUG_RADIO
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
        switch( radioRxBuf[radrxbufidx] ){
          case PTYPE_CMD:
            if( sizeof(cmd_t) > (radioRxBufSize - radrxbufidx)){
              // short read, ditch it
              // TODO: handle this better
              radrxbufidx = radioRxBufSize;
              #ifdef DEBUG_RADIO
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
                Serial.println("RADIO:  less data in the radio buffer than expected");
                xSemaphoreGive( dbSem );
              }
              #endif
            } else {
              memcpy(&inCmd, &radioRxBuf[radrxbufidx], sizeof(cmd_t));
              radrxbufidx += sizeof(cmd_t);
              dispatchCommand(fromNode, inCmd);
            }
            break;
          default:
            #ifdef DEBUG_RADIO
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
              Serial.println("RADIO:  received unknown command byte: ");
              Serial.print(radioRxBuf[radrxbufidx], HEX);
              xSemaphoreGive( dbSem );
            }
            #endif
            break;
        }
        if( radrxbufidx == radioRxBufSize ){
          goon = false;
        }
      }

    }

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
        if ( xSemaphoreTake( spiSem, ( TickType_t ) 1000 ) == pdTRUE ) {

          // if there is more than 60 bytes in the send buffer, send 60 of it over and over
          if( (radioTxBufSize - txBufPos) > 60 ){
            if (radio.sendWithRetry(targetNode, &radioTxBuf[txBufPos], 60 )){
              // success
              #ifdef DEBUG_RADIO
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
                Serial.print(" ok!");
                xSemaphoreGive( dbSem );
              }
              #endif
              txBufPos += 60;
            } else {
              // sending failed 
              #ifdef DEBUG_RADIO
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
                Serial.print(" nothing..."); 
                xSemaphoreGive( dbSem );
              }
              #endif
            }
          } else { // else if there is 60 or less bytes in the send buffer, just send what is there
            if (radio.sendWithRetry(targetNode, &radioTxBuf[txBufPos], (radioTxBufSize - txBufPos) )){
              // send success
              #ifdef DEBUG_RADIO
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
                Serial.print(" ok!");
                xSemaphoreGive( dbSem );
              }
              #endif
              txBufPos += (radioTxBufSize - txBufPos);
            } else {
              // sending failed 
              #ifdef DEBUG_RADIO
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
                Serial.print(" nothing..."); 
                xSemaphoreGive( dbSem );
              }
              #endif          
            }
          }

          

          xSemaphoreGive( spiSem );
        } // end access to the SPI bus
      }

      radioTxBufSize = 0;
    } // end if radioTxBufSize > 0
  }

  vTaskDelete (NULL);
}



/// @brief 
/// @param param 
void serialThread( void *param ){
  serial_commands_.SetDefaultHandler(cmd_help);
  serial_commands_.AddCommand(&cmd_bp_);
  serial_commands_.AddCommand(&cmd_sp_);
  serial_commands_.AddCommand(&cmd_st_);
  
  while (1) {
    serial_commands_.ReadSerial();
  }
  
  vTaskDelete (NULL);

}


void setup() {
  PC_SERIAL.begin(115200);
  delay(10);
  

  // manual reset on RFM69 radio, won't do anything if unpowered (3v32_ctrl pin)
  pinMode(PIN_RADIO_RESET, OUTPUT);
  digitalWrite(PIN_RADIO_RESET, HIGH);
  delay(10);
  digitalWrite(PIN_RADIO_RESET, LOW);
  delay(10);

  delay(4000);
  
  #if DEBUG
  PC_SERIAL.println("Starting...");
  #endif
  
  // INIT SEMAPHORES
  // setup debug serial log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  // setup command struct semaphore
  if ( cmdSem == NULL ) {
    cmdSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( cmdSem ) != NULL )
      xSemaphoreGive( ( cmdSem ) );  // make available
  }

  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(radioThread, "Radio Control", 1000, NULL, tskIDLE_PRIORITY + 2, &Handle_radTask);
  xTaskCreate(serialThread, "Serial Interface", 1000, NULL, tskIDLE_PRIORITY + 2, &Handle_serTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);
  
  #if DEBUG
  PC_SERIAL.println("Created tasks...");
  #endif
  
  delay(100);
  
  // start the scheduler
  vTaskStartScheduler();

  // error scheduler failed to start
  while(1)
  {
    #ifdef DEBUG
	  PC_SERIAL.println("Scheduler Failed! \n");
    #endif
	  delay(1000);
  }
  
}

void loop() {
  // tasks!
}