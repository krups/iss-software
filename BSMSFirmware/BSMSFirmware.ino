// BSMS Firmware for KREPE-2 ISS mission
// Matt Ruffner, [other members of the team here], 2022
// This software runs on the BSMS processor, reading the spectrometer and battery status 
// and reporting to the main flight computer processor


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_NeoPixel.h>
#include <FreeRTOS_SAMD21.h>
#include <ArduinoJson.h>
#include <semphr.h>

#define DEBUG 1
#ifdef DEBUG
    // more specific debug directives
#endif

#include "src/delay_helpers.h" // rtos delay helpers
#include "src/config.h"        // project wide defs
#include "src/packet.h"        // data packet defs
#include "pins.h"              // BSMS system pinouts

#define SERIAL Serial

// freertos task handles
TaskHandle_t Handle_specTask;
TaskHandle_t Handle_serTask;
TaskHandle_t Handle_batTask;

// freeRTOS semaphores
SemaphoreHandle_t dbSem; // serial debug logging
SemaphoreHandle_t specBufSem; // access to spectrometer data buffer
SemaphoreHandle_t batBufSem;  // access to battery data buffer

// for printing data to serial
StaticJsonDocument<1024> doc;


/// @brief specThread interfaces with the spectrometer to read the data
//          and puts it into a shared buffer
void specThread( void *param ){
  

  while( 1 ){

  }

  vTaskDelete (NULL);
}



/// @brief serialThread takes data from a shared buffer and ouputs it over the
//          serial port to the main flight computer
void serialThread( void *param ){
 
  while (1) {
    // read spectro data from buffer and send out over serial

    // read battery info from buffer and send out over serial
  }
  
  vTaskDelete (NULL);

}


/// @brief thread monitoring battery info and storing in shared buffer to be sent back 
//         to the  main flight computer
void batThread( void *param ){
  
  // initialize battery monitoring

  while (1) {
    // collect and store battery info
  }
  
  vTaskDelete (NULL);

}


void setup() {
  SERIAL.begin(115200);
  delay(10);
  
  delay(4000);
  
  #if DEBUG
  SERIAL.println("Starting...");
  #endif
  
  // INIT SEMAPHORES
  // setup debug serial log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  // setup debug serial log semaphore
  if ( specBufSem == NULL ) {
    specBufSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( specBufSem ) != NULL )
      xSemaphoreGive( ( specBufSem ) );  // make available
  }
  // setup debug serial log semaphore
  if ( batBufSem == NULL ) {
    batBufSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( batBufSem ) != NULL )
      xSemaphoreGive( ( batBufSem ) );  // make available
  }
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(specThread, "Radio Control", 1000, NULL, tskIDLE_PRIORITY, &Handle_specTask);
  xTaskCreate(serialThread, "Serial Interface", 1000, NULL, tskIDLE_PRIORITY, &Handle_serTask);
  xTaskCreate(batThread, "Battery monitoring",  1000, NULL, tskIDLE_PRIORITY, &Handle_batTask);
  
  #if DEBUG
  SERIAL.println("Created tasks...");
  #endif
  
  delay(100);
  
  // start the scheduler
  vTaskStartScheduler();

  // error scheduler failed to start
  while(1)
  {
    #ifdef DEBUG
	  SERIAL.println("Scheduler Failed! \n");
    #endif
	  delay(1000);
  }
  
}

void loop() {
  // tasks!
}