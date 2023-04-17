// BSMS Firmware for KREPE-2 ISS mission
// Matt Ruffner, [other members of the team here], 2022
// This software runs on the BSMS processor, reading the spectrometer and battery status 
// and reporting to the main flight computer processor


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_NeoPixel.h>
#include <FreeRTOS_SAMD51.h>
//#include <ArduinoJson.h>
#include <semphr.h>

#define DEBUG 1
#ifdef DEBUG
    // more specific debug directives
#endif

#include "delay_helpers.h" // rtos delay helpers
#include "config.h"        // project wide defs
#include "packet.h"        // data packet defs
#include "pins.h"              // BSMS system pinouts

#define SERIAL_DEBUG Serial
#define SERIAL_FC    Serial1

spec_t spec_data;
uint16_t batt_data[BATT_MEASUREMENTS];
int address = 0x0B;
uint16_t reading, cell_1, cell_2, cell_3, pack, current;

// freertos task handles
TaskHandle_t Handle_specTask;
TaskHandle_t Handle_serTask;
TaskHandle_t Handle_batTask;

// freeRTOS semaphores
SemaphoreHandle_t dbSem; // serial debug logging
SemaphoreHandle_t specBufSem; // access to spectrometer data buffer
SemaphoreHandle_t batBufSem;  // access to battery data buffer

// spectrometer variables
uint16_t data_spec[NUM_SPEC_CHANNELS]; // Define an array for the data read by the spectrometer
int multipliers_1[NUM_SPEC_CHANNELS] = {0}; // Define an array for the coefficients of the simpson's rule
float const coeff = ((850.0 - 340.0) / (NUM_SPEC_CHANNELS - 1) / 3); // Initial coefficient for the simpson's rule

void get_voltage1(){
  Serial.println("collect voltage of cell 1:");
  Wire.beginTransmission(address);
  Wire.write(0x3F);
  Wire.endTransmission();
  Wire.requestFrom(address, 2);
   if(2 <= Wire.available())    // if 3 bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading += Wire.read(); // receive low byte as lower 8 bits
    reading = reading << 8; 
    Serial.println(reading);   // print the reading
    batt_data[0] = reading;
  } 
}

void get_voltage2(){
  Serial.println("collect voltage of cell 2:");
  Wire.beginTransmission(address);
  Wire.write(0x3E);
  Wire.endTransmission();
  Wire.requestFrom(address, 2);
   if(2 <= Wire.available())    // if 3 bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading += Wire.read(); // receive low byte as lower 8 bits
    reading = reading << 8; 
    Serial.println(reading);   // print the reading
    batt_data[1] = reading;
  } 
}

void get_voltage3(){
  Serial.println("collect voltage of cell 3:");
  Wire.beginTransmission(address);
  Wire.write(0x3D);
  Wire.endTransmission();
  Wire.requestFrom(address, 2);
   if(2 <= Wire.available())    // if 3 bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading += Wire.read(); // receive low byte as lower 8 bits
    reading = reading << 8; 
    Serial.println(reading);   // print the reading
    batt_data[2] = reading;
  } 
}

void get_total_voltage(){
  Serial.println("collect total voltage:");
  Wire.beginTransmission(address);
  Wire.write(0x09);
  Wire.endTransmission();
  Wire.requestFrom(address, 2);
   if(2 <= Wire.available())    // if 3 bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading += Wire.read(); // receive low byte as lower 8 bits
    reading = reading << 8; 
    Serial.println(reading);   // print the reading
    batt_data[3] = reading;
  } 
}

void get_current(){
  Serial.println("Collect Current:");
  Wire.beginTransmission(address);
  Wire.write(0x0B);
  Wire.endTransmission();
  Wire.requestFrom(address, 2);
   if(2 <= Wire.available())    // if 3 bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading += Wire.read(); // receive low byte as lower 8 bits
    reading = reading << 8; 
    Serial.println(reading);   // print the reading
    batt_data[4] = reading;
  } 
}

void readSpectrometerFast(uint16_t *data)
{ // from the spec sheet of the spectrometer
  int delayTime = 1; // delay time
  i = 0;
  // start clock cycle and set start pulse to signal start
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA14;
  i++; i++;
  PORT->Group[PORTA].OUTSET.reg = PORT_PA14;
  i++; i++;
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA14;
  //digitalWrite(SPEC_ST, HIGH);
  PORT->Group[PORTA].OUTSET.reg = PORT_PA05;
  i++; i++;

  // integration time
  for (int j = 0; j < 7; j++)
  {
    PORT->Group[PORTA].OUTSET.reg = PORT_PA14;
    i++; i++;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA14;
    i++; i++;
  }

  // Set SPEC_ST to low
  //digitalWrite(SPEC_ST, LOW);
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA05;
  // Sample for a period of time
  for (int j = 0; j < 87; j++)
  {
    PORT->Group[PORTA].OUTSET.reg = PORT_PA14;
    i++; i++;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA14;
    i++; i++;
  }

  // Read from SPEC_VIDEO
  for (int j = 0; j < 288; j++)
  {
    data[j] = analogRead(SPEC_VIDEO);
    PORT->Group[PORTA].OUTSET.reg = PORT_PA14;
    i++; i++;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA14;
    i++; i++;  
  }
  PORT->Group[PORTA].OUTSET.reg = PORT_PA14;
  i++; i++;  
}

void readSpectrometer(uint16_t *data)
{ // This is from the spec sheet of the spectrometer

    int delayTime = 1; // delay time

    // Start clock cycle and set start pulse to signal start
    digitalWrite(SPEC_CLK, LOW);
    myDelayUs(delayTime);
    digitalWrite(SPEC_CLK, HIGH);
    myDelayUs(delayTime);
    digitalWrite(SPEC_CLK, LOW);
    digitalWrite(SPEC_ST, HIGH);
    myDelayUs(delayTime);

    // Sample for a period of time
    for (int i = 0; i < 15; i++)
    {

        digitalWrite(SPEC_CLK, HIGH);
        myDelayUs(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        myDelayUs(delayTime);
    }

    // Set SPEC_ST to low
    digitalWrite(SPEC_ST, LOW);

    // Sample for a period of time
    for (int i = 0; i < 85; i++)
    {

        digitalWrite(SPEC_CLK, HIGH);
        myDelayUs(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        myDelayUs(delayTime);
    }

    // One more clock pulse before the actual read
    digitalWrite(SPEC_CLK, HIGH);
    myDelayUs(delayTime);
    digitalWrite(SPEC_CLK, LOW);
    myDelayUs(delayTime);

    // Read from SPEC_VIDEO
    for (int i = 0; i < NUM_SPEC_CHANNELS; i++)
    {

        data[i] = analogRead(SPEC_VIDEO);

        digitalWrite(SPEC_CLK, HIGH);
        myDelayUs(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        myDelayUs(delayTime);
    }

    // Set SPEC_ST to high
    digitalWrite(SPEC_ST, HIGH);

    // Sample for a small amount of time
    for (int i = 0; i < 7; i++)
    {

        digitalWrite(SPEC_CLK, HIGH);
        myDelayUs(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        myDelayUs(delayTime);
    }

    digitalWrite(SPEC_CLK, HIGH);
    myDelayUs(delayTime);
}

// print out a big line of spectrometer data, and timestamp and the number of channels
// not final by any means (timestamp first?)
void printData(uint16_t *data, float result[2], int id)
{ // Print the NUM_SPEC_CHANNELS data, then print the current time, the current color, and the number of channels.
    //SERIAL_DEBUG.print(id);
    //SERIAL_DEBUG.print(',');
    for (int i = 0; i < NUM_SPEC_CHANNELS-1; i++)
    {
        //    data_matrix(i) = data[i];
        SERIAL_DEBUG.print(data[i]);
        SERIAL_DEBUG.print(',');
    }
    SERIAL_DEBUG.println(NUM_SPEC_CHANNELS-1);
    //SERIAL_DEBUG.print(result[0] + result[1]);
    //SERIAL_DEBUG.print(',');
    //SERIAL_DEBUG.print(xTaskGetTickCount());
    //SERIAL_DEBUG.print(',');
    //SERIAL_DEBUG.print(NUM_SPEC_CHANNELS);
}

// terrible copy of print data to just use a different serial port 
// aka to main flight computer over hardware serial as opposed to us serial debug
void printDataToFC(spec_t *data)
{ // Print the NUM_SPEC_CHANNELS data, then print the current time, the current color, and the number of channels.
  SERIAL_FC.write(PTYPE_SPEC);
  SERIAL_FC.write((uint8_t*)data, sizeof(spec_t));
    // SERIAL_FC.print(id);
    // SERIAL_FC.print(',');
    // for (int i = 0; i < NUM_SPEC_CHANNELS; i++)
    // {
    //     //    data_matrix(i) = data[i];
    //     SERIAL_FC.print(data[i]);
    //     SERIAL_FC.print(',');
    // }
    // SERIAL_FC.print(result[0] + result[1]);
    // SERIAL_FC.print(',');
    // SERIAL_FC.print(xTaskGetTickCount());
    // SERIAL_FC.print(',');
    // SERIAL_FC.print(NUM_SPEC_CHANNELS);

    // SERIAL_FC.print("\n");
}

void calcIntLoop(uint16_t *data, int *multipliers, float* result)
{
    for (int i = 0; i < 88; i++)
    { // Calculate each value for the simpson's rule.
        result[0] += multipliers[i] * data[i];
    }
    result[0] *= coeff;

    for (int i = 88; i < NUM_SPEC_CHANNELS; i++)
    { // Calculate each value for the simpson's rule.
        result[1] += multipliers[i] * data[i];
    }
    result[1] *= coeff;
}


/// @brief specThread interfaces with the spectrometer to read the data
//          and puts it into a shared buffer
void specThread( void *param ){
  
  float res1[2];
  //float res2[2];


  while( 1 ){
    res1[0] = 0; res1[1] = 0;
    //res2[0] = 0; res2[1] = 0;

    // take reading from spectrometer, disabling interrupts to ensure proper timing
    taskENTER_CRITICAL();
    readSpectrometerFast(data_spec);
    taskEXIT_CRITICAL();

    // put data in logging struct
    spec_data.t = xTaskGetTickCount();
    memcpy(spec_data.data, data_spec, sizeof(data_spec));

    // debug log
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      printData(data_spec, res1, PTYPE_SPEC);
      printDataToFC(&spec_data);

      //Serial.print("Spectro 1 Res: ");
      //Serial.print(res1[0]);
      //Serial.print("/");
      //Serial.println(res1[1]);
      // printData(data_spec_1, res1, SPECTROMETER_1_SERIAL);
      // Serial.print("Spectro 1 Res: ");
      // Serial.print(res1[0]);
      // Serial.print("/");
      // Serial.println(res1[1]);
      xSemaphoreGive( dbSem );
    }
    #endif

    myDelayMs(SPEC_SAMPLE_PERIOD_MS);
  }

  vTaskDelete (NULL);
}



/// @brief serialThread takes data from a shared buffer and ouputs it over the
//          serial port to the main flight computer
void serialThread( void *param ){
 

  while (1)
  {
    
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
  SERIAL_DEBUG.begin(115200);
  SERIAL_FC.begin(115200);
  
  delay(4000);

  pinMode(SPEC_ST, OUTPUT);
  pinMode(SPEC_TRIG, INPUT);
  pinMode(SPEC_CLK, OUTPUT);
  pinMode(SPEC_EOS, INPUT);
  pinMode(SPEC_VIDEO, INPUT);
  

  int num = 0;

  for (int i = 0; i < NUM_SPEC_CHANNELS; i++)
  { // Create the coefficients for the simpson's rule integral
      if ((i == 0) || (i == NUM_SPEC_CHANNELS))
      { // I = Delta x/3 * ( 1 f(x_0) + 4 f(x_1) + 2 f(x_2) + ... + 2 f(x_{n-2}) + 4 f(x_{n-1}) + f(x_n) )
          num = 1;
      }
      else if (i % 2 == 1)
      {
          num = 4;
      }
      else if (i % 2 == 0)
      {
          num = 2;
      }
      multipliers_1[i] = num;
      //multipliers_2[i] = num;
  }


  #if DEBUG
  SERIAL_DEBUG.println("Starting...");
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
  SERIAL_DEBUG.println("Created tasks...");
  #endif
  
  delay(100);
  
  // start the scheduler
  vTaskStartScheduler();

  // error scheduler failed to start
  while(1)
  {
    #ifdef DEBUG
	  SERIAL_DEBUG.println("Scheduler Failed! \n");
    #endif
	  delay(1000);
  }
  
}

void loop() {
  // tasks!
}