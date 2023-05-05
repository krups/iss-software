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
//#include <ArduinoJson.h>
#include <semphr.h>

#define SPEC_SATURATION 700
#define SPEC_UNDEREXPOSED 600

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
batt_t batt_data;
uint16_t data_batt[BATT_MEASUREMENTS];
uint8_t address = 0x0B;

// freertos task handles
TaskHandle_t Handle_specTask;
TaskHandle_t Handle_serTask;
TaskHandle_t Handle_batTask;

// freeRTOS semaphores
SemaphoreHandle_t dbSem; // serial debug logging
SemaphoreHandle_t specBufSem; // access to spectrometer data buffer
SemaphoreHandle_t batBufSem;  // access to battery data buffer
SemaphoreHandle_t s1Sem; // access to serial 1

// spectrometer variables
volatile unsigned long c= 0;
unsigned long delayTime = 100; // integration time
uint16_t data_spec[NUM_SPEC_CHANNELS]; // Define an array for the data read by the spectrometer
int multipliers_1[NUM_SPEC_CHANNELS] = {0}; // Define an array for the coefficients of the simpson's rule
float const coeff = ((850.0 - 340.0) / (NUM_SPEC_CHANNELS - 1) / 3); // Initial coefficient for the simpson's rule

uint16_t get_voltage1(){
  uint16_t reading = 0;
  Wire.beginTransmission(address);
  Wire.write(0x3F);
  Wire.endTransmission();
  Wire.requestFrom(address, 2);
  //if(2 <= Wire.available())    // if 3 bytes were received
  //{
    myDelayMs(10);
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading += Wire.read(); // receive low byte as lower 8 bits
    reading = reading << 8; 
  //} 
  return reading;
}

uint16_t get_voltage2(){
  uint16_t reading = 0;
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
  } 
  return reading;
}

uint16_t get_voltage3(){
  uint16_t reading = 0;
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
  } 
  return reading;
}

uint16_t get_total_voltage(){
  uint16_t reading = 0;
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
  } 
  return reading;
}

uint16_t get_current(){
  uint16_t reading = 0;
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
  } 
  return reading;
}

void quickDelay(unsigned int val){
//val *= 10;
 while(val>1){
  c--;
  val--;
 }
}

void readSpectrometerFast(uint16_t *data)
{ // from the spec sheet of the spectrometer
  int i = 0;
  // start clock cycle and set start pulse to signal start
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA12;
  i++; i++;
  PORT->Group[PORTA].OUTSET.reg = PORT_PA12;
  i++; i++;
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA12;
  //digitalWrite(SPEC_ST, HIGH);
  PORT->Group[PORTB].OUTSET.reg = PORT_PB10;
  i++; i++;

  // integration time
  for (int j = 0; j < 7+delayTime; j++)
  {
    
    PORT->Group[PORTA].OUTSET.reg = PORT_PA12;
    i++; i++;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA12;
    i++; i++;
  }

  // Set SPEC_ST to low
  //digitalWrite(SPEC_ST, LOW);
  PORT->Group[PORTB].OUTCLR.reg = PORT_PB10;
  // Sample for a period of time
  for (int j = 0; j < 87; j++)
  {
    PORT->Group[PORTA].OUTSET.reg = PORT_PA12;
    i++; i++;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA12;
    i++; i++;
  }

  // Read from SPEC_VIDEO
  for (int j = 0; j < 288; j++)
  {
    data[j] = analogRead(SPEC_VIDEO);
    PORT->Group[PORTA].OUTSET.reg = PORT_PA12;
    i++; i++;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA12;
    i++; i++;  
  }
  PORT->Group[PORTA].OUTSET.reg = PORT_PA12;
  i++; i++;  
}

void readSpectrometerFast2(uint16_t *data)
{// This is from the spec sheet of the spectrometer

    // Start clock cycle and set start pulse to signal start
    digitalWrite(SPEC_CLK, LOW);
    quickDelay(delayTime);
    digitalWrite(SPEC_CLK, HIGH);
    quickDelay(delayTime);
    digitalWrite(SPEC_CLK, LOW);
    digitalWrite(SPEC_ST, HIGH);
    quickDelay(delayTime);

    // Sample for a period of time
    for (int i = 0; i < 15; i++)
    {

        digitalWrite(SPEC_CLK, HIGH);
        quickDelay(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        quickDelay(delayTime);
    }

    // Set SPEC_ST to low
    digitalWrite(SPEC_ST, LOW);

    // Sample for a period of time
    for (int i = 0; i < 85; i++)
    {

        digitalWrite(SPEC_CLK, HIGH);
        quickDelay(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        quickDelay(delayTime);
    }

    // One more clock pulse before the actual read
    digitalWrite(SPEC_CLK, HIGH);
    quickDelay(delayTime);
    digitalWrite(SPEC_CLK, LOW);
    quickDelay(delayTime);

    // Read from SPEC_VIDEO
    for (int i = 0; i < NUM_SPEC_CHANNELS; i++)
    {

        data[i] = analogRead(SPEC_VIDEO);

        digitalWrite(SPEC_CLK, HIGH);
        quickDelay(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        quickDelay(delayTime);
    }

    // Set SPEC_ST to high
    digitalWrite(SPEC_ST, HIGH);

    // Sample for a small amount of time
    for (int i = 0; i < 7; i++)
    {

        digitalWrite(SPEC_CLK, HIGH);
        quickDelay(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        quickDelay(delayTime);
    }

    digitalWrite(SPEC_CLK, HIGH);
    quickDelay(delayTime); 
}

void readSpectrometer(uint16_t *data)
{ // This is from the spec sheet of the spectrometer

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

int arrMax(uint16_t *data, int len) {
  int mval = 0;
  for( int i=0; i<len; i++ ){
    mval = max(mval, data[i]);
  }
  return mval;
}

int countOver( uint16_t* data, int len, int val) {
  int count = 0;
  for( int i=0; i<len-1; i++ ){
    if( data[i] > val )
      count ++;
  }
  return count;
}

// print out a big line of spectrometer data, and timestamp and the number of channels
// not final by any means (timestamp first?)
void printData(uint16_t *data, bool goFast)
{ // Print the NUM_SPEC_CHANNELS data, then print the current time, the current color, and the number of channels.
    //SERIAL_DEBUG.print(id);
    //SERIAL_DEBUG.print(',');
    for (int i = 0; i < NUM_SPEC_CHANNELS-1; i++)
    {
        //    data_matrix(i) = data[i];
        SERIAL_DEBUG.print(data[i]);
        SERIAL_DEBUG.print(',');
    }
    SERIAL_DEBUG.print(data[NUM_SPEC_CHANNELS-1]);
    SERIAL_DEBUG.print(", ");
    SERIAL_DEBUG.print(goFast ? "FAST, " : "NORMAL, ");
    SERIAL_DEBUG.println(delayTime);
    //SERIAL_DEBUG.print(result[0] + result[1]);
    //SERIAL_DEBUG.print(',');
    //SERIAL_DEBUG.print(xTaskGetTickCount());
    //SERIAL_DEBUG.print(',');
    //SERIAL_DEBUG.print(NUM_SPEC_CHANNELS);
}

// terrible copy of print data to just use a different serial port 
// aka to main flight computer over hardware serial as opposed to us serial debug
void printDataToFC(uint8_t *data, uint8_t PTYPE)
{ // Print the NUM_SPEC_CHANNELS data, then print the current time, the current color, and the number of channels.
  if(PTYPE == PTYPE_SPEC) {
    SERIAL_FC.write(PTYPE_SPEC);
    SERIAL_FC.write((uint8_t*)data, sizeof(spec_t));
  }
  else if(PTYPE == PTYPE_BATT) {
    SERIAL_FC.write(PTYPE_BATT);
    SERIAL_FC.write((uint8_t*)data, sizeof(batt_t));
  }
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
  unsigned long lastLogTime = 0;
  bool goFast = false;
  //float res2[2];


  while( 1 ){
    // take reading from spectrometer, disabling interrupts to ensure proper timing
    taskENTER_CRITICAL();
    //if( goFast )
      readSpectrometerFast(data_spec);
    //else
     // readSpectrometer(data_spec);
    taskEXIT_CRITICAL();

    // log at slower rate
    spec_data.t = xTaskGetTickCount();
    if ( spec_data.t - lastLogTime > SPEC_SAMPLE_PERIOD_MS ){
      // put data in logging struct
      memcpy(spec_data.data, data_spec, sizeof(data_spec));
      if ( xSemaphoreTake( s1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
        printData(data_spec, goFast);
        printDataToFC((uint8_t *)&spec_data, PTYPE_SPEC);
        xSemaphoreGive( s1Sem );
      }
      lastLogTime = spec_data.t;
    }

    // exposure adjustment, update 20Hz
    if( arrMax(data_spec, NUM_SPEC_CHANNELS) > SPEC_SATURATION){
      if( countOver(data_spec, NUM_SPEC_CHANNELS, SPEC_SATURATION) > 20 ){
        if( delayTime > 2) delayTime /= 2;
        if( delayTime==0) delayTime = 2;
        // else if (!goFast){
        //   goFast = true;
        //   delayTime = 50;
        // }

      }
    }
    if( arrMax(data_spec, NUM_SPEC_CHANNELS) < SPEC_UNDEREXPOSED) {
      if( delayTime < 100000 ) delayTime *= 2;
      // else if( goFast ) {
      //   goFast = false;
      //   delayTime = 20;
      // }
    }

    myDelayMs(10);
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
    data_batt[0] = get_voltage1(); // cell 1
    data_batt[1] = get_voltage2(); // cell 2
    data_batt[2] = get_voltage3(); // cell 3
    data_batt[3] = get_total_voltage(); // pack
    data_batt[4] = get_current();   

    // debug log
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      // printData(data_batt, res1, PTYPE_BATT);
      Serial.print("collect voltage of cell 1:");
      Serial.println(data_batt[0]);   // print the reading
      Serial.print("collect voltage of cell 2:");
      Serial.println(data_batt[1]);   // print the reading
      Serial.print("collect voltage of cell 3:");
      Serial.println(data_batt[2]);   // print the reading
      Serial.print("collect voltage of pack:");
      Serial.println(data_batt[3]);   // print the reading
      Serial.print("collect current:");
      Serial.println(data_batt[4]);   // print the reading
      xSemaphoreGive( dbSem );
    }
    #endif

    // put data in logging struct
    batt_data.t = xTaskGetTickCount();
    memcpy(batt_data.data, data_batt, sizeof(data_batt));
    printDataToFC((uint8_t *)&batt_data, PTYPE_BATT);

    if ( xSemaphoreTake( s1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      // printData(data_batt, res1, PTYPE_BATT);
      printDataToFC((uint8_t *)&batt_data, PTYPE_BATT);
      xSemaphoreGive( s1Sem );
    }

    myDelayMs(BATT_SAMPLE_PERIOD_MS);
    
  }    
  
  vTaskDelete (NULL);

}

void setup() {
  SERIAL_DEBUG.begin(115200);
  SERIAL_FC.begin(115200);

  Wire.begin();
  
  delay(4000);

  pinMode(SPEC_ST, OUTPUT);
  // pinMode(SPEC_TRIG, INPUT);
  pinMode(SPEC_CLK, OUTPUT);
  // pinMode(SPEC_EOS, INPUT);
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
  // setup serial 1 semaphore
  if ( s1Sem == NULL ) {
    s1Sem = xSemaphoreCreateMutex(); // create mutex
    if ( ( s1Sem ) != NULL )
      xSemaphoreGive( ( s1Sem ) ); // make available
  }  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(specThread, "Radio Control", 1000, NULL, tskIDLE_PRIORITY, &Handle_specTask);
  xTaskCreate(serialThread, "Serial Interface", 1000, NULL, tskIDLE_PRIORITY, &Handle_serTask);
  //xTaskCreate(batThread, "Battery monitoring",  1000, NULL, tskIDLE_PRIORITY, &Handle_batTask);
  
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