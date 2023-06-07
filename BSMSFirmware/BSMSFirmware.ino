// BSMS Firmware for KREPE-2 ISS mission
// Matt Ruffner, [other members of the team here], 2022
// This software runs on the BSMS processor, reading the spectrometer and battery status 
// and reporting to the main flight computer processor


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_NeoPixel.h>
//#include <ArduinoJson.h>

// set which board this runs on 
// matt's proof of concept board is samd21
// senior design uses sam51 feather
#define USE_SAMD51 1
//#define USE_SAMD21 1

// include the right rtos library
#ifdef USE_SAMD51 
#include <FreeRTOS_SAMD51.h>
#elif USE_SAMD21
#include <FreeRTOS_SAMD21.h>
#endif
#if (USE_SAMD21 && USE_SAMD51)
#error "Cannot define both samd51 and 21"
#endif
#include <semphr.h>


//#define DEBUG 1
#ifdef DEBUG
    // more specific debug directives
#endif

// symlinks only work in unix
#include "src/delay_helpers.h" // rtos delay helpers
#include "src/config.h"        // project wide defs
#include "src/packet.h"        // data packet defs
#include "pins.h"              // BSMS specific system pinouts

#ifdef USE_SAMD51
#define SPEC_SATURATION 680
#define SPEC_UNDEREXPOSED 400
#elif USE_SAMD21
#define SPEC_SATURATION 700
#define SPEC_UNDEREXPOSED 500
#endif

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

// uint16_t get_voltage1(){
//   uint16_t reading = 0;
//   Wire.beginTransmission(address);
//   Wire.write(0x3F);
//   Wire.endTransmission();
//   Wire.requestFrom(address, 2);
//   //if(2 <= Wire.available())    // if 3 bytes were received
//   //{
//     myDelayMs(10);
//     reading = Wire.read();  // receive high byte (overwrites previous reading)
//     reading = reading << 8;    // shift high byte to be high 8 bits
//     reading += Wire.read(); // receive low byte as lower 8 bits
//     reading = reading << 8; 
//   //} 
//   return reading;
// }

// uint16_t get_voltage2(){
//   uint16_t reading = 0;
//   Wire.beginTransmission(address);
//   Wire.write(0x3E);
//   Wire.endTransmission();
//   Wire.requestFrom(address, 2);
//    if(2 <= Wire.available())    // if 3 bytes were received
//   {
//     reading = Wire.read();  // receive high byte (overwrites previous reading)
//     reading = reading << 8;    // shift high byte to be high 8 bits
//     reading += Wire.read(); // receive low byte as lower 8 bits
//     reading = reading << 8; 
//   } 
//   return reading;
// }

// uint16_t get_voltage3(){
//   uint16_t reading = 0;
//   Wire.beginTransmission(address);
//   Wire.write(0x3D);
//   Wire.endTransmission();
//   Wire.requestFrom(address, 2);
//    if(2 <= Wire.available())    // if 3 bytes were received
//   {
//     reading = Wire.read();  // receive high byte (overwrites previous reading)
//     reading = reading << 8;    // shift high byte to be high 8 bits
//     reading += Wire.read(); // receive low byte as lower 8 bits
//     reading = reading << 8; 
//   } 
//   return reading;
// }

// uint16_t get_total_voltage(){
//   uint16_t reading = 0;
//   Wire.beginTransmission(address);
//   Wire.write(0x09);
//   Wire.endTransmission();
//   Wire.requestFrom(address, 2);
//    if(2 <= Wire.available())    // if 3 bytes were received
//   {
//     reading = Wire.read();  // receive high byte (overwrites previous reading)
//     reading = reading << 8;    // shift high byte to be high 8 bits
//     reading += Wire.read(); // receive low byte as lower 8 bits
//     reading = reading << 8; 
//   } 
//   return reading;
// }

// uint16_t get_current(){
//   uint16_t reading = 0;
//   Wire.beginTransmission(address);
//   Wire.write(0x0B);
//   Wire.endTransmission();
//   Wire.requestFrom(address, 2);
//   if(2 <= Wire.available())    // if 3 bytes were received
//   {
//     reading = Wire.read();  // receive high byte (overwrites previous reading)
//     reading = reading << 8;    // shift high byte to be high 8 bits
//     reading += Wire.read(); // receive low byte as lower 8 bits
//     reading = reading << 8; 
//   } 
//   return reading;
// }

// void quickDelay(unsigned int val){
// //val *= 10;
//  while(val>1){
//   c--;
//   val--;
//  }
// }

// Spectrometer reading function, to be atomically, without a context switch
//
// integration time for SAMD21 running at 48MHz
// t_integ = (94+delayTime)*271 ns
//
// integration time for SAMD51 running at 120 MHz
// t_integ = (94+delayTime)*58 ns

void readSpectrometerFast(uint16_t *data)
{ // from the spec sheet of the spectrometer
  int i = 0;
  // start clock cycle and set start pulse to signal start
  PORT->Group[SPEC_CLK_PORT].OUTCLR.reg = SPEC_CLK_PORT_PIN;
  i++;
  PORT->Group[SPEC_CLK_PORT].OUTSET.reg = SPEC_CLK_PORT_PIN;
  i++;
  PORT->Group[SPEC_CLK_PORT].OUTCLR.reg = SPEC_CLK_PORT_PIN;
  //digitalWrite(SPEC_ST, HIGH);
  PORT->Group[SPEC_ST_PORT].OUTSET.reg = SPEC_ST_PORT_PIN;
  i++;

  // integration time
  for (int j = 0; j < 7+delayTime; j++)
  {
    
    PORT->Group[SPEC_CLK_PORT].OUTSET.reg = SPEC_CLK_PORT_PIN;
    i++;
    PORT->Group[SPEC_CLK_PORT].OUTCLR.reg = SPEC_CLK_PORT_PIN;
    i++;
  }

  // Set SPEC_ST to low
  //digitalWrite(SPEC_ST, LOW);
  PORT->Group[SPEC_ST_PORT].OUTCLR.reg = SPEC_ST_PORT_PIN;
  // Tlp low period of ST pin must be 375/f_clk
  for (int j = 0; j < 87; j++)
  {
    PORT->Group[SPEC_CLK_PORT].OUTSET.reg = SPEC_CLK_PORT_PIN;
    i++;
    PORT->Group[SPEC_CLK_PORT].OUTCLR.reg = SPEC_CLK_PORT_PIN;
    i++;
  }

  // Read from SPEC_VIDEO
  for (int j = 0; j < 288; j++)
  {
    PORT->Group[SPEC_CLK_PORT].OUTSET.reg = SPEC_CLK_PORT_PIN;
    data[j] = analogRead(SPEC_VIDEO);
    PORT->Group[SPEC_CLK_PORT].OUTCLR.reg = SPEC_CLK_PORT_PIN;
    i++;
  }
  PORT->Group[SPEC_CLK_PORT].OUTSET.reg = SPEC_CLK_PORT_PIN;
  i++;
}


uint16_t arrMin(uint16_t *data, int len) {
  uint16_t mval = 65535;
  for( int i=0; i<len; i++ ){
    mval = min(mval, data[i]);
  }
  return mval;
}

uint16_t arrMax(uint16_t *data, int len) {
  uint16_t mval = 0;
  for( int i=0; i<len; i++ ){
    mval = max(mval, data[i]);
  }
  return mval;
}

int countOver( uint16_t* data, int len, int val) {
  int count = 0;
  for( int i=0; i<len; i++ ){
    if( data[i] > val )
      count ++;
  }
  return count;
}

uint16_t countUnderOrEqual( uint16_t* data, int len, uint16_t val) {
  uint16_t count = 0;
  for( int i=0; i<len; i++ ){
    if( data[i] <= val )
      count ++;
  }
  return count;
}

void printSpecPacketToSerial(spec_t data) {
  SERIAL_DEBUG.print(((float)(data.t)/1000.0)); SERIAL_DEBUG.print(", ");
  SERIAL_DEBUG.print(data.itime); SERIAL_DEBUG.print(", ");
  SERIAL_DEBUG.print(data.data[0]); SERIAL_DEBUG.print(", ");
  SERIAL_DEBUG.print(data.data[1]); SERIAL_DEBUG.print(", ");
  SERIAL_DEBUG.print(data.data[2]); SERIAL_DEBUG.print(", ");
  SERIAL_DEBUG.print(data.data[3]); SERIAL_DEBUG.print(", ");
  SERIAL_DEBUG.print(data.data[4]); SERIAL_DEBUG.print(", ");
  SERIAL_DEBUG.println(data.data[5]);
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
        SERIAL_DEBUG.print(data[i]);
        SERIAL_DEBUG.print(',');
    }
    SERIAL_DEBUG.print(data[NUM_SPEC_CHANNELS-1]);
    SERIAL_DEBUG.print(", ");
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

// nm / bin  = (850nm - 340nm) / 288 bins 
// bin 1: 340nm - 496, 88 raw bins
// bins 2-6: 40 raw bins each, 71 nm bandwidth
// actual binwidth: 2.17 nm
// 
// bin1: 340-496nm: bin 0 through bin 72
// bin2-6, 32 bins wide, 71 nm bin width
void binSpectrometer( uint16_t *rawData, spec_t *output) {
  int i,j;
  float sums[6] = {0,0,0,0,0,0}; // bin sums
  int thresh[6] = {72,104,136,168,200,232}; // spectral bin boundaries
  for( i=0; i<6; i++ ){
    output->peaks[i] = 0; // zero out peaks
  }
  for( i=0,j=0; i<288; i++ ){
    sums[j] += rawData[i];
    if( rawData[i] > output->peaks[j] )
      output->peaks[j] = rawData[i];
    if( i > thresh[j] )
      j++;
  }
  
  sums[0] /= 72.0;
  sums[1] /= 32.0; sums[2] /= 32.0;
  sums[3] /= 32.0; sums[4] /= 32.0;
  sums[5] /= 56.0;

  output->data[0] = (uint8_t)sums[0];
  output->data[1] = (uint8_t)sums[1];
  output->data[2] = (uint8_t)sums[2];
  output->data[3] = (uint8_t)sums[3];
  output->data[4] = (uint8_t)sums[4];
  output->data[5] = (uint8_t)sums[5];
}

// zero mean and rescale to a certain max value
// in this case, 8 bit representation
void rescaleSpectrometer(uint16_t *rawData) {
  float temp[288];
  for(int i=0;i<288;i++) temp[i] = rawData[i];
  float aMin = (float)arrMin(rawData, 288);
  float aMax = (float)arrMax(rawData, 288);
  for(int i=0;i<288;i++) temp[i] -= aMin;
  for(int i=0;i<288;i++) temp[i] /= aMax;
  for(int i=0;i<288;i++) {
    rawData[i] = (uint16_t)(temp[i]*255);
    if( rawData[i] > 255 ) rawData[i] = 255;
  }
}

/// @brief specThread interfaces with the spectrometer to read the data
//          and puts it into a shared buffer
void specThread( void *param ){
  unsigned long lastLogTime = 0;
  unsigned long sampleInterval = SPEC_SAMPLE_PERIOD_MS;

  // readSpec fast creates a 271ns clock period, or a 3.69 MHz clock signal
  // available integration time ranges from 7 clock cycles (1.897us)
  // to around 200000+7 clock cycles (54.2ms) when running on the SAMD21 @ 48MHz
  // the times for running on a 120MHz SAMD51 are different and listed above the
  // read spectrometer function


  while( 1 ){
    // take reading from spectrometer, disabling interrupts to ensure proper timing
    taskENTER_CRITICAL();
    readSpectrometerFast(data_spec);
    taskEXIT_CRITICAL();

    spec_data.itime = delayTime;

    // exposure adjustment, update 20Hz, operate on raw data, before binning
    if( arrMax(data_spec, NUM_SPEC_CHANNELS) > SPEC_SATURATION){
      if( countOver(data_spec, NUM_SPEC_CHANNELS, SPEC_SATURATION) > 20 ){
        if( delayTime > 2) delayTime /= 2;
        if( delayTime==0) delayTime = 2;
      }
    }
    if( arrMax(data_spec, NUM_SPEC_CHANNELS) < SPEC_UNDEREXPOSED) {
      // get a similar maximum integration time with different cpu clock speeds
      // by allowing the feather m4 integration counter to go higher
      #ifdef USE_SAMD51
      if( delayTime < 500000 ) delayTime *= 2;
      #elif USE_SAMD21
      if( delayTime < 100000 ) delayTime *= 2;
      #endif
    }

    rescaleSpectrometer(data_spec);
    binSpectrometer(data_spec, &spec_data);

    // log at slower rate
    spec_data.t = xTaskGetTickCount();
    if ( spec_data.t - lastLogTime > sampleInterval ){
      if ( xSemaphoreTake( s1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
        printData(data_spec);
        //printSpecPacketToSerial(spec_data);
        printDataToFC((uint8_t *)&spec_data, PTYPE_SPEC);
        xSemaphoreGive( s1Sem );
      }
      lastLogTime = spec_data.t;
      if( lastLogTime > 15000 && sampleInterval < 1000 ){
        sampleInterval = 1000;
      }
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


// /// @brief thread monitoring battery info and storing in shared buffer to be sent back 
// //         to the  main flight computer
// void batThread( void *param ){
  
//   // initialize battery monitoring

//   while (1) {
//     // collect and store battery info
//     data_batt[0] = get_voltage1(); // cell 1
//     data_batt[1] = get_voltage2(); // cell 2
//     data_batt[2] = get_voltage3(); // cell 3
//     data_batt[3] = get_total_voltage(); // pack
//     data_batt[4] = get_current();   

//     // debug log
//     #ifdef DEBUG
//     if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//       // printData(data_batt, res1, PTYPE_BATT);
//       Serial.print("collect voltage of cell 1:");
//       Serial.println(data_batt[0]);   // print the reading
//       Serial.print("collect voltage of cell 2:");
//       Serial.println(data_batt[1]);   // print the reading
//       Serial.print("collect voltage of cell 3:");
//       Serial.println(data_batt[2]);   // print the reading
//       Serial.print("collect voltage of pack:");
//       Serial.println(data_batt[3]);   // print the reading
//       Serial.print("collect current:");
//       Serial.println(data_batt[4]);   // print the reading
//       xSemaphoreGive( dbSem );
//     }
//     #endif

//     // put data in logging struct
//     batt_data.t = xTaskGetTickCount();
//     memcpy(batt_data.data, data_batt, sizeof(data_batt));
//     printDataToFC((uint8_t *)&batt_data, PTYPE_BATT);

//     if ( xSemaphoreTake( s1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
//       // printData(data_batt, res1, PTYPE_BATT);
//       printDataToFC((uint8_t *)&batt_data, PTYPE_BATT);
//       xSemaphoreGive( s1Sem );
//     }

//     myDelayMs(BATT_SAMPLE_PERIOD_MS);
    
//   }    
  
//   vTaskDelete (NULL);

// }

void setup() {
  #ifdef DEBUG
  SERIAL_DEBUG.begin(115200);
  #endif

  SERIAL_FC.begin(115200);
  
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