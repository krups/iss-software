// Defines which board we are compiling for
// Written by: Hersch Nathan

// current options are:
// Kentucky Flight Computer for KANGS (KFC_KANGS)
// Fempto Sats for KANGS (Fempto_KANGS)
// Rocketstation Transmitter for Kangs (RST_KANGS)
// Ground Sation (GS)


//Constant Addresses are defined in this file

// configre format:
// if(current option)
//   define if i2c is enabled
//   define if spi is enabled
//   define if sensors are enabled
//   define if which communicatio is enabled
//   define I2C pins
//
//   for each group define pins and define which sensors are available




#ifndef CONFIG_H
#define CONFIG_H

#define Fempto_KANGS 1


// Constant I2C Addresses
#define BME280_I2C_ADDRESS 0x76
#define BN0066_I2C_ADDRESS 0x4A


#if defined(KFC_KANGS)

#elif defined(Fempto_KANGS)
    #define I2C1 1
    #define SPI1 1
   
    #define LoRa 1 
    #define SENSORS 1

    // I2C pins 
    #define SDA1_PIN 6
    #define SCL1_PIN 7

    // I2C Sensors
    #define BME280_I2C 1
    #define BNO086_I2C 1
    
    // BN0086 Pins for 
    #define BNO086_HINTN 4
    #define BNO086_RESET 5

    // LoRa Pins
    #define LORA1_TXEN 39
    #define LORA1_RXEN 38







#elif defined(RST_KANGS)

#elif defined(GS)

#else
    #error "No valid configuration defined"
#endif
    



#endif

