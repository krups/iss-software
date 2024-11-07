//define which board is th4e compilation targer in the .ino file. When this gets updated to ESF-IDF we will create a .ini file
// Hersch Nathan updating Matt Ruffner's Code for ESP32S3
// Last Update 10/07/2024

#ifndef DELAY_HELPERS_H
#define DELAY_HELPERS_H

#if (defined(USE_SAMD21) && defined(USE_SAMD51)) || \
    (defined(USE_SAMD21) && defined(USE_ESP32S3)) || \
    (defined(USE_SAMD51) && defined(USE_ESP32S3))
#error "Only one of USE_SAMD21, USE_SAMD51, or USE_ESP32S3 should be defined."
#elif defined(USE_SAMD21) || defined(USE_SAMD51)
// rtos thread delay helpers
//**************************************************************************
void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}
//**************************************************************************
#elif defined(USE_ESP32S3)
void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_MS );  
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_MS );  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_MS );  
}
#else
#error "No valid microcontroller defined. Please define one of USE_SAMD21, USE_SAMD51, or USE_ESP32S3."


#endif
#endif
