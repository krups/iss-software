/*
* KREPE ISS Mission
* Dec 2020
*
* Main mission config params
* 
*/


#ifndef CONFIG_H
#define CONFIG_H

#include "pins.h"

// the different mission scenarios
#define MISSION_REAL_SHUTTLE 1
#define MISSION_REAL_JSC     2
#define MISSION_TEST_SHUTTLE 3
#define MISSION_TEST_JSC     4

// mission type
#define MISSION_TYPE         MISSION_TEST_JSC

//////////////////////////////////////////
// real mission for the two capsules with shuttle tile TPS
#if (MISSION_TYPE == MISSION_REAL_SHUTTLE)
  #define USBSERIAL_DEBUG      0
  #define ISM_DEBUG            0
  #define TC_COUNT             4
  
  // TODO: refer to TcInterface for which physical connections these are if all
  //       8 possible thermocouple connections are not specified
  #define TC_TYPE_STRING       "KKKK" 
#endif

// real mission for the JSC TPS
//////////////////////////////////////////
#if (MISSION_TYPE == MISSION_REAL_JSC)
  #define USBSERIAL_DEBUG      0
  #define ISM_DEBUG            0
  #define TC_COUNT             5
  
  #define TC_TYPE_STRING       "RKRKR" 
#endif

// ground trial setup for shuttle tile TPS capsules
//////////////////////////////////////////
#if (MISSION_TYPE == MISSION_TEST_SHUTTLE)
  #define USBSERIAL_DEBUG      1
  #define ISM_DEBUG            1
  #define TC_COUNT             4
  
  #define TC_TYPE_STRING       "KKKK" 
#endif

// ground trial setup for the capsule with JSC TPS 
//////////////////////////////////////////
#if (MISSION_TYPE == MISSION_TEST_JSC)
  #define USBSERIAL_DEBUG      1
  #define ISM_DEBUG            1
  #define TC_COUNT             5
  
  #define TC_TYPE_STRING       "RKRKR" 
#endif







#endif
