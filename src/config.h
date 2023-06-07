#ifndef CONFIG_H
#define CONFIG_H


#define MISSION_AMTPS     1
#define MISSION_LI2200    2
#define MISSION_SPICA     3
#define MISSION_CPICA     4
#define MISSION_REUSE     5
#define MISSION_ROCKSAT   6

#define MISSION_ID MISSION_ROCKSAT

#ifndef MISSION_ID 
#error "Must define MISSION_ID"
#endif

#define USE_DEBUG_RADIO 1
//#define DEBUG 1 // usb serial debug switch
#ifdef DEBUG
  //#define DEBUG_GPS 1 // print raw gga to serial
  //#define DEBUG_QUEUE 1 // print info on log queue operations
  //#define DEBUG_VERBOSE 1
  //#define DEBUG_BARO 1
  #define DEBUG_PI 1
  #define DEBUG_IRD 1
  #define DEBUG_LOG 1
  //#define DEBUG_TICK 1
  //#define DEBUG_RADIO 1
  //#define DEBUG_PRESSURE 1
  //#define DEBUG_SPEC 1
  //#define DEBUG_IMU 1
  //#define DEBUG_TC
  //#define DEBUG_MCP_STARTUP
#endif

#define RAD2DEG 57.2957795131

#define TIME_SCALE 100.0f // ticks per time unit

#define UNIT_SCALE 10.0f // multiplier applied

#define PRS_UNIT_SCALE 100.0f // multiplier for pressure data


#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey"
#define NETWORK_ID    100
#define NODE_ADDRESS_STATION    1
#define NODE_ADDRESS_TESTNODE   2
#define NODE_ADDRESS_KREPE2_001 3 // krepe - amtps
#define NODE_ADDRESS_KREPE2_002 4 // krepe - li2200
#define NODE_ADDRESS_KREPE2_003 5 // krepe - spice
#define NODE_ADDRESS_KREPE2_004 6 // krepe - cpica
#define NODE_ADDRESS_KREPE2_005 7 // krepe - reuse
#define NODE_ADDRESS_KREPE2_006 8 // rocksat

// uncomment to enable GPS
// gps sample period is default 1Hz
#define USE_GPS


#define TC_SAMPLE_PERIOD_MS   1000
#define SPEC_SAMPLE_PERIOD_MS 250
#define IMU_SAMPLE_PERIOD_MS  500
#define PRS_SAMPLE_PERIOD_MS  1000

// TODO: update to BSMS language
// uncomment to enable spectrometer
#define USE_SPECTROMETER
#ifdef USE_SPECTROMETER
#define NUM_SPEC_CHANNELS 288
#endif

#define BATT_MEASUREMENTS 5

// uncomment to enable error led reporting
#define USE_LEDS

#define SEND_PACKETS 1 // set 1 for mission
#define IRIDIUM_PACKET_PERIOD 20000 // milliseconds, send a packet every minute
#define CHECK_SIGNAL_PERIOD   10000 // milliseconds
#define DIAGNOSTICS false// Change this to see diagnostics
#define SBD_TX_SZ 1960

// how often to build a packet (in milliseconds)
#define PACKET_BUILD_PERIOD 30000

// calibration for barometer altitude reading
#define BAR_SEA_PRESSURE 1013.26

// NUM_TC_CHANNELS + NUM_HF_CHANNELS should always be equal to the total number of MCP9600 chips (TOT_MCP_COUNT)
#define NUM_TC_CHANNELS       6 // deg celcius

// number of pressure sensors attached to I2C mux
#define NUM_PRS_CHANNELS      5

// TODO: MCORRECT TYPES MMKAY
#if MISSION_ID == MISSION_AMTPS
  #define NODE_ADDRESS NODE_ADDRESS_KREPE2_001

  #define TC_1_TYPE MCP9600_TYPE_K
  #define TC_2_TYPE MCP9600_TYPE_K
  #define TC_3_TYPE MCP9600_TYPE_K
  #define TC_4_TYPE MCP9600_TYPE_K
  #define TC_5_TYPE MCP9600_TYPE_K
  #define TC_6_TYPE MCP9600_TYPE_K

  #define PRS1_ADDRESS          0x78  // i2c address
  #define PRS1_MAX              160.0 //kpa
  #define PRS2_ADDRESS          0x78
  #define PRS2_MAX              160.0 //kpa
  #define PRS3_ADDRESS          0x78
  #define PRS3_MAX              160.0 //kpa
  #define PRS4_ADDRESS          0x78
  #define PRS4_MAX              160.0 //kpa
  #define PRS5_ADDRESS          0x78
  #define PRS5_MAX              160.0 //kpa
#endif

// TODO: MCORRECT TYPES MMKAY
#if MISSION_ID == MISSION_LI220
  #define NODE_ADDRESS NODE_ADDRESS_KREPE2_002

  #define TC_1_TYPE MCP9600_TYPE_K
  #define TC_2_TYPE MCP9600_TYPE_K
  #define TC_3_TYPE MCP9600_TYPE_K
  #define TC_4_TYPE MCP9600_TYPE_K
  #define TC_5_TYPE MCP9600_TYPE_K
  #define TC_6_TYPE MCP9600_TYPE_K

  // TODO: add pressure sensor definitions
#endif

// TODO: MCORRECT TYPES MMKAY
#if MISSION_ID == MISSION_SPICA
  #define NODE_ADDRESS NODE_ADDRESS_KREPE2_003

  #define TC_1_TYPE MCP9600_TYPE_K
  #define TC_2_TYPE MCP9600_TYPE_K
  #define TC_3_TYPE MCP9600_TYPE_K
  #define TC_4_TYPE MCP9600_TYPE_K
  #define TC_5_TYPE MCP9600_TYPE_K
  #define TC_6_TYPE MCP9600_TYPE_K

  // TODO: add pressure sensor definitions
#endif

// TODO: MCORRECT TYPES MMKAY
#if MISSION_ID == MISSION_CPICA
  #define NODE_ADDRESS NODE_ADDRESS_KREPE2_004

  #define TC_1_TYPE MCP9600_TYPE_K
  #define TC_2_TYPE MCP9600_TYPE_K
  #define TC_3_TYPE MCP9600_TYPE_K
  #define TC_4_TYPE MCP9600_TYPE_K
  #define TC_5_TYPE MCP9600_TYPE_K
  #define TC_6_TYPE MCP9600_TYPE_K
#endif

// TODO: MCORRECT TYPES MMKAY
#if MISSION_ID == MISSION_REUSE
  #define NODE_ADDRESS NODE_ADDRESS_KREPE2_005

  #define TC_1_TYPE MCP9600_TYPE_K
  #define TC_2_TYPE MCP9600_TYPE_K
  #define TC_3_TYPE MCP9600_TYPE_K
  #define TC_4_TYPE MCP9600_TYPE_K
  #define TC_5_TYPE MCP9600_TYPE_K
  #define TC_6_TYPE MCP9600_TYPE_K
#endif

// ROCKSAT
#if MISSION_ID == MISSION_ROCKSAT
  #define NODE_ADDRESS NODE_ADDRESS_KREPE2_006

  #define TC_1_TYPE MCP9600_TYPE_K
  #define TC_2_TYPE MCP9600_TYPE_K
  #define TC_3_TYPE MCP9600_TYPE_K
  #define TC_4_TYPE MCP9600_TYPE_K
  #define TC_5_TYPE MCP9600_TYPE_K
  #define TC_6_TYPE MCP9600_TYPE_K

  #define PRS1_ADDRESS          0x38  // i2c address
  #define PRS1_MAX              103.4 //kpa
  #define PRS2_ADDRESS          0x38
  #define PRS2_MAX              103.4 //kpa
  #define PRS3_ADDRESS          0x78
  #define PRS3_MAX              160.0 //kpa
  #define PRS4_ADDRESS          0x38
  #define PRS4_MAX              103.4 //kpa
  #define PRS5_ADDRESS          0x38
  #define PRS5_MAX              103.4 //kpa
#endif

// the logfilename to use in the format [A-Z]{3}[0-9]{2}.CSV
// see https://regex101.com/
#define LOGFILE_NAME              "LG000.DAT"
#define LOGFILE_NAME_LENGTH 10 // including null terminator

// pi buffer configuration
// 5 lines of 200 chars each max string length
#define  PI_BUFFER_LINES 3
#define  PI_LINE_SIZE    200

// radio buffer configuration
#define RADIO_TX_BUFSIZE 1024
#define RADIO_RX_BUFSIZE 512

// header info has definitions of packet types and sizes for reference when decoding
#define LOGBUF_HEADER_SIZE 2048

// log buffer size in bytes (how many to accumulate before a write)
#define LOGBUF_BLOCK_SIZE         4096              // 32768 / 32
#define LOGBUF_FULL_SIZE    LOGBUF_BLOCK_SIZE - 1960 // compressed iridium packet gauranteed to fit

// for debug radio
#define TLM_SEND_PERIOD   5000 // in scheduler ticks (should be 1ms)
#define RX_TIMEOUT_PERIOD 500  // also in scheduler ticks
#define RBUF_SIZE 260
#define SBUF_SIZE 240

#define LOGID_TMP             0
#define LOGID_PRS             1
#define LOGID_GGA             2
#define LOGID_RMC             3
#define LOGID_BAR             4
#define LOGID_ACC             5
#define LOGID_IMU             6

#define ERR_BOOT              0
#define ERR_2                 1
#define ERR_3                 2
#define ERR_4                 3
#define ERR_SD_BUSY           -1
#define OK                    123

#define I2CMUX_ADDR (0x70) 


#endif
