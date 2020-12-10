#ifndef COMMANDS_H
#define COMMANDS_H

/****************************
* Commands
*/
#define CMDID_IR_ON             1       // power on iridium
#define CMDID_IR_OFF            2       // power off iridium


// setting log sources
#define CMDID_SLS               3       // set log sources
                                        // each byte of the payload indicates sources to enable
                                        // ex data[] = {1,2,3,4} enables all 4 log sources
#define CMDID_SLP               4       // set log period
                                        // arg byte specifies which log source's interval to set. 
                                        // data[] is interval in milliseconds.
// log source identifiers
#define LOG_SOURCE_TC           1       
#define LOG_SOURCE_TELEM        2
#define LOG_SOURCE_ACC          3
#define LOG_SOURCE_IMU          4



#endif
