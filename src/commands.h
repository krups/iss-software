#ifndef COMMANDS_H
#define COMMANDS_H

/************************************************************************************************
* Commands
*/
// typically sent from station -> capsule
#define CMDID_IR_ON             1       // power on iridium
#define CMDID_IR_OFF            2       // power off iridium
#define CMDID_IR_BP             8       // build packet
#define CMDID_IR_SP             9       // send packet
#define CMDID_IR_TEST           10      // send a "hello world" packet

// typicall sent from capsule -> station
#define CMDID_OK                3       // acknowledge command for an ack packet or keepalive 
#define CMDID_ACT_DETECTED      4       // temp and cap sense have detected activation 



// setting log sources
#define CMDID_LOG_SOURCE        3       // set log sources
                                        // each byte of the payload indicates sources to enable
                                        // ex data[] = {1,2,3,4} enables all 4 log sources
#define CMDID_LOG_PERIOD        4       // set log period
                                        // arg byte specifies which log source's interval to set. 
                                        // data[] is interval in milliseconds.


// queue length for command receiving thread
#define CMD_QUEUE_LENGTH        20      // max command packets waiting to be processed by command thread


#endif
