#ifndef COMMANDS_H
#define COMMANDS_H

/************************************************************************************************
* Commands
*/
// power switch for iridium and GPS mosfet,
#define CMDID_5VPWR_ON          41       // power on iridium
#define CMDID_5VPWR_OFF         42       // power off iridium

#define CMDID_IR_BP             43       // build packet
#define CMDID_IR_SP             44       // send packet

// 5VPWR must be enabled for PIPWR and BSMSPWR commands to have effect

// commands for controlling the NanoPi 5v switch
#define CMDID_PIPWR_ON          70
#define CMDID_PIPWR_OFF         71
// for BSMS 5v switch
#define CMDID_BSMSPWR_ON        72
#define CMDID_BSMSPWR_OFF       73

#endif
