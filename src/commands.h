#ifndef COMMANDS_H
#define COMMANDS_H

/************************************************************************************************
* Commands
*/
// power switch for iridium and GPS mosfet,
#define CMDID_5VPWR_ON          41       // power on iridium
#define CMDID_5VPWR_OFF         42       // power off iridium

#define CMDID_IR_BP             43       // build packet using internal packet building thread
#define CMDID_IR_SP             44       // send locally source all natural grassfed packet

#define CMDID_PI_BP             45       // trigger asking the neoPi to build a packet for us
#define CMDID_PI_SP             46        // ask the iridium modem to send a pi-build packet

#define CMDID_SET_IMU_PER       50
#define CMDID_SET_TC_PER        51
#define CMDID_SET_PRS_PER       52

#define CMDID_RADLOGON_TC         60
#define CMDID_RADLOGOFF_TC        61

#define CMDID_RADLOGON_IMU        62
#define CMDID_RADLOGOFF_IMU       63

#define CMDID_RADLOGON_PRS        64
#define CMDID_RADLOGOFF_PRS       65

#define CMDID_RADLOGON_QUAT       66
#define CMDID_RADLOGOFF_QUAT      67

#define CMDID_RADLOGON_GGA        68
#define CMDID_RADLOGON_RMC        69

#define CMDID_RADLOGOFF_GGA       70
#define CMDID_RADLOGOFF_RMC       71

#define CMDID_RADLOGON_ACC       72
#define CMDID_RADLOGOFF_ACC       73

// for the following commands,
// 5VPWR must be enabled for PIPWR and BSMSPWR commands to have effect

// commands for controlling the NanoPi 5v switch
#define CMDID_PIPWR_ON          80
#define CMDID_PIPWR_OFF         81
// for BSMS 5v switch
#define CMDID_BSMSPWR_ON        82
#define CMDID_BSMSPWR_OFF       83

#endif
