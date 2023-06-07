#ifndef COMMANDS_H
#define COMMANDS_H

/************************************************************************************************
* Commands
* sent from the groundstation to a capsule, groundstation does not listen for commands
*/

// power switch for iridium and GPS mosfet,
#define CMDID_5VPWR_ON          41       // power on iridium
#define CMDID_5VPWR_OFF         42       // power off iridium

#define CMDID_3VPWR_ON          39       // external 3v3 power on
#define CMDID_3VPWR_OFF         40       // external 3v3 power off

#define CMDID_IR_BP             43       // build packet using internal packet building thread
#define CMDID_IR_SP             44       // send locally source all natural grassfed packet

#define CMDID_PI_BP             45       // trigger asking the neoPi to build a packet for us
#define CMDID_PI_SP             46        // ask the iridium modem to send a pi-build packet

#define CMDID_AB_INTERNAL_ON    47       // set autobuild internal packets to true
#define CMDID_AB_INTERNAL_OFF   48       
#define CMDID_AB_INTERNAL_PER   49       // set autobuild internal period

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

#define CMDID_RADLOGON_ACC        72
#define CMDID_RADLOGOFF_ACC       73

#define CMDID_RADLOGON_SPEC       74
#define CMDID_RADLOGOFF_SPEC      75

#define CMDID_SPOFF               76
#define CMDID_SPON                77


// operations on SD card
#define CMDID_LS                  100
#define CMDID_WIRELESSDUMP        101
#define CMDID_CLEAR               102

#endif
