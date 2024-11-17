## ISS Software

Software supporting the [KREPE Flight Computer (KFC)](https://github.com/krups/iss-hardware) ISS 2023 Flight (KREPE-2)

### Firmware 
Main flight computer (Feather M4 processor) firmware is in ```KREPE2Firmware``` folder.

Ground station firmware (Feather M0) processor is in ```GroundstationFirmware``` folder.

Spectrometer subsystem (FeatherM0/M4) firmware is in ```BSMSFirmware``` folder.

Shared code and parameters between these three firmwares is in ```src/``` directory.


### Data Analysis
Tools to decompress received iridium packets and convert them from binary format to CSV format are in the ```tools/``` directory.
