## ISS Software

Software supporting the [KREPE Flight Computer (KFC)](https://github.com/krups/iss-hardware) v1.1 for the ISS 2021 Flight.

### Structure
The main arduino sketch is ```hardware_tests/IridiumThreadTest/IridiumThreadTest.ino```. Supporting code is in ```src```. If not in a unix like environment, the symlink to ```src``` in the sketch folder must be replaced by the actual folder. This needs to be fixed in future updates to this repository.

Configuration parameters for the three capsules can be set in ```src/config.h```

### Data Analysis
Tools to decompress received iridium packets and convert them from binary format to CSV format are in the ```tools/``` directory.
