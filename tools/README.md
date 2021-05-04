## KREPE Tools

These utilities for decompressing and decoding binary packets from the Iridium satellite network are written and tested on an Ubuntu 20.04 machine.

### Installation

To plot the data in a set of Iridium packets, first run the build script with ```sh build.sh``` which creates the directory ```bin``` and compiles the two helper programs. The necessary source for these two programs is in the ```src/``` directory and only rely on standard C++ header libraries.

To use the plotting utilility, a working python3 environment is needed with several common data analysis packages installed including:
- numpy
- matplotlib
- scipy (for median filtering implementation)

### Decompressing and decoding packets

The two binaries that are built, ```binlog_to_csv``` and ```decompress``` can be used to produce a CSV of the data in the Iridium packet.

First, the ```decompress``` tool must be used such as in 

```
==> ./bin/decompress infile.sbd outfile.sbd.dat
```

with an infile and outfile as mandatory arguments. Then the ```binlog_to_csv``` tool can be used to decode the packets according to the definitions in	```src/packets.h``` within the ```tools``` directory such as in

```
==> ./bin/binlog_to_csv decompressed_packet.dat
```

### Usage

For convenience,  the plotting script ```plot_tc.py``` can be run. This tool has the syntax of

```python3 plot_tc.py packet1.sbd [packet2.sbd packet3.sbd ... packetN.sbd]```

where packet1.sbd is a required argument and passing more packet filenames, separated by spaces, is optional and will also plot the data contained in them.

Currently this tool must be run from the ```tools/``` directory so that the plotting script knows where the binaries for the tools are.

#### Output

Because this tool currently only supports (assumes) that input packets contain all thermocouple data, a 5 sample median filter to  help eliminate sample jitter is used on all TC channels.