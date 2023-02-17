#!/usr/bin/env python3

# file getter for KREPE-2 flight computer for when its in dump mode
# for when SD card is permanently mounted
# Matt Ruffner Feb 2023

import io
import sys
import time
import serial

if len(sys.argv) < 3:
  print("Usage: {} /path/to/serial_port command".format(sys.argv[0]))
  sys.exit(1);

ser = serial.Serial(sys.argv[1], 115200, timeout=0.5)  # open serial port
cmd = sys.argv[2]

#if len(sys.argv) < 4:
#  print("using stdout for bytes")

#print("command is {}".format(cmd))

data = bytes()

if ser.is_open:
  
  ser.write(bytes("{}\r\n".format(cmd), 'utf-8'))     # write a string
  
  while True:
    chunk = ser.read(100)
    if len(chunk)==0:
      break
    data += chunk


sys.stdout.buffer.write(data[:-2])
ser.close()

