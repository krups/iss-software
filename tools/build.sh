#!/bin/bash

[ ! -d "bin" ] && mkdir bin

g++ src/binlog_to_csv.cpp -o bin/binlog_to_csv
g++ src/main.cpp src/depacks.c -o bin/decompress



