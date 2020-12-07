/*
* KREPE ISS Mission
* Matt Ruffner Dec 2020
*
* SDLogger supports writing to multiple files, referenced by a file ID.
* 
*/

#ifndef SDLOGGER
#define SDLOGGER

#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <TeensyThreads.h>

#include "config.h"
#include "packets.h"

#define PIN_SD_CS  BUILTIN_SDCARD

#define MAX_OPEN_FILES 5

extern void safePrintln(String s);
extern void safePrint(String s);

class SDLogger {

public:
  SDLogger() : _ready(false), _logNum(0), _openFileCount(0) {}
  ~SDLogger() {}
  
  /***********************
  * init the SD card and make sure we have one
  * read in the log count from EEPROM, increment and store
  */
  void begin() {
    if (!SD.begin(PIN_SD_CS)) {
      if(USBSERIAL_DEBUG) safePrintln("Card failed, or not present");
      // don't do anything more:
      _ready = false;
      return;
    }
    if(USBSERIAL_DEBUG) safePrintln("card initialized.");
    _ready = true;
    
    for( int i=0; i<MAX_OPEN_FILES; i++ ){
      _open[i] = false;
    }
    
    // read in last log id from memory , sizeof(int)
    EEPROM.get(0, _logNum);
    EEPROM.update(0, _logNum+1);

    if(USBSERIAL_DEBUG) safePrint("-----> log # "); safePrintln(_logNum);
  }
  
  /*******************************
  * create an entry in the current file list, stored by filename at index fileID
  * param logtag: start of filename (appends a rolling number and .txt to the end)
  * returns the fileId to reference this file
  */  
  int createLog(String logtag) {
    if( !_ready ){
      if(USBSERIAL_DEBUG) safePrintln("SD not ready, please call begin()");
      return -1;
    }
  
    // only allow a few files open
    if( _openFileCount >= MAX_OPEN_FILES ){
      return -1;
    }
    
    // create filename and try to open file
    String fname;
    fname += (logtag + String(_logNum) + ".txt");

    if(USBSERIAL_DEBUG) safePrint("filename is '");safePrint(fname);safePrintln("'");
    
    if(USBSERIAL_DEBUG) safePrint(("storing logfile entry: " + fname)); safePrintln(", file not yet created");
    _fh = SD.open(fname.c_str(), FILE_WRITE);
    if( _fh ) {
      _fnames[_openFileCount] = fname;
      if(USBSERIAL_DEBUG) safePrintln("  ok. opened log file: " + fname);
      _fh.close();
      _openFileCount += 1;
      return _openFileCount - 1;
    } else {
      if(USBSERIAL_DEBUG) safePrintln("  err. couldnt open file: " + fname);
      return -1;
    }
  }
  
  /******************************
  * for logging text
  */
  byte logMsg(int id, String s) {
    // the bouncer
    if( !_idValid(id) ) return 0;
    
    if(USBSERIAL_DEBUG) safePrint("trying to open logfile #"); safePrint(String(id)); safePrint(" with filename "); safePrintln(_fnames[id]);
  
    _fh = SD.open(_fnames[id].c_str(), FILE_WRITE);
  
    if( _fh ){
      //safePrint("ok. openend file "); safePrintln(_fnames[id]);
      //safePrint("  file has "); safePrint(_fh.size()); safePrint(" bytes in it, seeking to end");
      
      _fh.seek(_fh.size());
      byte ret = _fh.println(s);
      _fh.close();
      return ret;
  
    } else {
      if(USBSERIAL_DEBUG) safePrint("error: could not re-open file "); safePrint(_fnames[id]); safePrintln(" for logging");
      return 0;
    }      
  }
  
  /*******************************
  * for logging packets in binary format:
  * [ packet type (1 byte)     ]
  * [ packet len  (2 bytes)    ]
  * [ packet data (len bytes) ]
  */
  byte logBin(int id, Packet *p) {
    // bounce
    if( !_idValid(id) ) return 0;
    
    if(USBSERIAL_DEBUG) safePrint("trying to open logfile #"); safePrint(String(id)); safePrint(" with filename "); safePrintln(_fnames[id]);
  
    // open file referenced by id
    _fh = SD.open(_fnames[id].c_str(), FILE_WRITE);
  
    // if file opened
    if( _fh ){
      //safePrint("ok. openend file "); safePrintln(_fnames[id]);
      //safePrint("  file has "); safePrint(_fh.size()); safePrint(" bytes in it, seeking to end");
      
      // seek to end of file
      _fh.seek(_fh.size());
      
      // write packet data
      byte ret = _fh.write(p->data(), p->size());
      _fh.close(); //close file
      return ret; // return the number of bytes written. TODO: make sure this is equal to the number attempted to write.
  
    } else {
      if(USBSERIAL_DEBUG) safePrint("error: could not re-open file "); safePrint(_fnames[id]); safePrintln(" for logging");
      return 0;
    }   
  
  }
  
  // Take uniform sample with number of bytes = `size` from file with id=`id`
  // Put result in sam_buf
  void sample(int id, byte* sam_buf, unsigned long size){
    // open file referenced by id
    _fh = SD.open(_fnames[id].c_str(), FILE_WRITE);
    char type_buf[1];
    _fh.read(type_buf, 1);
    _fh.seek(0);
    unsigned long packet_size;
    switch (type_buf[0])
    {
    case 'M':
      packet_size = TELEM_T_SIZE;
      break;
    case 'A':
      packet_size = ACC_STAT_T_SIZE;
      break;
    case 'T':
      packet_size = TC_T_SIZE;
      break;
    case 'B':
      packet_size = ACC_T_SIZE;
      break;
    case 'I':
      //packet_size = IMU
      break;
    default:
      if(USBSERIAL_DEBUG) safePrint(_fnames[id]); safePrintln(" has an unexpected layout.");
      return;
      break;
    }
    unsigned long num_samples_needed = size/packet_size;
    unsigned long num_packets_available = _fh.size()/packet_size;
    unsigned long interval = num_packets_available/num_samples_needed;
    for(int i = 0; i< num_samples_needed; i = i + interval){
      _fh.read(sam_buf[i*packet_size], packet_size);
    }

  }
  bool isReady() { return _ready; }

private:
  // parameter validation for the ID corresponding to the the file to write to
  bool _idValid(int id) {
    if( !_ready ){
      if(USBSERIAL_DEBUG) safePrintln("SD card not initialized");
      return false;
    }
    if( (id >= _openFileCount) || (id < 0) ){
      if(USBSERIAL_DEBUG) safePrintln("invalid file index");
      return false;
    }
    return true;
  }

  bool _ready, _open[MAX_OPEN_FILES];
  int _logNum;
  int _openFileCount;
  
  String _fnames[MAX_OPEN_FILES];
  File _fh;

};

#endif
