
#ifndef SDLOGGER
#define SDLOGGER

#include <SD.h>
#include <EEPROM.h>
#include <vector>

#define PIN_SD_CS  BUILTIN_SDCARD

#define MAX_OPEN_FILES 5

extern void safePrintln(String s);
extern void safePrint(String s);

class SDLogger {

public:
  SDLogger() : _ready(false), _logNum(0) {
    
  }
  ~SDLogger() {}
  
  void begin() {
    if (!SD.begin(PIN_SD_CS)) {
      safePrintln("Card failed, or not present");
      // don't do anything more:
      _ready = false;
      return;
    }
    safePrintln("card initialized.");
    _ready = true;
    
    for( int i=0; i<MAX_OPEN_FILES; i++ ){
      _open[i] = false;
    }
    
    // read in last log id from memory , sizeof(int)
    EEPROM.get(0, _logNum);    
  }
  
  int open(String logtag) {
    // only allow a few files open
    if( _openFileCount >= MAX_OPEN_FILES ){
      return -1;
    }
    
    // create filename and try to open file
    String fname;
    fname += logtag + "_" + String(_logNum);
    _fh[_openFileCount] = SD.open(fname.c_str(), FILE_WRITE);
    
    // check if we could open the file
    // if we can, return the file reference ID to pass back when logging
    if ( _fh[_openFileCount] ) {
      _open[_openFileCount-1] = true;
      safePrintln("opened log file " + fname);
      _logNum += 1;
      EEPROM.update(0, _logNum);
      _openFileCount += 1;
      return _openFileCount-1;
    }  
    
    // return -1 if we cant open
    else {
      safePrintln("couldn't open file");
      return -1;
    } 
  }
  
  void close(int id) {
    if( _open[id] ){
      _fh[id].close();
      _openFileCount -= 1;
    }
  }
  
  int logMsg(int id, String s) {}
  //int logData(int id, Packet p) {}
  
  bool isOpen(uint8_t id) { return _open[id]; }
  bool isReady() { return _ready; }

private:
  bool _ready, _open[MAX_OPEN_FILES];
  int _logNum;
  
  int _openFileCount;
  
  
  File _fh[MAX_OPEN_FILES];

};

#endif

