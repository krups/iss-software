
#ifndef SDLOGGER
#define SDLOGGER

#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <TeensyThreads.h>

#define PIN_SD_CS  BUILTIN_SDCARD

#define MAX_OPEN_FILES 5

extern void safePrintln(String s);
extern void safePrint(String s);

class SDLogger {

public:
  SDLogger() : _ready(false), _logNum(0), _openFileCount(0) {
    
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
    EEPROM.update(0, _logNum+1);

    safePrint("-----> log # "); safePrintln(_logNum);
  }
  
  int createLog(String logtag) {
    if( !_ready ){
      safePrintln("SD not ready, please call begin()");
      return -1;
    }
  
    // only allow a few files open
    if( _openFileCount >= MAX_OPEN_FILES ){
      return -1;
    }
    
    // create filename and try to open file
    String fname;
    fname += (logtag + String(_logNum) + ".txt");

    safePrint("filename is '");safePrint(fname);safePrintln("'");
    
    safePrint(("storing logfile entry: " + fname)); safePrintln(", file not yet created");
    _fh = SD.open(fname.c_str(), FILE_WRITE);
    if( _fh ) {
      _fnames[_openFileCount] = fname;
      safePrintln("  ok. opened log file: " + fname);
      _fh.close();
      _openFileCount += 1;
      return _openFileCount - 1;
    } else {
      safePrintln("  err. couldnt open file: " + fname);
      return -1;
    }
  }
  
  byte logMsg(int id, String s) {
    if( !_ready ){
      safePrintln("SD card not initialized");
      return 0;
    }
    safePrint("trying to open logfile #"); safePrint(String(id)); safePrint(" with filename "); safePrintln(_fnames[id]);
  
    _fh = SD.open(_fnames[id].c_str(), FILE_WRITE);
  
    if( _fh ){
      safePrint("ok. openend file "); safePrintln(_fnames[id]);
      safePrint("  file has "); safePrint(_fh.size()); safePrint(" bytes in it, seeking to end");
      
      _fh.seek(_fh.size());
      byte ret = _fh.println(s);
      _fh.close();
      return ret;
  
    } else {
      safePrint("error: could not re-open file "); safePrint(_fnames[id]); safePrintln(" for logging");
      return 0;
    }      
  }
  //int logData(int id, Packet p) {}
  
  bool isReady() { return _ready; }

private:
  bool _ready, _open[MAX_OPEN_FILES];
  int _logNum;
  
  int _openFileCount;
  
  String _fnames[MAX_OPEN_FILES];
  
  File _fh;

};

#endif
