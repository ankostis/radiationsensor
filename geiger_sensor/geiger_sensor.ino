/*  
 *  Arduino Geiger Counter - Radiation Sensor Board + RTC + EEPROM storage
 *  
 *  Copyright (C) ankostis@gmail.com & Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see http://www.gnu.org/licenses/. 
 */

#include <Arduino.h>
#include <Streaming.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <EEPROM.h>

#include "recstorage.h"


////////// USER CONFIG ///////////
//
#define DEBUG

#ifdef DEBUG
  #define LOG_ACTIONS       // Serial-log commands send over serial (good).
  #define LOG_BOOT_CONFIG   // Serial-log config vars on reset.
  //#define LOG_NEXT_REC    // Serial-log each rec traversed (much stuff).
  //#define LOG_NEW_REC     // Serial-log values of each new rec.
  #define LOG_LONG_LOOP     // Serial-log entrance to long-loop.
  #define REC_DISABLED      // Do not actually write EEPROM.
#endif

const ulong  SHORT_LOOP_MSEC   = 10 * 1000;
const ulong  LONG_LOOP_MSEC    = SHORT_LOOP_MSEC * 60;

/** 
 * A uinte-buffer of click timings 
 * used to derive `maxCPM` among those elements.
 */
#define     NCLICK_TIMES          5   // Size of `maxCPM` timestamp-buffer.
/** Threshold `maxCPM` values for the led bar (==NLEDS). */
const int LED_BAR_THRESH[]      = {30, 70, 150, 350, 800};
//
//////////////////////////////////


const int   LED_BAR_PINS []     = {10,11,12,13,9};
const int   NLEDS               = ARRAYLEN(LED_BAR_PINS);
const int   GEIGER_PIN          = 2;
/** J305βγ Conversion factor [CPM to uSV/h] - unused, for my reference. */
const float SIEVERT_CONV_FACTOR = 0.00812;


//////////// GLOBALS /////////////
//
ulong shortLoopMillis           = 0;
ulong longLoopMillis            = 0;
bool is_recording               = false;

volatile uint clicks            = 0;
volatile int maxCPM           = 0.0;

/** 
 * The `maxCPM uinte-buffer.
 */
ulong clickTimesBuffer[NCLICK_TIMES] = {0};
int nextClickIx = 0;
//
//////////////////////////////////


/**
 * Compress timestamps by storing them as 10-min offsets 
 * from 11-Sep-2016 (MYEPOCH).
 * So int (2^16) timestamps would work in the future for:
 * - 65536 * 6            = 393216 hours
 * - 65536 * 6 / 24       = 16384 days
 * - 65536 * 6 / 24 / 365 = ~44.8 years
 * 
 * That's enough for a humble arduino :-)
 */
 // MYEPOCH expressed from EPOCH(1970/1/1)
const DateTime MYEPOCH(2016, 9, 11);
const ulong MYEPOCH_sec = MYEPOCH.unixtime();


uint _compress_time(ulong sec) {
  //Serial << "TCOMP: " << sec << ", " << MYEPOCH_sec << ", " <<  ((sec - MYEPOCH_sec) /60/10) << endl;
  return (sec - MYEPOCH_sec) / 60 / 10; 
}

ulong _decompress_time(uint tenmins) {
  return MYEPOCH_sec + tenmins * 10 * 60; 
}


/** From: http://www.leonardomiliani.com/en/2013/un-semplice-crc8-per-arduino/
 */
byte _crc8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}


byte _crc_rec(const Rec &rec) {
  return _crc8((byte *)&rec, sizeof(Rec) - 1);
}


// Libellium LCD pin numbers.
LiquidCrystal lcd(3,4,5,6,7,8);
RTC_DS1307 rtc;


/**
 * Traverse all storage-recs untill invalid CRC met.
 * 
 * :param start_ix: index into eeprom to start traversing from, 0-->EEPROM.length()-1
 * :param recHandler_func: a function to handle each valid rec
 * :return: the index of the 1st invalid rec met
 */
int _STORAGE_traverse(const int start_ix, void (*recHandler_func)(Rec)) {
  int indx = (start_ix >= EEPROM.length())? 0 : start_ix;

  Rec rec;
  for(; indx < EEPROM.length(); indx += sizeof(Rec)) {
    EEPROM.get(indx, rec);
    int crc = _crc_rec(rec);
    #ifdef LOG_NEXT_REC
      Serial << F("Check EEPROM: indx=") << indx << F(", crc=") << crc << F(", r.crc=") << rec.crc << endl;
    #endif
    if (rec.crc != crc) 
      break;
    if (recHandler_func) {
      recHandler_func(rec);
    }
  }
  if (indx >= EEPROM.length())
    indx = 0;

  return indx;
}

/**
 * 2-pass traversal of storage-recs to wrap past the last valid rec, 
 * and detect the begining of uint list-of-recs.
 * 
 * See `_STORAGE_traverse()` for params/return value.
 */
int STORAGE_traverse(const int start_ix = 0, void (*recHandler_func)(Rec) = NULL) {
  return _STORAGE_traverse(start_ix, recHandler_func);
}



void _seal_rec(Rec &rec) {
    DateTime rnow = rtc.now();
    rec.tmstmp = _compress_time(rnow.unixtime());
    rec.crc = _crc_rec(rec);
}


void STORAGE_append_rec() {
    int eix = STORAGE_traverse();
    Serial << F("Store rec: ") << eix << endl;

    Rec rec;
    rec.clicks = clicks;
    rec.maxCPM = maxCPM;
    _seal_rec(rec);
    
    #ifdef LOG_NEW_REC
      Serial << F("tmstmp, clicks, maxCPM, crc\n");
      Serial << 0 << "," << clicks << "," << maxCPM  << "," << 0 << endl;
      Serial << rec.tmstmp << "," << rec.clicks << "," << rec.maxCPM  << "," << rec.crc << endl;
    #endif LOG_NEW_REC
    #ifndef REC_DISABLED
      EEPROM.put(eix, rec);  
      EEPROM.write(eix + sizeof(Rec) + 1, 0); // Just to break next CRC.
    #endif
}


void STORAGE_clear() {
  EEPROM.write(0, 0); // Breaks CRC.
}

void send_rec(Rec &rec) {
  Serial << _decompress_time(rec.tmstmp) << F(", ") << rec.clicks << F(", ") << rec.maxCPM << endl;
}



void init_RTC() {
  if (! rtc.begin()) {
      Serial << F("Couldn't find RTC") << endl;
  } else {
    if (! rtc.isrunning()) {
      Serial << F("Initializing RTC time!") << endl;
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
  }
}

void send_rtc() {
  if (rtc.isrunning()) {
    DateTime now = rtc.now();
    
    Serial << now.year() << '/' << now.month() << '/' << now.day();
    Serial << F("-") << now.hour() << ':' << now.minute() << ':' << now.second() << ',';  
  }
}



void setup(){
  pinMode(GEIGER_PIN, INPUT);
  digitalWrite(GEIGER_PIN,HIGH);
  for (int i = 0; i < NLEDS; i++){
    pinMode(LED_BAR_PINS[i],OUTPUT);
  }

  Serial.begin(19200);
  
  //set up the LCD\'s number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  //lcd.setCursor(0, 0);
  lcd << F("Radiation Sensor");
  delay(700);
  for (int i=0;i<10;i++){
    delay(100);  
    lcd.scrollDisplayLeft();
  }

  lcd.clear();
  lcd <<  __DATE__ ;  
  lcd.setCursor(0,1);
  lcd << __TIME__;
  delay(1700);

  Serial << F("PROG: " __DATE__ ", " __TIME__) << endl;
  init_RTC();
  
  Serial << F("RTC: ");
  send_rtc();
  Serial << endl;

  #ifdef LOG_BOOT_CONFIG
    send_config();
  #endif

  Serial << F("CLICKS, MaxCPM" STR(NCLICK_TIMES)) << endl;
  update_stats(0);

  shortLoopMillis = longLoopMillis = millis();
  attachInterrupt(0,INT_countPulseclicks,FALLING);
}



void update_stats(int send_serial) {
    if (send_serial) {
      Serial << clicks << ',' << maxCPM << endl;
    }
    
    lcd.clear();    

    lcd << "C10Sec=" << clicks;
    lcd.setCursor(11, 0);
    lcd << "Rec=" << is_recording;
    lcd.setCursor(0, 1);
    lcd << "MaxCPM" STR(NCLICK_TIMES) "=" << maxCPM;
}


void send_config() {
  Serial << F("ShortMs=") << SHORT_LOOP_MSEC << ",";
  Serial << F("LongNs=") << LONG_LOOP_MSEC << ",";
  Serial << F("NClickTimes=") << NCLICK_TIMES << ",";
  Serial << F("LedThresh=[");
  for (int i = 0; i < NLEDS; i++)
    Serial << LED_BAR_THRESH[i] << F(",");
  Serial << F("]\n");
}

void send_state(ulong now) {
  Serial << F("shortETA=") << (SHORT_LOOP_MSEC - now + shortLoopMillis) << F(",");
  Serial << F("longETA=") << (LONG_LOOP_MSEC - now + longLoopMillis) << F(",");
  Serial << F("isRec=") << is_recording << F(",");
  Serial << F("clicks=") << clicks << F(",");
  Serial << F("maxCPM=") << maxCPM << F(",");
  Serial << F("clickTimes=[");
  for (int i = 0; i < NCLICK_TIMES; i++)
    Serial << (now - clickTimesBuffer[i]) << F(",");
  Serial << F("]\n");
}



void read_keys(ulong now) {
  char inp = Serial.read();
  if (inp == 'R') {
    is_recording ^= true;       
    #ifdef LOG_ACTIONS
      Serial << F("REC ") << (is_recording? F("STARTED...") : F("STOPPED.")) << endl;
    #endif
    lcd.clear();
    lcd << F("REC ") << (is_recording? F("STARTED...") : F("STOPPED."));
  
  
  } else if (inp == 'C') {
    #ifdef LOG_ACTIONS
      Serial << F("Clearing Storage!") << endl;
    #endif
    STORAGE_clear();
    lcd.clear();
    lcd << F("Storage cleared!");
    
  
  } else if ( (inp | 1<<5) == 's') {
    send_config();
    send_state(now);
    
  } else if ( (inp | 1<<5) == 'p') {
    // prints storage to serial.
    
    Serial << F("Recorded data::\ntime,clicks,maxCPM") << endl;
    STORAGE_traverse(0, send_rec);
    
  } else if ( (inp | 1<<5) == 'h') {
    Serial << F("R - > Record on/off!\nC -> Clear store(!)\ns -> log Status\n<p> -> Play recording\n");
    STORAGE_clear();
    lcd.clear();
    lcd << F("R:rec1/0 C:clear");
    lcd.setCursor(0, 1);
    lcd << F("p:play s:status");

  } else if (inp > 0) {
    #ifdef LOG_ACTIONS
      Serial << F("Invalid key! try <h> for help\n");
    #endif
    lcd.clear();
    lcd << F("Try h: Help");
  }
}


void loop(){
  ulong now = millis();

  read_keys(now);

  // SHORT loop
  //
  if (now - shortLoopMillis > SHORT_LOOP_MSEC) {
    shortLoopMillis = now;
    update_stats(1);
    clicks = maxCPM = 0;
  } // short loop


  // LONG loop
  //
  if (now - longLoopMillis > LONG_LOOP_MSEC) {
    longLoopMillis = now;
    #ifdef LOG_LONG_LOOP
       Serial << F("Long loop: REC=") << is_recording << endl; 
    #endif
    if (is_recording) {
      STORAGE_append_rec();
    }
  } // long loop
}



void update_leds(int CPM) {
  //led var setting
  int i = 0;
  for(int i=0; i < NLEDS; i++) {
    const int state = (CPM > LED_BAR_THRESH[i])? HIGH: LOW;
    digitalWrite(LED_BAR_PINS[i], state);
  }
}


/**
 * Interupt geiger routine.
 */
void INT_countPulseclicks(){
  detachInterrupt(0);
  ulong now = millis();
  
  clicks++;
  
  clickTimesBuffer[nextClickIx] = now;
  nextClickIx++;
  if (nextClickIx == NCLICK_TIMES) {
    nextClickIx = 0;
  }
  
  int CPM = int(NCLICK_TIMES * 60000.0 / (now - clickTimesBuffer[nextClickIx]));
  update_leds(CPM);
  
  if (maxCPM < CPM)
    maxCPM = CPM;
  
  while(digitalRead(2)==0);
  attachInterrupt(0,INT_countPulseclicks,FALLING);
}

