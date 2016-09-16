/*  
 *  Arduino Geiger Counter - Radiation Sensor Board + RTC + EEPROM storage (edisk)
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
#include <assert.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <EEPROM.h>

#include "clickmeter.h"


////////// USER CONFIG ///////////
//
#define DEBUG_LOG

#ifdef DEBUG_LOG
  #define LOG_ACTIONS           // Serial-log commands send over serial (good).
  #define LOG_BOOT_CONFIG       // Serial-log config vars on reset.
//  #define LOG_REC_VISIT         // Serial-log each rec traversed (much stuff).
  #define LOG_NEW_REC           // Serial-log values of each new rec.
  #define LOG_EDISK_INTERVAL    // Serial-log entrance to long-loop.
#endif

//#define REC_DISABLED          // Do not actually write EEPROM (norrmaly off).

#define STATS_DELAY_SEC         10L   // Short, to detect "max" CPMs & update LCD.
#define EDISK_DELAY_SEC         1500L // Longer, 600-->10min: ~1 day fits in edisk(EEPROM).

/** 
 * The buffer of click timestamps is used 
 * to derive `maxCPM` among those number of elements every `STATS_DELAY_SEC`.
 */
#define     NCLICK_TIMES          5   // Size of `maxCPM` timestamp-buffer.
/** Threshold `maxCPM` values for the led bar (len==NLEDS). */
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
ulong lastStatsMs               = 0;
ulong lastEdiskMs               = 0;
bool is_recording               = false;

volatile uint clicks            = 0;
volatile int maxCPM             = 0.0;

/** 
 * The `maxCPM` click-timings buffer.
 */
ulong clickTimesBuffer[NCLICK_TIMES] = {0};
int nextClickIx = 0;

int EDISK_nextIx                = -1;   // Index in EEPROM to write next rec (avoid scan every time).
int EDISC_nrecs_saved           = 0;
//
//////////////////////////////////


const ulong  STATS_DELAY_MSEC   = STATS_DELAY_SEC * 1000L;
const ulong  EDISK_DELAY_MSEC   = EDISK_DELAY_SEC * 1000L;

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
  return (sec - MYEPOCH_sec) / EDISK_DELAY_SEC; 
}

ulong _decompress_time(uint tenmins) {
  return MYEPOCH_sec + tenmins * EDISK_DELAY_SEC; 
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

byte _Rec_crc(const Rec &rec) {
  return _crc8((byte *)&rec, sizeof(Rec) - 1);
}


void _Rec_seal(Rec &rec, const DateTime rnow) {
    rec.tmstmp = _compress_time(rnow.unixtime());
    rec.crc = _Rec_crc(rec);
}


bool _Rec_is_valid(Rec &rec) {
  int crc = _Rec_crc(rec);
    #ifdef LOG_REC_VISIT
      Serial << F("  calced_crc=") << crc << endl;
    #endif  
  return bool(rec.crc == crc);
}




// Libellium LCD pin numbers.
LiquidCrystal lcd(3,4,5,6,7,8);
RTC_DS1307 rtc;




const int EDSIK_is_rec_eix = EEPROM.length() - 1;

void EDISK_rec_flip() {
  is_recording = 1 ^ EEPROM.read(EDSIK_is_rec_eix);
  EEPROM.write(EDSIK_is_rec_eix, is_recording); 
}

int _EDISK_next_eix(int eix) {
    eix += sizeof(Rec);
    if (eix >= EEPROM.length())
      eix = 0;
    return eix;
}

/**
 * Loops around all EDISK-recs untill and invalid CRC met or reach start point.
 * 
 * :param start_ix:         index into eeprom to start traversing from, 0-->EEPROM.length()-1
 * :param recHandler_func:  a function receiving each rec and return `false` to break traversal.
 * :return:                 the index of the 1st invalid rec met
 */
int EDISK_traverse(const int start_ix = INT_MAX, bool (*recHandler_func)(Rec&) = NULL) {
  
  int eix = (start_ix < 0 || start_ix >= EEPROM.length())? EDISK_nextIx : start_ix;

  if (!recHandler_func)
    recHandler_func = _Rec_is_valid;
    
  Rec rec;
  do {
    EEPROM.get(eix, rec);
    #ifdef LOG_REC_VISIT
      Serial << F("EDISK: visit_eix=") << eix << F(", r.crc=") << rec.crc << endl;
    #endif
    
    if (!recHandler_func(rec))
      return eix;
    
    eix = _EDISK_next_eix(eix);
  } while (eix != start_ix);

  // Reaching here means all Recs were valid:
  //  --> either BAD EDISK 
  //  --> or BAD algo/handler.
  #ifdef LOG_REC_VISIT
    Serial << F("EDISK: Looped around eix: ") << eix << endl;
  #endif  

  return eix; // Signal error.
}


void EDISK_append_rec() {
    int eix = EDISK_nextIx;

    Rec rec;
    rec.clicks = clicks;
    rec.maxCPM = maxCPM;
    _Rec_seal(rec, rtc.now());
    
    #ifdef LOG_NEW_REC
      Serial << F("EDISK append_eix: ") << eix;
      Serial << F(",tmstmp=") << rec.tmstmp << F(",clicks=") << rec.clicks << F(",maxCPM=") << rec.maxCPM ;
      Serial << F(",CRC=") << rec.crc << endl;
    #endif
    
    #ifndef REC_DISABLED
      EEPROM.put(eix, rec);  
    #endif
    
    EDISK_nextIx = _EDISK_next_eix(eix);
    
    #ifndef REC_DISABLED
      EEPROM.write(EDISK_nextIx, 0); // Probably breaks next CRC...
    #endif
    EDISC_nrecs_saved++;
}


void EDISK_clear() {
  EEPROM.write(0, 0); // Probably breaks CRC...
  EDISK_nextIx = 0;

}

bool send_rec(Rec &rec) {
  if (_Rec_is_valid(rec))
    Serial << _decompress_time(rec.tmstmp) << F(", ") << rec.clicks << F(", ") << rec.maxCPM << endl;
  return true;
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

  // Flip REC on boot, to allow to control it.
  EDISK_rec_flip();

  lcd.clear();
  lcd <<  __DATE__ ;  
  lcd.setCursor(0,1);
  lcd << __TIME__;
  delay(1700);

  // Re-flip REC to remain the same, unless reset.
  EDISK_rec_flip();
  EDISK_nextIx = EDISK_traverse(0);

  Serial << F("PROG: " __DATE__ ", " __TIME__) << endl;
  init_RTC();
  
  Serial << F("RTC: ");
  send_rtc();
  Serial << endl;

  #ifdef LOG_BOOT_CONFIG
    send_config();
    send_state(millis());
  #endif
  Serial << F("CLICKS, MaxCPM" STR(NCLICK_TIMES)) << endl;
  update_stats(0);

  lastStatsMs = lastEdiskMs = millis();
  attachInterrupt(0,INT_countPulseclicks,FALLING);
}



void update_stats(int send_serial) {
    if (send_serial) {
      Serial << clicks << ',' << maxCPM << endl;
    }
    
    lcd.clear();    

    lcd << "C10S=" << clicks;
    lcd.setCursor(9, 0);
    lcd << "Rec=";
    if (is_recording)
      lcd << int(EEPROM.length() / sizeof(Rec))- EDISC_nrecs_saved;
    else
      lcd << F("_");
    lcd.setCursor(0, 1);
    lcd << "CPM" STR(NCLICK_TIMES) "=" << maxCPM;
}


void send_config() {
  Serial << F("StatsDelayMs=") << STATS_DELAY_MSEC << ",";
  Serial << F("EdiskDelayMs=") << EDISK_DELAY_MSEC << ",";
  Serial << F("NClickTimes=") << NCLICK_TIMES << ",";
  Serial << F("LedsThresh=[");
  for (int i = 0; i < NLEDS; i++)
    Serial << LED_BAR_THRESH[i] << F(",");
  Serial << F("]\n");
}

void send_state(ulong now) {
  Serial << F("StatsETA=") << (STATS_DELAY_MSEC - now + lastStatsMs) << F(",");
  Serial << F("EdiskETA=") << (EDISK_DELAY_MSEC - now + lastEdiskMs) << F(",");
  Serial << F("isRec=") << is_recording << F(",");
  Serial << F("clicks=") << clicks << F(",");
  Serial << F("maxCPM=") << maxCPM << F(",");
  Serial << F("EDISK_nextIx=") << EDISK_nextIx << F(",");
  Serial << F("\n  clickTimes=[");
  for (int i = 0; i < NCLICK_TIMES; i++)
    Serial << (now - clickTimesBuffer[i]) << F(",");
  Serial << F("]\n");
}



void read_keys(ulong now) {
  char inp = Serial.read();
  if (inp == 'R') {
    EDISK_rec_flip();
    #ifdef LOG_ACTIONS
      Serial << F("REC ") << (is_recording? F("STARTED...") : F("STOPPED.")) << endl;
    #endif
    lcd.clear();
    lcd << F("REC ") << (is_recording? F("STARTED...") : F("STOPPED."));
  
  
  } else if (inp == 'C') {
    #ifdef LOG_ACTIONS
      Serial << F("Clearing EDISK!") << endl;
    #endif
    EDISK_clear();
    lcd.clear();
    lcd << F("EDISK cleared!");
    
  
  } else if ( (inp | 1<<5) == 's') {
    send_config();
    send_state(now);
    
  } else if ( (inp | 1<<5) == 'p') {
    // prints EDISK to serial.
    
    Serial << F("Recorded data::\ntime,clicks,maxCPM") << endl;
    EDISK_traverse(EDISK_nextIx, send_rec);
    
  } else if ( (inp | 1<<5) == 'h') {
    Serial << F("R - > Record on/off!\nC -> Clear store(!)\ns -> log Status\n<p> -> Play recording\n");
    EDISK_clear();
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
  if (now - lastStatsMs > STATS_DELAY_MSEC) {
    lastStatsMs = now;
    update_stats(1);
    clicks = maxCPM = 0;
  } // short loop


  // LONG loop
  //
  if (now - lastEdiskMs > EDISK_DELAY_MSEC) {
    lastEdiskMs = now;
    #ifdef LOG_EDISK_INTERVAL
       Serial << F("Long loop: REC=") << is_recording << endl; 
    #endif
    if (is_recording) {
      EDISK_append_rec();
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

