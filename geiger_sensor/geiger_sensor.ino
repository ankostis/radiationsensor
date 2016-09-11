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


#define DEBUG

#ifdef DEBUG
  #define D_NEXT_REC
  #define D_NEW_REC
  #define D_LONG_LOOP
#endif

const long SHORT_LOOP_MSEC  = 10 * 1000;
const long LONG_LOOP_MSEC   = SHORT_LOOP_MSEC * 60;

/** 
 * A circulare-buffer of click timings 
 * used to derive `maxCPM`.
 */
#define NMAX_CPM 5
unsigned long lastClickMillis[NMAX_CPM] = {0};
int nextClickIx = 0;

/**
 * `maxCPM` threshold values for the led bar.
 */
const int NLEDS = 5;
const int LED_THRESH[] = {30, 70, 150, 350, 800};

// Trick from http://stackoverflow.com/questions/5459868/c-preprocessor-concatenate-int-to-string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)


// J305βγ Conversion factor - CPM to uSV/h
const float CONV_FACTOR = 0.00812;
const int GEIGER_PIN = 2;
const int LED_ARRAY [] = {10,11,12,13,9};

//////////// GLOBALS /////////////
unsigned long shortLoopMillis = 0;
unsigned long longLoopMillis = 0;
bool is_recording = false;

volatile int clicks = 0;
volatile float maxCPM = 0.0;
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
const long MYEPOCH_sec = MYEPOCH.unixtime();


unsigned int _compress_time(long sec) {
  //Serial << "TCOMP: " << sec << ", " << MYEPOCH_sec << ", " <<  ((sec - MYEPOCH_sec) /60/10) << endl;
  return (sec - MYEPOCH_sec) / 60 / 10; 
}

long _decompress_time(unsigned int tenmins) {
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
 * :param recHandler_func: a function to handle each valid rec
 */
int STORAGE_traverse(void (*recHandler_func)(Rec) = NULL) {
  int indx = 0;
  Rec rec;
  for(; indx < EEPROM.length(); indx += sizeof(Rec)) {
    EEPROM.get(indx, rec);
    int crc = _crc_rec(rec);
    #ifdef D_NEXT_REC
      Serial << F("Check EEPROM: indx=") << indx << F(", crc=") << crc << F(", r.crc=") << rec.crc << endl;
    #endif
    if (rec.crc != crc) 
      break;
    if (recHandler_func) {
      recHandler_func(rec);
    }
  }
  indx &= EEPROM.length() - 1;

  return indx;
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
    rec.clicks = int(clicks);
    rec.maxCPM = int(maxCPM);
    _seal_rec(rec);
    
    #ifdef D_NEW_REC
      Serial << F("tmstmp, clicks, maxCPM, crc\n");
      Serial << 0 << "," << clicks << "," << maxCPM  << "," << 0 << endl;
      Serial << rec.tmstmp << "," << rec.clicks << "," << rec.maxCPM  << "," << rec.crc << endl;
    #endif D_NEW_REC
    //EEPROM.put(eix, rec);  
    //EEPROM.write(eix + sizeof(Rec) + 1, 0); // Just to break next CRC.
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
    
    Serial << F("RTC: ") << now.year() << '/' << now.month() << '/' << now.day();
    Serial << F("-") << now.hour() << ':' << now.minute() << ':' << now.second() << ',';  
  }
}


void setup(){
  pinMode(GEIGER_PIN, INPUT);
  digitalWrite(GEIGER_PIN,HIGH);
  for (int i=0;i<5;i++){
    pinMode(LED_ARRAY[i],OUTPUT);
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
  send_rtc();
  Serial << endl;
  Serial << F("CLICKS, MaxCPM" STR(NMAX_CPM)) << endl;
  update_stats(0);

  shortLoopMillis = longLoopMillis = millis();
  attachInterrupt(0,INT_countPulseclicks,FALLING);
}



void update_stats(int send_serial) {
    if (send_serial) {
      Serial << clicks << ',' << _FLOAT(maxCPM, 4) << endl;
    }
    
    lcd.clear();    

    lcd << "C10Sec=" << clicks;
    lcd.setCursor(11, 0);
    lcd << "Rec=" << is_recording;
    lcd.setCursor(0, 1);
    lcd << "MaxCPM" STR(NMAX_CPM) "=" << _FLOAT(maxCPM, 4);
}


void read_keys() {
  char inp = Serial.read();
  if (inp == 'R') {
    is_recording ^= true;       
    #ifdef DEBUG
      Serial << F("REC ") << (is_recording? F("STARTED...") : F("STOPPED.")) << endl;
    #endif
    lcd.clear();
    lcd << F("REC ") << (is_recording? F("STARTED...") : F("STOPPED.")) << endl;
  
  
  } else if (inp == 'C') {
    #ifdef DEBUG
      Serial << F("Clearing Storage!") << endl;
    #endif
    STORAGE_clear();
    lcd.clear();
    lcd << F("Storage cleared!");
    
  
  } else if ( (inp | 1<<5) == 'h') {
    #ifdef DEBUG
      Serial << F("R - > Record on/off!\nC -> Clear store\n<any> -> Print store(!)\n");
    #endif
    STORAGE_clear();
    lcd.clear();
    lcd << F("R:rec1/0 C:clear");
    lcd.setCursor(0, 1);
    lcd << F("<any>:print store");

  
  } else if (inp > 0) {
    // Any key prints storage to serial.
    
    Serial << F("time,clicks,maxCPM") << endl;
    STORAGE_traverse(send_rec);
  }
}


void loop(){
  long now = millis();

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
    #ifdef D_LONG_LOOP
       Serial << F("Long loop: REC=") << is_recording << endl; 
    #endif
    if (is_recording) {
      STORAGE_append_rec();
    }
  } // long loop


  read_keys();
}



void update_leds(float CPM) {
  //led var setting
  int i = 0;
  for(int i=0; i < NLEDS; i++) {
    const int state = (CPM > LED_THRESH[i])? HIGH: LOW;
    digitalWrite(LED_ARRAY[i], state);
  }
}


/**
 * Interupt geiger routine.
 */
void INT_countPulseclicks(){
  detachInterrupt(0);
  long now = millis();
  
  clicks++;
  
  lastClickMillis[nextClickIx] = now;
  nextClickIx++;
  if (nextClickIx == NMAX_CPM) {
    nextClickIx = 0;
  }
  
  float CPM = NMAX_CPM * 60000.0 / (now - lastClickMillis[nextClickIx]);
  update_leds(CPM);
  
  if (maxCPM < CPM)
    maxCPM = CPM;
  
  while(digitalRead(2)==0);
  attachInterrupt(0,INT_countPulseclicks,FALLING);
}

