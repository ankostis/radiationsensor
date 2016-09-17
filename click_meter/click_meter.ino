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
#define ZIPPED_TIME_STEP_SEC    120   // Time resolution for `Rec.timestamp`.

/** 
 * The buffer of click timestamps is used 
 * to derive `maxCPM` among those number of elements every `STATS_DELAY_SEC`.
 */
#define     NCLICK_TIMES          5   // Size of `maxCPM` timestamp-buffer.
/** Threshold `maxCPM` values for the led bar (len==NLEDS). */
const int LED_BAR_THRESH[]      = {30, 70, 150, 350, 800};
//
//////////// HARDWARE ////////////
//
const int   LED_BAR_PINS []     = {10,11,12,13,9};
const int   NLEDS               = ARRAYLEN(LED_BAR_PINS);
const int   GEIGER_PIN          = 2;
#define     LCD_PINS            3,4,5,6,7,8   // Libellium LCD pin numbers.

/** J305βγ Conversion factor [CPM to uSV/h] - unused, for my reference. */
const float SIEVERT_CONV_FACTOR = 0.00812;
//
//////////// GLOBALS /////////////
//
ulong lastStatsMs               = 0;
ulong lastEdiskMs               = 0;
bool is_recording               = false;
uint rec_clicks                 = 0;
uint rec_maxCPM                 = 0;

volatile uint clicks            = 0;
volatile int maxCPM             = 0.0;

/** 
 * The `maxCPM` click-timings buffer.
 */
ulong clickTimesBuffer[NCLICK_TIMES] = {0};
int nextClickIx = 0;

int EDISK_nextIx                = -1;   // Index in EEPROM to write next rec (scanned on boot).
int EDISC_nrecs_saved           = 0;
//
//////////////////////////////////


const ulong  STATS_DELAY_MSEC   = 1000L * STATS_DELAY_SEC;
const ulong  EDISK_DELAY_MSEC   = 1000L * EDISK_DELAY_SEC;

const ZippedTime tzipper(ZIPPED_TIME_STEP_SEC);
LiquidCrystal lcd(LCD_PINS);
const RTC_DS1307 rtc;



bool send_rec(Rec &rec) {
  if (Rec_is_valid(rec))
    Serial << tzipper.unzip(rec.tmstmp) << F(", ") << rec.clicks << F(", ") << rec.maxCPM << endl;
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



void update_stats(int send_serial, ulong now) {
    if (send_serial) {
      Serial << clicks << F(",") << maxCPM << endl;
    }
    
    lcd.clear();    

    lcd << F("C10S=") << clicks;
    lcd.setCursor(9, 0);
    lcd << F("Rec=");
    if (is_recording)
      lcd << int(EEPROM.length() / sizeof(Rec)) - EDISC_nrecs_saved;
    else
      lcd << F("_");
    lcd.setCursor(0, 1);
    lcd << F("CPM" STR(NCLICK_TIMES) "=") << maxCPM;
    if (is_recording) {
      lcd.setCursor(9, 1);
      lcd << F("T=") << (EDISK_DELAY_MSEC - now + lastEdiskMs) / 1000;
    }
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
    is_recording = EDISK_rec_flip(is_recording);
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
  is_recording = EDISK_rec_flip(is_recording);

  lcd.clear();
  lcd <<  __DATE__ ;  
  lcd.setCursor(0,1);
  lcd << __TIME__;
  delay(1700);

  // Re-flip REC to remain the same, unless reset.
  is_recording = EDISK_rec_flip(is_recording);
  EDISK_nextIx = EDISK_traverse(0);

  Serial << F("PROG: " __DATE__ ", " __TIME__) << endl;
  init_RTC();
  
  Serial << F("RTC: ");
  send_rtc();
  Serial << endl;

  ulong now = millis();
  #ifdef LOG_BOOT_CONFIG
    send_config();
    send_state(now);
  #endif
  Serial << F("CLICKS, MaxCPM" STR(NCLICK_TIMES)) << endl;
  update_stats(0, now);

  lastStatsMs = lastEdiskMs = millis();
  attachInterrupt(0,INT_countPulseClicks,FALLING);
}




void loop(){
  ulong now = millis();

  read_keys(now);

  // SHORT loop
  //
  if (now - lastStatsMs > STATS_DELAY_MSEC) {
    lastStatsMs = now;
    update_stats(1, now);
    rec_clicks += clicks;
    if (maxCPM > rec_maxCPM) 
      rec_maxCPM = maxCPM;
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
      EDISK_append_rec(rec_clicks, rec_maxCPM);
    }
    rec_clicks = rec_maxCPM = 0;
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
void INT_countPulseClicks(){
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
  attachInterrupt(0,INT_countPulseClicks,FALLING);
}

