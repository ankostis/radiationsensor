/*  
 *  Geiger Counter - Radiation Sensor Board
 *  
 *  Copyright (C) Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com 
 *  
 *  This program is free software: you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License as published by 
 *  the Free Software Foundation, either version 3 of the License, or 
 *  (at your option) any later version. 
 *  a
 *  This program is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License 
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *  
 *  Version:           3.0
 *  Design:            David Gascón 
 *  Implementation:    Marcos Yarza
 */

#include <EEPROM.h>
#include <Streaming.h>
#include <LiquidCrystal.h>
#include <RTClib.h>


// Trick from http://stackoverflow.com/questions/5459868/c-preprocessor-concatenate-int-to-string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// J305βγ Conversion factor - CPM to uSV/h
const float CONV_FACTOR = 0.00812;
const int ledArray [] = {10,11,12,13,9};
const int geiger_input = 2;


/** 
 * A circulare-buffer of click timings.
 */
#define NMAX_CPM 5
long lastClickMillis[NMAX_CPM] = {0};
int nextClickIx = 0;

volatile float maxCPM = 0.0;
// maxCPM threshold values for the led bar
const int NLEDS = 5;
int LED_THRESH[] = {30, 70, 150, 350, 800};

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(3,4,5,6,7,8);

long loopMillis = 0;
volatile int count = 0;


RTC_DS1307 rtc;
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


void update_stats(int send_serial) {
    if (send_serial) {
      Serial << count << ',' << _FLOAT(maxCPM, 4) << endl;
    }
    
    lcd.clear();    

    lcd << "C10SEC=" << count;
    lcd.setCursor(0, 1);
    lcd << "MaxCPM" STR(NMAX_CPM) "=" << _FLOAT(maxCPM, 4);
}

void update_leds(float CPM) {
  //led var setting
  int i = 0;
  for(int i=0; i < NLEDS; i++) {
    int state = (CPM > LED_THRESH[i])? HIGH: LOW;
    digitalWrite(ledArray[i], state);
  }
}

void setup(){
  pinMode(geiger_input, INPUT);
  digitalWrite(geiger_input,HIGH);
  for (int i=0;i<5;i++){
    pinMode(ledArray[i],OUTPUT);
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

  attachInterrupt(0,countPulse,FALLING);
}

void loop(){
  long now = millis();
  if (now - loopMillis > 10000) {
    loopMillis = now;
    update_stats(1);

    count = maxCPM = 0;
  }

}

void countPulse(){
  detachInterrupt(0);
  long now = millis();
  
  count++;
  
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
  attachInterrupt(0,countPulse,FALLING);
}

