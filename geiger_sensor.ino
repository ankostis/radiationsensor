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

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(3,4,5,6,7,8);


// Threshold values for the led bar
#define NLEDS 5
int LED_THRESH[] = {40, 100, 250, 600, 1500};

// Conversion factor - CPM to uSV/h
#define CONV_FACTOR 0.00812

// Variables
int ledArray [] = {10,11,12,13,9};
int geiger_input = 2;
long previous10secMillis = 0;
volatile int count = 0;

#define CLICK_CYCLE 7
long lastClickMillis[CLICK_CYCLE] = {0};
byte nextClickIx = 0;
volatile float maxCPM = 0.0;

void update_stats() {
    Serial.print(count,DEC);
    Serial.print(',');
    Serial.println(maxCPM, 4);

    lcd.clear();    

    lcd.setCursor(0,0);
    lcd.print("C10SEC=");
    lcd.setCursor(7,0);
    lcd.print(count);

    lcd.setCursor(0, 1);
    lcd.print("MaxCPM=");
    lcd.setCursor(7,1);
    lcd.print(maxCPM, 4);
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
  lcd.setCursor(0, 0);
  lcd.print("Radiation Sensor");
  delay(700);
  for (int i=0;i<10;i++){
    delay(100);  
    lcd.scrollDisplayLeft();
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print( __DATE__);  
  lcd.setCursor(0,1);
  lcd.print(__TIME__);
  delay(1700);

  Serial.println("CLICKS, MaxCPM");
  update_stats();

  attachInterrupt(0,countPulse,FALLING);
}

void loop(){
  if (millis() - previous10secMillis > 10000) {
    previous10secMillis = millis();
    update_stats();

    count = maxCPM = 0;
  }

}

void countPulse(){
  detachInterrupt(0);
  long now = millis();
  
  count++;
  
  lastClickMillis[nextClickIx] = now;
  nextClickIx++;
  if (nextClickIx == CLICK_CYCLE) {
    nextClickIx = 0;
  }
  
  float CPM = CLICK_CYCLE * 100000.0 / (now - lastClickMillis[nextClickIx]);
  update_leds(CPM);
  
  if (maxCPM < CPM)
    maxCPM = CPM;
  
  while(digitalRead(2)==0);
  attachInterrupt(0,countPulse,FALLING);
}

