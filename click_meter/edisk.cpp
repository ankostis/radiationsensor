/*  
 *  Arduino Geiger Counter - EEPROM storage (edisk)
 *  
 *  Copyright (C) ankostis@gmail.com
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
#include <EEPROM.h>

#include "clickmeter.h"

ZippedTime::ZippedTime(uint time_step_sec) {
  this->time_step_sec = time_step_sec;
}

uint ZippedTime::zip(ulong sec) {
  //Serial << "TCOMP: " << sec << ", " << MYEPOCH_sec << ", " <<  ((sec - MYEPOCH_sec) /60/10) << endl;
  return (sec - this->MYEPOCH_sec) / this->time_step_sec; 
}

ulong ZippedTime::unzip(uint ztime) {
  return this->MYEPOCH_sec + ztime * this->time_step_sec;
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
    rec.tmstmp = tzipper.zip(rnow.unixtime());
    rec.crc = _Rec_crc(rec);
}


bool Rec_is_valid(Rec &rec) {
  int crc = _Rec_crc(rec);
    #ifdef LOG_REC_VISIT
      Serial << F("  calced_crc=") << crc << endl;
    #endif  
  return bool(rec.crc == crc);
}

/** The byte-address in EEPROM for the the "is_recording" flag. */
const int EDSIK_is_rec_eix = EEPROM.length() - 1;

int EDISK_rec_flip(int is_recording) {
  is_recording = 1 ^ EEPROM.read(EDSIK_is_rec_eix);
  EEPROM.write(EDSIK_is_rec_eix, is_recording);
  
  return is_recording;
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
    recHandler_func = Rec_is_valid;
    
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


void EDISK_append_rec(uint clicks, uint maxCPM) {
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
      EEPROM.write(EDISK_nextIx, 1 ^ EEPROM.read(EDISK_nextIx)); // Break next CRC.
    #endif
    EDISC_nrecs_saved++;
}


void EDISK_clear() {
  EEPROM.write(0, 0); // Probably breaks CRC...
  EDISK_nextIx = 0;

}

