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

#ifndef RADIATION
#define RADIATION

#include <RTClib.h>
#include <LiquidCrystal.h>

typedef unsigned int  uint;
typedef unsigned long ulong;
const int INT_MAX = 2147483647;

/**
 *  EEPROM record:
 *  The size_t(Rec) == 7, so EEPROM fits: 
 *      1024 / 7    = 146 records
 *      146 / 6 / 24  = 1.014 days
 *  Assuming 10min timestamps, a whole day fits in!
 */ 
typedef struct {
  ulong tmstmp :20;
  uint clicks: 14;
  uint maxCPM: 14;
  byte crc;
} Rec;

/**
 * Compress timestamps by storing them as X-min offsets  from 11-Sep-2016 (MYEPOCH).
 * 
 * For instance, assuming (2^20) timestamps of each Rec, then  
 * a 2-min steps can represent can represent:
 *   - 2^20                   =  1,048,576    max timestamp
 *   - 2^20 * 120             = 62,914,560    sec
 *   - 2^20 * 120/3600        ~=    34,952.33 hours
 *   - 2^20 * 120/8640        ~=     1,456.35 days
 *   - 2^20 * 120/86400/365   ~=         4    years
 * 
 * Recompile to update MYEPOCH after 4 years :-(
 */
class ZippedTime {
 // MYEPOCH(2016/09/11) expressed from EPOCH(1970/1/1):
  const ulong MYEPOCH_sec = DateTime(2016, 9, 11).unixtime();
  uint time_step_sec; // WARN!!! changing delay, back-dates recordings!!!

  public: 
    ZippedTime(uint tsteps);  
    uint zip(ulong sec);
    ulong unzip(uint ztime);
};


extern const ZippedTime tzipper;
extern const RTC_DS1307 rtc;
extern LiquidCrystal lcd;  // Cannot const due to `<<` operator.
extern int EDISK_nextIx, EDISC_nrecs_saved;

bool Rec_is_valid(Rec &rec);
int EDISK_traverse(const int start_ix = INT_MAX, bool (*recHandler_func)(Rec&) = NULL);
int EDISK_rec_flip(int is_recording);
void EDISK_append_rec(uint clicks, uint maxCPM);
void EDISK_clear();

#define ARRAYLEN(x) sizeof((x)) / sizeof((x)[0])
// Trick from http://stackoverflow.com/questions/5459868/c-preprocessor-concatenate-int-to-string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)


#endif //RADIATION

