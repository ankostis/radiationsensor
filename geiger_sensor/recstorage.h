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

typedef unsigned int  uint;
typedef unsigned long ulong;

/**
 *  EEPROM record:
 *  The size_t(Rec) == 7, so EEPROM fits: 
 *      1024 / 7    = 146 records
 *      146 / 6 / 24  = 1.014 days
 *  Assuming 10min timestamps, a whole day fits in!
 */ 
typedef struct _Rec {
  uint tmstmp;
  uint clicks;
  uint maxCPM;
  byte crc;
} Rec;

int STORAGE_traverse(int start_ix = 0, void (*printout)(Rec) = NULL);

#define ARRAYLEN(x) sizeof((x)) / sizeof((x)[0])
// Trick from http://stackoverflow.com/questions/5459868/c-preprocessor-concatenate-int-to-string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#endif //RADIATION
