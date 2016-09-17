# radiationsensor
Arduino sources for Libelium's radiation-sensor shield + RTC.

### LCD explanation
```
                                          Records remaining before
          Clicks during                   EEPROM exhausted since boot 
        the last 10secs─────┐       ┌─────(counting down from 146)
                        ┌───┴──┐ ┌──┴──┐
                       ╔╪══════╪═╪═════╪╗
                       ║C10S=6   Rec=55 ║
                       ║CPM5=78  T=1045 ║
                       ╚╪══════╪═╪═════╪╝
                        └───┬──┘ └──┬──┘
        MaxCPM based on─────┘       └─────Secs remaining to write 
   5 conjecutive clicks                   the next record
    for the last 10 sec
 (reflected on Led-bar) 
```

LCD when record is OFF:
```
                       ╔════════════════╗
                       ║C10S=12  Rec=_  ║
                       ║CPM5=313        ║
                       ╚════════════════╝
```
