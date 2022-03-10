#include <Arduino.h>
#include "src/macros.h"

const int   ON =    0 ;
const int FAST =  250 ;
const int SLOW = 500 ;

enum events
{
    pointSet = 1,
    enteringTeachin,
    indexReceived,
    leavingTeachin,
    pointAdded,
} ;

enum modes
{
    idling,               // blink green slow
    teachin,            // orange on
    settingPoints,      // green on
} ;

extern void eventHandler() ;
extern void setEvent( uint8 ) ;
extern void setMode( uint8 ) ;