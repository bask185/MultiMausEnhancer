#include <Arduino.h>
#include "src/macros.h"

const int FAST = 250 ;
const int SLOW = 500 ;

// enum events
// {
    
    
// } ;

enum modes
{
    // ready,
    // settingStreet,
    // gettingStreetIndex,
    // gettingPoints,
    // recording,
    // playing,
} ;

extern void eventHandler() ;
extern void setLights( uint8 ) ;
extern uint8 mode ;
