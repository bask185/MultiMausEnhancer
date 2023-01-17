#include <Arduino.h>
#include "src/macros.h"

enum
{
    IDLE,
    settingStreet,
    getIndex,
    addPoints,
} ;

extern void beginEeprom() ;
extern uint8_t handlePoints () ;
extern void passPoint( uint16 ) ;
extern void setPoint( uint16, uint8 ) ;
