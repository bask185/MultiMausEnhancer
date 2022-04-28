#include <Arduino.h>
#include "macros.h"

class i2cEEPROM
{
public:
    i2cEEPROM() ;
    void  put( uint16, uint8* ) ;
    void  get( uint16, uint8* ) ;

private:
    uint16 eeAddress ;
} ;


Hey Ronald, 

Bedankt voor het aanbod.