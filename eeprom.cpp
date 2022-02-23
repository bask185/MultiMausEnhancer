#include "eeprom.h"

void clear( uint8 streetIndex ) 
{

}

void addPoint( uint16 point )
{

}

void getStreet( uint8 streetIndex )
    uint16 eeAddress = streetNumber * nPointsPerStreet * 2 ;                      // calculate address
   
    EEPROM.get( eeAddress, points ) ;                                             // fetch array from EEPROM
    pointIndex = 0 ;   