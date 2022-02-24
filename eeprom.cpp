#include "eeprom.h"

uint16  points[ nPointsPerStreet ] ;
uint16  streetIndex ;
uint16  pointIndex ;

void clearStreet( uint8 _streetIndex ) 
{
    streetIndex = _streetIndex ;

    uint16 beginAddress = streetIndex * nPointsPerStreet * 2 ;                  // 0 -> 0, 1 -> 32, 2-> 64, 3 -> 96  ...  16 -> 512
    uint16 endAddress   = beginAddress + (nPointsPerStreet * 2) ;
    for ( uint16 address = beginAddress ; address < streetIndex + (nPointsPerStreet * 2) ; address++ ) 
    {
        EEPROM.write( address, 0xFF ) ;
    }
}

void addPoint( uint16 newAddress )
{
    bool pointIsTaken = false ;
    uint16 beginAddress = streetIndex * nPointsPerStreet * 2 ;                  // 0 -> 0, 1 -> 32, 2-> 64, 3 -> 96  ...  16 -> 512
    uint16 endAddress   = beginAddress + (nPointsPerStreet * 2) ;
    for ( uint16 eeAddress = beginAddress ; eeAddress < streetIndex + (nPointsPerStreet * 2) ; eeAddress++ ) 
    {
        uint16 currentAddress ;
        EEPROM.get( eeAddress, currentAddress );

        if( (newAddress & 0x3FFF) == (newAddress & 0x3FFF) ) pointIsTaken = true ; // ERROR BUG need to update same point in the event of the state changes
    }

    if( pointIsTaken == true ) return ;                                         // if address is already taken

    uint16 eeAddress = streetIndex * nPointsPerStreet * 2 + (pointIndex * 2 )  ;

    EEPROM.put( eeAddress, newAddress ) ;
    pointIndex ++ ;
}

void getStreet( uint16 *ptr uint8 streetIndex )
{
    uint16 eeAddress = streetIndex * nPointsPerStreet * 2 ;                      // calculate address

    EEPROM.get( eeAddress, points ) ;                                             // fetch array from EEPROM
    pointIndex = 0 ;  
}

uint16 getPoint()
{
    EEPROM.get( eeAddress, points ) ;
}