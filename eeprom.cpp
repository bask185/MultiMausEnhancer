#include "eeprom.h"
#include <EEPROM.h>
#include "src/stateMachineClass.h"


const int nPointsPerStreet = 16 ;
const int nStreets = 16 ;
const int baseAddress = 100 ;
const int endAddress (baseAddress + nPointsPerStreet) ;
// const int maxAddress = 1024 ;   unused

static StateMachine sm ;

uint16  points[ nPointsPerStreet ] ;    // array to store one street in

uint16  streetIndex ;
uint16  pointIndex ;
uint16  newAddress ;
uint16  oldAddress ;
bool    newState ;
uint16  newRaw ;

void beginEeprom()
{
    sm.setState( IDLE ) ;
}

void passPoint( uint16 _address )
{
    newRaw     = _address ;
    newAddress = newRaw & 0x3FFF ;
    newState   = newRaw >> 15 ;
}



/* NOTE
need control functions
Need to be able to send STATE and ADDRESS of points to this file
*/

StateFunction( IDLE )
{
    if( sm.entryState() )
    {
        newAddress = oldAddress = 9999 ;
    }
    if( sm.onState() )
    {
        if( newAddress == baseAddress && newState == true                      // -> teachin points
        ||( newAddress  > baseAddress && newAddress <= endAddress ) )          // -> set one of the streets
        {
            sm.exit() ;
        }
    }
    if( sm.exitState() )
    {

    }
    return sm.endState() ;
}

StateFunction( settingStreet )
{
    if( sm.entryState() )
    {
        streetIndex = newAddress - baseAddress ;
        uint16 eeAddress = streetIndex * nPointsPerStreet * 2 ;                 // calculate address

        EEPROM.get( eeAddress, points ) ;                                       // load array from EEPROM
        pointIndex = 0 ;     
    }
    if( sm.onState() )
    {
        if( sm.repeat( 500 ) )                                                        // set a point every 500ms
        {
            uint16 raw = points[ pointIndex ++ ] ;
            uint16 pointAddress = raw & 0x03FF ;
            uint8  state        = raw >> 15 ;

            if( pointIndex == nPointsPerStreet 
            ||  raw == 0xFFFF )
            {
                sm.exit() ;                                                     // if max amount is reached or invalid addres -> exit
            }
            else                                                                // otherwise set the point
            {
                setPoint( pointAddress, state ) ;
            }
        }
    }
    if( sm.exitState() )
    {
        
    }
    return sm.endState() ;
}

StateFunction( getIndex )
{
    if( sm.entryState() )
    {

    }
    if( sm.onState() )
    {
        if( newAddress > baseAddress                                            // if valid address for street index is entered, go on
        &&  newAddress <= endAddress ) 
        {
            sm.exit() ;
        }
    }
    if( sm.exitState() )
    {
        streetIndex = newAddress - baseAddress ;                                // calculate street index and whipe matching part in EEPROM

        uint16 beginAddress = streetIndex  *  nPointsPerStreet * 2  ;           
        uint16 endAddress   = beginAddress + (nPointsPerStreet * 2) ;

        for ( uint16 address = beginAddress ; address < endAddress ; address++ ) 
        {
            EEPROM.write( address, 0xFF ) ;
        }  
    }

    return sm.endState() ;
}

/*
    adding a point:
    need to uncondonditionally store the new address with state on current index. 
    If the address differs with the previous address, the index needs to be incremented first.
    if the index reaches the value of nPointsPerStreet OR the point with base address is set straight, we exit
*/
StateFunction( addPoints )
{
    if( sm.entryState() )
    {
        pointIndex = 0 ;
        oldAddress = newAddress = 9999 ;
    }
    if( sm.onState() ) // BUG prevent that this can run limitless
    {
        if( newAddress != oldAddress )
        {   oldAddress  = newAddress ;
            
            pointIndex ++ ;
        
            if( pointIndex == nPointsPerStreet                                      // if we added max amount of points
            ||  newAddress == baseAddress )                                         // or base address is entered...
            {
                sm.exit() ;                                                         // -> exit
            }
            else
            {
                uint16 eeAddress ; // ADDED CALCULATION HERE
                EEPROM.put(eeAddress, newAddress) ;                                 // otherwise store the point
            }
        }
    }
    if( sm.exitState() )
    {
        
    }
    return sm.endState() ;
}

/**
 * @brief FSM for handling storing and setting point streets
 * 
 * @return current state
 *  
 */

uint8 handlePoints ()
{
    STATE_MACHINE_BEGIN

        State( IDLE ) {
            if( newAddress == baseAddress ) sm.nextState( getIndex, 0 ) ;
            else                            sm.nextState( settingStreet, 0 ) ; }

        State( settingStreet ) {
            sm.nextState( IDLE, 0 ) ; }

        State( getIndex ) {
            sm.nextState( addPoints, 0 ) ; }

        State( addPoints ) {
            sm.nextState(IDLE, 0 ) ; }

    STATE_MACHINE_END
}