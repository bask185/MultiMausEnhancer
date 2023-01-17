#include "eeprom.h"
#include <EEPROM.h>
#include "src/stateMachineClass.h"
#include "src/XpressNetMaster.h"


const int nPointsPerStreet = 16 ;
const int nStreets = 16 ;
const int baseAddress = 1000 ;
const int endAddress (baseAddress + nPointsPerStreet) ;
// const int maxAddress = 1024 ;   unused

static StateMachine sm ;

uint16  points[ nPointsPerStreet ] ;    // array to store one street in

uint16  streetIndex ;
int16   pointIndex ;
uint16  newAddress ;
uint16  oldAddress ;
bool    newState ;
uint16  newRaw ;
bool    pointReceived = false ;

void beginEeprom()
{
    sm.setState( IDLE ) ;
}

#ifdef DEBUG

#define debug( x ) {Serial.print( x ) ; }
#define _debug( x ) { Serial.write(' ') ; Serial.print( x ) ; } 
#define debugln( x ) { Serial.println( x ) ; }
#define debugNumber(  x ) { Serial.print( x ) ; }
#define debugNumberln( x ) { Serial.println( x ) ; }

#else

#define debug( x ) ;
#define _debug( x ) ;
#define debugln( x ) ;
#define debugNumber( x ) ;
#define debugNumberln( x ) ;

#endif


void passPoint( uint16 _address )
{
    newRaw     = _address ;
    newAddress = newRaw & 0x3FFF ;
    newState   = newRaw >> 15 ;
    debug("new address = "); debugNumber(newAddress); 
    debug(" new state "); debugNumberln( newState ) ;
    pointReceived = true ;
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
        debug("IDLE") ;
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
       // debug("new address ") ; debugNumberln( newAddress ) ;
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
        debug("setting street: ") ;  debugNumberln( streetIndex ) ;    
        debug("eeprom address = ") ; debugNumberln( eeAddress ) ;    
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
                debug("setting point ") ; debugNumber( pointAddress ) ; if( state ) debugln(" curved") else debugln(" straight") 
            }
        }
    }
    if( sm.exitState() )
    {
        if( pointIndex == nPointsPerStreet )  debugln("max points reached")
        else                                  debugln("invalid address")
        
    }
    return sm.endState() ;
}

StateFunction( getIndex )
{
    if( sm.entryState() )
    {
         debugln("getting street index for teaching in points") ;
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

        debug("received index = "); debugNumberln( streetIndex ) ; 
        debug("clearing street from: "); debugNumber( beginAddress ) ; 
        debug(" to: "); debugNumberln( endAddress ) ; 

        for ( uint16 address = beginAddress ; address < endAddress ; address++ ) 
        {
            EEPROM.write( address, 0xFF ) ;
        }  
        debugln("street wiped!");
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
        pointReceived = false ;
        pointIndex = -1 ;
        oldAddress = newAddress = 9999 ;
        debugln("adding points");
    }
    if( sm.onState() )
    {
        if( pointReceived == true )
        {   pointReceived  = false ;

            if( newAddress != oldAddress )
            {   oldAddress  = newAddress ;
                
                pointIndex ++ ;
                debug("new address, index = ") ;
                debugNumberln( pointIndex ) ;
            }

            if( pointIndex == nPointsPerStreet                                  // if we added max amount of points FIXME
            ||  newAddress == baseAddress )                                     // or base address is entered...
            {
                sm.exit() ;                                                     // -> exit
            }
            else
            {
                uint16 eeAddress = streetIndex * nPointsPerStreet * 2 + (pointIndex*2); 
                EEPROM.put(eeAddress, newAddress | (newState<<15)) ;            // otherwise store the point

                debug("storing point: ") ;  debugNumber( newAddress & 0x03FF ) ;
                debug(" state: ") ;         debugNumber( newState ) ;
                debug(" on address: ") ;    debugNumberln( eeAddress ) ;
            }
        }
    }
    if( sm.exitState() )
    {
        if( pointIndex == nPointsPerStreet ) debugln("maximum amount of points added")
        else                                 debugln("base address received, all points saved")
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