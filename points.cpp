#include "points.h"
#include "event.h"
#include <EEPROM.h>
#include "src/stateMachineClass.h"

extern void message( String mess, uint16 val1, uint16 val2 ) ;

const int nPointsPerStreet = 16 ;
const int nStreets = 16 ;
const int baseAddress = 999 ;
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


void passPoint( uint16 _address )
{
    newRaw     = _address ;
    newAddress = newRaw & 0x3FFF ;
    newState   = newRaw >> 15 ;
    //message( F("passed points"),  newAddress,  newState ) ;
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
        message( F("IDLE"),  0,  0 ) ;

        // setMode( idling ) ;
    }
    if( sm.onState() )
    {
        if( newAddress == baseAddress && newState == false                     // -> point 1000 is set curved -> teachin points
        ||( newAddress  > baseAddress && newAddress <= endAddress ) )          // -> set one of the streets
        {
            sm.exit() ;
        }
    }
    if( sm.exitState() )
    {
         message( "IDLE",  0,  0 ) ;
    }
    return sm.endState() ;
}

StateFunction( settingStreet )
{
    if( sm.entryState() )
    {
        streetIndex = newAddress - baseAddress - 1 ;
        uint16 eeAddress = streetIndex * nPointsPerStreet * 2 ;                 // calculate address

        EEPROM.get( eeAddress, points ) ;                                       // load array from EEPROM
        pointIndex = 0 ; 
        message( F("setting street index/eeAddress"),  streetIndex,  eeAddress ) ;

        // setMode( settingPoints ) ;
    }
    if( sm.onState() )
    {
        if( sm.repeat( 1500 ) )                                                        // set a point every 500ms
        {
            uint16 raw = points[ pointIndex ] ;
            uint16 pointAddress = raw & 0x03FF ;
            uint8  state        = raw >> 15 ;

            if( ++ pointIndex == nPointsPerStreet
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
        if( pointIndex == nPointsPerStreet )  message(F("max points reached"),nPointsPerStreet,0);
        else                                  message(F("invalid address"),0,0);

        
    }
    return sm.endState() ;
}

StateFunction( getIndex )
{
    if( sm.entryState() )
    {
        // setEvent( enteringTeachin ) ;
        message(F("getting street index for teaching in points"),0,0) ;
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
        streetIndex = newAddress - baseAddress - 1 ;                                // calculate street index and whipe matching part in EEPROM

        uint16 beginAddress = streetIndex  *  nPointsPerStreet * 2  ;           
        uint16 endAddress   = beginAddress + (nPointsPerStreet * 2) ;

        message(F("received index = "),streetIndex,0); 
        message(F("clearing street from/to: "),beginAddress,endAddress);

        for ( uint16 address = beginAddress ; address < endAddress ; address++ ) 
        {
            EEPROM.write( address, 0xFF ) ;
        }  
        // setEvent( indexReceived ) ;
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
        message(F("adding points"),0,0);

        // setMode( teachin ) ;
    }
    if( sm.onState() )
    {
        if( pointReceived == true )
        {   pointReceived  = false ;

            if( newAddress != oldAddress )
            {   oldAddress  = newAddress ;
                
                pointIndex ++ ;
                message(F("new address, index = "),newAddress, pointIndex) ;
            }

            if( pointIndex == nPointsPerStreet                                  // if we added max amount of points FIXME
            ||  newAddress == baseAddress )                                     // or base address is entered...
            {
                sm.exit() ;                                                     // -> exit
            }
            else
            {
                if( newAddress >= baseAddress )
                {
                    message(F("invalid address "),newAddress, newState) ;
                }
                else
                {
                    uint16 eeAddress = streetIndex * nPointsPerStreet * 2 + (pointIndex*2); 
                    EEPROM.put(eeAddress, newAddress | (newState<<15)) ;            // otherwise store the point

                    message(F("storing point: "),newAddress, newState) ;
                    // setEvent( pointAdded ) ;
                }
            }
        }
    }
    if( sm.exitState() )
    {
        if( pointIndex == nPointsPerStreet ) message(F("maximum amount of points added"),0,0) ;
        else                                 message(F("base address received, all points saved"),0,0) ;

        // setEvent( leavingTeachin ) ;
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