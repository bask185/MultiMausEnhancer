#include <EEPROM.h>
#include "shuttle.h"

extern void message( String mess, int val1, int val2 ) ;

const int 		baseEEaddress  	= 512 ;
uint16			eeAddress ;
static uint32	prevTime ;

void startPlaying()
{
	eeAddress = baseEEaddress ;
}

void storeEvent( uint8 type, uint16 address, uint8 data )
{
	Event localEvent ;
	uint32	currTime = millis() ;

	if( type == event_start )
	{
		eeAddress = baseEEaddress ;
		prevTime = millis() ;			// reset time when recording starts
	}

	switch( type )		// show message of what is being stored
	{
		case event_speed: 	message(F("storing speed event"), address, data ) ; break ;
		case event_point: 	message(F("storing point event"), address, data ) ; break ;
		case event_start:	message(F("storing start event"), 0, 0 			) ; break ;
		case event_stop: 	message(F("storing stop event" ), 0, 0 			) ; break ;
	}

	localEvent.type = type ;
	localEvent.address = address ;
	localEvent.data = data ;
	localEvent.time2nextEvent = currTime - prevTime ;
	prevTime = millis() ;

	EEPROM.put( eeAddress, localEvent ) ;

	eeAddress += sizeof( localEvent ) ;			// increase EEPROM address for next event ;
}

Event getEvent()
{
	Event localEvent ;

	EEPROM.get( eeAddress, localEvent ) ;

	eeAddress += sizeof( localEvent ) ;			// increase EEPROM address for next event ;

	return localEvent ;
}


