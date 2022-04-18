#include <Arduino.h>
#include "src/macros.h"

enum events
{
	event_start,
	event_stop,
	event_speed,
	event_F0_F4,
	event_F5_F8,
	event_F9_F12,
	event_F13_F20,
	event_point,
	event_feedback,		// extern input NOT YET IN USE
} ;

typedef struct 				// 8 bytes per event
{
	uint8 	type ;			// loco or accessory
	uint16 	address ;
	uint8	data ;
	uint32  time2nextEvent ;
} Event ;

extern void startPlaying() ;
extern void storeEvent( uint8, uint16, uint8 ) ;
extern Event getEvent() ;