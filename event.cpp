#include <EEPROM.h>
#include "event.h"

typedef struct someName 				// 8 bytes per event
{
	uint8 	data1 ;
	uint16 	data2 ;
	uint8	data3 ;
	uint32  time2nextEvent ;
} Event ;

Event   event ;

enum eventModes
{
    idle,
    playing,
    recording,
    finishing,
} ;


const int       baseEEaddress = 0x14 ;
static uint16   eeAddress ;
static uint32   prevTime ;
static uint16   newSensor ;
uint8           recordingDevice = idle ;
bool            playingAllowed ;

void startRecording() 
{
    if( recordingDevice == idle )
    {
        eeAddress = 0 ;                     // set EEPROM adres to 0
        recordingDevice = recording ;       
        prevTime = millis() ;               // record starting time
        storeEvent( START, 1, 1 ) ;
    }
}

void stopRecording() 
{
    if( recordingDevice == recording )
    {
        recordingDevice = idle ;
        storeEvent( STOP, 1, 1 ) ;
    }
}
void startPlaying() 
{
    if( recordingDevice == idle )
    {
        eeAddress = 0 ;
        event.time2nextEvent = 10 ;
        prevTime = millis() ;
        recordingDevice = playing ;
        playingAllowed = true ;
    }
}
void stopPlaying() 
{
    if( recordingDevice == playing )
    {
        recordingDevice = finishing ;
    }
}

void storeEvent( uint8 _data1, uint16 _data2, uint8 _data3 )
{
    if( recordingDevice != recording ) return ;

    Event     localEvent ;
    uint32    currTime = millis() ;

    localEvent.data1 = _data1 ;
    localEvent.data2 = _data2 ;
    localEvent.data3 = _data3 ;

    if( _data1 == FEEDBACK ) { localEvent.time2nextEvent = 0 ; }                       // feedback has 0 time
    else                     { localEvent.time2nextEvent = currTime - prevTime ; }

    prevTime = millis() ;

    EEPROM.put( eeAddress, localEvent ) ;

    eeAddress += sizeof( localEvent ) ;            // increase EEPROM address for next event ;
}

Event getEvent()
{
    Event localEvent ;

    EEPROM.get( eeAddress, localEvent ) ;

    eeAddress += sizeof( localEvent ) ;            // increase EEPROM address for next event ;

    return localEvent ;
}


void sendFeedbackEvent( uint16 number )
{
    newSensor = number ;
}

void eventHandler()
{
    uint32 currTime = millis() ;

    if( recordingDevice == playing && (currTime - prevTime) >= event.time2nextEvent )
    {
        if( event.time2nextEvent == 0 )                                         
        {
            if( newSensor == event.data2 ) event.time2nextEvent = 1 ;
            return ;
        }
                                       //    8bit         16bit        8bit       // for here and now, data1, is type, data2 is address and data 3 just data.
        if( notifyEvent ) notifyEvent( event.data1, event.data2, event.data3 ) ;

        prevTime = currTime ;
        event = getEvent() ;     

        newSensor = 0 ;
    }
}